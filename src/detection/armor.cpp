// ============================================================================
// armor.cpp —— 绿灯启动确认器（StartupArmer，armor.hpp）的实现。
//
// 滑窗关联 + 三道门（命中数 / 位移平滑性 / 物理性）。
// 注意：装甲板那一路**不在**这里，它在 src/detection/armor/ 下
// （armor_detector.cpp / armor_geometry.cpp / armor_scale.cpp /
//   armor_rules.cpp / armor_windows.cpp）。
// ============================================================================

#include "detection/armor.hpp"

#include <algorithm>
#include <cmath>

namespace dart::detection {

namespace {

inline float hypot_f(float dx, float dy) { return std::sqrt(dx * dx + dy * dy); }

// 候选排序：亮（面积大）在前；面积相同按 y/cx 排 —— 让同一批输入永远给出同一结果，
// 否则"确认了哪一个"会随排序稳定性变化，板端复盘时对不上。
bool cand_before(const LightCandidate &a, const LightCandidate &b) {
    if (a.score != b.score)
        return a.score > b.score;
    if (a.cy != b.cy)
        return a.cy < b.cy;
    return a.cx < b.cx;
}

} // namespace

StartupArmer::StartupArmer(const ArmConfig &cfg) : cfg_(cfg) { reset(); }

void StartupArmer::reset() {
    head_ = 0;
    filled_ = 0;
    last_reject_ = 0;
    for (uint32_t i = 0; i < kArmMaxWindow; ++i) {
        ring_[i].mono_us = 0;
        ring_[i].n = 0;
    }
}

const StartupArmer::Slot &StartupArmer::at_lag(uint32_t lag) const {
    const uint32_t w = cfg_.window ? cfg_.window : 1;
    const uint32_t idx = (head_ + w - (lag % w)) % w;
    return ring_[idx];
}

StartupArmer::Confirmation StartupArmer::push(uint64_t mono_us, const LightCandidate *cands, size_t n) {
    Confirmation out{};
    last_reject_ = 0;
    last_border_skip_ = 0;

    const uint32_t w = cfg_.window ? cfg_.window : 1;

    // ---- 存本帧候选（先推进 head_，最老的一格被覆盖）----
    head_ = (filled_ == 0) ? 0 : (head_ + 1) % w;
    Slot &s = ring_[head_];
    s.mono_us = mono_us;
    s.n = 0;
    if (cands != nullptr) {
        const size_t take = std::min<size_t>(n, kArmMaxPerFrame);
        for (size_t i = 0; i < take; ++i)
            s.cands[i] = cands[i];
        std::sort(s.cands, s.cands + take, cand_before);
        s.n = static_cast<uint32_t>(take);
    }
    if (filled_ < w)
        ++filled_;

    if (filled_ < w)
        return out; // 滑窗还没填满：没有 3 帧数据，不确认（也不计数）

    // ---- 对最新一帧的每个候选，往回串轨迹 ----
    const Slot &newest = at_lag(0);
    for (uint32_t ci = 0; ci < newest.n; ++ci) {
        // 贴边候选直接跳过，且**不计入拒闪**：它不是"闪烁坏点"，而是被画面边界切掉的块
        // （面积只是下界、质心偏）。两者要分开统计，否则状态行里的"拒闪"会误导调参
        // （round12 就是一堆贴边的大亮块把"确认"骗走了）。
        if (cfg_.skip_border && newest.cands[ci].border) {
            ++last_border_skip_;
            continue;
        }
        Confirmation cand{};
        cand.chain[0] = newest.cands[ci];
        cand.times_us[0] = newest.mono_us;

        uint32_t hits = 1;
        for (uint32_t lag = 1; lag < w; ++lag) {
            const Slot &prev = at_lag(lag);
            if (prev.n == 0)
                break; // 那一帧没有候选（全黑帧）：轨迹到此为止

            // 预测位置：有两点就匀速外推，只有一点就按"原地"（gate 内即可）
            float gate = cfg_.gate_px;
            float px, py;
            if (hits >= 2) {
                const LightCandidate &a = cand.chain[hits - 1]; // 较新
                const LightCandidate &b = cand.chain[hits - 2]; // 更旧
                const float vx = a.cx - b.cx;
                const float vy = a.cy - b.cy;
                px = a.cx + vx; // 往回外推一帧
                py = a.cy + vy;
                gate += cfg_.gate_speed_slack * hypot_f(vx, vy);
            } else {
                px = cand.chain[0].cx;
                py = cand.chain[0].cy;
            }

            // 找门内最近的候选（同时要求尺度不突跳）
            int32_t best = -1;
            float   best_d2 = gate * gate;
            for (uint32_t j = 0; j < prev.n; ++j) {
                const LightCandidate &c = prev.cands[j];
                const float r_ref = cand.chain[hits - 1].radius;
                const float dt = static_cast<float>(static_cast<double>(cand.times_us[hits - 1] -
                                                                        prev.mono_us) *
                                                    1e-6);
                const float radius_slack = cfg_.max_radius_rate * std::fabs(dt) + 1.0f;
                if (std::fabs(c.radius - r_ref) > radius_slack)
                    continue;
                const float dx = c.cx - px;
                const float dy = c.cy - py;
                const float d2 = dx * dx + dy * dy;
                if (d2 <= best_d2) { // <= 保证"并列时取更靠前的（更亮的）"
                    best_d2 = d2;
                    best = static_cast<int32_t>(j);
                }
            }
            if (best < 0)
                break; // 门内没有可关联的候选：轨迹断在这里

            cand.chain[hits] = prev.cands[static_cast<uint32_t>(best)];
            cand.times_us[hits] = prev.mono_us;
            ++hits;
            if (hits >= w)
                break;
        }

        // ---- 门 ① 命中数 ----
        if (hits < cfg_.min_hits) {
            ++last_reject_;
            continue;
        }

        // ---- 门 ②③ 平滑性 + 物理性 ----
        float    travel = 0.0f;
        float    max_accel = 0.0f;
        float    prev_dx = 0.0f, prev_dy = 0.0f;
        bool     have_prev = false;
        for (uint32_t i = 0; i + 1 < hits; ++i) {
            const float dx = cand.chain[i + 1].cx - cand.chain[i].cx; // 在时间上是"向前"
            const float dy = cand.chain[i + 1].cy - cand.chain[i].cy;
            travel += hypot_f(dx, dy);
            if (have_prev) {
                const float ax = dx - prev_dx;
                const float ay = dy - prev_dy;
                const float acc = hypot_f(ax, ay);
                if (acc > max_accel)
                    max_accel = acc;
            }
            prev_dx = dx;
            prev_dy = dy;
            have_prev = true;
        }
        const float straight = hypot_f(cand.chain[0].cx - cand.chain[hits - 1].cx,
                                       cand.chain[0].cy - cand.chain[hits - 1].cy);

        const double dt_total = static_cast<double>(cand.times_us[0] - cand.times_us[hits - 1]) * 1e-6;
        const float  radius_rate =
            dt_total > 1e-9 ? static_cast<float>(std::fabs(static_cast<double>(cand.chain[0].radius -
                                                                               cand.chain[hits - 1].radius)) /
                                                  dt_total)
                            : 0.0f;

        bool reject = false;
        if (max_accel > cfg_.max_accel_px || straight > cfg_.max_travel_px ||
            travel > cfg_.max_travel_px || radius_rate > cfg_.max_radius_rate ||
            cand.chain[0].radius < cfg_.min_radius)
            reject = true;
        if (reject) {
            ++last_reject_;
            continue;
        }

        // ---- 确认：偏好最亮的那个（候选已按面积降序，第一个通过的就是它）----
        out.ok = true;
        out.cand = cand.chain[0];
        out.cand.continuity = static_cast<float>(hits) / static_cast<float>(w);
        out.hits = hits;
        out.travel_px = straight;
        out.max_accel_px = max_accel;
        out.radius_rate = radius_rate;
        for (uint32_t i = 0; i < hits; ++i) {
            out.chain[i] = cand.chain[hits - 1 - i]; // 反成时间升序（旧 → 新）
            out.times_us[i] = cand.times_us[hits - 1 - i];
        }
        out.n = hits;
        return out;
    }
    return out;
}

} // namespace dart::detection
