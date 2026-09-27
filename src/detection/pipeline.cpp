// ============================================================================
// DetectionPipeline 实现：一帧一拍，两个阶段。
//
//   启动/丢失重扫帧：LightScanner（RVV 全图粗筛）→ 启动确认器（3 帧滑窗平滑性）
//   跟踪/丢失 ROI 帧：RoiBlobMeasurer（游程连通域）→ 卡尔曼更新（马氏门限）
//
// 本文件只做编排 + 计时 + 结果翻译，不含任何算法（算法都在各自的类里）。
// ============================================================================

#include "detection/pipeline.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace dart::detection {

namespace {

uint64_t real_now_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ull + static_cast<uint64_t>(ts.tv_nsec) / 1000ull;
}

// 配置先夹到可用范围再构造各部件（手输的 CLI 数字不做校验会直接算坏窗口）
DetectionConfig sanitized(DetectionConfig c) {
    c.sanitize();
    return c;
}

// ROI 测量器的解析自检：造一张"已知圆心/已知半径"的二值图，看测量结果对不对。
// 这一步与 RVV 无关（宿主机也跑），但它守的是"窗口偏移/绝对坐标/面积换算"这类
// 最容易写错又最难在板端看出来的东西（窗口坐标少加一个 w.x 就是整幅图的系统性偏差）。
bool measurer_selftest() {
    const uint32_t W = 160, H = 96;
    std::vector<uint8_t> px(static_cast<size_t>(W) * H, 0);
    const int      cx = 100, cy = 50, r = 9;
    uint32_t       truth_area = 0;
    for (int y = cy - r; y <= cy + r; ++y) {
        for (int x = cx - r; x <= cx + r; ++x) {
            if (x < 0 || y < 0 || x >= static_cast<int>(W) || y >= static_cast<int>(H))
                continue;
            const int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= r * r) {
                px[static_cast<size_t>(y) * W + x] = 255;
                ++truth_area;
            }
        }
    }

    GrayFrame f{};
    f.pixels = px.data();
    f.width = W;
    f.height = H;
    f.stride = W;

    MeasureConfig mcfg;
    RoiBlobMeasurer meas(mcfg);
    RoiWindow w;
    w.x = 64;
    w.y = 24;
    w.w = 64;
    w.h = 48;
    w.full_frame = false;

    const TargetMeasurement m = meas.measure(f, w);
    if (!m.valid)
        return false;
    if (m.area != truth_area)
        return false;
    if (std::fabs(m.cx - static_cast<float>(cx)) > 0.6f || std::fabs(m.cy - static_cast<float>(cy)) > 0.6f)
        return false;
    if (m.roi != RoiState::Hit)
        return false;

    // 贴框语义：窗口缩到刚好吃住目标 → 必须报 TargetOutOfRoi + 降质量
    RoiWindow tight;
    tight.x = static_cast<uint32_t>(cx - r);
    tight.y = static_cast<uint32_t>(cy - r);
    tight.w = static_cast<uint32_t>(2 * r);
    tight.h = static_cast<uint32_t>(2 * r);
    tight.full_frame = false;
    const TargetMeasurement mt = meas.measure(f, tight);
    if (!mt.valid || mt.roi != RoiState::TargetOutOfRoi || !(mt.quality < 1.0f))
        return false;

    // 空窗口 → 无目标
    RoiWindow none;
    none.x = 0;
    none.y = 0;
    none.w = 32;
    none.h = 32;
    none.full_frame = false;
    if (meas.measure(f, none).valid)
        return false;
    return true;
}

} // namespace

DetectionPipeline::DetectionPipeline(const DetectionConfig &cfg) : DetectionPipeline(cfg, Deps{}) {}

DetectionPipeline::DetectionPipeline(const DetectionConfig &cfg, const Deps &deps)
    : cfg_(sanitized(cfg)), owned_scanner_(cfg_.scan), owned_measurer_(cfg_.measure),
      scanner_(deps.scanner ? deps.scanner : &owned_scanner_),
      measurer_(deps.measurer ? deps.measurer : &owned_measurer_), now_fn_(deps.now_us),
      tracker_(cfg_, deps.noise), armor_(cfg_.armor, cfg_.measure) {
    // 候选缓冲只分配一次：全图扫描每帧都要写它，运行期不允许分配（板端堆只有 16MB，
    // 而且"帧内分配"会把 11ms 的帧预算啃掉一块）。
    cands_.resize(cfg_.scan.top_k ? cfg_.scan.top_k : 1);
}

DetectionPipeline::~DetectionPipeline() = default;

uint64_t DetectionPipeline::now_us() const { return now_fn_ ? now_fn_() : real_now_us(); }

void DetectionPipeline::reset() {
    tracker_.reset();
    armor_.reset();
    stats_ = DetectionStats{};
}

DetectResult DetectionPipeline::detect(const GrayFrame &frame) {
    DetectResult r{};
    const uint64_t t0 = now_us();

    // 1) 本帧怎么扫（含 KF 预测与动态 ROI）
    const TargetTracker::Plan plan = tracker_.begin_frame(t0, frame.width, frame.height);

    // 2) 按计划扫
    ScanReport rep{};
    rep.window = plan.window;
    const uint64_t t_scan = now_us();
    if (plan.full_scan) {
        const size_t n = scanner_->scan(frame, cands_.data(), cands_.size());
        rep.cands = cands_.data();
        rep.count = n;
        stats_.cands_last = n;
        if (scanner_ == &owned_scanner_)
            stats_.scan_trace = owned_scanner_.last_trace();
    } else {
        TargetMeasurement m = measurer_->measure(frame, plan.window);
        m.cost_us = static_cast<uint32_t>(now_us() - t_scan);
        rep.measurement = m;
        stats_.cands_last = 0;
        if (measurer_ == &owned_measurer_)
            stats_.measure_trace = owned_measurer_.last_trace();
    }
    rep.scan_us = static_cast<uint32_t>(now_us() - t_scan);

    // 3) 状态机 + 滤波
    const TrackOutput out = tracker_.end_frame(rep);

    // 3.5) 装甲板（第二路输出）：绿灯当锚 → 它上方那块窗口里找两片灯条 → 对角线交点。
    // 用的是**同一张二值图**（所以装甲板天然就是"黑白口径、不看颜色"），
    // 窗口由锚现算、只在窗口内做连通域，代价与绿灯那一路的 ROI 测量同量级。
    // 绿灯没锚（!out.found）就不出结果 —— 宁可不报，也不拿一个凭空的位置开窗。
    if (cfg_.armor.enable) { // --armor-off：绿灯照跑，装甲板这一路整段跳过
        armor_.detect(frame, out, now_us(), &r.armor);
        stats_.armor_trace = armor_.last_trace();
    }

    // 4) 翻译成核心层的结果类型（录像标注 / CSV 取证 / 状态行都读它）
    r.roi = out.roi;
    r.radius = out.found ? out.radius : -1.0f;
    r.circ = out.found ? out.circularity : -1.0f;
    r.state = static_cast<uint8_t>(out.state);
    // 窗口内部用 (x,y,w,h) 表示（扫描时就是这个形状），对外一律给**闭区间的四个角**：
    // 录 CSV / 画框 / 和 PBM 对照时角坐标直接可用，不用再做一次 x+w-1 的换算。
    r.roi_x0 = out.window.x;
    r.roi_y0 = out.window.y;
    if (!out.window.empty()) {
        r.roi_x1 = out.window.right() - 1u;
        r.roi_y1 = out.window.bottom() - 1u;
    }
    if (out.found) {
        r.cx = static_cast<int32_t>(std::lround(out.cx));
        r.cy = static_cast<int32_t>(std::lround(out.cy));
    }

    // 5) 记账
    ++stats_.frames;
    if (out.found)
        ++stats_.hits;
    else
        ++stats_.misses;
    stats_.state = out.state;
    stats_.last_window = out.window;
    stats_.last_scan_us = rep.scan_us;
    stats_.tracker = tracker_.counters();
    r.cost_us = static_cast<uint32_t>(now_us() - t0);
    stats_.last_total_us = r.cost_us;
    return r;
}

bool DetectionPipeline::selftest() {
    if (!LightScanner::selftest())
        return false;
    if (!measurer_selftest())
        return false;
    return true;
}

void DetectionPipeline::describe(const DetectionConfig &cfg_in, char *buf, size_t n) {
    if (buf == nullptr || n == 0)
        return;
    const DetectionConfig cfg = sanitized(cfg_in);
    const LightScanner    scanner(cfg.scan);
    const RoiBlobMeasurer measurer(cfg.measure);
    const LinearScaleNoiseModel noise(cfg.kf);
    std::snprintf(buf, n,
                  "扫描=%s(瓦片%u min_area=%u topk=%u) ROI测量=%s(min_area=%u fill>=%.2f) "
                  "噪声=%s(R_scale 远%.2f/近%.2f @ s %.1f..%.1f) ROI: W=kp*s+margin+kσ*σ "
                  "(kp=%.1f margin=%.1f kσ=%.1f) 滑窗=%u帧/命中>=%u 门: accel<=%.1fpx gate=%.1fpx 跟踪=%s",
                  scanner.name(), cfg.scan.tile, cfg.scan.min_area, cfg.scan.top_k, measurer.name(),
                  cfg.measure.min_area, static_cast<double>(cfg.measure.min_fill), noise.name(),
                  static_cast<double>(cfg.kf.r_scale_far), static_cast<double>(cfg.kf.r_scale_near),
                  static_cast<double>(cfg.kf.s_far), static_cast<double>(cfg.kf.s_near),
                  static_cast<double>(cfg.roi.kp), static_cast<double>(cfg.roi.margin),
                  static_cast<double>(cfg.roi.k_sigma), cfg.arm.window, cfg.arm.min_hits,
                  static_cast<double>(cfg.arm.max_accel_px), static_cast<double>(cfg.arm.gate_px),
                  cfg.enable_tracking ? "on" : "off(--no-track)");
    // 装甲板这一路单独接一段：它有自己的门限口径（灯尺倍数 + 地板 + 刚性先验）
    const size_t used = std::strlen(buf);
    if (used < n) {
        std::snprintf(buf + used, n - used,
                      " 装甲板=%s(条长 %.2f~%.2f·s 地板%u 近档长宽比>=%.1f 远档圆度<=%.2f "
                      "窗 %.1f~%.1f×2r 先验间距 %.1f~%.1f·s 保持%u)",
                      cfg.armor.enable ? "on" : "off(--armor-off)",
                      static_cast<double>(cfg.armor.len_min_k), static_cast<double>(cfg.armor.len_max_k),
                      cfg.armor.len_floor, static_cast<double>(cfg.armor.aspect_min),
                      static_cast<double>(cfg.armor.circ_max_far), static_cast<double>(cfg.armor.roi_w_k),
                      static_cast<double>(cfg.armor.roi_h_k), static_cast<double>(cfg.armor.span_lo_k),
                      static_cast<double>(cfg.armor.span_hi_k), cfg.armor.hold);
    }
}

} // namespace dart::detection
