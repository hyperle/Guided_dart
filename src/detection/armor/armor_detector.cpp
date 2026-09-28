// ============================================================================
// armor_detector.cpp —— ArmorDetector 的状态机（detect / track / 判决 / 认亲）
//
// 本文件只管"状态怎么流转"，判据与几何都在 armor_rules / armor_geometry 里：
//   ① 锚：绿灯没锁住就不动手（绿灯是窗口的锚，没锚不猜）
//   ② 窗口：armor_windows 决定本帧扫哪 1~2 个矩形
//   ③ track 优先：已有跟踪状态就只"认亲"，一条形状判据都不做
//   ④ detect：全门限 + O(n²) 配对 + 刚性先验（含先关联、再保持、最后才重选）
// ============================================================================

#include "detection/armor/armor_detector.hpp"

#include "detection/armor/armor_rules.hpp"

namespace dart::detection {

// 本 TU 内的简写：Bar 现在是 armor 命名空间里的值类型。
// （类内的 using Bar 只在类作用域可见，而限定名之前的返回类型不在类作用域里查。）
using armor::Bar;

ArmorDetector::ArmorDetector(const ArmorConfig &cfg, const MeasureConfig &meas_cfg)
    : cfg_(cfg), meas_(meas_cfg) {}

void ArmorDetector::reset() {
    a_ = Bar{};
    b_ = Bar{};
    have_pair_ = false;
    age_ = 0;
    armor::scale_reset(&scale_);
    r_last_ = 0.0f;
    trace_ = Trace{};
}

armor::PairView ArmorDetector::pair_view() const {
    armor::PairView pv;
    pv.a = &a_;
    pv.b = &b_;
    pv.have_pair = have_pair_;
    pv.r_last = r_last_;
    return pv;
}

void ArmorDetector::detect(const GrayFrame &bin, const TrackOutput &led, uint64_t now_us,
                           ArmorTarget *out) {
    trace_ = Trace{};
    *out = ArmorTarget{};
    const uint64_t t0 = now_us;

    // 1) 锚：跟踪层说"本帧有目标"才动手。它已经含了确认/门限/丢帧语义，
    //    这里不再自己判"锁没锁"（那是 TargetTracker 的职责）。
    if (!led.found || led.radius <= 0.0f)
        return;
    trace_.anchored = true;

    const int32_t s = static_cast<int32_t>(2.0f * led.radius + 0.5f); // 灯尺
    armor::scale_update(&scale_, s);
    trace_.scale = s;
    trace_.far = (static_cast<float>(s) < cfg_.tiny_px);

    // 2) 本帧扫哪里：
    //    · 首次/重捕 → "绿灯上方整块"
    //    · 跟踪中且绿灯够大 → **每条灯条各一个小窗**（只需盖住它自己，与板跨度无关）
    //    · 跟踪中但目标还小 → 一个合窗
    RoiWindow full{};
    if (!armor::led_window(cfg_, bin, led, &full))
        return;
    RoiWindow      wins[2];
    const uint32_t nwin = armor::search_windows(cfg_, pair_view(), full, led, bin, wins);
    trace_.windows = nwin;

    const uint32_t pxmin = armor::bar_px_min(cfg_, s);

    // 3) **track 优先**：已有跟踪状态 → 只"认亲"，一条形状判据都不做。
    //    判定"还是同一个目标"靠的就是"在它自己的窗里连续变化"：
    //    位置容差挡住跳到别处、尺寸容差挡住换了个大小的东西。
    //    （这是 detect-track 的分工：门限只在 detect 需要，track 只认。）
    if (have_pair_ && track_step(bin, wins, nwin, led, pxmin, out)) {
        trace_.cost_us = static_cast<uint32_t>(now_us - t0); // track 路径也要记耗时
        return;
    }

    // 4) detect：全门限（as_bar 长度/长宽比/填充率/圆度）+ O(n²) 配对 + 刚性先验
    //    （走到这里说明：没有跟踪状态，或者 track 认亲失败且保持到期）
    trace_.window_px = 0;
    trace_.blobs = 0;
    trace_.rejected = 0;
    uint32_t nbar = 0, nblob_total = 0, px_total = 0;
    for (uint32_t k = 0; k < nwin; ++k) {
        px_total += static_cast<uint32_t>(wins[k].pixels());
        const uint32_t nblob = meas_.collect(bin, wins[k], pxmin, blobs_, kArmorBarMax);
        nblob_total += nblob;
        for (uint32_t i = 0; i < nblob && nbar < kArmorBarMax; ++i) {
            Bar bar;
            if (!armor::as_bar(cfg_, blobs_[i], s, &bar)) {
                ++trace_.rejected;
                continue;
            }
            // 两个窗万一重叠，同一条灯条会被带出来两次 → 按包围盒去重
            // （不去重也不会配错：两条"同一条灯条"的间距为 0，会被配对门限挡掉，
            //   但白算一遍不划算，而且会把候选数灌水）
            bool dup = false;
            for (uint32_t j = 0; j < nbar; ++j)
                if (bars_[j].x == bar.x && bars_[j].y == bar.y && bars_[j].w == bar.w &&
                    bars_[j].h == bar.h)
                    dup = true;
            if (!dup)
                bars_[nbar++] = bar;
        }
    }
    trace_.blobs = nblob_total;
    trace_.bars = nbar;
    trace_.window_px = px_total;

    // 4) 检测-跟踪
    const bool ok = decide(bars_, nbar, led, s, out);
    trace_.pairs = pair_count_;
    trace_.rejected_prior = prior_reject_;
    trace_.cost_us = static_cast<uint32_t>(now_us - t0);
    (void)ok;
    fill_window(out, wins, nwin);
}

bool ArmorDetector::track_step(const GrayFrame &bin, const RoiWindow *wins, uint32_t nwin,
                               const TrackOutput &led, uint32_t pxmin, ArmorTarget *out) {
    Bar  na{}, nb{};
    bool ok = false;
    if (nwin == 2) { // 单条窗：每个窗里只认自己那条
        bool a_ok = false, b_ok = false;
        for (uint32_t k = 0; k < 2; ++k) {
            trace_.window_px += static_cast<uint32_t>(wins[k].pixels());
            const uint32_t n = meas_.collect(bin, wins[k], pxmin, blobs_, kArmorBarMax);
            trace_.blobs += n;
            const Bar &ref = (k == 0) ? a_ : b_;
            Bar        tmp{};
            if (assoc(blobs_, n, ref, &tmp)) {
                if (k == 0) {
                    na = tmp;
                    a_ok = true;
                } else {
                    nb = tmp;
                    b_ok = true;
                }
            }
        }
        ok = a_ok && b_ok;
    } else { // 合窗：两条 incumbent 在同一个池里各认各的
        trace_.window_px += static_cast<uint32_t>(wins[0].pixels());
        const uint32_t n = meas_.collect(bin, wins[0], pxmin, blobs_, kArmorBarMax);
        trace_.blobs += n;
        const bool a_ok = assoc(blobs_, n, a_, &na);
        const bool b_ok = assoc(blobs_, n, b_, &nb);
        ok = a_ok && b_ok && !(na.x == nb.x && na.y == nb.y && na.w == nb.w && na.h == nb.h);
    }
    if (ok) {
        int32_t cx = 0, cy = 0;
        if (armor::armor_center(na, nb, cfg_.diag_min_sin, cfg_.flat_deg, &cx, &cy) &&
            armor::same_body(cfg_, led, cx, cy)) {
            a_ = na;
            b_ = nb;
            c_[0] = cx;
            c_[1] = cy;
            age_ = 0;
            r_last_ = led.radius;
            trace_.mode = 1;
            trace_.bars = 2;
            *out = make_target(ArmorTarget::Fresh, 0);
            fill_window(out, wins, nwin);
            return true; // cost_us 由 detect() 统一算
        }
    }
    // 认不到（或认到的东西跑出锚的够得着范围）→ 记 miss，而不是静默给一个坏中心；
    // 连续超过 hold 帧就释放这一对、退回 detect 重捕。
    ++age_;
    trace_.assoc_fail = 1;
    if (age_ <= cfg_.hold) {
        trace_.mode = 1;
        *out = make_target(ArmorTarget::Held, static_cast<uint8_t>(age_));
        fill_window(out, wins, nwin);
        return true;
    }
    a_ = Bar{};
    b_ = Bar{};
    have_pair_ = false;
    age_ = 0;
    return false; // 落回 detect
}

bool ArmorDetector::decide(const Bar *bars, uint32_t n, const TrackOutput &led, int32_t s,
                         ArmorTarget *out) {
    pair_count_ = 0;
    prior_reject_ = 0;

    if (have_pair_) {
        const Bar *na = assoc(bars, n, a_);
        const Bar *nb = assoc(bars, n, b_);
        int32_t    cx, cy;
        if (na != nullptr && nb != nullptr && na != nb &&
            armor::armor_center(*na, *nb, cfg_.diag_min_sin, cfg_.flat_deg, &cx, &cy)) {
            a_ = *na;
            b_ = *nb;
            c_[0] = cx;
            c_[1] = cy;
            age_ = 0;
            r_last_ = led.radius; // 记下"这一对是在多大尺度下采到的"

            *out = make_target(ArmorTarget::Fresh, 0);
            ++pair_count_;
            return true;
        }
        ++age_;
        if (age_ <= cfg_.hold) {
            *out = make_target(ArmorTarget::Held, static_cast<uint8_t>(age_));
            return true; // 短暂丢失：先保持（旧中心 + Held 标记）
        }
        have_pair_ = false; // 保持到期 → 放弃这一对，重新 detect
        age_ = 0;
    }

    // 重选：两两配对，取分数最小者
    int32_t best_score = 0;
    Bar     best_a{}, best_b{};
    int32_t best_cx = 0, best_cy = 0, best_px = -1;
    bool    found = false;
    for (uint32_t i = 0; i < n; ++i) {
        for (uint32_t j = i + 1; j < n; ++j) {
            int32_t score;
            if (!armor::pair_score(cfg_, bars[i], bars[j], &score))
                continue;
            int32_t cx, cy;
            if (!armor::armor_center(bars[i], bars[j], cfg_.diag_min_sin, cfg_.flat_deg, &cx, &cy))
                continue;
            ++pair_count_;
            if (!armor::prior_ok(cfg_, led, s, bars[i], bars[j], cx, cy)) {
                ++prior_reject_;
                continue;
            }
            const int32_t px = static_cast<int32_t>(bars[i].px + bars[j].px);
            if (!found || score < best_score || (score == best_score && px > best_px)) {
                found = true;
                best_score = score;
                best_a = bars[i];
                best_b = bars[j];
                best_cx = cx;
                best_cy = cy;
                best_px = px;
            }
        }
    }
    if (!found)
        return false;
    a_ = best_a;
    b_ = best_b;
    c_[0] = best_cx;
    c_[1] = best_cy;
    have_pair_ = true;
    age_ = 0;
    r_last_ = led.radius;

    *out = make_target(ArmorTarget::Fresh, 0);
    return true;
}

bool ArmorDetector::assoc(const RunLengthMeasurer::BlobInfo *blobs, uint32_t n, const Bar &ref,
                          Bar *out) const {
    const float tx = cfg_.assoc_k * static_cast<float>(ref.w) > cfg_.assoc_min
                         ? cfg_.assoc_k * static_cast<float>(ref.w)
                         : cfg_.assoc_min;
    const float ty = cfg_.assoc_k * static_cast<float>(ref.h) > cfg_.assoc_min
                         ? cfg_.assoc_k * static_cast<float>(ref.h)
                         : cfg_.assoc_min;
    bool    found = false;
    int64_t best = 0;
    for (uint32_t i = 0; i < n; ++i) {
        Bar b{};
        armor::fill_bar(blobs[i], &b);
        if (static_cast<float>(armor::abs_i(b.cx() - ref.cx())) > tx ||
            static_cast<float>(armor::abs_i(b.cy() - ref.cy())) > ty ||
            static_cast<float>(armor::abs_i(b.w - ref.w)) > tx ||
            static_cast<float>(armor::abs_i(b.h - ref.h)) > ty)
            continue;
        const int64_t dx = b.cx() - ref.cx(), dy = b.cy() - ref.cy();
        const int64_t d2 = dx * dx + dy * dy;
        if (!found || d2 < best) {
            best = d2;
            found = true;
            *out = b;
        }
    }
    return found;
}

const ArmorDetector::Bar *ArmorDetector::assoc(const Bar *bars, uint32_t n, const Bar &ref) const {
    const float tx = cfg_.assoc_k * static_cast<float>(ref.w) > cfg_.assoc_min
                         ? cfg_.assoc_k * static_cast<float>(ref.w)
                         : cfg_.assoc_min;
    const float ty = cfg_.assoc_k * static_cast<float>(ref.h) > cfg_.assoc_min
                         ? cfg_.assoc_k * static_cast<float>(ref.h)
                         : cfg_.assoc_min;
    const Bar  *found = nullptr;
    int64_t     best = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const Bar &b = bars[i];
        if (static_cast<float>(armor::abs_i(b.cx() - ref.cx())) > tx ||
            static_cast<float>(armor::abs_i(b.cy() - ref.cy())) > ty ||
            static_cast<float>(armor::abs_i(b.w - ref.w)) > tx ||
            static_cast<float>(armor::abs_i(b.h - ref.h)) > ty)
            continue;
        const int64_t dx = b.cx() - ref.cx(), dy = b.cy() - ref.cy();
        const int64_t d2 = dx * dx + dy * dy;
        if (found == nullptr || d2 < best) {
            best = d2;
            found = &b;
        }
    }
    return found;
}

ArmorTarget ArmorDetector::make_target(uint8_t mode, uint8_t held) const {
    ArmorTarget t{};
    t.cx = c_[0];
    t.cy = c_[1];
    t.mode = mode;
    t.held = held;
    t.a_cx = a_.cx();
    t.b_cx = b_.cx();
    t.a_len = static_cast<uint32_t>(a_.len + 0.5f);
    t.b_len = static_cast<uint32_t>(b_.len + 0.5f);
    return t;
}

void ArmorDetector::fill_window(ArmorTarget *t, const RoiWindow *wins, uint32_t n) {
    if (n == 0 || wins[0].empty())
        return;
    uint32_t x0 = wins[0].x, y0 = wins[0].y, x1 = wins[0].right() - 1u, y1 = wins[0].bottom() - 1u;
    for (uint32_t i = 1; i < n; ++i) {
        if (wins[i].empty())
            continue;
        if (wins[i].x < x0)
            x0 = wins[i].x;
        if (wins[i].y < y0)
            y0 = wins[i].y;
        if (wins[i].right() - 1u > x1)
            x1 = wins[i].right() - 1u;
        if (wins[i].bottom() - 1u > y1)
            y1 = wins[i].bottom() - 1u;
    }
    t->x0 = x0;
    t->y0 = y0;
    t->x1 = x1;
    t->y1 = y1;
}

} // namespace dart::detection
