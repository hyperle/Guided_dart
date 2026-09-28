// ============================================================================
// armor_windows.cpp —— armor_windows.hpp 的实现（窗口规划，不碰像素）
// ============================================================================

#include "detection/armor/armor_windows.hpp"

namespace dart::detection::armor {

bool clamp_window(float x, float y, float w, float h, uint32_t fw, uint32_t fh, float min_side,
                  RoiWindow *out) {
    int32_t ix = static_cast<int32_t>(x);
    int32_t iy = static_cast<int32_t>(y);
    int32_t iw = static_cast<int32_t>(w);
    int32_t ih = static_cast<int32_t>(h);
    if (ix < 0) {
        iw += ix;
        ix = 0;
    }
    if (iy < 0) {
        ih += iy;
        iy = 0;
    }
    if (ix + iw > static_cast<int32_t>(fw))
        iw = static_cast<int32_t>(fw) - ix;
    if (iy + ih > static_cast<int32_t>(fh))
        ih = static_cast<int32_t>(fh) - iy;
    if (iw < static_cast<int32_t>(min_side) || ih < static_cast<int32_t>(min_side))
        return false; // 窗口被裁得没意义：本帧不出装甲板结果（宁可不报）
    out->x = static_cast<uint32_t>(ix);
    out->y = static_cast<uint32_t>(iy);
    out->w = static_cast<uint32_t>(iw);
    out->h = static_cast<uint32_t>(ih);
    out->full_frame = false;
    return true;
}

bool led_window(const ArmorConfig &cfg, const GrayFrame &f, const TrackOutput &led,
                RoiWindow *win) {
    const float r = led.radius;
    float       rw = cfg.roi_w_k * 2.0f * r;
    float       rh = cfg.roi_h_k * 2.0f * r;
    const float gap = cfg.roi_gap_k * 2.0f * r;
    if (rw < cfg.roi_min_side)
        rw = cfg.roi_min_side;
    if (rh < cfg.roi_min_side)
        rh = cfg.roi_min_side;
    const float top = led.cy - r; // 绿灯上沿
    float       x = led.cx - rw * 0.5f;
    float       y = top - gap - rh;
    return clamp_window(x, y, rw, rh, f.width, f.height, cfg.roi_min_side, win);
}

int32_t bar_margin(const ArmorConfig &cfg, float bar_len_px, int32_t pad) {
    float m = cfg.bar_margin_k * bar_len_px;
    if (m < cfg.bar_margin_min)
        m = cfg.bar_margin_min;
    else if (m > cfg.bar_margin_max)
        m = cfg.bar_margin_max;
    return static_cast<int32_t>(m) + pad;
}

int32_t grow_pad(const PairView &pair, const TrackOutput &led, float bar_len_px) {
    if (pair.r_last <= 0.0f || !(led.radius > pair.r_last))
        return 0;
    return static_cast<int32_t>(bar_len_px * (led.radius / pair.r_last - 1.0f) * 0.5f);
}

bool reach_box(const ArmorConfig &cfg, const TrackOutput &led, const GrayFrame &f, RoiWindow *out) {
    const float half = cfg.near_k * led.radius;
    return clamp_window(led.cx - half, led.cy - half, half * 2.0f, half * 2.0f, f.width, f.height,
                        4.0f, out);
}

bool bar_window(const ArmorConfig &cfg, const PairView &pair, const Bar &bar, const TrackOutput &led,
                const RoiWindow &reach, const GrayFrame &f, RoiWindow *out) {
    const int32_t m = bar_margin(cfg, bar.len, grow_pad(pair, led, bar.len));
    const int32_t x = bar.x - m, y = bar.y - m;
    const int32_t w = bar.w + 2 * m, h = bar.h + 2 * m;
    // 与"够得着"范围取交（都是闭区间外的开区间表示：x..x+w-1）
    int32_t rx = x, ry = y, rw = w, rh = h;
    if (rx < static_cast<int32_t>(reach.x)) {
        rw -= (static_cast<int32_t>(reach.x) - rx);
        rx = static_cast<int32_t>(reach.x);
    }
    if (ry < static_cast<int32_t>(reach.y)) {
        rh -= (static_cast<int32_t>(reach.y) - ry);
        ry = static_cast<int32_t>(reach.y);
    }
    const int32_t rx1 = static_cast<int32_t>(reach.right());
    const int32_t ry1 = static_cast<int32_t>(reach.bottom());
    if (rx + rw > rx1)
        rw = rx1 - rx;
    if (ry + rh > ry1)
        rh = ry1 - ry;
    if (rw < 8 || rh < 8)
        return false;
    return clamp_window(static_cast<float>(rx), static_cast<float>(ry), static_cast<float>(rw),
                        static_cast<float>(rh), f.width, f.height, 8.0f, out);
}

RoiWindow union_window(const ArmorConfig &cfg, const PairView &pair, const RoiWindow &full,
                       const TrackOutput &led) {
    if (!pair.have_pair)
        return full;
    const Bar    &a_ = *pair.a;
    const Bar    &b_ = *pair.b;
    const int32_t x0 = a_.x < b_.x ? a_.x : b_.x;
    const int32_t y0 = a_.y < b_.y ? a_.y : b_.y;
    const int32_t ax1 = a_.x + a_.w, bx1 = b_.x + b_.w;
    const int32_t ay1 = a_.y + a_.h, by1 = b_.y + b_.h;
    const int32_t x1 = ax1 > bx1 ? ax1 : bx1;
    const int32_t y1 = ay1 > by1 ? ay1 : by1;
    const float   side = a_.len > b_.len ? a_.len : b_.len;
    const int32_t m = bar_margin(cfg, side, grow_pad(pair, led, side));
    int32_t       sx = x0 - m, sy = y0 - m;
    int32_t       sw = (x1 - x0) + 2 * m, sh = (y1 - y0) + 2 * m;
    if (sx < static_cast<int32_t>(full.x)) {
        sw -= (static_cast<int32_t>(full.x) - sx);
        sx = static_cast<int32_t>(full.x);
    }
    if (sy < static_cast<int32_t>(full.y)) {
        sh -= (static_cast<int32_t>(full.y) - sy);
        sy = static_cast<int32_t>(full.y);
    }
    const int32_t fx1 = static_cast<int32_t>(full.right());
    const int32_t fy1 = static_cast<int32_t>(full.bottom());
    if (sx + sw > fx1)
        sw = fx1 - sx;
    if (sy + sh > fy1)
        sh = fy1 - sy;
    if (sw < 8 || sh < 8)
        return full; // 小窗被挤没了：退回整块（宁可多扫，不可漏掉）
    RoiWindow w{};
    w.x = static_cast<uint32_t>(sx);
    w.y = static_cast<uint32_t>(sy);
    w.w = static_cast<uint32_t>(sw);
    w.h = static_cast<uint32_t>(sh);
    w.full_frame = false;
    return w;
}

uint32_t search_windows(const ArmorConfig &cfg, const PairView &pair, const RoiWindow &full,
                        const TrackOutput &led, const GrayFrame &f, RoiWindow *out) {
    out[0] = full;
    if (!pair.have_pair)
        return 1;
    if (!(led.radius >= cfg.per_bar_r)) { // 远距离：合窗（单条窗太小不可靠）
        out[0] = union_window(cfg, pair, full, led);
        return 1;
    }
    RoiWindow reach{};
    if (!reach_box(cfg, led, f, &reach)) {
        out[0] = union_window(cfg, pair, full, led);
        return 1;
    }
    uint32_t    n = 0;
    const Bar  *bars[2] = {pair.a, pair.b};
    for (int i = 0; i < 2; ++i) {
        RoiWindow w{};
        if (bar_window(cfg, pair, *bars[i], led, reach, f, &w))
            out[n++] = w;
    }
    if (n < 2) { // 有窗建不起来 → 整帧退回合窗（宁可多扫，不可漏）
        out[0] = union_window(cfg, pair, full, led);
        return 1;
    }
    return n;
}

} // namespace dart::detection::armor
