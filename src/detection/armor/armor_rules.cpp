// ============================================================================
// armor_rules.cpp —— armor_rules.hpp 的实现（灯条判据 / 配对 / 刚性先验）
// ============================================================================

#include "detection/armor/armor_rules.hpp"

#include <cmath>

#include "detection/armor/armor_scale.hpp"

namespace dart::detection::armor {

void fill_bar(const RoiBlobMeasurer::BlobInfo &b, Bar *out) {
    const int32_t w = static_cast<int32_t>(b.x1) - b.x0 + 1;
    const int32_t h = static_cast<int32_t>(b.y1) - b.y0 + 1;
    out->x = b.x0;
    out->y = b.y0;
    out->w = static_cast<int16_t>(w);
    out->h = static_cast<int16_t>(h);
    out->px = b.area;
    out->border = b.border;
    out->theta = b.theta;
    if (b.len_major > 0.0f && b.len_minor > 0.0f) {
        out->len = b.len_major;
        out->thick = b.len_minor;
    } else {
        out->len = static_cast<float>(w > h ? w : h);
        out->thick = static_cast<float>(w > h ? h : w);
        out->theta = (h >= w) ? 1.5707963267948966f : 0.0f;
    }
}

bool as_bar(const ArmorConfig &cfg, const RoiBlobMeasurer::BlobInfo &b, int32_t s, Bar *out) {
    const int32_t w = static_cast<int32_t>(b.x1) - b.x0 + 1;
    const int32_t h = static_cast<int32_t>(b.y1) - b.y0 + 1;
    int32_t       lmin, lmax, pxmin;
    scale_gate(cfg, s, s, &lmin, &lmax, &pxmin);
    if (b.area < static_cast<uint32_t>(pxmin) || w <= 0 || h <= 0)
        return false;
    out->x = b.x0;
    out->y = b.y0;
    out->w = static_cast<int16_t>(w);
    out->h = static_cast<int16_t>(h);
    out->px = b.area;
    out->border = b.border;
    out->theta = b.theta;
    // 主轴可用就用主轴（与旋转无关）；退化（1~2px 或病态矩）时退回包围盒口径
    if (b.len_major > 0.0f && b.len_minor > 0.0f) {
        out->len = b.len_major;
        out->thick = b.len_minor;
    } else {
        out->len = static_cast<float>(w > h ? w : h);
        out->thick = static_cast<float>(w > h ? h : w);
        out->theta = (h >= w) ? 1.5707963267948966f : 0.0f;
    }
    if (out->len < static_cast<float>(lmin) || out->len > static_cast<float>(lmax))
        return false;
    if (static_cast<float>(out->bbox_len()) >= cfg.tiny_px) {
        // 近档：长宽比用**主轴**长短边（斜灯条也成立）
        if (out->len < cfg.aspect_min * out->thick)
            return false;
        // 填充率只在"基本正放"时才是可信的：斜的时候包围盒被撑大，fill 天然变低
        // （30° 时 0.22），拿它筛等于筛掉真灯条。
        if (out->axis_skew_deg() <= cfg.flat_deg && b.fill < cfg.fill_min)
            return false;
    } else if (b.area > 1u && b.circularity > cfg.circ_max_far) {
        return false; // 远档：太圆 → 不是细长灯条（这条与尺度无关，3px 也成立）
    }
    return true;
}

bool pair_score(const ArmorConfig &cfg, const Bar &a, const Bar &b, int32_t *score) {
    const bool a_vert = a.h >= a.w;
    if (a_vert != (b.h >= b.w))
        return false; // 偏竖/偏横不能配（先粗筛，省掉下面的三角计算）

    // **倾角要一致**：这就是"两条灯条平行"这条已知事实的用法。只比"都竖"不够 ——
    // 两条都歪着、但歪的角度差很多，那不是一块板。轴是直线(mod π)，所以比 |cos Δθ|。
    const float dtheta = a.theta - b.theta;
    const float par = std::fabs(std::cos(dtheta));                // |cos Δθ|（轴 mod π）
    if (par < std::cos(cfg.parallel_deg * 0.017453292519943295f)) // parallel_deg 是度
        return false;

    // **两条一样粗**：同一块板上两条灯条厚度相同 → 厚度比卡一道。它能挡掉
    // "灯条上糊了一块亮斑"这种被撑变形的块（厚度被撑大，主轴分解一眼看出来）。
    {
        const float tmax = a.thick > b.thick ? a.thick : b.thick;
        const float tmin = a.thick > b.thick ? b.thick : a.thick;
        if (tmax > 0.0f && tmin < cfg.thick_ratio * tmax)
            return false;
    }
    int32_t la, lb, sep, half, lo, hi, off;
    if (a_vert) {
        la = static_cast<int32_t>(a.len + 0.5f);
        lb = static_cast<int32_t>(b.len + 0.5f);
        sep = abs_i(a.cx() - b.cx());
        half = a.w + b.w;
        lo = a.y > b.y ? a.y : b.y;
        const int32_t hia = a.y + a.h, hib = b.y + b.h;
        hi = hia < hib ? hia : hib;
        off = abs_i(a.cy() - b.cy());
    } else {
        la = static_cast<int32_t>(a.len + 0.5f);
        lb = static_cast<int32_t>(b.len + 0.5f);
        sep = abs_i(a.cy() - b.cy());
        half = a.h + b.h;
        lo = a.x > b.x ? a.x : b.x;
        const int32_t hia = a.x + a.w, hib = b.x + b.w;
        hi = hia < hib ? hia : hib;
        off = abs_i(a.cx() - b.cx());
    }
    const int32_t lmax = la > lb ? la : lb;
    const int32_t lmin = la > lb ? lb : la;
    if (lmax <= 0)
        return false;
    if (static_cast<float>(lmin) < cfg.len_ratio * static_cast<float>(lmax))
        return false;
    if (static_cast<float>(hi - lo) < cfg.overlap * static_cast<float>(lmin))
        return false;
    if (sep * 2 <= half)
        return false; // 横向重叠 → 在二值图上本来就是同一块
    if (static_cast<float>(sep) > cfg.gap_max_k * static_cast<float>(lmax))
        return false;
    *score = (lmax - lmin) * 100 / lmax + off * 100 / lmax +
             cfg.border_penalty * (a.border + b.border);
    return true;
}

bool same_body(const ArmorConfig &cfg, const TrackOutput &led, int32_t cx, int32_t cy) {
    const float r = led.radius;
    if (!(r > 0.0f))
        return true; // 半径没建立：没有可比的基准 → 放行
    const float dx = static_cast<float>(cx) - led.cx;
    const float dy = static_cast<float>(cy) - led.cy;
    float       lim = cfg.near_k * r;
    if (lim < cfg.near_min_px)
        lim = cfg.near_min_px; // 小半径下按"半径倍数"算会被量化噪声判死（见 ArmorConfig）
    return dx * dx + dy * dy <= lim * lim;
}

bool prior_ok(const ArmorConfig &cfg, const TrackOutput &led, int32_t s, const Bar &a, const Bar &b,
              int32_t cx, int32_t cy) {
    // ① 是不是**同一个整体**：板心到灯心的径向距离 < near_k × 绿灯半径。
    //    用径向（不是矩形框）：距离与倾角无关，板斜着转也不会把真目标切掉。
    if (!same_body(cfg, led, cx, cy))
        return false;
    if (s <= 0)
        return true; // 灯尺没建立：只剩①可比
    // ② 板的**尺寸**对不对：两灯条间距 ∝ 灯尺（板上间距是固定的）
    const float fs = static_cast<float>(s);
    const float span = static_cast<float>(pair_span(a, b));
    if (span < cfg.span_lo_k * fs || span > cfg.span_hi_k * fs)
        return false;
    return true;
}

} // namespace dart::detection::armor
