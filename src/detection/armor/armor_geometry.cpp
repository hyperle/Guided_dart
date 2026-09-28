// ============================================================================
// armor_geometry.cpp —— armor_geometry.hpp 的实现（纯几何，无状态）
// ============================================================================

#include "detection/armor/armor_geometry.hpp"

#include <cmath>

namespace dart::detection::armor {

namespace {

// 四舍五入整除（den > 0）。只被 line_intersection 用，所以不出头文件。
inline int64_t round_div(int64_t num, int64_t den) { return (2 * num + den) / (2 * den); }

} // namespace

void bar_ends(const Bar &b, float flat_deg, int32_t *x0, int32_t *y0, int32_t *x1, int32_t *y1) {
    if (b.axis_skew_deg() <= flat_deg || !(b.len > 0.0f)) {
        if (b.h >= b.w) {
            *x0 = b.cx();
            *y0 = b.y;
            *x1 = b.cx();
            *y1 = b.y + b.h - 1;
        } else {
            *x0 = b.x;
            *y0 = b.cy();
            *x1 = b.x + b.w - 1;
            *y1 = b.cy();
        }
        return;
    }
    const float c = std::cos(b.theta), sn = std::sin(b.theta);
    const float half = b.len * 0.5f;
    *x0 = b.cx() + static_cast<int32_t>(lroundf(c * half));
    *y0 = b.cy() + static_cast<int32_t>(lroundf(sn * half));
    *x1 = b.cx() - static_cast<int32_t>(lroundf(c * half));
    *y1 = b.cy() - static_cast<int32_t>(lroundf(sn * half));
}

bool line_intersection(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                       int32_t x3, int32_t y3, float min_sin, int32_t *ox, int32_t *oy) {
    const int64_t ax = static_cast<int64_t>(x1) - x0, ay = static_cast<int64_t>(y1) - y0;
    const int64_t bx = static_cast<int64_t>(x3) - x2, by = static_cast<int64_t>(y3) - y2;
    const int64_t d = ax * by - ay * bx;
    if (d == 0)
        return false; // 平行或共线：没有唯一交点

    // 夹角门限：|sin| = |d| / (|a|·|b|)，用整数百分比比。
    // 为什么不开方后用平方比较：d 最大 ~3.4e7（画幅边长量级），10000·d² ≈ 1.1e19
    // 会溢出 int64。两次 sqrt（double 在 3.4e7 内是精确的）代价可忽略。
    const double la = std::sqrt(static_cast<double>(ax * ax + ay * ay));
    const double lb = std::sqrt(static_cast<double>(bx * bx + by * by));
    if (la <= 0.0 || lb <= 0.0)
        return false;
    const double sin_abs = static_cast<double>(d < 0 ? -d : d) / (la * lb);
    if (sin_abs < static_cast<double>(min_sin))
        return false; // 近平行：交点对 1px 抖动极敏感，宁可不报

    int64_t tn = (static_cast<int64_t>(x2) - x0) * by - (static_cast<int64_t>(y2) - y0) * bx;
    int64_t un = (static_cast<int64_t>(x2) - x0) * ay - (static_cast<int64_t>(y2) - y0) * ax;
    int64_t dd = d;
    if (dd < 0) {
        dd = -dd;
        tn = -tn;
        un = -un;
    }
    if (tn < 0 || tn > dd || un < 0 || un > dd)
        return false; // 交点在延长线上 → 不是这个四边形的中心
    *ox = x0 + static_cast<int32_t>(round_div(ax * tn, dd));
    *oy = y0 + static_cast<int32_t>(round_div(ay * tn, dd));
    return true;
}

bool armor_center(const Bar &a, const Bar &b, float min_sin, float flat_deg, int32_t *cx,
                  int32_t *cy) {
    int32_t a0x, a0y, a1x, a1y, b0x, b0y, b1x, b1y;
    bar_ends(a, flat_deg, &a0x, &a0y, &a1x, &a1y);
    bar_ends(b, flat_deg, &b0x, &b0y, &b1x, &b1y);
    return line_intersection(a0x, a0y, b1x, b1y, a1x, a1y, b0x, b0y, min_sin, cx, cy);
}

int32_t pair_span(const Bar &a, const Bar &b) {
    return a.h >= a.w ? abs_i(a.cx() - b.cx()) : abs_i(a.cy() - b.cy());
}

} // namespace dart::detection::armor
