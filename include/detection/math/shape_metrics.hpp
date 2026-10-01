#pragma once

#include <cmath>

namespace dart::detection {

inline float roundness_from_moments(double area, double sx, double sy, double sxx, double syy,
                                    double sxy, double bw, double bh) {
    if (area <= 0.0)
        return 0.0f;
    const double mx = sx / area, my = sy / area;
    const double cxx = sxx / area - mx * mx;
    const double cyy = syy / area - my * my;
    const double cxy = sxy / area - mx * my;
    const double tr = cxx + cyy;
    const double det = cxx * cyy - cxy * cxy;
    const double disc = tr * tr * 0.25 - det;
    const double sq = disc > 0.0 ? std::sqrt(disc) : 0.0;
    const double lmax = tr * 0.5 + sq;
    const double lmin = tr * 0.5 - sq;

    double elong = 0.0;
    if (lmax <= 1e-9)
        elong = 0.6;
    else if (lmin > 0.0)
        elong = std::sqrt(lmin / lmax);

    const double box = bw * bh > 0.0 ? bw * bh : 1.0;
    double fill_norm = (area / box) / 0.7853981633974483;
    if (fill_norm > 1.0)
        fill_norm = 1.0;
    return static_cast<float>(elong * fill_norm);
}

inline float shape_score(float area, float circularity, float weight) {
    if (area <= 0.0f)
        return 0.0f;
    if (weight <= 0.0f)
        return area;
    const float c = circularity > 0.05f ? circularity : 0.05f;
    return area * std::pow(c, weight);
}

inline float radius_from_area(float area) {
    return area > 0.0f ? std::sqrt(area / 3.14159265358979323846f) : 0.0f;
}

inline float area_from_radius(float radius) {
    return radius > 0.0f ? 3.14159265358979323846f * radius * radius : 0.0f;
}

inline void principal_axis(double area, double sx, double sy, double sxx, double syy,
                           double sxy, float *theta, float *len_major, float *len_minor) {
    if (theta != nullptr)
        *theta = 0.0f;
    if (len_major != nullptr)
        *len_major = 0.0f;
    if (len_minor != nullptr)
        *len_minor = 0.0f;
    if (area <= 0.0)
        return;

    const double mx = sx / area, my = sy / area;
    const double cxx = sxx / area - mx * mx;
    const double cyy = syy / area - my * my;
    const double cxy = sxy / area - mx * my;
    const double tr = cxx + cyy;
    const double det = cxx * cyy - cxy * cxy;
    const double disc = tr * tr * 0.25 - det;
    const double sq = disc > 0.0 ? std::sqrt(disc) : 0.0;
    const double major = tr * 0.5 + sq;
    const double minor = tr * 0.5 - sq;

    if (theta != nullptr)
        *theta = static_cast<float>(0.5 * std::atan2(2.0 * cxy, cxx - cyy));
    if (len_major != nullptr)
        *len_major = major > 0.0 ? static_cast<float>(std::sqrt(12.0 * major)) : 0.0f;
    if (len_minor != nullptr)
        *len_minor = minor > 0.0 ? static_cast<float>(std::sqrt(12.0 * minor)) : 0.0f;
}

} // namespace dart::detection
