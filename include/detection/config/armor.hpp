#pragma once

#include <cstdint>

namespace dart::detection {

struct ArmorConfig {
    bool enable = true;
    float len_min_k = 0.2f;
    float len_max_k = 8.0f;
    uint32_t len_floor = 2;
    uint32_t px_floor = 2;
    float tiny_px = 8.0f;
    float aspect_min = 2.0f;
    float fill_min = 0.5f;
    float circ_max_far = 0.85f;
    float flat_deg = 12.0f;
    float parallel_deg = 15.0f;
    float thick_ratio = 0.3f;
    float len_ratio = 0.4f;
    float overlap = 0.4f;
    float gap_max_k = 10.0f;
    float diag_min_sin = 0.30f;
    int32_t border_penalty = 30;
    float roi_w_k = 3.0f;
    float roi_h_k = 2.0f;
    float roi_gap_k = 0.0f;
    float roi_min_side = 24.0f;
    float per_bar_r = 20.0f;
    float bar_margin_k = 0.25f;
    float bar_margin_min = 8.0f;
    float bar_margin_max = 24.0f;
    float scale_v_k = 1.0f;
    float scale_v_max = 4.0f;
    uint32_t hold = 2;
    float assoc_k = 0.6f;
    float assoc_min = 4.0f;
    float near_k = 8.0f;
    float near_min_px = 32.0f;
    float span_lo_k = 0.2f;
    float span_hi_k = 12.0f;
};

constexpr uint32_t kArmorBarMax = 32;

} // namespace dart::detection
