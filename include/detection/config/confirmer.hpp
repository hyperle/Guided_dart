#pragma once

#include <cstdint>

namespace dart::detection {

constexpr uint32_t kBlipWindowMax = 8;

struct ConfirmerConfig {
    uint32_t window = 3;
    uint32_t min_hits = 3;
    float    gate_px = 8.0f;
    float    gate_speed_slack = 0.6f;
    float    max_accel_px = 2.5f;
    float    max_travel_px = 160.0f;
    float    max_radius_rate = 120.0f;
    float    min_radius = 0.8f;
    bool     exclusive_match = true;
    bool     skip_border = true;
};

} // namespace dart::detection
