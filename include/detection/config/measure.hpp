#pragma once

#include <cstdint>

namespace dart::detection {

struct MeasureConfig {
    uint32_t min_area = 4;
    float    min_fill = 0.15f;
    float    min_circularity = 0.0f;
    float    circ_weight = 1.0f;
    float    aspect_lo = 0.15f;
    float    aspect_hi = 6.0f;
    float    clip_quality = 0.25f;
    uint32_t max_runs = 16384;
};

} // namespace dart::detection
