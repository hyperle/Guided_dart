#pragma once

#include <cstdint>

namespace dart::detection {

struct ScannerConfig {
    uint32_t tile = 16;
    uint32_t min_area = 3;
    uint32_t max_area_frac_pct = 25;
    uint32_t top_k = 8;
    uint32_t max_components = 96;
    uint32_t refine_max_px = 4096;
    float    min_fill = 0.10f;
    float    circ_weight = 1.0f;
    float    min_circularity = 0.0f;
};

} // namespace dart::detection
