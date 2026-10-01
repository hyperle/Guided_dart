#pragma once

namespace dart::detection {

struct RoiConfig {
    float kp = 5.0f;
    float margin = 24.0f;
    float k_sigma = 2.0f;
    float grow_per_lost = 0.6f;
    float grow_max = 4.0f;
    float out_of_roi_grow = 1.6f;
    float min_side = 24.0f;
    float rescan_area_frac = 0.35f;
    float max_side_frac = 0.95f;
};

} // namespace dart::detection
