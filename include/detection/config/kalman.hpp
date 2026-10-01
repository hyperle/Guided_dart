#pragma once

namespace dart::detection {

struct KfConfig {
    float sigma_a_pos = 400.0f;
    float sigma_a_scale = 15.0f;
    float r_pos = 2.0f;
    float r_scale_far = 25.0f;
    float r_scale_near = 0.6f;
    float s_far = 5.0f;
    float s_near = 35.0f;
    float r_max = 625.0f;
    float mahalanobis_gate = 11.34f;
    float divergence_inflate = 4.0f;
    unsigned divergence_max_streak = 8;
    float dt_min = 1e-4f;
    float dt_max = 0.20f;
    float init_pos_var = 25.0f;
    float init_vel_var = 4.0e4f;
    float init_scale_var = 100.0f;
    float init_scale_vel_var = 1.0e4f;
    float min_radius = 0.5f;
    unsigned reinit_after_lost = 8;
};

} // namespace dart::detection
