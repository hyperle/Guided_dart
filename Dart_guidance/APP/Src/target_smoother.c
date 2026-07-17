#include "target_smoother.h"

#include <stddef.h>

#define GUIDANCE_TARGET_SMOOTHER_POSITION_Q_BITS 4U
#define GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE (1L << GUIDANCE_TARGET_SMOOTHER_POSITION_Q_BITS)
#define GUIDANCE_TARGET_SMOOTHER_POSITION_Q_HALF (GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE / 2L)

void GuidanceTargetSmoother_LoadDefaultConfig(GuidanceTargetSmootherConfig_t *config)
{
    if (config == NULL) {
        return;
    }

    config->enabled = (GUIDANCE_TARGET_SMOOTHER_DEFAULT_ENABLE != 0U);
    config->alpha_q15 = GUIDANCE_TARGET_SMOOTHER_DEFAULT_ALPHA_Q15;
    config->beta_q15 = GUIDANCE_TARGET_SMOOTHER_DEFAULT_BETA_Q15;
    config->max_velocity_px_per_frame = GUIDANCE_TARGET_SMOOTHER_DEFAULT_MAX_VELOCITY_PX_PER_FRAME;
}

void GuidanceTargetSmoother_Reset(GuidanceTargetSmoother_t *smoother)
{
    if (smoother == NULL) {
        return;
    }

    smoother->x_axis.position_q4 = 0;
    smoother->x_axis.velocity_q4 = 0;
    smoother->x_axis.initialized = false;
    smoother->y_axis.position_q4 = 0;
    smoother->y_axis.velocity_q4 = 0;
    smoother->y_axis.initialized = false;
}

void GuidanceTargetSmoother_Init(GuidanceTargetSmoother_t *smoother,
                                 const GuidanceTargetSmootherConfig_t *config)
{
    if (smoother == NULL) {
        return;
    }

    if (config != NULL) {
        smoother->config = *config;
    } else {
        GuidanceTargetSmoother_LoadDefaultConfig(&smoother->config);
    }

    GuidanceTargetSmoother_Reset(smoother);
}

bool GuidanceTargetSmoother_Update(GuidanceTargetSmoother_t *smoother,
                                   const GuidanceMeasurement_t *input,
                                   uint16_t image_width,
                                   uint16_t image_height,
                                   GuidanceMeasurement_t *output)
{
    uint16_t alpha_q15;
    uint16_t beta_q15;
    uint16_t x_limit;
    uint16_t y_limit;
    uint16_t measured_x;
    uint16_t measured_y;
    int32_t measured_x_q4;
    int32_t measured_y_q4;
    int32_t max_velocity_q4;
    int32_t predicted_x_q4;
    int32_t predicted_y_q4;
    int32_t residual_x_q4;
    int32_t residual_y_q4;
    int32_t correction_x_q4;
    int32_t correction_y_q4;
    int32_t velocity_correction_x_q4;
    int32_t velocity_correction_y_q4;
    int32_t x_limit_q4;
    int32_t y_limit_q4;

    if ((smoother == NULL) || (input == NULL) || (output == NULL)) {
        return false;
    }

    *output = *input;

    if ((input->x == GUIDANCE_NO_TARGET_COORDINATE) ||
        (input->y == GUIDANCE_NO_TARGET_COORDINATE)) {
        GuidanceTargetSmoother_Reset(smoother);
        return false;
    }

    if (!smoother->config.enabled || (image_width == 0U) || (image_height == 0U)) {
        GuidanceTargetSmoother_Reset(smoother);
        return true;
    }

    alpha_q15 = smoother->config.alpha_q15;
    beta_q15 = smoother->config.beta_q15;
    if (alpha_q15 > GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE) {
        alpha_q15 = GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE;
    }
    if (beta_q15 > GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE) {
        beta_q15 = GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE;
    }

    x_limit = (uint16_t)(image_width - 1U);
    y_limit = (uint16_t)(image_height - 1U);
    measured_x = (input->x > x_limit) ? x_limit : input->x;
    measured_y = (input->y > y_limit) ? y_limit : input->y;
    measured_x_q4 = (int32_t)measured_x * GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE;
    measured_y_q4 = (int32_t)measured_y * GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE;
    x_limit_q4 = (int32_t)x_limit * GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE;
    y_limit_q4 = (int32_t)y_limit * GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE;

    if (!smoother->x_axis.initialized || !smoother->y_axis.initialized) {
        smoother->x_axis.position_q4 = measured_x_q4;
        smoother->x_axis.velocity_q4 = 0;
        smoother->x_axis.initialized = true;
        smoother->y_axis.position_q4 = measured_y_q4;
        smoother->y_axis.velocity_q4 = 0;
        smoother->y_axis.initialized = true;
        output->x = measured_x;
        output->y = measured_y;
        return true;
    }

    predicted_x_q4 = smoother->x_axis.position_q4 + smoother->x_axis.velocity_q4;
    predicted_y_q4 = smoother->y_axis.position_q4 + smoother->y_axis.velocity_q4;
    residual_x_q4 = measured_x_q4 - predicted_x_q4;
    residual_y_q4 = measured_y_q4 - predicted_y_q4;

    correction_x_q4 = ((int32_t)alpha_q15 * residual_x_q4) /
                      (int32_t)GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE;
    correction_y_q4 = ((int32_t)alpha_q15 * residual_y_q4) /
                      (int32_t)GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE;
    velocity_correction_x_q4 = ((int32_t)beta_q15 * residual_x_q4) /
                               (int32_t)GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE;
    velocity_correction_y_q4 = ((int32_t)beta_q15 * residual_y_q4) /
                               (int32_t)GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE;

    smoother->x_axis.position_q4 = predicted_x_q4 + correction_x_q4;
    smoother->y_axis.position_q4 = predicted_y_q4 + correction_y_q4;
    smoother->x_axis.velocity_q4 += velocity_correction_x_q4;
    smoother->y_axis.velocity_q4 += velocity_correction_y_q4;

    max_velocity_q4 = (int32_t)smoother->config.max_velocity_px_per_frame *
                      GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE;
    if (smoother->x_axis.velocity_q4 > max_velocity_q4) {
        smoother->x_axis.velocity_q4 = max_velocity_q4;
    } else if (smoother->x_axis.velocity_q4 < -max_velocity_q4) {
        smoother->x_axis.velocity_q4 = -max_velocity_q4;
    }
    if (smoother->y_axis.velocity_q4 > max_velocity_q4) {
        smoother->y_axis.velocity_q4 = max_velocity_q4;
    } else if (smoother->y_axis.velocity_q4 < -max_velocity_q4) {
        smoother->y_axis.velocity_q4 = -max_velocity_q4;
    }

    if (smoother->x_axis.position_q4 < 0) {
        smoother->x_axis.position_q4 = 0;
        if (smoother->x_axis.velocity_q4 < 0) {
            smoother->x_axis.velocity_q4 = 0;
        }
    } else if (smoother->x_axis.position_q4 > x_limit_q4) {
        smoother->x_axis.position_q4 = x_limit_q4;
        if (smoother->x_axis.velocity_q4 > 0) {
            smoother->x_axis.velocity_q4 = 0;
        }
    }

    if (smoother->y_axis.position_q4 < 0) {
        smoother->y_axis.position_q4 = 0;
        if (smoother->y_axis.velocity_q4 < 0) {
            smoother->y_axis.velocity_q4 = 0;
        }
    } else if (smoother->y_axis.position_q4 > y_limit_q4) {
        smoother->y_axis.position_q4 = y_limit_q4;
        if (smoother->y_axis.velocity_q4 > 0) {
            smoother->y_axis.velocity_q4 = 0;
        }
    }

    output->x = (uint16_t)((smoother->x_axis.position_q4 +
                            GUIDANCE_TARGET_SMOOTHER_POSITION_Q_HALF) /
                           GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE);
    output->y = (uint16_t)((smoother->y_axis.position_q4 +
                            GUIDANCE_TARGET_SMOOTHER_POSITION_Q_HALF) /
                           GUIDANCE_TARGET_SMOOTHER_POSITION_Q_SCALE);
    return true;
}
