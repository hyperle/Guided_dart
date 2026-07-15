#ifndef DART_GUIDANCE_TARGET_SMOOTHER_H
#define DART_GUIDANCE_TARGET_SMOOTHER_H

#include "guidance_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Q15 gain base; 32768 means 1.0. */
#define GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE 32768U

/* Enable measurement alpha-beta smoothing by default. */
#define GUIDANCE_TARGET_SMOOTHER_DEFAULT_ENABLE 1U

/* alpha ~= 0.45, used for low-latency position correction. */
#define GUIDANCE_TARGET_SMOOTHER_DEFAULT_ALPHA_Q15 14746U

/* beta ~= 0.12, used to learn pixel velocity from position residual. */
#define GUIDANCE_TARGET_SMOOTHER_DEFAULT_BETA_Q15 3932U

/* Clamp velocity state so a single bad frame cannot pull prediction for long. */
#define GUIDANCE_TARGET_SMOOTHER_DEFAULT_MAX_VELOCITY_PX_PER_FRAME 64U

typedef struct
{
    bool enabled;
    uint16_t alpha_q15;
    uint16_t beta_q15;
    uint16_t max_velocity_px_per_frame;
} GuidanceTargetSmootherConfig_t;

typedef struct
{
    int32_t position_q4;
    int32_t velocity_q4;
    bool initialized;
} GuidanceTargetSmootherAxis_t;

typedef struct
{
    GuidanceTargetSmootherConfig_t config;
    GuidanceTargetSmootherAxis_t x_axis;
    GuidanceTargetSmootherAxis_t y_axis;
} GuidanceTargetSmoother_t;

void GuidanceTargetSmoother_LoadDefaultConfig(GuidanceTargetSmootherConfig_t *config);
void GuidanceTargetSmoother_Init(GuidanceTargetSmoother_t *smoother,
                                 const GuidanceTargetSmootherConfig_t *config);
void GuidanceTargetSmoother_Reset(GuidanceTargetSmoother_t *smoother);
bool GuidanceTargetSmoother_Update(GuidanceTargetSmoother_t *smoother,
                                   const GuidanceMeasurement_t *input,
                                   uint16_t image_width,
                                   uint16_t image_height,
                                   GuidanceMeasurement_t *output);

#ifdef __cplusplus
}
#endif

#endif
