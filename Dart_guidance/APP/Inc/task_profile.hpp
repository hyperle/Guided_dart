#ifndef DART_GUIDANCE_TASK_PROFILE_HPP
#define DART_GUIDANCE_TASK_PROFILE_HPP

#include "guidance_types.h"
#include "target_smoother.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t measurement_width;
    uint16_t measurement_height;
    GuidanceMeasurement_t upstream_measurement;
    bool upstream_measurement_ready;
    GuidanceSetpoint_t setpoint;
} GreenLightTaskInput_t;

typedef struct
{
    GuidanceTargetSmootherConfig_t measurement_smoother_config;
} GreenLightTaskParams_t;

typedef struct
{
    GuidanceMeasurement_t measurement;
    GuidanceDelta_t delta;
    bool target_detected;
} GreenLightTaskOutput_t;

typedef struct
{
    GreenLightTaskInput_t input;
    GreenLightTaskParams_t params;
    GreenLightTaskOutput_t output;
} GreenLightTaskProfile_t;

void GreenLightTaskProfile_ResetOutput(GreenLightTaskOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif
