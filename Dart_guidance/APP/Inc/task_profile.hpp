#ifndef DART_GUIDANCE_TASK_PROFILE_HPP
#define DART_GUIDANCE_TASK_PROFILE_HPP

#include "guidance_types.h"
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
    GuidanceMeasurement_t measurement;
    GuidanceDelta_t delta;
    bool target_detected;
    bool task_finished;
    bool task_success;
} GreenLightTaskOutput_t;

typedef struct
{
    GreenLightTaskInput_t input;
    GreenLightTaskOutput_t output;
} GreenLightTaskProfile_t;

void GreenLightTaskProfile_ResetOutput(GreenLightTaskOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif
