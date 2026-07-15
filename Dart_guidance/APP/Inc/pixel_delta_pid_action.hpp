#ifndef DART_GUIDANCE_PIXEL_DELTA_PID_ACTION_HPP
#define DART_GUIDANCE_PIXEL_DELTA_PID_ACTION_HPP

#include "guidance_controller.h"
#include "guidance_types.h"
#include "pid.h"
#include "task_action.hpp"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    GuidanceDelta_t target_delta;
    bool target_detected;
} PixelDeltaPwmPidActionInput_t;

typedef struct
{
    GuidanceHorizontalPwmPidConfig_t horizontal_pwm_pid_config;
    float output_limit_us;
} PixelDeltaPwmPidActionParams_t;

typedef struct
{
    GuidanceControlContribution_t contribution;
} PixelDeltaPwmPidActionOutput_t;

typedef struct
{
    PixelDeltaPwmPidActionInput_t input;
    PixelDeltaPwmPidActionParams_t params;
    PixelDeltaPwmPidActionOutput_t output;
} PixelDeltaPwmPidActionProfile_t;

typedef struct
{
    PixelDeltaPwmPidActionProfile_t *profile;
    TaskAction_t action;
    PID_t pid;
} PixelDeltaPwmPidAction_t;

void PixelDeltaPwmPidActionProfile_LoadDefault(PixelDeltaPwmPidActionProfile_t *profile);
void PixelDeltaPwmPidAction_Init(PixelDeltaPwmPidAction_t *pid_action,
                                 PixelDeltaPwmPidActionProfile_t *profile);
void PixelDeltaPwmPidAction_Reset(PixelDeltaPwmPidAction_t *pid_action);
TaskActionResult_t PixelDeltaPwmPidAction_Tick(PixelDeltaPwmPidAction_t *pid_action);

#ifdef __cplusplus
}
#endif

#endif
