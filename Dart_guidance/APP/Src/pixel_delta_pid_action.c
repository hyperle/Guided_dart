#include "pixel_delta_pid_action.hpp"

#include <stddef.h>

static TaskActionResult_t PixelDeltaPwmPidAction_OnRunning(TaskAction_t *action)
{
    PixelDeltaPwmPidAction_t *pid_action;
    PixelDeltaPwmPidActionProfile_t *profile;
    const GuidanceHorizontalPwmPidConfig_t *config;
    float output_us;
    float output_limit_us;

    if (action == NULL) {
        return TASK_ACTION_FAILURE;
    }

    profile = (PixelDeltaPwmPidActionProfile_t *)action->profile;
    pid_action = (PixelDeltaPwmPidAction_t *)action->context;
    if ((profile == NULL) || (pid_action == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    config = &profile->params.horizontal_pwm_pid_config;
    pid_action->pid.Kp = config->kp;
    pid_action->pid.Ki = config->ki;
    pid_action->pid.Kd = config->kd;
    pid_action->pid.integral_limit = config->integral_limit;

    if (!profile->input.target_detected) {
        profile->output.contribution.horizontal_pwm_active = false;
        profile->output.contribution.horizontal_pwm_us = 0.0f;
        return TASK_ACTION_FAILURE;
    }

    output_us = PID_Update(&pid_action->pid, (float)profile->input.target_delta.delta_x);
    if (config->invert_output) {
        output_us = -output_us;
    }

    output_limit_us = profile->params.output_limit_us;
    if (output_limit_us < 0.0f) {
        output_limit_us = -output_limit_us;
    }
    if (output_us > output_limit_us) {
        output_us = output_limit_us;
    } else if (output_us < -output_limit_us) {
        output_us = -output_limit_us;
    }
    pid_action->pid.output = output_us;

    profile->output.contribution.horizontal_pwm_active = true;
    profile->output.contribution.horizontal_pwm_us = output_us;
    return TASK_ACTION_RUNNING;
}

static TaskActionResult_t PixelDeltaPwmPidAction_OnEnter(TaskAction_t *action)
{
    PixelDeltaPwmPidAction_t *pid_action;
    PixelDeltaPwmPidActionProfile_t *profile;

    if (action == NULL) {
        return TASK_ACTION_FAILURE;
    }

    profile = (PixelDeltaPwmPidActionProfile_t *)action->profile;
    pid_action = (PixelDeltaPwmPidAction_t *)action->context;
    if ((profile == NULL) || (pid_action == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    PID_Init(&pid_action->pid,
             profile->params.horizontal_pwm_pid_config.kp,
             profile->params.horizontal_pwm_pid_config.ki,
             profile->params.horizontal_pwm_pid_config.kd,
             (float)GUIDANCE_CONTROLLER_LOOP_PERIOD_MS * 0.001f,
             profile->params.horizontal_pwm_pid_config.integral_limit);
    return PixelDeltaPwmPidAction_OnRunning(action);
}

static void PixelDeltaPwmPidAction_OnExit(TaskAction_t *action, TaskActionResult_t result)
{
    PixelDeltaPwmPidActionProfile_t *profile;

    (void)result;

    if (action == NULL) {
        return;
    }

    profile = (PixelDeltaPwmPidActionProfile_t *)action->profile;
    if (profile == NULL) {
        return;
    }

    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
}

void PixelDeltaPwmPidActionProfile_LoadDefault(PixelDeltaPwmPidActionProfile_t *profile)
{
    if (profile == NULL) {
        return;
    }

    profile->input.target_delta.delta_x = 0;
    profile->input.target_delta.delta_y = 0;
    profile->input.target_detected = false;
    profile->params.horizontal_pwm_pid_config.kp = GUIDANCE_HORIZONTAL_PID_KP;
    profile->params.horizontal_pwm_pid_config.ki = GUIDANCE_HORIZONTAL_PID_KI;
    profile->params.horizontal_pwm_pid_config.kd = GUIDANCE_HORIZONTAL_PID_KD;
    profile->params.horizontal_pwm_pid_config.integral_limit = GUIDANCE_HORIZONTAL_PID_INTEGRAL_LIMIT;
    profile->params.horizontal_pwm_pid_config.invert_output = GUIDANCE_HORIZONTAL_PID_INVERT_OUTPUT;
    profile->params.output_limit_us = GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US;
    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
}

void PixelDeltaPwmPidAction_Init(PixelDeltaPwmPidAction_t *pid_action,
                                 PixelDeltaPwmPidActionProfile_t *profile)
{
    if ((pid_action == NULL) || (profile == NULL)) {
        return;
    }

    pid_action->profile = profile;
    PID_Init(&pid_action->pid,
             profile->params.horizontal_pwm_pid_config.kp,
             profile->params.horizontal_pwm_pid_config.ki,
             profile->params.horizontal_pwm_pid_config.kd,
             (float)GUIDANCE_CONTROLLER_LOOP_PERIOD_MS * 0.001f,
             profile->params.horizontal_pwm_pid_config.integral_limit);
    TaskAction_Init(&pid_action->action,
                    profile,
                    pid_action,
                    PixelDeltaPwmPidAction_OnEnter,
                    PixelDeltaPwmPidAction_OnRunning,
                    PixelDeltaPwmPidAction_OnExit);
    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
}

void PixelDeltaPwmPidAction_Reset(PixelDeltaPwmPidAction_t *pid_action)
{
    if ((pid_action == NULL) || (pid_action->profile == NULL)) {
        return;
    }

    PID_Reset(&pid_action->pid);
    pid_action->profile->output.contribution.horizontal_pwm_active = false;
    pid_action->profile->output.contribution.horizontal_pwm_us = 0.0f;
    TaskAction_Reset(&pid_action->action);
}

TaskActionResult_t PixelDeltaPwmPidAction_Tick(PixelDeltaPwmPidAction_t *pid_action)
{
    if ((pid_action == NULL) || (pid_action->profile == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    return TaskAction_Tick(&pid_action->action);
}
