#include "pixel_delta_pid_action.hpp"

#include <stddef.h>

static float PixelDeltaPwmPidAction_ClampOutput(float output_us, float output_limit_us)
{
    if (output_limit_us < 0.0f) {
        output_limit_us = -output_limit_us;
    }
    if (output_us > output_limit_us) {
        output_us = output_limit_us;
    } else if (output_us < -output_limit_us) {
        output_us = -output_limit_us;
    }
    return output_us;
}

static TaskActionResult_t PixelDeltaPwmPidAction_OnRunning(TaskAction_t *action)
{
    PixelDeltaPwmPidAction_t *pid_action;
    PixelDeltaPwmPidActionProfile_t *profile;
    const GuidanceHorizontalPwmPidConfig_t *h_config;
    const GuidanceVerticalPwmPidConfig_t *v_config;
    float output_us;

    if (action == NULL) {
        return TASK_ACTION_FAILURE;
    }

    profile = (PixelDeltaPwmPidActionProfile_t *)action->profile;
    pid_action = (PixelDeltaPwmPidAction_t *)action->context;
    if ((profile == NULL) || (pid_action == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    if (!profile->input.target_detected) {
        profile->output.contribution.horizontal_pwm_active = false;
        profile->output.contribution.horizontal_pwm_us = 0.0f;
        profile->output.contribution.vertical_pwm_active = false;
        profile->output.contribution.vertical_pwm_us = 0.0f;
        return TASK_ACTION_FAILURE;
    }

    h_config = &profile->params.horizontal_pwm_pid_config;
    pid_action->pid.Kp = h_config->kp;
    pid_action->pid.Ki = h_config->ki;
    pid_action->pid.Kd = h_config->kd;
    pid_action->pid.integral_limit = h_config->integral_limit;

    output_us = PID_Update(&pid_action->pid, (float)profile->input.target_delta.delta_x);
    if (h_config->invert_output) {
        output_us = -output_us;
    }
    output_us = PixelDeltaPwmPidAction_ClampOutput(output_us, profile->params.output_limit_us);
    pid_action->pid.output = output_us;
    profile->output.contribution.horizontal_pwm_active = true;
    profile->output.contribution.horizontal_pwm_us = output_us;

    v_config = &profile->params.vertical_pwm_pid_config;
    pid_action->vertical_pid.Kp = v_config->kp;
    pid_action->vertical_pid.Ki = v_config->ki;
    pid_action->vertical_pid.Kd = v_config->kd;
    pid_action->vertical_pid.integral_limit = v_config->integral_limit;

    output_us = PID_Update(&pid_action->vertical_pid, (float)profile->input.target_delta.delta_y);
    if (v_config->invert_output) {
        output_us = -output_us;
    }
    output_us = PixelDeltaPwmPidAction_ClampOutput(output_us, profile->params.vertical_output_limit_us);
    pid_action->vertical_pid.output = output_us;
    profile->output.contribution.vertical_pwm_active = true;
    profile->output.contribution.vertical_pwm_us = output_us;

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
    PID_Init(&pid_action->vertical_pid,
             profile->params.vertical_pwm_pid_config.kp,
             profile->params.vertical_pwm_pid_config.ki,
             profile->params.vertical_pwm_pid_config.kd,
             (float)GUIDANCE_CONTROLLER_LOOP_PERIOD_MS * 0.001f,
             profile->params.vertical_pwm_pid_config.integral_limit);
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
    profile->output.contribution.vertical_pwm_active = false;
    profile->output.contribution.vertical_pwm_us = 0.0f;
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
    profile->params.vertical_pwm_pid_config.kp = GUIDANCE_VERTICAL_PID_KP;
    profile->params.vertical_pwm_pid_config.ki = GUIDANCE_VERTICAL_PID_KI;
    profile->params.vertical_pwm_pid_config.kd = GUIDANCE_VERTICAL_PID_KD;
    profile->params.vertical_pwm_pid_config.integral_limit = GUIDANCE_VERTICAL_PID_INTEGRAL_LIMIT;
    profile->params.vertical_pwm_pid_config.invert_output = GUIDANCE_VERTICAL_PID_INVERT_OUTPUT;
    profile->params.vertical_output_limit_us = GUIDANCE_VERTICAL_PID_OUTPUT_LIMIT_US;
    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
    profile->output.contribution.vertical_pwm_active = false;
    profile->output.contribution.vertical_pwm_us = 0.0f;
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
    PID_Init(&pid_action->vertical_pid,
             profile->params.vertical_pwm_pid_config.kp,
             profile->params.vertical_pwm_pid_config.ki,
             profile->params.vertical_pwm_pid_config.kd,
             (float)GUIDANCE_CONTROLLER_LOOP_PERIOD_MS * 0.001f,
             profile->params.vertical_pwm_pid_config.integral_limit);
    TaskAction_Init(&pid_action->action,
                    profile,
                    pid_action,
                    PixelDeltaPwmPidAction_OnEnter,
                    PixelDeltaPwmPidAction_OnRunning,
                    PixelDeltaPwmPidAction_OnExit);
    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
    profile->output.contribution.vertical_pwm_active = false;
    profile->output.contribution.vertical_pwm_us = 0.0f;
}

void PixelDeltaPwmPidAction_Reset(PixelDeltaPwmPidAction_t *pid_action)
{
    if ((pid_action == NULL) || (pid_action->profile == NULL)) {
        return;
    }

    PID_Reset(&pid_action->pid);
    PID_Reset(&pid_action->vertical_pid);
    pid_action->profile->output.contribution.horizontal_pwm_active = false;
    pid_action->profile->output.contribution.horizontal_pwm_us = 0.0f;
    pid_action->profile->output.contribution.vertical_pwm_active = false;
    pid_action->profile->output.contribution.vertical_pwm_us = 0.0f;
    TaskAction_Reset(&pid_action->action);
}

TaskActionResult_t PixelDeltaPwmPidAction_Tick(PixelDeltaPwmPidAction_t *pid_action)
{
    if ((pid_action == NULL) || (pid_action->profile == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    return TaskAction_Tick(&pid_action->action);
}
