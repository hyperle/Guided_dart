#include "imu_velocity_recovery_action.hpp"

#include "servo.h"
#include <stddef.h>

void ImuVelocityRecoveryActionProfile_LoadDefault(ImuVelocityRecoveryActionProfile_t *profile)
{
    if (profile == NULL) {
        return;
    }

    profile->input.target_delta.delta_x = 0;
    profile->input.target_delta.delta_y = 0;
    profile->input.target_detected = false;
    profile->input.now_tick = 0U;
    profile->input.imu_velocity_x_dps = 0.0f;
    profile->input.launch_velocity_x_dps = 0.0f;
    profile->input.launch_velocity_ready = false;

    profile->params.base_pulse_us.values[0] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_0;
    profile->params.base_pulse_us.values[1] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_1;
    profile->params.base_pulse_us.values[2] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_2;
    profile->params.base_pulse_us.values[3] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_3;
    profile->params.delta_to_tick_scale = IMU_VELOCITY_RECOVERY_DELTA_TO_TICK_SCALE;
    profile->params.min_tick_period = IMU_VELOCITY_RECOVERY_MIN_TICK_PERIOD;
    profile->params.max_tick_period = IMU_VELOCITY_RECOVERY_MAX_TICK_PERIOD;
    profile->params.kick_pwm_us = IMU_VELOCITY_RECOVERY_KICK_PWM_US;
    profile->params.velocity_pid_config.kp = IMU_VELOCITY_RECOVERY_PID_KP;
    profile->params.velocity_pid_config.ki = IMU_VELOCITY_RECOVERY_PID_KI;
    profile->params.velocity_pid_config.kd = IMU_VELOCITY_RECOVERY_PID_KD;
    profile->params.velocity_pid_config.integral_limit = IMU_VELOCITY_RECOVERY_PID_INTEGRAL_LIMIT;
    profile->params.velocity_pid_config.invert_output = false;
    profile->params.velocity_pid_output_limit_us = IMU_VELOCITY_RECOVERY_PID_OUTPUT_LIMIT_US;
    profile->params.success_band_dps = IMU_VELOCITY_RECOVERY_SUCCESS_BAND_DPS;
    profile->params.success_hold_ticks = IMU_VELOCITY_RECOVERY_SUCCESS_HOLD_TICKS;
    profile->params.velocity_sample_wait_ticks = IMU_VELOCITY_RECOVERY_VELOCITY_SAMPLE_WAIT_TICKS;

    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
    profile->output.preview_pulse_us = profile->params.base_pulse_us;
    profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_IDLE;
    profile->output.delta_sign = 0;
    profile->output.tick_period = 0U;
    profile->output.target_velocity_x_dps = 0.0f;
    profile->output.velocity_error_x_dps = 0.0f;
    profile->output.target_velocity_latched = false;
}

void ImuVelocityRecoveryAction_Init(ImuVelocityRecoveryAction_t *recovery_action,
                                    ImuVelocityRecoveryActionProfile_t *profile)
{
    if ((recovery_action == NULL) || (profile == NULL)) {
        return;
    }

    recovery_action->profile = profile;
    recovery_action->state.start_tick = 0U;
    recovery_action->state.velocity_sample_wait_count = 0U;
    recovery_action->state.success_count = 0U;
    recovery_action->state.latched_delta_sign = 0;
    recovery_action->state.latched_tick_period = 0U;
    recovery_action->state.latched_target_velocity_x_dps = 0.0f;
    recovery_action->state.running = false;
    recovery_action->state.target_velocity_latched = false;
    PID_Init(&recovery_action->velocity_pid,
             profile->params.velocity_pid_config.kp,
             profile->params.velocity_pid_config.ki,
             profile->params.velocity_pid_config.kd,
             (float)GUIDANCE_CONTROLLER_LOOP_PERIOD_MS * 0.001f,
             profile->params.velocity_pid_config.integral_limit);
    TaskAction_Init(&recovery_action->action,
                    profile,
                    recovery_action,
                    ImuVelocityRecoveryAction_OnEnter,
                    ImuVelocityRecoveryAction_OnRunning,
                    ImuVelocityRecoveryAction_OnExit);
    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
    profile->output.preview_pulse_us = profile->params.base_pulse_us;
    profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_IDLE;
    profile->output.target_velocity_latched = false;
}

void ImuVelocityRecoveryAction_Reset(ImuVelocityRecoveryAction_t *recovery_action)
{
    if ((recovery_action == NULL) || (recovery_action->profile == NULL)) {
        return;
    }

    PID_Reset(&recovery_action->velocity_pid);
    recovery_action->state.start_tick = 0U;
    recovery_action->state.velocity_sample_wait_count = 0U;
    recovery_action->state.success_count = 0U;
    recovery_action->state.latched_delta_sign = 0;
    recovery_action->state.latched_tick_period = 0U;
    recovery_action->state.latched_target_velocity_x_dps = 0.0f;
    recovery_action->state.running = false;
    recovery_action->state.target_velocity_latched = false;
    recovery_action->profile->output.contribution.horizontal_pwm_active = false;
    recovery_action->profile->output.contribution.horizontal_pwm_us = 0.0f;
    recovery_action->profile->output.preview_pulse_us =
        recovery_action->profile->params.base_pulse_us;
    recovery_action->profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_IDLE;
    recovery_action->profile->output.delta_sign = 0;
    recovery_action->profile->output.tick_period = 0U;
    recovery_action->profile->output.target_velocity_x_dps = 0.0f;
    recovery_action->profile->output.velocity_error_x_dps = 0.0f;
    recovery_action->profile->output.target_velocity_latched = false;
    TaskAction_Reset(&recovery_action->action);
}

TaskActionResult_t ImuVelocityRecoveryAction_Tick(ImuVelocityRecoveryAction_t *recovery_action)
{
    if ((recovery_action == NULL) || (recovery_action->profile == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    return TaskAction_Tick(&recovery_action->action);
}

TaskActionResult_t ImuVelocityRecoveryAction_OnEnter(TaskAction_t *action)
{
    ImuVelocityRecoveryAction_t *recovery_action;
    ImuVelocityRecoveryActionProfile_t *profile;
    int32_t delta_x;
    uint32_t delta_magnitude;
    float tick_period;

    if (action == NULL) {
        return TASK_ACTION_FAILURE;
    }

    profile = (ImuVelocityRecoveryActionProfile_t *)action->profile;
    recovery_action = (ImuVelocityRecoveryAction_t *)action->context;
    if ((profile == NULL) || (recovery_action == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    PID_Init(&recovery_action->velocity_pid,
             profile->params.velocity_pid_config.kp,
             profile->params.velocity_pid_config.ki,
             profile->params.velocity_pid_config.kd,
             (float)GUIDANCE_CONTROLLER_LOOP_PERIOD_MS * 0.001f,
             profile->params.velocity_pid_config.integral_limit);

    recovery_action->state.start_tick = profile->input.now_tick;
    recovery_action->state.velocity_sample_wait_count = 0U;
    recovery_action->state.success_count = 0U;
    recovery_action->state.latched_target_velocity_x_dps = 0.0f;
    recovery_action->state.target_velocity_latched = false;
    recovery_action->state.running = true;

    delta_x = (int32_t)profile->input.target_delta.delta_x;
    if (delta_x > 0) {
        recovery_action->state.latched_delta_sign = 1;
        delta_magnitude = (uint32_t)delta_x;
    } else if (delta_x < 0) {
        recovery_action->state.latched_delta_sign = -1;
        delta_magnitude = (uint32_t)(-delta_x);
    } else {
        recovery_action->state.latched_delta_sign = 0;
        delta_magnitude = 0U;
    }

    tick_period = (float)delta_magnitude * profile->params.delta_to_tick_scale;
    if ((delta_magnitude > 0U) && (tick_period < (float)profile->params.min_tick_period)) {
        tick_period = (float)profile->params.min_tick_period;
    }
    if (tick_period > (float)profile->params.max_tick_period) {
        tick_period = (float)profile->params.max_tick_period;
    }
    if (tick_period < 0.0f) {
        tick_period = 0.0f;
    }
    recovery_action->state.latched_tick_period = (uint16_t)(tick_period + 0.5f);

    profile->output.delta_sign = recovery_action->state.latched_delta_sign;
    profile->output.tick_period = recovery_action->state.latched_tick_period;
    profile->output.target_velocity_x_dps = 0.0f;
    profile->output.velocity_error_x_dps = 0.0f;
    profile->output.target_velocity_latched = false;
    profile->output.preview_pulse_us = profile->params.base_pulse_us;

    if (!profile->input.target_detected) {
        profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_IDLE;
        profile->output.contribution.horizontal_pwm_active = false;
        profile->output.contribution.horizontal_pwm_us = 0.0f;
        return TASK_ACTION_FAILURE;
    }

    return ImuVelocityRecoveryAction_OnRunning(action);
}

TaskActionResult_t ImuVelocityRecoveryAction_OnRunning(TaskAction_t *action)
{
    ImuVelocityRecoveryAction_t *recovery_action;
    ImuVelocityRecoveryActionProfile_t *profile;
    GuidanceHorizontalPwmPidConfig_t *config;
    uint32_t elapsed_ticks;
    float horizontal_us;
    float output_limit_us;
    float success_band_dps;
    float velocity_error_x_dps;
    uint16_t required_success_ticks;
    uint16_t index;

    if (action == NULL) {
        return TASK_ACTION_FAILURE;
    }

    profile = (ImuVelocityRecoveryActionProfile_t *)action->profile;
    recovery_action = (ImuVelocityRecoveryAction_t *)action->context;
    if ((profile == NULL) || (recovery_action == NULL) || !recovery_action->state.running) {
        return TASK_ACTION_FAILURE;
    }

    config = &profile->params.velocity_pid_config;
    required_success_ticks = profile->params.success_hold_ticks;
    if (required_success_ticks == 0U) {
        required_success_ticks = 1U;
    }
    recovery_action->velocity_pid.Kp = config->kp;
    recovery_action->velocity_pid.Ki = config->ki;
    recovery_action->velocity_pid.Kd = config->kd;
    recovery_action->velocity_pid.integral_limit = config->integral_limit;

    for (index = 0U; index < GUIDANCE_SERVO_COUNT; ++index) {
        profile->output.preview_pulse_us.values[index] = profile->params.base_pulse_us.values[index];
    }

    elapsed_ticks = (uint32_t)(profile->input.now_tick - recovery_action->state.start_tick);
    if (elapsed_ticks < (uint32_t)recovery_action->state.latched_tick_period) {
        horizontal_us = (float)recovery_action->state.latched_delta_sign * profile->params.kick_pwm_us;
        profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_KICK;
        profile->output.contribution.horizontal_pwm_active = (recovery_action->state.latched_delta_sign != 0);
        profile->output.contribution.horizontal_pwm_us = horizontal_us;
        profile->output.delta_sign = recovery_action->state.latched_delta_sign;
        profile->output.tick_period = recovery_action->state.latched_tick_period;
        profile->output.target_velocity_latched = recovery_action->state.target_velocity_latched;
    } else {
        if (!recovery_action->state.target_velocity_latched) {
            if (!profile->input.launch_velocity_ready) {
                profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_WAIT_VELOCITY_SAMPLE;
                profile->output.contribution.horizontal_pwm_active = false;
                profile->output.contribution.horizontal_pwm_us = 0.0f;
                profile->output.delta_sign = recovery_action->state.latched_delta_sign;
                profile->output.tick_period = recovery_action->state.latched_tick_period;
                profile->output.target_velocity_latched = false;
                if (recovery_action->state.velocity_sample_wait_count <
                    profile->params.velocity_sample_wait_ticks) {
                    recovery_action->state.velocity_sample_wait_count += 1U;
                    return TASK_ACTION_RUNNING;
                }
                return TASK_ACTION_FAILURE;
            }

            recovery_action->state.latched_target_velocity_x_dps =
                profile->input.launch_velocity_x_dps;
            recovery_action->state.target_velocity_latched = true;
            recovery_action->state.success_count = 0U;
            PID_Reset(&recovery_action->velocity_pid);
        }

        velocity_error_x_dps =
            recovery_action->state.latched_target_velocity_x_dps - profile->input.imu_velocity_x_dps;
        horizontal_us = PID_Update(&recovery_action->velocity_pid, velocity_error_x_dps);
        if (config->invert_output) {
            horizontal_us = -horizontal_us;
        }

        output_limit_us = profile->params.velocity_pid_output_limit_us;
        if (output_limit_us < 0.0f) {
            output_limit_us = -output_limit_us;
        }
        if (horizontal_us > output_limit_us) {
            horizontal_us = output_limit_us;
        } else if (horizontal_us < -output_limit_us) {
            horizontal_us = -output_limit_us;
        }
        recovery_action->velocity_pid.output = horizontal_us;

        success_band_dps = profile->params.success_band_dps;
        if (success_band_dps < 0.0f) {
            success_band_dps = -success_band_dps;
        }
        if ((velocity_error_x_dps <= success_band_dps) &&
            (velocity_error_x_dps >= -success_band_dps)) {
            if (recovery_action->state.success_count < required_success_ticks) {
                recovery_action->state.success_count += 1U;
            }
        } else {
            recovery_action->state.success_count = 0U;
        }

        profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_PID_RECOVERY;
        profile->output.contribution.horizontal_pwm_active = true;
        profile->output.contribution.horizontal_pwm_us = horizontal_us;
        profile->output.delta_sign = recovery_action->state.latched_delta_sign;
        profile->output.tick_period = recovery_action->state.latched_tick_period;
        profile->output.target_velocity_x_dps =
            recovery_action->state.latched_target_velocity_x_dps;
        profile->output.velocity_error_x_dps = velocity_error_x_dps;
        profile->output.target_velocity_latched = true;
    }

    if (profile->output.contribution.horizontal_pwm_active) {
        horizontal_us = profile->output.contribution.horizontal_pwm_us;
        profile->output.preview_pulse_us.values[0] -= horizontal_us;
        profile->output.preview_pulse_us.values[1] += horizontal_us;
        profile->output.preview_pulse_us.values[2] += horizontal_us;
        profile->output.preview_pulse_us.values[3] -= horizontal_us;
    }

    for (index = 0U; index < GUIDANCE_SERVO_COUNT; ++index) {
        if (profile->output.preview_pulse_us.values[index] < (float)SERVO_PULSE_MIN_US) {
            profile->output.preview_pulse_us.values[index] = (float)SERVO_PULSE_MIN_US;
        } else if (profile->output.preview_pulse_us.values[index] > (float)SERVO_PULSE_MAX_US) {
            profile->output.preview_pulse_us.values[index] = (float)SERVO_PULSE_MAX_US;
        }
    }

    if ((profile->output.stage == IMU_VELOCITY_RECOVERY_STAGE_PID_RECOVERY) &&
        (recovery_action->state.success_count >= required_success_ticks)) {
        profile->output.stage = IMU_VELOCITY_RECOVERY_STAGE_DONE;
        return TASK_ACTION_SUCCESS;
    }

    return TASK_ACTION_RUNNING;
}

void ImuVelocityRecoveryAction_OnExit(TaskAction_t *action, TaskActionResult_t result)
{
    ImuVelocityRecoveryAction_t *recovery_action;
    ImuVelocityRecoveryActionProfile_t *profile;

    (void)result;

    if (action == NULL) {
        return;
    }

    profile = (ImuVelocityRecoveryActionProfile_t *)action->profile;
    recovery_action = (ImuVelocityRecoveryAction_t *)action->context;
    if ((profile == NULL) || (recovery_action == NULL)) {
        return;
    }

    recovery_action->state.running = false;
    profile->output.contribution.horizontal_pwm_active = false;
    profile->output.contribution.horizontal_pwm_us = 0.0f;
    profile->output.preview_pulse_us = profile->params.base_pulse_us;
}
