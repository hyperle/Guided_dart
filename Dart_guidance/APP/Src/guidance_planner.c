#include "guidance_planner.h"

#include <math.h>
#include <stddef.h>

void GuidancePlanner_Init(GuidancePlanner_t *planner)
{
    if (planner == NULL) {
        return;
    }

    TimedTurnPulseActionProfile_LoadDefault(&planner->timed_turn_profile);
    ImuVelocityRecoveryActionProfile_LoadDefault(&planner->yaw_recovery_profile);
    ImuVelocityRecoveryAction_Init(&planner->yaw_recovery_action,
                                   &planner->yaw_recovery_profile);
    GuidancePlanner_Reset(planner);
}

void GuidancePlanner_Reset(GuidancePlanner_t *planner)
{
    if (planner == NULL) {
        return;
    }

    planner->phase = GUIDANCE_PLANNER_PHASE_IDLE;
    planner->timed_turn_tick = 0U;
    planner->yaw_recovery_tick = 0U;
    planner->launch_wait_ticks = 0U;
    planner->waiting_for_flight_end = false;
    planner->launch_threshold_active = false;
    planner->half_threshold_active = false;
    planner->launch_velocity_ready = false;
    planner->launch_velocity_x_dps = 0.0f;
    planner->launch_velocity_y_dps = 0.0f;
    planner->launch_velocity_z_dps = 0.0f;

    planner->timed_turn_profile.input.now_tick = 0U;
    planner->timed_turn_profile.input.enabled = true;
    planner->timed_turn_profile.output.pulse_us =
        planner->timed_turn_profile.params.base_pulse_us;
    planner->timed_turn_profile.output.stage = TIMED_TURN_PULSE_ACTION_STAGE_WAIT;
    planner->timed_turn_profile.output.active_segment_index = 0U;
    planner->timed_turn_profile.output.active_pwm_us = 0.0f;
    planner->timed_turn_profile.output.pwm_active = false;
    ImuVelocityRecoveryAction_Reset(&planner->yaw_recovery_action);

    planner->output.mode = GUIDANCE_PLANNER_MODE_DEFAULT_PID;
    planner->output.task_result = TASK_ACTION_RUNNING;
    planner->output.override_pulse_us = NULL;
    planner->output.override_contribution = NULL;
    planner->output.timed_turn_output = &planner->timed_turn_profile.output;
    planner->output.yaw_recovery_output = &planner->yaw_recovery_profile.output;
    planner->output.default_pid_task_enabled = true;
    planner->output.override_pwm_active = false;
    planner->output.override_contribution_active = false;
    planner->output.timed_turn_started = false;
    planner->output.timed_turn_active = false;
    planner->output.timed_turn_finished = false;
    planner->output.yaw_recovery_started = false;
    planner->output.yaw_recovery_active = false;
    planner->output.yaw_recovery_finished = false;
    planner->output.flight_end_detected = false;
    planner->output.launch_velocity_ready = false;
    planner->output.launch_velocity_x_dps = 0.0f;
    planner->output.launch_velocity_y_dps = 0.0f;
    planner->output.launch_velocity_z_dps = 0.0f;
}

TaskActionResult_t GuidancePlanner_Tick(GuidancePlanner_t *planner,
                                        const GuidancePlannerInput_t *input)
{
    float selected_accel_g = 0.0f;
    float accel_abs_mps2 = 0.0f;
    float launch_threshold_mps2 = 0.0f;
    float flight_end_threshold_mps2 = 0.0f;
    bool launch_threshold_active = false;
    bool half_threshold_active = false;
    bool imu_event_valid = false;
    bool launch_event = false;
    bool flight_end_event = false;
    TaskActionResult_t timed_turn_result = TASK_ACTION_RUNNING;
    TaskActionResult_t yaw_recovery_result = TASK_ACTION_RUNNING;

    if (planner == NULL) {
        return TASK_ACTION_FAILURE;
    }

    planner->output.mode = GUIDANCE_PLANNER_MODE_DEFAULT_PID;
    planner->output.task_result = TASK_ACTION_RUNNING;
    planner->output.override_pulse_us = NULL;
    planner->output.override_contribution = NULL;
    planner->output.timed_turn_output = &planner->timed_turn_profile.output;
    planner->output.yaw_recovery_output = &planner->yaw_recovery_profile.output;
    planner->output.default_pid_task_enabled = true;
    planner->output.override_pwm_active = false;
    planner->output.override_contribution_active = false;
    planner->output.timed_turn_started = false;
    planner->output.timed_turn_active = false;
    planner->output.timed_turn_finished = false;
    planner->output.yaw_recovery_started = false;
    planner->output.yaw_recovery_active = false;
    planner->output.yaw_recovery_finished = false;
    planner->output.flight_end_detected = false;
    planner->output.launch_velocity_ready = planner->launch_velocity_ready;
    planner->output.launch_velocity_x_dps = planner->launch_velocity_x_dps;
    planner->output.launch_velocity_y_dps = planner->launch_velocity_y_dps;
    planner->output.launch_velocity_z_dps = planner->launch_velocity_z_dps;

    imu_event_valid = ((input != NULL) && input->imu_data_valid);
    if (imu_event_valid) {
        switch (input->accel_axis) {
        case IMU_ACCEL_AXIS_Y:
            selected_accel_g = input->accel_y_g;
            break;
        case IMU_ACCEL_AXIS_Z:
            selected_accel_g = input->accel_z_g;
            break;
        case IMU_ACCEL_AXIS_X:
        default:
            selected_accel_g = input->accel_x_g;
            break;
        }

        accel_abs_mps2 = fabsf(selected_accel_g * IMU_STANDARD_GRAVITY_MPS2);
        launch_threshold_mps2 = fabsf(input->accel_threshold_mps2);
        flight_end_threshold_mps2 =
            launch_threshold_mps2 * GUIDANCE_PLANNER_FLIGHT_END_ACCEL_THRESHOLD_RATIO;
        launch_threshold_active = (accel_abs_mps2 > launch_threshold_mps2);
        half_threshold_active = (accel_abs_mps2 > flight_end_threshold_mps2);
    }

    if (imu_event_valid) {
        flight_end_event = planner->waiting_for_flight_end &&
                           (planner->phase == GUIDANCE_PLANNER_PHASE_IDLE) &&
                           half_threshold_active &&
                           !planner->half_threshold_active;
        launch_event = !planner->waiting_for_flight_end &&
                       (planner->phase == GUIDANCE_PLANNER_PHASE_IDLE) &&
                       launch_threshold_active &&
                       !planner->launch_threshold_active;
        planner->launch_threshold_active = launch_threshold_active;
        planner->half_threshold_active = half_threshold_active;
    }

    if (flight_end_event) {
        planner->phase = GUIDANCE_PLANNER_PHASE_IDLE;
        planner->timed_turn_tick = 0U;
        planner->yaw_recovery_tick = 0U;
        planner->launch_wait_ticks = 0U;
        planner->waiting_for_flight_end = false;
        planner->launch_velocity_ready = false;
        planner->launch_velocity_x_dps = 0.0f;
        planner->launch_velocity_y_dps = 0.0f;
        planner->launch_velocity_z_dps = 0.0f;
        planner->timed_turn_profile.input.now_tick = 0U;
        planner->timed_turn_profile.output.pulse_us =
            planner->timed_turn_profile.params.base_pulse_us;
        planner->timed_turn_profile.output.stage = TIMED_TURN_PULSE_ACTION_STAGE_WAIT;
        planner->timed_turn_profile.output.active_segment_index = 0U;
        planner->timed_turn_profile.output.active_pwm_us = 0.0f;
        planner->timed_turn_profile.output.pwm_active = false;
        ImuVelocityRecoveryAction_Reset(&planner->yaw_recovery_action);

        planner->output.mode = GUIDANCE_PLANNER_MODE_FLIGHT_END_RESET;
        planner->output.task_result = TASK_ACTION_SUCCESS;
        planner->output.override_pulse_us = &planner->timed_turn_profile.output.pulse_us;
        planner->output.default_pid_task_enabled = false;
        planner->output.override_pwm_active = true;
        planner->output.flight_end_detected = true;
        planner->output.launch_velocity_ready = false;
        planner->output.launch_velocity_x_dps = 0.0f;
        planner->output.launch_velocity_y_dps = 0.0f;
        planner->output.launch_velocity_z_dps = 0.0f;
        return TASK_ACTION_SUCCESS;
    }

    if (launch_event) {
        planner->phase = GUIDANCE_PLANNER_PHASE_LAUNCH_WAIT;
        planner->timed_turn_tick = 0U;
        planner->yaw_recovery_tick = 0U;
        planner->launch_wait_ticks = 0U;
        planner->waiting_for_flight_end = true;
        planner->launch_velocity_ready = false;
        planner->launch_velocity_x_dps = 0.0f;
        planner->launch_velocity_y_dps = 0.0f;
        planner->launch_velocity_z_dps = 0.0f;
        planner->timed_turn_profile.input.now_tick = 0U;
        planner->timed_turn_profile.output.pulse_us =
            planner->timed_turn_profile.params.base_pulse_us;
        planner->timed_turn_profile.output.stage = TIMED_TURN_PULSE_ACTION_STAGE_WAIT;
        planner->timed_turn_profile.output.active_segment_index = 0U;
        planner->timed_turn_profile.output.active_pwm_us = 0.0f;
        planner->timed_turn_profile.output.pwm_active = false;
        ImuVelocityRecoveryAction_Reset(&planner->yaw_recovery_action);
        planner->output.timed_turn_started = true;
    }

    if (planner->phase == GUIDANCE_PLANNER_PHASE_LAUNCH_WAIT) {
        planner->output.mode = GUIDANCE_PLANNER_MODE_LAUNCH_WAIT;
        planner->output.task_result = TASK_ACTION_RUNNING;
        planner->output.override_pulse_us = &planner->timed_turn_profile.output.pulse_us;
        planner->output.default_pid_task_enabled = false;
        planner->output.override_pwm_active = true;

        if (planner->launch_wait_ticks < UINT16_MAX) {
            planner->launch_wait_ticks++;
        }

        if ((planner->launch_wait_ticks >= GUIDANCE_PLANNER_LAUNCH_SAMPLE_WAIT_TICKS) &&
            imu_event_valid) {
            planner->launch_velocity_x_dps = input->gyro_x_dps;
            planner->launch_velocity_y_dps = input->gyro_y_dps;
            planner->launch_velocity_z_dps = input->gyro_z_dps;
            planner->launch_velocity_ready = true;
            planner->output.launch_velocity_ready = true;
            planner->output.launch_velocity_x_dps = planner->launch_velocity_x_dps;
            planner->output.launch_velocity_y_dps = planner->launch_velocity_y_dps;
            planner->output.launch_velocity_z_dps = planner->launch_velocity_z_dps;
            planner->phase = GUIDANCE_PLANNER_PHASE_TIMED_VERTICAL_TURN;
            planner->launch_wait_ticks = 0U;
        }

        return TASK_ACTION_RUNNING;
    }

    if (planner->phase == GUIDANCE_PLANNER_PHASE_TIMED_VERTICAL_TURN) {
        planner->timed_turn_profile.input.enabled = true;
        planner->timed_turn_profile.input.now_tick = planner->timed_turn_tick;
        timed_turn_result = TimedTurnPulseAction_EvaluateLaunchVerticalTurn(
            &planner->timed_turn_profile.input,
            &planner->timed_turn_profile.params,
            &planner->timed_turn_profile.output);

        planner->output.mode = GUIDANCE_PLANNER_MODE_TIMED_VERTICAL_TURN;
        planner->output.task_result = timed_turn_result;
        planner->output.override_pulse_us = &planner->timed_turn_profile.output.pulse_us;
        planner->output.timed_turn_output = &planner->timed_turn_profile.output;
        planner->output.default_pid_task_enabled = false;
        planner->output.override_pwm_active = true;
        planner->output.timed_turn_active = (timed_turn_result == TASK_ACTION_RUNNING);
        planner->output.timed_turn_finished = (timed_turn_result != TASK_ACTION_RUNNING);

        if (timed_turn_result == TASK_ACTION_RUNNING) {
            if (planner->timed_turn_tick < UINT32_MAX) {
                planner->timed_turn_tick += 1U;
            }
        } else {
            planner->phase = GUIDANCE_PLANNER_PHASE_PREPARE_YAW_RECOVERY;
            planner->timed_turn_tick = 0U;
        }

        return timed_turn_result;
    }

    if (planner->phase == GUIDANCE_PLANNER_PHASE_PREPARE_YAW_RECOVERY) {
        planner->output.mode = GUIDANCE_PLANNER_MODE_PREPARE_YAW_RECOVERY;
        planner->output.task_result = TASK_ACTION_RUNNING;
        planner->output.override_pulse_us = &planner->timed_turn_profile.params.base_pulse_us;
        planner->output.default_pid_task_enabled = false;
        planner->output.override_pwm_active = true;

        if ((input != NULL) && input->yaw_recovery_target_available &&
            imu_event_valid && planner->launch_velocity_ready) {
            planner->yaw_recovery_profile.input.target_delta = input->yaw_recovery_delta;
            planner->yaw_recovery_profile.input.target_detected = true;
            planner->yaw_recovery_profile.input.now_tick = 0U;
            planner->yaw_recovery_profile.input.imu_yaw_velocity_dps = input->gyro_z_dps;
            planner->yaw_recovery_profile.input.launch_yaw_velocity_dps =
                planner->launch_velocity_z_dps;
            planner->yaw_recovery_profile.input.launch_velocity_ready = true;
            ImuVelocityRecoveryAction_Reset(&planner->yaw_recovery_action);
            planner->phase = GUIDANCE_PLANNER_PHASE_YAW_VELOCITY_RECOVERY;
            planner->yaw_recovery_tick = 0U;
            planner->output.yaw_recovery_started = true;
        } else {
            return TASK_ACTION_RUNNING;
        }
    }

    if (planner->phase == GUIDANCE_PLANNER_PHASE_YAW_VELOCITY_RECOVERY) {
        if ((input == NULL) || !input->imu_data_valid) {
            planner->output.mode = GUIDANCE_PLANNER_MODE_YAW_VELOCITY_RECOVERY;
            planner->output.task_result = TASK_ACTION_RUNNING;
            planner->output.override_pulse_us = &planner->timed_turn_profile.params.base_pulse_us;
            planner->output.default_pid_task_enabled = false;
            planner->output.override_pwm_active = true;
            return TASK_ACTION_RUNNING;
        }

        planner->yaw_recovery_profile.input.target_delta = input->yaw_recovery_delta;
        planner->yaw_recovery_profile.input.target_detected =
            input->yaw_recovery_target_available;
        planner->yaw_recovery_profile.input.now_tick = planner->yaw_recovery_tick;
        planner->yaw_recovery_profile.input.imu_yaw_velocity_dps = input->gyro_z_dps;
        planner->yaw_recovery_profile.input.launch_yaw_velocity_dps =
            planner->launch_velocity_z_dps;
        planner->yaw_recovery_profile.input.launch_velocity_ready = planner->launch_velocity_ready;
        yaw_recovery_result = ImuVelocityRecoveryAction_Tick(&planner->yaw_recovery_action);

        planner->output.mode = GUIDANCE_PLANNER_MODE_YAW_VELOCITY_RECOVERY;
        planner->output.task_result = yaw_recovery_result;
        planner->output.override_contribution =
            &planner->yaw_recovery_profile.output.contribution;
        planner->output.yaw_recovery_output = &planner->yaw_recovery_profile.output;
        planner->output.default_pid_task_enabled = false;
        planner->output.override_contribution_active =
            planner->yaw_recovery_profile.output.contribution.horizontal_pwm_active ||
            planner->yaw_recovery_profile.output.contribution.vertical_pwm_active;
        planner->output.yaw_recovery_active = (yaw_recovery_result == TASK_ACTION_RUNNING);
        planner->output.yaw_recovery_finished = (yaw_recovery_result != TASK_ACTION_RUNNING);

        if (yaw_recovery_result == TASK_ACTION_RUNNING) {
            if (planner->yaw_recovery_tick < UINT32_MAX) {
                planner->yaw_recovery_tick += 1U;
            }
        } else {
            planner->phase = GUIDANCE_PLANNER_PHASE_IDLE;
            planner->yaw_recovery_tick = 0U;
            ImuVelocityRecoveryAction_Reset(&planner->yaw_recovery_action);
        }

        return yaw_recovery_result;
    }

    return TASK_ACTION_RUNNING;
}
