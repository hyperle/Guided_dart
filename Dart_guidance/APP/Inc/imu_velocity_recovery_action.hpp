#ifndef DART_GUIDANCE_IMU_VELOCITY_RECOVERY_ACTION_HPP
#define DART_GUIDANCE_IMU_VELOCITY_RECOVERY_ACTION_HPP

#include "guidance_controller.h"
#include "guidance_types.h"
#include "pid.h"
#include "task_action.hpp"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_VELOCITY_RECOVERY_ACTION_ENABLE_IN_MAIN
#define IMU_VELOCITY_RECOVERY_ACTION_ENABLE_IN_MAIN 0
#endif

#define IMU_VELOCITY_RECOVERY_DELTA_TO_TICK_SCALE 0.10f
#define IMU_VELOCITY_RECOVERY_MIN_TICK_PERIOD 2U
#define IMU_VELOCITY_RECOVERY_MAX_TICK_PERIOD 60U
#define IMU_VELOCITY_RECOVERY_SWEEP_START_PWM_US 120.0f
#define IMU_VELOCITY_RECOVERY_SWEEP_END_PWM_US -120.0f
#define IMU_VELOCITY_RECOVERY_PID_KP 2.0f
#define IMU_VELOCITY_RECOVERY_PID_KI 0.0f
#define IMU_VELOCITY_RECOVERY_PID_KD 0.0f
#define IMU_VELOCITY_RECOVERY_PID_INTEGRAL_LIMIT 100.0f
#define IMU_VELOCITY_RECOVERY_PID_OUTPUT_LIMIT_US 300.0f
#define IMU_VELOCITY_RECOVERY_SUCCESS_BAND_DPS 5.0f
#define IMU_VELOCITY_RECOVERY_SUCCESS_HOLD_TICKS 3U
#define IMU_VELOCITY_RECOVERY_VELOCITY_SAMPLE_WAIT_TICKS 20U

typedef enum
{
    IMU_VELOCITY_RECOVERY_STAGE_IDLE = 0,
    IMU_VELOCITY_RECOVERY_STAGE_SWEEP = 1,
    IMU_VELOCITY_RECOVERY_STAGE_WAIT_VELOCITY_SAMPLE = 2,
    IMU_VELOCITY_RECOVERY_STAGE_PID_RECOVERY = 3,
    IMU_VELOCITY_RECOVERY_STAGE_DONE = 4
} ImuVelocityRecoveryActionStage_t;

typedef struct
{
    GuidanceDelta_t target_delta;
    bool target_detected;
    uint32_t now_tick;
    float imu_yaw_velocity_dps;
    float launch_yaw_velocity_dps;
    bool launch_velocity_ready;
} ImuVelocityRecoveryActionInput_t;

typedef struct
{
    GuidanceServoPulseUs_t base_pulse_us;
    float delta_to_tick_scale;
    uint16_t min_tick_period;
    uint16_t max_tick_period;
    float sweep_start_pwm_us;
    float sweep_end_pwm_us;
    GuidanceHorizontalPwmPidConfig_t velocity_pid_config;
    float velocity_pid_output_limit_us;
    float success_band_dps;
    uint16_t success_hold_ticks;
    uint16_t velocity_sample_wait_ticks;
} ImuVelocityRecoveryActionParams_t;

typedef struct
{
    GuidanceControlContribution_t contribution;
    GuidanceServoPulseUs_t preview_pulse_us;
    ImuVelocityRecoveryActionStage_t stage;
    int8_t delta_sign;
    uint16_t tick_period;
    float sweep_start_pwm_us;
    float sweep_end_pwm_us;
    float active_pwm_us;
    float target_yaw_velocity_dps;
    float velocity_error_yaw_dps;
    bool target_velocity_latched;
} ImuVelocityRecoveryActionOutput_t;

typedef struct
{
    ImuVelocityRecoveryActionInput_t input;
    ImuVelocityRecoveryActionParams_t params;
    ImuVelocityRecoveryActionOutput_t output;
} ImuVelocityRecoveryActionProfile_t;

typedef struct
{
    uint32_t start_tick;
    uint16_t velocity_sample_wait_count;
    uint16_t success_count;
    int8_t latched_delta_sign;
    uint16_t latched_tick_period;
    float latched_sweep_start_pwm_us;
    float latched_sweep_end_pwm_us;
    float latched_sweep_step_pwm_us;
    float latched_target_yaw_velocity_dps;
    bool running;
    bool target_velocity_latched;
} ImuVelocityRecoveryActionState_t;

typedef struct
{
    ImuVelocityRecoveryActionProfile_t *profile;
    TaskAction_t action;
    PID_t velocity_pid;
    ImuVelocityRecoveryActionState_t state;
} ImuVelocityRecoveryAction_t;

void ImuVelocityRecoveryActionProfile_LoadDefault(ImuVelocityRecoveryActionProfile_t *profile);
void ImuVelocityRecoveryAction_Init(ImuVelocityRecoveryAction_t *recovery_action,
                                    ImuVelocityRecoveryActionProfile_t *profile);
void ImuVelocityRecoveryAction_Reset(ImuVelocityRecoveryAction_t *recovery_action);
TaskActionResult_t ImuVelocityRecoveryAction_Tick(ImuVelocityRecoveryAction_t *recovery_action);
TaskActionResult_t ImuVelocityRecoveryAction_OnEnter(TaskAction_t *action);
TaskActionResult_t ImuVelocityRecoveryAction_OnRunning(TaskAction_t *action);
void ImuVelocityRecoveryAction_OnExit(TaskAction_t *action, TaskActionResult_t result);

#ifdef __cplusplus
}
#endif

#endif
