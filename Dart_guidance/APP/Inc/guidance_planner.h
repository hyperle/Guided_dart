#ifndef DART_GUIDANCE_PLANNER_H
#define DART_GUIDANCE_PLANNER_H

#include "guidance_types.h"
#include "imu.h"
#include "imu_velocity_recovery_action.hpp"
#include "timed_turn_pulse_action.hpp"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GUIDANCE_PLANNER_LAUNCH_SAMPLE_WAIT_TICKS
#define GUIDANCE_PLANNER_LAUNCH_SAMPLE_WAIT_TICKS 5U
#endif

#ifndef GUIDANCE_PLANNER_FLIGHT_END_ACCEL_THRESHOLD_RATIO
#define GUIDANCE_PLANNER_FLIGHT_END_ACCEL_THRESHOLD_RATIO 0.5f
#endif

typedef enum
{
    GUIDANCE_PLANNER_MODE_DEFAULT_PID = 0,
    GUIDANCE_PLANNER_MODE_LAUNCH_WAIT = 1,
    GUIDANCE_PLANNER_MODE_TIMED_VERTICAL_TURN = 2,
    GUIDANCE_PLANNER_MODE_PREPARE_YAW_RECOVERY = 3,
    GUIDANCE_PLANNER_MODE_YAW_VELOCITY_RECOVERY = 4,
    GUIDANCE_PLANNER_MODE_FLIGHT_END_RESET = 5
} GuidancePlannerMode_t;

typedef enum
{
    GUIDANCE_PLANNER_PHASE_IDLE = 0,
    GUIDANCE_PLANNER_PHASE_LAUNCH_WAIT = 1,
    GUIDANCE_PLANNER_PHASE_TIMED_VERTICAL_TURN = 2,
    GUIDANCE_PLANNER_PHASE_PREPARE_YAW_RECOVERY = 3,
    GUIDANCE_PLANNER_PHASE_YAW_VELOCITY_RECOVERY = 4
} GuidancePlannerPhase_t;

typedef struct
{
    bool imu_data_valid;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    uint8_t accel_axis; /* ImuAccelAxis_t */
    float accel_threshold_mps2;
    GuidanceDelta_t target_delta;
    bool target_detected;
    GuidanceDelta_t yaw_recovery_delta;
    bool yaw_recovery_target_available;
} GuidancePlannerInput_t;

typedef struct
{
    GuidancePlannerMode_t mode;
    TaskActionResult_t task_result;
    const GuidanceServoPulseUs_t *override_pulse_us;
    const GuidanceControlContribution_t *override_contribution;
    const TimedTurnPulseActionOutput_t *timed_turn_output;
    const ImuVelocityRecoveryActionOutput_t *yaw_recovery_output;
    bool default_pid_task_enabled;
    bool override_pwm_active;
    bool override_contribution_active;
    bool timed_turn_started;
    bool timed_turn_active;
    bool timed_turn_finished;
    bool yaw_recovery_started;
    bool yaw_recovery_active;
    bool yaw_recovery_finished;
    bool flight_end_detected;
    bool launch_velocity_ready;
    float launch_velocity_x_dps;
    float launch_velocity_y_dps;
    float launch_velocity_z_dps;
} GuidancePlannerOutput_t;

typedef struct
{
    TimedTurnPulseActionProfile_t timed_turn_profile;
    ImuVelocityRecoveryActionProfile_t yaw_recovery_profile;
    ImuVelocityRecoveryAction_t yaw_recovery_action;
    GuidancePlannerOutput_t output;
    GuidancePlannerPhase_t phase;
    uint32_t timed_turn_tick;
    uint32_t yaw_recovery_tick;
    uint16_t launch_wait_ticks;
    bool waiting_for_flight_end;
    bool launch_threshold_active;
    bool half_threshold_active;
    bool launch_velocity_ready;
    float launch_velocity_x_dps;
    float launch_velocity_y_dps;
    float launch_velocity_z_dps;
} GuidancePlanner_t;

void GuidancePlanner_Init(GuidancePlanner_t *planner);
void GuidancePlanner_Reset(GuidancePlanner_t *planner);
TaskActionResult_t GuidancePlanner_Tick(GuidancePlanner_t *planner,
                                        const GuidancePlannerInput_t *input);

#ifdef __cplusplus
}
#endif

#endif
