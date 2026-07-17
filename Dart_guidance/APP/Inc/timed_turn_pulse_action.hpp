#ifndef DART_GUIDANCE_TIMED_TURN_PULSE_ACTION_HPP
#define DART_GUIDANCE_TIMED_TURN_PULSE_ACTION_HPP

#include "guidance_controller.h"
#include "guidance_types.h"
#include "task_action.hpp"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TIMED_TURN_PULSE_ACTION_ENABLE_IN_MAIN
#define TIMED_TURN_PULSE_ACTION_ENABLE_IN_MAIN 0
#endif

#define TIMED_TURN_PULSE_ACTION_MAX_SEGMENTS 3U
#define TIMED_TURN_PULSE_ACTION_DEFAULT_START_TICK 10U
#define TIMED_TURN_PULSE_ACTION_STAGE0_PERIOD_TICKS 50U
#define TIMED_TURN_PULSE_ACTION_STAGE1_PERIOD_TICKS 50U
#define TIMED_TURN_PULSE_ACTION_STAGE2_PERIOD_TICKS 50U
#define TIMED_TURN_PULSE_ACTION_DEFAULT_PWM_US 200.0f

typedef enum
{
    TIMED_TURN_PULSE_ACTION_STAGE_WAIT = 0,
    TIMED_TURN_PULSE_ACTION_STAGE_ACTIVE = 1,
    TIMED_TURN_PULSE_ACTION_STAGE_DONE = 2
} TimedTurnPulseActionStage_t;

typedef struct
{
    uint16_t start_tick;
    uint16_t duration_ticks;
    float pwm_us;
} TimedTurnPulseActionSegment_t;

typedef struct
{
    uint32_t now_tick;
    bool enabled;
} TimedTurnPulseActionInput_t;

typedef struct
{
    GuidanceServoPulseUs_t base_pulse_us;
    int8_t servo_direction[GUIDANCE_SERVO_COUNT];
    TimedTurnPulseActionSegment_t segments[TIMED_TURN_PULSE_ACTION_MAX_SEGMENTS];
    uint8_t segment_count;
} TimedTurnPulseActionParams_t;

typedef struct
{
    GuidanceServoPulseUs_t pulse_us;
    TimedTurnPulseActionStage_t stage;
    uint8_t active_segment_index;
    float active_pwm_us;
    bool pwm_active;
} TimedTurnPulseActionOutput_t;

typedef struct
{
    TimedTurnPulseActionInput_t input;
    TimedTurnPulseActionParams_t params;
    TimedTurnPulseActionOutput_t output;
} TimedTurnPulseActionProfile_t;

typedef struct
{
    TimedTurnPulseActionProfile_t *profile;
    TaskAction_t action;
} TimedTurnPulseAction_t;

void TimedTurnPulseActionProfile_LoadDefault(TimedTurnPulseActionProfile_t *profile);
TaskActionResult_t TimedTurnPulseAction_Evaluate(const TimedTurnPulseActionInput_t *input,
                                                 const TimedTurnPulseActionParams_t *params,
                                                 TimedTurnPulseActionOutput_t *output);
void TimedTurnPulseAction_Init(TimedTurnPulseAction_t *pulse_action,
                               TimedTurnPulseActionProfile_t *profile);
void TimedTurnPulseAction_Reset(TimedTurnPulseAction_t *pulse_action);
TaskActionResult_t TimedTurnPulseAction_Tick(TimedTurnPulseAction_t *pulse_action);
TaskActionResult_t TimedTurnPulseAction_OnEnter(TaskAction_t *action);
TaskActionResult_t TimedTurnPulseAction_OnRunning(TaskAction_t *action);
void TimedTurnPulseAction_OnExit(TaskAction_t *action, TaskActionResult_t result);

#ifdef __cplusplus
}
#endif

#endif
