#include "timed_turn_pulse_action.hpp"

#include "servo.h"
#include <stddef.h>

void TimedTurnPulseActionProfile_LoadDefault(TimedTurnPulseActionProfile_t *profile)
{
    if (profile == NULL) {
        return;
    }

    profile->input.now_tick = 0U;
    profile->input.enabled = true;

    profile->params.base_pulse_us.values[0] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_0;
    profile->params.base_pulse_us.values[1] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_1;
    profile->params.base_pulse_us.values[2] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_2;
    profile->params.base_pulse_us.values[3] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_3;
    profile->params.servo_direction[0] = -1;
    profile->params.servo_direction[1] = 1;
    profile->params.servo_direction[2] = 1;
    profile->params.servo_direction[3] = 1;
    profile->params.segment_count = TIMED_TURN_PULSE_ACTION_MAX_SEGMENTS;
    profile->params.segments[0].start_tick = TIMED_TURN_PULSE_ACTION_DEFAULT_START_TICK;
    profile->params.segments[0].duration_ticks =
        TIMED_TURN_PULSE_ACTION_STAGE0_PERIOD_TICKS + 1U;
    profile->params.segments[0].pwm_us = TIMED_TURN_PULSE_ACTION_STAGE0_START_PWM_US;
    profile->params.segments[0].pwm_step_us = TIMED_TURN_PULSE_ACTION_STAGE0_STEP_PWM_US;
    profile->params.segments[1].start_tick =
        TIMED_TURN_PULSE_ACTION_DEFAULT_START_TICK +
        TIMED_TURN_PULSE_ACTION_STAGE0_PERIOD_TICKS + 1U;
    profile->params.segments[1].duration_ticks = TIMED_TURN_PULSE_ACTION_STAGE1_PERIOD_TICKS;
    profile->params.segments[1].pwm_us = TIMED_TURN_PULSE_ACTION_STAGE1_PWM_US;
    profile->params.segments[1].pwm_step_us = TIMED_TURN_PULSE_ACTION_STAGE1_STEP_PWM_US;
    profile->params.segments[2].start_tick =
        TIMED_TURN_PULSE_ACTION_DEFAULT_START_TICK +
        TIMED_TURN_PULSE_ACTION_STAGE0_PERIOD_TICKS + 1U +
        TIMED_TURN_PULSE_ACTION_STAGE1_PERIOD_TICKS;
    profile->params.segments[2].duration_ticks =
        TIMED_TURN_PULSE_ACTION_STAGE2_PERIOD_TICKS + 1U;
    profile->params.segments[2].pwm_us = TIMED_TURN_PULSE_ACTION_STAGE2_START_PWM_US;
    profile->params.segments[2].pwm_step_us = TIMED_TURN_PULSE_ACTION_STAGE2_STEP_PWM_US;

    profile->output.pulse_us = profile->params.base_pulse_us;
    profile->output.stage = TIMED_TURN_PULSE_ACTION_STAGE_WAIT;
    profile->output.active_segment_index = 0U;
    profile->output.active_pwm_us = 0.0f;
    profile->output.pwm_active = false;
}

TaskActionResult_t TimedTurnPulseAction_EvaluateLaunchVerticalTurn(
    const TimedTurnPulseActionInput_t *input,
    const TimedTurnPulseActionParams_t *params,
    TimedTurnPulseActionOutput_t *output)
{
    uint32_t last_end_tick = 0U;
    uint8_t active_segment_count;
    uint8_t segment_index;
    uint8_t servo_index;
    bool has_segment = false;

    if ((input == NULL) || (params == NULL) || (output == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    output->pulse_us = params->base_pulse_us;
    output->stage = TIMED_TURN_PULSE_ACTION_STAGE_WAIT;
    output->active_segment_index = 0U;
    output->active_pwm_us = 0.0f;
    output->pwm_active = false;

    if (!input->enabled) {
        return TASK_ACTION_FAILURE;
    }

    active_segment_count = params->segment_count;
    if (active_segment_count > TIMED_TURN_PULSE_ACTION_MAX_SEGMENTS) {
        active_segment_count = TIMED_TURN_PULSE_ACTION_MAX_SEGMENTS;
    }
    if (active_segment_count == 0U) {
        return TASK_ACTION_FAILURE;
    }

    for (segment_index = 0U; segment_index < active_segment_count; ++segment_index) {
        const TimedTurnPulseActionSegment_t *segment = &params->segments[segment_index];
        uint32_t segment_start_tick = segment->start_tick;
        uint32_t segment_end_tick = segment_start_tick + segment->duration_ticks;
        uint32_t elapsed_tick;

        if (segment->duration_ticks == 0U) {
            continue;
        }
        has_segment = true;
        if (segment_end_tick > last_end_tick) {
            last_end_tick = segment_end_tick;
        }
        if ((input->now_tick >= segment_start_tick) && (input->now_tick < segment_end_tick)) {
            elapsed_tick = input->now_tick - segment_start_tick;
            output->active_segment_index = segment_index;
            output->active_pwm_us =
                segment->pwm_us + ((float)elapsed_tick * segment->pwm_step_us);
            output->pwm_active = (segment_index != 1U);
            output->stage = output->pwm_active ? TIMED_TURN_PULSE_ACTION_STAGE_ACTIVE :
                                                 TIMED_TURN_PULSE_ACTION_STAGE_WAIT;

            for (servo_index = 0U; servo_index < GUIDANCE_SERVO_COUNT; ++servo_index) {
                output->pulse_us.values[servo_index] +=
                    (float)params->servo_direction[servo_index] * output->active_pwm_us;
            }

            for (servo_index = 0U; servo_index < GUIDANCE_SERVO_COUNT; ++servo_index) {
                if (output->pulse_us.values[servo_index] < (float)SERVO_PULSE_MIN_US) {
                    output->pulse_us.values[servo_index] = (float)SERVO_PULSE_MIN_US;
                } else if (output->pulse_us.values[servo_index] > (float)SERVO_PULSE_MAX_US) {
                    output->pulse_us.values[servo_index] = (float)SERVO_PULSE_MAX_US;
                }
            }
            return TASK_ACTION_RUNNING;
        }
    }

    if (has_segment && (input->now_tick >= last_end_tick)) {
        output->stage = TIMED_TURN_PULSE_ACTION_STAGE_DONE;
        return TASK_ACTION_SUCCESS;
    }

    return TASK_ACTION_RUNNING;
}

TaskActionResult_t TimedTurnPulseAction_Evaluate(const TimedTurnPulseActionInput_t *input,
                                                 const TimedTurnPulseActionParams_t *params,
                                                 TimedTurnPulseActionOutput_t *output)
{
    return TimedTurnPulseAction_EvaluateLaunchVerticalTurn(input, params, output);
}

void TimedTurnPulseAction_Init(TimedTurnPulseAction_t *pulse_action,
                               TimedTurnPulseActionProfile_t *profile)
{
    if ((pulse_action == NULL) || (profile == NULL)) {
        return;
    }

    pulse_action->profile = profile;
    TaskAction_Init(&pulse_action->action,
                    profile,
                    pulse_action,
                    TimedTurnPulseAction_OnEnter,
                    TimedTurnPulseAction_OnRunning,
                    TimedTurnPulseAction_OnExit);
}

void TimedTurnPulseAction_Reset(TimedTurnPulseAction_t *pulse_action)
{
    if ((pulse_action == NULL) || (pulse_action->profile == NULL)) {
        return;
    }

    pulse_action->profile->output.pulse_us = pulse_action->profile->params.base_pulse_us;
    pulse_action->profile->output.stage = TIMED_TURN_PULSE_ACTION_STAGE_WAIT;
    pulse_action->profile->output.active_segment_index = 0U;
    pulse_action->profile->output.active_pwm_us = 0.0f;
    pulse_action->profile->output.pwm_active = false;
    TaskAction_Reset(&pulse_action->action);
}

TaskActionResult_t TimedTurnPulseAction_Tick(TimedTurnPulseAction_t *pulse_action)
{
    if ((pulse_action == NULL) || (pulse_action->profile == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    return TaskAction_Tick(&pulse_action->action);
}

TaskActionResult_t TimedTurnPulseAction_OnEnter(TaskAction_t *action)
{
    return TimedTurnPulseAction_OnRunning(action);
}

TaskActionResult_t TimedTurnPulseAction_OnRunning(TaskAction_t *action)
{
    TimedTurnPulseActionProfile_t *profile;

    if (action == NULL) {
        return TASK_ACTION_FAILURE;
    }

    profile = (TimedTurnPulseActionProfile_t *)action->profile;
    if (profile == NULL) {
        return TASK_ACTION_FAILURE;
    }

    return TimedTurnPulseAction_Evaluate(&profile->input,
                                         &profile->params,
                                         &profile->output);
}

void TimedTurnPulseAction_OnExit(TaskAction_t *action, TaskActionResult_t result)
{
    TimedTurnPulseActionProfile_t *profile;

    (void)result;

    if (action == NULL) {
        return;
    }

    profile = (TimedTurnPulseActionProfile_t *)action->profile;
    if (profile == NULL) {
        return;
    }

    profile->output.pulse_us = profile->params.base_pulse_us;
    profile->output.pwm_active = false;
}
