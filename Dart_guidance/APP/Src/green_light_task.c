#include "green_light_task.hpp"

#include <stddef.h>

static uint16_t GreenLightTask_ResolveSetpointAxis(uint16_t setpoint, uint16_t measurement_size)
{
    if (setpoint == GUIDANCE_NO_TARGET_COORDINATE) {
        if (measurement_size == 0U) {
            return 0U;
        }

        return (uint16_t)(measurement_size / 2U);
    }

    return setpoint;
}

static TaskActionResult_t AcquireMeasurementAction_OnEnter(TaskAction_t *action)
{
    GreenLightTaskProfile_t *profile = (GreenLightTaskProfile_t *)action->profile;
    GreenLightTask_t *task = (GreenLightTask_t *)action->context;

    if ((profile == NULL) || (task == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    if (profile->output.target_detected) {
        task->last_valid_measurement = profile->output.measurement;
        task->has_last_valid_measurement = true;
    }

    GreenLightTaskProfile_ResetOutput(&profile->output);
    if (!profile->input.upstream_measurement_ready) {
        return TASK_ACTION_RUNNING;
    }

    profile->output.measurement = profile->input.upstream_measurement;
    profile->output.target_detected = true;
    if ((profile->output.measurement.x == GUIDANCE_NO_TARGET_COORDINATE) ||
        (profile->output.measurement.y == GUIDANCE_NO_TARGET_COORDINATE)) {
        profile->output.target_detected = false;
        GuidanceTargetSmoother_Reset(&task->measurement_smoother);
        return TASK_ACTION_FAILURE;
    }

    task->measurement_smoother.config = profile->params.measurement_smoother_config;
    if (!GuidanceTargetSmoother_Update(&task->measurement_smoother,
                                       &profile->input.upstream_measurement,
                                       profile->input.measurement_width,
                                       profile->input.measurement_height,
                                       &profile->output.measurement)) {
        profile->output.target_detected = false;
        return TASK_ACTION_FAILURE;
    }

    task->last_valid_measurement = profile->output.measurement;
    task->has_last_valid_measurement = true;
    return TASK_ACTION_SUCCESS;
}

static TaskActionResult_t AcquireMeasurementAction_OnRunning(TaskAction_t *action)
{
    GreenLightTaskProfile_t *profile = (GreenLightTaskProfile_t *)action->profile;
    GreenLightTask_t *task = (GreenLightTask_t *)action->context;

    if ((profile == NULL) || (task == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    if (!profile->input.upstream_measurement_ready) {
        return TASK_ACTION_RUNNING;
    }

    profile->output.measurement = profile->input.upstream_measurement;
    profile->output.target_detected = true;
    if ((profile->output.measurement.x == GUIDANCE_NO_TARGET_COORDINATE) ||
        (profile->output.measurement.y == GUIDANCE_NO_TARGET_COORDINATE)) {
        profile->output.target_detected = false;
        GuidanceTargetSmoother_Reset(&task->measurement_smoother);
        return TASK_ACTION_FAILURE;
    }

    task->measurement_smoother.config = profile->params.measurement_smoother_config;
    if (!GuidanceTargetSmoother_Update(&task->measurement_smoother,
                                       &profile->input.upstream_measurement,
                                       profile->input.measurement_width,
                                       profile->input.measurement_height,
                                       &profile->output.measurement)) {
        profile->output.target_detected = false;
        return TASK_ACTION_FAILURE;
    }

    task->last_valid_measurement = profile->output.measurement;
    task->has_last_valid_measurement = true;
    return TASK_ACTION_SUCCESS;
}

static void AcquireMeasurementAction_OnExit(TaskAction_t *action, TaskActionResult_t result)
{
    GreenLightTaskProfile_t *profile = (GreenLightTaskProfile_t *)action->profile;

    if (profile == NULL) {
        return;
    }

    if (result == TASK_ACTION_FAILURE) {
        profile->output.target_detected = false;
    }
}

static TaskActionResult_t ComputeDeltaAction_OnEnter(TaskAction_t *action)
{
    GreenLightTaskProfile_t *profile = (GreenLightTaskProfile_t *)action->profile;
    uint16_t setpoint_x;
    uint16_t setpoint_y;

    if (profile == NULL) {
        return TASK_ACTION_FAILURE;
    }

    if (!profile->output.target_detected) {
        return TASK_ACTION_FAILURE;
    }

    setpoint_x = GreenLightTask_ResolveSetpointAxis(profile->input.setpoint.x,
                                                    profile->input.measurement_width);
    setpoint_y = GreenLightTask_ResolveSetpointAxis(profile->input.setpoint.y,
                                                    profile->input.measurement_height);

    profile->output.delta.delta_x = (int16_t)((int32_t)profile->output.measurement.x - (int32_t)setpoint_x);
    profile->output.delta.delta_y = (int16_t)((int32_t)setpoint_y - (int32_t)profile->output.measurement.y);
    return TASK_ACTION_SUCCESS;
}

void GreenLightTaskProfile_ResetOutput(GreenLightTaskOutput_t *output)
{
    if (output == NULL) {
        return;
    }

    output->measurement.x = GUIDANCE_NO_TARGET_COORDINATE;
    output->measurement.y = GUIDANCE_NO_TARGET_COORDINATE;
    output->measurement.area = 0U;
    output->delta.delta_x = 0;
    output->delta.delta_y = 0;
    output->target_detected = false;
}

void GreenLightTask_Init(GreenLightTask_t *task, GreenLightTaskProfile_t *profile)
{
    if ((task == NULL) || (profile == NULL)) {
        return;
    }

    task->profile = profile;
    GuidanceTargetSmoother_Init(&task->measurement_smoother,
                                &profile->params.measurement_smoother_config);
    task->last_valid_measurement.x = GUIDANCE_NO_TARGET_COORDINATE;
    task->last_valid_measurement.y = GUIDANCE_NO_TARGET_COORDINATE;
    task->last_valid_measurement.area = 0U;
    task->has_last_valid_measurement = false;

    TaskAction_Init(&task->acquire_measurement_action,
                    profile,
                    task,
                    AcquireMeasurementAction_OnEnter,
                    AcquireMeasurementAction_OnRunning,
                    AcquireMeasurementAction_OnExit);
    TaskAction_Init(&task->compute_delta_action,
                    profile,
                    task,
                    ComputeDeltaAction_OnEnter,
                    NULL,
                    NULL);

    task->sequence_slots[0].action = &task->acquire_measurement_action;
    task->sequence_slots[1].action = &task->compute_delta_action;
    TaskSequence_Init(&task->sequence_task, task->sequence_slots, 2U);

    GreenLightTaskProfile_ResetOutput(&profile->output);
}

void GreenLightTask_Reset(GreenLightTask_t *task)
{
    if ((task == NULL) || (task->profile == NULL)) {
        return;
    }

    TaskSequence_Reset(&task->sequence_task);
    GuidanceTargetSmoother_Reset(&task->measurement_smoother);
    GreenLightTaskProfile_ResetOutput(&task->profile->output);
}

TaskActionResult_t GreenLightTask_Tick(GreenLightTask_t *task)
{
    if ((task == NULL) || (task->profile == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    return TaskSequence_Tick(&task->sequence_task);
}

bool GreenLightTask_InsertInterrupt(GreenLightTask_t *task, TaskAction_t *interrupt_action)
{
    if (task == NULL) {
        return false;
    }

    return TaskSequence_PushInterrupt(&task->sequence_task, interrupt_action);
}