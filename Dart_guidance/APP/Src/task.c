#include "task.hpp"

#include <stddef.h>

void TaskHook_Reset(TaskHook_t *hook)
{
    if (hook == NULL) {
        return;
    }

    hook->pending = false;
    hook->id = TASK_HOOK_ID_NONE;
    hook->source = NULL;
}

bool TaskHook_Emit(TaskHook_t *hook, TaskHookId_t hook_id, void *source)
{
    if ((hook == NULL) || (hook_id == TASK_HOOK_ID_NONE) || hook->pending) {
        return false;
    }

    hook->pending = true;
    hook->id = hook_id;
    hook->source = source;
    return true;
}

bool TaskHook_Consume(TaskHook_t *hook, TaskHookId_t *hook_id, void **source)
{
    if ((hook == NULL) || !hook->pending) {
        return false;
    }

    if (hook_id != NULL) {
        *hook_id = hook->id;
    }
    if (source != NULL) {
        *source = hook->source;
    }

    TaskHook_Reset(hook);
    return true;
}

void TaskAction_Init(TaskAction_t *action,
                     void *profile,
                     void *context,
                     TaskActionEnterFn_t on_enter,
                     TaskActionRunningFn_t on_running,
                     TaskActionExitFn_t on_exit)
{
    if (action == NULL) {
        return;
    }

    action->profile = profile;
    action->context = context;
    action->on_enter = on_enter;
    action->on_running = on_running;
    action->on_exit = on_exit;
    TaskAction_Reset(action);
}

void TaskAction_Reset(TaskAction_t *action)
{
    if (action == NULL) {
        return;
    }

    action->phase = TASK_ACTION_PHASE_ENTER;
    action->latched_result = TASK_ACTION_RUNNING;
    action->exit_handled = false;
}

TaskActionResult_t TaskAction_Tick(TaskAction_t *action)
{
    TaskActionResult_t result;

    if (action == NULL) {
        return TASK_ACTION_FAILURE;
    }

    if ((action->phase == TASK_ACTION_PHASE_EXIT) && action->exit_handled) {
        return action->latched_result;
    }

    switch (action->phase) {
        case TASK_ACTION_PHASE_ENTER:
            result = (action->on_enter != NULL) ? action->on_enter(action) : TASK_ACTION_RUNNING;
            if (result == TASK_ACTION_RUNNING) {
                action->phase = TASK_ACTION_PHASE_RUNNING;
                return TASK_ACTION_RUNNING;
            }

            action->latched_result = result;
            action->phase = TASK_ACTION_PHASE_EXIT;
            break;

        case TASK_ACTION_PHASE_RUNNING:
            result = (action->on_running != NULL) ? action->on_running(action) : TASK_ACTION_FAILURE;
            if (result == TASK_ACTION_RUNNING) {
                return TASK_ACTION_RUNNING;
            }

            action->latched_result = result;
            action->phase = TASK_ACTION_PHASE_EXIT;
            break;

        case TASK_ACTION_PHASE_EXIT:
        default:
            break;
    }

    if (!action->exit_handled) {
        if (action->on_exit != NULL) {
            action->on_exit(action, action->latched_result);
        }
        action->exit_handled = true;
    }

    return action->latched_result;
}

void TaskSequence_Init(TaskSequence_t *sequence, TaskSequenceSlot_t *slots, size_t slot_count)
{
    size_t index;

    if (sequence == NULL) {
        return;
    }

    sequence->slots = slots;
    sequence->slot_count = slot_count;
    sequence->current_index = 0U;
    sequence->interrupt_depth = 0U;
    TaskHook_Reset(&sequence->hook);
    sequence->on_hook = NULL;
    sequence->hook_context = NULL;
    for (index = 0U; index < TASK_INTERRUPT_STACK_CAPACITY; ++index) {
        sequence->interrupt_stack[index] = NULL;
    }
}

void TaskSequence_Reset(TaskSequence_t *sequence)
{
    size_t index;

    if (sequence == NULL) {
        return;
    }

    sequence->current_index = 0U;
    sequence->interrupt_depth = 0U;
    TaskHook_Reset(&sequence->hook);

    for (index = 0U; index < sequence->slot_count; ++index) {
        if ((sequence->slots != NULL) && (sequence->slots[index].action != NULL)) {
            TaskAction_Reset(sequence->slots[index].action);
        }
    }

    for (index = 0U; index < TASK_INTERRUPT_STACK_CAPACITY; ++index) {
        if (sequence->interrupt_stack[index] != NULL) {
            TaskAction_Reset(sequence->interrupt_stack[index]);
            sequence->interrupt_stack[index] = NULL;
        }
    }
}

bool TaskSequence_PushInterrupt(TaskSequence_t *sequence, TaskAction_t *interrupt_action)
{
    if ((sequence == NULL) || (interrupt_action == NULL)) {
        return false;
    }

    if (sequence->interrupt_depth >= TASK_INTERRUPT_STACK_CAPACITY) {
        return false;
    }

    TaskAction_Reset(interrupt_action);
    sequence->interrupt_stack[sequence->interrupt_depth] = interrupt_action;
    sequence->interrupt_depth += 1U;
    return true;
}

void TaskSequence_SetHookNotify(TaskSequence_t *sequence,
                                TaskHookNotifyFn_t on_hook,
                                void *hook_context)
{
    if (sequence == NULL) {
        return;
    }

    sequence->on_hook = on_hook;
    sequence->hook_context = hook_context;
}

bool TaskSequence_EmitHook(TaskSequence_t *sequence, TaskHookId_t hook_id)
{
    bool handled;

    if (sequence == NULL) {
        return false;
    }

    if (!TaskHook_Emit(&sequence->hook, hook_id, sequence)) {
        return false;
    }

    if (sequence->on_hook == NULL) {
        return true;
    }

    handled = sequence->on_hook(sequence->hook_context, &sequence->hook);
    if (handled) {
        TaskHook_Reset(&sequence->hook);
    }

    return true;
}

bool TaskSequence_HasHook(const TaskSequence_t *sequence)
{
    return (sequence != NULL) && sequence->hook.pending;
}

bool TaskSequence_ConsumeHook(TaskSequence_t *sequence, TaskHookId_t *hook_id, void **source)
{
    if (sequence == NULL) {
        return false;
    }

    return TaskHook_Consume(&sequence->hook, hook_id, source);
}

TaskActionResult_t TaskSequence_Tick(TaskSequence_t *sequence)
{
    if ((sequence == NULL) || (sequence->slots == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    while (sequence->current_index < sequence->slot_count) {
        TaskActionResult_t result;
        TaskAction_t *action;

        if (sequence->interrupt_depth > 0U) {
            action = sequence->interrupt_stack[sequence->interrupt_depth - 1U];
            result = TaskAction_Tick(action);
            if (result == TASK_ACTION_RUNNING) {
                return TASK_ACTION_RUNNING;
            }
            if (result == TASK_ACTION_FAILURE) {
                TaskSequence_Reset(sequence);
                return TASK_ACTION_FAILURE;
            }

            sequence->interrupt_depth -= 1U;
            sequence->interrupt_stack[sequence->interrupt_depth] = NULL;
            continue;
        }

        action = sequence->slots[sequence->current_index].action;
        if (action == NULL) {
            sequence->current_index += 1U;
            continue;
        }

        result = TaskAction_Tick(action);
        if (result == TASK_ACTION_RUNNING) {
            return TASK_ACTION_RUNNING;
        }
        if (result == TASK_ACTION_FAILURE) {
            TaskSequence_Reset(sequence);
            return TASK_ACTION_FAILURE;
        }

        sequence->current_index += 1U;
    }

    TaskSequence_Reset(sequence);
    return TASK_ACTION_SUCCESS;
}

void TaskParallel_Init(TaskParallel_t *parallel,
                       TaskAction_t **actions,
                       TaskActionResult_t *results,
                       size_t action_count,
                       TaskParallelCompletePolicy_t complete_policy)
{
    if (parallel == NULL) {
        return;
    }

    parallel->actions = actions;
    parallel->results = results;
    parallel->action_count = action_count;
    parallel->complete_policy = complete_policy;
    TaskParallel_Reset(parallel);
}

void TaskParallel_Reset(TaskParallel_t *parallel)
{
    size_t index;

    if ((parallel == NULL) || (parallel->actions == NULL) || (parallel->results == NULL)) {
        return;
    }

    for (index = 0U; index < parallel->action_count; ++index) {
        parallel->results[index] = TASK_ACTION_RUNNING;
        if (parallel->actions[index] != NULL) {
            TaskAction_Reset(parallel->actions[index]);
        }
    }
}

TaskActionResult_t TaskParallel_Tick(TaskParallel_t *parallel)
{
    size_t index;
    size_t success_count = 0U;
    size_t failure_count = 0U;
    bool any_running = false;

    if ((parallel == NULL) || (parallel->actions == NULL) || (parallel->results == NULL)) {
        return TASK_ACTION_FAILURE;
    }

    for (index = 0U; index < parallel->action_count; ++index) {
        TaskActionResult_t result;

        if (parallel->actions[index] == NULL) {
            failure_count += 1U;
            continue;
        }

        if (parallel->results[index] == TASK_ACTION_SUCCESS) {
            success_count += 1U;
            continue;
        }
        if (parallel->results[index] == TASK_ACTION_FAILURE) {
            failure_count += 1U;
            continue;
        }

        result = TaskAction_Tick(parallel->actions[index]);
        if (result == TASK_ACTION_RUNNING) {
            any_running = true;
            continue;
        }

        parallel->results[index] = result;
        if (result == TASK_ACTION_SUCCESS) {
            success_count += 1U;
        } else {
            failure_count += 1U;
        }
    }

    if (parallel->complete_policy == TASK_PARALLEL_COMPLETE_ANY_SUCCESS) {
        if (success_count > 0U) {
            TaskParallel_Reset(parallel);
            return TASK_ACTION_SUCCESS;
        }
        if (any_running) {
            return TASK_ACTION_RUNNING;
        }

        TaskParallel_Reset(parallel);
        return TASK_ACTION_FAILURE;
    }

    if (failure_count > 0U) {
        TaskParallel_Reset(parallel);
        return TASK_ACTION_FAILURE;
    }

    if (success_count == parallel->action_count) {
        TaskParallel_Reset(parallel);
        return TASK_ACTION_SUCCESS;
    }

    return TASK_ACTION_RUNNING;
}
