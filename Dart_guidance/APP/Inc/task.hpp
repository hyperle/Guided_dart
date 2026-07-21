#ifndef DART_GUIDANCE_TASK_HPP
#define DART_GUIDANCE_TASK_HPP

#include "task_action.hpp"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TASK_INTERRUPT_STACK_CAPACITY 4U
#define TASK_HOOK_ID_NONE 0U

typedef uint16_t TaskHookId_t;

typedef struct
{
    bool pending;
    TaskHookId_t id;
    void *source;
} TaskHook_t;

typedef bool (*TaskHookNotifyFn_t)(void *context, const TaskHook_t *hook);

typedef enum
{
    TASK_PARALLEL_COMPLETE_ANY_SUCCESS = 0,
    TASK_PARALLEL_COMPLETE_ALL_SUCCESS = 1
} TaskParallelCompletePolicy_t;

typedef struct
{
    TaskAction_t *action;
} TaskSequenceSlot_t;

typedef struct
{
    TaskSequenceSlot_t *slots;
    size_t slot_count;
    size_t current_index;
    TaskAction_t *interrupt_stack[TASK_INTERRUPT_STACK_CAPACITY];
    size_t interrupt_depth;
    TaskHook_t hook;
    TaskHookNotifyFn_t on_hook;
    void *hook_context;
} TaskSequence_t;

typedef struct
{
    TaskAction_t **actions;
    TaskActionResult_t *results;
    size_t action_count;
    TaskParallelCompletePolicy_t complete_policy;
} TaskParallel_t;

void TaskHook_Reset(TaskHook_t *hook);
bool TaskHook_Emit(TaskHook_t *hook, TaskHookId_t hook_id, void *source);
bool TaskHook_Consume(TaskHook_t *hook, TaskHookId_t *hook_id, void **source);

void TaskSequence_Init(TaskSequence_t *sequence, TaskSequenceSlot_t *slots, size_t slot_count);
void TaskSequence_Reset(TaskSequence_t *sequence);
TaskActionResult_t TaskSequence_Tick(TaskSequence_t *sequence);
bool TaskSequence_PushInterrupt(TaskSequence_t *sequence, TaskAction_t *interrupt_action);
void TaskSequence_SetHookNotify(TaskSequence_t *sequence,
                                TaskHookNotifyFn_t on_hook,
                                void *hook_context);
bool TaskSequence_EmitHook(TaskSequence_t *sequence, TaskHookId_t hook_id);
bool TaskSequence_HasHook(const TaskSequence_t *sequence);
bool TaskSequence_ConsumeHook(TaskSequence_t *sequence, TaskHookId_t *hook_id, void **source);

void TaskParallel_Init(TaskParallel_t *parallel,
                       TaskAction_t **actions,
                       TaskActionResult_t *results,
                       size_t action_count,
                       TaskParallelCompletePolicy_t complete_policy);
void TaskParallel_Reset(TaskParallel_t *parallel);
TaskActionResult_t TaskParallel_Tick(TaskParallel_t *parallel);

#ifdef __cplusplus
}
#endif

#endif
