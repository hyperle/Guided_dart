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
} TaskSequence_t;

typedef struct
{
    TaskAction_t **actions;
    TaskActionResult_t *results;
    size_t action_count;
    TaskParallelCompletePolicy_t complete_policy;
} TaskParallel_t;

void TaskSequence_Init(TaskSequence_t *sequence, TaskSequenceSlot_t *slots, size_t slot_count);
void TaskSequence_Reset(TaskSequence_t *sequence);
TaskActionResult_t TaskSequence_Tick(TaskSequence_t *sequence);
bool TaskSequence_PushInterrupt(TaskSequence_t *sequence, TaskAction_t *interrupt_action);

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
