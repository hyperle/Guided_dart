#ifndef DART_GUIDANCE_TASK_ACTION_HPP
#define DART_GUIDANCE_TASK_ACTION_HPP

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TASK_ACTION_PHASE_ENTER = 0,
    TASK_ACTION_PHASE_RUNNING = 1,
    TASK_ACTION_PHASE_EXIT = 2
} TaskActionPhase_t;

typedef enum
{
    TASK_ACTION_RUNNING = 0,
    TASK_ACTION_SUCCESS = 1,
    TASK_ACTION_FAILURE = 2
} TaskActionResult_t;

struct TaskAction;

typedef TaskActionResult_t (*TaskActionEnterFn_t)(struct TaskAction *action);
typedef TaskActionResult_t (*TaskActionRunningFn_t)(struct TaskAction *action);
typedef void (*TaskActionExitFn_t)(struct TaskAction *action, TaskActionResult_t result);

typedef struct TaskAction
{
    TaskActionPhase_t phase;
    TaskActionResult_t latched_result;
    bool exit_handled;
    void *profile;
    void *context;
    TaskActionEnterFn_t on_enter;
    TaskActionRunningFn_t on_running;
    TaskActionExitFn_t on_exit;
} TaskAction_t;

void TaskAction_Init(TaskAction_t *action,
                     void *profile,
                     void *context,
                     TaskActionEnterFn_t on_enter,
                     TaskActionRunningFn_t on_running,
                     TaskActionExitFn_t on_exit);
void TaskAction_Reset(TaskAction_t *action);
TaskActionResult_t TaskAction_Tick(TaskAction_t *action);

#ifdef __cplusplus
}
#endif

#endif
