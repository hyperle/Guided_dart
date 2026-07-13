#ifndef DART_GUIDANCE_GREEN_LIGHT_TASK_HPP
#define DART_GUIDANCE_GREEN_LIGHT_TASK_HPP

#include "task.hpp"
#include "task_profile.hpp"

/* 默认测量图像宽度，单位 px；启动时先用它初始化 setpoint。 */
#define GREEN_LIGHT_TASK_DEFAULT_IMAGE_WIDTH 320U

/* 默认测量图像高度，单位 px；OpenMV 上报真实尺寸前用作 fallback。 */
#define GREEN_LIGHT_TASK_DEFAULT_IMAGE_HEIGHT 240U

/* 是否默认使用当前图像中心作为目标点；1 为自动居中，0 时使用下面的固定坐标。 */
#define GREEN_LIGHT_TASK_SETPOINT_USE_IMAGE_CENTER 0U

/* 固定目标点 X 坐标，单位 px；仅当 GREEN_LIGHT_TASK_SETPOINT_USE_IMAGE_CENTER 为 0 时生效。 */
#define GREEN_LIGHT_TASK_SETPOINT_X 160U

/* 固定目标点 Y 坐标，单位 px；仅当 GREEN_LIGHT_TASK_SETPOINT_USE_IMAGE_CENTER 为 0 时生效。 */
#define GREEN_LIGHT_TASK_SETPOINT_Y 120U

/* 任务与控制主循环周期，单位 ms；同时影响 IMU 更新、控制输出和 telemetry 刷新节奏。 */
#define GREEN_LIGHT_TASK_LOOP_PERIOD_MS 10U

/* 等待 OpenMV 首帧图像尺寸的最长时间，单位 ms；超时后继续使用默认尺寸。 */
#define GREEN_LIGHT_TASK_STARTUP_IMAGE_SIZE_TIMEOUT_MS 8000U

/* 无控制 tick 时的空闲延时，单位 ms；越小响应越快但主循环占用越高。 */
#define GREEN_LIGHT_TASK_IDLE_POLL_DELAY_MS 1U

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    GreenLightTaskProfile_t *profile;

    TaskAction_t acquire_measurement_action;
    TaskAction_t compute_delta_action;
    TaskAction_t finalize_action;

    TaskSequenceSlot_t sequence_slots[3];
    TaskSequence_t sequence_task;
} GreenLightTask_t;

void GreenLightTask_Init(GreenLightTask_t *task, GreenLightTaskProfile_t *profile);
void GreenLightTask_Reset(GreenLightTask_t *task);
TaskActionResult_t GreenLightTask_Tick(GreenLightTask_t *task);
bool GreenLightTask_InsertInterrupt(GreenLightTask_t *task, TaskAction_t *interrupt_action);

#ifdef __cplusplus
}
#endif

#endif
