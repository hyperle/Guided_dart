#ifndef __GUIDANCE_CONTROLLER_H
#define __GUIDANCE_CONTROLLER_H

#include "guidance_types.h"
#include "pid.h"
#include "servo.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* 控制器计算周期，单位 ms；PID 的 dt 由它换算，需和主循环调度周期保持一致。 */
#define GUIDANCE_CONTROLLER_LOOP_PERIOD_MS 10U

/* 自动目标点标记：setpoint 使用该值时，会在初始化时归一化为当前图像中心。 */
#define GUIDANCE_AIM_AUTO_SETPOINT_COORDINATE GUIDANCE_NO_TARGET_COORDINATE

/* 控制器默认图像宽度，单位 px；只作为模块默认值，main.c 启动后可用任务侧默认尺寸覆盖。 */
#define GUIDANCE_CONTROLLER_DEFAULT_IMAGE_WIDTH 320U

/* 控制器默认图像高度，单位 px；只作为模块默认值，main.c 启动后可用任务侧默认尺寸覆盖。 */
#define GUIDANCE_CONTROLLER_DEFAULT_IMAGE_HEIGHT 240U

/* 默认控制模式；GUIDANCE_CONTROL_MODE_FIXED_PWM 用四路固定脉宽，GUIDANCE_CONTROL_MODE_HORIZONTAL_PWM_PID 才启用水平 PID。 */
#define GUIDANCE_DEFAULT_CONTROL_MODE GUIDANCE_CONTROL_MODE_HORIZONTAL_PWM_PID

/* 四路舵机初始脉宽，单位 us。 */
#define GUIDANCE_SERVO_INITIAL_PWM_US_3 1900U       //1  大左小右
#define GUIDANCE_SERVO_INITIAL_PWM_US_0 2120U       //2  小左大右
#define GUIDANCE_SERVO_INITIAL_PWM_US_2 1950U       //3  大左小右
#define GUIDANCE_SERVO_INITIAL_PWM_US_1 1990U       //4  大左小右

/* 水平 PID 默认 Kp；主要决定 delta_x 误差到 PWM 修正量的比例。 */
#define GUIDANCE_HORIZONTAL_PID_KP 1.0f

/* 水平 PID 默认 Ki；用于消除长期偏差，过大容易积分累积。 */
#define GUIDANCE_HORIZONTAL_PID_KI 0.05f

/* 水平 PID 默认 Kd；用于抑制快速变化，目前默认关闭。 */
#define GUIDANCE_HORIZONTAL_PID_KD 0.01f

/* 水平 PID 积分限幅；限制积分项最大绝对值，避免长时间丢靶后输出冲击。 */
#define GUIDANCE_HORIZONTAL_PID_INTEGRAL_LIMIT 1000.0f

/* 水平 PID 输出限幅，单位 us；限制零点附近的 PWM 修正量。 */
#define GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US 100.0f

/* 水平 PID 输出方向开关；置 true 时会反向 delta_x 对 PWM 的修正方向。 */
#define GUIDANCE_HORIZONTAL_PID_INVERT_OUTPUT false

/* 竖直 PID 默认 Kp；主要决定 delta_y 误差到 PWM 修正量的比例。 */
#define GUIDANCE_VERTICAL_PID_KP 1.0f

/* 竖直 PID 默认 Ki；用于消除长期偏差，过大容易积分累积。 */
#define GUIDANCE_VERTICAL_PID_KI 0.05f

/* 竖直 PID 默认 Kd；用于抑制快速变化。 */
#define GUIDANCE_VERTICAL_PID_KD 0.01f

/* 竖直 PID 积分限幅；限制积分项最大绝对值，避免长时间丢靶后输出冲击。 */
#define GUIDANCE_VERTICAL_PID_INTEGRAL_LIMIT 1000.0f

/* 竖直 PID 输出限幅，单位 us；限制零点附近的 PWM 修正量。 */
#define GUIDANCE_VERTICAL_PID_OUTPUT_LIMIT_US 100.0f

/* 竖直 PID 输出方向开关；置 true 时会反向 delta_y 对 PWM 的修正方向。 */
#define GUIDANCE_VERTICAL_PID_INVERT_OUTPUT false

typedef enum
{
    GUIDANCE_CONTROL_MODE_FIXED_PWM = 1,
    GUIDANCE_CONTROL_MODE_HORIZONTAL_PWM_PID = 2
} GuidanceControlMode_t;

typedef struct
{
    uint16_t image_width;
    uint16_t image_height;
    uint16_t setpoint_x;
    uint16_t setpoint_y;
} GuidanceAimConfig_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    bool invert_output;
} GuidanceHorizontalPwmPidConfig_t;

typedef GuidanceHorizontalPwmPidConfig_t GuidanceVerticalPwmPidConfig_t;

typedef struct
{
    GuidanceAimConfig_t aim_config;
    GuidanceControlMode_t control_mode;
    GuidanceHorizontalPwmPidConfig_t horizontal_pwm_pid_config;
    GuidanceVerticalPwmPidConfig_t vertical_pwm_pid_config;
} GuidanceController_Config_t;

typedef struct
{
    GuidanceAimConfig_t aim_config;
    GuidanceAimCommand_t aim_command;
    GuidanceControlMode_t control_mode;
    GuidanceHorizontalPwmPidConfig_t horizontal_pwm_pid_config;
    GuidanceVerticalPwmPidConfig_t vertical_pwm_pid_config;
    PID_t horizontal_pid;
    GuidanceMeasurement_t measurement;
    GuidanceRelativeAttitudeError_t relative_attitude_error;
    GuidanceServoPulseUs_t servo_pulse_us;
} GuidanceController_t;

void GuidanceController_LoadDefaultConfig(GuidanceController_Config_t *config);
void GuidanceController_Init(GuidanceController_t *controller,
                             UART_HandleTypeDef *receiver_uart,
                             const GuidanceController_Config_t *config);
bool GuidanceController_FetchMeasurement(GuidanceController_t *controller);
bool GuidanceController_SetMeasurement(GuidanceController_t *controller,
                                       const GuidanceMeasurement_t *measurement);
bool GuidanceController_SetRelativeAttitudeError(GuidanceController_t *controller,
                                                 const GuidanceRelativeAttitudeError_t *error);
void GuidanceController_Solve(GuidanceController_t *controller);
void GuidanceController_ApplyOutputs(const GuidanceController_t *controller);
void GuidanceController_RunControl(GuidanceController_t *controller);
void GuidanceController_RunOnce(GuidanceController_t *controller);
void Controller_Horizontal_Turn(GuidanceController_t *controller);
void Controller_Vertical_Turn(GuidanceController_t *controller);
#endif
