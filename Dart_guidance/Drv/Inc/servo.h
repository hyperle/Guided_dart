#ifndef __SERVO_H
#define __SERVO_H

#include "stm32g4xx_hal.h"

// 四路物理 PWM 输出：PB3/PB4/PB5 为 TIM8 互补端，PB6 为 TIM4 普通端。
// 定时器保持 10MHz 计数，30303 tick 周期约等于 330Hz。

// PWM 目标频率，单位 Hz；需要和 CubeMX 定时器周期配置保持一致。
#define SERVO_PWM_FREQUENCY_HZ 330U

// 定时器计数频率，单位 Hz；当前 10MHz 表示 0.1us 一个 tick。
#define SERVO_TIMER_TICK_HZ    10000000U

// 每 1us 对应的定时器 tick 数；用于把上层 us 脉宽转换为 CCR。
#define SERVO_TIMER_TICKS_PER_US 10U

// PWM 周期 tick 数；必须大于最大输出脉宽换算后的 tick。
#define SERVO_PWM_PERIOD_TICKS 30303U

// PWM 周期，单位 us；用于编译期检查脉宽上限不能超过周期。
#define SERVO_PWM_PERIOD_US    3030U

// PWM 周期，单位 ms；仅用于日志或人工换算参考。
#define SERVO_PWM_PERIOD_MS    3.03f

// 实际控制的舵机通道数量。
#define SERVO_COUNT            4U

// 物理高电平脉宽下限，单位 us；Servo_SetPulseUs 会夹紧到该值以上。
#define SERVO_PULSE_MIN_US     0U

// 物理高电平脉宽上限，单位 us；Servo_SetPulseUs 会夹紧到该值以下。
#define SERVO_PULSE_MAX_US     3000U

// 上电和目标丢失时的默认脉宽，单位 us；固定 PWM 模式也默认引用它。
#define SERVO_PULSE_CENTER_US  2000U

// 角度接口允许的最小角度，单位 deg。
#define SERVO_ANGLE_MIN       0

// 角度接口允许的最大角度，单位 deg；超过后会被夹紧。
#define SERVO_ANGLE_MAX       120

#if SERVO_PULSE_MAX_US >= SERVO_PWM_PERIOD_US
#error "SERVO_PULSE_MAX_US must be less than the physical PWM period."
#endif

// 舵机数量和对应的定时器通道（根据实际连接修改）
#define SERVO1                 1
#define SERVO1_TIM             htim4
#define SERVO1_CHANNEL         TIM_CHANNEL_1
#define SERVO2                 2
#define SERVO2_TIM             htim8
#define SERVO2_CHANNEL         TIM_CHANNEL_1
#define SERVO3                 3
#define SERVO3_TIM             htim8
#define SERVO3_CHANNEL         TIM_CHANNEL_2
#define SERVO4                 4
#define SERVO4_TIM             htim8
#define SERVO4_CHANNEL         TIM_CHANNEL_3

/**
 * @brief 初始化舵机控制
 */
void Servo_Init(void);

/**
 * @brief 设置舵机PWM值
 * @param id     舵机ID（1~4）
 * @param angle  角度值（0~180）
 */
void Servo_SetAngle(uint8_t id, uint8_t angle);
void Servo_SetPulseUs(uint8_t id, float pulse_us);
void Servo_SetPulseUsBatch(const float pulse_us[SERVO_COUNT]);

#endif
