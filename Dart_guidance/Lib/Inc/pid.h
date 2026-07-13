#ifndef __PID_H
#define __PID_H

#include "stm32g4xx_hal.h"
#include <sys/types.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float output;
    float integral_limit;   // 积分限幅
    float dt;               // 控制周期
} PID_t;

/**
 * @brief 初始化PID控制器
 * @param pid    PID对象指针
 * @param Kp     比例系数
 * @param Ki     积分系数
 * @param Kd     微分系数
 * @param dt     控制周期
 * @param integral_limit 积分限幅
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float integral_limit);

/**
 * @brief 更新PID控制器
 * @param pid   PID对象指针
 * @param error 当前误差
 * @retval 控制输出
 */
float PID_Update(PID_t *pid, float error);
void PID_Reset(PID_t *pid);

#endif
