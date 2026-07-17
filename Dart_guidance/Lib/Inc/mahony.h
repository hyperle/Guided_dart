#ifndef __MAHONY_H
#define __MAHONY_H

#include <stdint.h>
#include <math.h>
typedef struct {
    float q0, q1, q2, q3;   // 四元数，表示从机体坐标系到导航坐标系的旋转
    float twoKp;            // 比例系数 * 2
    float twoKi;            // 积分系数 * 2
    float integralFBx, integralFBy, integralFBz; // 积分误差
    float roll, pitch, yaw; // 欧拉角（单位：度）
} mahony_t;

/**
 * @brief 初始化Mahony滤波器
 * @param m      Mahony对象指针
 * @param Kp     比例系数，通常取0.5~2.0
 * @param Ki     积分系数，通常取0.0~0.05
 */
void mahony_init(mahony_t *m, float Kp, float Ki);

/**
 * @brief 使用加速度计和陀螺仪数据更新姿态（四元数）
 * @param m      Mahony对象指针
 * @param gx, gy, gz  陀螺仪角速度（单位：rad/s）
 * @param ax, ay, az  加速度计读数（单位：g，即归一化重力加速度）
 * @param dt          采样时间间隔（单位：秒）
 */
void mahony_update(mahony_t *m, float gx, float gy, float gz,
                   float ax, float ay, float az, float dt);

/**
 * @brief 仅使用陀螺仪数据更新姿态（四元数）
 * @param m      Mahony对象指针
 * @param gx, gy, gz  陀螺仪角速度（单位：rad/s）
 * @param dt          采样时间间隔（单位：秒）
 */
void mahony_update_gyro_only(mahony_t *m, float gx, float gy, float gz, float dt);

/**
 * @brief 清空Mahony反馈积分项
 * @param m Mahony对象指针
 */
void mahony_reset_feedback(mahony_t *m);

/**
 * @brief 从当前四元数计算欧拉角（单位：度）
 * @param m Mahony对象指针
 */
void mahony_compute_euler(mahony_t *m);

#endif