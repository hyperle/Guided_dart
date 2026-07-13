#include "mahony.h"
#include "arm_math.h"

void mahony_reset_feedback(mahony_t *m)
{
    m->integralFBx = 0.0f;
    m->integralFBy = 0.0f;
    m->integralFBz = 0.0f;
}

void mahony_update_gyro_only(mahony_t *m, float gx, float gy, float gz, float dt)
{
    float norm;
    float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
    float q0dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float q1dot = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float q2dot = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float q3dot = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    m->q0 += q0dot * dt;
    m->q1 += q1dot * dt;
    m->q2 += q2dot * dt;
    m->q3 += q3dot * dt;
    arm_sqrt_f32(m->q0*m->q0 + m->q1*m->q1 + m->q2*m->q2 + m->q3*m->q3, &norm);
    if (norm > 0.0f) {
        m->q0 /= norm;
        m->q1 /= norm;
        m->q2 /= norm;
        m->q3 /= norm;
    }
}


void mahony_init(mahony_t *m, float Kp, float Ki)
{
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;
    m->twoKp = 2.0f * Kp;
    m->twoKi = 2.0f * Ki;
    mahony_reset_feedback(m);
    m->roll = 0.0f;
    m->pitch = 0.0f;
    m->yaw = 0.0f;
}

void mahony_update(mahony_t *m, float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 如果加速度计读数很小，则忽略此次修正（避免除以零）
    if (ax * ax + ay * ay + az * az == 0.0f) {
        mahony_update_gyro_only(m, gx, gy, gz, dt);
        return;
    }

    // 归一化加速度计读数
    arm_sqrt_f32(ax*ax + ay*ay + az*az,&norm);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 根据当前四元数估计重力向量在机体坐标系下的分量
    vx = 2.0f * (m->q1*m->q3 - m->q0*m->q2);
    vy = 2.0f * (m->q0*m->q1 + m->q2*m->q3);
    vz = m->q0*m->q0 - m->q1*m->q1 - m->q2*m->q2 + m->q3*m->q3;

    // 计算误差 = 测量向量与估计向量的叉积
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 积分误差
    if (m->twoKi > 0.0f) {
        m->integralFBx += m->twoKi * ex * dt;
        m->integralFBy += m->twoKi * ey * dt;
        m->integralFBz += m->twoKi * ez * dt;
        // 用积分误差修正陀螺仪
        gx += m->integralFBx;
        gy += m->integralFBy;
        gz += m->integralFBz;
    }

    // 比例修正陀螺仪
    gx += m->twoKp * ex;
    gy += m->twoKp * ey;
    gz += m->twoKp * ez;

    //静止下零漂补偿
    if(fabsf(norm - 1.0f) < 0.01f){
        gz = 0.0f;
    }

    // 一阶龙格-库塔法更新四元数
    float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;
    float q0dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float q1dot = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float q2dot = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float q3dot = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    m->q0 += q0dot * dt;
    m->q1 += q1dot * dt;
    m->q2 += q2dot * dt;
    m->q3 += q3dot * dt;

    // 四元数归一化
    arm_sqrt_f32(m->q0*m->q0 + m->q1*m->q1 + m->q2*m->q2 + m->q3*m->q3,&norm);
    if (norm > 0.0f) {
        m->q0 /= norm;
        m->q1 /= norm;
        m->q2 /= norm;
        m->q3 /= norm;
    }
}

void mahony_compute_euler(mahony_t *m)
{
    // 从四元数计算欧拉角（单位：度）
    // 旋转顺序：Z-Y-X（航向-俯仰-横滚），公式：
    // roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    // pitch = asin(2*(q0*q2 - q3*q1))
    // yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    float32_t q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

    m->roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / (float)M_PI;
    m->pitch = asinf(2.0f * (q0*q2 - q3*q1)) * 180.0f / (float)M_PI;
    m->yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / (float)M_PI;
}