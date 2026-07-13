#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float integral_limit){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->integral = 0;
    pid->last_error = 0;
    pid->output = 0;
    pid->integral_limit = integral_limit;
}

float PID_Update(PID_t *pid, float error) {
    // 比例项
    float p_out = pid->Kp * error;
    
    // 积分项（带限幅）
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float i_out = pid->Ki * pid->integral;
    
    // 微分项（采用测量值微分，避免微分冲击）
    float derivative = (error - pid->last_error) / pid->dt;
    float d_out = pid->Kd * derivative;
    pid->last_error = error;
    
    // 总输出
    float output = p_out + i_out + d_out;

    pid->output = output;
    return output;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}
