#pragma once

// IMU 模块公共接口（SPMC、无锁，设计见 IMU_DESIGN.md）
//   生产者：imu_provider_post_sample —— 只在 IMU 中断里调用，全模块唯一写者
//   消费者 A（快轨）：imu_get_latest —— 飞控姿态环 / 角速度环，只要"当下最新"
//   消费者 B（慢轨）：imu_get_delta_yaw_between —— 视觉去自旋，要"某段区间"
//
// 目前只有声明，实现（src/capture/imu_manager.c）还没写。

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro[3]; // 已标定的机体系角速度 (rad/s)
    float acc[3];  // 已标定的机体系加速度 (m/s^2)
    uint64_t timestamp;
} imu_sample_t;

void imu_provider_post_sample(const float raw_gyro[3], const float raw_acc[3], uint64_t timestamp_us);

bool imu_get_latest(imu_sample_t *out_sample);

// 区间角度增量（rad，机体系三轴）；控制环用"这一拍转了多少"做速率反馈
bool imu_get_delta_angle_between(uint64_t start_time_us, uint64_t end_time_us, float out_delta[3]);

bool imu_get_delta_yaw_between(uint64_t start_time_us, uint64_t end_time_us, float *out_delta_yaw);

#ifdef __cplusplus
}
#endif
