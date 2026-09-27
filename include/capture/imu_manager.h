#pragma once

// IMU 模块公共接口（SPMC、无锁，设计见 IMU_DESIGN.md）
//
// 职责边界：**积分在模块内完成**，消费侧只消费结果。
//   模块负责：标定 → 姿态（角速度积分）、线速度（比力积分，含重力扣除）→ 发布
//   消费侧拿到：姿态角、角速度、线速度；另可按区间要增量，也可要未积分的原始数据
//
// 为什么线速度必须归模块：比力积分要减重力，而重力项要用**姿态**才能转到机体系；
//   姿态又只在模块里维护。放外面做就得把姿态再导出一遍，还得保证两侧同一拍。
//
// 参考坐标系（两侧的共同基准）：机体 x 前、y 右、z 下。
//   imu_set_reference() 把"调用那一刻的姿态"定为初始坐标系，之后 angle[] 都是相对
//   它的累计转角。调用方必须知道这次调用发生在什么时候——那就是它眼里的"零姿态"。
//
// 发射段：出仓加速度可能远超加速度计量程，这一段**不积分**（该帧 saturated 置位），
//   出仓初速度由调用方用 imu_seed_initial_velocity() 人工给定，线速度从它起累加。
//
// 时间基准：模块内部按采样时间戳的 dt 积分（采样不等周期，必须用 dt）；控制律的
//   "按拍、不乘 dt"是另一回事，两者别混。
//
// 目前只有声明，实现（src/capture/imu_manager.c）还没写。

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 未积分的原始数据（标定后）
typedef struct {
    float gyro[3]; // 机体系角速度 (rad/s)
    float acc[3];  // 机体系比力 (m/s^2)
    uint64_t timestamp;
    bool saturated; // 这一帧加速度计量程被打满
} imu_raw_t;

// 积分结果（相对 imu_set_reference() 那一刻）
typedef struct {
    float angle[3];    // 累计转角 (rad)
    float rate[3];     // 角速度 (rad/s)
    float velocity[3]; // 机体系线速度 (m/s)
    uint64_t timestamp;
    bool velocity_valid; // 只在 seed 之后为真；之前 velocity 无意义
} imu_state_t;

// ---- 生产者：只在 IMU 中断/采集线程里调用，全模块唯一写者 ----
void imu_provider_post_sample(const float raw_gyro[3], const float raw_acc[3], uint64_t timestamp_us);

// ---- 参考系与初速度：发射前 / 出仓瞬间各调一次 ----
void imu_set_reference(void);
void imu_seed_initial_velocity(const float velocity[3]);

// ---- 消费者 A（快轨）：最新积分结果，seqlock，非阻塞 ----
bool imu_get_state(imu_state_t *out_state);

// ---- 消费者 B（慢轨）：区间量，时间戳精确可查；越界/丢数据在模块内消化 ----
bool imu_get_delta_angle_between(uint64_t start_time_us, uint64_t end_time_us, float out_delta[3]);
bool imu_get_delta_velocity_between(uint64_t start_time_us, uint64_t end_time_us, float out_delta[3]);
bool imu_get_delta_yaw_between(uint64_t start_time_us, uint64_t end_time_us, float *out_delta_yaw);

// ---- 原始（未积分）数据：给需要自己算的调用方 ----
bool imu_get_raw_latest(imu_raw_t *out_raw);

#ifdef __cplusplus
}
#endif
