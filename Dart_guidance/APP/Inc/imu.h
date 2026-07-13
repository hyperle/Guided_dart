#ifndef __IMU_H
#define __IMU_H

#include "stm32g4xx_hal.h"
#include "mahony.h"
#include <stdint.h>

/* 稳定姿态滚动窗口长度；发射脉冲触发时会用该窗口均值锁存参考姿态。 */
#define IMU_REFERENCE_WINDOW_SIZE 10U

/* 可信静止加速度目标值，单位 g；理想静止时加速度模长约为 1g。 */
#define IMU_ACCEL_NORM_TARGET_G 1.0f

/* BMI088 加速度计满量程，单位 g；与 bmi088.c 中的 ACC_RANGE_24 配置保持一致。 */
#define IMU_ACCEL_FULL_SCALE_G 24.0f

/* 加速度可信窗口半宽，单位 g；模长偏离 1g 超过该值时切到 gyro-only。 */
#define IMU_DEFAULT_ACCEL_TRUST_DELTA_G 0.35f

/* 发射/冲击脉冲判定阈值，单位 g；模长相对 1g 偏差超过该值会锁存发射参考。 */
#define IMU_DEFAULT_ACCEL_PULSE_THRESHOLD_G 2.0f

/* 单轴接近饱和比例；任一轴达到满量程乘以该比例时认为加速度不可用于姿态纠偏。 */
#define IMU_DEFAULT_ACCEL_AXIS_SATURATION_RATIO 0.98f

/* 高动态后恢复延时，单位 ms；加速度恢复可信后等待该时间再恢复 accel+gyro Mahony 纠偏。 */
#define IMU_DEFAULT_RECOVERY_DELAY_MS 150U

/* Mahony 默认比例增益；越大越信任加速度纠偏，越小越依赖陀螺积分。 */
#define GUIDANCE_IMU_MAHONY_KP 1.0f

/* Mahony 默认积分增益；用于慢速偏置修正，目前默认关闭积分。 */
#define GUIDANCE_IMU_MAHONY_KI 0.0f

typedef struct {
    float accel_trust_delta_g;
    float accel_pulse_threshold_g;
    float accel_axis_saturation_ratio;
    uint16_t recovery_delay_ms;
} imu_config_t;

/* IMU 状态结构体 */
typedef struct {
    mahony_t mahony;                        // Mahony滤波器实例
    SPI_HandleTypeDef *hspi;                // SPI句柄（用于BMI088通信）
    uint8_t initialized;                    // BMI088启动并通过就绪检验后置1
    uint32_t last_tick;                     // 上一次更新的系统滴答计数（毫秒）
    uint32_t stable_since_tick;             // 最近一次非可信加速度样本时刻（毫秒）
    float dt;                               // 实际时间间隔（秒）
    float roll;                             // 当前横滚角（度）
    float pitch;                            // 当前俯仰角（度）
    float yaw;                              // 当前航向角（度）
    float mean_roll;                        // 滚动窗口横滚均值（度）
    float mean_pitch;                       // 滚动窗口俯仰均值（度）
    float mean_yaw;                         // 滚动窗口航向均值（度）
    float launch_ref_roll;                  // 发射锁存横滚参考（度）
    float launch_ref_pitch;                 // 发射锁存俯仰参考（度）
    float launch_ref_yaw;                   // 发射锁存航向参考（度）
    float relative_roll;                    // 相对参考横滚误差（度）
    float relative_pitch;                   // 相对参考俯仰误差（度）
    float relative_yaw;                     // 相对参考航向误差（度）
    float roll_window[IMU_REFERENCE_WINDOW_SIZE];
    float pitch_window[IMU_REFERENCE_WINDOW_SIZE];
    float yaw_window[IMU_REFERENCE_WINDOW_SIZE];
    float roll_sum;
    float pitch_sum;
    float yaw_sum;
    uint8_t window_index;                   // 窗口写指针
    uint8_t window_count;                   // 当前窗口样本数
    uint8_t accel_is_trusted;               // 当前加速度是否可信
    uint8_t high_dynamic_mode;              // 当前是否处于高动态gyro-only模式
    uint8_t launch_reference_valid;         // 是否已锁存发射参考姿态
    uint8_t pulse_active;                   // 当前是否处于加速度脉冲中
    float accel_x;                          // 最新 X 轴加速度（g）
    float accel_y;                          // 最新 Y 轴加速度（g）
    float accel_z;                          // 最新 Z 轴加速度（g）
    float gyro_x;                           // 最新 X 轴角速度（deg/s）
    float gyro_y;                           // 最新 Y 轴角速度（deg/s）
    float gyro_z;                           // 最新 Z 轴角速度（deg/s）
    imu_config_t config;                    // 由头文件宏初始化的配置快照
} imu_t;

/**
 * @brief 初始化IMU模块
 * @param imu  IMU对象指针
 * @param hspi SPI句柄（已配置好时钟、极性相位等）
 * @param Kp   Mahony滤波器比例系数
 * @param Ki   Mahony滤波器积分系数
 * @retval HAL_OK 成功，否则错误码
 */
HAL_StatusTypeDef imu_init(imu_t *imu, SPI_HandleTypeDef *hspi, float Kp, float Ki);

/**
 * @brief 更新IMU姿态（应周期性调用）
 * @param imu IMU对象指针
 * @retval HAL_OK 成功，否则错误码
 */
HAL_StatusTypeDef imu_update(imu_t *imu);

/**
 * @brief 获取当前欧拉角（度）
 * @param imu   IMU对象指针
 * @param roll  横滚角输出指针
 * @param pitch 俯仰角输出指针
 * @param yaw   航向角输出指针
 */
void imu_get_euler(imu_t *imu, float *roll, float *pitch, float *yaw);

/**
 * @brief 获取滚动窗口姿态均值（度）
 */
void imu_get_mean_euler(imu_t *imu, float *roll, float *pitch, float *yaw);

/**
 * @brief 获取最近一次发射锁存的参考姿态（度）
 */
void imu_get_launch_reference_euler(imu_t *imu, float *roll, float *pitch, float *yaw);

/**
 * @brief 获取相对发射参考的姿态误差（度）
 */
void imu_get_relative_euler(imu_t *imu, float *roll, float *pitch, float *yaw);

/**
 * @brief 获取最新原始加速度（g）
 * @param imu  IMU对象指针
 * @param ax   X轴加速度输出指针
 * @param ay   Y轴加速度输出指针
 * @param az   Z轴加速度输出指针
 */
void imu_get_accel(imu_t *imu, float *ax, float *ay, float *az);

/**
 * 获取最新原始角速度（deg/s）。
 * gx/gy/gz 为输出指针，可为 NULL。
 */
void imu_get_gyro(imu_t *imu, float *gx, float *gy, float *gz);

#endif /* __IMU_H */
