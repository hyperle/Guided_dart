#pragma once
#include <stdint.h>
typedef struct {
    int bus;
    uint8_t address;
    uint32_t bus_hz;
    uint32_t timeout_ms;
    uint8_t scl_pin;
    uint8_t sda_pin;
    float accel_fs_g;
    float gyro_fs_dps;
} mpu6050_config_t;

typedef struct {
    float x_g, y_g, z_g;
    float temperature_c;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
} mpu6050_sample_t;

int  mpu6050_init(const mpu6050_config_t* config);
int  mpu6050_read(mpu6050_sample_t* sample);
int  mpu6050_set_accel_range(float full_scale_g);
int  mpu6050_set_gyro_range(float full_scale_dps);
void mpu6050_deinit(void);
