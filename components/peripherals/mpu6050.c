#include "mpu6050.h"

#include "drv_fpioa.h"
#include "drv_i2c.h"

#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6b
#define MPU6050_REG_ACCEL_CONFIG 0x1c
#define MPU6050_REG_GYRO_CONFIG 0x1b
#define MPU6050_REG_ACCEL_XOUT_H 0x3b

static drv_i2c_inst_t *g_i2c;
static mpu6050_config_t g_config;
static int g_accel_lsb = 16384;
static int g_gyro_lsb = 131;

static int write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    i2c_msg_t message = {
        .addr = g_config.address,
        .flags = DRV_I2C_WR,
        .len = 2,
        .buf = buffer,
    };
    return drv_i2c_transfer(g_i2c, &message, 1) == 1 ? 0 : -1;
}

static int read_registers(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    i2c_msg_t messages[2] = {
        {.addr = g_config.address, .flags = DRV_I2C_WR, .len = 1, .buf = &reg},
        {.addr = g_config.address, .flags = DRV_I2C_RD, .len = length, .buf = buffer},
    };
    return drv_i2c_transfer(g_i2c, messages, 2) == 2 ? 0 : -1;
}

static int set_range(float value, const float *ranges, int count,
                     uint8_t reg, int gyro)
{
    int index = 0;
    while (index < count && value != ranges[index]) {
        ++index;
    }
    if (index == count || write_register(reg, (uint8_t)(index << 3)) != 0) {
        return -1;
    }

    if (gyro) {
        g_gyro_lsb = 131 / (1 << index);
    } else {
        g_accel_lsb = 16384 / (1 << index);
    }
    return 0;
}

int mpu6050_init(const mpu6050_config_t *config)
{
    static const int scl_functions[] = {IIC0_SCL, IIC1_SCL, IIC2_SCL,
                                        IIC3_SCL, IIC4_SCL};
    static const int sda_functions[] = {IIC0_SDA, IIC1_SDA, IIC2_SDA,
                                        IIC3_SDA, IIC4_SDA};
    uint8_t device_id = 0;

    if (!config || config->bus < 0 || config->bus > 4 ||
        config->address > 0x7f) {
        return -1;
    }

    g_config = *config;
    if (drv_fpioa_set_pin_func(config->scl_pin, scl_functions[config->bus]) != 0 ||
        drv_fpioa_set_pin_func(config->sda_pin, sda_functions[config->bus]) != 0 ||
        drv_i2c_inst_create(config->bus, config->bus_hz, config->timeout_ms,
                            0xff, 0xff, &g_i2c) != 0 ||
        read_registers(MPU6050_REG_WHO_AM_I, &device_id, 1) != 0 ||
        (device_id != 0x68 && device_id != 0x69) ||
        write_register(MPU6050_REG_PWR_MGMT_1, 0) != 0 ||
        mpu6050_set_accel_range(config->accel_fs_g) != 0 ||
        mpu6050_set_gyro_range(config->gyro_fs_dps) != 0) {
        mpu6050_deinit();
        return -1;
    }
    return 0;
}

int mpu6050_set_accel_range(float full_scale_g)
{
    static const float ranges[] = {2, 4, 8, 16};
    return g_i2c ? set_range(full_scale_g, ranges, 4,
                             MPU6050_REG_ACCEL_CONFIG, 0) : -1;
}

int mpu6050_set_gyro_range(float full_scale_dps)
{
    static const float ranges[] = {250, 500, 1000, 2000};
    return g_i2c ? set_range(full_scale_dps, ranges, 4,
                             MPU6050_REG_GYRO_CONFIG, 1) : -1;
}

int mpu6050_read(mpu6050_sample_t *sample)
{
    uint8_t buffer[14];
    if (!g_i2c || !sample || read_registers(MPU6050_REG_ACCEL_XOUT_H,
                                            buffer, sizeof(buffer)) != 0) {
        return -1;
    }

    sample->x_g = (int16_t)((buffer[0] << 8) | buffer[1]) /
                  (float)g_accel_lsb;
    sample->y_g = (int16_t)((buffer[2] << 8) | buffer[3]) /
                  (float)g_accel_lsb;
    sample->z_g = (int16_t)((buffer[4] << 8) | buffer[5]) /
                  (float)g_accel_lsb;
    sample->temperature_c = (int16_t)((buffer[6] << 8) | buffer[7]) /
                            340.0f + 36.53f;
    sample->gyro_x_dps = (int16_t)((buffer[8] << 8) | buffer[9]) /
                         (float)g_gyro_lsb;
    sample->gyro_y_dps = (int16_t)((buffer[10] << 8) | buffer[11]) /
                         (float)g_gyro_lsb;
    sample->gyro_z_dps = (int16_t)((buffer[12] << 8) | buffer[13]) /
                         (float)g_gyro_lsb;
    return 0;
}

void mpu6050_deinit(void)
{
    if (g_i2c) {
        drv_i2c_inst_destroy(&g_i2c);
    }
}
