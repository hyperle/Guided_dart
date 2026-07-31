#include "bmi088.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_spi.h"

#define BMI088_SPI_TIMEOUT_MS 2U

/* 私有函数：SPI读写（带片选控制） */
static void BMI088_CS_Select(uint16_t pin)
{
    if (pin == ACC_CS_PIN)
        HAL_GPIO_WritePin(ACC_CS_PORT, pin, GPIO_PIN_RESET);
    else if (pin == GYRO_CS_PIN)
        HAL_GPIO_WritePin(GYRO_CS_PORT, pin, GPIO_PIN_RESET);
}

static void BMI088_CS_Release(uint16_t pin)
{
    if (pin == ACC_CS_PIN)
        HAL_GPIO_WritePin(ACC_CS_PORT, pin, GPIO_PIN_SET);
    else if (pin == GYRO_CS_PIN)
        HAL_GPIO_WritePin(GYRO_CS_PORT, pin, GPIO_PIN_SET);
}

/* 读单个寄存器（加速度计） */
static HAL_StatusTypeDef BMI088_Accel_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data = reg | 0x80; // 读操作：最高位为1
    uint8_t dummy;

    BMI088_CS_Select(ACC_CS_PIN);
    status = HAL_SPI_Transmit(hspi, &tx_data, 1, BMI088_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        BMI088_CS_Release(ACC_CS_PIN);
        return status;
    }
    // 加速度计SPI读需要先接收一个dummy字节，再接收真实数据
    status = HAL_SPI_Receive(hspi, &dummy, 1, BMI088_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        BMI088_CS_Release(ACC_CS_PIN);
        return status;
    }
    status = HAL_SPI_Receive(hspi, data, 1, BMI088_SPI_TIMEOUT_MS);
    BMI088_CS_Release(ACC_CS_PIN);

    return status;
}

/* 读多个寄存器（加速度计） */
static HAL_StatusTypeDef BMI088_Accel_ReadRegs(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data = reg | 0x80;
    uint8_t dummy;

    BMI088_CS_Select(ACC_CS_PIN);
    status = HAL_SPI_Transmit(hspi, &tx_data, 1, BMI088_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        BMI088_CS_Release(ACC_CS_PIN);
        return status;
    }
    // 丢弃第一个dummy字节
    status = HAL_SPI_Receive(hspi, &dummy, 1, BMI088_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        BMI088_CS_Release(ACC_CS_PIN);
        return status;
    }
    status = HAL_SPI_Receive(hspi, data, len, BMI088_SPI_TIMEOUT_MS);
    BMI088_CS_Release(ACC_CS_PIN);

    return status;
}

/* 读单个寄存器（陀螺仪） */
static HAL_StatusTypeDef BMI088_Gyro_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data = reg | 0x80;  // Set MSB for read operation

    BMI088_CS_Select(GYRO_CS_PIN);
    status = HAL_SPI_Transmit(hspi, &tx_data, 1, BMI088_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(hspi, data, 1, BMI088_SPI_TIMEOUT_MS);
    }
    BMI088_CS_Release(GYRO_CS_PIN);
    return status;
}

/* 读多个寄存器（陀螺仪） */
static HAL_StatusTypeDef BMI088_Gyro_ReadRegs(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data = reg | 0x80;

    BMI088_CS_Select(GYRO_CS_PIN);
    status = HAL_SPI_Transmit(hspi, &tx_data, 1, BMI088_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        BMI088_CS_Release(GYRO_CS_PIN);
        return status;
    }
    status = HAL_SPI_Receive(hspi, data, len, BMI088_SPI_TIMEOUT_MS);
    BMI088_CS_Release(GYRO_CS_PIN);

    return status;
}

/* 读取加速度计数据（单位：mg） */
HAL_StatusTypeDef bmi088_read_accel(SPI_HandleTypeDef *hspi, float accel[3])
{
    uint8_t raw_data[6] = {0};
    int16_t raw[3];
    uint8_t range_reg;
    float scale;

    // 读取当前量程
    if (BMI088_Accel_ReadReg(hspi, ACC_RANGE, &range_reg) != HAL_OK) {
        return HAL_ERROR;
    }
    range_reg &= 0x03; // 只取低两位

    // 读取6字节原始数据（X, Y, Z，每个2字节，LSB在前）
    if (BMI088_Accel_ReadRegs(hspi, ACC_X_LSB, raw_data, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    // 组合为16位有符号数（小端序）
    raw[0] = (int16_t)((raw_data[1] << 8) | raw_data[0]); // X
    raw[1] = (int16_t)((raw_data[3] << 8) | raw_data[2]); // Y
    raw[2] = (int16_t)((raw_data[5] << 8) | raw_data[4]); // Z


    switch (range_reg) {
        case ACC_RANGE_3:
            scale = 3.0f * 1000.0f / 32768.0f;
            break;
        case ACC_RANGE_6:
            scale = 6.0f * 1000.0f / 32768.0f;
            break;
        case ACC_RANGE_12:
            scale = 12.0f * 1000.0f / 32768.0f;
            break;
        case ACC_RANGE_24:
            scale = 24.0f * 1000.0f / 32768.0f;
            break;
        default:
            scale = 24.0f * 1000.0f / 32768.0f;
            break;
    }
    for (int i = 0; i < 3; i++) {
        accel[i] = raw[i] * scale;
    }
    return HAL_OK;
}

/* 读取陀螺仪数据（单位：°/s） */
HAL_StatusTypeDef bmi088_read_gyro(SPI_HandleTypeDef *hspi, float gyro[3])
{
    uint8_t raw_data[6] = {0};
    int16_t raw[3];
    uint8_t range_reg = 0x00;
    float full_scale;

    // 读取量程
    if (BMI088_Gyro_ReadReg(hspi, GYRO_RANGE, &range_reg) != HAL_OK) {
        return HAL_ERROR;
    }
    range_reg &= 0x07; // 低3位有效
    switch (range_reg) {
        case 0x00: full_scale = 2000.0f; break;
        case 0x01: full_scale = 1000.0f; break;
        case 0x02: full_scale = 500.0f; break;
        case 0x03: full_scale = 250.0f; break;
        case 0x04: full_scale = 125.0f; break;
        default: full_scale = 2000.0f; break;
    }

    // 读取6字节原始数据（X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB）
    if (BMI088_Gyro_ReadRegs(hspi, GYRO_X_LSB, raw_data, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    // 组合为16位有符号数（小端序）
    raw[0] = (int16_t)((raw_data[1] << 8) | raw_data[0]); // X
    raw[1] = (int16_t)((raw_data[3] << 8) | raw_data[2]); // Y
    raw[2] = (int16_t)((raw_data[5] << 8) | raw_data[4]); // Z

    // 转换：°/s = raw * full_scale / 32768
    for (int i = 0; i < 3; i++) {
        gyro[i] = raw[i] * full_scale / 32768.0f;
    }
    return HAL_OK;
}

/* 读取温度（单位：°C） */
HAL_StatusTypeDef bmi088_read_temperature(SPI_HandleTypeDef *hspi, float *temp)
{
    uint8_t data[2];
    if (BMI088_Accel_ReadRegs(hspi, ACC_TEMP_MSB, data, 2) != HAL_OK) {
        return HAL_ERROR;
    }

    // 组合为11位有符号数
    uint16_t temp_uint11 = ((uint16_t)data[0] << 3) | ((uint16_t)data[1] >> 5);
    int16_t temp_int11;
    if (temp_uint11 > 1023) {
        temp_int11 = temp_uint11 - 2048;
    } else {
        temp_int11 = temp_uint11;
    }
    *temp = temp_int11 * 0.125f + 23.0f;
    return HAL_OK;
}

/* 读取所有数据（一次调用） */
HAL_StatusTypeDef bmi088_read_all(SPI_HandleTypeDef *hspi, bmi088_data_t *data)
{
    if (bmi088_read_accel(hspi, data->accel) != HAL_OK) return HAL_ERROR;
    if (bmi088_read_gyro(hspi, data->gyro) != HAL_OK) return HAL_ERROR;
    if (bmi088_read_temperature(hspi, &data->temperature) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}