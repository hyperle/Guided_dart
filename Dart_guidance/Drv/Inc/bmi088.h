#ifndef __BMI088_H
#define __BMI088_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"

/* 加速度计寄存器地址 */
/*******寄存器名称***********寄存器地址*******备注*******/
#define ACC_CHIP_ID         0x00            //加速度计ID
#define ACC_ERR_REG         0x02            //报告传感器错误条件
#define ACC_STATUS          0x03            //传感器状态标志
#define ACC_X_LSB           0x12            //X_ACC_LSB
#define ACC_X_MSB           0x13            //X_ACC_MSB
#define ACC_Y_LSB           0x14            //Y_ACC_LSB
#define ACC_Y_MSB           0x15            //Y_ACC_MSB
#define ACC_Z_LSB           0x16            //Z_ACC_LSB
#define ACC_Z_MSB           0x17            //Z_ACC_MSB
#define ACC_TEMP_MSB        0x22            //TEMP_MSB
#define ACC_TEMP_LSB        0x23            //TEMP_LSB
#define ACC_INT_STAT_1      0x1D            //中断状态寄存器
#define ACC_CONF            0x40            //配置加速度计的寄存器
#define ACC_RANGE           0x41            //加速度计范围设置寄存器
#define ACC_PWR_CONF        0x7C            //加速度计电源模式配置寄存器
#define ACC_PWR_CTRL        0x7D            //加速度计电源控制寄存器
#define ACC_SOFTRESET       0x7E            //加速度计软件复位寄存器

/*******名称*****************值**************备注*******/
#define ACC_PWR_ON			0X04	        //加速度计开启电源
#define ACC_PWR_OFF			0X00	        //加速度计关闭电源
#define ACC_RST_VAL			0XB6	        //加速度计软件重启
#define ACC_PWR_ACT			0X00	        //加速度计主动模式
#define ACC_RANGE_3			0X00	        //加速度计量程±3g
#define ACC_RANGE_6			0X01	        //加速度计量程±6g
#define ACC_RANGE_12		0X02	        //加速度计量程±12g
#define ACC_RANGE_24		0X03	        //加速度计量程±24g
#define ACC_BW_OSR4 		0X00	        //加速度计4倍带宽
#define ACC_BW_OSR2 		0X01	        //加速度计2倍带宽
#define ACC_BW_NORMAL		0X02	        //加速度计带宽正常模式
#define ACC_ODR_1600HZ		0X0C	        //加速度计滤波

/* 陀螺仪寄存器地址 */
/*******寄存器名称***********寄存器地址*******备注*******/
#define GYRO_CHIP_ID        0x00
#define GYRO_X_LSB          0x02
#define GYRO_X_MSB          0x03
#define GYRO_Y_LSB          0x04
#define GYRO_Y_MSB          0x05
#define GYRO_Z_LSB          0x06
#define GYRO_Z_MSB          0x07
#define GYRO_INT_STATUS_1   0x0A
#define GYRO_FIFO_STATUS    0x0E
#define GYRO_RANGE          0x0F
#define GYRO_BANDWIDTH      0x10
#define GYRO_LPM1           0x11
#define GYRO_SOFTRESET      0x14
#define GYRO_INT_CTRL       0x15
#define GYRO_SELF_TEST      0x3C

/*******名称*****************值**************备注*******/
#define GYRO_RST_VAL		0XB6	        //陀螺仪软件重启
#define GYRO_LPM1_VAL		0X00	        //陀螺仪正常模式
#define GYRO_RANGE_2000		0X00	        //陀螺仪量程±2000°/s
#define GYRO_RANGE_1000		0X01	        //陀螺仪量程±1000°/s
#define GYRO_RANGE_500		0X02	        //陀螺仪量程±500°/s
#define GYRO_RANGE_250		0X03	        //陀螺仪量程±250°/s
#define GYRO_RANGE_125		0X04	        //陀螺仪量程±125°/s
#define GYRO_BW_532			0X00	        //陀螺仪输出频率2000Hz,滤波器带宽532Hz
#define GYRO_BW_230			0X01	        //陀螺仪输出频率2000Hz,滤波器带宽230Hz
#define GYRO_BW_116			0X02	        //陀螺仪输出频率1000Hz,滤波器带宽116Hz
#define GYRO_BW_47			0X03	        //陀螺仪输出频率400Hz,滤波器带宽47Hz
#define	GYRO_BW_23			0X04	        //陀螺仪输出频率200Hz,滤波器带宽23Hz
#define GYRO_BW_12			0X05	        //陀螺仪输出频率100Hz,滤波器带宽12Hz
#define	GYRO_BW_64			0X06	        //陀螺仪输出频率200Hz,滤波器带宽64Hz
#define GYRO_BW_32			0X07	        //陀螺仪输出频率100Hz,滤波器带宽32Hz


/* 芯片ID期望值 */
#define ACC_CHIP_ID_VALUE   0x1E
#define GYRO_CHIP_ID_VALUE  0x0F

/* SPI片选引脚定义 */
#define ACC_CS_PIN          GPIO_PIN_2
#define ACC_CS_PORT         GPIOB
#define GYRO_CS_PIN         GPIO_PIN_10
#define GYRO_CS_PORT        GPIOB

/* 传感器数据结构 */
typedef struct {
    float accel[3];      // 加速度 X, Y, Z (mg)
    float gyro[3];       // 角速度 X, Y, Z (°/s)
    float temperature;   // 温度 (°C)
} bmi088_data_t;

/* 函数声明 */
HAL_StatusTypeDef bmi088_start(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef bmi088_check_ready(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef bmi088_init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef bmi088_read_accel(SPI_HandleTypeDef *hspi, float accel[3]);
HAL_StatusTypeDef bmi088_read_gyro(SPI_HandleTypeDef *hspi, float gyro[3]);
HAL_StatusTypeDef bmi088_read_temperature(SPI_HandleTypeDef *hspi, float *temp);
HAL_StatusTypeDef bmi088_read_all(SPI_HandleTypeDef *hspi, bmi088_data_t *data);

#endif