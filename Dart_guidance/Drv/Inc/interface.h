#ifndef __INTERFACE_H
#define __INTERFACE_H
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t area;
    uint16_t image_width;
    uint16_t image_height;
    bool has_image_size;
} UartReceiverMeasurement_t;

/**
 * @brief 初始化串口接收模块
 * @param huart 已初始化的UART句柄
 */
void uart_receiver_init(UART_HandleTypeDef *huart);

/**
 * @brief 启动串口接收（使能中断）
 */
void uart_receiver_start(void);

/**
 * @brief 获取最新解析到的完整测量数据
 * @param measurement 输出测量数据，可为NULL
 * @return true - 有新数据可用；false - 无新数据
 * @note 当x=0xFFFF且y=0xFFFF时表示未识别到目标
 */
bool uart_receiver_get_measurement(UartReceiverMeasurement_t *measurement);

#endif
