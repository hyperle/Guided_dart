#ifndef __ESP32_LINK_H
#define __ESP32_LINK_H

#include "guidance_protocol_generated.h"
#include "guidance_types.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* UART 发送到 ESP32 的超时时间，单位 ms；过短可能导致繁忙时 telemetry 丢包。 */
#define ESP32_LINK_TX_TIMEOUT_MS 5U

/* IMU motion 上行分频；每 N 个 10 ms 主循环 tick 发布一次 0x05 motion 帧。 */
#define ESP32_LINK_IMU_MOTION_PUBLISH_DIVIDER 4U

typedef struct
{
    UART_HandleTypeDef *uart;
    uint16_t tx_sequence;
    uint32_t tx_timeout_ms;
} Esp32Link_t;

void Esp32Link_Init(Esp32Link_t *link, UART_HandleTypeDef *uart, uint32_t tx_timeout_ms);
bool Esp32Link_PublishGuidanceTelemetry(Esp32Link_t *link, const GuidanceTelemetry_t *telemetry);
bool Esp32Link_PublishImuMotion(Esp32Link_t *link,
                                float gx,
                                float gy,
                                float gz,
                                float ax,
                                float ay,
                                float az);

#endif
