#include "esp32_link.h"

static int16_t Esp32Link_EncodeCentideg(float angle_deg)
{
    float scaled;

    scaled = angle_deg * 100.0f;
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    if (scaled >= 0.0f) {
        return (int16_t)(scaled + 0.5f);
    }

    return (int16_t)(scaled - 0.5f);
}

#define ESP32_LINK_GUIDANCE_PAYLOAD_SIZE 28U
#define ESP32_LINK_IMU_MOTION_PAYLOAD_SIZE 24U
#define ESP32_LINK_GUIDANCE_FRAME_SIZE (4U + ESP32_LINK_GUIDANCE_PAYLOAD_SIZE + 1U)
#define ESP32_LINK_FLAG_TARGET_DETECTED (1U << 0U)
#define ESP32_LINK_FLAG_TASK_FINISHED (1U << 1U)
#define ESP32_LINK_FLAG_TASK_SUCCESS (1U << 2U)
#define ESP32_LINK_GUIDANCE_TELEMETRY_VERSION 3U

void Esp32Link_Init(Esp32Link_t *link, UART_HandleTypeDef *uart, uint32_t tx_timeout_ms)
{
    if (link == NULL) {
        return;
    }

    link->uart = uart;
    link->tx_sequence = 0U;
    link->tx_timeout_ms = tx_timeout_ms;
}

bool Esp32Link_PublishGuidanceTelemetry(Esp32Link_t *link, const GuidanceTelemetry_t *telemetry)
{
    uint8_t frame[ESP32_LINK_GUIDANCE_FRAME_SIZE];
    uint8_t checksum = 0U;
    uint8_t flags = 0U;
    uint16_t sequence;
    int16_t relative_pitch_cd;
    int16_t relative_roll_cd;
    size_t index;
    HAL_StatusTypeDef status;

    if ((link == NULL) || (link->uart == NULL) || (telemetry == NULL)) {
        return false;
    }

    if (telemetry->target_detected) {
        flags |= ESP32_LINK_FLAG_TARGET_DETECTED;
    }
    if (telemetry->task_finished) {
        flags |= ESP32_LINK_FLAG_TASK_FINISHED;
    }
    if (telemetry->task_success) {
        flags |= ESP32_LINK_FLAG_TASK_SUCCESS;
    }

    sequence = link->tx_sequence;
    relative_pitch_cd = Esp32Link_EncodeCentideg(telemetry->relative_attitude_error.pitch_deg);
    relative_roll_cd = Esp32Link_EncodeCentideg(telemetry->relative_attitude_error.roll_deg);

    frame[0] = GUIDANCE_PROTOCOL_FRAME_HEADER_0;
    frame[1] = GUIDANCE_PROTOCOL_FRAME_HEADER_1;
    frame[2] = ESP32_LINK_MESSAGE_TYPE_GUIDANCE_TELEMETRY;
    frame[3] = ESP32_LINK_GUIDANCE_PAYLOAD_SIZE;
    frame[4] = (uint8_t)(sequence & 0xFFU);
    frame[5] = (uint8_t)((sequence >> 8U) & 0xFFU);
    frame[6] = flags;
    frame[7] = ESP32_LINK_GUIDANCE_TELEMETRY_VERSION;
    frame[8] = (uint8_t)(telemetry->measurement.x & 0xFFU);
    frame[9] = (uint8_t)((telemetry->measurement.x >> 8U) & 0xFFU);
    frame[10] = (uint8_t)(telemetry->measurement.y & 0xFFU);
    frame[11] = (uint8_t)((telemetry->measurement.y >> 8U) & 0xFFU);
    frame[12] = (uint8_t)(telemetry->measurement.area & 0xFFU);
    frame[13] = (uint8_t)((telemetry->measurement.area >> 8U) & 0xFFU);
    frame[14] = (uint8_t)((uint16_t)telemetry->delta.delta_x & 0xFFU);
    frame[15] = (uint8_t)(((uint16_t)telemetry->delta.delta_x >> 8U) & 0xFFU);
    frame[16] = (uint8_t)((uint16_t)telemetry->delta.delta_y & 0xFFU);
    frame[17] = (uint8_t)(((uint16_t)telemetry->delta.delta_y >> 8U) & 0xFFU);
    frame[18] = (uint8_t)(telemetry->setpoint.x & 0xFFU);
    frame[19] = (uint8_t)((telemetry->setpoint.x >> 8U) & 0xFFU);
    frame[20] = (uint8_t)(telemetry->setpoint.y & 0xFFU);
    frame[21] = (uint8_t)((telemetry->setpoint.y >> 8U) & 0xFFU);
    frame[22] = (uint8_t)(telemetry->image_width & 0xFFU);
    frame[23] = (uint8_t)((telemetry->image_width >> 8U) & 0xFFU);
    frame[24] = (uint8_t)(telemetry->image_height & 0xFFU);
    frame[25] = (uint8_t)((telemetry->image_height >> 8U) & 0xFFU);
    frame[26] = (uint8_t)(telemetry->measurement_radius_px & 0xFFU);
    frame[27] = (uint8_t)((telemetry->measurement_radius_px >> 8U) & 0xFFU);
    frame[28] = (uint8_t)((uint16_t)relative_pitch_cd & 0xFFU);
    frame[29] = (uint8_t)(((uint16_t)relative_pitch_cd >> 8U) & 0xFFU);
    frame[30] = (uint8_t)((uint16_t)relative_roll_cd & 0xFFU);
    frame[31] = (uint8_t)(((uint16_t)relative_roll_cd >> 8U) & 0xFFU);

    for (index = 0U; index < (ESP32_LINK_GUIDANCE_FRAME_SIZE - 1U); ++index) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    frame[ESP32_LINK_GUIDANCE_FRAME_SIZE - 1U] = checksum;

    status = HAL_UART_Transmit(link->uart,
                               frame,
                               (uint16_t)ESP32_LINK_GUIDANCE_FRAME_SIZE,
                               link->tx_timeout_ms);
    if (status != HAL_OK) {
        return false;
    }

    link->tx_sequence = (uint16_t)(sequence + 1U);
    return true;
}

bool Esp32Link_PublishImuMotion(Esp32Link_t *link,
                                float gx,
                                float gy,
                                float gz,
                                float ax,
                                float ay,
                                float az)
{
    uint8_t frame[4U + ESP32_LINK_IMU_MOTION_PAYLOAD_SIZE + 1U];
    uint8_t checksum = 0U;
    size_t index;
    float motion_values[6];
    uint8_t *payload_ptr;
    HAL_StatusTypeDef status;

    if ((link == NULL) || (link->uart == NULL)) {
        return false;
    }

    frame[0] = GUIDANCE_PROTOCOL_FRAME_HEADER_0;
    frame[1] = GUIDANCE_PROTOCOL_FRAME_HEADER_1;
    frame[2] = ESP32_LINK_MESSAGE_TYPE_IMU_MOTION;
    frame[3] = ESP32_LINK_IMU_MOTION_PAYLOAD_SIZE;

    motion_values[0] = gx;
    motion_values[1] = gy;
    motion_values[2] = gz;
    motion_values[3] = ax;
    motion_values[4] = ay;
    motion_values[5] = az;
    payload_ptr = (uint8_t *)motion_values;
    for (index = 0U; index < ESP32_LINK_IMU_MOTION_PAYLOAD_SIZE; ++index) {
        frame[4U + index] = payload_ptr[index];
    }

    for (index = 0U; index < (4U + ESP32_LINK_IMU_MOTION_PAYLOAD_SIZE); ++index) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    frame[4U + ESP32_LINK_IMU_MOTION_PAYLOAD_SIZE] = checksum;

    status = HAL_UART_Transmit(link->uart,
                               frame,
                               (uint16_t)(4U + ESP32_LINK_IMU_MOTION_PAYLOAD_SIZE + 1U),
                               link->tx_timeout_ms);
    return status == HAL_OK;
}
