#include "interface.h"


#define UART_RECEIVER_MEASUREMENT_HEADER 0x5AU
#define UART_RECEIVER_MEASUREMENT_LEGACY_LENGTH 0x06U
#define UART_RECEIVER_MEASUREMENT_EXTENDED_LENGTH 0x0AU
#define UART_RECEIVER_MEASUREMENT_MAX_LENGTH UART_RECEIVER_MEASUREMENT_EXTENDED_LENGTH
// 状态机状态
typedef enum {
    STATE_IDLE,      // 等待帧头 0x5A
    STATE_LEN,       // 等待长度字节
    STATE_DATA,      // 接收测量数据
    STATE_CHECKSUM   // 接收校验和
} rx_state_t;

// 私有变量（volatile 因在中断和主循环间共享）
static UART_HandleTypeDef *uart_handle = NULL;          // UART句柄
static volatile rx_state_t state = STATE_IDLE;          // 当前状态
static volatile uint8_t data_buffer[UART_RECEIVER_MEASUREMENT_MAX_LENGTH];
static volatile uint8_t data_index = 0;                 // 数据缓冲区索引
static volatile uint8_t data_length = 0;                // 当前帧payload长度
static volatile UartReceiverMeasurement_t last_measurement;
static volatile uint8_t data_ready = 0;                 // 新数据标志
static volatile uint8_t rx_byte;                        // 单字节接收缓冲区

// 内部函数
static void parse_byte(uint8_t byte);
static void reset_state(void);
static bool measurement_length_is_valid(uint8_t length);
static uint16_t read_be16_volatile(const volatile uint8_t *data);

//------------------------------------------------------------------------------
void uart_receiver_init(UART_HandleTypeDef *huart) {
    uart_handle = huart;
    reset_state();
    last_measurement.x = 0;
    last_measurement.y = 0;
    last_measurement.area = 0;
    last_measurement.image_width = 0;
    last_measurement.image_height = 0;
    last_measurement.has_image_size = false;
    data_ready = 0;
    rx_byte = 0;
}

//------------------------------------------------------------------------------
void uart_receiver_start(void) {
    if (uart_handle != NULL) {
        // 启动第一次接收（接收一个字节）
        HAL_UART_Receive_IT(uart_handle, (uint8_t*)&rx_byte, 1);
    }
}

//------------------------------------------------------------------------------
// 状态机解析字节
static void parse_byte(uint8_t byte) {
    switch (state) {
        case STATE_IDLE:
            if (byte == UART_RECEIVER_MEASUREMENT_HEADER) {          // 检测到帧头
                state = STATE_LEN;
            }
            break;

        case STATE_LEN:
            if (measurement_length_is_valid(byte)) {
                data_length = byte;
                data_index = 0;
                state = STATE_DATA;
            } else {
                state = (byte == UART_RECEIVER_MEASUREMENT_HEADER) ? STATE_LEN : STATE_IDLE;
            }
            break;

        case STATE_DATA:
            if ((data_index < data_length) && (data_index < sizeof(data_buffer))) {
                data_buffer[data_index++] = byte;
                if (data_index >= data_length) {
                    state = STATE_CHECKSUM;
                }
            } else {
                reset_state();
            }
            break;

        case STATE_CHECKSUM:
        {
            uint8_t checksum = (uint8_t)(UART_RECEIVER_MEASUREMENT_HEADER + data_length);
            uint8_t index;

            for (index = 0U; index < data_length; ++index) {
                checksum = (uint8_t)(checksum + data_buffer[index]);
            }

            if (checksum == byte) {
                last_measurement.x = read_be16_volatile(&data_buffer[0]);
                last_measurement.y = read_be16_volatile(&data_buffer[2]);
                last_measurement.area = read_be16_volatile(&data_buffer[4]);
                if (data_length >= UART_RECEIVER_MEASUREMENT_EXTENDED_LENGTH) {
                    last_measurement.image_width = read_be16_volatile(&data_buffer[6]);
                    last_measurement.image_height = read_be16_volatile(&data_buffer[8]);
                    last_measurement.has_image_size = (last_measurement.image_width > 0U) &&
                                                      (last_measurement.image_height > 0U);
                } else {
                    last_measurement.image_width = 0;
                    last_measurement.image_height = 0;
                    last_measurement.has_image_size = false;
                }
                data_ready = 1;            // 标记新数据可用
            }
            // 无论校验成功与否，回到空闲状态准备下一个包
            state = STATE_IDLE;
            break;
        }

        default:
            reset_state();
            break;
    }
}

static bool measurement_length_is_valid(uint8_t length)
{
    return (length == UART_RECEIVER_MEASUREMENT_LEGACY_LENGTH) ||
           (length == UART_RECEIVER_MEASUREMENT_EXTENDED_LENGTH);
}

static uint16_t read_be16_volatile(const volatile uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

//------------------------------------------------------------------------------
// 重置状态机（可留作备用，当前未使用）
static void reset_state(void) {
    state = STATE_IDLE;
    data_index = 0;
    data_length = 0;
}

//------------------------------------------------------------------------------
bool uart_receiver_get_measurement(UartReceiverMeasurement_t *measurement) {
    bool ret = false;
    UartReceiverMeasurement_t snapshot;

    // 进入临界区，防止中断更新 data_ready 和 last_measurement
    __disable_irq();
    if (data_ready) {
        snapshot.x = last_measurement.x;
        snapshot.y = last_measurement.y;
        snapshot.area = last_measurement.area;
        snapshot.image_width = last_measurement.image_width;
        snapshot.image_height = last_measurement.image_height;
        snapshot.has_image_size = last_measurement.has_image_size;
        data_ready = 0;   // 清除标志
        ret = true;
    }
    __enable_irq();

    if (ret && (measurement != NULL)) {
        *measurement = snapshot;
    }
    return ret;
}

bool uart_receiver_get_data(uint16_t *x, uint16_t *y, uint16_t *area) {
    UartReceiverMeasurement_t measurement;

    if (!uart_receiver_get_measurement(&measurement)) {
        return false;
    }

    if (x != NULL) *x = measurement.x;
    if (y != NULL) *y = measurement.y;
    if (area != NULL) *area = measurement.area;
    return true;
}

//------------------------------------------------------------------------------
// HAL库UART接收完成回调（在中断中执行）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_handle) {
        // 解析刚收到的字节
        parse_byte(rx_byte);
        // 继续接收下一个字节
        HAL_UART_Receive_IT(uart_handle, (uint8_t*)&rx_byte, 1);
        return;
    }
}
