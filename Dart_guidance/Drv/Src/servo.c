#include "servo.h"
#include "tim.h"
#include "stm32g4xx_hal_tim.h"

static const uint32_t servo_pulse_min_us = SERVO_PULSE_MIN_US;
static const uint32_t servo_pulse_max_us = SERVO_PULSE_MAX_US;
static uint8_t servo_sync_ready = 0U;


static uint32_t Servo_ClampPulseUs(float pulse_us)
{
    if (pulse_us != pulse_us) {
        return SERVO_PULSE_CENTER_US;
    }
    if (pulse_us < 0.0f) {
        return 0U;
    }
    if (pulse_us > (float)servo_pulse_max_us) {
        return servo_pulse_max_us;
    }
    return (uint32_t)(pulse_us + 0.5f);
}

#define SERVO_PULSE_US_TO_TICKS(pulse_us) \
    (Servo_ClampPulseUs((pulse_us)) * SERVO_TIMER_TICKS_PER_US)

static uint32_t AngleToPulseUs(uint8_t angle)
{
    return servo_pulse_min_us +
           (uint32_t)((float)angle * (servo_pulse_max_us - servo_pulse_min_us) / SERVO_ANGLE_MAX);
}

static void Servo_SynchronizeTimers(void)
{
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_SlaveConfigTypeDef slave_config = {0};

    if (servo_sync_ready != 0U) {
        return;
    }

    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &master_config) != HAL_OK) {
        Error_Handler();
    }

    slave_config.SlaveMode = TIM_SLAVEMODE_RESET;
    slave_config.InputTrigger = TIM_TS_ITR5;
    slave_config.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
    slave_config.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    slave_config.TriggerFilter = 0U;
    if (HAL_TIM_SlaveConfigSynchro(&htim4, &slave_config) != HAL_OK) {
        Error_Handler();
    }

    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);

    servo_sync_ready = 1U;
}

void Servo_Init(void)
{
    float center_pulse[SERVO_COUNT] = {
        SERVO_PULSE_CENTER_US,
        SERVO_PULSE_CENTER_US,
        SERVO_PULSE_CENTER_US,
        SERVO_PULSE_CENTER_US
    };

    Servo_SynchronizeTimers();

    // 初始化所有物理输出到当前配置的脉宽。
    Servo_SetPulseUsBatch(center_pulse);

    // 启动定时器PWM输出
    if (HAL_TIM_PWM_Start(&SERVO1_TIM, SERVO1_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&SERVO2_TIM, SERVO2_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&SERVO3_TIM, SERVO3_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&SERVO4_TIM, SERVO4_CHANNEL) != HAL_OK) {
        Error_Handler();
    }

    // 触发一次同步更新，把两个 timer 的 shadow register 一起装载到活动寄存器
    if (HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
        Error_Handler();
    }
}   

// 上层统一传入物理高电平脉宽；TIM8_CHxN 互补端需要换算为反向比较值。
void Servo_SetPulseUs(uint8_t id, float pulse_us)
{
    uint32_t pulse_ticks = SERVO_PULSE_US_TO_TICKS(pulse_us);

    switch (id) {
        case 1:
            __HAL_TIM_SET_COMPARE(&SERVO1_TIM, SERVO1_CHANNEL, pulse_ticks);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&SERVO2_TIM, SERVO2_CHANNEL, SERVO_PWM_PERIOD_TICKS - pulse_ticks);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&SERVO3_TIM, SERVO3_CHANNEL, SERVO_PWM_PERIOD_TICKS - pulse_ticks);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&SERVO4_TIM, SERVO4_CHANNEL, SERVO_PWM_PERIOD_TICKS - pulse_ticks);
            break;
        default:
            break;
    }
}

void Servo_SetAngle(uint8_t id, uint8_t angle)
{
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }
    Servo_SetPulseUs(id, (float)AngleToPulseUs(angle));
}

void Servo_SetPulseUsBatch(const float pulse_us[SERVO_COUNT])
{
    uint32_t pulse_ticks;

    if (pulse_us == NULL) {
        return;
    }

    __HAL_TIM_SET_COMPARE(&SERVO1_TIM, SERVO1_CHANNEL, SERVO_PULSE_US_TO_TICKS(pulse_us[0]));

    pulse_ticks = SERVO_PULSE_US_TO_TICKS(pulse_us[1]);
    __HAL_TIM_SET_COMPARE(&SERVO2_TIM, SERVO2_CHANNEL, SERVO_PWM_PERIOD_TICKS - pulse_ticks);

    pulse_ticks = SERVO_PULSE_US_TO_TICKS(pulse_us[2]);
    __HAL_TIM_SET_COMPARE(&SERVO3_TIM, SERVO3_CHANNEL, SERVO_PWM_PERIOD_TICKS - pulse_ticks);

    pulse_ticks = SERVO_PULSE_US_TO_TICKS(pulse_us[3]);
    __HAL_TIM_SET_COMPARE(&SERVO4_TIM, SERVO4_CHANNEL, SERVO_PWM_PERIOD_TICKS - pulse_ticks);
}

