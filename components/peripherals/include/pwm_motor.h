#pragma once
#include <stdint.h>
typedef struct {
    int channel;
    int pin;
    int fpioa_func;
    uint32_t frequency_hz;
    int active_high;
} pwm_motor_config_t;

int  pwm_motor_init(const pwm_motor_config_t* config);
int  pwm_motor_set_percent(int percent);
int  pwm_motor_stop(void);
void pwm_motor_deinit(void);
