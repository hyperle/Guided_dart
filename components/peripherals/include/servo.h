#pragma once
#include <stdint.h>
typedef struct {
    int channel;
    int pin;
    int fpioa_func;
    uint32_t frequency_hz;
    uint32_t min_pulse_us;
    uint32_t max_pulse_us;
    float min_angle_deg;
    float max_angle_deg;
} servo_config_t;

int  servo_init(const servo_config_t* config);
int  servo_set_angle(float angle_deg);
int  servo_set_pulse_us(uint32_t pulse_us);
int  servo_stop(void);
void servo_deinit(void);
