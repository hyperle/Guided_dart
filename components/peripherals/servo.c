#include "servo.h"

#include "drv_fpioa.h"
#include "drv_pwm.h"

static servo_config_t g_config;
static int g_ready;

static uint32_t clamp_pulse(uint32_t pulse_us)
{
    if (pulse_us < g_config.min_pulse_us) {
        return g_config.min_pulse_us;
    }
    if (pulse_us > g_config.max_pulse_us) {
        return g_config.max_pulse_us;
    }
    return pulse_us;
}

int servo_init(const servo_config_t *config)
{
    if (!config || config->channel < 0 || config->channel > 5 ||
        config->frequency_hz == 0 ||
        config->min_pulse_us >= config->max_pulse_us ||
        config->min_angle_deg >= config->max_angle_deg) {
        return -1;
    }

    if (drv_fpioa_set_pin_func(config->pin, config->fpioa_func) != 0 ||
        drv_pwm_init() != 0 ||
        drv_pwm_set_freq(config->channel, config->frequency_hz) != 0) {
        return -1;
    }

    g_config = *config;
    g_ready = 1;
    if (drv_pwm_enable(g_config.channel) != 0) {
        g_ready = 0;
        drv_pwm_deinit();
        return -1;
    }
    return servo_set_pulse_us(g_config.min_pulse_us);
}

int servo_set_pulse_us(uint32_t pulse_us)
{
    if (!g_ready) {
        return -1;
    }
    return drv_pwm_set_duty_ns(g_config.channel,
                               clamp_pulse(pulse_us) * 1000U);
}

int servo_set_angle(float angle_deg)
{
    if (!g_ready || angle_deg < g_config.min_angle_deg ||
        angle_deg > g_config.max_angle_deg) {
        return -1;
    }

    float ratio = (angle_deg - g_config.min_angle_deg) /
                  (g_config.max_angle_deg - g_config.min_angle_deg);
    uint32_t pulse_us = g_config.min_pulse_us +
                        (uint32_t)(ratio * (g_config.max_pulse_us -
                                            g_config.min_pulse_us));
    return servo_set_pulse_us(pulse_us);
}

int servo_stop(void)
{
    if (!g_ready) {
        return -1;
    }
    return drv_pwm_disable(g_config.channel);
}

void servo_deinit(void)
{
    if (!g_ready) {
        return;
    }

    drv_pwm_disable(g_config.channel);
    drv_pwm_deinit();
    g_ready = 0;
}
