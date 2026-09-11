#include "pwm_motor.h"

#include "drv_fpioa.h"
#include "drv_pwm.h"

static pwm_motor_config_t g_config;
static int g_ready;

int pwm_motor_init(const pwm_motor_config_t *config)
{
    if (!config || config->channel < 0 || config->channel > 5 ||
        config->frequency_hz == 0 || config->pin < 0) {
        return -1;
    }

    if (drv_fpioa_set_pin_func(config->pin, config->fpioa_func) != 0 ||
        drv_pwm_init() != 0 ||
        drv_pwm_set_freq(config->channel, config->frequency_hz) != 0) {
        return -1;
    }

    g_config = *config;
    g_ready = 1;
    return pwm_motor_stop();
}

int pwm_motor_set_percent(int percent)
{
    if (!g_ready || percent < 0 || percent > 100) {
        return -1;
    }

    if (!g_config.active_high) {
        percent = 100 - percent;
    }

    return drv_pwm_set_duty(g_config.channel, (uint32_t)percent);
}

int pwm_motor_stop(void)
{
    if (!g_ready) {
        return -1;
    }
    return drv_pwm_set_duty(g_config.channel, 0);
}

void pwm_motor_deinit(void)
{
    if (!g_ready) {
        return;
    }

    drv_pwm_disable(g_config.channel);
    drv_pwm_deinit();
    g_ready = 0;
}
