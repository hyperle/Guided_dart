#include "control_mixer.h"

#include "servo.h"
#include <stddef.h>

void ControlMixer_LoadDefaultConfig(ControlMixer_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->base_pulse_us.values[0] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_0;
    config->base_pulse_us.values[1] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_1;
    config->base_pulse_us.values[2] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_2;
    config->base_pulse_us.values[3] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_3;
}

void ControlMixer_ClearContribution(ControlMixer_t *mixer)
{
    if (mixer == NULL) {
        return;
    }

    mixer->contribution.horizontal_pwm_active = false;
    mixer->contribution.horizontal_pwm_us = 0.0f;
}

void ControlMixer_Init(ControlMixer_t *mixer, const ControlMixer_Config_t *config)
{
    if (mixer == NULL) {
        return;
    }

    if (config != NULL) {
        mixer->config = *config;
    } else {
        ControlMixer_LoadDefaultConfig(&mixer->config);
    }

    ControlMixer_ClearContribution(mixer);
    ControlMixer_Solve(mixer);
}

void ControlMixer_SetContribution(ControlMixer_t *mixer,
                                  const GuidanceControlContribution_t *contribution)
{
    if (mixer == NULL) {
        return;
    }

    if (contribution == NULL) {
        ControlMixer_ClearContribution(mixer);
        return;
    }

    mixer->contribution = *contribution;
}

void ControlMixer_Solve(ControlMixer_t *mixer)
{
    float horizontal_us;
    size_t index;

    if (mixer == NULL) {
        return;
    }

    for (index = 0U; index < GUIDANCE_SERVO_COUNT; ++index) {
        mixer->output_pulse_us.values[index] = mixer->config.base_pulse_us.values[index];
    }

    if (mixer->contribution.horizontal_pwm_active) {
        horizontal_us = mixer->contribution.horizontal_pwm_us;
        mixer->output_pulse_us.values[3] += horizontal_us;
        mixer->output_pulse_us.values[0] += horizontal_us;
        mixer->output_pulse_us.values[2] -= horizontal_us;
        mixer->output_pulse_us.values[1] += horizontal_us;
    }

    for (index = 0U; index < GUIDANCE_SERVO_COUNT; ++index) {
        if (mixer->output_pulse_us.values[index] < (float)SERVO_PULSE_MIN_US) {
            mixer->output_pulse_us.values[index] = (float)SERVO_PULSE_MIN_US;
        } else if (mixer->output_pulse_us.values[index] > (float)SERVO_PULSE_MAX_US) {
            mixer->output_pulse_us.values[index] = (float)SERVO_PULSE_MAX_US;
        }
    }
}

void ControlMixer_ApplyOutputs(const ControlMixer_t *mixer)
{
    if (mixer == NULL) {
        return;
    }

    Servo_SetPulseUsBatch(mixer->output_pulse_us.values);
}
