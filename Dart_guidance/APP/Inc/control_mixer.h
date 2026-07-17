#ifndef DART_GUIDANCE_CONTROL_MIXER_H
#define DART_GUIDANCE_CONTROL_MIXER_H

#include "guidance_controller.h"
#include "guidance_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    GuidanceServoPulseUs_t base_pulse_us;
} ControlMixer_Config_t;

typedef struct
{
    ControlMixer_Config_t config;
    GuidanceControlContribution_t contribution;
    GuidanceServoPulseUs_t output_pulse_us;
} ControlMixer_t;

void ControlMixer_LoadDefaultConfig(ControlMixer_Config_t *config);
void ControlMixer_Init(ControlMixer_t *mixer, const ControlMixer_Config_t *config);
void ControlMixer_ClearContribution(ControlMixer_t *mixer);
void ControlMixer_SetContribution(ControlMixer_t *mixer,
                                  const GuidanceControlContribution_t *contribution);
void ControlMixer_Solve(ControlMixer_t *mixer);
void ControlMixer_ApplyOutputs(const ControlMixer_t *mixer);

#ifdef __cplusplus
}
#endif

#endif
