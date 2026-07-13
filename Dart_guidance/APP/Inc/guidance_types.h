#ifndef __GUIDANCE_TYPES_H
#define __GUIDANCE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define GUIDANCE_NO_TARGET_COORDINATE 0xFFFFU
#define GUIDANCE_SERVO_COUNT 4U

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t area;
} GuidanceMeasurement_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
} GuidanceSetpoint_t;

typedef struct
{
    int16_t x;
    int16_t y;
} GuidancePixelError_t;

typedef struct
{
    int16_t delta_x;
    int16_t delta_y;
} GuidanceDelta_t;

typedef struct
{
    float pitch_deg;
    float roll_deg;
} GuidanceRelativeAttitudeError_t;

typedef struct
{
    GuidanceMeasurement_t measurement;
    GuidanceSetpoint_t setpoint;
    GuidanceDelta_t delta;
    uint16_t image_width;
    uint16_t image_height;
    uint16_t measurement_radius_px;
    GuidanceRelativeAttitudeError_t relative_attitude_error;
    bool target_detected;
    bool task_finished;
    bool task_success;
} GuidanceTelemetry_t;

typedef struct
{
    bool target_detected;
    GuidancePixelError_t pixel_error;
    float pitch_deg;
    float roll_deg;
} GuidanceAimCommand_t;

typedef struct
{
    float values[GUIDANCE_SERVO_COUNT];
} GuidanceServoPulseUs_t;

#endif
