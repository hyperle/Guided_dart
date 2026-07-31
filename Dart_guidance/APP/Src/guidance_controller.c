#include "guidance_controller.h"

#include "interface.h"
#include "servo.h"
#include <stddef.h>

static uint16_t GuidanceController_DefaultSetpoint(uint16_t image_size)
{
    return (uint16_t)(image_size / 2U);
}

static uint16_t GuidanceController_NormalizeSetpoint(uint16_t setpoint, uint16_t image_size)
{
    if (image_size == 0U) {
        return 0U;
    }

    if (setpoint == GUIDANCE_AIM_AUTO_SETPOINT_COORDINATE) {
        return GuidanceController_DefaultSetpoint(image_size);
    }

    if (setpoint >= image_size) {
        return (uint16_t)(image_size - 1U);
    }

    return setpoint;
}

static void GuidanceController_LoadDefaultAimConfig(GuidanceAimConfig_t *config)
{
    if (config == NULL) {
        return;
    }

    config->image_width = GUIDANCE_CONTROLLER_DEFAULT_IMAGE_WIDTH;
    config->image_height = GUIDANCE_CONTROLLER_DEFAULT_IMAGE_HEIGHT;
    config->setpoint_x = GUIDANCE_AIM_AUTO_SETPOINT_COORDINATE;
    config->setpoint_y = GUIDANCE_AIM_AUTO_SETPOINT_COORDINATE;
}

static void GuidanceController_InitAimConfig(GuidanceAimConfig_t *target,
                                            const GuidanceAimConfig_t *source)
{
    if ((target == NULL) || (source == NULL)) {
        return;
    }

    *target = *source;
    target->setpoint_x = GuidanceController_NormalizeSetpoint(target->setpoint_x,
                                                              target->image_width);
    target->setpoint_y = GuidanceController_NormalizeSetpoint(target->setpoint_y,
                                                              target->image_height);
}

static void GuidanceController_ClearAimCommand(GuidanceAimCommand_t *aim_command,
                                               float pitch_deg,
                                               float roll_deg)
{
    if (aim_command == NULL) {
        return;
    }

    aim_command->target_detected = false;
    aim_command->pixel_error.x = 0;
    aim_command->pixel_error.y = 0;
    aim_command->pitch_deg = pitch_deg;
    aim_command->roll_deg = roll_deg;
}


static float GuidanceController_GetTurnOutputUs(const GuidanceController_t *controller)
{
    if (controller == NULL) {
        return 0.0f;
    }

    if (controller->control_mode == GUIDANCE_CONTROL_MODE_FIXED_PWM) {
        return 0.0f;
    }

    return controller->horizontal_pid.output;
}

static void GuidanceController_SolveHorizontalPwmPid(GuidanceController_t *controller)
{
    int16_t delta_x;
    float output_us;

    if (controller == NULL) {
        return;
    }

    if ((controller->measurement.x == GUIDANCE_NO_TARGET_COORDINATE) &&
        (controller->measurement.y == GUIDANCE_NO_TARGET_COORDINATE)) {
        PID_Reset(&controller->horizontal_pid);
        GuidanceController_ClearAimCommand(&controller->aim_command,
                                           controller->relative_attitude_error.pitch_deg,
                                           controller->relative_attitude_error.roll_deg);
        Controller_Horizontal_Turn(controller);
        return;
    }

    delta_x = (int16_t)((int32_t)controller->measurement.x - (int32_t)controller->aim_config.setpoint_x);
    output_us = PID_Update(&controller->horizontal_pid, (float)delta_x);
    if (controller->horizontal_pwm_pid_config.invert_output) {
        output_us = -output_us;
    }

    if (output_us > GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US) {
        output_us = GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US;
    } else if (output_us < -GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US) {
        output_us = -GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US;
    }
    controller->horizontal_pid.output = output_us;

    Controller_Horizontal_Turn(controller);

    controller->aim_command.target_detected = true;
    controller->aim_command.pixel_error.x = delta_x;
    controller->aim_command.pitch_deg = controller->relative_attitude_error.pitch_deg;
    controller->aim_command.roll_deg = controller->relative_attitude_error.roll_deg;
}

static void GuidanceController_SolveVerticalPwmPid(GuidanceController_t *controller)
{
    int16_t delta_y;
    float output_us;

    if (controller == NULL) {
        return;
    }

    if ((controller->measurement.x == GUIDANCE_NO_TARGET_COORDINATE) &&
        (controller->measurement.y == GUIDANCE_NO_TARGET_COORDINATE)) {
        PID_Reset(&controller->horizontal_pid);
        GuidanceController_ClearAimCommand(&controller->aim_command,
                                           controller->relative_attitude_error.pitch_deg,
                                           controller->relative_attitude_error.roll_deg);
        Controller_Horizontal_Turn(controller);
        return;
    }

    delta_y = (int16_t)((int32_t)controller->aim_config.setpoint_y - (int32_t)controller->measurement.y);
    output_us = PID_Update(&controller->horizontal_pid, (float)delta_y);
    if (controller->horizontal_pwm_pid_config.invert_output) {
        output_us = -output_us;
    }

    if (output_us > GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US) {
        output_us = GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US;
    } else if (output_us < -GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US) {
        output_us = -GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US;
    }
    controller->horizontal_pid.output = output_us;

    Controller_Vertical_Turn(controller);

    controller->aim_command.target_detected = true;
    controller->aim_command.pixel_error.y = delta_y;
    controller->aim_command.pitch_deg = controller->relative_attitude_error.pitch_deg;
    controller->aim_command.roll_deg = controller->relative_attitude_error.roll_deg;
}

void GuidanceController_LoadDefaultConfig(GuidanceController_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    GuidanceController_LoadDefaultAimConfig(&config->aim_config);
    config->control_mode = GUIDANCE_DEFAULT_CONTROL_MODE;
    config->horizontal_pwm_pid_config.kp = GUIDANCE_HORIZONTAL_PID_KP;
    config->horizontal_pwm_pid_config.ki = GUIDANCE_HORIZONTAL_PID_KI;
    config->horizontal_pwm_pid_config.kd = GUIDANCE_HORIZONTAL_PID_KD;
    config->horizontal_pwm_pid_config.integral_limit = GUIDANCE_HORIZONTAL_PID_INTEGRAL_LIMIT;
    config->horizontal_pwm_pid_config.invert_output = GUIDANCE_HORIZONTAL_PID_INVERT_OUTPUT;
    config->vertical_pwm_pid_config.kp = GUIDANCE_VERTICAL_PID_KP;
    config->vertical_pwm_pid_config.ki = GUIDANCE_VERTICAL_PID_KI;
    config->vertical_pwm_pid_config.kd = GUIDANCE_VERTICAL_PID_KD;
    config->vertical_pwm_pid_config.integral_limit = GUIDANCE_VERTICAL_PID_INTEGRAL_LIMIT;
    config->vertical_pwm_pid_config.invert_output = GUIDANCE_VERTICAL_PID_INVERT_OUTPUT;
}

void GuidanceController_Init(GuidanceController_t *controller,
                             UART_HandleTypeDef *receiver_uart,
                             const GuidanceController_Config_t *config)
{
    if ((controller == NULL) || (receiver_uart == NULL) || (config == NULL)) {
        return;
    }

    controller->measurement.x = GUIDANCE_NO_TARGET_COORDINATE;
    controller->measurement.y = GUIDANCE_NO_TARGET_COORDINATE;
    controller->measurement.area = 0U;
    controller->relative_attitude_error.pitch_deg = 0.0f;
    controller->relative_attitude_error.roll_deg = 0.0f;
    controller->control_mode = config->control_mode;
    controller->horizontal_pwm_pid_config = config->horizontal_pwm_pid_config;
    controller->vertical_pwm_pid_config = config->vertical_pwm_pid_config;

    GuidanceController_InitAimConfig(&controller->aim_config, &config->aim_config);
    GuidanceController_ClearAimCommand(&controller->aim_command, 0.0f, 0.0f);
    PID_Init(&controller->horizontal_pid,
             controller->horizontal_pwm_pid_config.kp,
             controller->horizontal_pwm_pid_config.ki,
             controller->horizontal_pwm_pid_config.kd,
             (float)GUIDANCE_CONTROLLER_LOOP_PERIOD_MS * 0.001f,
             controller->horizontal_pwm_pid_config.integral_limit);
    Controller_Horizontal_Turn(controller);

    Servo_Init();
    GuidanceController_ApplyOutputs(controller);
    uart_receiver_init(receiver_uart);
    uart_receiver_start();
}

bool GuidanceController_FetchMeasurement(GuidanceController_t *controller)
{
    UartReceiverMeasurement_t measurement;

    if (controller == NULL) {
        return false;
    }

    if (!uart_receiver_get_measurement(&measurement)) {
        return false;
    }

    controller->measurement.x = measurement.x;
    controller->measurement.y = measurement.y;
    controller->measurement.area = measurement.area;
    if (measurement.has_image_size) {
        controller->aim_config.image_width = measurement.image_width;
        controller->aim_config.image_height = measurement.image_height;
    }
    return true;
}

bool GuidanceController_SetMeasurement(GuidanceController_t *controller,
                                       const GuidanceMeasurement_t *measurement)
{
    if ((controller == NULL) || (measurement == NULL)) {
        return false;
    }

    controller->measurement = *measurement;
    return true;
}

bool GuidanceController_SetRelativeAttitudeError(GuidanceController_t *controller,
                                                 const GuidanceRelativeAttitudeError_t *error)
{
    if ((controller == NULL) || (error == NULL)) {
        return false;
    }

    controller->relative_attitude_error = *error;
    return true;
}

void GuidanceController_Solve(GuidanceController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    if (controller->control_mode == GUIDANCE_CONTROL_MODE_FIXED_PWM) {
        PID_Reset(&controller->horizontal_pid);
        GuidanceController_ClearAimCommand(&controller->aim_command,
                                           controller->relative_attitude_error.pitch_deg,
                                           controller->relative_attitude_error.roll_deg);
        Controller_Horizontal_Turn(controller);
        return;
    }

    if (controller->control_mode == GUIDANCE_CONTROL_MODE_HORIZONTAL_PWM_PID) {
        GuidanceController_SolveHorizontalPwmPid(controller);
        return;
    }

    PID_Reset(&controller->horizontal_pid);
    GuidanceController_ClearAimCommand(&controller->aim_command,
                                       controller->relative_attitude_error.pitch_deg,
                                       controller->relative_attitude_error.roll_deg);
    Controller_Horizontal_Turn(controller);
}

void GuidanceController_ApplyOutputs(const GuidanceController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    Servo_SetPulseUsBatch(controller->servo_pulse_us.values);
}

void Controller_Horizontal_Turn(GuidanceController_t *controller)
{
    float output_us;

    if (controller == NULL) {
        return;
    }

    output_us = GuidanceController_GetTurnOutputUs(controller);
    controller->servo_pulse_us.values[0] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_0 - output_us;
    controller->servo_pulse_us.values[1] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_1 - output_us;
    controller->servo_pulse_us.values[2] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_2 + output_us;
    controller->servo_pulse_us.values[3] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_3 - output_us;
}


void Controller_Vertical_Turn(GuidanceController_t *controller)
{
    float output_us;

    if (controller == NULL) {
        return;
    }

    output_us = GuidanceController_GetTurnOutputUs(controller);
    controller->servo_pulse_us.values[0] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_0 - output_us;
    controller->servo_pulse_us.values[1] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_1 + output_us;
    controller->servo_pulse_us.values[2] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_2 + output_us;
    controller->servo_pulse_us.values[3] = (float)GUIDANCE_SERVO_INITIAL_PWM_US_3 + output_us;
}

void GuidanceController_RunControl(GuidanceController_t *controller)
{
    if (controller == NULL) {
        return;
    }

    GuidanceController_Solve(controller);
    GuidanceController_ApplyOutputs(controller);
}

void GuidanceController_RunOnce(GuidanceController_t *controller)
{
    if (!GuidanceController_FetchMeasurement(controller)) {
        return;
    }

    GuidanceController_RunControl(controller);
}

