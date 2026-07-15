/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control_mixer.h"
#include "guidance_controller.h"
#include "green_light_task.hpp"
#include "esp32_link.h"
#include "imu.h"
#include "interface.h"
#include "pixel_delta_pid_action.hpp"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GUIDANCE_TARGET_LOST_NO_UPDATE_TICKS 10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 运行参数已回归到各模块 .h 宏定义：green_light_task.hpp / guidance_controller.h / imu.h / esp32_link.h / servo.h。 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static GreenLightTaskProfile_t green_light_task_profile;
static GreenLightTask_t green_light_task;
static Esp32Link_t esp32_link;
static GuidanceTelemetry_t esp32_telemetry;
static GuidanceController_Config_t guidance_controller_config;
static GuidanceController_t guidance_controller;
static ControlMixer_Config_t control_mixer_config;
static ControlMixer_t control_mixer;
static PixelDeltaPwmPidActionProfile_t pixel_delta_pid_profile;
static PixelDeltaPwmPidAction_t pixel_delta_pid_action;
static imu_t guidance_imu = {0};
static uint32_t guidance_last_loop_tick_ms;
static uint8_t guidance_target_no_update_ticks;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t Guidance_IntegerSqrtU32(uint32_t value)
{
    uint32_t result = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0U) {
        if (value >= (result + bit)) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }

    return result;
}

static uint16_t GuidanceTelemetry_EstimateRadiusPx(const GuidanceMeasurement_t *measurement)
{
    uint32_t scaled_area;
    uint32_t radius_px;

    if ((measurement == NULL) ||
        (measurement->x == GUIDANCE_NO_TARGET_COORDINATE) ||
        (measurement->y == GUIDANCE_NO_TARGET_COORDINATE) ||
        (measurement->area == 0U)) {
        return 0U;
    }

    scaled_area = ((uint32_t)measurement->area * 1000U) / 3141U;
    radius_px = Guidance_IntegerSqrtU32(scaled_area);
    if (radius_px == 0U) {
        radius_px = 1U;
    }

    return (radius_px > 0xFFFFU) ? 0xFFFFU : (uint16_t)radius_px;
}

static void Guidance_ApplyMeasurementGeometry(uint16_t width, uint16_t height)
{
    if ((width == 0U) || (height == 0U)) {
        return;
    }

    green_light_task_profile.input.measurement_width = width;
    green_light_task_profile.input.measurement_height = height;

    guidance_controller_config.aim_config.image_width = width;
    guidance_controller_config.aim_config.image_height = height;
    guidance_controller.aim_config.image_width = width;
    guidance_controller.aim_config.image_height = height;
}

static void Guidance_PublishImuMotion(bool imu_data_valid)
{
    static uint8_t motion_pub_divider = 0U;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    uint16_t dart_launch_counter_ticks = 0U;
    float dart_launch_velocity_x_dps = 0.0f;
    float dart_launch_velocity_y_dps = 0.0f;
    float dart_launch_velocity_z_dps = 0.0f;

    if (++motion_pub_divider < ESP32_LINK_IMU_MOTION_PUBLISH_DIVIDER) {
        return;
    }
    motion_pub_divider = 0U;

    if (imu_data_valid) {
        imu_get_gyro(&guidance_imu, &gx, &gy, &gz);
        imu_get_accel(&guidance_imu, &ax, &ay, &az);
        imu_get_dart_launch_velocity_sample(&guidance_imu,
                                            &dart_launch_counter_ticks,
                                            &dart_launch_velocity_x_dps,
                                            &dart_launch_velocity_y_dps,
                                            &dart_launch_velocity_z_dps);
    }
    (void)Esp32Link_PublishImuMotion(&esp32_link,
                                     gx,
                                     gy,
                                     gz,
                                     ax,
                                     ay,
                                     az,
                                     dart_launch_counter_ticks,
                                     dart_launch_velocity_x_dps,
                                     dart_launch_velocity_y_dps,
                                     dart_launch_velocity_z_dps);
}

static bool Guidance_UpdateImuAndPublishMotion(void)
{
    GuidanceRelativeAttitudeError_t relative_attitude_error;

    if (imu_update(&guidance_imu) != HAL_OK) {
        relative_attitude_error.roll_deg = 0.0f;
        relative_attitude_error.pitch_deg = 0.0f;
        (void)GuidanceController_SetRelativeAttitudeError(&guidance_controller,
                                                          &relative_attitude_error);
        Guidance_PublishImuMotion(false);
        return false;
    }

    imu_get_relative_euler(&guidance_imu,
                           &relative_attitude_error.roll_deg,
                           &relative_attitude_error.pitch_deg,
                           NULL);
    (void)GuidanceController_SetRelativeAttitudeError(&guidance_controller,
                                                      &relative_attitude_error);
    Guidance_PublishImuMotion(true);
    return true;
}

static bool Guidance_CaptureStartupImageSize(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_motion_tick_ms = start_tick;
    UartReceiverMeasurement_t measurement;

    while ((uint32_t)(HAL_GetTick() - start_tick) < timeout_ms) {
        uint32_t now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - last_motion_tick_ms) >= GREEN_LIGHT_TASK_LOOP_PERIOD_MS) {
            last_motion_tick_ms = now_ms;
            (void)Guidance_UpdateImuAndPublishMotion();
        }

        if (uart_receiver_get_measurement(&measurement) && measurement.has_image_size) {
            Guidance_ApplyMeasurementGeometry(measurement.image_width, measurement.image_height);
            return true;
        }

        HAL_Delay(GREEN_LIGHT_TASK_IDLE_POLL_DELAY_MS);
    }

    return false;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Esp32Link_Init(&esp32_link, &huart2, ESP32_LINK_TX_TIMEOUT_MS);
  GreenLightTaskProfile_ResetOutput(&green_light_task_profile.output);
  GuidanceTargetSmoother_LoadDefaultConfig(&green_light_task_profile.params.measurement_smoother_config);
  GuidanceController_LoadDefaultConfig(&guidance_controller_config);
  ControlMixer_LoadDefaultConfig(&control_mixer_config);
  PixelDeltaPwmPidActionProfile_LoadDefault(&pixel_delta_pid_profile);
  pixel_delta_pid_profile.params.horizontal_pwm_pid_config =
      guidance_controller_config.horizontal_pwm_pid_config;
  GuidanceController_Init(&guidance_controller, &huart1, &guidance_controller_config);
  ControlMixer_Init(&control_mixer, &control_mixer_config);
  ControlMixer_ApplyOutputs(&control_mixer);
  Guidance_ApplyMeasurementGeometry(GREEN_LIGHT_TASK_DEFAULT_IMAGE_WIDTH,
                                    GREEN_LIGHT_TASK_DEFAULT_IMAGE_HEIGHT);
  (void)imu_init(&guidance_imu, &hspi1, GUIDANCE_IMU_MAHONY_KP, GUIDANCE_IMU_MAHONY_KI);
  (void)Guidance_CaptureStartupImageSize(GREEN_LIGHT_TASK_STARTUP_IMAGE_SIZE_TIMEOUT_MS);
  green_light_task_profile.input.upstream_measurement.x = GUIDANCE_NO_TARGET_COORDINATE;
  green_light_task_profile.input.upstream_measurement.y = GUIDANCE_NO_TARGET_COORDINATE;
  green_light_task_profile.input.upstream_measurement.area = 0U;
  green_light_task_profile.input.upstream_measurement_ready = false;
#if GREEN_LIGHT_TASK_SETPOINT_USE_IMAGE_CENTER
  green_light_task_profile.input.setpoint.x = (uint16_t)(green_light_task_profile.input.measurement_width / 2U);
  green_light_task_profile.input.setpoint.y = (uint16_t)(green_light_task_profile.input.measurement_height / 2U);
#else
  green_light_task_profile.input.setpoint.x = GREEN_LIGHT_TASK_SETPOINT_X;
  green_light_task_profile.input.setpoint.y = GREEN_LIGHT_TASK_SETPOINT_Y;
#endif

  GreenLightTask_Init(&green_light_task, &green_light_task_profile);
  PixelDeltaPwmPidAction_Init(&pixel_delta_pid_action, &pixel_delta_pid_profile);
  guidance_last_loop_tick_ms = HAL_GetTick();
  guidance_target_no_update_ticks = 0U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now_ms;
    bool run_control_tick = false;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    now_ms = HAL_GetTick();
    while ((uint32_t)(now_ms - guidance_last_loop_tick_ms) >= GREEN_LIGHT_TASK_LOOP_PERIOD_MS) {
      guidance_last_loop_tick_ms += GREEN_LIGHT_TASK_LOOP_PERIOD_MS;
      run_control_tick = true;
    }

    if (!run_control_tick) {
      HAL_Delay(GREEN_LIGHT_TASK_IDLE_POLL_DELAY_MS);
      continue;
    }

    {
      TaskActionResult_t task_result;
      bool target_lost = false;

    (void)Guidance_UpdateImuAndPublishMotion();

    green_light_task_profile.input.upstream_measurement_ready =
        uart_receiver_get_data(&green_light_task_profile.input.upstream_measurement.x,
                               &green_light_task_profile.input.upstream_measurement.y,
                               &green_light_task_profile.input.upstream_measurement.area);
    if (!green_light_task_profile.input.upstream_measurement_ready) {
      green_light_task_profile.input.upstream_measurement.x = GUIDANCE_NO_TARGET_COORDINATE;
      green_light_task_profile.input.upstream_measurement.y = GUIDANCE_NO_TARGET_COORDINATE;
      green_light_task_profile.input.upstream_measurement.area = 0U;
    }

    task_result = GreenLightTask_Tick(&green_light_task);
    pixel_delta_pid_profile.params.horizontal_pwm_pid_config =
        guidance_controller_config.horizontal_pwm_pid_config;
    pixel_delta_pid_profile.params.output_limit_us = GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US;

    if ((task_result == TASK_ACTION_SUCCESS) && green_light_task_profile.output.target_detected) {
      TaskActionResult_t control_result;

      guidance_target_no_update_ticks = 0U;
      pixel_delta_pid_profile.input.target_delta = green_light_task_profile.output.delta;
      pixel_delta_pid_profile.input.target_detected = true;

      if (guidance_controller_config.control_mode == GUIDANCE_CONTROL_MODE_HORIZONTAL_PWM_PID) {
        control_result = PixelDeltaPwmPidAction_Tick(&pixel_delta_pid_action);
        if (control_result == TASK_ACTION_FAILURE) {
          PixelDeltaPwmPidAction_Reset(&pixel_delta_pid_action);
          ControlMixer_ClearContribution(&control_mixer);
        } else {
          ControlMixer_SetContribution(&control_mixer,
                                       &pixel_delta_pid_profile.output.contribution);
        }
        ControlMixer_Solve(&control_mixer);
        ControlMixer_ApplyOutputs(&control_mixer);
      } else {
        PixelDeltaPwmPidAction_Reset(&pixel_delta_pid_action);
        ControlMixer_ClearContribution(&control_mixer);
        ControlMixer_Solve(&control_mixer);
        ControlMixer_ApplyOutputs(&control_mixer);
      }
    } else {
      target_lost = (task_result == TASK_ACTION_FAILURE);

      if (task_result == TASK_ACTION_RUNNING) {
        if (guidance_target_no_update_ticks < GUIDANCE_TARGET_LOST_NO_UPDATE_TICKS) {
          guidance_target_no_update_ticks += 1U;
        }
        target_lost = (guidance_target_no_update_ticks >= GUIDANCE_TARGET_LOST_NO_UPDATE_TICKS);
      } else {
        guidance_target_no_update_ticks = GUIDANCE_TARGET_LOST_NO_UPDATE_TICKS;
        target_lost = true;
      }

      if ((guidance_controller_config.control_mode != GUIDANCE_CONTROL_MODE_HORIZONTAL_PWM_PID) ||
          target_lost) {
        PixelDeltaPwmPidAction_Reset(&pixel_delta_pid_action);
        ControlMixer_ClearContribution(&control_mixer);
        ControlMixer_Solve(&control_mixer);
        ControlMixer_ApplyOutputs(&control_mixer);
      }
    }

    if ((task_result != TASK_ACTION_RUNNING) || target_lost) {
      esp32_telemetry.measurement = green_light_task_profile.output.measurement;
      esp32_telemetry.setpoint = green_light_task_profile.input.setpoint;
      esp32_telemetry.delta = green_light_task_profile.output.delta;
      esp32_telemetry.image_width = green_light_task_profile.input.measurement_width;
      esp32_telemetry.image_height = green_light_task_profile.input.measurement_height;
      esp32_telemetry.measurement_radius_px =
          GuidanceTelemetry_EstimateRadiusPx(&green_light_task_profile.output.measurement);
      esp32_telemetry.relative_attitude_error = guidance_controller.relative_attitude_error;
      esp32_telemetry.target_detected = green_light_task_profile.output.target_detected;
      esp32_telemetry.task_finished = true;
      esp32_telemetry.task_success = ((task_result == TASK_ACTION_SUCCESS) && !target_lost);
      (void)Esp32Link_PublishGuidanceTelemetry(&esp32_link, &esp32_telemetry);
    }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC_Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  const float error_servo_pulse_us[SERVO_COUNT] = {
    1700U,
    1700U,
    1700U,
    1700U
  };

  __disable_irq();
  while (1)
  {
    Servo_SetPulseUsBatch(error_servo_pulse_us);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
