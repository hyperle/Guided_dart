#include "imu.h"
#include "bmi088.h"
#include <math.h>

/* 将毫秒转换为秒 */
#define MS_TO_SEC(x)   ((float)(x) / 1000.0f)

static float imu_wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void imu_reset_reference_window(imu_t *imu)
{
    uint32_t index;

    imu->roll_sum = 0.0f;
    imu->pitch_sum = 0.0f;
    imu->yaw_sum = 0.0f;
    imu->mean_roll = 0.0f;
    imu->mean_pitch = 0.0f;
    imu->mean_yaw = 0.0f;
    imu->window_index = 0U;
    imu->window_count = 0U;

    for (index = 0U; index < IMU_REFERENCE_WINDOW_SIZE; ++index) {
        imu->roll_window[index] = 0.0f;
        imu->pitch_window[index] = 0.0f;
        imu->yaw_window[index] = 0.0f;
    }
}

static void imu_push_reference_sample(imu_t *imu, float roll, float pitch, float yaw)
{
    uint8_t sample_index;
    float divisor;

    sample_index = imu->window_index;
    if (imu->window_count == IMU_REFERENCE_WINDOW_SIZE) {
        imu->roll_sum -= imu->roll_window[sample_index];
        imu->pitch_sum -= imu->pitch_window[sample_index];
        imu->yaw_sum -= imu->yaw_window[sample_index];
    } else {
        imu->window_count++;
    }

    imu->roll_window[sample_index] = roll;
    imu->pitch_window[sample_index] = pitch;
    imu->yaw_window[sample_index] = yaw;
    imu->roll_sum += roll;
    imu->pitch_sum += pitch;
    imu->yaw_sum += yaw;
    imu->window_index = (uint8_t)((sample_index + 1U) % IMU_REFERENCE_WINDOW_SIZE);

    divisor = (imu->window_count > 0U) ? (float)imu->window_count : 1.0f;
    imu->mean_roll = imu->roll_sum / divisor;
    imu->mean_pitch = imu->pitch_sum / divisor;
    imu->mean_yaw = imu->yaw_sum / divisor;
}

static void imu_latch_launch_reference(imu_t *imu)
{
    if (imu->window_count > 0U) {
        imu->launch_ref_roll = imu->mean_roll;
        imu->launch_ref_pitch = imu->mean_pitch;
        imu->launch_ref_yaw = imu->mean_yaw;
    } else {
        imu->launch_ref_roll = imu->roll;
        imu->launch_ref_pitch = imu->pitch;
        imu->launch_ref_yaw = imu->yaw;
    }

    imu->launch_reference_valid = 1U;
}

HAL_StatusTypeDef imu_init(imu_t *imu, SPI_HandleTypeDef *hspi, float Kp, float Ki)
{
    HAL_StatusTypeDef status;

    if (imu == NULL) return HAL_ERROR;

    imu->initialized = 0U;
    imu->hspi = hspi;
    if (hspi == NULL) return HAL_ERROR;

    mahony_init(&imu->mahony, Kp, Ki);

    imu->config.accel_trust_delta_g = IMU_DEFAULT_ACCEL_TRUST_DELTA_G;
    imu->config.accel_pulse_threshold_g = IMU_DEFAULT_ACCEL_PULSE_THRESHOLD_G;
    imu->config.accel_axis_saturation_ratio = IMU_DEFAULT_ACCEL_AXIS_SATURATION_RATIO;
    imu->config.recovery_delay_ms = IMU_DEFAULT_RECOVERY_DELAY_MS;

    imu->last_tick = HAL_GetTick();
    imu->stable_since_tick = imu->last_tick;
    imu->dt = 0.01f;
    imu->roll = 0.0f;
    imu->pitch = 0.0f;
    imu->yaw = 0.0f;
    imu->launch_ref_roll = 0.0f;
    imu->launch_ref_pitch = 0.0f;
    imu->launch_ref_yaw = 0.0f;
    imu->relative_roll = 0.0f;
    imu->relative_pitch = 0.0f;
    imu->relative_yaw = 0.0f;
    imu->accel_is_trusted = 1U;
    imu->high_dynamic_mode = 0U;
    imu->launch_reference_valid = 0U;
    imu->pulse_active = 0U;
    imu->accel_x = 0.0f;
    imu->accel_y = 0.0f;
    imu->accel_z = 0.0f;
    imu->gyro_x = 0.0f;
    imu->gyro_y = 0.0f;
    imu->gyro_z = 0.0f;
    imu_reset_reference_window(imu);

    status = bmi088_start(hspi);
    if (status != HAL_OK) return status;

    status = bmi088_check_ready(hspi);
    if (status != HAL_OK) return status;

    imu->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef imu_update(imu_t *imu)
{
    HAL_StatusTypeDef status;
    uint32_t now_tick;
    float dt;
    float accel_mg[3];
    float gyro_dps[3];
    float ax_g, ay_g, az_g;
    float gx_rad, gy_rad, gz_rad;
    float accel_norm_sq;
    float accel_norm;
    float axis_saturation_limit_g;
    uint8_t axis_near_saturation;
    uint8_t accel_trusted;
    uint8_t pulse_detected;

    if ((imu == NULL) || (imu->initialized == 0U)) return HAL_ERROR;

    now_tick = HAL_GetTick();
    dt = MS_TO_SEC(now_tick - imu->last_tick);
    if (dt > 0.1f) dt = 0.1f;
    if (dt < 0.0001f) dt = 0.0001f;
    imu->dt = dt;
    imu->last_tick = now_tick;

    status = bmi088_read_accel(imu->hspi, accel_mg);
    if (status != HAL_OK) return status;

    status = bmi088_read_gyro(imu->hspi, gyro_dps);
    if (status != HAL_OK) return status;

    imu->gyro_x = gyro_dps[0];
    imu->gyro_y = gyro_dps[1];
    imu->gyro_z = gyro_dps[2];

    ax_g = accel_mg[0] / 1000.0f;
    ay_g = accel_mg[1] / 1000.0f;
    az_g = accel_mg[2] / 1000.0f;

    imu->accel_x = ax_g;
    imu->accel_y = ay_g;
    imu->accel_z = az_g;

    accel_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    accel_norm = (accel_norm_sq > 0.0f) ? sqrtf(accel_norm_sq) : 0.0f;
    axis_saturation_limit_g = IMU_ACCEL_FULL_SCALE_G * imu->config.accel_axis_saturation_ratio;
    axis_near_saturation = 0U;
    if ((fabsf(ax_g) >= axis_saturation_limit_g) ||
        (fabsf(ay_g) >= axis_saturation_limit_g) ||
        (fabsf(az_g) >= axis_saturation_limit_g)) {
        axis_near_saturation = 1U;
    }

    accel_trusted = 1U;
    if ((axis_near_saturation != 0U) ||
        (accel_norm <= 0.0f) ||
        (fabsf(accel_norm - IMU_ACCEL_NORM_TARGET_G) > imu->config.accel_trust_delta_g)) {
        accel_trusted = 0U;
    }

    pulse_detected = 0U;
    if ((axis_near_saturation != 0U) ||
        (accel_norm > 0.0f &&
         (fabsf(accel_norm - IMU_ACCEL_NORM_TARGET_G) >= imu->config.accel_pulse_threshold_g))) {
        pulse_detected = 1U;
    }

    if ((pulse_detected != 0U) && (imu->pulse_active == 0U)) {
        imu_latch_launch_reference(imu);
    }
    imu->pulse_active = pulse_detected;

    gx_rad = gyro_dps[0] * (float)M_PI / 180.0f;
    gy_rad = gyro_dps[1] * (float)M_PI / 180.0f;
    gz_rad = gyro_dps[2] * (float)M_PI / 180.0f;

    imu->accel_is_trusted = accel_trusted;

    if (accel_trusted == 0U) {
        imu->high_dynamic_mode = 1U;
        imu->stable_since_tick = now_tick;
        mahony_reset_feedback(&imu->mahony);
        mahony_update_gyro_only(&imu->mahony, gx_rad, gy_rad, gz_rad, dt);
    } else {
        if (imu->high_dynamic_mode != 0U) {
            if ((uint32_t)(now_tick - imu->stable_since_tick) >= imu->config.recovery_delay_ms) {
                imu->high_dynamic_mode = 0U;
                mahony_reset_feedback(&imu->mahony);
            } else {
                mahony_update_gyro_only(&imu->mahony, gx_rad, gy_rad, gz_rad, dt);
            }
        }

        if (imu->high_dynamic_mode == 0U) {
            mahony_update(&imu->mahony, gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, dt);
        }
    }

    mahony_compute_euler(&imu->mahony);
    imu->roll = imu->mahony.roll;
    imu->pitch = imu->mahony.pitch;
    imu->yaw = imu->mahony.yaw;

    if (imu->launch_reference_valid != 0U) {
        imu->relative_roll = imu_wrap_angle_deg(imu->roll - imu->launch_ref_roll);
        imu->relative_pitch = imu_wrap_angle_deg(imu->pitch - imu->launch_ref_pitch);
        imu->relative_yaw = imu_wrap_angle_deg(imu->yaw - imu->launch_ref_yaw);
    } else {
        imu->relative_roll = 0.0f;
        imu->relative_pitch = 0.0f;
        imu->relative_yaw = 0.0f;
    }

    if ((imu->accel_is_trusted != 0U) && (imu->high_dynamic_mode == 0U)) {
        imu_push_reference_sample(imu, imu->roll, imu->pitch, imu->yaw);
    }

    return HAL_OK;
}

void imu_get_euler(imu_t *imu, float *roll, float *pitch, float *yaw)
{
    if (imu == NULL) return;
    if (roll != NULL) *roll = imu->roll;
    if (pitch != NULL) *pitch = imu->pitch;
    if (yaw != NULL) *yaw = imu->yaw;
}

void imu_get_mean_euler(imu_t *imu, float *roll, float *pitch, float *yaw)
{
    if (imu == NULL) return;
    if (roll != NULL) *roll = imu->mean_roll;
    if (pitch != NULL) *pitch = imu->mean_pitch;
    if (yaw != NULL) *yaw = imu->mean_yaw;
}

void imu_get_launch_reference_euler(imu_t *imu, float *roll, float *pitch, float *yaw)
{
    if (imu == NULL) return;
    if (roll != NULL) *roll = imu->launch_ref_roll;
    if (pitch != NULL) *pitch = imu->launch_ref_pitch;
    if (yaw != NULL) *yaw = imu->launch_ref_yaw;
}

void imu_get_relative_euler(imu_t *imu, float *roll, float *pitch, float *yaw)
{
    if (imu == NULL) return;
    if (roll != NULL) *roll = imu->relative_roll;
    if (pitch != NULL) *pitch = imu->relative_pitch;
    if (yaw != NULL) *yaw = imu->relative_yaw;
}

void imu_get_accel(imu_t *imu, float *ax, float *ay, float *az)
{
    if (imu == NULL) return;
    if (ax != NULL) *ax = imu->accel_x;
    if (ay != NULL) *ay = imu->accel_y;
    if (az != NULL) *az = imu->accel_z;
}

void imu_get_gyro(imu_t *imu, float *gx, float *gy, float *gz)
{
    if (imu == NULL) return;
    if (gx != NULL) *gx = imu->gyro_x;
    if (gy != NULL) *gy = imu->gyro_y;
    if (gz != NULL) *gz = imu->gyro_z;
}
