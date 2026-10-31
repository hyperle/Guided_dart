#ifndef DART_FIRMWARE_IIM42652_H
#define DART_FIRMWARE_IIM42652_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DART_IIM42652_OK = 0,
    DART_IIM42652_INVALID_ARGUMENT = -1,
    DART_IIM42652_ALREADY_INITIALIZED = -2,
    DART_IIM42652_NOT_INITIALIZED = -3,
    DART_IIM42652_NO_MEMORY = -4,
    DART_IIM42652_BUS_ERROR = -5,
    DART_IIM42652_UNEXPECTED_DEVICE_ID = -6,
    DART_IIM42652_CLOCK_ERROR = -7,
    DART_IIM42652_CONFIGURATION_MISMATCH = -8,
    DART_IIM42652_READY_TIMEOUT = -9,
    /* A sense register returned -32768: that sensor has not produced a sample
     * since the last reset (DS-000440 p62). Not saturation, not a bus error. */
    DART_IIM42652_NO_DATA = -10,
} dart_iim42652_status_t;

/* IIM-42652 DS-000440; other 426xx variants need their own validation. */
#define DART_IMU_DEVICE_ID_IIM42652  0x6fu
const char* dart_iim42652_device_name(uint8_t device_id);

typedef enum {
    DART_IIM42652_ACCEL_16G = 0,
    DART_IIM42652_ACCEL_8G = 1,
    DART_IIM42652_ACCEL_4G = 2,
    DART_IIM42652_ACCEL_2G = 3,
} dart_iim42652_accel_range_t;

typedef enum {
    DART_IIM42652_GYRO_2000DPS = 0,
    DART_IIM42652_GYRO_1000DPS = 1,
    DART_IIM42652_GYRO_500DPS = 2,
    DART_IIM42652_GYRO_250DPS = 3,
    DART_IIM42652_GYRO_125DPS = 4,
    DART_IIM42652_GYRO_62_5DPS = 5,
    DART_IIM42652_GYRO_31_25DPS = 6,
    DART_IIM42652_GYRO_15_625DPS = 7,
} dart_iim42652_gyro_range_t;

typedef struct {
    int bus;
    uint32_t bus_hz;
    uint32_t timeout_ms;
    int chip_select_pin;
    uint8_t accelerometer_range;
    uint8_t gyroscope_range;
    uint8_t output_data_rate;
    float gyroscope_bias_rad_s[3];
    /* Diagnostic overrides: zero selects 0 / 50 / 1000 ms respectively.
     * The readiness timeout starts after the post-enable wait. */
    uint32_t pre_access_delay_ms;
    uint32_t post_enable_delay_ms;
    uint32_t ready_timeout_ms;
    /* Diagnostic override: bypass identity comparisons, retain bus/config checks.
     * Also capture raw registers even if later initialization fails. Default 0. */
    uint8_t skip_device_id_check;
    /* Enable Pin 9 CLKIN and RTC_MODE. Requires an external 31..50 kHz signal;
     * this selects the input, it does not generate a clock. Default 0. */
    uint8_t use_external_clock;
} dart_iim42652_config_t;

enum {
    DART_IIM42652_SAMPLE_ACCEL_SATURATED = 1u << 0,
    DART_IIM42652_SAMPLE_GYRO_SATURATED = 1u << 1,
    /* -32768 in a sense register: no sample since reset (DS-000440 p62). */
    DART_IIM42652_SAMPLE_ACCEL_NO_DATA = 1u << 2,
    DART_IIM42652_SAMPLE_GYRO_NO_DATA = 1u << 3,
    DART_IIM42652_SAMPLE_TEMP_NO_DATA = 1u << 4,
};

typedef struct {
    int16_t accelerometer[3];
    int16_t gyroscope[3];
    float acceleration_m_s2[3];
    float angular_velocity_rad_s[3];
    float temperature_c;
    uint64_t timestamp_ns;
    uint32_t flags;
} dart_iim42652_sample_t;

typedef struct dart_iim42652 dart_iim42652_t;

typedef enum {
    DART_IIM42652_STAGE_BUS = 0,
    DART_IIM42652_STAGE_IDENTIFY,
    DART_IIM42652_STAGE_RESET,
    DART_IIM42652_STAGE_CONFIGURE,
    DART_IIM42652_STAGE_ENABLE,
    DART_IIM42652_STAGE_VERIFY,
    DART_IIM42652_STAGE_WAIT_DATA,
    DART_IIM42652_STAGE_READY,
} dart_iim42652_init_stage_t;
const char* dart_iim42652_stage_name(dart_iim42652_init_stage_t stage);

typedef struct {
    uint8_t device_id_read;
    uint8_t expected_device_id;
    uint8_t observed_device_id;
    uint8_t device_id_check_skipped;
    uint8_t reset_device_id_read;
    uint8_t reset_device_id;
    uint8_t registers_read;
    uint8_t interface_config0;
    uint8_t interface_config1;
    uint8_t power_config;
    uint8_t gyro_config;
    uint8_t accel_config;
    uint8_t interrupt_status;
    uint8_t interrupt_status_seen; /* OR of read-to-clear INT_STATUS observations. */
    uint8_t sensor_config_read;
    uint8_t sensor_config0; /* Bank 1: gyro axis-disable bits 5:3. */
    uint8_t individual_gyroscope_read;
    int16_t individual_gyroscope[3];
    int16_t last_accelerometer[3];
    int16_t last_gyroscope[3];
    uint32_t readiness_polls;
    /* Polls in which INT_STATUS.DATA_RDY was set. A stalled part latches it
     * once; a sampling part sets it on nearly every poll. */
    uint32_t data_ready_polls;
    dart_iim42652_init_stage_t stage;
    dart_iim42652_status_t result;
    /* Absolute CLOCK_MONOTONIC timestamps; zero means the stage was not reached. */
    uint64_t started_ns;
    uint64_t first_access_ns;
    uint64_t reset_ns;
    uint64_t configured_ns;
    uint64_t enabled_ns;
    uint64_t first_read_ns;
    uint64_t ready_ns;
    uint64_t finished_ns;
    uint8_t raw_probe_read;
    uint8_t raw_probe[14]; /* Bank 0 0x1d..0x2a; an unverified register snapshot. */
    dart_iim42652_init_stage_t raw_probe_stage;
    uint8_t external_clock_config_read;
    uint8_t interface_config5; /* Bank 1: PIN9_FUNCTION bits 2:1. */
} dart_iim42652_diagnostic_t;

dart_iim42652_status_t
    dart_iim42652_init(const dart_iim42652_config_t* config, dart_iim42652_t** out_sensor);
/* Optional diagnostics survive failed initialization without a live handle. */
dart_iim42652_status_t dart_iim42652_init_with_diagnostic(
    const dart_iim42652_config_t* config, dart_iim42652_t** out_sensor,
    dart_iim42652_diagnostic_t* diagnostic);
dart_iim42652_status_t
    dart_iim42652_read_sample(dart_iim42652_t* sensor, dart_iim42652_sample_t* sample);
void dart_iim42652_deinit(dart_iim42652_t** sensor);

#ifdef __cplusplus
}
#endif

#endif
