#include "camera/k230/internal.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "camera/c_api/raw10.h"
#include "mpi_sensor_api.h"

/* SC038HGS timing policy belongs to this backend. No Pixel CLK is needed
 * when exposure is expressed in lines. The SDK's seconds/fps controls are
 * deliberately unused: raw register writes do not update its timing cache.
 */
enum {
    SC038HGS_MIN_EXPOSURE_LINES = 4u,
    SC038HGS_MIN_FRAME_LINES = 600u,
    SC038HGS_MAX_FRAME_LINES = 0xfff0u,
    SC038HGS_EXPOSURE_MARGIN = 11u,
};

static int sc038hgs_matches(const k_vicap_sensor_info* sensor) {
    return sensor->sensor_name != NULL
        && strncmp(sensor->sensor_name, "sc038hgs_csi", 11u) == 0;
}

static int sc038hgs_configure(dart_camera_t* camera, const k_vicap_sensor_info* sensor) {
    dart_camera_config_t* config = &camera->config;
    /* The RAW path has no scaler: the default output is the native mode. */
    if (config->output_width == 0u && config->output_height == 0u) {
        config->output_width = sensor->width;
        config->output_height = sensor->height;
    }
    if (config->analog_gain_milli != 0u || sensor->data_type != VICAP_CSI_DATA_TYPE_RAW10
        || sensor->width != 640u || sensor->height != 480u || config->output_width != sensor->width
        || config->output_height != sensor->height
        || config->exposure_mode != DART_CAMERA_EXPOSURE_FIXED
        || config->raw10_layout != DART_CAMERA_RAW10_PACKED_LSB) {
        return DART_CAMERA_E_NOT_SUPPORTED;
    }
    /* Audited MCM RAW10 has compact 4-pixel / 5-byte rows. The DMA capacity
     * span is deliberately separate and remains owned by the MCM adapter.
     */
    camera->raw_stride = sensor->width / 4u * 5u;
    return DART_CAMERA_OK;
}

static int sc038hgs_exposure_range(
    const dart_camera_t* camera, uint32_t* min_lines, uint32_t* max_lines) {
    (void)camera;
    *min_lines = SC038HGS_MIN_EXPOSURE_LINES;
    *max_lines = SC038HGS_MAX_FRAME_LINES - SC038HGS_EXPOSURE_MARGIN;
    return DART_CAMERA_OK;
}

static int exposure_fault(dart_camera_t* camera, int code) {
    /* A failed I2C transfer may still have reached the sensor. Stop/init must
     * reset any incomplete group before another exposure or frame is used. */
    camera->state = DART_CAMERA_STATE_FAULTED;
    camera->exposure_configured = 0;
    return code;
}

static int sc038hgs_set_exposure_lines(dart_camera_t* camera, uint32_t lines) {
    int rc;
    uint32_t frame_lines = lines + SC038HGS_EXPOSURE_MARGIN;
    if (frame_lines < SC038HGS_MIN_FRAME_LINES)
        frame_lines = SC038HGS_MIN_FRAME_LINES;
    const uint32_t raw = lines * 16u;
    k_u32 high = 0;
    rc = kd_mpi_sensor_reg_read(camera->sensor_fd, 0x3e00, &high);
    if (rc != 0)
        return rc;
    const struct {
        k_u32 address;
        k_u32 value;
    } registers[] = {
        {0x320e, frame_lines >> 8},
        {0x320f, frame_lines & 0xffu},
        {0x3e00, (high & 0xf0u) | ((raw >> 16) & 0x0fu)},
        {0x3e01, (raw >> 8) & 0xffu},
        {0x3e02, raw & 0xffu},
    };

    /* Group 0 has capacity for eight registers; exposure and frame length
     * occupy five and must be launched together (SC038HGS manual p. 29). */
    rc = kd_mpi_sensor_reg_write(camera->sensor_fd, 0x3800, 0x00);
    if (rc != 0)
        return exposure_fault(camera, rc);
    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); ++i) {
        rc = kd_mpi_sensor_reg_write(camera->sensor_fd, registers[i].address, registers[i].value);
        if (rc != 0)
            break;
    }
    const int end_rc = kd_mpi_sensor_reg_write(camera->sensor_fd, 0x3800, 0x10);
    if (rc != 0 || end_rc != 0)
        return exposure_fault(camera, rc != 0 ? rc : end_rc);
    rc = kd_mpi_sensor_reg_write(camera->sensor_fd, 0x3800, 0x60);
    if (rc != 0)
        return exposure_fault(camera, rc);

    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); ++i) {
        k_u32 value = 0;
        rc = kd_mpi_sensor_reg_read(camera->sensor_fd, registers[i].address, &value);
        if (rc != 0)
            return exposure_fault(camera, rc);
        if (value != registers[i].value)
            return exposure_fault(camera, DART_CAMERA_E_EXPOSURE_WRITE_FAILED);
    }
    /* Frame timing changed; do not publish the mode table's old rate. */
    camera->caps.sensor_fps = 0;
    camera->exposure_configured = 1;
    return DART_CAMERA_OK;
}

static int sc038hgs_convert_gray(
    const dart_camera_t* camera, const uint8_t* raw, size_t raw_bytes,
    uint8_t* gray, size_t gray_bytes) {
    return dart_camera_raw10_gray(
        raw, raw_bytes, camera->config.output_width, camera->config.output_height,
        camera->raw_stride, camera->config.raw10_layout, camera->config.mirror,
        gray, gray_bytes);
}

const dart_camera_sensor_ops_t dart_camera_sc038hgs_ops = {
    .pipeline = DART_CAMERA_PIPELINE_MCM_RAW10,
    .matches = sc038hgs_matches,
    .configure = sc038hgs_configure,
    .exposure_range = sc038hgs_exposure_range,
    .set_exposure_lines = sc038hgs_set_exposure_lines,
    .set_analog_gain = NULL,
    .convert_gray = sc038hgs_convert_gray,
};
