#include "camera/k230/internal.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mpi_sensor_api.h"

/* SC132GS on the Dart adapter runs the vendor ISP. The sensor driver owns the
 * exposure/gain registers and their timing cache, so this backend goes through
 * its MPI calls rather than raw register writes. Board evidence (CanMV, same
 * kernel driver and ISP profile): the full 1080x1280 120 fps mode, offline ISP,
 * scaled to 480x640 by the channel without a crop window, sustained 119.8 fps.
 * A 1080-wide output with any further DDR load stalled the ISP and wedged
 * stop(), so outputs at the native width are rejected here.
 */
enum {
    SC132GS_SENSOR_WIDTH = 1080u,
    SC132GS_SENSOR_HEIGHT = 1280u,
    SC132GS_SENSOR_FPS = 120u,
    SC132GS_DEFAULT_OUTPUT_WIDTH = 480u,
    SC132GS_DEFAULT_OUTPUT_HEIGHT = 640u,
    /* The ISP writes YUV/RGB rows at a 16-pixel pitch (board-measured). */
    SC132GS_PITCH_ALIGNMENT = 16u,
    SC132GS_MIN_GAIN_MILLI = 1000u,
    SC132GS_MAX_GAIN_MILLI = 28546u, /* Driver analog limit, 28.546875x. */
};

static int sc132gs_matches(const k_vicap_sensor_info* sensor) {
    return sensor->sensor_name != NULL
        && strncmp(sensor->sensor_name, "sc132gs_csi", 11u) == 0;
}

static int sc132gs_configure(dart_camera_t* camera, const k_vicap_sensor_info* sensor) {
    dart_camera_config_t* config = &camera->config;
    if (sensor->data_type != VICAP_CSI_DATA_TYPE_RAW10 || sensor->width != SC132GS_SENSOR_WIDTH
        || sensor->height != SC132GS_SENSOR_HEIGHT
        || config->exposure_mode != DART_CAMERA_EXPOSURE_FIXED || config->mirror != 0u) {
        return DART_CAMERA_E_NOT_SUPPORTED;
    }
    if (config->output_width == 0u && config->output_height == 0u) {
        config->output_width = SC132GS_DEFAULT_OUTPUT_WIDTH;
        config->output_height = SC132GS_DEFAULT_OUTPUT_HEIGHT;
    }
    if (config->output_width >= sensor->width)
        return DART_CAMERA_E_NOT_SUPPORTED;
    if (config->analog_gain_milli != 0u
        && (config->analog_gain_milli < SC132GS_MIN_GAIN_MILLI
            || config->analog_gain_milli > SC132GS_MAX_GAIN_MILLI)) {
        return DART_CAMERA_E_OUT_OF_RANGE;
    }
    const uint32_t mask = SC132GS_PITCH_ALIGNMENT - 1u;
    camera->raw_stride = (config->output_width + mask) & ~mask;
    return DART_CAMERA_OK;
}

static int sc132gs_mode(const dart_camera_t* camera, k_sensor_ae_info* ae) {
    k_sensor_mode mode;
    memset(&mode, 0, sizeof(mode));
    mode.sensor_type = camera->sensor_type;
    const k_s32 rc = kd_mpi_sensor_mode_get(camera->sensor_fd, &mode);
    if (rc != 0)
        return rc;
    if (!(mode.ae_info.one_line_exp_time > 0.0f) || mode.ae_info.max_integraion_line == 0u)
        return DART_CAMERA_E_NOT_SUPPORTED;
    *ae = mode.ae_info;
    return DART_CAMERA_OK;
}

static int sc132gs_exposure_range(
    const dart_camera_t* camera, uint32_t* min_lines, uint32_t* max_lines) {
    k_sensor_ae_info ae;
    const int rc = sc132gs_mode(camera, &ae);
    if (rc != DART_CAMERA_OK)
        return rc;
    *min_lines = 1u;
    *max_lines = ae.max_integraion_line;
    return DART_CAMERA_OK;
}

static int sensor_fault(dart_camera_t* camera, int code) {
    /* The driver writes exposure and gain as one sensor register group; a failed
     * transfer may have reached the sensor. Stop/init re-initializes it. */
    camera->state = DART_CAMERA_STATE_FAULTED;
    camera->exposure_configured = 0;
    return code;
}

static int sc132gs_set_exposure_lines(dart_camera_t* camera, uint32_t lines) {
    k_sensor_ae_info ae;
    int rc = sc132gs_mode(camera, &ae);
    if (rc != DART_CAMERA_OK)
        return rc;
    k_sensor_intg_time time;
    memset(&time, 0, sizeof(time));
    time.intg_time[0] = (float)lines * ae.one_line_exp_time;
    rc = kd_mpi_sensor_intg_time_set(camera->sensor_fd, time);
    if (rc != 0)
        return sensor_fault(camera, rc);

    /* The driver rounds to 1/16 line; a readback outside half a line means the
     * request was clamped or not applied. Frame timing is unchanged. */
    k_sensor_intg_time applied;
    memset(&applied, 0, sizeof(applied));
    rc = kd_mpi_sensor_intg_time_get(camera->sensor_fd, &applied);
    if (rc != 0)
        return sensor_fault(camera, rc);
    const float error = applied.intg_time[0] - time.intg_time[0];
    if (error > ae.one_line_exp_time * 0.5f || error < -ae.one_line_exp_time * 0.5f)
        return sensor_fault(camera, DART_CAMERA_E_EXPOSURE_WRITE_FAILED);
    camera->exposure_configured = 1;
    return DART_CAMERA_OK;
}

static int sc132gs_set_analog_gain(dart_camera_t* camera, uint32_t gain_milli) {
    if (gain_milli < SC132GS_MIN_GAIN_MILLI || gain_milli > SC132GS_MAX_GAIN_MILLI)
        return DART_CAMERA_E_OUT_OF_RANGE;
    k_sensor_gain gain;
    memset(&gain, 0, sizeof(gain));
    gain.gain[0] = (float)gain_milli / 1000.0f;
    const k_s32 rc = kd_mpi_sensor_again_set(camera->sensor_fd, gain);
    return rc == 0 ? DART_CAMERA_OK : sensor_fault(camera, rc);
}

static int sc132gs_convert_gray(
    const dart_camera_t* camera, const uint8_t* luma, size_t luma_bytes,
    uint8_t* gray, size_t gray_bytes) {
    const uint32_t width = camera->config.output_width;
    const uint32_t height = camera->config.output_height;
    const uint32_t pitch = camera->raw_stride;
    if (luma == NULL || gray == NULL || pitch < width
        || luma_bytes < (size_t)pitch * height || gray_bytes < (size_t)width * height) {
        return DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH;
    }
    if (pitch == width) {
        memcpy(gray, luma, (size_t)width * height);
        return DART_CAMERA_OK;
    }
    for (uint32_t y = 0; y < height; ++y)
        memcpy(gray + (size_t)y * width, luma + (size_t)y * pitch, width);
    return DART_CAMERA_OK;
}

const dart_camera_sensor_ops_t dart_camera_sc132gs_ops = {
    .pipeline = DART_CAMERA_PIPELINE_ISP_YUV420,
    .default_width = SC132GS_SENSOR_WIDTH,
    .default_height = SC132GS_SENSOR_HEIGHT,
    .default_fps = SC132GS_SENSOR_FPS,
    .matches = sc132gs_matches,
    .configure = sc132gs_configure,
    .exposure_range = sc132gs_exposure_range,
    .set_exposure_lines = sc132gs_set_exposure_lines,
    .set_analog_gain = sc132gs_set_analog_gain,
    .convert_gray = sc132gs_convert_gray,
};
