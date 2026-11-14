#include "camera/c_api/camera.h"

#include <stddef.h>
#include <string.h>

void dart_camera_config_init(dart_camera_config_t* config) {
    if (config == NULL)
        return;

    memset(config, 0, sizeof(*config));
    config->abi_version = DART_CAMERA_ABI_VERSION;
    config->struct_size = sizeof(*config);
    config->capture_buffers = 4;
    config->channel_buffers = 4;
    /* Each held lease keeps one grayscale CPU slot busy. */
    config->max_outstanding_frames = 1;
    config->exposure_mode = DART_CAMERA_EXPOSURE_FIXED;
    config->raw10_layout = DART_CAMERA_RAW10_PACKED_LSB;
}

int dart_camera_config_validate(const dart_camera_config_t* config) {
    if (config == NULL)
        return DART_CAMERA_E_INVALID_ARGUMENT;
    if (config->abi_version != DART_CAMERA_ABI_VERSION || config->struct_size != sizeof(*config)) {
        return DART_CAMERA_E_CONFIG_VERSION;
    }

    const int automatic_mode =
        config->sensor_width == 0u && config->sensor_height == 0u && config->sensor_fps == 0u;
    const int automatic_output = config->output_width == 0u && config->output_height == 0u;
    if (config->csi_port > 2u || config->mirror > 3u
        || (!automatic_mode && (config->sensor_width == 0u || config->sensor_height == 0u))
        || config->sensor_width > 3840u || config->sensor_height > 2160u
        || config->sensor_fps > UINT16_MAX
        || (!automatic_output
            && (config->output_width < 320u || config->output_width > 3840u
                || config->output_width % 32u != 0u || config->output_height < 240u
                || config->output_height > 2160u || config->output_height % 2u != 0u))
        || (!automatic_mode && !automatic_output
            && (config->output_width > config->sensor_width
                || config->output_height > config->sensor_height))
        || config->capture_buffers < 3u || config->capture_buffers > DART_CAMERA_MAX_BUFFERS
        || config->channel_buffers < 3u || config->channel_buffers > DART_CAMERA_MAX_BUFFERS
        || config->max_outstanding_frames == 0u
        || config->max_outstanding_frames > DART_CAMERA_MAX_OUTSTANDING_FRAMES
        || config->max_outstanding_frames > config->channel_buffers
        || config->raw10_layout > DART_CAMERA_RAW10_PACKED_LSB
        || config->exposure_mode > DART_CAMERA_EXPOSURE_AUTO
        || (config->exposure_mode == DART_CAMERA_EXPOSURE_AUTO
            && (config->initial_exposure_lines != 0u || config->analog_gain_milli != 0u))) {
        return DART_CAMERA_E_CONFIG_INVALID;
    }
    return DART_CAMERA_OK;
}
