#include "camera/k230/internal.h"

#include <stddef.h>

int dart_camera_get_exposure_range(
    const dart_camera_t* camera, uint32_t* min_lines, uint32_t* max_lines) {
    if (camera == NULL || camera->magic != DART_CAMERA_MAGIC
        || min_lines == NULL || max_lines == NULL)
        return DART_CAMERA_E_INVALID_ARGUMENT;
    /* MCM powers the sensor itself; the ISP pipeline powers it inside VICAP. */
    if (camera->sensor_ops == NULL || camera->sensor_fd < 0
        || !(camera->sensor_powered || camera->stream_started)
        || camera->state == DART_CAMERA_STATE_FAULTED)
        return DART_CAMERA_E_INVALID_STATE;
    if (camera->config.exposure_mode == DART_CAMERA_EXPOSURE_AUTO
        || camera->sensor_ops->exposure_range == NULL)
        return DART_CAMERA_E_NOT_SUPPORTED;
    return camera->sensor_ops->exposure_range(camera, min_lines, max_lines);
}

int dart_camera_set_exposure_lines(dart_camera_t* camera, uint32_t lines) {
    uint32_t min_lines, max_lines;
    const int result = dart_camera_get_exposure_range(camera, &min_lines, &max_lines);
    if (result != DART_CAMERA_OK)
        return result;
    if (lines < min_lines || lines > max_lines)
        return DART_CAMERA_E_OUT_OF_RANGE;
    if (camera->sensor_ops->set_exposure_lines == NULL)
        return DART_CAMERA_E_NOT_SUPPORTED;
    return camera->sensor_ops->set_exposure_lines(camera, lines);
}
