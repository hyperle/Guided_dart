#ifndef DART_CAMERA_K230_SENSOR_H
#define DART_CAMERA_K230_SENSOR_H

#include <stddef.h>
#include <stdint.h>

#include "camera/c_api/camera.h"
#include "k_vicap_comm.h"

/* How a sensor's pixels reach DDR. MCM bypasses the ISP and the sensor ops
 * convert RAW10 themselves; ISP runs the vendor pipeline and the channel
 * delivers YUV420SP, whose luma plane is the Y8 frame.
 */
typedef enum dart_camera_pipeline {
    DART_CAMERA_PIPELINE_MCM_RAW10 = 0,
    DART_CAMERA_PIPELINE_ISP_YUV420 = 1,
} dart_camera_pipeline_t;

/* Immutable sensor operations. The camera owns all mutable state and resources;
 * operations borrow it only for the call and never allocate on the frame path.
 * A non-zero default mode replaces the SDK probe's choice when the caller left
 * the sensor geometry automatic (the probe prefers the highest frame rate).
 * configure() resolves a zero output geometry to the sensor's default and sets
 * camera->raw_stride, the DMA row pitch in bytes of the plane convert_gray reads.
 * set_analog_gain may be NULL when the sensor has no gain control here.
 */
typedef struct dart_camera_sensor_ops {
    dart_camera_pipeline_t pipeline;
    uint16_t default_width;
    uint16_t default_height;
    uint16_t default_fps;
    int (*matches)(const k_vicap_sensor_info* sensor);
    int (*configure)(dart_camera_t* camera, const k_vicap_sensor_info* sensor);
    int (*exposure_range)(const dart_camera_t* camera, uint32_t* min_lines, uint32_t* max_lines);
    int (*set_exposure_lines)(dart_camera_t* camera, uint32_t lines);
    int (*set_analog_gain)(dart_camera_t* camera, uint32_t gain_milli);
    int (*convert_gray)(
        const dart_camera_t* camera, const uint8_t* raw, size_t raw_bytes,
        uint8_t* gray, size_t gray_bytes);
} dart_camera_sensor_ops_t;

const dart_camera_sensor_ops_t* dart_camera_find_sensor(const k_vicap_sensor_info* sensor);
extern const dart_camera_sensor_ops_t dart_camera_sc038hgs_ops;
extern const dart_camera_sensor_ops_t dart_camera_sc132gs_ops;

#endif
