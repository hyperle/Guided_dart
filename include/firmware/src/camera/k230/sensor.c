#include "camera/k230/sensor.h"

#include <stddef.h>

const dart_camera_sensor_ops_t* dart_camera_find_sensor(const k_vicap_sensor_info* sensor) {
    static const dart_camera_sensor_ops_t* const sensors[] = {
        &dart_camera_sc038hgs_ops,
        &dart_camera_sc132gs_ops,
    };
    if (sensor == NULL)
        return NULL;
    for (size_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); ++i) {
        if (sensors[i]->matches(sensor))
            return sensors[i];
    }
    return NULL;
}
