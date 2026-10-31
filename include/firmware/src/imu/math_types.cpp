module;
#include <cstdint>

export module dart.imu.math_types;



export namespace dart::imu {

struct Vector3f {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct Vector3i16 {
    std::int16_t x = 0;
    std::int16_t y = 0;
    std::int16_t z = 0;
};

struct Quaternionf {
    /* Rotation from sensor/body axes into the selected reference frame. */
    float w = 1.0F;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

} // namespace dart::imu
