#pragma once

#include <cstdint>
#include "sensor_concepts.hpp"

namespace dart::hardware {

template <SensorDriver Driver>
class AttitudeSolver {
public:
    AttitudeSolver(Driver& driver) : driver_(driver) {}

    void update() {
        // 编译期静态绑定，直接内联为纯寄存器访问代码

        int16_t raw_x = driver_.read_raw();

        // if (accel_x > 2.5g) {
        //     sensor.reset();
        // }
    }

private:
    Driver& driver_;
};
} //namespace dart::hardware