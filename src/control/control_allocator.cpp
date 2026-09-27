// 控制分配实现：矩阵混合 → 每片舵面限幅到 ±servo_limit。

#include "control/control_allocator.hpp"

#include <cmath>

namespace dart::control {

void ControlAllocator::ApplyMix(const double moment[kBodyAxisCount], double out[kFinCount]) const {
    for (int fin = 0; fin < kFinCount; ++fin) {
        double sum = 0.0;
        for (int axis = 0; axis < kBodyAxisCount; ++axis) {
            sum += params_.mix[fin][axis] * moment[axis];
        }
        out[fin] = sum;
    }
}

ServoCommand ControlAllocator::Allocate(const double moment[kBodyAxisCount]) const {
    ServoCommand out;
    double raw[kFinCount] = {};
    ApplyMix(moment, raw);
    for (int fin = 0; fin < kFinCount; ++fin) {
        const double limited = Clamp(raw[fin], -params_.servo_limit, params_.servo_limit);
        if (std::fabs(raw[fin]) > params_.servo_limit) out.saturated = true;
        out.fin[fin] = limited;
    }
    return out;
}

} // namespace dart::control
