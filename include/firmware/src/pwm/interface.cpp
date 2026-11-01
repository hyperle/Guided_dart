#include "pwm/interface.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>

namespace dart::pwm {

bool valid_pulses(const Pulses& pulses) noexcept {
    for (const auto pulse : pulses) {
        const double ns = static_cast<double>(pulse) * 1000.0;
        if (!std::isfinite(ns) || ns < 1.0 || ns > kPeriodNs)
            return false;
    }
    return true;
}

} // namespace dart::pwm
