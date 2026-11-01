#pragma once

#include <cstdint>

#include "imu/math_types.hpp"

namespace dart::imu {
enum class SampleFlag : std::uint32_t {
    kNone = 0U,
    kAccelerometerSaturated = 1U << 0U,
    kGyroscopeSaturated = 1U << 1U,
};

struct RawCounts {
    Vector3i16 accelerometer{};
    Vector3i16 gyroscope{};
};

struct Sample {
    RawCounts raw_counts{};
    /* Specific force in m/s^2 and angular rate in rad/s, both sensor/body axes. */
    Vector3f acceleration_m_s2{};
    Vector3f angular_velocity_rad_s{};
    float temperature_c = 0.0F;
    std::uint64_t timestamp_ns = 0U; /* CLOCK_MONOTONIC domain. */
    std::uint32_t flags = 0U;

    static constexpr bool has_flag(std::uint32_t flags, SampleFlag flag) noexcept {
        return (flags & static_cast<std::uint32_t>(flag)) != 0U;
    }
};

} // namespace dart::imu
