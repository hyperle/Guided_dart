#pragma once 

#include <concepts>
#include <cstdint>

namespace dart::hardware {
template <typename Sensor>
concept SensorDriver = requires (Sensor sensor) {
    { sensor.init() } -> std::same_as<bool>;

    { sensor.read_raw() } -> std::convertible_to<int16_t>;

    { sensor.reset() } -> std::same_as<void>;

    // requires std::unsigned_integral<decltype(Sensor::RATEING_FREQUENCE)>;
};
}