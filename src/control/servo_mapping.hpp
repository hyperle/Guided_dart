#pragma once

#include <array>
#include <cstddef>
#include <expected>

#include "src/detector/result.hpp"

namespace dart::control {
class ServoMapping final {
public:
    static constexpr std::size_t kChannelCount = 4;
    using PwmValues = std::array<float, kChannelCount>;

    // CH1..CH4 correspond to PWM1/2/3/5 on the tested Dart V1 board.
    struct Channel {
        float initial_pulse_us = 1520.0F;
        // UNVALIDATED: zero disables that axis until its signed gain is set.
        float horizontal_us_per_pixel = 0.0F;
        float vertical_us_per_pixel = 0.0F;
    };

    struct Config {
        // Mechanical zeros calibrated in tools/canmv_tune.py (CH1..CH4).
        std::array<Channel, kChannelCount> channels{{
            {1490.0F},
            {1448.0F},
            {1475.0F},
            {1150.0F},
        }};
        // One symmetric limit for each channel's combined offset from zero.
        float max_delta_us = 500.0F;
    };

    enum class Error { kInvalidConfig, kInvalidCoordinate, kInvalidIncrement };

    struct Increment {
        PwmValues delta_us{}; // Offset from initial position, never from the previous call.
        detection::TargetKind source = detection::TargetKind::kNone;
    };

    struct Output {
        PwmValues pulse_us{};
        PwmValues delta_us{}; // Actual offsets after combining both axes and clamping.
        std::array<bool, kChannelCount> saturated{};
    };

    ServoMapping() noexcept;
    explicit ServoMapping(const Config& config) noexcept;
    bool valid_config() const noexcept;

    // Image coordinates: x right, y down. Errors are target minus preset.
    std::expected<Increment, Error> horizontal_increment(
        const detection::Target& armor, const detection::Target& light,
        detection::Point preset) const noexcept;
    std::expected<Increment, Error> vertical_increment(
        const detection::Target& armor, const detection::Target& light,
        detection::Point preset) const noexcept;

    // No arguments produces the four independent initial positions.
    // Clamp only after adding both axes, preserving cancellation between them.
    std::expected<Output, Error> pulses(
        const PwmValues& horizontal_delta_us = {},
        const PwmValues& vertical_delta_us = {}) const noexcept;

private:
    std::expected<Increment, Error> increment(
        const detection::Target& armor, const detection::Target& light, detection::Point preset,
        bool horizontal) const noexcept;

    Config config_;
};

} // namespace dart::control
