#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>

#include "pwm/interface.hpp"
#include "runtime/component.hpp"
#include "src/control/blackboard.hpp"

namespace dart::control {

struct ServoConfig {
    struct Channel {
        float initial_pulse_us;
        float horizontal_us_per_pixel;
        float vertical_us_per_pixel;
    };
    // Physical CH1..4, preserving the component's four-surface mapping.
    std::array<Channel, pwm::kChannelCount> channels{{
        {1445.0F, -1.0F,  1.0F},
        {1490.0F, -1.0F, -1.0F},
        {1150.0F,  1.0F, -1.0F},
        {1475.0F,  1.0F,  1.0F},
    }};
    // Limit the combined image-target correction only; roll compensation is unbounded.
    float max_delta_us = 150.0F;
    detection::Point aim{320.0F, 240.0F};
    std::chrono::milliseconds target_timeout{200};
    bool valid() const noexcept;
};

struct ServoStatus {
    std::atomic<std::uint64_t> writes{0}, stale_targets{0};
    std::atomic<bool> active{false};
    std::array<std::atomic<float>, pwm::kChannelCount> pulse_us{};
    std::atomic<const char*> operation{"none"};
    std::atomic<int> channel{-1}, native_error{0};
};

class ServoFactory final {
public:
    static std::expected<std::unique_ptr<executor::Component>, executor::Error> create(
        const executor::Component::Spec& spec, const ServoConfig& config,
        BlackBoard& blackboard, pwm::IPwm& pwm, ServoStatus& status) noexcept;
};

} // namespace dart::control
