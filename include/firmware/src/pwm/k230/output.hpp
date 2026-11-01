#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>

#include "pwm/interface.hpp"

namespace dart::pwm::k230 {
// Same connector routing and setup order as tools/pwm_swing.
// One owner per process; the HAL has process-global device state.
class Output final : public IPwm {
public:
    ~Output() override { (void)stop(); }
    Output() = default;
    Output(const Output&) = delete;
    Output& operator=(const Output&) = delete;

    std::expected<void, Error> start(const Pulses& initial) noexcept override;

    std::expected<void, Error> write(const Pulses& pulses) noexcept override;

    std::expected<void, Error> stop() noexcept override;

private:
    std::expected<void, Error> fail(const char* operation, int channel, int code) noexcept;

    bool initialized_ = false;
};

} // namespace dart::pwm::k230
