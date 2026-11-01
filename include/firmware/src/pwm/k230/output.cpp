#include "pwm/k230/output.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>

#include "drv_fpioa.h"
#include "drv_pwm.h"

#include "pwm/interface.hpp"

namespace dart::pwm::k230 {
constexpr std::array<int, kChannelCount> channels_{1, 2, 3, 5};
constexpr std::array<int, kChannelCount> pins_{61, 7, 8, 59};
constexpr std::array<fpioa_func_t, kChannelCount> functions_{PWM1, PWM2, PWM3, PWM5};

std::expected<void, Error> Output::start(const Pulses& initial) noexcept {
    if (initialized_)
        return std::unexpected(Error{"already_started", -1, -1});
    if (!valid_pulses(initial))
        return std::unexpected(Error{"invalid_pulse", -1, -1});
    const int status = drv_pwm_init();
    if (status != 0)
        return std::unexpected(Error{"initialize", -1, status});
    initialized_ = true;
    for (const auto channel : channels_) {
        if (const int rc = drv_pwm_disable(channel); rc != 0)
            return fail("disable", channel, rc);
    }
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        fpioa_iomux_cfg_t before{}, after{};
        fpioa_func_t function{};
        if (const int rc = drv_fpioa_get_pin_cfg(pins_[i], &before.u.value); rc != 0)
            return fail("pin_read", channels_[i], rc);
        if (const int rc = drv_fpioa_set_pin_func(pins_[i], functions_[i]); rc != 0)
            return fail("pin_route", channels_[i], rc);
        if (const int rc = drv_fpioa_set_pin_ie(pins_[i], 1); rc != 0)
            return fail("pin_input", channels_[i], rc);
        if (const int rc = drv_fpioa_get_pin_func(pins_[i], &function); rc != 0)
            return fail("pin_verify", channels_[i], rc);
        if (const int rc = drv_fpioa_get_pin_cfg(pins_[i], &after.u.value); rc != 0)
            return fail("pin_verify", channels_[i], rc);
        if (function != functions_[i] || !after.u.bit.oe || !after.u.bit.ie
            || after.u.bit.msc != before.u.bit.msc)
            return fail("pin_mismatch", channels_[i], -1);
    }
    // PWM0/1/2 and PWM3/4/5 share their periods.
    for (const int channel : {1, 3}) {
        if (const int rc = drv_pwm_set_freq(channel, kFrequencyHz); rc != 0)
            return fail("frequency", channel, rc);
    }
    auto written = write(initial);
    if (!written)
        return written;
    for (const auto channel : channels_) {
        if (const int rc = drv_pwm_enable(channel); rc != 0)
            return fail("enable", channel, rc);
    }
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        std::uint32_t frequency = 0, pulse_ns = 0;
        if (const int rc = drv_pwm_get_freq(channels_[i], &frequency); rc != 0)
            return fail("read_frequency", channels_[i], rc);
        if (const int rc = drv_pwm_get_duty_ns(channels_[i], &pulse_ns); rc != 0)
            return fail("read_pulse", channels_[i], rc);
        const auto requested = static_cast<std::int64_t>(std::llround(initial[i] * 1000.0));
        if (frequency != kFrequencyHz
            || std::abs(static_cast<std::int64_t>(pulse_ns) - requested) > 1000)
            return fail("readback_mismatch", channels_[i], -1);
    }
    return {};
}

std::expected<void, Error> Output::write(const Pulses& pulses) noexcept {
    if (!initialized_)
        return std::unexpected(Error{"not_started", -1, -1});
    // Validate the whole command before updating any physical output.
    if (!valid_pulses(pulses))
        return fail("invalid_pulse", -1, -1);
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        const auto ns = static_cast<std::uint32_t>(std::llround(pulses[i] * 1000.0));
        if (const int rc = drv_pwm_set_duty_ns(channels_[i], ns); rc != 0)
            return fail("write", channels_[i], rc);
    }
    return {};
}

std::expected<void, Error> Output::stop() noexcept {
    if (!initialized_)
        return {};
    Error error{};
    for (const auto channel : channels_) {
        if (const int rc = drv_pwm_disable(channel); rc != 0 && error.code == 0)
            error = {"disable", channel, rc};
    }
    if (const int rc = drv_pwm_deinit(); rc != 0 && error.code == 0)
        error = {"deinitialize", -1, rc};
    initialized_ = false;
    if (error.code != 0)
        return std::unexpected(error);
    return {};
}

std::expected<void, Error> Output::fail(const char* operation, int channel, int code) noexcept {
    (void)stop();
    return std::unexpected(Error{operation, channel, code});
}

} // namespace dart::pwm::k230
