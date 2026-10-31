module;
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>

export module dart.pwm.interface;



export namespace dart::pwm {

inline constexpr std::size_t kChannelCount = 4;
inline constexpr std::uint32_t kFrequencyHz = 333;
inline constexpr std::uint32_t kPeriodNs = 1'000'000'000 / kFrequencyHz;
using Pulses = std::array<float, kChannelCount>; // CH1..4, absolute microseconds.

struct Error {
    const char* operation = "none";
    int channel = -1; // Hardware PWM channel, or -1 for a device/session error.
    int code = 0;
};

bool valid_pulses(const Pulses& pulses) noexcept {
    for (const auto pulse : pulses) {
        const double ns = static_cast<double>(pulse) * 1000.0;
        if (!std::isfinite(ns) || ns < 1.0 || ns > kPeriodNs)
            return false;
    }
    return true;
}

class IPwm {
public:
    virtual ~IPwm() = default;
    // Configure all channels at their initial positions before enabling any.
    virtual std::expected<void, Error> start(const Pulses& initial) noexcept = 0;
    virtual std::expected<void, Error> write(const Pulses& pulses) noexcept = 0;
    // Idempotent; attempt every channel even after a disable error.
    virtual std::expected<void, Error> stop() noexcept = 0;
};

} // namespace dart::pwm
