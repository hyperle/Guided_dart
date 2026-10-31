module;
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <expected>
#include <limits>

export module dart.control.servo_mapping;

export import dart.detection.result;

export namespace dart::control {

class ServoMapping final {
public:
    static constexpr std::size_t kChannelCount = 4;
    using PwmValues = std::array<float, kChannelCount>;

    // CH1..CH4 correspond to PWM1/2/3/5 on the tested Dart V1 board.
    struct Channel {
        float initial_pulse_us = 1520.0F;
        float min_pulse_us = 1420.0F;
        float max_pulse_us = 1620.0F;
        // UNVALIDATED: zero disables that axis until its signed gain is set.
        float horizontal_us_per_pixel = 0.0F;
        float vertical_us_per_pixel = 0.0F;
    };

    struct Config {
        std::array<Channel, kChannelCount> channels{};
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

    // Image coordinates: x right, y down. Positive requests mean right/up.
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
        const detection::Target& armor, const detection::Target& light,
        detection::Point preset, bool horizontal) const noexcept;

    Config config_;
};

ServoMapping::ServoMapping() noexcept : ServoMapping(Config{}) {}

ServoMapping::ServoMapping(const Config& config) noexcept : config_(config) {}

bool ServoMapping::valid_config() const noexcept {
    for (const auto& channel : config_.channels) {
        if (!std::isfinite(channel.initial_pulse_us) || !std::isfinite(channel.min_pulse_us)
            || !std::isfinite(channel.max_pulse_us)
            || !std::isfinite(channel.horizontal_us_per_pixel)
            || !std::isfinite(channel.vertical_us_per_pixel) || channel.min_pulse_us <= 0.0F
            || channel.min_pulse_us > channel.initial_pulse_us
            || channel.initial_pulse_us > channel.max_pulse_us)
            return false;
    }
    return true;
}

std::expected<ServoMapping::Increment, ServoMapping::Error>
ServoMapping::horizontal_increment(
    const detection::Target& armor, const detection::Target& light,
    detection::Point preset) const noexcept {
    return increment(armor, light, preset, true);
}

std::expected<ServoMapping::Increment, ServoMapping::Error>
ServoMapping::vertical_increment(
    const detection::Target& armor, const detection::Target& light,
    detection::Point preset) const noexcept {
    return increment(armor, light, preset, false);
}

std::expected<ServoMapping::Increment, ServoMapping::Error> ServoMapping::increment(
    const detection::Target& armor, const detection::Target& light, detection::Point preset,
    bool horizontal) const noexcept {
    if (!valid_config())
        return std::unexpected(Error::kInvalidConfig);
    if (!std::isfinite(preset.x) || !std::isfinite(preset.y))
        return std::unexpected(Error::kInvalidCoordinate);

    Increment result{};
    const auto* target = armor.valid ? &armor : light.valid ? &light : nullptr;
    if (target == nullptr)
        return result;
    if (!std::isfinite(target->center.x) || !std::isfinite(target->center.y))
        return std::unexpected(Error::kInvalidCoordinate);
    result.source = armor.valid ? detection::TargetKind::kArmor : detection::TargetKind::kLight;

    const double error_px = horizontal ? static_cast<double>(target->center.x) - preset.x
                                       : static_cast<double>(preset.y) - target->center.y;
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        const auto& channel = config_.channels[i];
        const double gain = horizontal ? channel.horizontal_us_per_pixel
                                       : channel.vertical_us_per_pixel;
        const double delta = error_px * gain;
        if (std::abs(delta) > std::numeric_limits<float>::max())
            return std::unexpected(Error::kInvalidIncrement);
        result.delta_us[i] = static_cast<float>(delta);
    }
    return result;
}

std::expected<ServoMapping::Output, ServoMapping::Error> ServoMapping::pulses(
    const PwmValues& horizontal_delta_us, const PwmValues& vertical_delta_us) const noexcept {
    if (!valid_config())
        return std::unexpected(Error::kInvalidConfig);

    Output result{};
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        if (!std::isfinite(horizontal_delta_us[i]) || !std::isfinite(vertical_delta_us[i]))
            return std::unexpected(Error::kInvalidIncrement);
        const auto& channel = config_.channels[i];
        const double requested = static_cast<double>(channel.initial_pulse_us)
                               + horizontal_delta_us[i] + vertical_delta_us[i];
        const double bounded = std::clamp(
            requested, static_cast<double>(channel.min_pulse_us),
            static_cast<double>(channel.max_pulse_us));
        result.pulse_us[i] = static_cast<float>(bounded);
        result.delta_us[i] = result.pulse_us[i] - channel.initial_pulse_us;
        result.saturated[i] = requested != bounded;
    }
    return result;
}

} // namespace dart::control
