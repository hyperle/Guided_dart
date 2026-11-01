#include "src/control/servo_mapping.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <expected>
#include <limits>

#include "src/detector/result.hpp"

namespace dart::control {

ServoMapping::ServoMapping() noexcept
    : ServoMapping(Config{}) {}

ServoMapping::ServoMapping(const Config& config) noexcept
    : config_(config) {}

bool ServoMapping::valid_config() const noexcept {
    if (!std::isfinite(config_.max_delta_us) || config_.max_delta_us < 0.0F)
        return false;
    for (const auto& channel : config_.channels) {
        if (!std::isfinite(channel.initial_pulse_us)
            || !std::isfinite(channel.horizontal_us_per_pixel)
            || !std::isfinite(channel.vertical_us_per_pixel)
            || channel.initial_pulse_us <= config_.max_delta_us)
            return false;
    }
    return true;
}

std::expected<ServoMapping::Increment, ServoMapping::Error> ServoMapping::horizontal_increment(
    const detection::Target& armor, const detection::Target& light,
    detection::Point preset) const noexcept {
    return increment(armor, light, preset, true);
}

std::expected<ServoMapping::Increment, ServoMapping::Error> ServoMapping::vertical_increment(
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
                                       : static_cast<double>(target->center.y) - preset.y;
    for (std::size_t i = 0; i < kChannelCount; ++i) {
        const auto& channel = config_.channels[i];
        const double gain =
            horizontal ? channel.horizontal_us_per_pixel : channel.vertical_us_per_pixel;
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
        const double requested = static_cast<double>(horizontal_delta_us[i]) + vertical_delta_us[i];
        const double limit = config_.max_delta_us;
        const double bounded = std::clamp(requested, -limit, limit);
        result.pulse_us[i] = static_cast<float>(channel.initial_pulse_us + bounded);
        result.delta_us[i] = result.pulse_us[i] - channel.initial_pulse_us;
        result.saturated[i] = requested != bounded;
    }
    return result;
}

} // namespace dart::control
