#include "src/detector/tracking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

#include "src/detector/numeric.hpp"
#include "src/detector/result.hpp"

namespace dart::detection {

bool TargetTracker::ValidConfig(const Config& c) noexcept {
    return IsPositive(c.association_k) && IsPositive(c.association_min_px)
        && IsPositive(c.min_size_ratio) && c.min_size_ratio <= 1.0F
        && IsFraction(c.min_contrast_ratio) && c.validation_interval > 0
        && IsFraction(c.quality_drop_ratio) && IsPositive(c.velocity_alpha)
        && c.velocity_alpha <= 1.0F && c.prediction_max_ms > 0 && c.prediction_max_ms <= 1000;
}

float TargetTracker::MatchCost(
    const Target& value, const Target& predicted, const Target& reference,
    const Config& c) noexcept {
    const auto invalid = std::numeric_limits<float>::infinity();
    if (!value.valid || !predicted.valid || !reference.valid || value.bounds.width == 0
        || value.bounds.height == 0 || reference.bounds.width == 0 || reference.bounds.height == 0)
        return invalid;
    const float wr = static_cast<float>(value.bounds.width) / reference.bounds.width;
    const float hr = static_cast<float>(value.bounds.height) / reference.bounds.height;
    const float lo = std::min(value.contrast, reference.contrast);
    const float hi = std::max(value.contrast, reference.contrast);
    if (wr < c.min_size_ratio || wr > 1.0F / c.min_size_ratio || hr < c.min_size_ratio
        || hr > 1.0F / c.min_size_ratio || (hi > 0.0F && lo < c.min_contrast_ratio * hi))
        return invalid;
    const float tolerance = std::max(
        c.association_min_px,
        c.association_k * std::max(reference.bounds.width, reference.bounds.height));
    const float dx = value.center.x - predicted.center.x, dy = value.center.y - predicted.center.y;
    const float position = (dx * dx + dy * dy) / (tolerance * tolerance);
    if (!std::isfinite(position) || position > 1.0F)
        return invalid;
    return position + 0.1F * (std::fabs(1.0F - wr) + std::fabs(1.0F - hr))
         + (hi > 0.0F ? 0.1F * (1.0F - lo / hi) : 0.0F);
}

float TargetTracker::MatchCost(const Target& value, const Target& predicted) const noexcept {
    return MatchCost(value, predicted, last_, config_);
}

Target TargetTracker::Predict(
    std::chrono::microseconds now, std::uint32_t width, std::uint32_t height) const noexcept {
    if (!last_.valid || width == 0 || height == 0)
        return {};
    const auto elapsed = std::clamp<std::int64_t>(
        (now - received_).count(), 0, static_cast<std::int64_t>(config_.prediction_max_ms) * 1000);
    const float seconds = static_cast<float>(elapsed) * 1.0e-6F;
    const float limit =
        4.0F
        * std::max(
            config_.association_min_px,
            config_.association_k * std::max(last_.bounds.width, last_.bounds.height));
    const float dx = std::clamp(velocity_.x * seconds, -limit, limit);
    const float dy = std::clamp(velocity_.y * seconds, -limit, limit);
    auto predicted = last_;
    predicted.center = {
        std::clamp(last_.center.x + dx, 0.0F, static_cast<float>(width - 1)),
        std::clamp(last_.center.y + dy, 0.0F, static_cast<float>(height - 1))};
    predicted.bounds.width = std::min(last_.bounds.width, width);
    predicted.bounds.height = std::min(last_.bounds.height, height);
    predicted.bounds.x = static_cast<std::uint32_t>(std::clamp(
        std::round(last_.bounds.x + dx), 0.0F, static_cast<float>(width - predicted.bounds.width)));
    predicted.bounds.y = static_cast<std::uint32_t>(std::clamp(
        std::round(last_.bounds.y + dy), 0.0F,
        static_cast<float>(height - predicted.bounds.height)));
    return predicted;
}

void TargetTracker::Observe(const Target& value, std::chrono::microseconds now) noexcept {
    if (!value.valid)
        return;
    if (last_.valid && now > received_) {
        const float dt = static_cast<float>((now - received_).count()) * 1.0e-6F;
        const float alpha = velocity_valid_ ? config_.velocity_alpha : 1.0F;
        velocity_.x += alpha * ((value.center.x - last_.center.x) / dt - velocity_.x);
        velocity_.y += alpha * ((value.center.y - last_.center.y) / dt - velocity_.y);
        velocity_valid_ = true;
    }
    last_ = value;
    received_ = now;
}

} // namespace dart::detection
