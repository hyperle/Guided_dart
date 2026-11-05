#pragma once

#include <chrono>
#include <cstdint>

#include "src/detector/numeric.hpp"
#include "src/detector/result.hpp"

namespace dart::detection {
class TargetTracker {
public:
    struct Config {
        float association_k = 1.5F;
        float association_min_px = 6.0F;
        float min_size_ratio = 0.5F;
        float min_contrast_ratio = 0.5F;
        std::uint32_t validation_interval = 8;
        float quality_drop_ratio = 0.65F;
        float velocity_alpha = 0.5F;
        std::uint32_t prediction_max_ms = 50;
    };

    explicit TargetTracker(Config config) noexcept
        : config_(config) {}
    static bool ValidConfig(const Config& c) noexcept;

    static float MatchCost(
        const Target& value, const Target& predicted, const Target& reference,
        const Config& c) noexcept;

    float MatchCost(const Target& value, const Target& predicted) const noexcept;

    Target Predict(
        std::chrono::microseconds now, std::uint32_t width, std::uint32_t height) const noexcept;

    void Observe(const Target& value, std::chrono::microseconds now) noexcept;

    void Reset() noexcept {
        last_ = {};
        received_ = {};
        velocity_ = {};
        velocity_valid_ = false;
    }
    const Target& last() const noexcept { return last_; }

private:
    Config config_;
    Target last_{};
    std::chrono::microseconds received_{};
    Point velocity_{};
    bool velocity_valid_ = false;
};

} // namespace dart::detection
