#pragma once

#include "src/detector/detect.hpp"

namespace dart::tests {

// The production calculation objects share no camera or component lifecycle.
class DetectionCalculator final {
public:
    using Config = detection::Detect::Config;

    DetectionCalculator() noexcept
        : DetectionCalculator(Config{}) {}
    explicit DetectionCalculator(Config config) noexcept
        : detect_(config)
        , track_(config.light, config.armor, config.track, config.roi) {}

    std::expected<void, detection::DetectionError>
        Prepare(std::uint32_t width, std::uint32_t height) noexcept {
        track_.Reset();
        return detect_.Prepare(width, height);
    }

    std::expected<detection::DetectResult, detection::DetectionError>
        Detect(const camera::FrameView& frame) noexcept {
        return detect_.update(frame, track_);
    }

    void Reset() noexcept { track_.Reset(); }

private:
    detection::Detect detect_;
    detection::Track track_;
};

} // namespace dart::tests
