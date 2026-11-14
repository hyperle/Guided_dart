#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "src/detector/detect.hpp"

namespace dart::tests {

// Colors inside the default Lab profiles (verified by detector_behavior_test).
inline constexpr std::uint16_t kGreen = 0x2EED; // Lab (78, -68, 44)
inline constexpr std::uint16_t kRed = 0xE145;   // Lab (50, 69, 48)
inline constexpr std::uint16_t kBlue = 0x3CBF;  // Lab (61, 8, -60)
inline constexpr std::uint16_t kDark = 0x10A3;  // Lab (6, -1, -4)
inline constexpr std::uint16_t kWhite = 0xFFFF; // Lab (100, 0, -1)

// An RGB565 test frame with optional row padding to catch stride mistakes.
class ColorImage final {
public:
    ColorImage(std::uint32_t width, std::uint32_t height, std::uint32_t padding = 0) noexcept
        : width_(width), height_(height), stride_((width + padding) * 2U),
          bytes_(static_cast<std::size_t>(stride_) * height, std::byte{0xA5}) {
        clear();
    }

    void clear(std::uint16_t color = kDark) noexcept { rect(0, 0, width_, height_, color); }
    void set(std::uint32_t x, std::uint32_t y, std::uint16_t color) noexcept {
        if (x >= width_ || y >= height_)
            return;
        auto* pixel = &bytes_[static_cast<std::size_t>(y) * stride_ + 2U * x];
        pixel[0] = std::byte(color & 0xFFU);
        pixel[1] = std::byte(color >> 8U);
    }
    void rect(std::uint32_t x, std::uint32_t y, std::uint32_t w, std::uint32_t h, std::uint16_t color) noexcept {
        for (auto row = y; row < y + h; ++row)
            for (auto col = x; col < x + w; ++col)
                set(col, row, color);
    }
    // Pixel centers inside the ellipse ((x-cx)/rx)^2 + ((y-cy)/ry)^2 <= 1.
    void ellipse(float cx, float cy, float rx, float ry, std::uint16_t color) noexcept {
        for (std::uint32_t y = 0; y < height_; ++y)
            for (std::uint32_t x = 0; x < width_; ++x) {
                const float dx = (static_cast<float>(x) - cx) / rx, dy = (static_cast<float>(y) - cy) / ry;
                if (dx * dx + dy * dy <= 1.0F)
                    set(x, y, color);
            }
    }
    void disk(float cx, float cy, float radius, std::uint16_t color) noexcept {
        ellipse(cx, cy, radius, radius, color);
    }
    // A filled rectangle of the given long-axis length and thickness, rotated
    // by `degrees` from the x axis around (cx, cy).
    void bar(float cx, float cy, float length, float thickness, float degrees, std::uint16_t color) noexcept {
        const float angle = degrees * 3.14159265F / 180.0F, c = std::cos(angle), s = std::sin(angle);
        for (std::uint32_t y = 0; y < height_; ++y)
            for (std::uint32_t x = 0; x < width_; ++x) {
                const float dx = static_cast<float>(x) - cx, dy = static_cast<float>(y) - cy;
                const float along = dx * c + dy * s, across = -dx * s + dy * c;
                if (std::fabs(along) <= length * 0.5F && std::fabs(across) <= thickness * 0.5F)
                    set(x, y, color);
            }
    }

    camera::FrameView view(std::uint64_t sequence = 1, std::int64_t received_us = 10'000) const noexcept {
        camera::FrameView frame{};
        frame.pixels = bytes_;
        frame.format = camera::PixelFormat::kRgb565;
        frame.width = width_;
        frame.height = height_;
        frame.stride = stride_;
        frame.sequence = sequence;
        frame.received = std::chrono::microseconds{received_us};
        return frame;
    }
    camera::FrameInfo info(float pixel_aspect = 1.0F) const noexcept {
        return {width_, height_, camera::PixelFormat::kRgb565, pixel_aspect};
    }
    std::span<const std::byte> bytes() const noexcept { return bytes_; }

private:
    std::uint32_t width_, height_, stride_;
    std::vector<std::byte> bytes_;
};

// The production calculation objects share no camera or component lifecycle.
class DetectionCalculator final {
public:
    using Config = detection::Detect::Config;

    DetectionCalculator() noexcept
        : DetectionCalculator(Config{}) {}
    explicit DetectionCalculator(Config config) noexcept
        : detect_(config)
        , track_(config.light, config.armor, config.track, config.roi) {}

    std::expected<void, detection::DetectionError> Prepare(const camera::FrameInfo& info) noexcept {
        track_.Reset();
        return detect_.Prepare(info);
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
