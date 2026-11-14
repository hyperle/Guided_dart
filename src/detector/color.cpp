#include "src/detector/color.hpp"

#include <algorithm>
#include <cmath>
#include <new>

namespace dart::detection {

float LabRange::centrality(Lab lab) const noexcept {
    if (!contains(lab))
        return 0.0F;
    const auto distance = [](float value, int low, int high) {
        const float half = std::max(1.0F, static_cast<float>(high - low) * 0.5F);
        return std::fabs(value - static_cast<float>(low + high) * 0.5F) / half;
    };
    const float worst = std::max({
        distance(lab.l, l_min, l_max), distance(lab.a, a_min, a_max),
        distance(lab.b, b_min, b_max)});
    return 1.0F - std::min(1.0F, worst);
}

bool ColorTable::ValidConfig(const Config& config) noexcept {
    return std::isfinite(config.min_chroma) && config.min_chroma >= 0.0F
        && std::all_of(config.ranges.begin(), config.ranges.end(),
                       [](const LabRange& range) { return range.valid(); });
}

bool ColorTable::Prepare(const Config& config) noexcept {
    if (!masks_)
        masks_.reset(new (std::nothrow) ColorMask[1U << 16U]);
    if (!labs_)
        labs_.reset(new (std::nothrow) std::int8_t[(1U << 16U) * 3U]);
    if (!masks_ || !labs_) {
        masks_.reset();
        labs_.reset();
        return false;
    }
    config_ = config;
    for (std::uint32_t pixel = 0; pixel < (1U << 16U); ++pixel) {
        const auto lab = ToLab(static_cast<std::uint16_t>(pixel));
        std::copy(lab.begin(), lab.end(), &labs_[pixel * 3U]);
        const Lab value{static_cast<float>(lab[0]), static_cast<float>(lab[1]), static_cast<float>(lab[2])};
        ColorMask mask = 0;
        for (std::size_t color = 0; color < kColorCount; ++color)
            if (config_.ranges[color].contains(value))
                mask |= static_cast<ColorMask>(1U << color);
        masks_[pixel] = mask;
    }
    return true;
}

std::array<std::int8_t, 3> ColorTable::ToLab(std::uint16_t pixel) noexcept {
    // Same bit replication as COLOR_RGB565_TO_R8/G8/B8.
    const unsigned r5 = (pixel >> 11U) & 0x1FU, g6 = (pixel >> 5U) & 0x3FU, b5 = pixel & 0x1FU;
    const unsigned r8 = (r5 << 3U) | (r5 >> 2U), g8 = (g6 << 2U) | (g6 >> 4U),
                   b8 = (b5 << 3U) | (b5 >> 2U);
    const auto linear = [](unsigned value) {
        const float c = static_cast<float>(value) / 255.0F;
        return 100.0F * (c > 0.04045F ? std::pow((c + 0.055F) / 1.055F, 2.4F) : c / 12.92F);
    };
    const float r = linear(r8), g = linear(g8), b = linear(b8);
    const auto f = [](float t) { return t > 0.008856F ? std::cbrt(t) : t * 7.787037F + 0.137931F; };
    const float x = f((r * 0.4124F + g * 0.3576F + b * 0.1805F) / 95.047F);
    const float y = f((r * 0.2126F + g * 0.7152F + b * 0.0722F) / 100.0F);
    const float z = f((r * 0.0193F + g * 0.1192F + b * 0.9505F) / 108.883F);
    const auto clamp = [](float value, float low, float high) {
        return static_cast<std::int8_t>(std::clamp(std::floor(value), low, high));
    };
    return {clamp(116.0F * y - 16.0F, 0.0F, 100.0F), clamp(500.0F * (x - y), -128.0F, 127.0F),
            clamp(200.0F * (y - z), -128.0F, 127.0F)};
}

} // namespace dart::detection
