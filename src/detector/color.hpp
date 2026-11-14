#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace dart::detection {

// Named color profiles. Masks select several profiles at once, one bit each.
enum class Color : std::uint8_t { kGreen = 0, kRed = 1, kBlue = 2 };
inline constexpr std::size_t kColorCount = 3;
using ColorMask = std::uint8_t;
inline constexpr ColorMask kAllColors = (1U << kColorCount) - 1U;

constexpr ColorMask mask_of(Color color) noexcept {
    return static_cast<ColorMask>(1U << static_cast<unsigned>(color));
}

// CIE Lab in the CanMV/OpenMV convention: L 0..100, a/b -128..127.
struct Lab {
    float l = 0.0F, a = 0.0F, b = 0.0F;

    float chroma() const noexcept { return std::hypot(a, b); }
    static float distance(Lab x, Lab y) noexcept {
        const float dl = x.l - y.l, da = x.a - y.a, db = x.b - y.b;
        return std::sqrt(dl * dl + da * da + db * db);
    }
};

// One inclusive CanMV threshold tuple (L0, L1, a0, a1, b0, b1).
struct LabRange {
    int l_min = 0, l_max = 100, a_min = -128, a_max = 127, b_min = -128, b_max = 127;

    bool valid() const noexcept {
        return 0 <= l_min && l_min <= l_max && l_max <= 100 && -128 <= a_min && a_min <= a_max
            && a_max <= 127 && -128 <= b_min && b_min <= b_max && b_max <= 127;
    }
    bool contains(Lab lab) const noexcept {
        return l_min <= lab.l && lab.l <= l_max && a_min <= lab.a && lab.a <= a_max
            && b_min <= lab.b && lab.b <= b_max;
    }
    // 1 at the interval center, 0 at its boundary or outside.
    float centrality(Lab lab) const noexcept;
};

// Per-pixel color evidence for RGB565 frames. Prepare() fills two lookup
// tables (64 KiB profile masks, 192 KiB Lab values); the frame path only reads.
class ColorTable {
public:
    struct Config {
        // A blob's mean Lab needs at least this sqrt(a*a + b*b); white
        // highlights pass most L ranges but are no evidence of a color.
        float min_chroma = 10.0F;
        // Defaults from the SC132GS AWB 480x640 tuning; calibrate on the target.
        std::array<LabRange, kColorCount> ranges{{
            {56, 96, -83, -43, 7, 47},  // green
            {15, 100, 20, 127, 0, 127}, // red
            {50, 80, -20, 10, -70, 10}, // blue
        }};
    };

    static bool ValidConfig(const Config& config) noexcept;
    // Returns false when the tables cannot be allocated.
    bool Prepare(const Config& config) noexcept;
    bool prepared() const noexcept { return masks_ != nullptr; }

    ColorMask classify(std::uint16_t pixel) const noexcept { return masks_[pixel]; }
    const std::int8_t* lab(std::uint16_t pixel) const noexcept { return &labs_[pixel * 3U]; }
    const Config& config() const noexcept { return config_; }
    const LabRange& range(Color color) const noexcept {
        return config_.ranges[static_cast<std::size_t>(color)];
    }

    // The OpenMV imlib conversion: RGB565 expanded to RGB888, sRGB D65, floored.
    static std::array<std::int8_t, 3> ToLab(std::uint16_t pixel) noexcept;

    static std::uint16_t Rgb565(const std::byte* row, std::uint32_t x) noexcept {
        return static_cast<std::uint16_t>(
            std::to_integer<std::uint16_t>(row[2U * x])
            | std::to_integer<std::uint16_t>(row[2U * x + 1U]) << 8U);
    }

private:
    Config config_{};
    std::unique_ptr<ColorMask[]> masks_{};
    std::unique_ptr<std::int8_t[]> labs_{};
};

} // namespace dart::detection
