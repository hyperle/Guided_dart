#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "src/detector/blob_workspace.hpp"
#include "src/detector/numeric.hpp"
#include "src/detector/result.hpp"

namespace dart::detection {
class ArmorDetector {
public:
    struct Config {
        std::uint32_t length_floor = 2;
        std::uint32_t area_floor = 2;
        float length_min_k = 0.2F;
        float length_max_k = 8.0F;
        float tiny_px = 8.0F;
        float aspect_min = 2.0F;
        float fill_min = 0.5F;
        float flat_deg = 12.0F;
        float parallel_deg = 15.0F;
        float thickness_ratio = 0.3F;
        float length_ratio = 0.4F;
        float overlap = 0.4F;
        float gap_max_k = 10.0F;
        float diagonal_min_sin = 0.30F;
        float border_penalty = 0.0F;
        float window_width_k = 3.0F;
        float window_height_k = 2.0F;
        float window_gap_k = 0.0F;
        float window_min_side = 24.0F;
        float near_k = 8.0F;
        float near_min_px = 32.0F;
        float span_min_k = 0.2F;
        float span_max_k = 12.0F;
        std::uint32_t plate_up_lo_pct = 0;
        std::uint32_t plate_up_hi_pct = 1200;
        std::uint32_t plate_offx_pct = 600;
        float association_k = 0.6F;
        float association_min_px = 4.0F;
        std::uint32_t track_hold = 2;
        std::uint32_t confirm_frames = 2;
        float min_brightness_ratio = 0.45F;
        float min_gap_contrast = 8.0F;
    };
    ArmorDetector() noexcept;
    explicit ArmorDetector(Config config) noexcept;
    bool ValidConfig() const noexcept;
    Target Detect(
        std::span<const Blob> blobs, const Target& light, const Target* previous = nullptr,
        const camera::FrameView* frame = nullptr, bool associated_only = false,
        std::array<Blob, 2>* selected = nullptr) const noexcept;
    Rect
        SearchWindow(const Target& light, std::uint32_t width, std::uint32_t height) const noexcept;

private:
    struct Bar {
        const Blob* blob = nullptr;
        Point center{};
        Point axis{};
        float length = 0.0F;
        float thickness = 0.0F;
    };

    // Recover the dimensions of a filled rectangular bar from its axis-aligned
    // bounds and area. A rotated bar's bounding-box aspect/fill can be poor even
    // when its principal dimensions are still a valid light bar. The moment
    // The caller keeps the axis-aligned bbox interpretation if recovery fails.
    static bool
        RecoverRectShape(const Blob& blob, float& length, float& thickness, float& tilt) noexcept;

    static float Dot(Point a, Point b) noexcept { return a.x * b.x + a.y * b.y; }

    static float Cross(Point a, Point b) noexcept { return a.x * b.y - a.y * b.x; }

    static Point Delta(Point a, Point b) noexcept { return {a.x - b.x, a.y - b.y}; }

    static bool PlateAnchorOk(const Config& config, Point center, const Target& light) noexcept;

    static bool MakeBar(const Config& config, const Blob& blob, float scale, Bar& bar) noexcept;

    static void
        BarEndpoints(const Config& config, const Bar& bar, Point& first, Point& last) noexcept;

    static bool ArmorCenter(const Config& config, const Bar& a, const Bar& b, Point& out) noexcept;

    static Rect UniteBounds(Rect a, Rect b) noexcept;

    static float GapLuma(const camera::FrameView& frame, const Bar& a, const Bar& b) noexcept;

    Config config_;
};

} // namespace dart::detection
