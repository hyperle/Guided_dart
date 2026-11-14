#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>

#include "camera/frame_view.hpp"
#include "src/detector/color.hpp"

namespace dart::detection {

struct Point {
    float x = 0.0F;
    float y = 0.0F;

    static constexpr Point missing() noexcept { return {-1.0F, -1.0F}; }
};

struct Rect {
    std::uint32_t x = 0, y = 0, width = 0, height = 0;

    static constexpr Rect full(std::uint32_t width, std::uint32_t height) noexcept {
        return {0, 0, width, height};
    }
    bool empty() const noexcept { return width == 0 || height == 0; }
    bool is_full(std::uint32_t frame_width, std::uint32_t frame_height) const noexcept {
        return x == 0 && y == 0 && width == frame_width && height == frame_height;
    }
    Rect unite(Rect other) const noexcept {
        if (empty())
            return other;
        if (other.empty())
            return *this;
        const auto left = std::min(x, other.x), top = std::min(y, other.y);
        return {
            left, top, std::max(x + width, other.x + other.width) - left,
            std::max(y + height, other.y + other.height) - top};
    }
    // Floor/ceil a floating rectangle into [0, width) x [0, height).
    static Rect Clip(float x, float y, float w, float h, std::uint32_t width,
                     std::uint32_t height) noexcept;
};

class Numeric final {
public:
    static bool IsPositive(float value) noexcept { return std::isfinite(value) && value > 0; }
    static bool IsFraction(float value) noexcept {
        return std::isfinite(value) && value >= 0 && value <= 1;
    }
    static void IncrementSaturated(std::uint32_t& value) noexcept {
        if (value != std::numeric_limits<std::uint32_t>::max())
            ++value;
    }
};

inline constexpr float kPi = std::numbers::pi_v<float>;
inline constexpr float kInvalidCost = std::numeric_limits<float>::infinity();

enum class DetectionError : int {
    kInvalidConfig = 1,
    kNoMemory,
    kNotPrepared,
    kInvalidFrame,
    kInvalidWindow,
    kWorkspaceExhausted,
    kOutOfOrderFrame,
    kUnsupportedFormat,
};

// One connected region of a single color profile, measured on full-resolution
// pixels. Coordinates are frame pixels; shape values use the camera pixel aspect.
class Blob {
public:
    Point center{};       // Pixel centroid.
    Rect bounds{};
    std::uint32_t area = 0;
    Color color = Color::kGreen;
    Lab lab{};            // Mean Lab of the region's pixels.
    float color_quality = 0.0F;
    float fill = 0.0F;    // area / bounding-box area.
    float circularity = 0.0F;
    float aspect = 0.0F;  // Corrected bounding-box width / height.
    // Minimum-area rectangle around the pixel centers, extents in pixels.
    float length = 0.0F, thickness = 0.0F;
    Point axis{1.0F, 0.0F};
    Point box_center{};
    float oriented_fill = 0.0F; // area / (length * thickness).
    bool clipped = false;       // Touches the scanned window border.
};

class Target {
public:
    bool valid = false;
    Point center = Point::missing();
    Rect bounds{};
    std::uint32_t area = 0;
    float radius = 0.0F;
    float quality = 0.0F;
    bool clipped = false;
    Color color = Color::kGreen;
    Lab lab{};
    float color_quality = 0.0F;
    float circularity = 0.0F;      // Light only.
    // Armor only: the two bar centers and the pair color evidence.
    bool has_bars = false;
    std::array<Point, 2> bars{};
    float bar_lab_distance = 0.0F;
    float gap_color_fraction = 0.0F;
};

class FrameMetadata {
public:
    std::uint64_t frame_id = 0;
    std::uint64_t source_pts = 0;
    std::chrono::microseconds received{};
    bool source_pts_timebase_validated = false;
};

// Per-frame candidate counts. Each rejection names the first failed gate.
class ScanCounters {
public:
    std::uint32_t raw_blobs = 0;        // Regions above the area floor, before color quality.
    std::uint32_t light_candidates = 0, armor_candidates = 0;
    std::uint32_t rejected_color = 0;   // Mean chroma below color.min_chroma.
    std::uint32_t rejected_light_area = 0, rejected_light_shape = 0;
    std::uint32_t rejected_bar_shape = 0;
    std::uint32_t rejected_pair_color = 0, rejected_pair_geometry = 0, rejected_pair_lab = 0;
    std::uint32_t rejected_gap = 0, rejected_anchor = 0, rejected_association = 0;
};

enum class DetectionPhase : std::uint8_t { kDetect, kTrack };

class ScanMetadata {
public:
    Rect search_window{};
    bool full_frame = false;
    bool rescanned = false;
    bool gap_reset = false;
    bool full_validation = false;
    bool armor_scanned = false;
    std::uint32_t candidates = 0; // light_candidates + armor_candidates.
    DetectionPhase phase = DetectionPhase::kDetect;
    ScanCounters counters{};
};

enum class TargetKind : std::uint8_t { kNone, kLight, kArmor };

// Why the published target is what it is, in pipeline order.
enum class DetectReason : std::uint8_t {
    kNoLightColor,      // No candidate of a light color.
    kLightColorQuality, // Only low-chroma regions of a light color.
    kLightRejected,     // Shape or association rejected every light.
    kLightConfirm,      // A light awaits consecutive confirmation.
    kLightLocked,       // Light tracked; armor not searched this frame.
    kNoArmorColor,      // Armor window had no bar-colored candidate.
    kArmorRejected,     // Bars found, no acceptable pair.
    kArmorConfirm,      // An armor pair awaits confirmation.
    kArmorLocked,
};

class DetectResult {
public:
    TargetKind kind = TargetKind::kNone;
    Target target{}; // Armor center when available, otherwise light center.
    Target light{};
    Target armor{};
    FrameMetadata source{};
    ScanMetadata scan{};
    std::uint32_t missed_frames = 0;
    std::uint32_t light_hits = 0, armor_hits = 0;
    DetectReason reason = DetectReason::kNoLightColor;

    void select_target() noexcept {
        target = armor.valid ? armor : light;
        kind = armor.valid ? TargetKind::kArmor
             : light.valid ? TargetKind::kLight
                           : TargetKind::kNone;
    }
    Point coordinate() const noexcept { return target.valid ? target.center : Point::missing(); }
};

// Identity test between a measurement and a predicted/previous target.
class Association final {
public:
    struct Config {
        float association_k = 1.5F;
        float association_min_px = 6.0F;
        float min_size_ratio = 0.5F;
        float min_color_ratio = 0.35F; // Current / previous color-quality similarity.
        float max_lab_distance = 30.0F;
    };
    static bool ValidConfig(const Config& config) noexcept;
    // Lower is better; kInvalidCost when color, size, color quality, Lab or
    // position falls outside the gates. The radius follows `reference` size.
    static float Cost(
        const Target& value, const Target& predicted, const Target& reference,
        const Config& config) noexcept;
    // Both bar centers lie within tolerance, in either order. Targets without
    // bars impose no constraint.
    static bool SameBars(const Target& value, const Target& reference, float tolerance) noexcept;
};

class Light final {
public:
    struct Config {
        ColorMask colors = mask_of(Color::kGreen);
        std::uint32_t min_area = 3;
        std::uint32_t far_area_threshold = 30;
        std::uint32_t far_size_px = 8;
        std::uint32_t search_step = 2; // Seed stride of a full-frame acquisition scan.
        std::uint32_t confirm_frames = 2;
        std::uint32_t far_confirm_frames = 3;
        float far_jump_k = 1.5F;
        std::uint32_t far_jump_min_px = 6;
        std::uint32_t deadband_px = 2;
        float min_fill = 0.5F;
        float min_circularity = 0.6F; // Every lamp size, far lamps included.
        float circularity_weight = 5.0F;
        float aspect_min = 0.7F;
        float aspect_max = 1.4F;
        float max_area_fraction = 0.25F;
    };
    enum class Rejection : std::uint8_t { kNone, kSmall, kArea, kCircularity, kFill, kAspect };

    explicit Light(Config config) noexcept : config_(config) {}
    bool ValidConfig() const noexcept;
    Rejection Check(const Blob& blob, std::uint64_t frame_area) const noexcept;
    // Acquisition ranking: area, weighted roundness and color quality.
    float Score(const Blob& blob) const noexcept;
    bool Far(const Target& target) const noexcept;
    static Target ToTarget(const Blob& blob) noexcept;

private:
    Config config_;
};

class Armor final {
public:
    struct Config {
        ColorMask colors = kAllColors; // Both bars must match the same profile.
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
        float window_width_k = 12.0F;
        float window_height_k = 12.0F;
        float window_gap_k = 0.0F;
        float span_min_k = 0.2F;
        float span_max_k = 12.0F;
        std::uint32_t plate_up_lo_pct = 0;
        std::uint32_t plate_up_hi_pct = 1200;
        std::uint32_t plate_offx_pct = 600;
        float association_k = 0.6F;
        float association_min_px = 4.0F;
        std::uint32_t track_hold = 2;
        std::uint32_t confirm_frames = 2;
        float max_lab_distance = 25.0F;       // Between the two bars.
        float max_gap_color_fraction = 0.4F;  // Gap samples matching the bar color.
    };

    // Inputs of one pair search. `predicted` and `reference` are valid only
    // while an armor is tracked; the search then accepts only that identity.
    struct Search {
        const Target& light;
        const Target& predicted;
        const Target& reference;
        const Association::Config& association;
        const camera::FrameView& frame;
        const ColorTable& colors;
    };

    explicit Armor(Config config) noexcept : config_(config) {}
    bool ValidConfig() const noexcept;
    // The window above the light where its armor plate may appear.
    static Rect SearchWindow(
        const Config& config, const Target& light, std::uint32_t width,
        std::uint32_t height) noexcept;
    Rect SearchWindow(const Target& light, std::uint32_t width, std::uint32_t height) const noexcept {
        return SearchWindow(config_, light, width, height);
    }
    Target Select(std::span<const Blob> blobs, const Search& search, ScanCounters& counters) const noexcept;
    float AssociationTolerance(const Target& target) const noexcept {
        return std::max(
            config_.association_min_px,
            config_.association_k * static_cast<float>(std::max(target.bounds.width, target.bounds.height)));
    }

private:
    struct Bar {
        const Blob* blob = nullptr;
        Point center{};
        Point axis{};
        float length = 0.0F;
        float thickness = 0.0F;
        Point first{}, last{}; // Long-axis endpoints.
    };

    bool MakeBar(const Blob& blob, float scale, Bar& bar) const noexcept;
    bool PairCenter(const Bar& a, const Bar& b, Point& center) const noexcept;
    static float GapColorFraction(
        const Bar& a, const Bar& b, float separation, const camera::FrameView& frame,
        const ColorTable& colors) noexcept;

    Config config_;
};

} // namespace dart::detection
