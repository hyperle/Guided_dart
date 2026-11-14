#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string_view>

#include "camera/interface.hpp"
#include "runtime/component.hpp"
#include "src/detector/color.hpp"
#include "src/detector/target.hpp"
#include "src/detector/track.hpp"

namespace dart::detection {

class Detect final {
public:
    // Bounded extraction of single-color regions from RGB565 frames. Every
    // buffer is allocated by Prepare(); scanning a frame never allocates.
    class Workspace {
    public:
        struct Config {
            static constexpr std::uint32_t kMaxCandidates = 256;
            std::uint32_t max_candidates = kMaxCandidates; // Overflow is an error.
            std::uint32_t max_spans = 16384;               // Pending fill spans.
        };
        struct Request {
            Rect window{};
            ColorMask colors = 0;
            std::uint32_t step = 1;     // Seed stride; regions are filled pixel-exactly.
            std::uint32_t min_area = 1; // Pixel count and bounding-box area floor.
        };

        std::expected<void, DetectionError> Prepare(
            std::uint32_t max_width, std::uint32_t max_height, float pixel_aspect,
            Config config) noexcept;
        // Candidates stay valid until the next Scan.
        std::expected<std::span<const Blob>, DetectionError> Scan(
            const camera::FrameView& frame, const ColorTable& table, const Request& request,
            ScanCounters& counters) noexcept;
        std::uint32_t span_high_water() const noexcept { return span_high_water_; }

    private:
        struct Span {
            std::uint32_t y = 0, x0 = 0, x1 = 0;
        };
        struct Region {
            std::uint64_t area = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
            std::int64_t l = 0, a = 0, b = 0;
            std::uint32_t x0 = 0, x1 = 0, y0 = 0, y1 = 0;
        };
        struct Corner {
            std::int32_t x = 0, y = 0;
        };
        struct Rectangle {
            Point axis{1.0F, 0.0F};
            Point center{};
            float length = 0.0F, thickness = 0.0F; // Center-to-center extents.
            bool degenerate = true;
        };

        bool Fill(
            const camera::FrameView& frame, const ColorTable& table, Rect window, ColorMask color,
            std::uint32_t seed_x, std::uint32_t seed_y, Region& region) noexcept;
        Rectangle MinimumRectangle(const Region& region) noexcept;
        Blob Measure(const Region& region, Rectangle box, Rect window, Color color,
                     const ColorTable& table) const noexcept;
        bool visited(std::uint32_t x, std::uint32_t y) const noexcept {
            return (visited_[y * words_ + x / 64U] >> (x % 64U)) & 1U;
        }
        void ClearVisited(Rect window) noexcept;

        std::unique_ptr<std::uint64_t[]> visited_{};
        std::unique_ptr<Span[]> spans_{};
        std::unique_ptr<std::uint32_t[]> row_min_{}, row_max_{};
        std::unique_ptr<Corner[]> points_{}, hull_{};
        std::unique_ptr<Blob[]> candidates_{};
        std::uint32_t max_width_ = 0, max_height_ = 0, words_ = 0;
        std::uint32_t max_spans_ = 0, max_candidates_ = 0, span_high_water_ = 0;
        float pixel_aspect_ = 1.0F;
    };

    struct Config {
        Workspace::Config workspace{};
        ColorTable::Config color{};
        Light::Config light{};
        Armor::Config armor{};
        Track::Config track{};
        Track::Window roi{};
    };

    explicit Detect(Config config) noexcept
        : config_(config), light_(config.light), armor_(config.armor) {}
    Detect() noexcept : Detect(Config{}) {}
    Detect(const Detect&) = delete;
    Detect& operator=(const Detect&) = delete;

    static bool Supports(camera::PixelFormat format) noexcept {
        return format == camera::PixelFormat::kRgb565;
    }
    // Validates the configuration and allocates every table and buffer for
    // frames up to the given geometry. The pixel aspect corrects lamp shape.
    std::expected<void, DetectionError> Prepare(const camera::FrameInfo& geometry) noexcept;

    // Pure calculation on a borrowed frame: the caller holds its lease and
    // owns Track. One Detect / Track pair serves one in-order frame stream.
    std::expected<DetectResult, DetectionError>
        update(const camera::FrameView& frame, Track& track) noexcept;

    std::uint32_t span_high_water() const noexcept { return workspace_.span_high_water(); }
    // Field paths as in Config, e.g. "light.min_area", "color.green.l_min",
    // "armor.colors" (bit mask: 1 green, 2 red, 4 blue).
    static bool set_parameter(Config& config, std::string_view name, double value) noexcept;

private:
    std::expected<void, DetectionError> measure(
        const camera::FrameView& frame, Rect window, const Track::Search& search, bool tracking,
        bool validate, const Track& track, DetectResult& result) noexcept;

    Config config_;
    Light light_;
    Armor armor_;
    ColorTable colors_{};
    Workspace workspace_{};
    camera::FrameInfo prepared_{};
};

// Per-cycle diagnostics are copied values. Pixels and leases are only borrowed
// during consume_frame, before Detector releases or the consumer moves the lease.
struct DetectorStatus {
    camera::AcquireStatus acquisition = camera::AcquireStatus::kNoFrame;
    std::optional<camera::CameraError> camera_error{}, release_error{};
    std::optional<DetectionError> detection_error{};
    std::chrono::microseconds acquire_time{}, detect_time{};
    std::uint32_t consecutive_acquire_errors = 0;
    std::uint32_t span_high_water = 0;
    bool vision_enabled = false;
    bool acquired_frame = false;
    bool released_frame = false;
};

class DetectorFactory final {
public:
    using FrameConsumer = std::function<void(camera::FrameLease&, const DetectResult*)>;

    // Every borrowed dependency must outlive the component. The target port is
    // write-only; binding it to Blackboard::target gives Detector sole publication.
    static std::expected<std::unique_ptr<executor::Component>, executor::Error> create(
        const executor::Component::Spec& spec, camera::ICamera& camera,
        executor::Output<Point> target, DetectorStatus& status, FrameConsumer consume_frame = {});
};

} // namespace dart::detection
