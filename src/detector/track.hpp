#pragma once

#include <chrono>
#include <cstdint>
#include <expected>

#include "camera/frame_view.hpp"
#include "src/detector/target.hpp"

namespace dart::detection {

// Temporal state of one Detect pipeline: confirmation, measured motion, ROI
// planning and loss handling. Predictions plan the search and association;
// they are never published as a measurement.
class Track final {
public:
    struct Config : Association::Config {
        std::uint32_t validation_interval = 8; // Tracking frames between armor searches.
        float quality_drop_ratio = 0.65F;      // Color quality vs. last full validation.
        float velocity_alpha = 0.5F;
        std::uint32_t prediction_max_ms = 50;
    };
    struct Window {
        std::uint32_t min_width = 24;
        std::uint32_t min_height = 64;
        float size_k = 2.6F;
        float margin = 12.0F;
        float grow_per_miss = 0.5F;
        float full_area_fraction = 0.75F;
        std::uint32_t reset_after_misses = 3;
        std::uint32_t full_scan_interval = 0;
        std::chrono::milliseconds max_gap{250}; // Receive-time gap that drops all history.
    };
    struct Search {
        Rect window{};
        Target predicted_light{}, predicted_armor{};
        bool tracking = false;
        bool validate = false;
        bool gap_reset = false;
    };

    // Measured-velocity extrapolation of one target.
    class Motion final {
    public:
        explicit Motion(const Config& config) noexcept : config_(config) {}
        Target Predict(std::chrono::microseconds now, std::uint32_t width, std::uint32_t height) const noexcept;
        void Observe(const Target& value, std::chrono::microseconds now) noexcept;
        void Reset() noexcept;
        const Target& last() const noexcept { return last_; }

    private:
        const Config& config_;
        Target last_{};
        std::chrono::microseconds received_{};
        Point velocity_{}; // px/ms
        bool velocity_valid_ = false;
    };

    Track() noexcept : Track(Light::Config{}, Armor::Config{}, Config{}, Window{}) {}
    Track(Light::Config light, Armor::Config armor, Config motion, Window roi) noexcept;
    Track(const Track&) = delete;
    Track& operator=(const Track&) = delete;

    bool valid_config() const noexcept;

    // Orders frames by sequence and receive time, then plans this frame's search.
    std::expected<Search, DetectionError> plan(const camera::FrameView& frame) noexcept;
    // Association cost of a light measurement with the tracked light.
    float light_cost(const Target& value, const Target& predicted) const noexcept {
        return Association::Cost(value, predicted, light_motion_.last(), config_.track);
    }
    const Target& armor_reference() const noexcept { return armor_motion_.last(); }
    const Config& association() const noexcept { return config_.track; }
    bool quality_dropped(const Target& light) const noexcept;
    // Confirms, updates history and selects the published target, once per frame.
    void finish(DetectResult& result, bool tracking) noexcept;
    // Drops target history after a failed frame; frame ordering is kept.
    void Abort() noexcept { ResetTracking(); }
    void Reset() noexcept;

private:
    struct Settings {
        Light::Config light;
        Armor::Config armor;
        Config track;
        Window roi;
    };
    struct FrameHistory {
        std::uint32_t width = 0, height = 0;
        std::uint64_t sequence = 0;
        std::chrono::microseconds received{};
        bool valid = false;
    };

    Rect NextWindow(
        std::uint32_t width, std::uint32_t height, const Target& predicted_light,
        const Target& predicted_armor, bool validate) const noexcept;
    bool ConfirmLight(const Target& light) noexcept;
    bool ConfirmArmor(const Target& armor) noexcept;
    void ResetTracking() noexcept;
    static DetectReason Reason(const DetectResult& result, const Target& measured_light,
                               const Target& measured_armor) noexcept;

    Settings config_;
    Light light_rules_;
    Motion light_motion_, armor_motion_;
    Target pending_light_{}, pending_armor_{};
    Target validated_light_{}, validated_armor_{}, published_light_{};
    std::uint32_t light_hits_ = 0, armor_hits_ = 0;
    std::uint32_t missed_frames_ = 0, armor_misses_ = 0;
    std::uint32_t frames_since_full_ = 0, tracking_frames_ = 0;
    FrameHistory frame_{};
};

} // namespace dart::detection
