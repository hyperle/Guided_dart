#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <expected>

#include "src/detector/armor.hpp"
#include "src/detector/blob_workspace.hpp"
#include "src/detector/geometry.hpp"
#include "src/detector/interface.hpp"
#include "src/detector/light.hpp"
#include "src/detector/numeric.hpp"
#include "src/detector/tracking.hpp"

namespace dart::detection {
class RoiDetector final : public IDetector {
public:
    struct Config {
        struct Window {
            std::uint32_t min_width = 24;
            std::uint32_t min_height = 64;
            float size_k = 2.6F;
            float margin = 12.0F;
            float grow_per_miss = 0.5F;
            float full_area_fraction = 0.75F;
            std::uint32_t reset_after_misses = 3;
            std::uint32_t full_scan_interval = 0;
            std::chrono::milliseconds max_gap{250};
        };
        std::uint8_t threshold = 50;
        BlobWorkspace::Config workspace{};
        LightDetector::Config light{};
        ArmorDetector::Config armor{};
        TargetTracker::Config track{};
        Window roi{};
    };
    RoiDetector() noexcept;
    explicit RoiDetector(Config config) noexcept;
    RoiDetector(const RoiDetector&) = delete;
    RoiDetector& operator=(const RoiDetector&) = delete;

    std::expected<void, DetectionError>
        Prepare(std::uint32_t max_width, std::uint32_t max_height) noexcept;
    std::expected<DetectResult, DetectionError>
        Detect(const camera::FrameView& frame) noexcept override;
    void Reset() noexcept override;
    std::uint32_t run_high_water() const noexcept { return workspace_.run_high_water(); }

private:
    struct RoiState {
        std::uint32_t missed_frames = 0;
        std::uint32_t frames_since_full = 0;
    };

    bool ValidRoiConfig() const noexcept;
    Rect NextWindow(
        std::uint32_t width, std::uint32_t height, const Target& predicted_light,
        const Target& predicted_armor, bool validate) const noexcept;

    struct FrameHistory {
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint64_t sequence = 0;
        std::chrono::microseconds received{};
        bool valid = false;
    };

    struct TargetHistory {
        std::uint32_t armor_misses = 0;
    };

    std::expected<void, DetectionError> Measure(
        const camera::FrameView& frame, Rect window, DetectResult& result, bool tracking,
        bool validate, const Target& predicted_light, const Target& predicted_armor) noexcept;
    void UpdateTracking(DetectResult& result, bool tracking) noexcept;
    bool ConfirmArmor(const Target& armor) noexcept;
    bool QualityDropped(const Target& value, const Target& validated) const noexcept;
    void ResetTracking() noexcept;

    Config config_;
    LightDetector light_;
    ArmorDetector armor_;
    BlobWorkspace workspace_;
    LightTracker light_tracker_;
    TargetTracker light_motion_, armor_motion_;
    Target validated_light_{}, validated_armor_{}, pending_armor_{}, published_light_{};
    std::uint32_t armor_hits_ = 0, tracking_frames_ = 0;
    TargetHistory targets_{};
    FrameHistory frame_{};
    RoiState roi_state_{};
};

} // namespace dart::detection
