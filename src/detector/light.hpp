#pragma once

#include <cstdint>
#include <span>

#include "src/detector/blob_workspace.hpp"
#include "src/detector/geometry.hpp"
#include "src/detector/numeric.hpp"
#include "src/detector/result.hpp"
#include "src/detector/tracking.hpp"

namespace dart::detection {
class LightDetector {
public:
    struct Config {
        std::uint32_t min_area = 3;
        std::uint32_t far_area_threshold = 30;
        std::uint32_t far_size_px = 8;
        // Acquisition uses sampling; locked ROI tracking scans at step 1.
        std::uint32_t search_step = 2;
        std::uint32_t confirm_frames = 2;
        std::uint32_t far_confirm_frames = 3;
        float far_jump_k = 1.5F;
        std::uint32_t far_jump_min_px = 6;
        std::uint32_t deadband_px = 2;
        float min_fill = 0.5F;
        float min_circularity = 0.0F;
        float circularity_weight = 1.0F;
        float aspect_min = 0.7F;
        float aspect_max = 1.4F;
        float max_area_fraction = 0.25F;
    };
    LightDetector() noexcept;
    explicit LightDetector(Config config) noexcept;
    bool ValidConfig() const noexcept;
    Target Detect(
        std::span<const Blob> blobs, std::uint64_t frame_area,
        const Blob** selected = nullptr) const noexcept;
    Target Track(
        std::span<const Blob> blobs, std::uint64_t frame_area, const TargetTracker& tracker,
        const Target& predicted, const Blob** selected = nullptr) const noexcept;
    static Target TargetFromBlob(const Blob& blob) noexcept;

private:
    Config config_;
    bool Eligible(const Blob& blob, std::uint64_t frame_area) const noexcept;
};

class LightTracker {
public:
    explicit LightTracker(LightDetector::Config config, TargetTracker::Config track = {}) noexcept;
    bool ValidConfig() const noexcept;
    bool Confirm(Target& light, std::uint64_t frame_id, const Target& previous) noexcept;
    void ClearConfirmation() noexcept;
    void Reset() noexcept;
    bool has_measurement() const noexcept { return state_.measurement_valid; }

private:
    struct State {
        Target last_measurement{};
        std::uint32_t hits = 0;
        std::uint64_t last_frame_id = 0;
        bool seen_frame = false;
        bool input_valid = false;
        bool measurement_valid = false;
        bool locked = false;
    };

    LightDetector::Config config_;
    TargetTracker::Config track_config_;
    State state_{};
};

} // namespace dart::detection
