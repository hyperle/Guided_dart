module;
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

export module dart.detection.light;

export import dart.detection.blob_workspace;
import dart.detection.geometry;
import dart.detection.numeric;
import dart.detection.result;
import dart.detection.tracking;

export namespace dart::detection {

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
    Target Detect(std::span<const Blob> blobs, std::uint64_t frame_area,
                  const Blob** selected = nullptr) const noexcept;
    Target Track(std::span<const Blob> blobs, std::uint64_t frame_area,
                 const TargetTracker& tracker, const Target& predicted,
                 const Blob** selected = nullptr) const noexcept;
    static Target TargetFromBlob(const Blob& blob) noexcept {
        return {
            true, blob.center, blob.bounds, blob.area,
            std::sqrt(static_cast<float>(blob.area) / static_cast<float>(kPi)),
            blob.gray_quality * (blob.clipped ? 0.25F : 1.0F),
            blob.clipped, blob.contrast, blob.snr, blob.stability, 0.0F, 0.0F,
            blob.stability_checked};
    }

private:
    Config config_;
    bool Eligible(const Blob& blob, std::uint64_t frame_area) const noexcept {
        if (blob.bounds.width == 0 || blob.bounds.height == 0) return false;
        const float aspect = static_cast<float>(blob.bounds.width) / blob.bounds.height;
        const bool far = blob.area < config_.far_area_threshold
            || std::max(blob.bounds.width, blob.bounds.height) < config_.far_size_px;
        return blob.area >= config_.min_area
            && static_cast<float>(blob.area) <= static_cast<float>(frame_area) * config_.max_area_fraction
            && (far || (blob.fill >= config_.min_fill && blob.circularity >= config_.min_circularity
                && aspect >= config_.aspect_min && aspect <= config_.aspect_max));
    }
};

LightDetector::LightDetector() noexcept : LightDetector(Config{}) {}

LightDetector::LightDetector(Config config) noexcept
    : config_(config) {}

bool LightDetector::ValidConfig() const noexcept {
    return config_.min_area > 0 && config_.far_area_threshold > 0 && config_.far_size_px > 0
        && config_.search_step > 0 && IsFraction(config_.min_fill)
        && IsFraction(config_.min_circularity) && std::isfinite(config_.circularity_weight)
        && config_.circularity_weight >= 0.0F && IsPositive(config_.aspect_min)
        && IsPositive(config_.aspect_max) && config_.aspect_min <= config_.aspect_max
        && IsPositive(config_.max_area_fraction) && config_.max_area_fraction <= 1.0F;
}

Target
    LightDetector::Detect(std::span<const Blob> blobs, std::uint64_t frame_area,
                          const Blob** selected) const noexcept {
    const Blob* best = nullptr;
    float best_score = -1.0F;
    for (const auto& blob : blobs) {
        if (!Eligible(blob, frame_area)) continue;
        const float roundness = std::max(0.05F, blob.circularity);
        const float weight = config_.circularity_weight;
        const float shape = weight == 0.0F ? 1.0F
                          : weight == 1.0F ? roundness
                                           : std::pow(roundness, weight);
        const float score = static_cast<float>(blob.area) * shape * std::max(0.05F, blob.gray_quality);
        if (score > best_score) {
            best = &blob;
            best_score = score;
        }
    }
    if (selected) *selected = best;
    return best ? TargetFromBlob(*best) : Target{};
}

Target LightDetector::Track(
    std::span<const Blob> blobs, std::uint64_t frame_area, const TargetTracker& tracker,
    const Target& predicted, const Blob** selected) const noexcept {
    const Blob* best = nullptr;
    float cost = std::numeric_limits<float>::infinity();
    for (const auto& blob : blobs) {
        if (!Eligible(blob, frame_area)) continue;
        const float candidate = tracker.MatchCost(TargetFromBlob(blob), predicted)
                              + 0.05F * (1.0F - blob.gray_quality);
        if (candidate < cost) { best = &blob; cost = candidate; }
    }
    if (selected) *selected = best;
    return best ? TargetFromBlob(*best) : Target{};
}

class LightTracker {
public:
    explicit LightTracker(LightDetector::Config config,
                          TargetTracker::Config track = {}) noexcept;
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

LightTracker::LightTracker(LightDetector::Config config, TargetTracker::Config track) noexcept
    : config_(config), track_config_(track) {}

bool LightTracker::ValidConfig() const noexcept {
    return config_.confirm_frames > 0 && config_.far_confirm_frames > 0
        && config_.far_jump_min_px > 0 && std::isfinite(config_.far_jump_k)
        && config_.far_jump_k > 0.0F;
}

bool
    LightTracker::Confirm(Target& light, std::uint64_t frame_id, const Target& previous) noexcept {
    state_.measurement_valid = light.valid;
    const bool same_frame = state_.seen_frame && frame_id == state_.last_frame_id;
    // A same-frame rescan may supply the first valid observation, but two valid
    // scans of one frame count only once toward temporal confirmation.
    if (same_frame && (state_.input_valid || !light.valid))
        return state_.locked;
    state_.last_frame_id = frame_id;
    state_.seen_frame = true;
    state_.input_valid = light.valid;
    if (!light.valid) {
        state_.hits = 0;
        state_.last_measurement = {};
        state_.locked = false;
        return false;
    }
    const bool far = light.area < config_.far_area_threshold
                  || std::max(light.bounds.width, light.bounds.height) < config_.far_size_px;
    if (state_.last_measurement.valid && !std::isfinite(TargetTracker::MatchCost(
            light, state_.last_measurement, state_.last_measurement, track_config_))) {
        state_.hits = 0;
        state_.locked = false;
    }
    if (far && state_.last_measurement.valid) {
        const float jump = std::max(
            static_cast<float>(config_.far_jump_min_px),
            config_.far_jump_k
                * static_cast<float>(std::max(
                    state_.last_measurement.bounds.width, state_.last_measurement.bounds.height)));
        const float dx = light.center.x - state_.last_measurement.center.x;
        const float dy = light.center.y - state_.last_measurement.center.y;
        if (std::fabs(dx) > jump || std::fabs(dy) > jump) {
            state_.hits = 0;
            state_.locked = false;
        }
    }
    state_.last_measurement = light;
    IncrementSaturated(state_.hits);
    const auto needed = far ? config_.far_confirm_frames : config_.confirm_frames;
    if (!state_.locked && state_.hits >= needed)
        state_.locked = true;
    if (!state_.locked)
        return false;
    if (config_.deadband_px > 0 && previous.valid
        && std::fabs(light.center.x - previous.center.x) <= static_cast<float>(config_.deadband_px)
        && std::fabs(light.center.y - previous.center.y)
               <= static_cast<float>(config_.deadband_px)) {
        light.center = previous.center;
    }
    return true;
}

void LightTracker::ClearConfirmation() noexcept {
    state_.last_measurement = {};
    state_.hits = 0;
    state_.locked = false;
}

void LightTracker::Reset() noexcept { state_ = {}; }

} // namespace dart::detection
