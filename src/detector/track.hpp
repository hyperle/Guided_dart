#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <expected>
#include <limits>
#include <span>

#include "src/detector/target.hpp"

namespace dart::detection {

class Track final {
public:
    class Motion {
    public:
        struct Config {
            float association_k = 1.5F;
            float association_min_px = 6.0F;
            float min_size_ratio = 0.5F;
            float min_contrast_ratio = 0.5F;
            std::uint32_t validation_interval = 8;
            float quality_drop_ratio = 0.65F;
            float velocity_alpha = 0.5F;
            std::uint32_t prediction_max_ms = 50;
        };

        explicit Motion(Config config) noexcept
            : config_(config) {}
        static bool ValidConfig(const Config& c) noexcept;

        static float MatchCost(
            const Target& value, const Target& predicted, const Target& reference,
            const Config& c) noexcept;

        float MatchCost(const Target& value, const Target& predicted) const noexcept;

        Target Predict(std::chrono::microseconds now, std::uint32_t width, std::uint32_t height)
            const noexcept;

        void Observe(const Target& value, std::chrono::microseconds now) noexcept;

        void Reset() noexcept {
            last_ = {};
            received_ = {};
            velocity_ = {};
            velocity_valid_ = false;
        }
        const Target& last() const noexcept { return last_; }

    private:
        Config config_;
        Target last_{};
        std::chrono::microseconds received_{};
        Point velocity_{};
        bool velocity_valid_ = false;
    };
    class Confirmation {
    public:
        explicit Confirmation(Light::Config config) noexcept;
        Confirmation(Light::Config config, Motion::Config track) noexcept;
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

        Light::Config config_;
        Motion::Config track_config_;
        State state_{};
    };
    using Config = Motion::Config;
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

    struct Search {
        Rect window{};
        Target predicted_light{}, predicted_armor{};
        bool tracking = false;
        bool validate = false;
    };

    Track() noexcept
        : Track(Light::Config{}, Armor::Config{}, Config{}, Window{}) {}
    Track(Light::Config light, Armor::Config armor, Config motion, Window roi) noexcept
        : config_{light, armor, motion, roi}
        , armor_(armor)
        , light_tracker_(light, motion)
        , light_motion_(motion)
        , armor_motion_(motion) {}

    bool valid_config() const noexcept {
        return light_tracker_.ValidConfig() && Motion::ValidConfig(config_.track)
            && ValidRoiConfig();
    }

    std::expected<Search, DetectionError> plan(const camera::FrameView& frame) noexcept {
        if (frame_.valid && (frame.sequence <= frame_.sequence || frame.received < frame_.received))
            return std::unexpected(DetectionError::kOutOfOrderFrame);
        if (frame_.valid
            && (frame.width != frame_.width || frame.height != frame_.height
                || frame.received - frame_.received > config_.roi.max_gap))
            ResetTracking();
        Search search{};
        search.tracking = light_motion_.last().valid;
        search.predicted_light = light_motion_.Predict(frame.received, frame.width, frame.height);
        search.predicted_armor = armor_motion_.Predict(frame.received, frame.width, frame.height);
        search.validate = tracking_frames_ + 1 >= config_.track.validation_interval
                       || pending_armor_.valid || targets_.armor_misses > 0;
        search.window = NextWindow(
            frame.width, frame.height, search.predicted_light, search.predicted_armor,
            search.validate);
        return search;
    }

    Target select_light(
        const Light& rules, std::span<const Blob> blobs, std::uint64_t frame_area,
        const Target& predicted, const Blob** selected) const noexcept {
        const Blob* best = nullptr;
        float cost = std::numeric_limits<float>::infinity();
        for (const auto& blob : blobs) {
            if (!rules.Eligible(blob, frame_area))
                continue;
            const float candidate = light_motion_.MatchCost(Light::TargetFromBlob(blob), predicted)
                                  + 0.05F * (1.0F - blob.gray_quality);
            if (candidate < cost) {
                best = &blob;
                cost = candidate;
            }
        }
        if (selected)
            *selected = best;
        return best ? Light::TargetFromBlob(*best) : Target{};
    }

    bool matches_armor(const Target& value, const Target& predicted) const noexcept {
        return std::isfinite(armor_motion_.MatchCost(value, predicted));
    }
    bool quality_dropped(const Target& light, const Target& armor = {}) const noexcept {
        return QualityDropped(light, validated_light_) || QualityDropped(armor, validated_armor_);
    }

    void finish(DetectResult& result, const camera::FrameView& frame, bool tracking) noexcept {
        if (!tracking) {
            const bool armor_confirmed = ConfirmArmor(result.armor);
            if (!light_tracker_.Confirm(result.light, frame.sequence, {}))
                result.light = {};
            if (!result.light.valid || !armor_confirmed)
                result.armor = {};
        } else if (!armor_motion_.last().valid && !ConfirmArmor(result.armor))
            result.armor = {};
        UpdateTracking(result, tracking);
        // Motion estimation observes unsmoothed measurements; only publication has a deadband.
        if (result.light.valid && published_light_.valid && config_.light.deadband_px > 0
            && std::fabs(result.light.center.x - published_light_.center.x)
                   <= config_.light.deadband_px
            && std::fabs(result.light.center.y - published_light_.center.y)
                   <= config_.light.deadband_px)
            result.light.center = published_light_.center;
        published_light_ = result.light;
        result.select_target();
        frame_ = {frame.width, frame.height, frame.sequence, frame.received, true};
    }

    void Reset() noexcept;

private:
    struct Settings {
        Light::Config light;
        Armor::Config armor;
        Config track;
        Window roi;
    };
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

    void UpdateTracking(DetectResult& result, bool tracking) noexcept;
    bool ConfirmArmor(const Target& armor) noexcept;
    bool QualityDropped(const Target& value, const Target& validated) const noexcept;
    void ResetTracking() noexcept;

    Settings config_;
    Armor armor_;
    Confirmation light_tracker_;
    Motion light_motion_, armor_motion_;
    Target validated_light_{}, validated_armor_{}, pending_armor_{}, published_light_{};
    std::uint32_t armor_hits_ = 0, tracking_frames_ = 0;
    TargetHistory targets_{};
    FrameHistory frame_{};
    RoiState roi_state_{};
};

inline bool Track::Motion::ValidConfig(const Config& c) noexcept {
    return Numeric::IsPositive(c.association_k) && Numeric::IsPositive(c.association_min_px)
        && Numeric::IsPositive(c.min_size_ratio) && c.min_size_ratio <= 1.0F
        && Numeric::IsFraction(c.min_contrast_ratio) && c.validation_interval > 0
        && Numeric::IsFraction(c.quality_drop_ratio) && Numeric::IsPositive(c.velocity_alpha)
        && c.velocity_alpha <= 1.0F && c.prediction_max_ms > 0 && c.prediction_max_ms <= 1000;
}

inline float Track::Motion::MatchCost(
    const Target& value, const Target& predicted, const Target& reference,
    const Config& c) noexcept {
    const auto invalid = std::numeric_limits<float>::infinity();
    if (!value.valid || !predicted.valid || !reference.valid || value.bounds.width == 0
        || value.bounds.height == 0 || reference.bounds.width == 0 || reference.bounds.height == 0)
        return invalid;
    const float wr = static_cast<float>(value.bounds.width) / reference.bounds.width;
    const float hr = static_cast<float>(value.bounds.height) / reference.bounds.height;
    const float lo = std::min(value.contrast, reference.contrast);
    const float hi = std::max(value.contrast, reference.contrast);
    if (wr < c.min_size_ratio || wr > 1.0F / c.min_size_ratio || hr < c.min_size_ratio
        || hr > 1.0F / c.min_size_ratio || (hi > 0.0F && lo < c.min_contrast_ratio * hi))
        return invalid;
    const float tolerance = std::max(
        c.association_min_px,
        c.association_k * std::max(reference.bounds.width, reference.bounds.height));
    const float dx = value.center.x - predicted.center.x, dy = value.center.y - predicted.center.y;
    const float position = (dx * dx + dy * dy) / (tolerance * tolerance);
    if (!std::isfinite(position) || position > 1.0F)
        return invalid;
    return position + 0.1F * (std::fabs(1.0F - wr) + std::fabs(1.0F - hr))
         + (hi > 0.0F ? 0.1F * (1.0F - lo / hi) : 0.0F);
}

inline float Track::Motion::MatchCost(const Target& value, const Target& predicted) const noexcept {
    return MatchCost(value, predicted, last_, config_);
}

inline Target Track::Motion::Predict(
    std::chrono::microseconds now, std::uint32_t width, std::uint32_t height) const noexcept {
    if (!last_.valid || width == 0 || height == 0)
        return {};
    const auto elapsed = std::clamp<std::int64_t>(
        (now - received_).count(), 0, static_cast<std::int64_t>(config_.prediction_max_ms) * 1000);
    const float seconds = static_cast<float>(elapsed) * 1.0e-6F;
    const float limit =
        4.0F
        * std::max(
            config_.association_min_px,
            config_.association_k * std::max(last_.bounds.width, last_.bounds.height));
    const float dx = std::clamp(velocity_.x * seconds, -limit, limit);
    const float dy = std::clamp(velocity_.y * seconds, -limit, limit);
    auto predicted = last_;
    predicted.center = {
        std::clamp(last_.center.x + dx, 0.0F, static_cast<float>(width - 1)),
        std::clamp(last_.center.y + dy, 0.0F, static_cast<float>(height - 1))};
    predicted.bounds.width = std::min(last_.bounds.width, width);
    predicted.bounds.height = std::min(last_.bounds.height, height);
    predicted.bounds.x = static_cast<std::uint32_t>(std::clamp(
        std::round(last_.bounds.x + dx), 0.0F, static_cast<float>(width - predicted.bounds.width)));
    predicted.bounds.y = static_cast<std::uint32_t>(std::clamp(
        std::round(last_.bounds.y + dy), 0.0F,
        static_cast<float>(height - predicted.bounds.height)));
    return predicted;
}

inline void Track::Motion::Observe(const Target& value, std::chrono::microseconds now) noexcept {
    if (!value.valid)
        return;
    if (last_.valid && now > received_) {
        const float dt = static_cast<float>((now - received_).count()) * 1.0e-6F;
        const float alpha = velocity_valid_ ? config_.velocity_alpha : 1.0F;
        velocity_.x += alpha * ((value.center.x - last_.center.x) / dt - velocity_.x);
        velocity_.y += alpha * ((value.center.y - last_.center.y) / dt - velocity_.y);
        velocity_valid_ = true;
    }
    last_ = value;
    received_ = now;
}

inline Track::Confirmation::Confirmation(Light::Config config) noexcept
    : Confirmation(config, Motion::Config{}) {}

inline Track::Confirmation::Confirmation(Light::Config config, Track::Motion::Config track) noexcept
    : config_(config)
    , track_config_(track) {}

inline bool Track::Confirmation::ValidConfig() const noexcept {
    return config_.confirm_frames > 0 && config_.far_confirm_frames > 0
        && config_.far_jump_min_px > 0 && std::isfinite(config_.far_jump_k)
        && config_.far_jump_k > 0.0F;
}

inline bool Track::Confirmation::Confirm(
    Target& light, std::uint64_t frame_id, const Target& previous) noexcept {
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
    if (state_.last_measurement.valid
        && !std::isfinite(
            Track::Motion::MatchCost(
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
    Numeric::IncrementSaturated(state_.hits);
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

inline void Track::Confirmation::ClearConfirmation() noexcept {
    state_.last_measurement = {};
    state_.hits = 0;
    state_.locked = false;
}

inline void Track::Confirmation::Reset() noexcept { state_ = {}; }

inline void Track::ResetTracking() noexcept {
    targets_ = {};
    roi_state_ = {};
    light_tracker_.Reset();
    light_motion_.Reset();
    armor_motion_.Reset();
    validated_light_ = validated_armor_ = pending_armor_ = published_light_ = {};
    armor_hits_ = tracking_frames_ = 0;
}

inline void Track::Reset() noexcept {
    ResetTracking();
    frame_ = {};
}

inline bool Track::QualityDropped(const Target& value, const Target& validated) const noexcept {
    return value.valid && validated.valid
        && (value.contrast < validated.contrast * config_.track.quality_drop_ratio
            || value.snr < validated.snr * config_.track.quality_drop_ratio);
}

inline bool Track::ConfirmArmor(const Target& armor) noexcept {
    if (!armor.valid) {
        pending_armor_ = {};
        armor_hits_ = 0;
        return false;
    }
    if (!pending_armor_.valid
        || !std::isfinite(Motion::MatchCost(armor, pending_armor_, pending_armor_, config_.track)))
        armor_hits_ = 0;
    pending_armor_ = armor;
    Numeric::IncrementSaturated(armor_hits_);
    if (armor_hits_ < config_.armor.confirm_frames)
        return false;
    pending_armor_ = {};
    armor_hits_ = 0;
    return true;
}

inline void Track::UpdateTracking(DetectResult& result, bool tracking) noexcept {
    if (result.light.valid) {
        light_motion_.Observe(result.light, result.source.received);
        if (result.scan.full_validation) {
            validated_light_ = result.light;
            tracking_frames_ = 0;
        } else
            Numeric::IncrementSaturated(tracking_frames_);
        if (result.armor.valid) {
            armor_motion_.Observe(result.armor, result.source.received);
            targets_.armor_misses = 0;
            if (result.scan.full_validation)
                validated_armor_ = result.armor;
        } else if (armor_motion_.last().valid) {
            Numeric::IncrementSaturated(targets_.armor_misses);
            if (targets_.armor_misses >= config_.armor.track_hold) {
                validated_armor_ = {};
                armor_motion_.Reset();
                tracking_frames_ = config_.track.validation_interval - 1;
            }
        }
        roi_state_.missed_frames = 0;
    } else {
        Numeric::IncrementSaturated(roi_state_.missed_frames);
        if (!tracking && !light_tracker_.has_measurement())
            light_tracker_.ClearConfirmation();
        if (tracking && roi_state_.missed_frames >= config_.roi.reset_after_misses) {
            const auto misses = roi_state_.missed_frames;
            ResetTracking();
            roi_state_.missed_frames = misses;
        }
    }
    result.missed_frames = roi_state_.missed_frames;
    if (result.scan.full_frame)
        roi_state_.frames_since_full = 0;
    else
        Numeric::IncrementSaturated(roi_state_.frames_since_full);
}

inline bool Track::ValidRoiConfig() const noexcept {
    const auto& config = config_.roi;
    return config.min_width > 0 && config.min_height > 0 && std::isfinite(config.size_k)
        && config.size_k >= 1.0F && std::isfinite(config.margin) && config.margin >= 0.0F
        && std::isfinite(config.grow_per_miss) && config.grow_per_miss >= 0.0F
        && std::isfinite(config.full_area_fraction) && config.full_area_fraction > 0.0F
        && config.full_area_fraction <= 1.0F && config.reset_after_misses > 0
        && config.max_gap.count() > 0;
}

inline Rect Track::NextWindow(
    std::uint32_t width, std::uint32_t height, const Target& predicted_light,
    const Target& predicted_armor, bool validate) const noexcept {
    const auto& config = config_.roi;
    const auto& state = roi_state_;
    const bool has_light = predicted_light.valid;
    if (!has_light || state.missed_frames >= config.reset_after_misses
        || (config.full_scan_interval > 0
            && state.frames_since_full >= config.full_scan_interval)) {
        return Rect::full(width, height);
    }
    Rect envelope = predicted_light.bounds;
    if (predicted_armor.valid)
        envelope = envelope.unite(predicted_armor.bounds);
    else if (validate)
        envelope = envelope.unite(armor_.SearchWindow(predicted_light, width, height));
    const double grow = 1.0 + static_cast<double>(config.grow_per_miss) * state.missed_frames;
    const double rw = std::min<double>(
        width,
        std::max<double>(
            config.min_width,
            (envelope.width * static_cast<double>(config.size_k) + 2.0 * config.margin) * grow));
    const double rh = std::min<double>(
        height,
        std::max<double>(
            config.min_height,
            (envelope.height * static_cast<double>(config.size_k) + 2.0 * config.margin) * grow));
    const auto w = static_cast<std::uint32_t>(std::ceil(rw));
    const auto h = static_cast<std::uint32_t>(std::ceil(rh));
    if (static_cast<double>(w) * h
        >= static_cast<double>(width) * height * config.full_area_fraction) {
        return Rect::full(width, height);
    }
    // Translate the whole window into the frame, preserving its size at edges.
    const double cx = envelope.x + envelope.width * 0.5;
    const double cy = envelope.y + envelope.height * 0.5;
    const auto x = static_cast<std::uint32_t>(
        std::clamp(std::floor(cx - w * 0.5), 0.0, static_cast<double>(width - w)));
    const auto y = static_cast<std::uint32_t>(
        std::clamp(std::floor(cy - h * 0.5), 0.0, static_cast<double>(height - h)));
    return {x, y, w, h};
}

} // namespace dart::detection
