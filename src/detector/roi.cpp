#include "src/detector/roi.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <expected>
#include <limits>

#include "src/detector/armor.hpp"
#include "src/detector/blob_workspace.hpp"
#include "src/detector/geometry.hpp"
#include "src/detector/interface.hpp"
#include "src/detector/light.hpp"
#include "src/detector/numeric.hpp"
#include "src/detector/tracking.hpp"

namespace dart::detection {

RoiDetector::RoiDetector() noexcept
    : RoiDetector(Config{}) {}

RoiDetector::RoiDetector(Config config) noexcept
    : config_(config)
    , light_(config.light)
    , armor_(config.armor)
    , light_tracker_(config.light, config.track)
    , light_motion_(config.track)
    , armor_motion_(config.track) {}

std::expected<void, DetectionError>
    RoiDetector::Prepare(std::uint32_t max_width, std::uint32_t max_height) noexcept {
    if (!light_.ValidConfig() || !light_tracker_.ValidConfig() || !armor_.ValidConfig()
        || !TargetTracker::ValidConfig(config_.track) || !ValidRoiConfig()) {
        return std::unexpected(DetectionError::kInvalidConfig);
    }
    auto ready = workspace_.Prepare(max_width, max_height, config_.workspace);
    if (!ready)
        return ready;
    Reset();
    return {};
}

std::expected<DetectResult, DetectionError>
    RoiDetector::Detect(const camera::FrameView& frame) noexcept {
    auto valid = workspace_.Validate(frame);
    if (!valid)
        return std::unexpected(valid.error());
    if (frame_.valid && (frame.sequence <= frame_.sequence || frame.received < frame_.received)) {
        return std::unexpected(DetectionError::kOutOfOrderFrame);
    }
    if (frame_.valid
        && (frame.width != frame_.width || frame.height != frame_.height
            || frame.received - frame_.received > config_.roi.max_gap)) {
        ResetTracking();
    }
    DetectResult result{};
    result.source = {
        frame.sequence, frame.source_pts, frame.received, frame.source_pts_timebase_validated};
    const bool tracking = light_motion_.last().valid;
    const auto predicted_light = light_motion_.Predict(frame.received, frame.width, frame.height);
    const auto predicted_armor = armor_motion_.Predict(frame.received, frame.width, frame.height);
    const bool validate = tracking_frames_ + 1 >= config_.track.validation_interval
                       || pending_armor_.valid || targets_.armor_misses > 0;
    const Rect window =
        NextWindow(frame.width, frame.height, predicted_light, predicted_armor, validate);
    auto measured =
        Measure(frame, window, result, tracking, validate, predicted_light, predicted_armor);
    if (!measured)
        return std::unexpected(measured.error());
    if (tracking && !result.scan.full_frame
        && (!result.light.valid || result.light.clipped || result.armor.clipped)) {
        // The recovery scan still associates with the original target. A new
        // identity cannot inherit its lock through a same-frame rescan.
        result = {};
        result.source = {
            frame.sequence, frame.source_pts, frame.received, frame.source_pts_timebase_validated};
        result.scan.rescanned = true;
        measured = Measure(
            frame, FullWindow(frame.width, frame.height), result, true, true, predicted_light,
            predicted_armor);
        if (!measured)
            return std::unexpected(measured.error());
    }
    if (!tracking) {
        const bool armor_confirmed = ConfirmArmor(result.armor);
        if (!light_tracker_.Confirm(result.light, frame.sequence, {}))
            result.light = {};
        if (!result.light.valid || !armor_confirmed)
            result.armor = {};
    } else if (!armor_motion_.last().valid) {
        if (!ConfirmArmor(result.armor))
            result.armor = {};
    }
    result.target = result.armor.valid ? result.armor : result.light;
    result.kind = result.armor.valid ? TargetKind::kArmor
                : result.light.valid ? TargetKind::kLight
                                     : TargetKind::kNone;
    UpdateTracking(result, tracking);
    // Stabilize published coordinates only; motion estimation always sees the
    // unsmoothed current measurement, so a slow-moving target is not frozen.
    if (result.light.valid && published_light_.valid && config_.light.deadband_px > 0
        && std::fabs(result.light.center.x - published_light_.center.x) <= config_.light.deadband_px
        && std::fabs(result.light.center.y - published_light_.center.y)
               <= config_.light.deadband_px) {
        result.light.center = published_light_.center;
        if (result.kind == TargetKind::kLight)
            result.target.center = result.light.center;
    }
    published_light_ = result.light;
    frame_ = {frame.width, frame.height, frame.sequence, frame.received, true};
    return result;
}

void RoiDetector::ResetTracking() noexcept {
    targets_ = {};
    roi_state_ = {};
    light_tracker_.Reset();
    light_motion_.Reset();
    armor_motion_.Reset();
    validated_light_ = validated_armor_ = pending_armor_ = published_light_ = {};
    armor_hits_ = tracking_frames_ = 0;
}

void RoiDetector::Reset() noexcept {
    ResetTracking();
    frame_ = {};
}

bool RoiDetector::QualityDropped(const Target& value, const Target& validated) const noexcept {
    return value.valid && validated.valid
        && (value.contrast < validated.contrast * config_.track.quality_drop_ratio
            || value.snr < validated.snr * config_.track.quality_drop_ratio);
}

bool RoiDetector::ConfirmArmor(const Target& armor) noexcept {
    if (!armor.valid) {
        pending_armor_ = {};
        armor_hits_ = 0;
        return false;
    }
    if (!pending_armor_.valid
        || !std::isfinite(
            TargetTracker::MatchCost(armor, pending_armor_, pending_armor_, config_.track)))
        armor_hits_ = 0;
    pending_armor_ = armor;
    IncrementSaturated(armor_hits_);
    if (armor_hits_ < config_.armor.confirm_frames)
        return false;
    pending_armor_ = {};
    armor_hits_ = 0;
    return true;
}

void RoiDetector::UpdateTracking(DetectResult& result, bool tracking) noexcept {
    if (result.light.valid) {
        light_motion_.Observe(result.light, result.source.received);
        if (result.scan.full_validation) {
            validated_light_ = result.light;
            tracking_frames_ = 0;
        } else
            IncrementSaturated(tracking_frames_);
        if (result.armor.valid) {
            armor_motion_.Observe(result.armor, result.source.received);
            targets_.armor_misses = 0;
            if (result.scan.full_validation)
                validated_armor_ = result.armor;
        } else if (armor_motion_.last().valid) {
            IncrementSaturated(targets_.armor_misses);
            if (targets_.armor_misses >= config_.armor.track_hold) {
                validated_armor_ = {};
                armor_motion_.Reset();
                tracking_frames_ = config_.track.validation_interval - 1;
            }
        }
        roi_state_.missed_frames = 0;
    } else {
        IncrementSaturated(roi_state_.missed_frames);
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
        IncrementSaturated(roi_state_.frames_since_full);
}

std::expected<void, DetectionError> RoiDetector::Measure(
    const camera::FrameView& frame, Rect window, DetectResult& result, bool tracking, bool validate,
    const Target& predicted_light, const Target& predicted_armor) noexcept {
    result.scan.phase = tracking ? DetectionPhase::kTrack : DetectionPhase::kDetect;
    result.scan.search_window = window;
    result.scan.full_frame = IsFullWindow(window, frame.width, frame.height);
    const bool sampled_acquire =
        !tracking && result.scan.full_frame && config_.light.search_step > 1;
    auto blobs = workspace_.Scan(
        frame, window, config_.threshold,
        std::min(config_.light.min_area, config_.armor.area_floor),
        sampled_acquire ? config_.light.search_step : 1, !tracking);
    if (!blobs)
        return std::unexpected(blobs.error());
    const auto collect_stats = [&] {
        result.scan.candidates += static_cast<std::uint32_t>(blobs->size());
        result.scan.rejected_contrast += workspace_.rejected_contrast();
        result.scan.rejected_snr += workspace_.rejected_snr();
        result.scan.rejected_stability += workspace_.rejected_stability();
        result.scan.stability_checks += workspace_.stability_checks();
    };
    const Blob* selected_light = nullptr;
    const auto frame_area = static_cast<std::uint64_t>(frame.width) * frame.height;
    result.light =
        tracking ? light_.Track(*blobs, frame_area, light_motion_, predicted_light, &selected_light)
                 : light_.Detect(*blobs, frame_area, &selected_light);
    Blob light_blob = selected_light ? *selected_light : Blob{};
    validate = !tracking || validate || QualityDropped(result.light, validated_light_);
    std::array<Blob, 2> pair{};
    const bool associate_armor = tracking && predicted_armor.valid;
    if (sampled_acquire && result.light.valid) {
        collect_stats();
        const auto armor_window = armor_.SearchWindow(result.light, frame.width, frame.height);
        if (armor_window.width > 0 && armor_window.height > 0) {
            blobs = workspace_.Scan(
                frame, armor_window, config_.threshold,
                std::min(config_.light.min_area, config_.armor.area_floor), 1, true);
            if (!blobs)
                return std::unexpected(blobs.error());
            result.armor = armor_.Detect(*blobs, result.light, nullptr, &frame, false, &pair);
        } else {
            // No second scan was made; its statistics were already collected.
            result.scan.full_validation = true;
            return {};
        }
    } else if (result.light.valid && (associate_armor || validate)) {
        result.armor = armor_.Detect(
            *blobs, result.light, associate_armor ? &predicted_armor : nullptr, &frame,
            associate_armor, &pair);
        if (associate_armor
            && !std::isfinite(armor_motion_.MatchCost(result.armor, predicted_armor)))
            result.armor = {};
    }
    validate = validate || QualityDropped(result.armor, validated_armor_);
    result.scan.full_validation = validate;
    if (tracking && validate && result.light.valid) {
        auto accepted = workspace_.ValidateStability(frame, light_blob, config_.threshold, window);
        if (!accepted)
            return std::unexpected(accepted.error());
        result.light = *accepted ? LightDetector::TargetFromBlob(light_blob) : Target{};
        if (!result.light.valid)
            result.armor = {};
        if (result.armor.valid) {
            bool pair_ok = true;
            for (auto& bar : pair) {
                auto stable = workspace_.ValidateStability(frame, bar, config_.threshold, window);
                if (!stable)
                    return std::unexpected(stable.error());
                pair_ok = pair_ok && *stable;
            }
            result.armor = pair_ok
                             ? armor_.Detect(
                                   pair, result.light, associate_armor ? &predicted_armor : nullptr,
                                   &frame, associate_armor)
                             : Target{};
        }
    }
    collect_stats();
    return {};
}

bool RoiDetector::ValidRoiConfig() const noexcept {
    const auto& config = config_.roi;
    return config.min_width > 0 && config.min_height > 0 && std::isfinite(config.size_k)
        && config.size_k >= 1.0F && std::isfinite(config.margin) && config.margin >= 0.0F
        && std::isfinite(config.grow_per_miss) && config.grow_per_miss >= 0.0F
        && std::isfinite(config.full_area_fraction) && config.full_area_fraction > 0.0F
        && config.full_area_fraction <= 1.0F && config.reset_after_misses > 0
        && config.max_gap.count() > 0;
}

Rect RoiDetector::NextWindow(
    std::uint32_t width, std::uint32_t height, const Target& predicted_light,
    const Target& predicted_armor, bool validate) const noexcept {
    const auto& config = config_.roi;
    const auto& state = roi_state_;
    const bool has_light = predicted_light.valid;
    if (!has_light || state.missed_frames >= config.reset_after_misses
        || (config.full_scan_interval > 0
            && state.frames_since_full >= config.full_scan_interval)) {
        return FullWindow(width, height);
    }
    Rect envelope = predicted_light.bounds;
    if (predicted_armor.valid)
        envelope = UniteWindows(envelope, predicted_armor.bounds);
    else if (validate)
        envelope = UniteWindows(envelope, armor_.SearchWindow(predicted_light, width, height));
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
        return FullWindow(width, height);
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
