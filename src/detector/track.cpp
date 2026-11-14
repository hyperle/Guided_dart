#include "src/detector/track.hpp"

#include <algorithm>
#include <cmath>

namespace dart::detection {

Target Track::Motion::Predict(
    std::chrono::microseconds now, std::uint32_t width, std::uint32_t height) const noexcept {
    if (!last_.valid || width < last_.bounds.width || height < last_.bounds.height)
        return {};
    const float elapsed_ms = std::clamp(
        static_cast<float>((now - received_).count()) / 1000.0F, 0.0F,
        static_cast<float>(config_.prediction_max_ms));
    const float limit = 4.0F * std::max(
        config_.association_min_px,
        config_.association_k * static_cast<float>(std::max(last_.bounds.width, last_.bounds.height)));
    const float dx = std::clamp(velocity_.x * elapsed_ms, -limit, limit);
    const float dy = std::clamp(velocity_.y * elapsed_ms, -limit, limit);
    // Shift by whole pixels so the center stays consistent with the bounds.
    const auto x = static_cast<std::uint32_t>(std::clamp(
        std::nearbyint(static_cast<float>(last_.bounds.x) + dx), 0.0F,
        static_cast<float>(width - last_.bounds.width)));
    const auto y = static_cast<std::uint32_t>(std::clamp(
        std::nearbyint(static_cast<float>(last_.bounds.y) + dy), 0.0F,
        static_cast<float>(height - last_.bounds.height)));
    const float shift_x = static_cast<float>(x) - static_cast<float>(last_.bounds.x);
    const float shift_y = static_cast<float>(y) - static_cast<float>(last_.bounds.y);
    Target predicted = last_;
    predicted.bounds.x = x;
    predicted.bounds.y = y;
    predicted.center = {last_.center.x + shift_x, last_.center.y + shift_y};
    for (auto& bar : predicted.bars)
        bar = {bar.x + shift_x, bar.y + shift_y};
    return predicted;
}

void Track::Motion::Observe(const Target& value, std::chrono::microseconds now) noexcept {
    if (!value.valid)
        return;
    if (last_.valid && now > received_) {
        const float elapsed_ms = static_cast<float>((now - received_).count()) / 1000.0F;
        Point measured{(value.center.x - last_.center.x) / elapsed_ms,
                       (value.center.y - last_.center.y) / elapsed_ms};
        if (velocity_valid_) {
            const float alpha = config_.velocity_alpha;
            measured = {alpha * measured.x + (1.0F - alpha) * velocity_.x,
                        alpha * measured.y + (1.0F - alpha) * velocity_.y};
        }
        velocity_ = measured;
        velocity_valid_ = true;
    }
    last_ = value;
    received_ = now;
}

void Track::Motion::Reset() noexcept {
    last_ = {};
    received_ = {};
    velocity_ = {};
    velocity_valid_ = false;
}

Track::Track(Light::Config light, Armor::Config armor, Config motion, Window roi) noexcept
    : config_{light, armor, motion, roi}
    , light_rules_(light)
    , light_motion_(config_.track)
    , armor_motion_(config_.track) {}

bool Track::valid_config() const noexcept {
    const auto& t = config_.track;
    const auto& r = config_.roi;
    return light_rules_.ValidConfig() && Armor{config_.armor}.ValidConfig()
        && Association::ValidConfig(t) && t.validation_interval > 0
        && Numeric::IsFraction(t.quality_drop_ratio) && Numeric::IsPositive(t.velocity_alpha)
        && t.velocity_alpha <= 1.0F && t.prediction_max_ms > 0 && t.prediction_max_ms <= 1000
        && r.min_width > 0 && r.min_height > 0 && std::isfinite(r.size_k) && r.size_k >= 1.0F
        && std::isfinite(r.margin) && r.margin >= 0.0F && std::isfinite(r.grow_per_miss)
        && r.grow_per_miss >= 0.0F && Numeric::IsPositive(r.full_area_fraction)
        && r.full_area_fraction <= 1.0F && r.reset_after_misses > 0 && r.max_gap.count() > 0;
}

std::expected<Track::Search, DetectionError> Track::plan(const camera::FrameView& frame) noexcept {
    Search search{};
    if (frame_.valid) {
        if (frame.sequence <= frame_.sequence || frame.received < frame_.received) {
            ResetTracking();
            return std::unexpected(DetectionError::kOutOfOrderFrame);
        }
        if (frame.width != frame_.width || frame.height != frame_.height
            || frame.received - frame_.received > config_.roi.max_gap) {
            ResetTracking();
            search.gap_reset = true;
        }
    }
    frame_ = {frame.width, frame.height, frame.sequence, frame.received, true};
    search.tracking = light_motion_.last().valid;
    search.predicted_light = light_motion_.Predict(frame.received, frame.width, frame.height);
    search.predicted_armor = armor_motion_.Predict(frame.received, frame.width, frame.height);
    search.validate = !search.tracking
                   || tracking_frames_ + 1 >= config_.track.validation_interval
                   || pending_armor_.valid || armor_misses_ > 0;
    search.window = NextWindow(
        frame.width, frame.height, search.predicted_light, search.predicted_armor, search.validate);
    return search;
}

bool Track::quality_dropped(const Target& light) const noexcept {
    return light.valid && validated_light_.valid
        && light.color_quality < validated_light_.color_quality * config_.track.quality_drop_ratio;
}

Rect Track::NextWindow(
    std::uint32_t width, std::uint32_t height, const Target& predicted_light,
    const Target& predicted_armor, bool validate) const noexcept {
    const auto& c = config_.roi;
    const Rect full = Rect::full(width, height);
    if (!predicted_light.valid || missed_frames_ >= c.reset_after_misses
        || (c.full_scan_interval > 0 && frames_since_full_ >= c.full_scan_interval))
        return full;
    Rect bounds = predicted_light.bounds;
    if (predicted_armor.valid)
        bounds = bounds.unite(predicted_armor.bounds);
    else if (validate)
        bounds = bounds.unite(Armor::SearchWindow(config_.armor, predicted_light, width, height));
    const double grow = 1.0 + static_cast<double>(c.grow_per_miss) * missed_frames_;
    const auto side = [&](std::uint32_t extent, std::uint32_t minimum, std::uint32_t limit) {
        const double wanted = std::max<double>(
            minimum, (extent * static_cast<double>(c.size_k) + 2.0 * c.margin) * grow);
        return static_cast<std::uint32_t>(std::min<double>(limit, std::ceil(wanted)));
    };
    const auto w = side(bounds.width, c.min_width, width);
    const auto h = side(bounds.height, c.min_height, height);
    if (static_cast<double>(w) * h >= static_cast<double>(width) * height * c.full_area_fraction)
        return full;
    // Translate the whole window into the frame, preserving its size at edges.
    const double cx = bounds.x + bounds.width * 0.5, cy = bounds.y + bounds.height * 0.5;
    return {
        static_cast<std::uint32_t>(std::clamp(std::floor(cx - w * 0.5), 0.0, static_cast<double>(width - w))),
        static_cast<std::uint32_t>(std::clamp(std::floor(cy - h * 0.5), 0.0, static_cast<double>(height - h))),
        w, h};
}

bool Track::ConfirmLight(const Target& light) noexcept {
    if (!light.valid) {
        pending_light_ = pending_armor_ = {};
        light_hits_ = armor_hits_ = 0;
        return false;
    }
    const Target& previous = pending_light_;
    bool matched = std::isfinite(Association::Cost(light, previous, previous, config_.track));
    const bool far = light_rules_.Far(light);
    if (matched && far) {
        // Far lamps are too quantized for shape; a jump restarts confirmation.
        const float jump = std::max(
            static_cast<float>(config_.light.far_jump_min_px),
            config_.light.far_jump_k * static_cast<float>(std::max(previous.bounds.width, previous.bounds.height)));
        matched = std::fabs(light.center.x - previous.center.x) <= jump
               && std::fabs(light.center.y - previous.center.y) <= jump;
    }
    if (!matched) {
        light_hits_ = armor_hits_ = 0;
        pending_armor_ = {};
    }
    pending_light_ = light;
    const auto needed = far ? config_.light.far_confirm_frames : config_.light.confirm_frames;
    light_hits_ = std::min(needed, light_hits_ + 1);
    return light_hits_ >= needed;
}

bool Track::ConfirmArmor(const Target& armor) noexcept {
    if (!armor.valid) {
        pending_armor_ = {};
        armor_hits_ = 0;
        return false;
    }
    const float tolerance = Armor{config_.armor}.AssociationTolerance(armor);
    if (!std::isfinite(Association::Cost(armor, pending_armor_, pending_armor_, config_.track))
        || !Association::SameBars(armor, pending_armor_, tolerance))
        armor_hits_ = 0;
    pending_armor_ = armor;
    armor_hits_ = std::min(config_.armor.confirm_frames, armor_hits_ + 1);
    return armor_hits_ >= config_.armor.confirm_frames;
}

void Track::finish(DetectResult& result, bool tracking) noexcept {
    const Target measured_light = result.light, measured_armor = result.armor;
    Target& light = result.light;
    Target& armor = result.armor;
    if (!tracking) {
        const bool light_confirmed = ConfirmLight(light);
        const bool armor_confirmed = ConfirmArmor(armor);
        if (!light_confirmed)
            light = {};
        if (!light.valid || !armor_confirmed)
            armor = {};
    } else if (!armor_motion_.last().valid && !ConfirmArmor(armor)) {
        armor = {};
    }
    const auto received = result.source.received;
    if (light.valid) {
        light_motion_.Observe(light, received);
        missed_frames_ = 0;
        if (result.scan.full_validation) {
            validated_light_ = light;
            tracking_frames_ = 0;
        } else {
            Numeric::IncrementSaturated(tracking_frames_);
        }
        if (armor.valid) {
            armor_motion_.Observe(armor, received);
            validated_armor_ = armor;
            armor_misses_ = armor_hits_ = 0;
            pending_armor_ = {};
        } else if (armor_motion_.last().valid) {
            Numeric::IncrementSaturated(armor_misses_);
            if (armor_misses_ >= config_.armor.track_hold) {
                // Release the old pair; search every frame until a new one is confirmed.
                armor_motion_.Reset();
                validated_armor_ = pending_armor_ = {};
                armor_hits_ = 0;
                tracking_frames_ = config_.track.validation_interval - 1;
            }
        }
    } else {
        armor = {};
        if (tracking || !measured_light.valid) {
            pending_armor_ = {};
            armor_hits_ = 0;
        }
        Numeric::IncrementSaturated(missed_frames_);
        if (tracking && missed_frames_ >= config_.roi.reset_after_misses) {
            const auto misses = missed_frames_;
            ResetTracking();
            missed_frames_ = misses;
        }
    }
    // Motion observes measured centers; only the published light has a deadband.
    const auto deadband = static_cast<float>(config_.light.deadband_px);
    if (light.valid && published_light_.valid
        && std::fabs(light.center.x - published_light_.center.x) <= deadband
        && std::fabs(light.center.y - published_light_.center.y) <= deadband)
        light.center = published_light_.center;
    published_light_ = light;
    result.select_target();
    result.missed_frames = missed_frames_;
    result.light_hits = light_hits_;
    result.armor_hits = armor.valid ? config_.armor.confirm_frames : armor_hits_;
    result.reason = Reason(result, measured_light, measured_armor);
    if (result.scan.full_frame)
        frames_since_full_ = 0;
    else
        Numeric::IncrementSaturated(frames_since_full_);
}

DetectReason Track::Reason(
    const DetectResult& result, const Target& measured_light, const Target& measured_armor) noexcept {
    const auto& counters = result.scan.counters;
    if (!measured_light.valid) {
        if (counters.light_candidates == 0)
            return counters.rejected_color == 0 ? DetectReason::kNoLightColor
                                                : DetectReason::kLightColorQuality;
        return DetectReason::kLightRejected;
    }
    if (!result.light.valid)
        return DetectReason::kLightConfirm;
    if (result.armor.valid)
        return DetectReason::kArmorLocked;
    if (measured_armor.valid)
        return DetectReason::kArmorConfirm;
    if (!result.scan.armor_scanned)
        return DetectReason::kLightLocked;
    return counters.armor_candidates == 0 ? DetectReason::kNoArmorColor : DetectReason::kArmorRejected;
}

void Track::ResetTracking() noexcept {
    light_motion_.Reset();
    armor_motion_.Reset();
    pending_light_ = pending_armor_ = {};
    validated_light_ = validated_armor_ = published_light_ = {};
    light_hits_ = armor_hits_ = 0;
    missed_frames_ = armor_misses_ = 0;
    frames_since_full_ = tracking_frames_ = 0;
}

void Track::Reset() noexcept {
    ResetTracking();
    frame_ = {};
}

} // namespace dart::detection
