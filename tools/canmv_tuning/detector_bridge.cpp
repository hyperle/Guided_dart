#include <cstdint>
#include <cstddef>
#include <chrono>
#include "detector_bridge.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <new>

#include "camera/c_api/camera.h"

import dart.detection.roi;

namespace dart::canmv {

using dart::detection::DetectionError;
using dart::detection::RoiDetector;

struct dart_tuning_detector {
    explicit dart_tuning_detector(RoiDetector::Config config) noexcept : detector(config) {}
    RoiDetector detector;
};

bool set_parameter(RoiDetector::Config& config, const char* name, double value) {
    // Parameter names are the actual native field paths; defaults come from
    // the native Config constructors. No detection rules are duplicated here.
#define UINT_FIELD(field) \
    if (std::strcmp(name, #field) == 0) { \
        if (value < 0 || value > UINT32_MAX || std::floor(value) != value) return false; \
        config.field = static_cast<std::uint32_t>(value); return true; \
    }
#define FLOAT_FIELD(field) \
    if (std::strcmp(name, #field) == 0) { \
        if (std::fabs(value) > std::numeric_limits<float>::max()) return false; \
        config.field = static_cast<float>(value); return true; \
    }
    if (std::strcmp(name, "threshold") == 0) {
        if (value < 0 || value > 255 || std::floor(value) != value) return false;
        config.threshold = static_cast<std::uint8_t>(value);
        return true;
    }
    FLOAT_FIELD(workspace.min_contrast)
    UINT_FIELD(workspace.background_margin)
    FLOAT_FIELD(workspace.min_snr)
    UINT_FIELD(workspace.stability_delta)
    FLOAT_FIELD(workspace.min_stability)
    UINT_FIELD(workspace.max_runs)
    UINT_FIELD(workspace.max_candidates)
    UINT_FIELD(light.min_area)
    UINT_FIELD(light.far_area_threshold)
    UINT_FIELD(light.far_size_px)
    UINT_FIELD(light.search_step)
    UINT_FIELD(light.confirm_frames)
    UINT_FIELD(light.far_confirm_frames)
    UINT_FIELD(light.far_jump_min_px)
    UINT_FIELD(light.deadband_px)
    FLOAT_FIELD(light.far_jump_k)
    FLOAT_FIELD(light.min_fill)
    FLOAT_FIELD(light.min_circularity)
    FLOAT_FIELD(light.circularity_weight)
    FLOAT_FIELD(light.aspect_min)
    FLOAT_FIELD(light.aspect_max)
    FLOAT_FIELD(light.max_area_fraction)
    UINT_FIELD(armor.length_floor)
    UINT_FIELD(armor.area_floor)
    UINT_FIELD(armor.plate_up_lo_pct)
    UINT_FIELD(armor.plate_up_hi_pct)
    UINT_FIELD(armor.plate_offx_pct)
    UINT_FIELD(armor.track_hold)
    UINT_FIELD(armor.confirm_frames)
    FLOAT_FIELD(armor.min_brightness_ratio)
    FLOAT_FIELD(armor.min_gap_contrast)
    FLOAT_FIELD(armor.length_min_k)
    FLOAT_FIELD(armor.length_max_k)
    FLOAT_FIELD(armor.tiny_px)
    FLOAT_FIELD(armor.aspect_min)
    FLOAT_FIELD(armor.fill_min)
    FLOAT_FIELD(armor.flat_deg)
    FLOAT_FIELD(armor.parallel_deg)
    FLOAT_FIELD(armor.thickness_ratio)
    FLOAT_FIELD(armor.length_ratio)
    FLOAT_FIELD(armor.overlap)
    FLOAT_FIELD(armor.gap_max_k)
    FLOAT_FIELD(armor.diagonal_min_sin)
    FLOAT_FIELD(armor.border_penalty)
    FLOAT_FIELD(armor.window_width_k)
    FLOAT_FIELD(armor.window_height_k)
    FLOAT_FIELD(armor.window_gap_k)
    FLOAT_FIELD(armor.window_min_side)
    FLOAT_FIELD(armor.near_k)
    FLOAT_FIELD(armor.near_min_px)
    FLOAT_FIELD(armor.span_min_k)
    FLOAT_FIELD(armor.span_max_k)
    FLOAT_FIELD(armor.association_k)
    FLOAT_FIELD(armor.association_min_px)
    UINT_FIELD(roi.min_width)
    UINT_FIELD(roi.min_height)
    UINT_FIELD(roi.reset_after_misses)
    UINT_FIELD(roi.full_scan_interval)
    FLOAT_FIELD(roi.size_k)
    FLOAT_FIELD(roi.margin)
    FLOAT_FIELD(roi.grow_per_miss)
    FLOAT_FIELD(roi.full_area_fraction)
    FLOAT_FIELD(track.association_k)
    FLOAT_FIELD(track.association_min_px)
    FLOAT_FIELD(track.min_size_ratio)
    FLOAT_FIELD(track.min_contrast_ratio)
    UINT_FIELD(track.validation_interval)
    FLOAT_FIELD(track.quality_drop_ratio)
    FLOAT_FIELD(track.velocity_alpha)
    UINT_FIELD(track.prediction_max_ms)
#undef UINT_FIELD
#undef FLOAT_FIELD
    if (std::strcmp(name, "roi.max_gap") == 0) {
        if (value <= 0 || value > UINT32_MAX || std::floor(value) != value) return false;
        config.roi.max_gap = std::chrono::milliseconds{static_cast<std::uint32_t>(value)};
        return true;
    }
    return false;
}

dart_tuning_rect rect(dart::detection::Rect value) {
    return {value.x, value.y, value.width, value.height};
}

dart_tuning_target target(const dart::detection::Target& value) {
    return {value.valid, value.center.x, value.center.y, rect(value.bounds), value.area,
            value.radius, value.quality, value.clipped, value.contrast, value.snr,
            value.stability, value.brightness_ratio, value.gap_contrast, value.stability_checked};
}

extern "C" int dart_tuning_detector_create(const dart_tuning_parameter* parameters, size_t count,
                                           dart_tuning_detector** out, size_t* bad_parameter) {
    *out = nullptr;
    *bad_parameter = count;
    RoiDetector::Config config{};
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(parameters[i].value)
            || !set_parameter(config, parameters[i].name, parameters[i].value)) {
            *bad_parameter = i;
            return 100;
        }
    }
    auto* value = new (std::nothrow) dart_tuning_detector(config);
    if (value == nullptr) return static_cast<int>(DetectionError::kNoMemory);
    const auto prepared = value->detector.Prepare(640, 480);
    if (!prepared) {
        const int error = static_cast<int>(prepared.error());
        delete value;
        return error;
    }
    *out = value;
    return 0;
}

extern "C" int dart_tuning_detector_detect(dart_tuning_detector* self, const uint8_t* pixels,
                                           size_t bytes, uint64_t sequence, uint64_t received_us,
                                           uint64_t source_pts, uint32_t flags,
                                           dart_tuning_detection* out) {
    if (self == nullptr || pixels == nullptr || bytes != 640u * 480u
        || received_us > static_cast<uint64_t>(INT64_MAX))
        return static_cast<int>(DetectionError::kInvalidFrame);
    dart::camera::FrameView view{};
    view.luma = {reinterpret_cast<const std::byte*>(pixels), bytes};
    view.width = view.stride = 640;
    view.height = 480;
    view.sequence = sequence;
    view.received = std::chrono::microseconds{received_us};
    view.source_pts = source_pts;
    view.source_pts_timebase_validated = (flags & DART_CAMERA_FRAME_SOURCE_PTS_UNVALIDATED) == 0;
    const auto result = self->detector.Detect(view);
    if (!result) return static_cast<int>(result.error());
    *out = {static_cast<int>(result->kind), target(result->target), target(result->light),
            target(result->armor), rect(result->scan.search_window), result->scan.full_frame,
            result->scan.rescanned, result->scan.candidates, result->missed_frames,
            self->detector.run_high_water(), result->scan.rejected_contrast,
            result->scan.rejected_snr, result->scan.rejected_stability,
            static_cast<int>(result->scan.phase), result->scan.full_validation,
            result->scan.stability_checks};
    return 0;
}

extern "C" void dart_tuning_detector_reset(dart_tuning_detector* self) {
    if (self != nullptr) self->detector.Reset();
}

extern "C" void dart_tuning_detector_destroy(dart_tuning_detector* self) { delete self; }

} // namespace dart::canmv
