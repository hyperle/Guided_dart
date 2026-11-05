#include <cstddef>
#include <chrono>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

#include "tools/canmv_tuning/detector_bridge.h"

#include "src/detector/roi.hpp"

int main() {
    using dart::detection::RoiDetector;
    using dart::detection::DetectionError;
    using dart::canmv::dart_tuning_parameter;
    using dart::canmv::dart_tuning_detector;
    using dart::canmv::dart_tuning_detection;
    using dart::canmv::dart_tuning_detector_create;
    const dart_tuning_parameter parameters[] = {
        {"light.search_step", 1}, {"light.confirm_frames", 1}, {"light.far_confirm_frames", 1},
        {"threshold", 80}, {"workspace.min_contrast", 16}, {"workspace.background_margin", 4},
        {"workspace.min_snr", 4}, {"workspace.stability_delta", 6},
        {"workspace.min_stability", 0.65}, {"armor.min_brightness_ratio", 0.6},
        {"armor.min_gap_contrast", 12}, {"armor.confirm_frames", 1},
        {"track.association_k", 1.25}, {"track.association_min_px", 5},
        {"track.min_size_ratio", 0.6}, {"track.min_contrast_ratio", 0.55},
        {"track.validation_interval", 3}, {"track.quality_drop_ratio", 0.7},
        {"track.velocity_alpha", 0.6}, {"track.prediction_max_ms", 40}};
    dart_tuning_detector* bridge = nullptr;
    size_t bad = 0;
    assert(dart_tuning_detector_create(parameters, std::size(parameters), &bridge, &bad) == 0);
    RoiDetector::Config config{};
    config.threshold = 80;
    config.workspace.min_contrast = 16;
    config.workspace.background_margin = 4;
    config.workspace.min_snr = 4;
    config.workspace.stability_delta = 6;
    config.workspace.min_stability = 0.65F;
    config.armor.min_brightness_ratio = 0.6F;
    config.armor.min_gap_contrast = 12;
    config.armor.confirm_frames = 1;
    config.track = {1.25F, 5, 0.6F, 0.55F, 3, 0.7F, 0.6F, 40};
    config.light.search_step = config.light.confirm_frames = config.light.far_confirm_frames = 1;
    RoiDetector native(config);
    assert(native.Prepare(640, 480));
    std::vector<std::byte> pixels(640 * 480, std::byte{40});
    auto box = [&](int x, int y, int w, int h) {
        for (int row = y; row < y + h; ++row)
            for (int col = x; col < x + w; ++col) pixels[row * 640 + col] = std::byte{180};
    };
    box(300, 300, 12, 12); // Light anchor and two bars above it.
    box(290, 270, 2, 20);
    box(310, 270, 2, 20);
    dart_tuning_detection result{};
    for (unsigned sequence = 1; sequence <= 3; ++sequence) {
        if (sequence == 3) std::fill(pixels.begin(), pixels.end(), std::byte{0});
        dart::camera::FrameView frame{};
        frame.luma = pixels;
        frame.width = frame.stride = 640;
        frame.height = 480;
        frame.sequence = sequence;
        frame.received = std::chrono::microseconds{sequence * 10000};
        const auto expected = native.Detect(frame);
        assert(expected);
        assert(dart_tuning_detector_detect(bridge, reinterpret_cast<const uint8_t*>(pixels.data()),
                                          pixels.size(), sequence, sequence * 10000, 0, 1,
                                          &result) == 0);
        assert(result.kind == static_cast<int>(expected->kind));
        assert(result.light.valid == expected->light.valid);
        assert(result.armor.valid == expected->armor.valid);
        assert(result.target.area == expected->target.area);
        assert(result.target.quality == expected->target.quality);
        assert(result.target.contrast == expected->target.contrast);
        assert(result.target.snr == expected->target.snr);
        assert(result.target.stability == expected->target.stability);
        assert(result.target.brightness_ratio == expected->target.brightness_ratio);
        assert(result.target.gap_contrast == expected->target.gap_contrast);
        assert(result.rejected_snr == expected->scan.rejected_snr);
        assert(result.rejected_stability == expected->scan.rejected_stability);
        assert(result.phase == static_cast<int>(expected->scan.phase));
        assert(result.full_validation == expected->scan.full_validation);
        assert(result.stability_checks == expected->scan.stability_checks);
        assert(result.target.stability_checked == expected->target.stability_checked);
        assert(std::fabs(result.target.x - expected->target.center.x) < 0.0001F);
        assert(std::fabs(result.target.y - expected->target.center.y) < 0.0001F);
        assert(result.window.x == expected->scan.search_window.x);
        assert(result.window.y == expected->scan.search_window.y);
        assert(result.window.width == expected->scan.search_window.width);
        assert(result.window.height == expected->scan.search_window.height);
        assert(result.candidates == expected->scan.candidates);
        assert(result.full_frame == expected->scan.full_frame);
        assert(result.rescanned == expected->scan.rescanned);
        if (sequence == 1) assert(result.armor.valid && result.kind == 2);
        if (sequence == 3) assert(!result.target.valid && result.kind == 0);
    }
    const auto* data = reinterpret_cast<const uint8_t*>(pixels.data());
    assert(dart_tuning_detector_detect(bridge, data, pixels.size(), 3, 30000, 0, 1, &result)
           == static_cast<int>(DetectionError::kOutOfOrderFrame));
    dart_tuning_detector_reset(bridge);
    assert(dart_tuning_detector_detect(bridge, data, pixels.size(), 1, 10000, 0, 1, &result) == 0);
    dart_tuning_detector_destroy(bridge);
    const dart_tuning_parameter bad_name{"light.misspelling", 1};
    assert(dart_tuning_detector_create(&bad_name, 1, &bridge, &bad) == 100);
    assert(bridge == nullptr && bad == 0);
    const dart_tuning_parameter bad_integer{"light.min_area", 3.5};
    assert(dart_tuning_detector_create(&bad_integer, 1, &bridge, &bad) == 100);
    const dart_tuning_parameter bad_threshold{"threshold", 256};
    assert(dart_tuning_detector_create(&bad_threshold, 1, &bridge, &bad) == 100);
    const dart_tuning_parameter bad_contrast{"workspace.min_contrast", -1};
    assert(dart_tuning_detector_create(&bad_contrast, 1, &bridge, &bad)
           == static_cast<int>(DetectionError::kInvalidConfig));
    const dart_tuning_parameter bad_config{"workspace.max_runs", 0};
    assert(dart_tuning_detector_create(&bad_config, 1, &bridge, &bad)
           == static_cast<int>(DetectionError::kInvalidConfig));
}
