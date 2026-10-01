#pragma once

#include <cstddef>
#include <cstdint>

#include "detection/domain/observations.hpp"
#include "detection/domain/tracking.hpp"

namespace dart::detection {

// 由采集层提供的最小帧上下文。跟踪器不需要知道 FramePool 或像素内存。
struct FrameContext {
    uint64_t mono_us = 0;
    uint32_t width = 0;
    uint32_t height = 0;
};

enum class ScanMode : uint8_t {
    FullFrame = 0,
    Roi = 1,
};

struct ScanPlan {
    TrackState state = TrackState::Startup;
    ScanMode  mode = ScanMode::FullFrame;
    RoiWindow window{};
    bool      kf_ready = false;
    float     pred_x = 0.0f;
    float     pred_y = 0.0f;
    float     pred_s = 0.0f;

    bool full_scan() const { return mode == ScanMode::FullFrame; }
};

struct ScanObservation {
    ScanMode mode = ScanMode::FullFrame;
    const Blip *cands = nullptr;
    size_t count = 0;
    TargetMeasurement measurement{};
    uint32_t scan_us = 0;
};

} // namespace dart::detection
