#pragma once

#include <cstdint>

#include "detection/domain/geometry.hpp"
#include "detection/domain/tracking.hpp"

namespace dart::detection {

// 只包含跨实现稳定的运行指标；具体扫描器的 Trace 留在实现层。
struct DetectionTelemetry {
    uint64_t frames = 0;
    uint64_t hits = 0;
    uint64_t misses = 0;
    uint64_t candidates_last = 0;
    uint32_t scan_us_last = 0;
    uint32_t total_us_last = 0;
    TrackState state = TrackState::Startup;
    RoiWindow window{};
    TrackerCounters tracker{};
};

} // namespace dart::detection
