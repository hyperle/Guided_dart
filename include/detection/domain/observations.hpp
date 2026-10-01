#pragma once

#include <cstddef>
#include <cstdint>

#include "core/frame.hpp"
#include "detection/domain/geometry.hpp"

namespace dart::detection {

struct Blip {
    float    cx = 0.0f;
    float    cy = 0.0f;
    uint32_t area = 0;
    float    radius = 0.0f;
    uint32_t x0 = 0;
    uint32_t y0 = 0;
    uint32_t x1 = 0;
    uint32_t y1 = 0;
    float    fill = 0.0f;
    float    circularity = 0.0f;
    float    continuity = 0.0f;
    float    score = 0.0f;
    bool     border = false;
};

struct TargetMeasurement {
    bool     valid = false;
    float    cx = 0.0f;
    float    cy = 0.0f;
    float    radius = 0.0f;
    uint32_t area = 0;
    float    fill = 0.0f;
    float    circularity = 0.0f;
    float    quality = 1.0f;
    RoiState roi = RoiState::NoTargetInRoi;
    uint32_t cost_us = 0;
};

// 扫描器到状态机的统一观察协议。
struct ScanReport {
    const Blip *cands = nullptr;
    size_t     count = 0;
    TargetMeasurement measurement{};
    RoiWindow  window{};
    uint32_t   scan_us = 0;

    bool full_scan() const { return window.full_frame; }
};

} // namespace dart::detection
