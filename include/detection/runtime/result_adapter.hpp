#pragma once

#include <cmath>

#include "core/frame.hpp"
#include "detection/domain/tracking.hpp"

namespace dart::detection {

// 把检测域结果翻译到 core 的稳定 DTO。编排层不再重复处理坐标和无目标哨兵值。
inline void apply_track_result(const TrackOutput &out, DetectResult *result) {
    if (result == nullptr)
        return;
    result->roi = out.roi;
    result->radius = out.found ? out.radius : -1.0f;
    result->circ = out.found ? out.circularity : -1.0f;
    result->state = static_cast<uint8_t>(out.state);
    result->roi_x0 = out.window.x;
    result->roi_y0 = out.window.y;
    result->roi_x1 = 0;
    result->roi_y1 = 0;
    if (!out.window.empty()) {
        result->roi_x1 = out.window.right() - 1u;
        result->roi_y1 = out.window.bottom() - 1u;
    }
    result->cx = -1;
    result->cy = -1;
    if (out.found) {
        result->cx = static_cast<int32_t>(std::lround(out.cx));
        result->cy = static_cast<int32_t>(std::lround(out.cy));
    }
}

} // namespace dart::detection
