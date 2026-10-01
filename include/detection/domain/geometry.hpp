#pragma once

#include <cstdint>

namespace dart::detection {

// 半开区间 ROI：覆盖 x..x+w-1、y..y+h-1。
struct RoiWindow {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t w = 0;
    uint32_t h = 0;
    bool     full_frame = true;
    bool     clamped = false;

    bool empty() const { return w == 0 || h == 0; }
    uint32_t right() const { return x + w; }
    uint32_t bottom() const { return y + h; }
    uint64_t pixels() const { return static_cast<uint64_t>(w) * h; }

    bool contains(float px, float py) const {
        return px >= static_cast<float>(x) && py >= static_cast<float>(y) &&
               px < static_cast<float>(right()) && py < static_cast<float>(bottom());
    }
};

} // namespace dart::detection
