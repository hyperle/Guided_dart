#include "src/detector/geometry.hpp"

#include <algorithm>
#include <cstdint>

namespace dart::detection {

Rect FullWindow(std::uint32_t width, std::uint32_t height) noexcept {
    return {0, 0, width, height};
}

bool IsFullWindow(Rect window, std::uint32_t width, std::uint32_t height) noexcept {
    return window.x == 0 && window.y == 0 && window.width == width && window.height == height;
}

Rect UniteWindows(Rect a, Rect b) noexcept {
    if (b.width == 0 || b.height == 0)
        return a;
    const auto x = std::min(a.x, b.x);
    const auto y = std::min(a.y, b.y);
    return {
        x, y, std::max(a.x + a.width, b.x + b.width) - x,
        std::max(a.y + a.height, b.y + b.height) - y};
}

} // namespace dart::detection
