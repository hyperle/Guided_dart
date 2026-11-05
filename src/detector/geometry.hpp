#pragma once

#include <cstdint>

namespace dart::detection {
struct Point {
    float x = 0.0F;
    float y = 0.0F;
};

struct Rect {
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

Rect FullWindow(std::uint32_t width, std::uint32_t height) noexcept;

bool IsFullWindow(Rect window, std::uint32_t width, std::uint32_t height) noexcept;

Rect UniteWindows(Rect a, Rect b) noexcept;

} // namespace dart::detection
