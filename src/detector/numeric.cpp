module;
#include <cmath>
#include <cstdint>
#include <limits>

export module dart.detection.numeric;



export namespace dart::detection {

inline constexpr double kPi = 3.14159265358979323846;

inline constexpr float kDegrees = 180.0F / static_cast<float>(kPi);

bool IsPositive(float value) noexcept { return std::isfinite(value) && value > 0.0F; }

bool IsFraction(float value) noexcept {
    return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

std::uint64_t SumSquares(std::uint64_t n) noexcept {
    return n * (n + 1) * (2 * n + 1) / 6;
}

void IncrementSaturated(std::uint32_t& value) noexcept {
    if (value < std::numeric_limits<std::uint32_t>::max())
        ++value;
}

} // namespace dart::detection
