#pragma once

#include <cstdint>

namespace dart::detection {
inline constexpr double kPi = 3.14159265358979323846;

inline constexpr float kDegrees = 180.0F / static_cast<float>(kPi);

bool IsPositive(float value) noexcept;

bool IsFraction(float value) noexcept;

std::uint64_t SumSquares(std::uint64_t n) noexcept;

void IncrementSaturated(std::uint32_t& value) noexcept;

} // namespace dart::detection
