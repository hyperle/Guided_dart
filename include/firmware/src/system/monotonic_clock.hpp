#pragma once

#include <chrono>
#include <cstdint>
#include <time.h>

namespace dart::platform {
std::int64_t mono_ms() noexcept;

void sleep_ms(int ms) noexcept;

} // namespace dart::platform
