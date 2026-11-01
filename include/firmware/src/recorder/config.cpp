#include "recorder/config.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <string_view>

namespace dart::recorder {

bool Config::variable_record_size() const noexcept {
    return image_format == ImageFormat::kPng || image_format == ImageFormat::kDbv;
}

} // namespace dart::recorder
