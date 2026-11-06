#include "recorder/config.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <string_view>

namespace dart::recorder {

bool Config::variable_record_size() const noexcept {
    return image_format == ImageFormat::kPng || image_format == ImageFormat::kDbv
        || image_format == ImageFormat::kPngStream || image_format == ImageFormat::kH264;
}

} // namespace dart::recorder
