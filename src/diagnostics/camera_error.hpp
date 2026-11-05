#pragma once

#include <string_view>

#include "camera/error.hpp"
#include "logging/logger.hpp"

namespace dart::app {
std::string_view camera_operation_text(camera::Operation operation) noexcept;

/* Recovery belongs to the caller. This function only formats one observation. */
void log_camera_error(
    logging::Logger& log, const char* op, const char* action,
    const camera::CameraError& error) noexcept;

} // namespace dart::app
