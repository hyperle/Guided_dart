#pragma once

#include <chrono>

#include "logging/logger.hpp"
#include "runtime/imu/session.hpp"
#include "src/startup/argument_parser.hpp"
#include "streaming/interface.hpp"
#include "system/device_services.hpp"
#include "system/run_control.hpp"

namespace dart::app {
int run_services(
    const Arguments& args, platform::CameraFactory factory, platform::RunControl& control,
    logging::Logger& log, imu::Session* imu_session = nullptr,
    platform::WriteControl* writes = nullptr, platform::PwmFactory pwm_factory = nullptr,
    streaming::TransportFactory wifi_factory = nullptr) noexcept;

} // namespace dart::app
