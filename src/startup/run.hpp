#pragma once

#include <utility>

#include "logging/logger.hpp"
#include "runtime/imu/session.hpp"
#include "src/recording/trigger.hpp"
#include "src/startup/argument_parser.hpp"
#include "src/startup/services.hpp"
#include "streaming/interface.hpp"
#include "system/device_services.hpp"
#include "system/launcher_exit.hpp"
#include "system/run_control.hpp"
#include "system/write_control.hpp"

namespace dart::app {
template <typename ImuRunner>
int run_capture(
    const Arguments& args, platform::CameraFactory camera_factory, ImuRunner&& run_imu,
    platform::PwmFactory pwm_factory = nullptr,
    streaming::TransportFactory wifi_factory = nullptr) noexcept {
    platform::RunControl control{args.seconds};
    platform::WriteControl writes;
    if (!writes.start())
        return 1;
    writes.request_recording(args.record_on && !args.record_auto);
    RecordingTrigger trigger{
        writes, static_cast<float>(args.record_trigger_accel), args.record_trigger_ms,
        args.record_duration_ms};
    logging::Logger log{&writes};
    (void)log.open(args.log_path);
    const platform::ImuOptions options{
        args.imu_on,
        args.imu_bus,
        args.imu_cs,
        args.imu_log,
        args.imu_ignore_id,
        &writes,
        args.record_auto ? &RecordingTrigger::observe : nullptr,
        &trigger};
    return run_imu(options, log, [&](imu::Session* session) noexcept {
        return run_services(
            args, camera_factory, control, log, session, &writes, pwm_factory, wifi_factory);
    });
}

template <typename ImuRunner>
int run_application(
    int argc, char* argv[], platform::CameraFactory camera_factory, ImuRunner&& run_imu,
    platform::PwmFactory pwm_factory = nullptr,
    streaming::TransportFactory wifi_factory = nullptr) noexcept {
    auto args = parse_args(argc, argv);
    if (!args)
        return args.error();
    const int result = run_capture(
        args.value(), camera_factory, std::forward<ImuRunner>(run_imu), pwm_factory, wifi_factory);
    platform::notify_launcher_exit();
    return result;
}

} // namespace dart::app
