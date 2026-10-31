#ifndef DART_STARTUP_RUN_CPP
#define DART_STARTUP_RUN_CPP

#include <utility>

#include "argument_parser.cpp"
#include "logging/logger.cpp"
#include "services.cpp"
#include "src/recording/trigger.cpp"
#include "system/device_services.cpp"
#include "system/launcher_exit.cpp"
#include "system/run_control.cpp"
#include "system/write_control.cpp"

namespace dart::app {

template <typename ImuRunner>
inline int run_capture(
    const Arguments& args, platform::CameraFactory camera_factory, ImuRunner&& run_imu,
    platform::PwmFactory pwm_factory = nullptr) noexcept {
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
    log.set_console(args.log_console);
    const platform::ImuOptions options{
        args.imu_on, args.imu_bus, args.imu_cs, args.imu_log, args.imu_ignore_id, &writes,
        args.record_auto ? &RecordingTrigger::observe : nullptr, &trigger};
    return run_imu(options, log, [&](imu::Session* session) noexcept {
        return run_services(args, camera_factory, control, log, session, &writes, pwm_factory);
    });
}

template <typename ImuRunner>
inline int run_application(
    int argc, char* argv[], platform::CameraFactory camera_factory, ImuRunner&& run_imu,
    platform::PwmFactory pwm_factory = nullptr) noexcept {
    auto args = parse_args(argc, argv);
    if (!args)
        return args.error();
    const int result = run_capture(args.value(), camera_factory, std::forward<ImuRunner>(run_imu),
                                   pwm_factory);
    platform::notify_launcher_exit();
    return result;
}

} // namespace dart::app

#endif // DART_STARTUP_RUN_CPP
