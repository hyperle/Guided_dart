#pragma once

#include <cstdio>
#include <utility>

#include "logging/logger.hpp"
#include "runtime/argument_parser.hpp"
#include "runtime/imu/session.hpp"
#include "runtime/recording_trigger.hpp"
#include "streaming/interface.hpp"
#include "system/device_services.hpp"
#include "system/launcher_exit.hpp"
#include "system/run_control.hpp"
#include "system/write_control.hpp"

namespace dart::app {

// Application assembly owns the component graph for the duration of run().
// Board factories select adapters; components receive only their device/port APIs.
class Application final {
public:
    Application(
        const Arguments& args, platform::CameraFactory camera, platform::RunControl& control,
        logging::Logger& log, imu::Session* imu = nullptr, platform::WriteControl* writes = nullptr,
        platform::PwmFactory pwm = nullptr, streaming::TransportFactory wifi = nullptr,
        recorder::VideoEncoderFactory video_encoder = nullptr) noexcept
        : args_(args), camera_factory_(camera), control_(control), log_(log), imu_(imu),
          writes_(writes), pwm_factory_(pwm), wifi_factory_(wifi), video_encoder_factory_(video_encoder) {}
    int run() noexcept;

private:
    class Execution;
    const Arguments& args_;
    platform::CameraFactory camera_factory_;
    platform::RunControl& control_;
    logging::Logger& log_;
    imu::Session* imu_;
    platform::WriteControl* writes_;
    platform::PwmFactory pwm_factory_;
    streaming::TransportFactory wifi_factory_;
    recorder::VideoEncoderFactory video_encoder_factory_;
};

template <typename ImuRunner>
int run_application(
    int argc, char* argv[], platform::CameraFactory camera_factory, ImuRunner&& run_imu,
    platform::PwmFactory pwm_factory = nullptr,
    streaming::TransportFactory wifi_factory = nullptr,
    recorder::VideoEncoderFactory video_encoder_factory = nullptr) noexcept {
    auto args = parse_args(argc, argv);
    if (!args)
        return args.error();
    platform::RunControl control{args->seconds};
    platform::WriteControl writes;
    if (!writes.start())
        return 1;
    writes.request_recording(args->record_on && !args->record_auto);
    RecordingTrigger trigger{
        writes, static_cast<float>(args->record_trigger_accel), args->record_trigger_ms,
        args->record_duration_ms};
    logging::Logger log{&writes};
    log.set_console(args->log_console);
    if (!log.open(args->log_path)) {
        std::fprintf(stderr, "[log] level=ERROR op=start result=failed action=exit\n");
        return 1;
    }
    const platform::ImuOptions options{
        args->imu_on, args->imu_bus, args->imu_cs, args->imu_log, args->imu_ignore_id, &writes,
        args->record_auto ? &RecordingTrigger::observe : nullptr, &trigger};
    const int result = std::forward<ImuRunner>(run_imu)(options, log, [&](imu::Session* session) noexcept {
        return Application{*args, camera_factory, control, log, session, &writes,
                           pwm_factory, wifi_factory, video_encoder_factory}.run();
    });
    platform::notify_launcher_exit();
    return result;
}

} // namespace dart::app
