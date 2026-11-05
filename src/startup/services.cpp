#include "src/startup/services.hpp"

#include <chrono>
#include <expected>
#include <memory>
#include <utility>

#include "logging/logger.hpp"
#include "runtime/imu/session.hpp"
#include "src/diagnostics/camera_error.hpp"
#include "src/startup/argument_parser.hpp"
#include "src/vision/capture_pipeline.hpp"
#include "streaming/interface.hpp"
#include "system/device_services.hpp"
#include "system/monotonic_clock.hpp"
#include "system/run_control.hpp"

namespace dart::app {

int run_services(
    const Arguments& args, platform::CameraFactory factory, platform::RunControl& control,
    logging::Logger& log, imu::Session* imu_session, platform::WriteControl* writes,
    platform::PwmFactory pwm_factory, streaming::TransportFactory wifi_factory) noexcept {
    log.line(
        "level=INFO module=app op=start version=4 exposure=fixed exp_lines=%u "
        "vision=%d pwm=%d record=%d record_auto=%d rec_fps=%u rec_timeout_ms=%u rec_keep=%u "
        "seconds=%u imu=%d "
        "capture=mcm_raw10 output=gray8 raw_layout=%u threshold=%u\n",
        args.exposure_lines, args.vision_on, args.pwm_on, args.record_on, args.record_auto,
        args.record_fps, args.record_timeout_ms, args.record_keep, args.seconds,
        imu_session != nullptr, static_cast<unsigned>(args.raw10_layout), args.threshold);
    camera::Controls controls{};
    controls.initial_exposure_lines = args.exposure_lines;
    controls.raw10_layout = args.raw10_layout;
    auto created = factory(controls);
    bool camera_failed = !created;
    if (!created) {
        log_camera_error(log, "create", "disable_capture", created.error());
    } else {
        camera_failed = CapturePipeline::execute(
                            args, std::move(created.value()), control, log, imu_session, writes,
                            pwm_factory, wifi_factory)
                     != 0;
    }
    if (camera_failed) {
        log.line(
            "level=WARN module=app op=degraded camera=disabled vision=unavailable "
            "record=unavailable imu=%s action=continue\n",
            imu_session != nullptr ? "running" : "disabled");
    }
    auto next_status = std::chrono::steady_clock::now();
    while (!control.finished()) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_status) {
            next_status = now + std::chrono::milliseconds{args.status_ms};
            log.line("level=INFO module=app op=status mode=degraded camera=disabled\n");
            if (imu_session != nullptr) {
                const auto stats = imu_session->stats();
                log.line(
                    "level=INFO module=imu op=status samples=%llu read_errors=%llu "
                    "written=%llu dropped=%llu io_errors=%llu log_bytes=%llu "
                    "log_limit_reached=%d\n",
                    static_cast<unsigned long long>(stats.samples),
                    static_cast<unsigned long long>(stats.read_errors),
                    static_cast<unsigned long long>(stats.written),
                    static_cast<unsigned long long>(stats.dropped),
                    static_cast<unsigned long long>(stats.io_errors),
                    static_cast<unsigned long long>(stats.log_bytes), stats.log_limit_reached);
                const auto estimate = imu_session->current();
                const auto& sample = estimate.sample;
                log.line(
                    "level=INFO module=imu op=sample raw_accel=%d,%d,%d raw_gyro=%d,%d,%d "
                    "accel_m_s2=%.5g,%.5g,%.5g gyro_rad_s=%.5g,%.5g,%.5g temp_c=%.5g "
                    "flags=%u quality=%u accepted=%d reference=%d\n",
                    sample.raw_counts.accelerometer.x, sample.raw_counts.accelerometer.y,
                    sample.raw_counts.accelerometer.z, sample.raw_counts.gyroscope.x,
                    sample.raw_counts.gyroscope.y, sample.raw_counts.gyroscope.z,
                    sample.acceleration_m_s2.x, sample.acceleration_m_s2.y,
                    sample.acceleration_m_s2.z, sample.angular_velocity_rad_s.x,
                    sample.angular_velocity_rad_s.y, sample.angular_velocity_rad_s.z,
                    sample.temperature_c, sample.flags, estimate.quality_events,
                    estimate.sample_accepted, estimate.has_reference_frame);
            }
        }
        platform::sleep_ms(20);
    }
    log.line("level=INFO module=app op=services_stop reason=%s\n", control.reason());
    return 0;
}

} // namespace dart::app
