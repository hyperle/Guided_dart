#include "src/vision/capture_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string_view>
#include <utility>

#include "camera/interface.hpp"
#include "logging/logger.hpp"
#include "recorder/recorder.hpp"
#include "runtime/imu/session.hpp"
#include "src/detector/roi.hpp"
#include "src/diagnostics/camera_error.hpp"
#include "src/diagnostics/detection_csv.hpp"
#include "src/recording/frame_recording.hpp"
#include "src/startup/argument_parser.hpp"
#include "src/vision/visual_servo.hpp"
#include "streaming/interface.hpp"
#include "streaming/session.hpp"
#include "system/device_services.hpp"
#include "system/monotonic_clock.hpp"
#include "system/run_control.hpp"
#include "system/write_control.hpp"

namespace dart::app {

int CapturePipeline::execute(
    const Arguments& args, std::unique_ptr<camera::ICamera> cam, platform::RunControl& control,
    logging::Logger& log, imu::Session* imu_session, platform::WriteControl* writes,
    platform::PwmFactory pwm_factory, streaming::TransportFactory wifi_factory) noexcept {
    bool vision_on = args.vision_on;
    bool record_on = args.record_on;
    log.line(
        "level=INFO module=app op=camera_create result=ok capacity=%u frame=%ux%u\n",
        cam->capacity(), cam->frame_info().width, cam->frame_info().height);

    detection::RoiDetector::Config detect_config{};
    detect_config.threshold = static_cast<std::uint8_t>(args.threshold);
    detect_config.workspace.min_contrast = args.min_contrast;
    detection::RoiDetector detector{detect_config};
    const auto max_width = cam->frame_info().width;
    const auto max_height = cam->frame_info().height;
    if (vision_on) {
        auto prepared = detector.Prepare(max_width, max_height);
        if (!prepared) {
            log.line(
                "level=ERROR module=app op=detector_prepare code=%d "
                "max=%ux%u action=disable_vision\n",
                static_cast<int>(prepared.error()), max_width, max_height);
            vision_on = false;
        }
        if (vision_on)
            log.line(
                "level=INFO module=app op=detector_prepare result=ok max=%ux%u "
                "threshold=%u min_contrast=%g input=gray8\n",
                max_width, max_height, args.threshold, static_cast<double>(args.min_contrast));
    }

    const auto recorder_config = recording_config(args, max_width, max_height);
    recorder::Recorder rec{recorder_config};

    DetectionCsv csv;
    if (args.det_csv[0] != '\0' && !csv.open(args.det_csv, writes)) {
        log.line(
            "level=WARN module=app op=csv_open result=failed path=%s "
            "action=continue\n",
            args.det_csv);
    } else if (args.det_csv[0] != '\0') {
        log.line("level=INFO module=app op=csv_open result=ok path=%s\n", args.det_csv);
    }

    auto started = cam->start();
    if (!started) {
        log_camera_error(log, "start", "disable_capture", started.error());
        csv.close();
        return 1;
    }
    log.line(
        "level=INFO module=app op=camera_start result=ok state=%u\n",
        static_cast<unsigned>(cam->state()));

    streaming::Session wifi;
    const bool wifi_started =
        args.wifi_on
        && wifi.start_controlled(
            args.wifi_config, max_width, max_height, wifi_factory, args.wifi_pause_file);
    if (args.wifi_on) {
        const auto error = wifi.error();
        log.line(
            "level=%s module=wifi op=start result=%s code=%u native=%d "
            "transport=http mode=controlled state=%s\n",
            wifi_started ? "INFO" : "ERROR", wifi_started ? "ok" : "disabled",
            static_cast<unsigned>(error.code), error.native, streaming::state_text(wifi.state()));
    }

    VisualServo servo{args, log};
    if (vision_on)
        servo.start(pwm_factory, max_width, max_height);

    if (record_on) {
        record_on = writes != nullptr ? rec.start_controlled(*writes)
                                      : start_recording(rec, recorder_config, log);
    }

    Counters counters{};
    const auto start_time = std::chrono::steady_clock::now();
    auto next_status = start_time + std::chrono::milliseconds{args.status_ms};
    std::uint32_t consecutive_acquire_errors = 0;
    bool capture_failed = false;
    auto timing_begin = start_time;
    std::uint64_t timing_frames = 0;
    StageTiming acquire_timing{}, detect_timing{}, pipeline_timing{}, telemetry_timing{};

    while (!control.finished()) {
        if (std::chrono::steady_clock::now() >= next_status) {
            const auto telemetry_begin = std::chrono::steady_clock::now();
            next_status =
                std::chrono::steady_clock::now() + std::chrono::milliseconds{args.status_ms};
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        std::chrono::steady_clock::now() - start_time)
                                        .count();
            const auto stats = rec.stats();
            log.begin_batch();
            servo.status();
            if (wifi_started) {
                const auto net = wifi.stats();
                const auto lifecycle_error = wifi.error();
                log.line(
                    "level=INFO module=wifi op=status connected=%d accepted=%llu "
                    "encoded=%llu uploaded=%llu bytes=%llu raw_replaced=%llu "
                    "jpeg_replaced=%llu busy=%llu invalid=%llu jpeg_errors=%llu "
                    "stale_dropped=%llu downsampled=%llu "
                    "network_errors=%llu native=%d state=%s active=%d buffers=%llu "
                    "lifecycle_code=%u lifecycle_native=%d\n",
                    net.connected, static_cast<unsigned long long>(net.accepted),
                    static_cast<unsigned long long>(net.encoded),
                    static_cast<unsigned long long>(net.uploaded),
                    static_cast<unsigned long long>(net.bytes),
                    static_cast<unsigned long long>(net.raw_replaced),
                    static_cast<unsigned long long>(net.jpeg_replaced),
                    static_cast<unsigned long long>(net.busy),
                    static_cast<unsigned long long>(net.invalid),
                    static_cast<unsigned long long>(net.jpeg_errors),
                    static_cast<unsigned long long>(net.stale_dropped),
                    static_cast<unsigned long long>(net.downsampled),
                    static_cast<unsigned long long>(net.network_errors), net.last_native,
                    streaming::state_text(wifi.state()), wifi.accepting(),
                    static_cast<unsigned long long>(net.buffer_bytes),
                    static_cast<unsigned>(lifecycle_error.code), lifecycle_error.native);
            }
            if (writes != nullptr)
                log.line(
                    "level=INFO module=app op=write_control recording=%d logging=%d "
                    "record_active=%d rec_error=%s record_trigger_ns=%llu\n",
                    writes->recording_enabled(), writes->logging_enabled(), rec.started(),
                    recorder::Recorder::error_text(rec.last_error()).data(),
                    static_cast<unsigned long long>(writes->recording_request_ns()));
            log.line(
                "level=INFO module=app op=status elapsed_s=%lld frames=%llu "
                "fps=%.1f no_frame=%llu lease_exhausted=%llu target=%llu "
                "detect_err=%llu rec_input=%llu rec_acc=%llu rec_down=%llu "
                "rec_busy=%llu rec_timed_out=%llu rec_queued=%u rec_pending=%u rec_written=%llu "
                "rec_event_dropped=%llu release_err=%llu run_hw=%u\n",
                static_cast<long long>(elapsed_ms / 1000),
                static_cast<unsigned long long>(counters.frames),
                elapsed_ms > 0 ? static_cast<double>(counters.frames) * 1000.0
                                     / static_cast<double>(elapsed_ms)
                               : 0.0,
                static_cast<unsigned long long>(counters.no_frame),
                static_cast<unsigned long long>(counters.lease_exhausted),
                static_cast<unsigned long long>(counters.target_hits),
                static_cast<unsigned long long>(counters.detect_errors),
                static_cast<unsigned long long>(stats.input),
                static_cast<unsigned long long>(stats.accepted),
                static_cast<unsigned long long>(stats.downsampled),
                static_cast<unsigned long long>(stats.busy),
                static_cast<unsigned long long>(stats.timed_out), stats.queued,
                stats.release_pending, static_cast<unsigned long long>(stats.written),
                static_cast<unsigned long long>(stats.event_dropped),
                static_cast<unsigned long long>(counters.release_errors),
                detector.run_high_water());
            log.line(
                "level=INFO module=app op=detect_track detect_frames=%llu track_frames=%llu "
                "validation_frames=%llu stability_checks=%llu\n",
                static_cast<unsigned long long>(counters.detect_frames),
                static_cast<unsigned long long>(counters.track_frames),
                static_cast<unsigned long long>(counters.validation_frames),
                static_cast<unsigned long long>(counters.stability_checks));
            if (stats.written > 0)
                log.line(
                    "level=INFO module=app op=recorder_timing bytes=%llu "
                    "copy_us=%.1f pack_us=%.1f io_us=%.1f io_max_us=%llu "
                    "space_us=%.1f open_us=%.1f write_us=%.1f close_us=%.1f\n",
                    static_cast<unsigned long long>(stats.bytes),
                    stats.copied > 0 ? static_cast<double>(stats.copy_total_us) / stats.copied
                                     : 0.0,
                    static_cast<double>(stats.pack_total_us) / stats.written,
                    static_cast<double>(stats.io_total_us) / stats.written,
                    static_cast<unsigned long long>(stats.io_max_us),
                    static_cast<double>(stats.space_total_us) / stats.written,
                    static_cast<double>(stats.open_total_us) / stats.written,
                    static_cast<double>(stats.write_total_us) / stats.written,
                    static_cast<double>(stats.close_total_us) / stats.written);
            const auto window_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                       telemetry_begin - timing_begin)
                                       .count();
            log.line(
                "level=INFO module=app op=timing window_ms=%lld fps=%.1f "
                "acquire_us=%.1f acquire_max_us=%llu detect_us=%.1f detect_max_us=%llu "
                "pipeline_us=%.1f pipeline_max_us=%llu telemetry_us=%.1f "
                "telemetry_max_us=%llu note=acquire_includes_wait\n",
                static_cast<long long>(window_us / 1000),
                window_us > 0 ? static_cast<double>(counters.frames - timing_frames) * 1000000.0
                                    / static_cast<double>(window_us)
                              : 0.0,
                acquire_timing.average_us(), static_cast<unsigned long long>(acquire_timing.max_us),
                detect_timing.average_us(), static_cast<unsigned long long>(detect_timing.max_us),
                pipeline_timing.average_us(),
                static_cast<unsigned long long>(pipeline_timing.max_us),
                telemetry_timing.average_us(),
                static_cast<unsigned long long>(telemetry_timing.max_us));
            if (imu_session != nullptr) {
                const auto estimate = imu_session->current();
                const auto imu_stats = imu_session->stats();
                log.line(
                    "level=INFO module=imu op=status timestamp_ns=%llu generation=%llu "
                    "quality_events=%u accepted=%d integrated=%d reference=%d "
                    "samples=%llu read_errors=%llu dropped=%llu io_errors=%llu "
                    "log_bytes=%llu log_limit_reached=%d\n",
                    static_cast<unsigned long long>(estimate.sample.timestamp_ns),
                    static_cast<unsigned long long>(estimate.generation), estimate.quality_events,
                    estimate.sample_accepted, estimate.integrated, estimate.has_reference_frame,
                    static_cast<unsigned long long>(imu_stats.samples),
                    static_cast<unsigned long long>(imu_stats.read_errors),
                    static_cast<unsigned long long>(imu_stats.dropped),
                    static_cast<unsigned long long>(imu_stats.io_errors),
                    static_cast<unsigned long long>(imu_stats.log_bytes),
                    imu_stats.log_limit_reached);
            }
            log.end_batch();
            csv.flush();
            timing_begin = telemetry_begin;
            timing_frames = counters.frames;
            acquire_timing = {};
            detect_timing = {};
            pipeline_timing = {};
            telemetry_timing = {};
            // Report this status write/flush cost in the following window.
            telemetry_timing.observe(telemetry_begin);
        }
        const auto acquire_begin = std::chrono::steady_clock::now();
        auto acquired = cam->acquire(std::chrono::milliseconds{args.acquire_ms});
        acquire_timing.observe(acquire_begin);
        if (!acquired) {
            servo.observe(nullptr);
            counters.acquire_errors++;
            consecutive_acquire_errors++;
            log_camera_error(log, "camera_acquire", "retry", acquired.error());
            if (consecutive_acquire_errors >= args.max_acquire_errors) {
                log.line(
                    "level=ERROR module=app op=camera_acquire "
                    "result=disabled consecutive=%u limit=%u\n",
                    consecutive_acquire_errors, args.max_acquire_errors);
                capture_failed = true;
                break;
            }
            platform::sleep_ms(10);
            continue;
        }
        consecutive_acquire_errors = 0;

        if (acquired->status == camera::AcquireStatus::kNoFrame) {
            servo.observe(nullptr);
            /* 正常否定结果：不是错误，只计数。 */
            counters.no_frame++;
            (void)rec.drain_releases();
            continue;
        }
        if (acquired->status == camera::AcquireStatus::kLeaseExhausted) {
            servo.observe(nullptr);
            /* 容量性丢弃：驱动从未被触碰，这一帧在驱动池里自生自灭。 */
            counters.lease_exhausted++;
            (void)rec.drain_releases();
            continue;
        }

        counters.frames++;
        auto& lease = acquired->lease;
        // Keep metadata valid after release or transfer to the recording worker.
        const auto view = lease.view();

        const detection::DetectResult* result_ptr = nullptr;
        detection::DetectResult result{};
        if (vision_on) {
            counters.detect_calls++;
            const auto detect_begin = std::chrono::steady_clock::now();
            auto detected = detector.Detect(view);
            detect_timing.observe(detect_begin);
            if (!detected) {
                counters.detect_errors++;
                log.line(
                    "level=WARN module=app op=detect code=%d sequence=%llu "
                    "action=skip_frame\n",
                    static_cast<int>(detected.error()),
                    static_cast<unsigned long long>(view.sequence));
            } else {
                result = detected.value();
                if (result.scan.phase == detection::DetectionPhase::kTrack)
                    ++counters.track_frames;
                else
                    ++counters.detect_frames;
                if (result.scan.full_validation)
                    ++counters.validation_frames;
                counters.stability_checks += result.scan.stability_checks;
                result_ptr = &result;
                if (result.target.valid)
                    counters.target_hits++;
            }
        }

        // Publish the current full-image observation before any recording IO.
        servo.observe(result_ptr);

        /* 所有权：只有 kAccepted 会把租约转给 Recorder。 */
        const auto pipeline_begin = std::chrono::steady_clock::now();
        if (wifi_started)
            (void)wifi.try_submit(view);
        // The worker may finish while acquire/detection runs. Reclaim those
        // leases before admission so completed writes do not look busy.
        (void)rec.drain_releases();
        const char* admission = "off";
        if (record_on) {
            const auto decision = rec.try_submit(std::move(lease));
            admission = admission_text(decision);
            switch (decision) {
            case recorder::Admission::kAccepted: counters.rec_accepted++; break;
            case recorder::Admission::kDownsampled: counters.rec_downsampled++; break;
            case recorder::Admission::kBusy: counters.rec_busy++; break;
            case recorder::Admission::kInvalidFrame: counters.rec_invalid++; break;
            case recorder::Admission::kStopped: break;
            }
            if (decision != recorder::Admission::kAccepted) {
                auto released = lease.release();
                if (released)
                    counters.release_ok++;
                else {
                    counters.release_errors++;
                    log_camera_error(log, "camera_release", "count_and_continue", released.error());
                }
            }
        } else {
            auto released = lease.release();
            if (released)
                counters.release_ok++;
            else {
                counters.release_errors++;
                log_camera_error(log, "camera_release", "count_and_continue", released.error());
            }
        }

        csv.row(view, result_ptr, admission);
        (void)rec.drain_releases();
        if (record_on && writes == nullptr && stop_recording_if_finished(rec, recorder_config, log))
            record_on = false;
        pipeline_timing.observe(pipeline_begin);
    }

    /* ---------------- 收尾：停止输入 → 停消费者 → 排空 → 摘要 → 释放 SDK ---- */
    log.line(
        "level=INFO module=app op=shutdown step=input_stop reason=%s "
        "frames=%llu\n",
        capture_failed ? "camera_failure" : control.reason(),
        static_cast<unsigned long long>(counters.frames));

    servo.stop();
    wifi.stop();
    if (wifi_started) {
        const auto net = wifi.stats();
        log.line(
            "level=INFO module=wifi op=stop uploaded=%llu bytes=%llu network_errors=%llu\n",
            static_cast<unsigned long long>(net.uploaded),
            static_cast<unsigned long long>(net.bytes),
            static_cast<unsigned long long>(net.network_errors));
    }

    if (record_on) {
        rec.stop();
        log.line(
            "level=INFO module=app op=shutdown step=recorder_stopped "
            "pending=%u\n",
            rec.pending_releases());
    }
    drain_recording_releases(rec, log);

    if (cam->state() == camera::State::kStarted) {
        auto stopped = cam->stop();
        if (!stopped) {
            log_camera_error(log, "camera_stop", "destroy_anyway", stopped.error());
        } else {
            log.line(
                "level=INFO module=app op=shutdown step=camera_stopped "
                "result=ok\n");
        }
    }

    const auto stats = rec.stats();
    log.line(
        "level=INFO module=app op=shutdown step=summary frames=%llu "
        "no_frame=%llu lease_exhausted=%llu acquire_errors=%llu "
        "target_hits=%llu detect_calls=%llu detect_errors=%llu "
        "rec_accepted=%llu rec_downsampled=%llu rec_busy=%llu rec_invalid=%llu "
        "rec_written=%llu rec_timed_out=%llu rec_bytes=%llu rec_queue_hw=%u rec_event_dropped=%llu "
        "rec_copy_fail=%llu rec_write_fail=%llu rec_lease_release_fail=%llu "
        "release_ok=%llu release_errors=%llu run_hw=%u\n",
        static_cast<unsigned long long>(counters.frames),
        static_cast<unsigned long long>(counters.no_frame),
        static_cast<unsigned long long>(counters.lease_exhausted),
        static_cast<unsigned long long>(counters.acquire_errors),
        static_cast<unsigned long long>(counters.target_hits),
        static_cast<unsigned long long>(counters.detect_calls),
        static_cast<unsigned long long>(counters.detect_errors),
        static_cast<unsigned long long>(counters.rec_accepted),
        static_cast<unsigned long long>(counters.rec_downsampled),
        static_cast<unsigned long long>(counters.rec_busy),
        static_cast<unsigned long long>(counters.rec_invalid),
        static_cast<unsigned long long>(stats.written),
        static_cast<unsigned long long>(stats.timed_out),
        static_cast<unsigned long long>(stats.bytes), stats.queue_high_water,
        static_cast<unsigned long long>(stats.event_dropped),
        static_cast<unsigned long long>(stats.copy_fail),
        static_cast<unsigned long long>(stats.write_fail),
        static_cast<unsigned long long>(stats.lease_release_fail),
        static_cast<unsigned long long>(counters.release_ok),
        static_cast<unsigned long long>(counters.release_errors), detector.run_high_water());

    csv.close();

    cam.reset(); /* deinit -> destroy -> media destroy */
    log.line("level=INFO module=app op=shutdown step=camera_destroyed result=ok\n");
    log.line("level=INFO module=app op=exit result=done\n");
    return capture_failed ? 1 : 0;
}

} // namespace dart::app
