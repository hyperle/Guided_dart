#include "runtime/application.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "logging/csv_writer.hpp"
#include "runtime/tunner.hpp"
#include "src/control/blackboard.hpp"
#include "src/control/servo.hpp"
#include "src/detector/detect.hpp"
#include "src/recorder/frame_channel.hpp"
#include "streaming/session.hpp"
#include "system/monotonic_clock.hpp"

namespace dart::app {

class Application::Execution final {
public:
    explicit Execution(Application& app) noexcept
        : app_(app), args_(app.args_), log_(app.log_), blackboard_(app.imu_) {}
    ~Execution() { stop_servo(); stop_capture(); }

    int run() noexcept {
        log_.line(
            "level=INFO module=app op=start vision=%d pwm=%d record=%d record_auto=%d imu=%d\n",
            args_.vision_on, args_.pwm_on, args_.record_on, args_.record_auto, app_.imu_ != nullptr);
        if (args_.det_csv[0] != '\0' && !csv_.open(
                args_.det_csv,
                "sequence,received_us,source_pts,kind,valid,cx,cy,area,radius,quality,clipped,"
                "missed_frames,candidates,full_frame,rescanned,win_x0,win_y0,win_x1,win_y1,admission\n",
                app_.writes_))
            log_.line("level=WARN module=app op=csv_open result=failed path=%s\n", args_.det_csv);
        if (start_capture()) {
            start_recorder();
            start_servo();
            wifi_started_ = args_.wifi_on && wifi_.start_controlled(
                args_.wifi_config, camera_->frame_info().width, camera_->frame_info().height,
                app_.wifi_factory_, args_.wifi_pause_file);
            if (args_.wifi_on)
                log_.line("level=%s module=wifi op=start result=%s\n",
                          wifi_started_ ? "INFO" : "ERROR", wifi_started_ ? "ok" : "disabled");
        } else {
            stop_capture();
            degraded();
        }

        auto next_status = std::chrono::steady_clock::now();
        while (!app_.control_.finished()) {
            if (capture_started_) {
                (void)frames_->drain_releases();
                const auto stepped = detector_->step();
                (void)frames_->drain_releases();
                if (!stepped) {
                    if (detector_status_.camera_error)
                        camera_error("acquire", *detector_status_.camera_error);
                    else
                        component_error("detector", stepped.error());
                    stop_capture();
                    degraded();
                } else if (!detector_status_.acquired_frame) {
                    platform::sleep_ms(1);
                }
            } else {
                platform::sleep_ms(10);
            }
            observe_workers();
            if (std::chrono::steady_clock::now() >= next_status) {
                report();
                next_status = std::chrono::steady_clock::now()
                            + std::chrono::milliseconds{args_.status_ms};
            }
        }
        stop_servo();
        stop_capture();
        report();
        csv_.close();
        log_.line("level=INFO module=app op=services_stop reason=%s\n", app_.control_.reason());
        return 0;
    }

private:
    bool start_capture() noexcept {
        if (app_.camera_factory_ == nullptr) {
            log_.line("level=ERROR module=camera op=create result=disabled action=disable_capture\n");
            return false;
        }
        camera::Controls controls{};
        controls.initial_exposure_lines = args_.exposure_lines;
        controls.raw10_layout = args_.raw10_layout;
        auto created = app_.camera_factory_(controls);
        if (!created) {
            camera_error("create", created.error());
            return false;
        }
        camera_ = std::move(*created);
        if (!camera_) {
            log_.line("level=ERROR module=camera op=create result=disabled action=disable_capture\n");
            return false;
        }
        frames_.emplace();
        const executor::Component::Spec spec{
            "detector", "detector", 240.0,
            {{"enabled", args_.vision_on ? 1.0 : 0.0},
             {"threshold", static_cast<double>(args_.threshold)},
             {"workspace.min_contrast", args_.min_contrast},
             {"acquire_ms", static_cast<double>(args_.acquire_ms)},
             {"max_acquire_errors", static_cast<double>(args_.max_acquire_errors)}}};
        auto component = detection::DetectorFactory::create(
            spec, *camera_, executor::Output<detection::Point>{blackboard_.target}, detector_status_,
            [this](camera::FrameLease& lease, const detection::DetectResult* result) {
                consume_frame(lease, result);
            });
        if (!component) {
            component_error("detector", component.error());
            return false;
        }
        detector_ = std::move(*component);
        if (const auto initialized = detector_->initialze(); !initialized) {
            if (detector_status_.camera_error)
                camera_error("start", *detector_status_.camera_error);
            else
                component_error("detector", initialized.error());
            return false;
        }
        capture_started_ = true;
        log_.line("level=INFO module=app op=detector_prepare result=ok max=%ux%u enabled=%d\n",
                  camera_->frame_info().width, camera_->frame_info().height, args_.vision_on);
        return true;
    }

    void start_recorder() noexcept {
        if (!args_.record_on)
            return;
        auto config = recording_config(
            args_, camera_->frame_info().width, camera_->frame_info().height);
        config.video_encoder_factory = app_.video_encoder_factory_;
        const executor::Component::Spec spec{"recorder", "recorder", 240.0, {}};
        auto component = recorder::RecorderFactory::create(spec, config, *frames_, recording_, app_.writes_);
        if (!component) {
            component_error("recorder", component.error());
            return;
        }
        if (const auto started = recorder_tunner_.start(std::move(*component)); !started) {
            component_error("recorder", started.error());
            return;
        }
        recorder_started_ = true;
    }

    void start_servo() noexcept {
        if (!args_.pwm_on || !args_.vision_on || app_.pwm_factory_ == nullptr)
            return;
        auto created = app_.pwm_factory_();
        if (!created || !*created) {
            log_.line("level=ERROR module=servo op=create result=disabled action=disable_pwm\n");
            return;
        }
        pwm_ = std::move(*created);
        auto config = args_.servo;
        config.target_timeout = std::chrono::milliseconds{args_.servo_timeout_ms};
        config.aim = args_.aim_explicit ? args_.aim : detection::Point{
            static_cast<float>(camera_->frame_info().width) / 2.0F,
            static_cast<float>(camera_->frame_info().height) / 2.0F};
        const executor::Component::Spec spec{"servo", "servo", 333.0, {}};
        auto component = control::ServoFactory::create(spec, config, blackboard_, *pwm_, servo_status_);
        if (!component) {
            component_error("servo", component.error());
            return;
        }
        if (const auto started = servo_tunner_.start(std::move(*component)); !started) {
            component_error("servo", started.error());
            return;
        }
        servo_started_ = true;
    }

    void consume_frame(camera::FrameLease& lease, const detection::DetectResult* result) noexcept {
        // Only scalar metadata is used after the recorder accepts the lease.
        const auto view = lease.view();
        ++frames_seen_;
        if (result != nullptr && result->target.valid)
            ++target_hits_;
        if (wifi_started_)
            (void)wifi_.try_submit(view);
        (void)frames_->drain_releases();
        auto admission = recorder::Admission::kStopped;
        if (recorder_started_)
            admission = frames_->try_submit(std::move(lease));
        const char* decision = admission_text(admission);
        if (result == nullptr) {
            csv_.row("%llu,%lld,%llu,none,0,,,,,,,,,,,,,,%s\n",
                     static_cast<unsigned long long>(view.sequence),
                     static_cast<long long>(view.received.count()),
                     static_cast<unsigned long long>(view.source_pts), decision);
        } else {
            const auto& target = result->target;
            const auto& scan = result->scan;
            csv_.row("%llu,%lld,%llu,%s,%d,%.1f,%.1f,%u,%.2f,%.3f,%d,%u,%u,%d,%d,%u,%u,%u,%u,%s\n",
                static_cast<unsigned long long>(result->source.frame_id),
                static_cast<long long>(result->source.received.count()),
                static_cast<unsigned long long>(result->source.source_pts), kind_text(result->kind),
                target.valid, static_cast<double>(target.center.x), static_cast<double>(target.center.y),
                target.area, static_cast<double>(target.radius), static_cast<double>(target.quality),
                target.clipped, result->missed_frames, scan.candidates, scan.full_frame, scan.rescanned,
                scan.search_window.x, scan.search_window.y, scan.search_window.x + scan.search_window.width,
                scan.search_window.y + scan.search_window.height, decision);
        }
    }

    void observe_workers() noexcept {
        if (servo_started_ && servo_tunner_.finished()) {
            if (const auto error = servo_tunner_.error())
                component_error("servo", *error);
            stop_servo();
        }
        if (recorder_started_ && recorder_tunner_.finished()) {
            if (const auto error = recorder_tunner_.error())
                component_error("recorder", *error);
            recorder_tunner_.join();
            recorder_started_ = false;
        }
        const auto error = recording_.last_error();
        if (error != recorder::Error::kNone && error != recording_error_) {
            recording_error_ = error;
            log_.line("level=ERROR module=recorder op=write error=%s action=disable_recording\n",
                      recorder::RecorderStatus::error_text(error).data());
        }
    }

    void stop_capture() noexcept {
        // No new detector ticks are issued while a recorder still owns a lease.
        if (recorder_started_) {
            recorder_tunner_.stop();
            recorder_started_ = false;
            if (const auto error = recorder_tunner_.error())
                component_error("recorder", *error);
        }
        if (wifi_started_) {
            wifi_.stop();
            wifi_started_ = false;
        }
        if (frames_) {
            for (unsigned retry = 0; retry < 500 && frames_->pending_releases() != 0; ++retry) {
                (void)frames_->drain_releases();
                if (frames_->pending_releases() != 0)
                    platform::sleep_ms(10);
            }
            if (frames_->pending_releases() != 0)
                log_.line("level=ERROR module=recorder op=release result=failed\n");
            // Destroy any retained failed lease on the camera thread, before the owner.
            frames_.reset();
        }
        if (detector_) {
            if (const auto stopped = detector_->shutdown(); !stopped)
                component_error("detector", stopped.error());
            detector_.reset();
        }
        capture_started_ = false;
        camera_.reset();
    }

    void stop_servo() noexcept {
        if (!servo_started_ && !pwm_)
            return;
        if (servo_started_) {
            servo_tunner_.stop();
            servo_started_ = false;
        }
        if (servo_status_.native_error.load() != 0)
            log_.line("level=ERROR module=servo op=stop operation=%s channel=%d native=%d\n",
                      servo_status_.operation.load(), servo_status_.channel.load(),
                      servo_status_.native_error.load());
        pwm_.reset();
    }

    void report() noexcept {
        const auto rec = recording_.stats();
        log_.line("level=INFO module=servo op=status running=%d writes=%llu timeouts=%llu "
                  "pulse_us=%.3f,%.3f,%.3f,%.3f\n",
                  servo_status_.active.load(), static_cast<unsigned long long>(servo_status_.writes.load()),
                  static_cast<unsigned long long>(servo_status_.stale_targets.load()),
                  static_cast<double>(servo_status_.pulse_us[0].load()),
                  static_cast<double>(servo_status_.pulse_us[1].load()),
                  static_cast<double>(servo_status_.pulse_us[2].load()),
                  static_cast<double>(servo_status_.pulse_us[3].load()));
        log_.line("level=INFO module=recorder op=status rec_acc=%llu rec_down=%llu rec_queued=%u "
                  "rec_event_dropped=%llu rec_bytes=%llu rec_lease_release_fail=%llu\n",
                  static_cast<unsigned long long>(rec.accepted), static_cast<unsigned long long>(rec.downsampled),
                  rec.queued, static_cast<unsigned long long>(rec.event_dropped),
                  static_cast<unsigned long long>(rec.bytes), static_cast<unsigned long long>(rec.lease_release_fail));
        if (app_.writes_ != nullptr)
            log_.line("level=INFO module=app op=write_control recording=%d logging=%d "
                      "record_active=%d record_trigger_ns=%llu\n",
                      app_.writes_->recording_enabled(), app_.writes_->logging_enabled(), recording_.started(),
                      static_cast<unsigned long long>(app_.writes_->recording_request_ns()));
        log_.line("level=INFO module=app op=status frames=%llu target=%llu rec_accepted=%llu "
                  "rec_written=%llu rec_busy=%llu rec_timed_out=%llu rec_copy_fail=%llu "
                  "rec_write_fail=%llu release_errors=%llu capture=%d\n",
                  static_cast<unsigned long long>(frames_seen_), static_cast<unsigned long long>(target_hits_),
                  static_cast<unsigned long long>(rec.accepted), static_cast<unsigned long long>(rec.written),
                  static_cast<unsigned long long>(rec.busy), static_cast<unsigned long long>(rec.timed_out),
                  static_cast<unsigned long long>(rec.copy_fail), static_cast<unsigned long long>(rec.write_fail),
                  static_cast<unsigned long long>(rec.lease_release_fail), capture_started_);
        if (app_.imu_ != nullptr) {
            const auto imu = app_.imu_->stats();
            log_.line("level=INFO module=imu op=status samples=%llu read_errors=%llu written=%llu dropped=%llu\n",
                      static_cast<unsigned long long>(imu.samples), static_cast<unsigned long long>(imu.read_errors),
                      static_cast<unsigned long long>(imu.written), static_cast<unsigned long long>(imu.dropped));
        }
        if (wifi_started_) {
            const auto wifi = wifi_.stats();
            log_.line("level=INFO module=wifi op=status accepted=%llu uploaded=%llu busy=%llu\n",
                      static_cast<unsigned long long>(wifi.accepted), static_cast<unsigned long long>(wifi.uploaded),
                      static_cast<unsigned long long>(wifi.busy));
        }
    }

    void camera_error(const char* operation, const camera::CameraError& error) noexcept {
        char context[1024]{};
        camera::ErrorTextSink sink{context, sizeof(context)};
        error.append_context(sink);
        log_.line("level=ERROR module=camera op=%s native=%d code=%.*s ctx=%s action=disable_capture\n",
                  operation, error.code(), static_cast<int>(error.category().size()), error.category().data(), context);
    }
    void component_error(const char* component, executor::Error error) noexcept {
        log_.line("level=ERROR module=executor component=%s error=%s\n",
                  component, executor::discribe(error).data());
    }
    void degraded() noexcept {
        log_.line("level=WARN module=app op=degraded camera=disabled imu=%s action=continue\n",
                  app_.imu_ != nullptr ? "running" : "disabled");
    }
    static const char* admission_text(recorder::Admission admission) noexcept {
        switch (admission) {
        case recorder::Admission::kAccepted: return "accepted";
        case recorder::Admission::kDownsampled: return "downsampled";
        case recorder::Admission::kBusy: return "busy";
        case recorder::Admission::kInvalidFrame: return "invalid";
        case recorder::Admission::kStopped: return "off";
        }
        return "off";
    }
    static const char* kind_text(detection::TargetKind kind) noexcept {
        switch (kind) {
        case detection::TargetKind::kArmor: return "armor";
        case detection::TargetKind::kLight: return "light";
        case detection::TargetKind::kNone: return "none";
        }
        return "none";
    }

    Application& app_;
    const Arguments& args_;
    logging::Logger& log_;
    std::unique_ptr<camera::ICamera> camera_{};
    control::BlackBoard blackboard_;
    std::unique_ptr<pwm::IPwm> pwm_{};
    recorder::RecorderStatus recording_{};
    std::optional<recorder::FrameChannel> frames_{};
    detection::DetectorStatus detector_status_{};
    control::ServoStatus servo_status_{};
    executor::Tunner recorder_tunner_{}, servo_tunner_{};
    std::unique_ptr<executor::Component> detector_{};
    streaming::Session wifi_{};
    logging::CsvWriter csv_{};
    std::uint64_t frames_seen_ = 0, target_hits_ = 0;
    recorder::Error recording_error_ = recorder::Error::kNone;
    bool capture_started_ = false, recorder_started_ = false;
    bool servo_started_ = false, wifi_started_ = false;
};

int Application::run() noexcept {
    Execution execution{*this};
    return execution.run();
}

} // namespace dart::app
