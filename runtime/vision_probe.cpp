#include "board/k230.hpp"
#include "logging/logger.hpp"
#include "runtime/component.hpp"
#include "src/detector/detect.hpp"
#include "system/launcher_exit.hpp"
#include "system/monotonic_clock.hpp"
#include "system/run_control.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>

namespace dart::diagnostics {

// Uses the production camera and Detector component. All file logging happens
// outside the measured loop; the frame consumer only counts scalar metadata.
class VisionProbe final {
public:
    struct Settings {
        std::uint32_t exposure_lines = 250;
        std::uint32_t warmup_seconds = 1;
        std::uint32_t measure_seconds = 8;
        std::uint32_t threshold = 50;
        float min_contrast = 12.0F;
    };

    int run() noexcept {
        if (!log_.open("/data/app/logs/vision_probe.log")) {
            std::fprintf(stderr, "vision_probe: cannot open result log\n");
            return 1;
        }
        log_.line("level=INFO module=vision_probe op=start version=1 exposure_lines=%u "
                  "warmup_s=%u measure_s=%u record=0 pwm=0 wifi=0 imu=0\n",
                  settings_.exposure_lines, settings_.warmup_seconds, settings_.measure_seconds);
        if (!log_.flush(std::chrono::milliseconds{3000})) return 1;
        for (const auto& item : std::array<Case, 2>{{{"capture_only", false}, {"detection", true}}}) {
            if (!run_case(item)) {
                log_.line("level=ERROR module=vision_probe op=complete result=failed case=%s\n", item.name);
                (void)log_.flush(std::chrono::milliseconds{3000});
                return 1;
            }
        }
        log_.line("level=INFO module=vision_probe op=complete result=ok\n");
        return log_.flush(std::chrono::milliseconds{3000}) ? 0 : 1;
    }

private:
    using Clock = std::chrono::steady_clock;
    struct Case { const char* name; bool vision; };
    struct Metric {
        std::uint64_t samples = 0, total_us = 0, max_us = 0;
        void add(std::uint64_t us) noexcept {
            ++samples;
            total_us += us;
            max_us = std::max(max_us, us);
        }
        std::uint64_t mean_us() const noexcept { return samples == 0 ? 0 : total_us / samples; }
    };
    struct Result {
        Metric acquire{}, detect{}, step{}, received_gap{}, source_delta_raw{};
        std::uint64_t frames = 0, detected = 0, targets = 0, no_frame = 0;
        std::uint64_t acquire_errors = 0, detect_errors = 0, release_errors = 0;
        std::uint64_t source_nonincreasing = 0;
        std::int64_t first_received_us = 0, last_received_us = 0;
        std::uint64_t last_source_pts = 0;
        bool source_pts_timebase_validated = true;
    };

    static std::uint64_t elapsed_us(Clock::time_point begin, Clock::time_point end) noexcept {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
    }
    void camera_error(const char* operation, const camera::CameraError& error) noexcept {
        char context[1024]{};
        camera::ErrorTextSink sink{context, sizeof(context)};
        error.append_context(sink);
        log_.line("level=ERROR module=vision_probe op=%s native=%d context=%s\n",
                  operation, error.code(), context);
    }
    void metric(const char* name, const char* stage, const Metric& value) noexcept {
        log_.line("level=INFO module=vision_probe op=timing case=%s stage=%s "
                  "samples=%llu mean_us=%llu max_us=%llu\n", name, stage,
                  static_cast<unsigned long long>(value.samples),
                  static_cast<unsigned long long>(value.mean_us()),
                  static_cast<unsigned long long>(value.max_us));
    }
    void report(const Case& item, const Result& result, std::uint64_t window_us) noexcept {
        const auto span_us = result.last_received_us - result.first_received_us;
        const double fps = window_us == 0 ? 0.0 : result.frames * 1'000'000.0 / window_us;
        const double cadence = span_us <= 0 ? 0.0 : (result.frames - 1U) * 1'000'000.0 / span_us;
        log_.line("level=INFO module=vision_probe op=result case=%s vision=%d frames=%llu "
                  "window_us=%llu fps=%.3f cadence_fps=%.3f results=%llu targets=%llu\n",
                  item.name, item.vision, static_cast<unsigned long long>(result.frames),
                  static_cast<unsigned long long>(window_us), fps, cadence,
                  static_cast<unsigned long long>(result.detected),
                  static_cast<unsigned long long>(result.targets));
        log_.line("level=INFO module=vision_probe op=errors case=%s no_frame=%llu "
                  "acquire=%llu detect=%llu release=%llu source_nonincreasing=%llu\n",
                  item.name, static_cast<unsigned long long>(result.no_frame),
                  static_cast<unsigned long long>(result.acquire_errors),
                  static_cast<unsigned long long>(result.detect_errors),
                  static_cast<unsigned long long>(result.release_errors),
                  static_cast<unsigned long long>(result.source_nonincreasing));
        metric(item.name, "acquire_including_wait_and_raw10", result.acquire);
        metric(item.name, "detect", result.detect);
        metric(item.name, "whole_step", result.step);
        metric(item.name, "received_gap", result.received_gap);
        // The SDK PTS epoch and unit have not been calibrated. Preserve raw
        // deltas without presenting them as microseconds or exposure timing.
        log_.line("level=INFO module=vision_probe op=source_pts case=%s validated=%d "
                  "samples=%llu mean_delta_raw=%llu max_delta_raw=%llu\n", item.name,
                  result.source_pts_timebase_validated,
                  static_cast<unsigned long long>(result.source_delta_raw.samples),
                  static_cast<unsigned long long>(result.source_delta_raw.mean_us()),
                  static_cast<unsigned long long>(result.source_delta_raw.max_us));
    }
    bool run_case(const Case& item) noexcept {
        camera::Controls controls{};
        controls.initial_exposure_lines = settings_.exposure_lines;
        auto created = board::create_camera(controls);
        if (!created || !*created) {
            if (!created) camera_error("create", created.error());
            return false;
        }
        auto camera = std::move(*created);
        executor::AtomicValue<detection::Point> target{detection::Point::missing()};
        detection::DetectorStatus status{};
        Result result{};
        bool measuring = false;
        const executor::Component::Spec spec{
            "vision_probe", "detector", 240.0,
            {{"enabled", item.vision ? 1.0 : 0.0},
             {"threshold", static_cast<double>(settings_.threshold)},
             {"workspace.min_contrast", settings_.min_contrast},
             {"acquire_ms", 200.0}, {"max_acquire_errors", 8.0}}};
        auto component = detection::DetectorFactory::create(
            spec, *camera, executor::Output<detection::Point>{target}, status,
            [&](camera::FrameLease& lease, const detection::DetectResult* detected) {
                if (!measuring) return;
                const auto& view = lease.view();
                const auto received = view.received.count();
                if (result.frames != 0) {
                    result.received_gap.add(static_cast<std::uint64_t>(received - result.last_received_us));
                    if (view.source_pts > result.last_source_pts)
                        result.source_delta_raw.add(view.source_pts - result.last_source_pts);
                    else
                        ++result.source_nonincreasing;
                } else {
                    result.first_received_us = received;
                }
                result.last_received_us = received;
                result.last_source_pts = view.source_pts;
                result.source_pts_timebase_validated = result.source_pts_timebase_validated
                                                       && view.source_pts_timebase_validated;
                ++result.frames;
                if (detected != nullptr) {
                    ++result.detected;
                    if (detected->target.valid) ++result.targets;
                }
            });
        if (!component) return false;
        auto detector = std::move(*component);
        if (const auto initialized = detector->initialze(); !initialized) {
            if (status.camera_error) camera_error("start", *status.camera_error);
            return false;
        }
        log_.line("level=INFO module=vision_probe op=case_start case=%s width=%u height=%u\n",
                  item.name, camera->frame_info().width, camera->frame_info().height);
        if (!log_.flush(std::chrono::milliseconds{3000})) {
            (void)detector->shutdown();
            return false;
        }
        platform::RunControl control{settings_.warmup_seconds + settings_.measure_seconds};
        const auto measurement_begin = Clock::now() + std::chrono::seconds{settings_.warmup_seconds};
        bool ok = true;
        while (!control.finished()) {
            const auto before = Clock::now();
            measuring = before >= measurement_begin;
            const auto stepped = detector->step();
            const auto after = Clock::now();
            if (measuring) {
                result.step.add(elapsed_us(before, after));
                result.acquire.add(static_cast<std::uint64_t>(status.acquire_time.count()));
                if (status.acquired_frame && item.vision)
                    result.detect.add(static_cast<std::uint64_t>(status.detect_time.count()));
                if (!status.acquired_frame) ++result.no_frame;
                if (status.camera_error) ++result.acquire_errors;
                if (status.detection_error) ++result.detect_errors;
                if (status.release_error) ++result.release_errors;
            }
            if (!stepped) {
                if (status.camera_error) camera_error("acquire", *status.camera_error);
                ok = false;
                break;
            }
            if (!status.acquired_frame) platform::sleep_ms(1);
        }
        const auto finished = Clock::now();
        const auto window_us = finished > measurement_begin ? elapsed_us(measurement_begin, finished) : 0;
        const auto stopped = detector->shutdown();
        ok = ok && stopped.has_value() && result.frames > 1 && result.acquire_errors == 0
             && result.detect_errors == 0 && result.release_errors == 0;
        report(item, result, window_us);
        detector.reset();
        camera.reset();
        log_.line("level=INFO module=vision_probe op=case_stop case=%s result=%s reason=%s\n",
                  item.name, ok ? "ok" : "failed", control.reason());
        return log_.flush(std::chrono::milliseconds{3000}) && ok;
    }

    Settings settings_{};
    logging::Logger log_{};
};

int run_vision_probe() noexcept {
    VisionProbe probe;
    return probe.run();
}

} // namespace dart::diagnostics

int main() {
    const int result = dart::diagnostics::run_vision_probe();
    dart::platform::notify_launcher_exit();
    return result;
}
