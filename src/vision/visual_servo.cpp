module;
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

export module dart.app.vision.visual_servo;

import dart.app.startup.argument_parser;
import dart.control.servo_mapping;
import dart.logging.logger;
import dart.pwm.session;
import dart.system.device_services;

export namespace dart::app {

class VisualServo final {
public:
    VisualServo(const Arguments& args, logging::Logger& log) noexcept
        : args_(args), log_(log), mapping_(args.servo) {}
    ~VisualServo() { stop(); }

    void start(platform::PwmFactory factory, std::uint32_t width, std::uint32_t height) noexcept {
        if (!args_.pwm_on)
            return;
        const auto initial = mapping_.pulses();
        aim_ = args_.aim_explicit ? args_.aim
                                 : detection::Point{width * 0.5F, height * 0.5F};
        if (!initial || width == 0 || height == 0 || !std::isfinite(aim_.x)
            || !std::isfinite(aim_.y) || aim_.x < 0 || aim_.y < 0
            || aim_.x >= width || aim_.y >= height) {
            error({"mapping_config", -1, -1});
            return;
        }
        for (const auto& channel : args_.servo.channels) {
            if (channel.min_pulse_us * 1000.0 < 1.0
                || channel.max_pulse_us * 1000.0 > pwm::kPeriodNs) {
                error({"pulse_range", -1, -1});
                return;
            }
        }
        if (factory == nullptr) {
            error({"backend_unavailable", -1, -1});
            return;
        }
        auto created = factory();
        if (!created) {
            error(created.error());
            return;
        }
        device_ = std::move(*created);
        if (!device_) {
            error({"null_device", -1, -1});
            return;
        }
        const auto started = session_.start(*device_, initial->pulse_us, args_.servo_timeout_ms);
        if (!started) {
            error(started.error());
            return;
        }
        active_ = true;
        log_.line(
            "level=INFO module=servo op=start mode=vision frequency_hz=%u update_ms=10 "
            "timeout_ms=%u aim=%.3f,%.3f initial_us=%.3f,%.3f,%.3f,%.3f\n",
            pwm::kFrequencyHz, args_.servo_timeout_ms, aim_.x, aim_.y,
            initial->pulse_us[0], initial->pulse_us[1], initial->pulse_us[2], initial->pulse_us[3]);
        for (std::size_t i = 0; i < args_.servo.channels.size(); ++i) {
            const auto& channel = args_.servo.channels[i];
            log_.line(
                "level=INFO module=servo op=config channel=%u min_us=%.3f max_us=%.3f "
                "horizontal_us_per_px=%.5g vertical_us_per_px=%.5g\n",
                static_cast<unsigned>(i + 1), channel.min_pulse_us, channel.max_pulse_us,
                channel.horizontal_us_per_pixel, channel.vertical_us_per_pixel);
        }
    }

    // A missing/error observation explicitly clears the preceding command.
    void observe(const detection::DetectResult* detected) noexcept {
        if (!active_)
            return;
        const detection::Target none{};
        const auto& armor = detected != nullptr ? detected->armor : none;
        const auto& light = detected != nullptr ? detected->light : none;
        const auto horizontal = mapping_.horizontal_increment(armor, light, aim_);
        const auto vertical = mapping_.vertical_increment(armor, light, aim_);
        if (!horizontal || !vertical) {
            ++mapping_errors_;
            (void)session_.publish(mapping_.pulses()->pulse_us, false);
            return;
        }
        const auto output = mapping_.pulses(horizontal->delta_us, vertical->delta_us);
        if (!output) {
            ++mapping_errors_;
            (void)session_.publish(mapping_.pulses()->pulse_us, false);
            return;
        }
        (void)session_.publish(output->pulse_us, horizontal->source != detection::TargetKind::kNone);
        ++observations_;
        for (const bool saturated : output->saturated)
            saturations_ += saturated;
    }

    void status() noexcept {
        if (!active_)
            return;
        const auto stats = session_.stats();
        log_.line(
            "level=%s module=servo op=status running=%d neutral=%d observations=%llu "
            "writes=%llu timeouts=%llu saturations=%llu mapping_errors=%llu "
            "pulse_us=%.3f,%.3f,%.3f,%.3f\n",
            stats.error.code == 0 ? "INFO" : "ERROR", stats.running, stats.neutral,
            static_cast<unsigned long long>(observations_),
            static_cast<unsigned long long>(stats.writes),
            static_cast<unsigned long long>(stats.timeouts),
            static_cast<unsigned long long>(saturations_),
            static_cast<unsigned long long>(mapping_errors_),
            stats.pulse_us[0], stats.pulse_us[1], stats.pulse_us[2], stats.pulse_us[3]);
        if (!stats.running) {
            error(stats.error);
            stop();
        }
    }

    void stop() noexcept {
        if (!active_)
            return;
        session_.stop();
        const auto stats = session_.stats();
        log_.line("level=%s module=servo op=stop operation=%s channel=%d code=%d\n",
                  stats.error.code == 0 ? "INFO" : "ERROR", stats.error.operation,
                  stats.error.channel, stats.error.code);
        active_ = false;
    }

private:
    void error(pwm::Error failure) noexcept {
        log_.line("level=ERROR module=servo op=error operation=%s channel=%d code=%d "
                  "action=disable_pwm\n", failure.operation, failure.channel, failure.code);
    }

    const Arguments& args_;
    logging::Logger& log_;
    control::ServoMapping mapping_;
    detection::Point aim_{};
    std::unique_ptr<pwm::IPwm> device_;
    pwm::Session session_; // Destroy the session before its device.
    bool active_ = false;
    std::uint64_t observations_ = 0, mapping_errors_ = 0, saturations_ = 0;
};

} // namespace dart::app
