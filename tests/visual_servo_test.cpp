#include <cstddef>
#include <expected>
#include <cassert>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

import dart.app.vision.visual_servo;

import dart.app.startup.argument_parser;
import dart.pwm.interface;

import dart.logging.logger;
import dart.system.device_services;
import dart.system.run_control;
import dart.control.servo_mapping;

namespace dart::tests {

class VisualServoTest final {
    struct Probe {
        std::mutex mutex;
        std::vector<pwm::Pulses> commands;
        unsigned starts = 0, stops = 0;
        bool fail_start = false, fail_write = false;
    };
    inline static Probe* probe_ = nullptr;

    class Device final : public pwm::IPwm {
    public:
        std::expected<void, pwm::Error> start(const pwm::Pulses& initial) noexcept override {
            std::lock_guard lock{probe_->mutex};
            ++probe_->starts;
            if (probe_->fail_start)
                return std::unexpected(pwm::Error{"fake_start", -1, -7});
            probe_->commands.push_back(initial);
            return {};
        }
        std::expected<void, pwm::Error> write(const pwm::Pulses& pulses) noexcept override {
            std::lock_guard lock{probe_->mutex};
            if (probe_->fail_write)
                return std::unexpected(pwm::Error{"fake_write", 2, -8});
            probe_->commands.push_back(pulses);
            return {};
        }
        std::expected<void, pwm::Error> stop() noexcept override {
            std::lock_guard lock{probe_->mutex};
            ++probe_->stops;
            return {};
        }
    };

    static std::expected<std::unique_ptr<pwm::IPwm>, pwm::Error> create() noexcept {
        return std::unique_ptr<pwm::IPwm>{new Device};
    }

    template <typename Predicate>
    static void wait_for(Predicate predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
        while (!predicate()) {
            assert(std::chrono::steady_clock::now() < deadline);
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
        }
    }

    static void expect(Probe& probe, const pwm::Pulses& pulses, std::size_t count) {
        wait_for([&] {
            std::lock_guard lock{probe.mutex};
            return probe.commands.size() >= count && probe.commands.back() == pulses;
        });
    }

public:
    static void run() {
        logging::Logger log;
        app::Arguments args;
        args.servo_timeout_ms = 60;
        args.servo.channels = {{{1510, 1, 2}, {1520, -1, 2},
                               {1530, -1, -2}, {1540, 1, -2}}};
        args.servo.max_delta_us = 160;
        const pwm::Pulses initial{1510, 1520, 1530, 1540};
        Probe probe;
        probe_ = &probe;
        {
            app::VisualServo servo{args, log};
            servo.start(create, 640, 480);
            expect(probe, initial, 1);
            detection::DetectResult observation;
            observation.armor.valid = observation.light.valid = true;
            observation.armor.center = {330, 220};
            observation.light.center = {300, 250};
            // Armor wins, and both axes combine relative to initial positions.
            servo.observe(&observation);
            expect(probe, {1480, 1470, 1560, 1590}, 2);
            // No capture callback: independent timeout must clear the command.
            expect(probe, initial, 3);
            observation.armor.valid = false;
            servo.observe(&observation);
            expect(probe, {1510, 1560, 1530, 1500}, 4);
            servo.observe(nullptr);
            expect(probe, initial, 5);
            // A valid target at the origin is mapped and clamped.
            observation.light.center = {0, 0};
            servo.observe(&observation);
            expect(probe, {1350, 1360, 1690, 1700}, 6);
            observation.light.center.x = std::numeric_limits<float>::quiet_NaN();
            servo.observe(&observation);
            expect(probe, initial, 7);
            servo.stop();
            assert(probe.stops == 1);
        }
        assert(probe.starts == 1 && probe.stops == 1);

        Probe bounded;
        probe_ = &bounded;
        {
            app::Arguments limited;
            limited.servo.channels = {{{1445, 10, 10}, {1490, -10, 10},
                                       {1150, -10, -10}, {1475, 10, -10}}};
            app::VisualServo servo{limited, log};
            servo.start(create, 640, 480);
            expect(bounded, {1445, 1490, 1150, 1475}, 1);
            detection::DetectResult result;
            result.light.valid = true;
            result.light.center = {420, 240};
            servo.observe(&result);
            expect(bounded, {1945, 990, 650, 1975}, 2);
            result.light.center = {420, 140};
            servo.observe(&result);
            expect(bounded, {1445, 990, 1150, 1975}, 3);
            servo.observe(nullptr);
            expect(bounded, {1445, 1490, 1150, 1475}, 4);
            servo.stop();
        }
        assert(bounded.starts == 1 && bounded.stops == 1);

        Probe zero;
        probe_ = &zero;
        {
            app::Arguments defaults;
            app::VisualServo servo{defaults, log};
            servo.start(create, 320, 240);
            detection::DetectResult result;
            result.light.valid = true;
            result.light.center = {300, 200};
            servo.observe(&result);
            std::this_thread::sleep_for(std::chrono::milliseconds{25});
            servo.stop();
        }
        assert(zero.commands.size() == 1); // Explicit zero-gain default.
        assert((zero.commands.front() == pwm::Pulses{1490, 1448, 1475, 1150}));

        Probe failed;
        failed.fail_write = true;
        probe_ = &failed;
        {
            app::VisualServo servo{args, log};
            servo.start(create, 640, 480);
            detection::DetectResult result;
            result.light.valid = true;
            result.light.center = {330, 220};
            servo.observe(&result);
            wait_for([&] {
                std::lock_guard lock{failed.mutex};
                return failed.stops == 1;
            });
            servo.status();
        }
        assert(failed.stops == 1 && failed.commands.size() == 1);

        Probe unavailable;
        unavailable.fail_start = true;
        probe_ = &unavailable;
        {
            app::VisualServo servo{args, log};
            servo.start(create, 640, 480);
            servo.observe(nullptr);
        }
        assert(unavailable.starts == 1 && unavailable.commands.empty());
        {
            args.aim_explicit = true;
            args.aim = {640, 240}; // Outside actual frame: never initialize PWM.
            app::VisualServo servo{args, log};
            servo.start(create, 640, 480);
        }
        assert(unavailable.starts == 1);
    }
};

} // namespace dart::tests

int main() { dart::tests::VisualServoTest::run(); }
