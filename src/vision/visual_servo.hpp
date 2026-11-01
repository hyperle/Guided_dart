#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "logging/logger.hpp"
#include "pwm/session.hpp"
#include "src/control/servo_mapping.hpp"
#include "src/startup/argument_parser.hpp"
#include "system/device_services.hpp"

namespace dart::app {
class VisualServo final {
public:
    VisualServo(const Arguments& args, logging::Logger& log) noexcept
        : args_(args)
        , log_(log)
        , mapping_(args.servo) {}
    ~VisualServo() { stop(); }

    void start(platform::PwmFactory factory, std::uint32_t width, std::uint32_t height) noexcept;

    // A missing/error observation explicitly clears the preceding command.
    void observe(const detection::DetectResult* detected) noexcept;

    void status() noexcept;

    void stop() noexcept;

private:
    void error(pwm::Error failure) noexcept;

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
