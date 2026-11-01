#pragma once

#include <expected>
#include <memory>

#include "camera/interface.hpp"
#include "imu/sample.hpp"
#include "pwm/interface.hpp"
#include "system/write_control.hpp"

namespace dart::platform {
// Device selection is supplied by the process entry. Business modules use these
// callbacks without selecting a board backend or configuring hardware registers.
using CameraFactory = std::expected<std::unique_ptr<camera::ICamera>, camera::CameraError> (*)(
    const camera::Controls&) noexcept;
using PwmFactory = std::expected<std::unique_ptr<pwm::IPwm>, pwm::Error> (*)() noexcept;

struct ImuOptions {
    bool enabled = false;
    int bus = -1;
    int chip_select = -2;
    const char* log_path = nullptr;
    bool ignore_device_id = false;
    WriteControl* writes = nullptr;
    void (*sample_observer)(const imu::Sample&, void*) noexcept = nullptr;
    void* sample_observer_context = nullptr;
};

} // namespace dart::platform
