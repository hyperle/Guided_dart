module;
#include <expected>
#include <memory>

export module dart.system.device_services;

export import dart.camera.interface;
export import dart.imu.sample;
export import dart.pwm.interface;
export import dart.system.write_control;

export namespace dart::platform {

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
