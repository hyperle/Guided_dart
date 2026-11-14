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
class CameraFactory {
public:
    using Result = std::expected<std::unique_ptr<camera::ICamera>, camera::CameraError>;
    using Create = Result (*)(const void*) noexcept;

    // The entry owns context until Application::run() finishes. Configuration
    // stays opaque to the application; invoking a factory allocates only at startup.
    constexpr CameraFactory(Create create = nullptr, const void* context = nullptr) noexcept
        : create_(create), context_(context) {}

    explicit operator bool() const noexcept { return create_ != nullptr; }
    Result operator()() const noexcept {
        if (create_ != nullptr)
            return create_(context_);
        camera::CallContext context{};
        context.operation = camera::Operation::kCreate;
        return std::unexpected(camera::CameraError{DART_CAMERA_E_INVALID_STATE, context});
    }

private:
    Create create_;
    const void* context_;
};
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
