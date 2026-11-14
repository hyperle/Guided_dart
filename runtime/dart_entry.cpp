#include "board/k230.hpp"
#include "board/k230_arguments.hpp"
#include "runtime/application.hpp"

int main(int argc, char* argv[]) {
    auto arguments = dart::board::Arguments::parse(argc, argv);
    if (!arguments)
        return arguments.error();
    // The bound configuration lives until the synchronous application returns.
    const dart::platform::CameraFactory camera{
        [](const void* context) noexcept {
            return dart::board::create_camera(
                *static_cast<const dart::board::Arguments::CameraConfig*>(context));
        }, &arguments->camera};
    return dart::app::run_application(
        arguments->application, camera,
        [](const auto& options, auto& log, auto&& service) noexcept {
            return dart::board::with_imu(options, log, service);
        }, dart::board::create_pwm, dart::board::create_wifi_transport,
        dart::board::create_video_encoder);
}
