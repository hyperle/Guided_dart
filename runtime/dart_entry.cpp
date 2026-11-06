#include "board/k230.hpp"
#include "runtime/application.hpp"

int main(int argc, char* argv[]) {
    return dart::app::run_application(
        argc, argv, dart::board::create_camera,
        [](const auto& options, auto& log, auto&& service) noexcept {
            return dart::board::with_imu(options, log, service);
        }, dart::board::create_pwm, dart::board::create_wifi_transport,
        dart::board::create_video_encoder);
}
