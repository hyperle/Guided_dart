#pragma once

#include <expected>

#include "camera/k230/camera.hpp"
#include "logging/logger.hpp"
#include "recorder/encoding/video_encoder.hpp"
#include "runtime/imu/session.hpp"
#include "streaming/interface.hpp"
#include "system/device_services.hpp"
#include <memory>
#include <type_traits>

namespace dart::board {
std::unique_ptr<recorder::IVideoEncoder> create_video_encoder() noexcept;
std::unique_ptr<streaming::ITransport> create_wifi_transport() noexcept;
std::expected<std::unique_ptr<pwm::IPwm>, pwm::Error> create_pwm() noexcept;
std::expected<std::unique_ptr<camera::ICamera>, camera::CameraError>
    create_camera(const camera::k230::Camera<4>::Config& config) noexcept;
using ImuService = int (*)(void*, imu::Session*) noexcept;
int with_imu_service(
    const platform::ImuOptions& options, logging::Logger& log, void* context,
    ImuService service) noexcept;

template <typename Service>
int with_imu(
    const platform::ImuOptions& options, logging::Logger& log, Service&& service) noexcept {
    return with_imu_service(
        options, log, const_cast<void*>(static_cast<const void*>(std::addressof(service))),
        [](void* context, imu::Session* session) noexcept {
            return (*static_cast<std::remove_reference_t<Service>*>(context))(session);
        });
}

} // namespace dart::board
