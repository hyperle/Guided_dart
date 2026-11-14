#pragma once

#include <expected>

#include "camera/k230/camera.hpp"
#include "runtime/argument_parser.hpp"

namespace dart::board {

class Arguments final {
public:
    using CameraConfig = dart::camera::k230::Camera<4>::Config;

    Arguments() noexcept { camera.initial_exposure_lines = 400; }

    app::Arguments application{};
    CameraConfig camera{};

    static std::expected<Arguments, int> parse(int argc, char* argv[]);

private:
    static std::expected<bool, int> parse_camera(
        void* context, int argc, char* argv[], int& index) noexcept;
    static void camera_usage() noexcept;
};

} // namespace dart::board
