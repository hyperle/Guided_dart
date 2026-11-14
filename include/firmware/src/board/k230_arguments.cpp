#include "board/k230_arguments.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <expected>
#include <string>
#include <string_view>
#include <utility>

namespace dart::board {

std::expected<Arguments, int> Arguments::parse(int argc, char* argv[]) {
    Arguments result{};
    const app::ArgumentExtension extension{&result.camera, parse_camera, camera_usage};
    auto application = app::parse_args(argc, argv, extension);
    if (!application)
        return std::unexpected(application.error());
    result.application = std::move(*application);
    return result;
}

std::expected<bool, int> Arguments::parse_camera(
    void* context, int argc, char* argv[], int& index) noexcept {
    auto& config = *static_cast<CameraConfig*>(context);
    const std::string_view option{argv[index]};
    if (option == "--ae") {
        std::fprintf(stderr, "%s: 当前相机后端只支持固定曝光（--exp/--gain）\n", argv[0]);
        return std::unexpected(2);
    }
    if (option != "--exp" && option != "--gain" && option != "--raw-layout")
        return false;
    if (index + 1 >= argc) {
        std::fprintf(stderr, "%s: %.*s 缺少参数\n", argv[0],
                     static_cast<int>(option.size()), option.data());
        return std::unexpected(2);
    }
    const std::string_view value{argv[++index]};
    if (option == "--exp") {
        const auto lines = app::parse_int(value);
        if (lines < 0 || lines > UINT32_MAX) {
            std::fprintf(stderr, "%s: --exp 需要非负整数行数\n", argv[0]);
            return std::unexpected(2);
        }
        config.initial_exposure_lines = static_cast<std::uint32_t>(lines);
        return true;
    }
    if (option == "--gain") {
        const std::string text{value};
        char* end = nullptr;
        const double gain = std::strtod(text.c_str(), &end);
        if (text.empty() || end != text.c_str() + text.size() || !(gain == 0.0 || gain >= 1.0)
            || gain > 64.0) {
            std::fprintf(stderr, "%s: --gain 需要 0（保留传感器增益）或 1..64 的倍数\n", argv[0]);
            return std::unexpected(2);
        }
        config.analog_gain_milli = static_cast<std::uint32_t>(gain * 1000.0 + 0.5);
        return true;
    }
    using Layout = dart::camera::k230::Raw10Layout;
    if (value == "le16-lsb")
        config.raw10_layout = Layout::kLe16Lsb;
    else if (value == "le16-msb")
        config.raw10_layout = Layout::kLe16Msb;
    else if (value == "mipi10")
        config.raw10_layout = Layout::kMipi;
    else if (value == "packed10-lsb")
        config.raw10_layout = Layout::kPackedLsb;
    else {
        std::fprintf(stderr, "%s: --raw-layout 需要 packed10-lsb|le16-lsb|le16-msb|mipi10\n",
                     argv[0]);
        return std::unexpected(2);
    }
    return true;
}

void Arguments::camera_usage() noexcept {
    std::printf("  --exp LINES      固定曝光行数（默认 400；0 保留传感器初始曝光）\n");
    std::printf("                   SC132GS 1080x1280@120：1 行约 6.17 us，上限 1342 行\n");
    std::printf("  --gain X         SC132GS 模拟增益倍数 1..28.5（默认 0 保留传感器增益）\n");
    std::printf("  --raw-layout F   packed10-lsb（默认，K230 实板验证）\n");
}

} // namespace dart::board
