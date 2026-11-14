#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <string_view>

#include "pwm/interface.hpp"
#include "recorder/config.hpp"
#include "src/control/servo.hpp"

namespace dart::app {
class Arguments {
public:
    std::uint32_t seconds = 0; /* 0 = 跑到信号 */
    bool wifi_on = false;
    char wifi_config[512] = "/data/app/wifi.conf";
    char wifi_pause_file[512] = "/sdcard/app/control/stream.pause";
    bool vision_on = true;
    bool pwm_on = true;
    control::ServoConfig servo{};
    bool aim_explicit = false;
    detection::Point aim{};    // Otherwise use the actual frame center.
    std::uint32_t servo_timeout_ms = 200;
    bool record_on = true;
    bool record_auto = false;
    std::uint32_t record_trigger_accel = 0;
    std::uint32_t record_trigger_ms = 0;
    std::uint32_t record_duration_ms = 0;
    std::uint32_t record_fps = recorder::Config{}.fps;
    std::uint32_t record_queue = recorder::Config{}.queue_slots;
    std::uint32_t record_timeout_ms = 0;
    std::uint32_t record_keep = 0;
    recorder::ImageFormat record_format = recorder::ImageFormat::kPgm;
    char rec_dir[512] = "/data/app/records";
    char det_csv[512] = "";    /* 空 = 关闭 */
    char log_path[512] = "/data/app/logs/dart_app.log";
    bool log_console = false;
    bool imu_on = false;
    bool imu_ignore_id = false;
    int imu_bus = -1;
    int imu_cs = -2;           /* -2 = 未配置；-1 = HAL 默认片选。 */
    char imu_log[512] = "/data/app/logs/imu.log";
    std::uint32_t status_ms = 1000;
    std::uint32_t acquire_ms = 200;
    std::uint32_t max_acquire_errors = 8;
    std::uint32_t threshold = 50; /* 录像 pbm/dbv 二值阈值 */
    // --detect KEY=VALUE in command-line order; keys are Detect::Config paths.
    struct DetectParameter {
        char name[48] = "";
        double value = 0.0;
    };
    static constexpr std::size_t kMaxDetectParameters = 32;
    std::array<DetectParameter, kMaxDetectParameters> detect{};
    std::size_t detect_count = 0;
};

recorder::Config recording_config(
    const Arguments& args, std::uint32_t width, std::uint32_t height) noexcept;

bool copy_path(char* out, std::size_t cap, std::string_view text) noexcept;
std::int64_t parse_int(std::string_view text) noexcept;

class ArgumentExtension {
public:
    // A recognized option returns true and advances index past its values.
    // An unrecognized option returns false without changing index. The entry
    // owns context for the duration of parse_args(); errors are exit codes.
    using Parse = std::expected<bool, int> (*)(void*, int, char*[], int&) noexcept;
    void* context = nullptr;
    Parse parse = nullptr;
    void (*usage)() noexcept = nullptr;
};

/* 失败值是退出码；帮助返回 0，参数非法返回 2，消息已打印。 */
std::expected<Arguments, int> parse_args(
    int argc, char* argv[], ArgumentExtension extension = {});

} // namespace dart::app
