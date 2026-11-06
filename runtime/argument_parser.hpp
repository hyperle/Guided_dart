#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <string_view>

#include "camera/interface.hpp"
#include "pwm/interface.hpp"
#include "recorder/config.hpp"
#include "src/control/servo.hpp"

namespace dart::app {
class Arguments {
public:
    std::uint32_t seconds = 0; /* 0 = 跑到信号 */
    std::uint32_t exposure_lines = 400;
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
    std::uint32_t record_timeout_ms = 0;
    std::uint32_t record_keep = 0;
    recorder::ImageFormat record_format = recorder::ImageFormat::kPgm;
    camera::Raw10Layout raw10_layout = camera::Raw10Layout::kPackedLsb;
    char rec_dir[512] = "/data/app/records";
    char det_csv[512] = "";    /* 空 = 关闭 */
    char log_path[512] = "/data/app/logs/dart_app.log";
    bool imu_on = false;
    bool imu_ignore_id = false;
    int imu_bus = -1;
    int imu_cs = -2;           /* -2 = 未配置；-1 = HAL 默认片选。 */
    char imu_log[512] = "/data/app/logs/imu.log";
    std::uint32_t status_ms = 1000;
    std::uint32_t acquire_ms = 200;
    std::uint32_t max_acquire_errors = 8;
    std::uint32_t threshold = 50;
    float min_contrast = 12.0F;
};

recorder::Config recording_config(
    const Arguments& args, std::uint32_t width, std::uint32_t height) noexcept;

bool copy_path(char* out, std::size_t cap, std::string_view text) noexcept;

/* 失败值是退出码；帮助返回 0，参数非法返回 2，消息已打印。 */
std::expected<Arguments, int> parse_args(int argc, char* argv[]);

} // namespace dart::app
