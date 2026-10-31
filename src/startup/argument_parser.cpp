#ifndef DART_APP_ARGUMENT_PARSER_CPP
#define DART_APP_ARGUMENT_PARSER_CPP

#include <cerrno>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <string_view>

#include "camera/interface.cpp"
#include "recorder/config.cpp"
#include "pwm/interface.cpp"
#include "src/control/servo_mapping.cpp"

namespace dart::app {

class Arguments {
public:
    std::uint32_t seconds = 0; /* 0 = 跑到信号 */
    std::uint32_t exposure_lines = 400;
    bool vision_on = true;
    bool pwm_on = true;
    control::ServoMapping::Config servo{};
    bool aim_explicit = false;
    detection::Point aim{}; // Otherwise use the actual frame center.
    std::uint32_t servo_timeout_ms = 200;
    bool record_on = true;
    bool record_auto = false;
    std::uint32_t record_trigger_accel = 0;
    std::uint32_t record_trigger_ms = 0;
    std::uint32_t record_duration_ms = 0;
    std::uint32_t record_fps = recorder::Config{}.fps;
    std::uint32_t record_keep = 0;
    recorder::ImageFormat record_format = recorder::ImageFormat::kPgm;
    camera::Raw10Layout raw10_layout = camera::Raw10Layout::kPackedLsb;
    char rec_dir[512] = "/data/app/records";
    char det_csv[512] = "";    /* 空 = 关闭 */
    char log_path[512] = "/data/app/logs/dart_app.log"; /* 空 = 只输出控制台 */
    bool log_console = false;
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

inline constexpr std::string_view kDefaultLog = "/data/app/logs/dart_app.log";

inline constexpr std::string_view kDefaultRecordDir = "/data/app/records";

inline std::int64_t parse_int(std::string_view text) noexcept {
    char buf[32];
    char* end = nullptr;
    long value;
    if (text.empty() || text.size() >= sizeof(buf))
        return -1;
    std::memcpy(buf, text.data(), text.size());
    buf[text.size()] = '\0';
    errno = 0;
    value = std::strtol(buf, &end, 10);
    if (errno != 0 || end == buf || *end != '\0' || value < 0)
        return -1;
    return static_cast<std::int64_t>(value);
}

inline bool parse_on_off(std::string_view text, bool& out) noexcept {
    if (text == "on" || text == "1" || text == "true") {
        out = true;
        return true;
    }
    if (text == "off" || text == "0" || text == "false") {
        out = false;
        return true;
    }
    return false;
}

inline bool copy_path(char* out, std::size_t cap, std::string_view text) noexcept {
    if (text.size() >= cap)
        return false;
    std::memcpy(out, text.data(), text.size());
    out[text.size()] = '\0';
    return true;
}

// Fixed-size comma-separated numbers, accepting signed/fractional gains.
template <std::size_t N>
inline bool parse_floats(std::string_view text, std::array<float, N>& out) noexcept {
    for (std::size_t i = 0; i < N; ++i) {
        const auto comma = text.find(',');
        const auto part = text.substr(0, comma);
        if (part.empty() || part.size() >= 48
            || part.find_first_of(" \t\r\n") != std::string_view::npos)
            return false;
        char buffer[48]{};
        std::memcpy(buffer, part.data(), part.size());
        char* end = nullptr;
        errno = 0;
        const float number = std::strtof(buffer, &end);
        if (errno != 0 || end == buffer || *end != '\0' || !std::isfinite(number))
            return false;
        out[i] = number;
        if (i + 1 == N)
            return comma == std::string_view::npos;
        if (comma == std::string_view::npos)
            return false;
        text.remove_prefix(comma + 1);
    }
    return false;
}

inline void usage(const char* argv0) {
    std::printf("用法: %s [选项]\n\n", argv0);
    std::printf("  --seconds N      有界运行秒数，0 = 跑到 SIGINT/SIGTERM（默认 0）\n");
    std::printf("  --exp LINES      固定曝光行数（默认 400；0 保留传感器初始曝光）\n");
    std::printf("  --raw-layout F   packed10-lsb（默认，K230 实板验证）\n");
    std::printf("  --vision on|off  识别开关（默认 on）\n");
    std::printf("  --pwm on|off     纯视觉四路舵机输出（默认 on；vision off 时停用）\n");
    std::printf("  --aim X,Y        全图预设像素坐标（默认实际图像中心）\n");
    std::printf("  --servo-initial A,B,C,D  四路初始脉宽 µs（默认各 1520）\n");
    std::printf("  --servo-min A,B,C,D      四路最小脉宽 µs（默认各 1420）\n");
    std::printf("  --servo-max A,B,C,D      四路最大脉宽 µs（默认各 1620）\n");
    std::printf("  --servo-horizontal A,B,C,D  左右增益 µs/像素（默认各 0）\n");
    std::printf("  --servo-vertical A,B,C,D    上下增益 µs/像素（默认各 0）\n");
    std::printf("  --servo-timeout-ms N  视觉数据超时回初始位置（默认 200，须大于 0）\n");
    std::printf("  --record on|off|auto  录像策略（默认 on；auto 需 IMU 与显式触发参数）\n");
    std::printf("  --rec-trigger-accel N  加速度模长阈值，单位 m/s²（无默认标定值）\n");
    std::printf("  --rec-trigger-ms N     连续超阈值毫秒数（auto 必填）\n");
    std::printf("  --rec-duration-ms N    触发后录像时长，0 = 持续请求（默认 0）\n");
    std::printf(
        "  --rec-fps N      目标录像帧率，按时间采样（默认 %u FPS）\n", recorder::Config{}.fps);
    std::printf("  --rec-keep N     只保留最近 N 轮录像，0 = 不清理（默认 0）\n");
    std::printf("  --rec-format F   pgm（灰度，默认）| png（灰度）| dbv / pbm（二值）\n");
    std::printf(
        "  --rec-dir DIR    录像根目录（默认 %.*s）\n", static_cast<int>(kDefaultRecordDir.size()),
        kDefaultRecordDir.data());
    std::printf("  --det-csv PATH   识别结果 CSV；off 关闭（默认关闭）\n");
    std::printf("  --threshold N    灰度候选阈值 0..255（默认 50），不改变相机灰度图\n");
    std::printf("  --min-contrast N 候选与背景最小灰度差 0..255，可小数（默认 12，0 关闭）\n");
    std::printf(
        "  --log PATH       结构化日志；off 只关文件日志（默认 %.*s）\n",
        static_cast<int>(kDefaultLog.size()), kDefaultLog.data());
    std::printf("  --log-console on|off  日志同时写 stderr，前台串口调试用（默认 off）\n");
    std::printf("  --status-ms N    状态行间隔（默认 1000）\n");
    std::printf("  --imu on|off     IMU 独立采样与日志（默认 off）\n");
    std::printf("  --imu-bus N      SPI 总线 0..2，启用 IMU 时必填\n");
    std::printf("  --imu-cs N       片选引脚 0..63，-1 使用 HAL 默认，启用 IMU 时必填\n");
    std::printf("  --imu-log PATH   IMU 日志基名，每次启动加编号（默认 /data/app/logs/imu.log）\n");
    std::printf("  --imu-ignore-id on|off  跳过 WHO_AM_I 校验并记录原始寄存器（诊断用，默认 off）\n");
    std::printf("  --acquire-ms N   单次取帧超时（默认 200）\n");
    std::printf("  --max-acquire-errors N  连续取帧错误上限，超过即退出（默认 8）\n");
    std::printf("  -h, --help       显示本帮助\n");
}

/* 失败值是退出码；帮助返回 0，参数非法返回 2，消息已打印。 */
inline std::expected<Arguments, int> parse_args(int argc, char* argv[]) {
    Arguments args;
    args.det_csv[0] = '\0';

    for (int i = 1; i < argc; ++i) {
        const std::string_view a{argv[i]};
        const auto need_value = [&](std::string_view& value) {
            if (i + 1 >= argc)
                return false;
            value = std::string_view{argv[++i]};
            return true;
        };

        if (a == "-h" || a == "--help") {
            usage(argv[0]);
            return std::unexpected<int>(0);
        }
        std::string_view value;
        if (a == "--aim" || a == "--servo-initial" || a == "--servo-min"
            || a == "--servo-max" || a == "--servo-horizontal" || a == "--servo-vertical") {
            std::array<float, 4> numbers{};
            std::array<float, 2> point{};
            if (!need_value(value)
                || !(a == "--aim" ? parse_floats(value, point) : parse_floats(value, numbers))) {
                std::fprintf(stderr, "%s: %.*s 需要 %s 个逗号分隔的有限数值\n", argv[0],
                             static_cast<int>(a.size()), a.data(), a == "--aim" ? "2" : "4");
                return std::unexpected<int>(2);
            }
            if (a == "--aim") {
                args.aim = {point[0], point[1]};
                args.aim_explicit = true;
            } else {
                for (std::size_t channel = 0; channel < numbers.size(); ++channel) {
                    auto& config = args.servo.channels[channel];
                    if (a == "--servo-initial") config.initial_pulse_us = numbers[channel];
                    else if (a == "--servo-min") config.min_pulse_us = numbers[channel];
                    else if (a == "--servo-max") config.max_pulse_us = numbers[channel];
                    else if (a == "--servo-horizontal") config.horizontal_us_per_pixel = numbers[channel];
                    else config.vertical_us_per_pixel = numbers[channel];
                }
            }
            continue;
        }
        if (a == "--imu-bus" || a == "--imu-cs") {
            if (!need_value(value)) {
                std::fprintf(
                    stderr, "%s: %.*s 缺少参数\n", argv[0], static_cast<int>(a.size()), a.data());
                return std::unexpected<int>(2);
            }
            const auto n = a == "--imu-cs" && value == "-1" ? -1 : parse_int(value);
            if ((n < 0 && !(a == "--imu-cs" && value == "-1")) || n > (a == "--imu-bus" ? 2 : 63)) {
                std::fprintf(
                    stderr, "%s: %.*s 超出范围\n", argv[0], static_cast<int>(a.size()), a.data());
                return std::unexpected<int>(2);
            }
            if (a == "--imu-bus")
                args.imu_bus = static_cast<int>(n);
            else
                args.imu_cs = static_cast<int>(n);
            continue;
        }
        if (a == "--min-contrast") {
            std::array<float, 1> number{};
            if (!need_value(value) || !parse_floats(value, number)
                || number[0] < 0.0F || number[0] > 255.0F) {
                std::fprintf(stderr, "%s: --min-contrast 必须是 0..255 的有限数值\n", argv[0]);
                return std::unexpected<int>(2);
            }
            args.min_contrast = number[0];
            continue;
        }
        if (a == "--seconds" || a == "--exp" || a == "--rec-fps" || a == "--rec-keep"
            || a == "--threshold" || a == "--status-ms" || a == "--acquire-ms"
            || a == "--max-acquire-errors" || a == "--rec-trigger-accel"
            || a == "--rec-trigger-ms" || a == "--rec-duration-ms" || a == "--servo-timeout-ms") {
            if (!need_value(value)) {
                std::fprintf(
                    stderr, "%s: %.*s 缺少参数\n", argv[0], static_cast<int>(a.size()), a.data());
                return std::unexpected<int>(2);
            }
            const auto n = parse_int(value);
            if (n < 0 || n > UINT32_MAX) {
                std::fprintf(
                    stderr, "%s: %.*s 需要非负整数，得到 %.*s\n", argv[0],
                    static_cast<int>(a.size()), a.data(), static_cast<int>(value.size()),
                    value.data());
                return std::unexpected<int>(2);
            }
            if (a == "--servo-timeout-ms")
                args.servo_timeout_ms = static_cast<std::uint32_t>(n);
            else if (a == "--rec-trigger-accel")
                args.record_trigger_accel = static_cast<std::uint32_t>(n);
            else if (a == "--rec-trigger-ms")
                args.record_trigger_ms = static_cast<std::uint32_t>(n);
            else if (a == "--rec-duration-ms")
                args.record_duration_ms = static_cast<std::uint32_t>(n);
            else if (a == "--seconds")
                args.seconds = static_cast<std::uint32_t>(n);
            else if (a == "--exp") {
                args.exposure_lines = static_cast<std::uint32_t>(n);
            } else if (a == "--rec-fps") {
                if (n == 0 || n > 1000000) {
                    std::fprintf(stderr, "%s: --rec-fps 必须在 1..1000000\n", argv[0]);
                    return std::unexpected<int>(2);
                }
                args.record_fps = static_cast<std::uint32_t>(n);
            } else if (a == "--rec-keep")
                args.record_keep = static_cast<std::uint32_t>(n);
            else if (a == "--threshold")
                args.threshold = static_cast<std::uint32_t>(n);
            else if (a == "--status-ms")
                args.status_ms = static_cast<std::uint32_t>(n);
            else if (a == "--acquire-ms")
                args.acquire_ms = static_cast<std::uint32_t>(n);
            else
                args.max_acquire_errors = static_cast<std::uint32_t>(n);
            continue;
        }
        if (a == "--vision" || a == "--record" || a == "--imu" || a == "--imu-ignore-id"
            || a == "--pwm") {
            if (!need_value(value)) {
                std::fprintf(
                    stderr, "%s: %.*s 缺少参数\n", argv[0], static_cast<int>(a.size()), a.data());
                return std::unexpected<int>(2);
            }
            bool flag = false;
            if (a == "--record" && value == "auto") {
                args.record_on = args.record_auto = true;
                continue;
            }
            if (!parse_on_off(value, flag)) {
                std::fprintf(
                    stderr, "%s: %.*s 需要 on|off\n", argv[0], static_cast<int>(a.size()),
                    a.data());
                return std::unexpected<int>(2);
            }
            if (a == "--vision")
                args.vision_on = flag;
            else if (a == "--pwm")
                args.pwm_on = flag;
            else if (a == "--imu")
                args.imu_on = flag;
            else if (a == "--imu-ignore-id")
                args.imu_ignore_id = flag;
            else {
                args.record_on = flag;
                args.record_auto = false;
            }
            continue;
        }
        if (a == "--ae") {
            std::fprintf(stderr, "%s: 绕过 ISP 的 RAW 路径暂不支持自动曝光\n", argv[0]);
            return std::unexpected<int>(2);
        }
        if (a == "--raw-layout") {
            if (!need_value(value)) {
                std::fprintf(stderr, "%s: --raw-layout 缺少参数\n", argv[0]);
                return std::unexpected<int>(2);
            }
            if (value == "le16-lsb")
                args.raw10_layout = camera::Raw10Layout::kLe16Lsb;
            else if (value == "le16-msb")
                args.raw10_layout = camera::Raw10Layout::kLe16Msb;
            else if (value == "mipi10")
                args.raw10_layout = camera::Raw10Layout::kMipi;
            else if (value == "packed10-lsb")
                args.raw10_layout = camera::Raw10Layout::kPackedLsb;
            else {
                std::fprintf(
                    stderr, "%s: --raw-layout 需要 packed10-lsb|le16-lsb|le16-msb|mipi10\n",
                    argv[0]);
                return std::unexpected<int>(2);
            }
            continue;
        }
        if (a == "--rec-format") {
            if (!need_value(value)) {
                std::fprintf(stderr, "%s: --rec-format 缺少参数\n", argv[0]);
                return std::unexpected<int>(2);
            }
            if (value == "pgm")
                args.record_format = recorder::ImageFormat::kPgm;
            else if (value == "pbm")
                args.record_format = recorder::ImageFormat::kPbm;
            else if (value == "dbv")
                args.record_format = recorder::ImageFormat::kDbv;
            else if (value == "png")
                args.record_format = recorder::ImageFormat::kPng;
            else {
                std::fprintf(stderr, "%s: --rec-format 需要 dbv|png|pgm|pbm\n", argv[0]);
                return std::unexpected<int>(2);
            }
            continue;
        }
        if (a == "--rec-dir" || a == "--det-csv" || a == "--log" || a == "--imu-log" ||
            a == "--log-console") {
            if (!need_value(value)) {
                std::fprintf(
                    stderr, "%s: %.*s 缺少参数\n", argv[0], static_cast<int>(a.size()), a.data());
                return std::unexpected<int>(2);
            }
            if (a == "--imu-log") {
                if (value.empty() || !copy_path(args.imu_log, sizeof(args.imu_log), value)) {
                    std::fprintf(stderr, "%s: --imu-log 路径为空或过长\n", argv[0]);
                    return std::unexpected<int>(2);
                }
            } else if (a == "--rec-dir") {
                if (!copy_path(args.rec_dir, sizeof(args.rec_dir), value)) {
                    std::fprintf(stderr, "%s: --rec-dir 路径过长\n", argv[0]);
                    return std::unexpected<int>(2);
                }
            } else if (a == "--det-csv") {
                if (value == "off") {
                    args.det_csv[0] = '\0';
                } else if (!copy_path(args.det_csv, sizeof(args.det_csv), value)) {
                    std::fprintf(stderr, "%s: --det-csv 路径过长\n", argv[0]);
                    return std::unexpected<int>(2);
                }
            } else if (a == "--log") {
                if (value == "off") {
                    args.log_path[0] = '\0';
                } else if (!copy_path(args.log_path, sizeof(args.log_path), value)) {
                    std::fprintf(stderr, "%s: --log 路径过长\n", argv[0]);
                    return std::unexpected<int>(2);
                }
            } else if (!parse_on_off(value, args.log_console)) {
                std::fprintf(stderr, "%s: --log-console 需要 on|off\n", argv[0]);
                return std::unexpected<int>(2);
            }
            continue;
        }
        std::fprintf(
            stderr, "%s: 未知选项 %.*s（-h 查看用法）\n", argv[0], static_cast<int>(a.size()),
            a.data());
        return std::unexpected<int>(2);
    }

    if (!control::ServoMapping{args.servo}.valid_config() || args.servo_timeout_ms == 0) {
        std::fprintf(stderr, "%s: 舵机配置需要 0 < min <= initial <= max，超时须大于 0\n", argv[0]);
        return std::unexpected<int>(2);
    }
    for (const auto& channel : args.servo.channels) {
        if (static_cast<double>(channel.min_pulse_us) * 1000.0 < 1.0
            || static_cast<double>(channel.max_pulse_us) * 1000.0 > pwm::kPeriodNs) {
            std::fprintf(stderr, "%s: 舵机脉宽必须在 1 ns 到 333 Hz 周期范围内\n", argv[0]);
            return std::unexpected<int>(2);
        }
    }
    if (args.aim_explicit && (args.aim.x < 0.0F || args.aim.y < 0.0F)) {
        std::fprintf(stderr, "%s: --aim 坐标必须非负\n", argv[0]);
        return std::unexpected<int>(2);
    }
    if (args.record_auto && (!args.imu_on || args.record_trigger_accel == 0
                            || args.record_trigger_ms == 0)) {
        std::fprintf(stderr, "%s: --record auto 需要 IMU、--rec-trigger-accel 和 --rec-trigger-ms\n", argv[0]);
        return std::unexpected<int>(2);
    }
    if (args.imu_on && (args.imu_bus < 0 || args.imu_cs < -1)) {
        std::fprintf(stderr, "%s: --imu on 需要显式指定 --imu-bus 和 --imu-cs\n", argv[0]);
        return std::unexpected<int>(2);
    }
    if (args.record_keep > 9999) {
        std::fprintf(stderr, "%s: --rec-keep 必须在 0..9999\n", argv[0]);
        return std::unexpected<int>(2);
    }
    if (args.status_ms == 0)
        args.status_ms = 1000;
    if (args.acquire_ms == 0)
        args.acquire_ms = 200;
    if (args.threshold > 255) {
        std::fprintf(stderr, "%s: --threshold 必须在 0..255\n", argv[0]);
        return std::unexpected<int>(2);
    }
    return args;
}

} // namespace dart::app

#endif // DART_APP_ARGUMENT_PARSER_CPP
