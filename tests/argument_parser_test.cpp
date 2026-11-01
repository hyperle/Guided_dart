#include <expected>
#include <chrono>
#include <cstdio>
#include <string_view>

#include <cassert>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

import dart.app.startup.argument_parser;
import dart.app.recording.frame_recording;

namespace dart::tests {

class ArgumentParserTest final {
    static std::expected<app::Arguments, int> parse(std::initializer_list<const char*> values) {
        std::vector<std::string> strings{"dart_app"};
        for (const auto* value : values)
            strings.emplace_back(value);
        std::vector<char*> arguments;
        for (auto& value : strings)
            arguments.push_back(value.data());
        return app::parse_args(static_cast<int>(arguments.size()), arguments.data());
    }
public:
    // Validate the deployed command with the production parser, before any
    // camera/PWM resources are opened. Unit defaults cannot catch stale bounds.
    static int validate_startup_list(const char* path) {
        std::ifstream input{path};
        if (!input) {
            std::fprintf(stderr, "Cannot read startup list: %s\n", path);
            return 1;
        }
        unsigned applications = 0, line_number = 0;
        for (std::string line; std::getline(input, line);) {
            ++line_number;
            std::istringstream words{line};
            std::vector<std::string> tokens;
            for (std::string word; words >> word;)
                tokens.push_back(word);
            if (tokens.empty() || !std::string_view{tokens.front()}.ends_with("/dart_app"))
                continue;
            ++applications;
            if (line.size() >= 512 || tokens.size() > 32)
                return 1; // Match the launcher's line/token limits, including &.
            if (tokens.back() == "&" || tokens.back() == "&!")
                tokens.pop_back();
            std::vector<char*> arguments;
            for (auto& token : tokens)
                arguments.push_back(token.data());
            const auto parsed = app::parse_args(static_cast<int>(arguments.size()), arguments.data());
            if (!parsed) {
                std::fprintf(stderr, "%s:%u: dart_app startup arguments rejected\n", path, line_number);
                return parsed.error();
            }
        }
        return applications == 1 ? 0 : 1;
    }

    static void run() {
        char program[] = "dart_app";
        char* defaults[] = {program};
        auto parsed = app::parse_args(1, defaults);
        assert(parsed && parsed->record_on && parsed->exposure_lines == 400);
        assert(parsed->record_fps == 60 && parsed->record_keep == 0);
        assert(parsed->record_timeout_ms == 0);
        assert(app::recording_config(*parsed, 640, 480).frame_timeout.count() == 0);
        const auto timeout = parse({"--rec-fps", "60", "--rec-timeout-ms", "100"});
        assert(timeout && timeout->record_fps == 60 && timeout->record_timeout_ms == 100);
        assert(app::recording_config(*timeout, 640, 480).frame_timeout
               == std::chrono::milliseconds{100});
        assert(parse({"--rec-timeout-ms", "0"})->record_timeout_ms == 0);
        assert(parse({"--rec-timeout-ms", "60000"})->record_timeout_ms == 60000);
        assert(!parse({"--rec-timeout-ms", "60001"}));
        assert(!parse({"--rec-timeout-ms", "-1"}));
        assert(!parse({"--rec-timeout-ms", "1.5"}));
        assert(!parse({"--rec-timeout-ms"}));
        assert(parsed->record_format == recorder::ImageFormat::kPgm && parsed->threshold == 50);
        assert(!parsed->wifi_on);
        assert(std::string_view{parsed->wifi_pause_file} == "/sdcard/app/control/stream.pause");
        assert(std::string_view{parse({"--wifi-pause-file", "/tmp/stream.pause"})->wifi_pause_file}
               == "/tmp/stream.pause");
        assert(!parse({"--wifi-pause-file", ""}));
        assert(!parse({"--wifi-pause-file"}));
        assert(parse({"--wifi", "on", "--wifi-config", "/tmp/test.conf"})->wifi_on);
        assert(!parse({"--wifi", "invalid"}));
        assert(!parse({"--wifi-config", ""}));
        assert(!parse({"--wifi-config"}));
        assert(parsed->min_contrast == 12);
        assert(parse({"--min-contrast", "0"})->min_contrast == 0);
        assert(parse({"--min-contrast", "255"})->min_contrast == 255);
        assert(parse({"--min-contrast", "12.5"})->min_contrast == 12.5F);
        assert(!parse({"--min-contrast", "256"}));
        assert(!parse({"--min-contrast", "-1"}));
        assert(!parse({"--min-contrast", "nan"}));
        assert(!parse({"--min-contrast", "inf"}));
        assert(!parse({"--min-contrast"}));
        assert(app::recording_config(*parse({"--threshold", "60"}), 640, 480).pbm_threshold == 60);
        assert(parsed->pwm_on && !parsed->aim_explicit && parsed->servo_timeout_ms == 200);
        const auto servo = parse({"--pwm", "on", "--aim", "310.5,235", "--servo-initial",
            "1510,1520,1530,1540", "--servo-max-delta", "200",
            "--servo-horizontal", "1,-2,0,0.25", "--servo-vertical",
            "-1,0,2,-0.5", "--servo-timeout-ms", "300"});
        assert(servo && servo->aim_explicit && servo->aim.x == 310.5F && servo->aim.y == 235);
        assert(servo->servo.channels[3].initial_pulse_us == 1540);
        assert(servo->servo.channels[1].horizontal_us_per_pixel == -2);
        assert(servo->servo.channels[3].vertical_us_per_pixel == -0.5F);
        assert(servo->servo_timeout_ms == 300);
        assert(parsed->servo.max_delta_us == 500 && servo->servo.max_delta_us == 200);
        assert(parse({"--servo-max-delta", "0"})->servo.max_delta_us == 0);
        assert(parse({"--servo-max-delta", "500.5"})->servo.max_delta_us == 500.5F);
        assert(parse({"--servo-max-delta", "1000"})->servo.max_delta_us == 1000);
        assert(parse({"--servo-max-delta", "500", "--servo-initial", "1445,1490,1150,1475"}));
        for (const auto* bad : {"-1", "nan", "inf", "1e50", "1,2", "2x", ""})
            assert(!parse({"--servo-max-delta", bad}));
        assert(!parse({"--servo-max-delta"}));
        assert(!parse({"--servo-max-delta", "1150"})); // One zero would reach 0 us.
        assert(!parse({"--servo-initial", "500,1490,1150,1475"}));
        assert(!parse({"--servo-initial", "2600,1490,1150,1475"})); // Above the PWM period.
        assert(!parse({"--servo-max-delta", "0", "--servo-initial", "0.0005,1490,1150,1475"}));
        assert(parse({"--servo-max-delta", "0", "--servo-initial", "3000,1490,1150,1475"}));
        assert(!parse({"--pwm", "off"})->pwm_on);
        for (const auto* bad : {"1,2,3", "1,2,3,4,5", "1,2,3,", "1,,2,3", "nan,0,0,0",
                                "0,inf,0,0", "0,0,0,1e50", "0,0,0,2x"})
            assert(!parse({"--servo-horizontal", bad}));
        assert(!parse({"--servo-horizontal"}));
        assert(!parse({"--aim", "-1,20"}));
        assert(!parse({"--aim", "1,2,3"}));
        assert(!parse({"--servo-timeout-ms", "0"}));
        assert(parse({"--servo-initial", "1300,1520,1520,1520"})); // Limits follow the new zeros.
        assert(!parse({"--servo-min", "0,1420,1420,1420"}));
        assert(!parse({"--servo-max", "3100,1620,1620,1620"}));
        assert(app::recording_config(*parsed, 640, 480).max_storage_bytes == 2'000'000'000);
        for (const auto directory : {"/data", "/data/", "/data/custom", "/database", "/tmp/records"}) {
            app::Arguments args;
            std::snprintf(args.rec_dir, sizeof(args.rec_dir), "%s", directory);
            const bool data_volume = std::string_view{directory}.starts_with("/data")
                                  && std::string_view{directory} != "/database";
            assert(app::recording_config(args, 640, 480).max_storage_bytes
                   == (data_volume ? 2'000'000'000 : 0));
        }
        char format[] = "--rec-format";
        char dbv[] = "dbv", png[] = "png", pgm[] = "pgm", pbm[] = "pbm", bad_format[] = "jpg";
        for (char* value : {dbv, png, pgm, pbm}) {
            char* options[] = {program, format, value};
            const auto chosen = app::parse_args(3, options);
            assert(chosen && chosen->record_format == (value == dbv ? recorder::ImageFormat::kDbv
                : value == png ? recorder::ImageFormat::kPng
                : value == pgm ? recorder::ImageFormat::kPgm : recorder::ImageFormat::kPbm));
        }
        char* bad_options[] = {program, format, bad_format};
        assert(!app::parse_args(3, bad_options));
        assert(parsed->raw10_layout == camera::Raw10Layout::kPackedLsb);
        assert(!parsed->imu_on && !parsed->imu_ignore_id);
        char record[] = "--record";
        char off[] = "off";
        char seconds[] = "--seconds";
        char ten[] = "10";
        char* valid[] = {program, record, off, seconds, ten};
        parsed = app::parse_args(5, valid);
        assert(parsed && !parsed->record_on && parsed->seconds == 10);
        char unknown[] = "--unknown";
        char* invalid[] = {program, unknown};
        assert(!app::parse_args(2, invalid));
        char* missing[] = {program, seconds};
        assert(!app::parse_args(2, missing));
        char capture_fps[] = "--fps";
        char capture_size[] = "--acq";
        char output_size[] = "--out";
        char csi[] = "--csi";
        char mirror[] = "--mirror";
        for (char* option : {capture_fps, capture_size, output_size, csi, mirror}) {
            char* old_option[] = {program, option, ten};
            assert(!app::parse_args(3, old_option));
        }
        char rec_fps[] = "--rec-fps";
        char* rate[] = {program, rec_fps, ten};
        parsed = app::parse_args(3, rate);
        assert(parsed && parsed->record_fps == 10);
        char ae[] = "--ae";
        char* automatic_exposure[] = {program, ae};
        parsed = app::parse_args(2, automatic_exposure);
        assert(!parsed);
        char exp[] = "--exp";
        char exposure[] = "300";
        char preserve[] = "0";
        char* fixed_exposure[] = {program, exp, exposure};
        parsed = app::parse_args(3, fixed_exposure);
        assert(parsed && parsed->exposure_lines == 300);
        fixed_exposure[2] = preserve;
        parsed = app::parse_args(3, fixed_exposure);
        assert(parsed && parsed->exposure_lines == 0);
        char* conflicting_exposure[] = {program, ae, exp, ten};
        assert(!app::parse_args(4, conflicting_exposure));
        char raw_layout[] = "--raw-layout";
        char msb[] = "le16-msb";
        char packed[] = "mipi10";
        char* raw_args[] = {program, raw_layout, msb};
        parsed = app::parse_args(3, raw_args);
        assert(parsed && parsed->raw10_layout == camera::Raw10Layout::kLe16Msb);
        raw_args[2] = packed;
        parsed = app::parse_args(3, raw_args);
        assert(parsed && parsed->raw10_layout == camera::Raw10Layout::kMipi);
        char mcm_packed[] = "packed10-lsb";
        raw_args[2] = mcm_packed;
        parsed = app::parse_args(3, raw_args);
        assert(parsed && parsed->raw10_layout == camera::Raw10Layout::kPackedLsb);
        raw_args[2] = ten;
        assert(!app::parse_args(3, raw_args));
        char imu[] = "--imu";
        char on[] = "on";
        char bus[] = "--imu-bus";
        char cs[] = "--imu-cs";
        char zero[] = "0";
        char hal_cs[] = "-1";
        char too_large[] = "64";
        char imu_log[] = "--imu-log";
        char path[] = "/tmp/imu.log";
        char* no_bus[] = {program, imu, on};
        assert(!app::parse_args(3, no_bus));
        char* imu_valid[] = {program, imu, on, bus, zero, cs, hal_cs, imu_log, path};
        parsed = app::parse_args(9, imu_valid);
        assert(parsed && parsed->imu_on && parsed->imu_bus == 0 && parsed->imu_cs == -1);
        assert(std::string_view{parsed->imu_log} == path);
        char* imu_invalid[] = {program, cs, too_large};
        assert(!app::parse_args(3, imu_invalid));
        char empty[] = "";
        char* empty_log[] = {program, imu_log, empty};
        assert(!app::parse_args(3, empty_log));
        char ignore_id[] = "--imu-ignore-id";
        char* bypass[] = {program, imu, on, bus, zero, cs, hal_cs, ignore_id, on};
        parsed = app::parse_args(9, bypass);
        assert(parsed && parsed->imu_on && parsed->imu_ignore_id);
        bypass[8] = off;
        parsed = app::parse_args(9, bypass);
        assert(parsed && !parsed->imu_ignore_id);
        bypass[8] = ten;
        assert(!app::parse_args(9, bypass));
        assert(!app::parse_args(8, bypass));
        char automatic[] = "auto";
        char trigger_accel[] = "--rec-trigger-accel";
        char trigger_ms[] = "--rec-trigger-ms";
        char* auto_missing[] = {program, record, automatic};
        assert(!app::parse_args(3, auto_missing));
        char* auto_valid[] = {program, record, automatic, imu, on, bus, zero, cs, hal_cs,
                             trigger_accel, ten, trigger_ms, ten};
        parsed = app::parse_args(13, auto_valid);
        assert(parsed && parsed->record_on && parsed->record_auto);
        assert(parsed->record_trigger_accel == 10 && parsed->record_trigger_ms == 10);
        auto_valid[12] = zero;
        assert(!app::parse_args(13, auto_valid));
    }
};

} // namespace dart::tests

int main(int argc, char* argv[]) {
    if (argc == 2)
        return dart::tests::ArgumentParserTest::validate_startup_list(argv[1]);
    assert(argc == 1);
    dart::tests::ArgumentParserTest::run();
}
