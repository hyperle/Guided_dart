

#include <cassert>
#include <initializer_list>
#include <string>
#include <vector>

#include "src/startup/argument_parser.cpp"
#include "src/recording/frame_recording.cpp"

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
    static void run() {
        char program[] = "dart_app";
        char* defaults[] = {program};
        auto parsed = app::parse_args(1, defaults);
        assert(parsed && parsed->record_on && parsed->exposure_lines == 400);
        assert(parsed->record_fps == 60 && parsed->record_keep == 0);
        assert(parsed->record_format == recorder::ImageFormat::kPgm && parsed->threshold == 50);
        assert(parsed->min_contrast == 12);
        assert(!parsed->log_console && std::string_view{parsed->log_path} == app::kDefaultLog);
        assert(parse({"--log-console", "on"})->log_console);
        assert(parse({"--log-console", "1"})->log_console);
        assert(!parse({"--log-console", "off"})->log_console);
        assert(!parse({"--log-console", "0"})->log_console);
        assert(!parse({"--log-console", "maybe"}));
        assert(!parse({"--log-console"}));
        assert(!parse({"--log"}));
        assert(parse({"--log", "off"})->log_path[0] == '\0');
        assert(std::string_view{parse({"--log", "/tmp/x.log"})->log_path} == "/tmp/x.log");
        const auto both = parse({"--log", "off", "--log-console", "on"});
        assert(both && both->log_path[0] == '\0' && both->log_console);
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
            "1510,1520,1530,1540", "--servo-horizontal", "1,-2,0,0.25", "--servo-vertical",
            "-1,0,2,-0.5", "--servo-timeout-ms", "300"});
        assert(servo && servo->aim_explicit && servo->aim.x == 310.5F && servo->aim.y == 235);
        assert(servo->servo.channels[3].initial_pulse_us == 1540);
        assert(servo->servo.channels[1].horizontal_us_per_pixel == -2);
        assert(servo->servo.channels[3].vertical_us_per_pixel == -0.5F);
        assert(servo->servo_timeout_ms == 300);
        assert(!parse({"--pwm", "off"})->pwm_on);
        for (const auto* bad : {"1,2,3", "1,2,3,4,5", "1,2,3,", "1,,2,3", "nan,0,0,0",
                                "0,inf,0,0", "0,0,0,1e50", "0,0,0,2x"})
            assert(!parse({"--servo-horizontal", bad}));
        assert(!parse({"--servo-horizontal"}));
        assert(!parse({"--aim", "-1,20"}));
        assert(!parse({"--aim", "1,2,3"}));
        assert(!parse({"--servo-timeout-ms", "0"}));
        assert(!parse({"--servo-initial", "1300,1520,1520,1520"}));
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

int main() { dart::tests::ArgumentParserTest::run(); }
