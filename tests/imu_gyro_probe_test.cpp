#include <cstddef>
#include <array>
#include <cassert>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <ctime>

#include "drv_spi.h"

int imu_raw_probe_main(int argc, char** argv);

static std::array<std::array<std::uint8_t, 128>, 2> registers{};
static const char* scenario = nullptr;
static unsigned bank = 0, resets = 0, samples = 0, destroys = 0, repairs = 0;
static std::uint64_t now_us = 0, no_access_until = 0, no_write_until = 0, gyro_on_at = 0, gyro_off_at = 0;
static bool interrupted = false, sample_error_sent = false;
static drv_spi_inst_t handle = reinterpret_cast<drv_spi_inst_t>(0x1234);

static bool mode(const char* name) { return std::strcmp(scenario, name) == 0; }
static bool selftest_mode() { return std::strncmp(scenario, "selftest_", 9) == 0; }
static bool external_mode() {
    return std::strncmp(scenario, "external_", 9) == 0 || selftest_mode();
}

static void reset_registers() {
    registers = {};
    registers[0][0x75] = mode("bad_id") ? 0xff : 0x6f;
    registers[0][0x4c] = mode("bad_format") ? 0xb0 : 0x30;
    registers[0][0x4d] = external_mode() ? 0x99 : 0x91;
    registers[0][0x16] = mode("fifo_stream") ? 0x45 : 0x00;
    registers[0][0x4f] = registers[0][0x50] = 0x06;
    registers[0][0x14] = 0xa0; // Reserved/INT2 settings must survive INT1 setup.
    registers[0][0x51] = 0xb2;
    registers[0][0x52] = 0xa7;
    registers[0][0x53] = 0x0d;
    registers[0][0x56] = 0xfe; // DMP save and APEX engines initially enabled.
    registers[0][0x57] = 0xab; // Active SMD with reserved/WOM fields to preserve.
    registers[0][0x54] = 0xa3;
    registers[0][0x62] = 0x9c;
    registers[0][0x64] = 0xb0;
    registers[1][0x03] = mode("axes_disabled") ? 0xb8 : 0x80;
    registers[1][0x0b] = 0xa0;
    registers[1][0x7a] = 0x03;
    registers[1][0x7b] = 0xab; // Exercise preservation of reserved bits.
    bank = 0;
}

extern "C" int drv_spi_inst_create(int bus, bool active_low, int clock_mode,
    std::uint32_t hz, std::uint8_t bits, int cs, std::uint8_t lines, drv_spi_inst_t* out) {
    assert(bus == 1 && active_low && clock_mode == SPI_HAL_MODE_3);
    assert(hz == 1'000'000 && bits == 8 && cs == 14 && lines == SPI_HAL_DATA_LINE_1);
    *out = handle;
    return 0;
}

extern "C" int drv_spi_write(drv_spi_inst_t spi, const void* tx,
                              std::size_t length, bool release_cs) {
    assert(spi == handle && release_cs && length == 2);
    assert(now_us >= no_access_until && now_us >= no_write_until);
    const auto* bytes = static_cast<const std::uint8_t*>(tx);
    const auto reg = bytes[0], value = bytes[1];
    assert((reg & 0x80) == 0);
    if (reg == 0x76) {
        assert(value <= 1);
        if ((mode("restore_failure") || mode("external_restore_failure")) && bank == 1 && value == 0)
            return -1;
        bank = value;
        return 2;
    }
    if (bank == 0 && reg == 0x11) {
        assert(value == 1);
        // Reset must be preceded by an explicit, verified sensor OFF.
        assert((registers[0][0x4e] & 0x0f) == 0);
        ++resets;
        reset_registers();
        no_access_until = now_us + 1'000;
        return 2;
    }
    if (bank == 0 && reg == 0x4e) {
        const unsigned before = (registers[0][0x4e] >> 2) & 3;
        const unsigned after = (value >> 2) & 3;
        assert(value == 0x00 || value == 0x0f);
        if (before != 0 && after == 0)
            assert(now_us - gyro_on_at >= 45'000);
        if (mode("off_failure") && value == 0)
            return -1;
        if (after == 0)
            gyro_off_at = now_us;
        if (before == 0 && after != 0) {
            assert(now_us - gyro_off_at > 150'000);
            gyro_on_at = now_us;
            no_write_until = now_us + 200;
        }
        if (after != 0) {
            assert(registers[0][0x56] == 0x0a && registers[0][0x57] == 0xa8);
        }
    } else {
        // Vendor self-test explicitly enables excitation while the gyro is ON.
        assert((registers[0][0x4e] & 0x0f) == 0 || (selftest_mode() && reg == 0x70));
        if (bank == 1) {
            if (reg == 0x7a) {
                // DS-000440 p67/p97: a whole-byte 0x02 write also clears the
                // reserved bits this part reports as 1, so the probe must only
                // read SPI_AP_4WIRE, never rewrite INTF_CONFIG4.
                assert(!"INTF_CONFIG4 must never be written");
            } else if (reg == 0x7b) {
                assert(external_mode() && value == 0xad);
                if (mode("external_pin_write_failure"))
                    return -1;
                if (mode("external_pin_mismatch"))
                    return 2; // A successful SPI transaction can still fail readback.
            } else {
                assert(reg == 0x03 && mode("axes_disabled") && value == 0x80);
                ++repairs;
            }
        } else if (reg == 0x4d) {
            assert(external_mode() ? value == 0x9d || value == 0x9c : value == 0x90);
            if (mode("external_rtc_write_failure"))
                return -1;
            if (mode("external_rtc_mismatch"))
                return 2;
        }
        else if (reg == 0x14) {
            assert(external_mode() && value == 0xa3);
            if (mode("external_int1_write_failure"))
                return -1;
            if (mode("external_int1_mismatch"))
                return 2;
        } else if (reg == 0x51 || reg == 0x52 || reg == 0x62 || reg == 0x54 || reg == 0x64) {
            assert(external_mode());
            assert((reg == 0x51 && (value == 0xb6 || (selftest_mode() && value == 0xba)))
                   || (reg == 0x52 && (value == 0xa1 || (selftest_mode() && value == 0xa4)))
                   || value == (reg == 0x62 ? 0x8c : reg == 0x54 ? 0xa9 : 0xa0));
            if (mode("external_vendor_mismatch"))
                return 2;
        } else if (reg == 0x70) {
            assert(selftest_mode() && (value == 7 || value == 0));
            if (mode("selftest_stimulus_write_failure") && value == 7) {
                registers[0][reg] = value; // Simulate an ambiguous short write.
                return 1;
            }
            if (mode("selftest_stimulus_mismatch") && value == 7)
                return 2;
        } else if (reg == 0x56 || reg == 0x57) {
            assert(value == (reg == 0x56 ? 0x0a : 0xa8));
            if (mode("power_save_mismatch") && reg == 0x56)
                return 2;
        } else if (reg == 0x16)
            assert(value == (mode("fifo_stream") ? 0x05 : 0x00));
        else
            assert((reg == 0x4f || reg == 0x50)
                   && (value == 0x06 || (selftest_mode() && reg == 0x4f && value == 0x66)));
    }
    registers[bank][reg] = value;
    return 2;
}

static bool gyro_ready() {
    if (((registers[0][0x4e] >> 2) & 3) != 3
        || (registers[1][0x03] & 0x38) != 0)
        return false;
    if (selftest_mode())
        return !mode("selftest_invalid") && registers[1][0x7a] == 0x03
            && (!mode("selftest_only_stimulus_ready") || (registers[0][0x70] & 7) == 7)
            && (!mode("selftest_baseline_ready") || registers[0][0x4f] == 0x66);
    if (external_mode())
        return !mode("external_invalid") && resets != 0 && (registers[1][0x7b] & 6) == 4
            && (registers[0][0x4d] & 4) != 0
            && (registers[0][0x4d] & 3) == 1
            && (!mode("external_int1_ready") || (registers[0][0x14] & 3) == 3)
            && (!mode("external_vendor_ready") || ((registers[0][0x14] & 3) == 3
                                                  && registers[0][0x51] == 0xb6
                                                  && registers[0][0x62] == 0x8c));
    return mode("ready") || mode("sample_io_failure")
        || (mode("reset_ready") && resets != 0)
        || (mode("rc_ready") && (registers[0][0x4d] & 7) == 0)
        || mode("axes_disabled");
}

extern "C" int drv_spi_transfer(drv_spi_inst_t spi, const void* tx,
    void* rx, std::size_t length, bool release_cs) {
    assert(spi == handle && release_cs && now_us >= no_access_until);
    const auto* bytes = static_cast<const std::uint8_t*>(tx);
    assert(bytes[0] & 0x80);
    const unsigned reg = bytes[0] & 0x7f;
    for (std::size_t i = 1; i < length; ++i)
        assert(bytes[i] == 0);
    std::array<std::uint8_t, 14> frame{};
    const std::array<std::int16_t, 7> values{512, 24, 2048, 48,
        static_cast<std::int16_t>(gyro_ready() ? -1234 : INT16_MIN),
        static_cast<std::int16_t>(gyro_ready() ? 22 : INT16_MIN),
        static_cast<std::int16_t>(gyro_ready() ? -10 : INT16_MIN)};
    for (unsigned i = 0; i < values.size(); ++i) {
        const auto bits = static_cast<std::uint16_t>(values[i]);
        frame[2 * i] = bits >> 8;
        frame[2 * i + 1] = bits & 0xff;
    }
    if (bank == 0 && reg == 0x1d) {
        assert(length == 15);
        assert((registers[0][0x4e] & 0x03) == 0x03); // Accel mode stays LN.
        assert((registers[0][0x16] & 0xc0) == 0); // Read sense registers, never FIFO.
        assert(now_us - gyro_on_at >= 50'000);
        ++samples;
    }
    if (mode("selftest_spi_failure") && bank == 1 && reg == 0x5f)
        return -1; // Factory self-test read that the read-only SPI4 check needs.
    if (mode("sample_io_failure") && reg == 0x2d && !sample_error_sent) {
        sample_error_sent = true;
        return -1;
    }
    auto* out = static_cast<std::uint8_t*>(rx);
    out[0] = 0;
    for (std::size_t i = 1; i < length; ++i) {
        const unsigned address = reg + i - 1;
        out[i] = address == 0x76 ? (mode("bank_mismatch") ? 0xff : bank)
            : bank == 0 && address >= 0x1d && address <= 0x2a
            ? frame[address - 0x1d] : bank == 0 && address == 0x2d
            ? (gyro_ready() ? 0x38 : 0x18) : registers[bank][address];
    }
    return static_cast<int>(length);
}

extern "C" void drv_spi_inst_destroy(drv_spi_inst_t* spi) {
    assert(*spi == handle);
    *spi = nullptr;
    ++destroys;
}

extern "C" int __wrap_nanosleep(const timespec* request, timespec* remaining) {
    const auto delay = request->tv_sec * 1'000'000 + request->tv_nsec / 1'000;
    assert(delay > 0);
    if (mode("external_interrupt_off") && !interrupted && delay == 200'000) {
        now_us += 100'000;
        *remaining = timespec{0, 100'000'000};
        errno = EINTR;
        interrupted = true;
        return -1;
    }
    if (mode("external_stop_off") && delay == 200'000) {
        std::raise(SIGTERM);
        errno = EINTR;
        return -1;
    }
    if (mode("interrupt_wait") && !interrupted && delay == 50'000) {
        now_us += 30'000;
        *remaining = timespec{0, 20'000'000};
        errno = EINTR;
        interrupted = true;
        return -1;
    }
    now_us += delay;
    if (mode("selftest_stop_stimulus") && !interrupted && samples == 601
        && (registers[0][0x70] & 7) == 7) {
        interrupted = true;
        std::raise(SIGTERM);
        errno = EINTR;
        return -1;
    }
    if (samples > 900u || (mode("stop") && samples == 1)) {
        std::raise(SIGTERM);
        errno = EINTR;
        return -1;
    }
    return 0;
}

int main(int argc, char** argv) {
    assert(argc == 2);
    scenario = argv[1];
    reset_registers();
    if (mode("warm_on") || mode("external_warm_on"))
        registers[0][0x4e] = 0x0f;
    char path[] = "/tmp/dart-imu-gyro-test-XXXXXX";
    const int fd = mkstemp(path);
    assert(fd >= 0);
    close(fd);
    char name[] = "imu_raw_probe", flag[] = "--diagnose", external_flag[] = "--external-clock";
    char selftest_flag[] = "--self-test";
    char* arguments[] = {name, path, selftest_mode() ? selftest_flag
                         : external_mode() ? external_flag : flag, nullptr};
    const bool failure = mode("bank_mismatch") || mode("power_save_mismatch") || mode("bad_id") || mode("bad_format") || mode("restore_failure")
        || mode("off_failure") || mode("external_pin_mismatch") || mode("external_rtc_mismatch")
        || mode("external_restore_failure") || mode("external_pin_write_failure")
        || mode("external_rtc_write_failure") || mode("external_int1_write_failure")
        || mode("external_int1_mismatch") || mode("external_vendor_mismatch")
        || mode("selftest_spi_failure") || mode("selftest_stimulus_write_failure")
        || mode("selftest_stimulus_mismatch");
    assert(imu_raw_probe_main(3, arguments) == (failure ? 2 : 0));
    assert(destroys == 1);
    FILE* log = std::fopen(path, "r");
    assert(log != nullptr);
    std::string text;
    std::array<char, 4096> chunk{};
    while (const auto size = std::fread(chunk.data(), 1, chunk.size(), log))
        text.append(chunk.data(), size);
    std::fclose(log);
    assert(unlink(path) == 0);
    assert(text.find(selftest_mode() ? "sensor_init=vendor_selftest_ln_v7"
                    : external_mode() ? "sensor_init=external_clock_ln_v7"
                                   : "sensor_init=gyro_diagnostic_ln_v7") != std::string::npos);
    if (selftest_mode()) {
        assert((registers[0][0x70] & 7) == 0);
        if (mode("selftest_spi_failure")) {
            assert(samples == 0 && text.find("op=setup result=error") != std::string::npos);
        } else if (failure || mode("selftest_stop_stimulus")) {
            assert(samples == (mode("selftest_stop_stimulus") ? 601u : 600u));
            assert((registers[0][0x4e] & 0x0f) == 0);
            assert(text.find("cleanup_ok=1 self_test_config=0x00") != std::string::npos);
            assert(text.find("op=complete") == std::string::npos);
        } else {
            assert(samples == 901 && resets == 3 && bank == 0 && registers[0][0x4e] == 0x0f);
            assert(text.find(mode("selftest_baseline_ready")
                ? "selected=gyro_st_baseline" : "selected=vendor_spi4_clkin") != std::string::npos);
            assert(text.find("cleanup_ok=1 self_test_config=0x00") != std::string::npos);
            assert(text.find(mode("selftest_invalid") ? "op=stimulus_result gyro_valid=0"
                                                     : "op=stimulus_result gyro_valid=300") != std::string::npos);
            if (mode("selftest_only_stimulus_ready"))
                assert(text.find("gyro_valid_cases=0") != std::string::npos);
        }
    } else if (failure) {
        assert(text.find("op=setup result=error") != std::string::npos);
        assert(text.find("op=complete") == std::string::npos);
        assert(samples == (mode("external_int1_write_failure") || mode("external_int1_mismatch")
                           ? 300u : mode("external_vendor_mismatch") ? 600u : 0u));
    } else if (mode("external_stop_off")) {
        assert(samples == 0 && resets == 0 && registers[0][0x4e] == 0);
        assert(text.find("op=complete") == std::string::npos);
    } else if (external_mode()) {
        assert(samples == 901 && resets == 4 && bank == 0);
        assert(registers[0][0x4e] == 0x0f && registers[1][0x7b] == 0xad);
        assert(registers[0][0x4d] == 0x9d && registers[0][0x53] == 0x0d);
        assert(registers[0][0x50] == 6);
        assert(text.find("pin9_function=2 verified=1") != std::string::npos);
        assert(text.find("rtc=1 clksel=1 verified=1") != std::string::npos);
        assert(text.find(mode("external_invalid")
            ? "op=summary samples=300 errors=0 accel_valid=300 gyro_valid=0"
            : "op=summary samples=300 errors=0 accel_valid=300 gyro_valid=300") != std::string::npos);
        const char* selected = mode("external_int1_ready") ? "off200_int1_clkin"
            : mode("external_invalid") || mode("external_vendor_ready")
            ? "vendor_poll_clkin" : "off200_clkin";
        assert(text.find(std::string("selected=") + selected) != std::string::npos);
        assert(text.find("off_wait_ms=200") != std::string::npos);
        assert(text.find("push_pull=1 active_high=1 verified=1") != std::string::npos);
        assert(text.find("op=vendor_config") != std::string::npos);
        if (mode("external_vendor_ready"))
            assert(text.find("gyro_valid_cases=1") != std::string::npos);
        if (mode("external_interrupt_off"))
            assert(interrupted);
    } else if (mode("stop")) {
        assert(samples == 1 && resets == 0);
        assert(text.find("op=complete") == std::string::npos);
    } else {
        assert(samples == 901);
        assert(text.find("case=standby_") == std::string::npos);
        assert(registers[0][0x4e] == 0x0f);
        assert(registers[0][0x56] == 0x0a && registers[0][0x57] == 0xa8);
        const char* selected = mode("ready") ? "as_is"
            : mode("rc_ready") ? "rc_clock"
            : "reset_ln";
        assert(text.find(std::string("selected=") + selected) != std::string::npos);
        if (mode("ready") || mode("reset_ready")
            || mode("rc_ready") || mode("axes_disabled"))
            assert(text.find("gyro=-1234,22,-10") != std::string::npos);
        if (mode("invalid") || mode("interrupt_wait"))
            assert(text.find("gyro_valid_cases=0") != std::string::npos);
        if (mode("axes_disabled"))
            assert(repairs >= 3 && text.find("gyro_disable=0x07") != std::string::npos);
        if (mode("sample_io_failure"))
            assert(text.find("case=as_is op=summary samples=300 errors=1") != std::string::npos);
        assert(text.find("op=quiesce") != std::string::npos);
        assert(text.find("fifo_config=0x") != std::string::npos);
        if (mode("interrupt_wait"))
            assert(interrupted);
    }
}
