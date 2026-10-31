#include <cstddef>
#define _POSIX_C_SOURCE 200809L

// Scope diagnostic: wake the sensor, then continuously read raw registers.
// Keep the wake sequence explicit so its SPI commands are visible on the scope.
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <unistd.h>

#include "drv_spi.h"

import dart.logging.logger;
import dart.system.monotonic_clock;

namespace dart::imu::raw_probe {

constexpr int kBus = 1;
constexpr int kChipSelect = 14;
constexpr std::uint32_t kBusHz = 1'000'000;
constexpr long kPeriodMs = 10;
constexpr long kWakeDelayMs = 50;
// InvenSense public.mcu.iim42652: gyro OFF->ON must be separated by >150 ms.
constexpr long kGyroOffDelayMs = 200;
volatile std::sig_atomic_t running = 1;

void stop(int) noexcept { running = 0; }

bool pause_ms(long milliseconds) noexcept {
    timespec remaining{milliseconds / 1000, (milliseconds % 1000) * 1'000'000};
    while (running && nanosleep(&remaining, &remaining) != 0) {
        if (errno != EINTR)
            return false;
    }
    return true;
}

bool verify_bank(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                 std::uint8_t expected) noexcept;
bool disable_power_save(drv_spi_inst_t spi, logging::Logger& log, const char* name) noexcept;

bool wake_sensor(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    // IIM-42652 DS-000440: Bank 0 PWR_MGMT0 defaults to 0 (both sensors OFF).
    // Both sensors measure continuously in Low Noise (LN), never Low Power or Standby.
    // Quiesce both sensors before touching FIFO configuration (DS-000440 p63).
    constexpr std::array<std::array<std::uint8_t, 2>, 4> commands{{
        {0x76, 0x00}, {0x4e, 0x00}, {0x16, 0x00}, {0x4e, 0x0f}}};
    for (std::size_t i = 0; i < commands.size(); ++i) {
        // A CPU-only reset can leave the IMU ON. Allow its minimum ON duration
        // before switching OFF; then allow >150 ms for proof-mass ring-down.
        if ((i == 1 && !pause_ms(50)) || (i == 2 && !pause_ms(kGyroOffDelayMs)))
            return false;
        if (!running)
            return true;
        if (i == 3 && !disable_power_save(spi, log, "wake_ln"))
            return false;
        const auto& command = commands[i];
        const int rc = drv_spi_write(spi, command.data(), command.size(), true);
        const bool ok = rc == static_cast<int>(command.size());
        log.line("imu_raw op=wake_write register=0x%02x value=0x%02x rc=%d result=%s\n",
                 static_cast<unsigned>(command[0]), static_cast<unsigned>(command[1]),
                 rc, ok ? "ok" : "error");
        if (!ok || (i == 0 && !verify_bank(spi, log, "wake_ln", 0)))
            return false;
    }
    // No register accesses during startup; this also covers the >=200 us
    // write prohibition after OFF->ON. A fixed delay is not a data-ready check.
    if (!pause_ms(kWakeDelayMs)) {
        log.line("imu_raw op=wake result=wait_error errno=%d\n", errno);
        return false;
    }
    if (!running)
        return true;
    constexpr std::array<std::uint8_t, 2> power_tx{0xce, 0x00};
    std::array<std::uint8_t, 2> power_rx{};
    const int rc = drv_spi_transfer(
        spi, power_tx.data(), power_rx.data(), power_tx.size(), true);
    // Never start measurement unless both sensors are confirmed in LN mode.
    log.line("imu_raw op=wake wait_ms=%ld read_rc=%d pwr_mgmt0=0x%02x expected=0x0f "
             "gyro_mode=%u accel_mode=%u verified=%u\n", kWakeDelayMs, rc, static_cast<unsigned>(power_rx[1]),
             static_cast<unsigned>((power_rx[1] >> 2) & 3),
             static_cast<unsigned>(power_rx[1] & 3),
             static_cast<unsigned>(rc == static_cast<int>(power_tx.size()) && power_rx[1] == 0x0f));
    return rc == static_cast<int>(power_tx.size()) && power_rx[1] == 0x0f;
}

struct DiagnosticCase {
    const char* name;
    bool reset;
    bool rc_clock;
    bool external_clock = false;
    bool int1_push_pull = false;
    bool vendor_filters = false;
    bool inspect_spi4 = false;
    bool selftest_settings = false;
};

bool diagnostic_write(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                      std::uint8_t reg, std::uint8_t value) noexcept {
    const std::array<std::uint8_t, 2> tx{reg, value};
    const int rc = drv_spi_write(spi, tx.data(), tx.size(), true);
    log.line("imu_diag case=%s op=write reg=0x%02x value=0x%02x rc=%d\n",
             name, static_cast<unsigned>(reg), static_cast<unsigned>(value), rc);
    if (rc != static_cast<int>(tx.size()))
        return false;
    if (reg == 0x76 && !verify_bank(spi, log, name, value)) {
        // Best effort Bank 0 restoration if selecting a bank failed readback.
        if (value != 0)
            (void)diagnostic_write(spi, log, name, 0x76, 0x00);
        return false;
    }
    return true;
}

bool diagnostic_read(drv_spi_inst_t spi, std::uint8_t reg,
                     std::uint8_t* out, std::size_t length) noexcept {
    if (length == 0 || length > 14)
        return false;
    std::array<std::uint8_t, 15> tx{}, rx{};
    tx[0] = reg | 0x80;
    if (drv_spi_transfer(spi, tx.data(), rx.data(), length + 1, true)
        != static_cast<int>(length + 1))
        return false;
    std::memcpy(out, rx.data() + 1, length);
    return true;
}

bool verify_bank(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                 std::uint8_t expected) noexcept {
    std::uint8_t actual = 0xff;
    const bool read_ok = diagnostic_read(spi, 0x76, &actual, 1);
    const bool verified = read_ok && actual == expected;
    log.line("imu_diag case=%s op=bank expected=%u readback=0x%02x read_ok=%u verified=%u\n",
             name, static_cast<unsigned>(expected), static_cast<unsigned>(actual),
             static_cast<unsigned>(read_ok), static_cast<unsigned>(verified));
    return verified;
}

// Bank 0, sensors OFF: disable DMP power saving and unused motion engines.
// DS-000440 p86: preserve APEX bit 3/DMP_ODR and all unrelated SMD bits.
bool disable_power_save(drv_spi_inst_t spi, logging::Logger& log, const char* name) noexcept {
    constexpr std::array<std::array<std::uint8_t, 2>, 2> updates{{
        {0x56, 0xf4}, {0x57, 0x03}}};
    for (const auto& update : updates) {
        std::uint8_t before = 0, applied = 0;
        if (!diagnostic_read(spi, update[0], &before, 1))
            return false;
        const std::uint8_t expected = before & ~update[1];
        if (!diagnostic_write(spi, log, name, update[0], expected)
            || !diagnostic_read(spi, update[0], &applied, 1))
            return false;
        log.line("imu_diag case=%s op=disable_power_save reg=0x%02x before=0x%02x "
                 "applied=0x%02x verified=%u\n", name,
                 static_cast<unsigned>(update[0]), static_cast<unsigned>(before),
                 static_cast<unsigned>(applied), static_cast<unsigned>(applied == expected));
        if (applied != expected)
            return false;
    }
    return true;
}

std::int16_t diagnostic_word(const std::uint8_t* bytes) noexcept {
    const std::uint16_t bits = (static_cast<std::uint16_t>(bytes[0]) << 8) | bytes[1];
    std::int16_t value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool diagnostic_bank0(drv_spi_inst_t spi, logging::Logger& log,
                      const char* name, const char* stage,
                      std::uint8_t expected_power = 0xff) noexcept {
    if (!verify_bank(spi, log, name, 0))
        return false;
    std::array<std::uint8_t, 5> config{};
    std::uint8_t id = 0, fsync = 0, fifo = 0;
    if (!diagnostic_read(spi, 0x75, &id, 1)
        || !diagnostic_read(spi, 0x4c, config.data(), config.size())
        || !diagnostic_read(spi, 0x62, &fsync, 1)
        || !diagnostic_read(spi, 0x16, &fifo, 1)) {
        log.line("imu_diag case=%s op=config stage=%s result=read_error\n", name, stage);
        return false;
    }
    log.line("imu_diag case=%s op=config stage=%s id=0x%02x intf0=0x%02x "
             "intf1=0x%02x power=0x%02x gyro_cfg=0x%02x accel_cfg=0x%02x "
             "fsync=0x%02x rtc=%u clksel=%u gyro_mode=%u accel_mode=%u "
             "fifo_config=0x%02x access=polling\n",
             name, stage, static_cast<unsigned>(id), static_cast<unsigned>(config[0]),
             static_cast<unsigned>(config[1]), static_cast<unsigned>(config[2]),
             static_cast<unsigned>(config[3]), static_cast<unsigned>(config[4]),
             static_cast<unsigned>(fsync), static_cast<unsigned>((config[1] >> 2) & 1),
             static_cast<unsigned>(config[1] & 3), static_cast<unsigned>((config[2] >> 2) & 3),
             static_cast<unsigned>(config[2] & 3), static_cast<unsigned>(fifo));
    // The decoded validity check below requires default invalid-code insertion
    // and big-endian data. Do not silently decode another interface format.
    return id == 0x6f && (config[0] & 0xf0) == 0x30
        && (expected_power == 0xff || (config[2] == expected_power && (fifo & 0xc0) == 0));
}

bool diagnostic_setup(drv_spi_inst_t spi, logging::Logger& log,
                      const DiagnosticCase& test) noexcept {
    if (!diagnostic_write(spi, log, test.name, 0x76, 0x00))
        return false;
    std::uint8_t id = 0;
    if (!diagnostic_read(spi, 0x75, &id, 1) || id != 0x6f) {
        log.line("imu_diag case=%s op=identify id=0x%02x result=error\n",
                 test.name, static_cast<unsigned>(id));
        return false;
    }
    std::uint8_t previous_power = 0, off_power = 0;
    if (!diagnostic_read(spi, 0x4e, &previous_power, 1))
        return false;
    if ((previous_power & 0x0f) != 0 && !pause_ms(50))
        return false;
    if (!running)
        return true;
    // Quiesce BEFORE reset, banked configuration or FIFO/clock changes.
    if (!diagnostic_write(spi, log, test.name, 0x4e, previous_power & 0xf0)
        || !pause_ms(kGyroOffDelayMs))
        return false;
    if (!running)
        return true;
    if (!diagnostic_read(spi, 0x4e, &off_power, 1))
        return false;
    log.line("imu_diag case=%s op=quiesce previous=0x%02x readback=0x%02x verified=%u off_wait_ms=%ld\n",
             test.name, static_cast<unsigned>(previous_power), static_cast<unsigned>(off_power),
             static_cast<unsigned>((off_power & 0x0f) == 0), kGyroOffDelayMs);
    if ((off_power & 0x0f) != 0)
        return false;
    if (test.reset) {
        // The explicit OFF above also preserves the minimum gyro ON duration.
        // DS-000440 p68: after reset allow >=1 ms without register access.
        if (!diagnostic_write(spi, log, test.name, 0x11, 0x01) || !pause_ms(2))
            return false;
        if (!running)
            return true;
    }
    if (!diagnostic_bank0(spi, log, test.name, "before"))
        return false;

    if (test.int1_push_pull) {
        // Match the vendor initialization: PP + active high on INT1.
        // Vendor comment: avoids entering Test Mode after a soft reset.
        std::uint8_t before = 0, applied = 0;
        if (!diagnostic_read(spi, 0x14, &before, 1)
            || !diagnostic_write(spi, log, test.name, 0x14, before | 0x03)
            || !diagnostic_read(spi, 0x14, &applied, 1))
            return false;
        const bool verified = applied == (before | 0x03);
        log.line("imu_diag case=%s op=int1 int_config_before=0x%02x "
                 "int_config=0x%02x push_pull=%u active_high=%u verified=%u\n", test.name,
                 static_cast<unsigned>(before), static_cast<unsigned>(applied),
                 static_cast<unsigned>((applied >> 1) & 1),
                 static_cast<unsigned>(applied & 1), static_cast<unsigned>(verified));
        if (!verified)
            return false;
    }
    if (test.vendor_filters) {
        // Vendor polling setup, preserving accel configuration and reserved bits.
        // GYRO UI: order 2, BW ODR/4; FSYNC tagging off; RTC timestamp resolution.
        constexpr std::array<std::array<std::uint8_t, 3>, 5> updates{{
            {0x51, 0x0c, 0x04}, {0x52, 0x0f, 0x01}, {0x62, 0x70, 0x00},
            {0x54, 0x0a, 0x08}, {0x64, 0x10, 0x00}}};
        for (const auto& update : updates) {
            std::uint8_t before = 0, applied = 0;
            if (!diagnostic_read(spi, update[0], &before, 1))
                return false;
            const std::uint8_t expected = (before & ~update[1]) | update[2];
            if (!diagnostic_write(spi, log, test.name, update[0], expected)
                || !diagnostic_read(spi, update[0], &applied, 1))
                return false;
            log.line("imu_diag case=%s op=vendor_config reg=0x%02x before=0x%02x "
                     "applied=0x%02x verified=%u\n", test.name,
                     static_cast<unsigned>(update[0]), static_cast<unsigned>(before),
                     static_cast<unsigned>(applied), static_cast<unsigned>(applied == expected));
            if (applied != expected)
                return false;
        }
    }
    std::uint8_t fifo = 0;
    if (!diagnostic_read(spi, 0x16, &fifo, 1)
        || !diagnostic_write(spi, log, test.name, 0x16, fifo & 0x3f))
        return false;
    // Both sensors are OFF for all banked configuration. Preserve reserved bits.
    if (!diagnostic_write(spi, log, test.name, 0x76, 0x01))
        return false;
    std::uint8_t axes = 0, filters = 0, interface4 = 0;
    const bool read_ok = diagnostic_read(spi, 0x03, &axes, 1)
        && diagnostic_read(spi, 0x0b, &filters, 1)
        && diagnostic_read(spi, 0x7a, &interface4, 1);
    log.line("imu_diag case=%s op=bank1 read_ok=%u sensor_config0=0x%02x "
             "gyro_disable=0x%02x accel_disable=0x%02x gyro_static2=0x%02x intf4=0x%02x\n",
             test.name, static_cast<unsigned>(read_ok), static_cast<unsigned>(axes),
             static_cast<unsigned>((axes >> 3) & 7), static_cast<unsigned>(axes & 7),
             static_cast<unsigned>(filters), static_cast<unsigned>(interface4));
    bool extra_ok = true;
    if (read_ok && test.inspect_spi4) {
        // DS-000440 p67/p97: SPI_AP_4WIRE is already enabled after reset.
        // Never write INTF_CONFIG4: literal 0x02 also cleared reserved bits 7/0.
        std::uint8_t applied = 0;
        std::array<std::uint8_t, 3> factory{};
        extra_ok = diagnostic_read(spi, 0x5f, factory.data(), factory.size())
            && diagnostic_read(spi, 0x7a, &applied, 1);
        extra_ok = extra_ok && applied == interface4 && (applied & 0x02) != 0;
        log.line("imu_diag case=%s op=spi4_readonly before=0x%02x applied=0x%02x "
                 "register_writes=0 factory_gyro=%u,%u,%u verified=%u\n", test.name,
                 static_cast<unsigned>(interface4), static_cast<unsigned>(applied),
                 static_cast<unsigned>(factory[0]), static_cast<unsigned>(factory[1]),
                 static_cast<unsigned>(factory[2]), static_cast<unsigned>(extra_ok));
    }
    bool clock_ok = true;
    if (read_ok && extra_ok && test.external_clock) {
        // DS-000440 p98: Bank 1 INTF_CONFIG5[2:1]=10 selects Pin 9 CLKIN.
        std::uint8_t before = 0, applied = 0;
        clock_ok = diagnostic_read(spi, 0x7b, &before, 1)
            && diagnostic_write(spi, log, test.name, 0x7b, (before & 0xf9) | 0x04)
            && diagnostic_read(spi, 0x7b, &applied, 1);
        clock_ok = clock_ok && applied == ((before & 0xf9) | 0x04);
        log.line("imu_diag case=%s op=clkin pin=9 intf5_before=0x%02x "
                 "intf5=0x%02x pin9_function=%u verified=%u\n", test.name,
                 static_cast<unsigned>(before), static_cast<unsigned>(applied),
                 static_cast<unsigned>((applied >> 1) & 3), static_cast<unsigned>(clock_ok));
    }
    bool axes_ok = true;
    if (read_ok && test.reset && (axes & 0x3f) != 0)
        axes_ok = diagnostic_write(spi, log, test.name, 0x03, axes & 0xc0);
    const bool restored = diagnostic_write(spi, log, test.name, 0x76, 0x00);
    if (!read_ok || !extra_ok || !clock_ok || !axes_ok || !restored)
        return false;
    if (test.reset) {
        if (!diagnostic_write(spi, log, test.name, 0x4f, 0x06)
            || !diagnostic_write(spi, log, test.name, 0x50, 0x06))
            return false;
    }
    if (test.external_clock) {
        // DS-000440 p79: RTC_MODE=1; compare CLKSEL=01 and 00 after reset.
        // Preserve ACCEL_LP_CLK_SEL and reserved bits. CLKIN needs a physical RTC signal.
        const std::uint8_t clock_bits = test.rc_clock ? 0x04 : 0x05;
        std::uint8_t before = 0, applied = 0;
        if (!diagnostic_read(spi, 0x4d, &before, 1)
            || !diagnostic_write(spi, log, test.name, 0x4d, (before & 0xf8) | clock_bits)
            || !diagnostic_read(spi, 0x4d, &applied, 1))
            return false;
        const bool verified = applied == ((before & 0xf8) | clock_bits);
        log.line("imu_diag case=%s op=external_clock intf1_before=0x%02x "
                 "intf1=0x%02x rtc=%u clksel=%u verified=%u\n", test.name,
                 static_cast<unsigned>(before), static_cast<unsigned>(applied),
                 static_cast<unsigned>((applied >> 2) & 1),
                 static_cast<unsigned>(applied & 3), static_cast<unsigned>(verified));
        if (!verified)
            return false;
    } else if (test.rc_clock) {
        std::uint8_t clock = 0;
        if (!diagnostic_read(spi, 0x4d, &clock, 1)
            || !diagnostic_write(spi, log, test.name, 0x4d, clock & 0xf8))
            return false;
    }
    if (test.selftest_settings) {
        // Vendor gyro self-test baseline: +/-250 dps, 1kHz, order 3, BW ODR/10.
        // No accel settings or offset/trim registers are changed.
        constexpr std::array<std::array<std::uint8_t, 3>, 3> updates{{
            {0x4f, 0xe0, 0x60}, {0x51, 0x0c, 0x08}, {0x52, 0x0f, 0x04}}};
        for (const auto& update : updates) {
            std::uint8_t before = 0, applied = 0;
            if (!diagnostic_read(spi, update[0], &before, 1))
                return false;
            const std::uint8_t expected = (before & ~update[1]) | update[2];
            if (!diagnostic_write(spi, log, test.name, update[0], expected)
                || !diagnostic_read(spi, update[0], &applied, 1) || applied != expected)
                return false;
        }
    }
    if (!disable_power_save(spi, log, test.name))
        return false;
    constexpr std::uint8_t power = 0x0f;
    if (!diagnostic_write(spi, log, test.name, 0x4e, power)
        || !pause_ms(50))
        return false;
    return !running || diagnostic_bank0(spi, log, test.name, "enabled", power);
}

bool diagnostic_samples(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                        unsigned& gyro_valid) noexcept {
    if (!verify_bank(spi, log, name, 0))
        return false;
    unsigned samples = 0, errors = 0, accel_valid = 0, status_seen = 0;
    bool first_gyro = true;
    for (unsigned i = 0; i < 300 && running; ++i) {
        std::array<std::uint8_t, 14> raw{};
        std::uint8_t status = 0;
        const bool status_ok = diagnostic_read(spi, 0x2d, &status, 1);
        status_seen |= status;
        const bool data_ok = diagnostic_read(spi, 0x1d, raw.data(), raw.size());
        errors += !status_ok;
        errors += !data_ok;
        if (data_ok) {
            std::array<std::int16_t, 7> values{};
            bool all_zero = true, all_ff = true;
            for (const auto byte : raw) {
                all_zero &= byte == 0;
                all_ff &= byte == 0xff;
            }
            for (unsigned axis = 0; axis < values.size(); ++axis)
                values[axis] = diagnostic_word(raw.data() + 2 * axis);
            const bool accel_ok = !all_zero && !all_ff && values[1] != INT16_MIN
                && values[2] != INT16_MIN && values[3] != INT16_MIN;
            const bool gyro_ok = !all_zero && !all_ff && values[4] != INT16_MIN
                && values[5] != INT16_MIN && values[6] != INT16_MIN;
            ++samples;
            accel_valid += accel_ok;
            gyro_valid += gyro_ok;
            if (i == 0 || (i + 1) % 100 == 0 || (first_gyro && gyro_ok)) {
                log.line("imu_diag case=%s op=sample poll=%u status=0x%02x temp=%d "
                         "accel=%d,%d,%d gyro=%d,%d,%d accel_valid=%u gyro_valid=%u\n",
                         name, i + 1, static_cast<unsigned>(status), values[0],
                         values[1], values[2], values[3], values[4], values[5], values[6],
                         static_cast<unsigned>(accel_ok), static_cast<unsigned>(gyro_ok));
            }
            if (gyro_ok)
                first_gyro = false;
        }
        if (!pause_ms(10))
            return false;
    }
    // Compare individual-byte gyro reads against burst results. These are
    // separate snapshots, so moving sensors need not give identical numbers.
    std::array<std::uint8_t, 6> individual{};
    bool individual_ok = true;
    if (running) {
        for (unsigned i = 0; i < individual.size(); ++i)
            individual_ok &= diagnostic_read(spi, 0x25 + i, &individual[i], 1);
        log.line("imu_diag case=%s op=individual read_ok=%u gyro=%d,%d,%d\n", name,
                 static_cast<unsigned>(individual_ok), diagnostic_word(individual.data()),
                 diagnostic_word(individual.data() + 2), diagnostic_word(individual.data() + 4));
    }
    log.line("imu_diag case=%s op=summary samples=%u errors=%u accel_valid=%u "
             "gyro_valid=%u status_seen=0x%02x individual_ok=%u\n", name, samples,
             errors, accel_valid, gyro_valid, status_seen, static_cast<unsigned>(individual_ok));
    if (errors != 0 || !individual_ok)
        gyro_valid = 0;
    return true;
}

bool diagnose_sensor(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    constexpr std::array<DiagnosticCase, 3> cases{{
        {"as_is", false, false},
        {"reset_ln", true, false},
        {"rc_clock", true, true}}};
    log.line("imu_diag op=start version=7 cases=3 polls_per_case=300 period_ms=10 "
             "access=polling low_power=disabled standby=disabled\n");
    unsigned successful = 0;
    std::size_t selected = 1; // Keep six-axis LN even if gyro data never become valid.
    for (std::size_t i = 0; i < cases.size() && running; ++i) {
        if (!diagnostic_setup(spi, log, cases[i])) {
            log.line("imu_diag case=%s op=setup result=error\n", cases[i].name);
            return false;
        }
        unsigned gyro_valid = 0;
        if (!diagnostic_samples(spi, log, cases[i].name, gyro_valid))
            return false;
        if (gyro_valid != 0) {
            if (successful == 0)
                selected = i;
            ++successful;
        }
    }
    if (!running)
        return true;
    log.line("imu_diag op=complete gyro_valid_cases=%u selected=%s action=resume_raw_polling\n",
             successful, cases[selected].name);
    return diagnostic_setup(spi, log, cases[selected]);
}

bool external_clock_sensor(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    // Separate OFF-time correction from the vendor's INT1 and filter setup.
    // All cases keep the existing +/-16g/1kHz accel settings and external CLKIN.
    constexpr std::array<DiagnosticCase, 3> cases{{
        {"off200_clkin", true, false, true},
        {"off200_int1_clkin", true, false, true, true},
        {"vendor_poll_clkin", true, false, true, true, true}}};
    log.line("imu_diag op=start version=7 cases=3 polls_per_case=300 period_ms=10 "
             "access=polling clock=clkin\n");
    unsigned successful = 0;
    std::size_t selected = 2; // Keep the vendor polling setup if all cases fail.
    for (std::size_t i = 0; i < cases.size() && running; ++i) {
        if (!diagnostic_setup(spi, log, cases[i])) {
            log.line("imu_diag case=%s op=setup result=error\n", cases[i].name);
            return false;
        }
        unsigned gyro_valid = 0;
        if (!diagnostic_samples(spi, log, cases[i].name, gyro_valid))
            return false;
        if (gyro_valid != 0) {
            if (successful == 0)
                selected = i;
            ++successful;
        }
    }
    if (!running)
        return true;
    log.line("imu_diag op=complete gyro_valid_cases=%u selected=%s "
             "action=resume_raw_polling\n", successful, cases[selected].name);
    return diagnostic_setup(spi, log, cases[selected]);
}

// Self-test cleanup must finish even if a stop signal interrupts the diagnostic.
bool cleanup_pause_ms(long milliseconds) noexcept {
    timespec remaining{milliseconds / 1000, (milliseconds % 1000) * 1'000'000};
    while (nanosleep(&remaining, &remaining) != 0)
        if (errno != EINTR)
            return false;
    return true;
}

bool gyro_stimulus(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    constexpr const char* name = "gyro_stimulus";
    std::uint8_t original = 0, applied = 0;
    if (!diagnostic_read(spi, 0x70, &original, 1))
        return false;
    // The manufacturer self-test routine explicitly changes SELF_TEST_CONFIG
    // while the gyro is running, then waits 200 ms before reading sense data.
    bool ok = diagnostic_write(spi, log, name, 0x70, original | 0x07)
        && diagnostic_read(spi, 0x70, &applied, 1) && applied == (original | 0x07);
    if (ok)
        ok = cleanup_pause_ms(200);
    unsigned valid = 0;
    if (ok && running)
        ok = diagnostic_samples(spi, log, name, valid);
    // Always attempt all cleanup operations, including after short writes,
    // readback mismatch or SIGTERM. Observe startup and OFF ring-down waits.
    bool cleaned = cleanup_pause_ms(50);
    if (!diagnostic_write(spi, log, name, 0x76, 0x00)) {
        // Do not address Bank 0 registers if its selection cannot be verified.
        log.line("imu_diag case=%s op=stimulus_result gyro_valid=%u cleanup_ok=0 "
                 "self_test_config=0xff grading=availability_only\n", name, valid);
        return false;
    }
    cleaned = diagnostic_write(spi, log, name, 0x4e, 0x00) && cleaned;
    cleaned = cleanup_pause_ms(kGyroOffDelayMs) && cleaned;
    cleaned = diagnostic_write(spi, log, name, 0x70, original & 0xf8) && cleaned;
    std::uint8_t readback = 0xff;
    cleaned = diagnostic_read(spi, 0x70, &readback, 1) && cleaned;
    cleaned = cleaned && (readback & 0x07) == 0;
    log.line("imu_diag case=%s op=stimulus_result gyro_valid=%u cleanup_ok=%u "
             "self_test_config=0x%02x grading=availability_only\n", name, valid,
             static_cast<unsigned>(cleaned), static_cast<unsigned>(readback));
    return ok && cleaned;
}

bool vendor_selftest_sensor(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    constexpr std::array<DiagnosticCase, 2> cases{{
        {"vendor_spi4_clkin", true, false, true, true, true, true},
        {"gyro_st_baseline", true, false, true, true, true, true, true}}};
    log.line("imu_diag op=start version=7 cases=2 stimulus_cases=1 polls_per_case=300 "
             "access=polling clock=clkin off_wait_ms=200\n");
    unsigned successful = 0;
    std::size_t selected = 0;
    for (std::size_t i = 0; i < cases.size() && running; ++i) {
        if (!diagnostic_setup(spi, log, cases[i])) {
            log.line("imu_diag case=%s op=setup result=error\n", cases[i].name);
            return false;
        }
        if (!cleanup_pause_ms(10)) // vendor startup settling: at least 60 ms total
            return false;
        unsigned valid = 0;
        if (!diagnostic_samples(spi, log, cases[i].name, valid))
            return false;
        if (valid != 0) {
            if (successful == 0)
                selected = i;
            ++successful;
        }
    }
    if (!running)
        return true;
    if (!gyro_stimulus(spi, log))
        return false;
    if (!running)
        return true;
    log.line("imu_diag op=complete gyro_valid_cases=%u selected=%s "
             "action=resume_raw_polling\n", successful, cases[selected].name);
    return diagnostic_setup(spi, log, cases[selected]);
}

// ---------------------------------------------------------------------------
// Gyro subsystem probe (--gyro-core)
//
// The v1..v7 rounds already covered start-up timing, INT1, CLKIN/RTC, the
// vendor filter set, Bank readback and self-test stimulation. Everything they
// read back matched the written value. This round covers what was still open:
//
//  1. Register integrity straight after a soft reset. Every documented field
//     must read its DS-000440 default, which both proves the part answers as
//     an IIM-42652 and rules out a stuck user offset (Bank 4 OFFSET_USER*)
//     silently shifting the gyro output.
//  2. The internal 20-bit time-stamp counter, latched with TMST_STROBE
//     (SIGNAL_PATH_RESET bit 2). Its rate is the only in-chip proof that the
//     sample/timebase domain runs, and with TMST_RES=1 the tick is exactly one
//     RTC period, so it also measures whether the external CLKIN is alive.
//  3. ABORT_AND_RESET (SIGNAL_PATH_RESET bit 3): restarts the ODR counter and
//     the signal path. Never issued by any earlier round.
//  4. Gyro anti-alias + notch filter disabled (Bank 1 0x0B bits 1:0): the one
//     remaining block in the gyro path that can suppress its output without
//     touching the sense registers.
//  5. Gyro-only LN. Temperature and gyro both belong to that sample domain, so
//     if the gyro domain never produced a first sample both stay -32768.
// ---------------------------------------------------------------------------

bool raw_write(drv_spi_inst_t spi, std::uint8_t reg, std::uint8_t value) noexcept {
    const std::array<std::uint8_t, 2> tx{reg, value};
    return drv_spi_write(spi, tx.data(), tx.size(), true) == static_cast<int>(tx.size());
}

bool raw_bank(drv_spi_inst_t spi, std::uint8_t bank) noexcept {
    if (!raw_write(spi, 0x76, bank))
        return false;
    std::uint8_t applied = 0xff;
    return diagnostic_read(spi, 0x76, &applied, 1) && applied == bank;
}

// 0xff in `expected` means "no documented default to compare against".
struct ResetProbe {
    std::uint8_t bank;
    std::uint8_t reg;
    std::uint8_t expected;
};

void dump_reset_registers(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    // DS-000440 sections 14/15/16/18. Only fields with an unambiguous reset
    // value are compared; everything else is reported for the record.
    static constexpr ResetProbe probes[]{
        {0, 0x11, 0x00}, {0, 0x13, 0x05}, {0, 0x14, 0x00}, {0, 0x16, 0x00},
        {0, 0x4c, 0x30}, {0, 0x4d, 0x91}, {0, 0x4e, 0x00}, {0, 0x4f, 0x06},
        {0, 0x50, 0x06}, {0, 0x51, 0x16}, {0, 0x52, 0x11}, {0, 0x53, 0xff},
        {0, 0x54, 0x23}, {0, 0x56, 0xff}, {0, 0x57, 0x00}, {0, 0x5f, 0xff},
        {0, 0x60, 0xff}, {0, 0x61, 0xff}, {0, 0x62, 0xff}, {0, 0x63, 0xff},
        {0, 0x64, 0xff}, {0, 0x70, 0x00}, {0, 0x75, 0x6f}, {0, 0x76, 0x00},
        // Bank 1: axis enables, gyro AAF/notch, factory gyro self-test data,
        // serial-interface setup.
        {1, 0x03, 0x80}, {1, 0x0b, 0xa0}, {1, 0x0c, 0x0d}, {1, 0x0d, 0xff},
        {1, 0x0e, 0xff}, {1, 0x0f, 0xff}, {1, 0x10, 0xff}, {1, 0x11, 0xff},
        {1, 0x12, 0xff}, {1, 0x13, 0xff}, {1, 0x5f, 0xff}, {1, 0x60, 0xff},
        {1, 0x61, 0xff}, {1, 0x7a, 0x03}, {1, 0x7b, 0x20}, {1, 0x7c, 0x5f},
        // Bank 2: accel AAF and factory accel self-test data.
        {2, 0x03, 0xff}, {2, 0x04, 0xff}, {2, 0x05, 0xff}, {2, 0x3b, 0xff},
        {2, 0x3c, 0xff}, {2, 0x3d, 0xff},
        // Bank 4: user offsets. Non-zero gyro offsets would shift the output.
        {4, 0x09, 0xff}, {4, 0x40, 0xff}, {4, 0x77, 0x00}, {4, 0x78, 0x00},
        {4, 0x79, 0x00}, {4, 0x7a, 0x00}, {4, 0x7b, 0x00}, {4, 0x7c, 0x00},
        {4, 0x7d, 0x00}, {4, 0x7e, 0x00}, {4, 0x7f, 0x00},
    };
    char text[512];
    std::size_t used = 0;
    unsigned checked = 0, deviations = 0, read_errors = 0;
    int bank = -1;
    for (const auto& probe : probes) {
        if (bank != static_cast<int>(probe.bank)) {
            if (bank >= 0 && used != 0) {
                text[used] = '\0';
                log.line("imu_core op=dump bank=%d regs=%s\n", bank, text);
                used = 0;
            }
            if (!raw_bank(spi, probe.bank))
                ++read_errors;
            bank = probe.bank;
        }
        std::uint8_t value = 0;
        const bool ok = diagnostic_read(spi, probe.reg, &value, 1);
        if (!ok)
            ++read_errors;
        bool compared = false;
        if (probe.expected != 0xff) {
            ++checked;
            compared = ok && value == probe.expected;
            if (!compared)
                ++deviations;
        }
        if (used < sizeof(text) - 16) {
            used += static_cast<std::size_t>(std::snprintf(
                text + used, sizeof(text) - used, "%s%02x=%02x%s", used == 0 ? "" : ",",
                static_cast<unsigned>(probe.reg), static_cast<unsigned>(value),
                (!compared && probe.expected != 0xff) ? "!" : ""));
        }
    }
    if (used != 0) {
        text[used] = '\0';
        log.line("imu_core op=dump bank=%d regs=%s\n", bank, text);
    }
    (void)raw_bank(spi, 0);
    log.line("imu_core op=dump_result compared=%u deviations=%u read_errors=%u "
             "note=0xff_entries_are_reported_only\n",
             checked, deviations, read_errors);
}

// Latch the 20-bit internal time stamp into Bank 1 TMSTVAL0..2 and read it.
bool read_tmst_value(drv_spi_inst_t spi, std::uint32_t& value) noexcept {
    value = 0;
    if (!raw_write(spi, 0x4b, 0x04)) // SIGNAL_PATH_RESET.TMST_STROBE
        return false;
    if (!raw_bank(spi, 1))
        return false;
    std::array<std::uint8_t, 3> raw{};
    const bool ok = diagnostic_read(spi, 0x62, raw.data(), raw.size());
    if (!raw_bank(spi, 0))
        return false;
    if (!ok)
        return false;
    value = static_cast<std::uint32_t>(raw[0]) | (static_cast<std::uint32_t>(raw[1]) << 8)
        | (static_cast<std::uint32_t>(raw[2] & 0x0f) << 16);
    return true;
}

// DS-000440 p63: only ODR/FSR/mode may change while a sensor runs, so the
// time-stamp resolution is configured in a quiesced window.
bool configure_tmst_res(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                        bool rtc_period) noexcept {
    std::uint8_t before = 0, applied = 0;
    if (!diagnostic_read(spi, 0x54, &before, 1))
        return false;
    const std::uint8_t expected =
        static_cast<std::uint8_t>((before & 0xf0) | 0x01 | (rtc_period ? 0x08 : 0x00));
    const bool ok = raw_write(spi, 0x54, expected)
        && diagnostic_read(spi, 0x54, &applied, 1) && applied == expected;
    log.line("imu_core case=%s op=tmst_config res=%s tmst_cfg=0x%02x verified=%u\n", name,
             rtc_period ? "rtc_period" : "us", static_cast<unsigned>(applied),
             static_cast<unsigned>(ok));
    return ok;
}

// TMST_RES=1 makes one tick one RTC period: 400 ms must show ~13107 ticks at
// 32.768 kHz, which measures the external CLKIN from inside the device.
// TMST_RES=0 ticks at 1 us and must show ~400000 over the same window.
// The expected value follows the resolution actually present in TMST_CONFIG.
void strobe_timebase(drv_spi_inst_t spi, logging::Logger& log, const char* name) noexcept {
    constexpr std::uint32_t kMask = 0x000fffff, kWindowMs = 400;
    std::uint8_t tmst = 0;
    if (!diagnostic_read(spi, 0x54, &tmst, 1)) {
        log.line("imu_core case=%s op=timebase result=config_read_error\n", name);
        return;
    }
    const bool rtc_period = (tmst & 0x08) != 0;
    const std::uint32_t expected = rtc_period ? 13107u : 400000u;
    std::uint32_t first = 0, second = 0;
    bool ok = read_tmst_value(spi, first);
    ok = ok && pause_ms(static_cast<long>(kWindowMs)) && running;
    ok = ok && read_tmst_value(spi, second);
    if (!ok) {
        log.line("imu_core case=%s op=timebase res=%s result=read_error\n", name,
                 rtc_period ? "rtc_period" : "us");
        return;
    }
    const std::uint32_t delta = (second - first) & kMask;
    log.line("imu_core case=%s op=timebase res=%s tmst_cfg=0x%02x first=%u second=%u "
             "window_ms=%u delta=%u expected=%u ratio_permille=%u\n", name,
             rtc_period ? "rtc_period" : "us", static_cast<unsigned>(tmst), first, second,
             static_cast<unsigned>(kWindowMs), delta, expected, (delta * 1000u) / expected);
}

bool cycle_power_off(drv_spi_inst_t spi, logging::Logger& log, const char* name) noexcept {
    std::uint8_t before = 0, after = 0xff;
    if (!diagnostic_read(spi, 0x4e, &before, 1) || !raw_write(spi, 0x4e, before & 0xf0))
        return false;
    if (!pause_ms(kGyroOffDelayMs))
        return false;
    if (!diagnostic_read(spi, 0x4e, &after, 1))
        return false;
    const bool verified = (after & 0x0f) == 0;
    log.line("imu_core case=%s op=quiesce before=0x%02x after=0x%02x verified=%u "
             "off_wait_ms=%ld\n", name, static_cast<unsigned>(before),
             static_cast<unsigned>(after), static_cast<unsigned>(verified), kGyroOffDelayMs);
    return verified;
}

bool enable_power(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                  std::uint8_t power) noexcept {
    if (!raw_write(spi, 0x4e, power) || !pause_ms(50))
        return false;
    std::uint8_t applied = 0xff;
    if (!diagnostic_read(spi, 0x4e, &applied, 1))
        return false;
    const bool verified = applied == power;
    log.line("imu_core case=%s op=enable power=0x%02x readback=0x%02x gyro_mode=%u "
             "accel_mode=%u verified=%u\n", name, static_cast<unsigned>(power),
             static_cast<unsigned>(applied), static_cast<unsigned>((applied >> 2) & 3),
             static_cast<unsigned>(applied & 3), static_cast<unsigned>(verified));
    return verified;
}

bool gyro_core_sensor(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    const DiagnosticCase base{"v7_clkin", true, false, true, true, true, true, false};
    log.line("imu_core op=start version=8 phases=6 polls_per_phase=300 period_ms=10 "
             "access=polling clock=clkin\n");
    if (!running)
        return true;

    // 1. Soft reset with both sensors confirmed OFF, then dump the documented
    //    defaults before anything is configured.
    if (!cycle_power_off(spi, log, "reset"))
        return false;
    if (!raw_write(spi, 0x11, 0x01) || !pause_ms(2))
        return false;
    if (!running)
        return true;
    dump_reset_registers(spi, log);

    // 2. The v7 reference configuration, then the 1 us time base and the
    //    baseline sample set.
    if (!diagnostic_setup(spi, log, base)) {
        log.line("imu_core op=setup result=error\n");
        return false;
    }
    if (!running)
        return true;
    strobe_timebase(spi, log, "baseline_6axis");
    unsigned valid = 0;
    if (!diagnostic_samples(spi, log, "baseline_6axis", valid))
        return false;
    if (!running)
        return true;

    // 3. ABORT_AND_RESET: restart the ODR counter and signal path.
    if (!raw_write(spi, 0x4b, 0x08) || !pause_ms(20))
        return false;
    log.line("imu_core case=abort_reset op=signal_path_reset reg=0x4b value=0x08 "
             "wait_ms=20\n");
    if (!diagnostic_samples(spi, log, "abort_reset", valid))
        return false;
    if (!running)
        return true;

    // 4. Gyro anti-alias and notch filters disabled (Bank 1 0x0B bits 1:0).
    if (!cycle_power_off(spi, log, "filters_off"))
        return false;
    std::uint8_t filters = 0, filters_applied = 0;
    if (!raw_bank(spi, 1) || !diagnostic_read(spi, 0x0b, &filters, 1)) {
        (void)raw_bank(spi, 0);
        return false;
    }
    const std::uint8_t filters_expected = static_cast<std::uint8_t>(filters | 0x03);
    const bool filters_ok = raw_write(spi, 0x0b, filters_expected)
        && diagnostic_read(spi, 0x0b, &filters_applied, 1)
        && filters_applied == filters_expected && raw_bank(spi, 0);
    log.line("imu_core case=filters_off op=gyro_filters before=0x%02x applied=0x%02x "
             "aaf_notch_disabled=1 verified=%u\n", static_cast<unsigned>(filters),
             static_cast<unsigned>(filters_applied), static_cast<unsigned>(filters_ok));
    if (!filters_ok || !raw_write(spi, 0x4f, 0x06) || !raw_write(spi, 0x50, 0x06)
        || !enable_power(spi, log, "filters_off", 0x0f))
        return false;
    if (!diagnostic_samples(spi, log, "filters_off", valid))
        return false;
    if (!running)
        return true;

    // 5. RTC-period time base: one tick is exactly one CLKIN period, so this
    //    measures the external 32.768 kHz input from inside the device.
    if (!cycle_power_off(spi, log, "rtc_resolution")
        || !configure_tmst_res(spi, log, "rtc_resolution", true)
        || !enable_power(spi, log, "rtc_resolution", 0x0f))
        return false;
    strobe_timebase(spi, log, "rtc_resolution");
    if (!diagnostic_samples(spi, log, "rtc_resolution", valid))
        return false;
    if (!running)
        return true;

    // 6. Gyro-only LN: temperature and gyro share that sample domain, so both
    //    staying -32768 confirms the domain produced no first sample at all.
    if (!cycle_power_off(spi, log, "gyro_only")
        || !configure_tmst_res(spi, log, "gyro_only", false)
        || !enable_power(spi, log, "gyro_only", 0x0c))
        return false;
    strobe_timebase(spi, log, "gyro_only");
    if (!diagnostic_samples(spi, log, "gyro_only", valid))
        return false;
    if (!running)
        return true;

    log.line("imu_core op=complete action=restore_v7_reference\n");
    return diagnostic_setup(spi, log, base);
}

// Observe application-entry state before resetting/enabling any sensor.
// These observations start after the firmware/launcher; they are not POR proof.
bool startup_audit_sensor(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    std::uint8_t entry_bank = 0xff;
    const auto first_read_ms = platform::mono_ms();
    if (!diagnostic_read(spi, 0x76, &entry_bank, 1) || entry_bank > 4 || entry_bank == 3) {
        log.line("imu_audit op=entry result=bank_read_error bank=0x%02x "
                 "sensor_config_writes=0\n", static_cast<unsigned>(entry_bank));
        return false;
    }
    // Selecting a bank is the only write allowed during entry observation.
    if (entry_bank != 0 && !diagnostic_write(spi, log, "entry", 0x76, 0))
        return false;
    std::array<std::uint8_t, 5> config{};
    std::uint8_t id = 0;
    if (!diagnostic_read(spi, 0x75, &id, 1)
        || !diagnostic_read(spi, 0x4c, config.data(), config.size()))
        return false;
    log.line("imu_audit op=entry first_read_ms=%lld entry_bank=%u id=0x%02x "
             "intf0=0x%02x intf1=0x%02x power=0x%02x gyro_cfg=0x%02x accel_cfg=0x%02x "
             "sensor_config_writes=0 scope=application_entry_not_por\n",
             static_cast<long long>(first_read_ms), static_cast<unsigned>(entry_bank),
             static_cast<unsigned>(id), static_cast<unsigned>(config[0]),
             static_cast<unsigned>(config[1]), static_cast<unsigned>(config[2]),
             static_cast<unsigned>(config[3]), static_cast<unsigned>(config[4]));
    if (id != 0x6f || (config[0] & 0xf0) != 0x30)
        return false;
    // Read-only zero-write sample snapshot. DS-000440 p62: from power-on reset
    // the part is asleep and the sense registers read -32768 until the first
    // sample, so a plausible accel sample here proves the device was already
    // sampling before this process opened the bus. INT_STATUS is deliberately
    // not read because it is cleared by the read.
    std::array<std::uint8_t, 14> entry_raw{};
    const bool entry_raw_ok = diagnostic_read(spi, 0x1d, entry_raw.data(), entry_raw.size());
    std::array<std::int16_t, 7> entry_values{};
    if (entry_raw_ok) {
        for (unsigned i = 0; i < entry_values.size(); ++i)
            entry_values[i] = diagnostic_word(entry_raw.data() + 2 * i);
    }
    const bool accel_no_data = !entry_raw_ok || entry_values[1] == INT16_MIN
        || entry_values[2] == INT16_MIN || entry_values[3] == INT16_MIN;
    const bool gyro_no_data = !entry_raw_ok || entry_values[4] == INT16_MIN
        || entry_values[5] == INT16_MIN || entry_values[6] == INT16_MIN;
    log.line("imu_audit op=entry_sample read_ok=%u temp=%d accel=%d,%d,%d gyro=%d,%d,%d "
             "accel_no_data=%u gyro_no_data=%u power=0x%02x sensor_config_writes=0 "
             "note=read_only_bank0\n",
             static_cast<unsigned>(entry_raw_ok), entry_values[0], entry_values[1],
             entry_values[2], entry_values[3], entry_values[4], entry_values[5],
             entry_values[6], static_cast<unsigned>(accel_no_data),
             static_cast<unsigned>(gyro_no_data), static_cast<unsigned>(config[2]));
    const std::uint8_t first_power = config[2];
    std::uint8_t previous_power = first_power;
    unsigned polls = 0, changes = 0;
    const auto observe_start_ms = platform::mono_ms();
    for (unsigned i = 0; i < 100 && running; ++i) {
        if (!pause_ms(10) || !running)
            break;
        std::uint8_t actual_bank = 0xff, power = 0xff;
        if (!diagnostic_read(spi, 0x76, &actual_bank, 1) || actual_bank != 0
            || !diagnostic_read(spi, 0x4e, &power, 1)) {
            log.line("imu_audit op=observe result=read_or_bank_error "
                     "sensor_config_writes=0\n");
            return false;
        }
        ++polls;
        if (power != previous_power) {
            ++changes;
            log.line("imu_audit op=power_change poll=%u before=0x%02x after=0x%02x "
                     "sensor_config_writes=0\n", polls, static_cast<unsigned>(previous_power),
                     static_cast<unsigned>(power));
            previous_power = power;
        }
    }
    log.line("imu_audit op=observe polls=%u elapsed_ms=%lld initial_power=0x%02x "
             "final_power=0x%02x changes=%u sensor_config_writes=0\n", polls,
             static_cast<long long>(platform::mono_ms() - observe_start_ms),
             static_cast<unsigned>(first_power), static_cast<unsigned>(previous_power), changes);
    if (changes != 0 || polls != 100) {
        // Another writer or an incomplete observation invalidates the baseline.
        return !running;
    }
    constexpr std::array<DiagnosticCase, 3> cases{{
        {"preserve_clkin", true, false, true, true, true, true},
        {"preserve_internal_auto", true, false, false, true, true, true},
        {"preserve_internal_rc", true, true, false, true, true, true}}};
    unsigned successful = 0;
    std::size_t selected = 0;
    for (std::size_t i = 0; i < cases.size() && running; ++i) {
        if (!diagnostic_setup(spi, log, cases[i])) {
            log.line("imu_audit case=%s op=setup result=error\n", cases[i].name);
            return false;
        }
        unsigned valid = 0;
        if (!diagnostic_samples(spi, log, cases[i].name, valid))
            return false;
        if (valid != 0) {
            if (successful == 0)
                selected = i;
            ++successful;
        }
    }
    if (!running)
        return true;
    log.line("imu_audit op=complete gyro_valid_cases=%u selected=%s "
             "intf4_writes=0 action=resume_raw_polling\n", successful, cases[selected].name);
    return diagnostic_setup(spi, log, cases[selected]);
}

// ---------------------------------------------------------------------------
// v10 metrology (--metrology)
//
// No bench instrument is available, so every remaining question is answered by
// counters the device exposes itself:
//
//  1. ODR metrology from FIFO_COUNT. DS-000440 p60: every ODR scales with the
//     external clock, so a 1 kHz setting with the on-board 32.768 kHz
//     oscillator must measure 1024 Hz. That measures the CLKIN from inside the
//     part. Comparing CLKSEL=01 with CLKSEL=00 shows whether the device
//     selected the gyro MEMS oscillator or fell back to the RC oscillator, and
//     comparing 6-axis with accel-only shows whether enabling the gyro changes
//     the selected clock at all.
//  2. INT_STATUS.PLL_RDY (bit 5) has never been observed. INT_STATUS is
//     read-to-clear, so it is latched over a dedicated window.
//  3. FIFO packet path. The sense registers are one readout path, FIFO is the
//     other. If the gyro appears in FIFO, the fix is to read the FIFO.
//  4. TMST_VALUE through both routes with TMST_TO_REGS_EN=1. DS-000440 p85
//     says TMST_VALUE reads as 0 while that bit is clear, so the earlier
//     "frozen counter" reading was taken with the gate closed.
// ---------------------------------------------------------------------------

constexpr std::uint8_t kFifoData = 0x30u, kFifoCountH = 0x2eu;
// Vendor (Ixm42xxxDefs.h) FIFO_16BYTES_PACKET_SIZE = header + accel 6 + gyro 6
// + temp 1 + timestamp 2.
constexpr std::size_t kPacket3Size = 16u;
// Verified from the raw FIFO dump of 2026-10-30: with FIFO_TMST_FSYNC_EN=0 the
// accel+gyro+temp packet is 14 bytes (header 1 + accel 6 + gyro 6 + temp 1) and
// the timestamp field is absent even though the header reports one.
constexpr std::size_t kPacketNoTmstSize = 14u;

bool fifo_flush(drv_spi_inst_t spi) noexcept {
    return raw_write(spi, 0x4b, 0x02); // SIGNAL_PATH_RESET.FIFO_FLUSH
}

bool fifo_count(drv_spi_inst_t spi, std::uint16_t& count) noexcept {
    std::array<std::uint8_t, 2> raw{};
    if (!diagnostic_read(spi, kFifoCountH, raw.data(), raw.size()))
        return false;
    count = static_cast<std::uint16_t>((static_cast<std::uint16_t>(raw[0]) << 8) | raw[1]);
    return true;
}

// Stream mode, FIFO_COUNT_REC=1 so FIFO_COUNT reports records like the vendor
// driver gets. INTF_CONFIG0 bit 7 (invalid-code mode) is preserved.
bool fifo_start(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                std::uint8_t enables) noexcept {
    std::uint8_t intf0 = 0, fifo = 0, applied = 0;
    const bool ok = raw_bank(spi, 0) && diagnostic_read(spi, 0x4c, &intf0, 1)
        && raw_write(spi, 0x4c, static_cast<std::uint8_t>((intf0 & 0xbfu) | 0x40u))
        && diagnostic_read(spi, 0x16, &fifo, 1)
        && raw_write(spi, 0x16, static_cast<std::uint8_t>((fifo & 0x3fu) | 0x40u))
        && raw_write(spi, 0x5f, enables) && diagnostic_read(spi, 0x5f, &applied, 1)
        && applied == enables;
    log.line("imu_met case=%s op=fifo_start intf0_before=0x%02x enables=0x%02x accel=%u gyro=%u "
             "temp=%u mode=stream count=records verified=%u\n", name,
             static_cast<unsigned>(intf0), static_cast<unsigned>(applied),
             static_cast<unsigned>(enables & 1u), static_cast<unsigned>((enables >> 1) & 1u),
             static_cast<unsigned>((enables >> 2) & 1u), static_cast<unsigned>(ok));
    return ok;
}

bool fifo_drain(drv_spi_inst_t spi, std::uint8_t* out, std::size_t length) noexcept {
    std::size_t done = 0;
    while (done < length) {
        const std::size_t remaining = length - done;
        const std::size_t chunk = remaining > 14u ? 14u : remaining;
        if (!diagnostic_read(spi, kFifoData, out + done, chunk))
            return false;
        done += chunk;
    }
    return true;
}

// Reads back INTF_CONFIG1 after a read-modify-write so CLKSEL changes are
// never assumed.
bool select_clock(drv_spi_inst_t spi, std::uint8_t bits, std::uint8_t& applied) noexcept {
    std::uint8_t before = 0;
    if (!diagnostic_read(spi, 0x4d, &before, 1))
        return false;
    const std::uint8_t expected = static_cast<std::uint8_t>((before & 0xf8u) | bits);
    return raw_write(spi, 0x4d, expected) && diagnostic_read(spi, 0x4d, &applied, 1)
        && applied == expected;
}

void observe_int_status(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                        long window_ms) noexcept {
    std::uint8_t seen = 0, first = 0;
    unsigned polls = 0;
    const auto start = platform::mono_ms();
    while (running && static_cast<long>(platform::mono_ms() - start) < window_ms) {
        std::uint8_t status = 0;
        if (!diagnostic_read(spi, 0x2d, &status, 1))
            break;
        if (status != 0 && first == 0)
            first = status;
        seen = static_cast<std::uint8_t>(seen | status);
        ++polls;
        if (!pause_ms(5))
            break;
    }
    log.line("imu_met case=%s op=int_status window_ms=%ld polls=%u seen=0x%02x first=0x%02x "
             "pll_rdy=%u reset_done=%u data_rdy=%u fifo_ths=%u agc_rdy=%u\n", name, window_ms,
             polls, static_cast<unsigned>(seen), static_cast<unsigned>(first),
             static_cast<unsigned>((seen >> 5) & 1u), static_cast<unsigned>((seen >> 4) & 1u),
             static_cast<unsigned>((seen >> 3) & 1u), static_cast<unsigned>((seen >> 2) & 1u),
             static_cast<unsigned>(seen & 1u));
}

// Packet rate from FIFO_COUNT over three 150 ms windows. FIFO is flushed before
// each window so nothing overflows and no packet is counted twice.
void measure_odr(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                 std::uint8_t clock_bits, std::uint8_t power) noexcept {
    constexpr long kWindowMs = 150;
    std::uint8_t applied = 0;
    if (!cycle_power_off(spi, log, name)
        || !select_clock(spi, clock_bits, applied)
        || !raw_write(spi, 0x4f, 0x06) || !raw_write(spi, 0x50, 0x06)
        || !fifo_start(spi, log, name, 0x05u) // accel + temp, gyro packets off
        || !enable_power(spi, log, name, power) || !pause_ms(100) || !running) {
        log.line("imu_met case=%s op=odr result=setup_error\n", name);
        return;
    }
    unsigned total = 0;
    long elapsed = 0;
    bool ok = true;
    for (unsigned window = 0; window < 3u && ok && running; ++window) {
        std::uint16_t count = 0;
        if (!fifo_flush(spi) || !running)
            break;
        const auto start = platform::mono_ms();
        if (!pause_ms(kWindowMs))
            break;
        const auto stop = platform::mono_ms();
        ok = fifo_count(spi, count);
        total += count;
        elapsed += static_cast<long>(stop - start);
    }
    if (!ok || elapsed <= 0) {
        log.line("imu_met case=%s op=odr result=measure_error\n", name);
        return;
    }
    const auto milli_hz = static_cast<long>((static_cast<long>(total) * 1000 * 1000) / elapsed);
    log.line("imu_met case=%s op=odr intf1=0x%02x rtc=%u clksel=%u power=0x%02x packets=%u "
             "elapsed_ms=%ld odr_millihz=%ld odr_hz=%ld.%03ld\n", name,
             static_cast<unsigned>(applied), static_cast<unsigned>((applied >> 2) & 1u),
             static_cast<unsigned>(applied & 3u), static_cast<unsigned>(power), total, elapsed,
             milli_hz, milli_hz / 1000, milli_hz % 1000);
    (void)cycle_power_off(spi, log, name);
}

// Sample rate from DATA_RDY events and from distinct data frames. This works
// even when the FIFO accumulates nothing, which is what RTC_MODE=1 does here.
void rate_by_data_ready(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                        std::uint8_t clock_bits, long window_ms) noexcept {
    std::uint8_t applied = 0;
    if (!cycle_power_off(spi, log, name) || !select_clock(spi, clock_bits, applied)
        || !raw_write(spi, 0x16, 0x00) // FIFO bypass: sense registers only
        || !raw_write(spi, 0x4f, 0x06) || !raw_write(spi, 0x50, 0x06)
        || !enable_power(spi, log, name, 0x0fu) || !pause_ms(100) || !running) {
        log.line("imu_met case=%s op=dr_rate result=setup_error\n", name);
        return;
    }
    unsigned events = 0, polls = 0, frames = 0;
    std::array<std::uint8_t, 14> previous{}, current{};
    bool have_previous = false;
    const auto start = platform::mono_ms();
    while (running && static_cast<long>(platform::mono_ms() - start) < window_ms) {
        std::uint8_t status = 0;
        if (diagnostic_read(spi, 0x2d, &status, 1) && (status & 0x08u) != 0u)
            ++events;
        if (diagnostic_read(spi, 0x1d, current.data(), current.size()) != 0) {
            if (have_previous
                && std::memcmp(previous.data(), current.data(), current.size()) != 0)
                ++frames;
            previous = current;
            have_previous = true;
        }
        ++polls;
        if (!pause_ms(2))
            break;
    }
    const auto elapsed = platform::mono_ms() - start;
    if (elapsed <= 0) {
        log.line("imu_met case=%s op=dr_rate result=measure_error\n", name);
        return;
    }
    log.line("imu_met case=%s op=dr_rate intf1=0x%02x rtc=%u clksel=%u window_ms=%ld "
             "polls=%u data_ready_events=%u distinct_frames=%u dr_hz=%ld frame_hz=%ld\n",
             name, static_cast<unsigned>(applied), static_cast<unsigned>((applied >> 2) & 1u),
             static_cast<unsigned>(applied & 3u), static_cast<long>(elapsed), polls, events,
             frames, static_cast<long>(events) * 1000 / static_cast<long>(elapsed),
             static_cast<long>(frames) * 1000 / static_cast<long>(elapsed));
    (void)cycle_power_off(spi, log, name);
}

// Packet 3 (accel + gyro + temp, 16-bit format) parses as
// [0] header, [1..6] accel, [7..12] gyro, [13] temp, [14..15] timestamp.
void fifo_gyro_path(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                    std::uint8_t clock_bits) noexcept {
    static std::array<std::uint8_t, 2048> buffer;
    std::uint8_t applied = 0;
    if (!cycle_power_off(spi, log, name) || !select_clock(spi, clock_bits, applied)
        || !raw_write(spi, 0x4f, 0x06) || !raw_write(spi, 0x50, 0x06)
        || !fifo_start(spi, log, name, 0x07u) // accel + gyro + temp
        || !enable_power(spi, log, name, 0x0fu) || !pause_ms(100) || !running
        || !fifo_flush(spi) || !pause_ms(100) || !running) {
        log.line("imu_met case=%s op=fifo_gyro result=setup_error\n", name);
        return;
    }
    std::uint16_t records = 0;
    if (!fifo_count(spi, records)) {
        log.line("imu_met case=%s op=fifo_gyro result=count_error\n", name);
        return;
    }
    std::size_t bytes = static_cast<std::size_t>(records) * kPacketNoTmstSize;
    if (bytes > buffer.size())
        bytes = (buffer.size() / kPacketNoTmstSize) * kPacketNoTmstSize;
    if (bytes == 0u || !fifo_drain(spi, buffer.data(), bytes)) {
        log.line("imu_met case=%s op=fifo_gyro result=drain_error records=%u\n", name, records);
        return;
    }
    const unsigned packets = static_cast<unsigned>(bytes / kPacketNoTmstSize);
    unsigned accel_packets = 0, gyro_packets = 0, gyro_no_data = 0, empty = 0, headers_odd = 0;
    std::int16_t first_gyro[3]{};
    std::uint8_t first_header = 0;
    bool have_first = false;
    for (unsigned i = 0; i < packets; ++i) {
        const std::uint8_t* packet =
            buffer.data() + static_cast<std::size_t>(i) * kPacketNoTmstSize;
        const std::uint8_t header = packet[0];
        if ((header & 0x80u) != 0u) {
            ++empty; // HEADER_MSG
            continue;
        }
        if ((header & 0x40u) != 0u)
            ++accel_packets;
        if ((header & 0x20u) != 0u) {
            ++gyro_packets;
            const std::int16_t gx = diagnostic_word(packet + 7);
            const std::int16_t gy = diagnostic_word(packet + 9);
            const std::int16_t gz = diagnostic_word(packet + 11);
            if (!have_first) {
                first_gyro[0] = gx;
                first_gyro[1] = gy;
                first_gyro[2] = gz;
                first_header = header;
                have_first = true;
            }
            if (gx == INT16_MIN || gy == INT16_MIN || gz == INT16_MIN)
                ++gyro_no_data;
        }
        if ((header & 0x40u) == 0u && (header & 0x20u) == 0u)
            ++headers_odd;
    }
    // Framing is not assumed: the packet stride depends on which fields are
    // present, so the raw bytes are logged and both candidate strides are
    // scored by how many plausible packet headers they produce.
    {
        constexpr unsigned kHexBytes = 48;
        char hex[kHexBytes * 2 + 1]{};
        const unsigned shown = bytes < kHexBytes ? static_cast<unsigned>(bytes) : kHexBytes;
        constexpr char digits[] = "0123456789abcdef";
        for (unsigned i = 0; i < shown; ++i) {
            hex[2 * i] = digits[buffer[i] >> 4];
            hex[2 * i + 1] = digits[buffer[i] & 0x0f];
        }
        unsigned matches8 = 0, matches16 = 0;
        for (unsigned i = 0; i * 8u < bytes; ++i)
            if ((buffer[i * 8u] & 0xc0u) != 0u && (buffer[i * 8u] & 0x80u) == 0u)
                ++matches8;
        for (unsigned i = 0; i * kPacket3Size < bytes; ++i)
            if ((buffer[i * kPacket3Size] & 0xc0u) != 0u && (buffer[i * kPacket3Size] & 0x80u) == 0u)
                ++matches16;
        log.line("imu_met case=%s op=fifo_raw records=%u bytes=%u first_hex_bytes=%u "
                 "header_matches_stride8=%u header_matches_stride16=%u bytes_hex=%s\n", name,
                 records, static_cast<unsigned>(bytes), shown, matches8, matches16, hex);
    }
    log.line("imu_met case=%s op=fifo_gyro records=%u packets=%u bytes=%u accel_packets=%u "
             "gyro_packets=%u gyro_no_data=%u empty_headers=%u odd_headers=%u first_header=0x%02x "
             "first_gyro=%d,%d,%d intf1=0x%02x enabled=0x0f\n", name, records, packets,
             static_cast<unsigned>(bytes), accel_packets, gyro_packets, gyro_no_data, empty,
             headers_odd, static_cast<unsigned>(first_header), first_gyro[0], first_gyro[1],
             first_gyro[2], static_cast<unsigned>(applied));
    (void)cycle_power_off(spi, log, name);
}

// DS-000440 p85: with TMST_TO_REGS_EN=0 TMST_VALUE reads as 0 as well, so both
// readout routes are captured with the gate open.
void tmst_views(drv_spi_inst_t spi, logging::Logger& log, const char* name,
                std::uint8_t clock_bits, bool rtc_period) noexcept {
    std::uint8_t before = 0, applied = 0, clock = 0;
    // TMST_CONFIG needs the sensors OFF, but the counter only advances while a
    // sensor runs, so configuration and measurement are separated here.
    if (!cycle_power_off(spi, log, name) || !select_clock(spi, clock_bits, clock)) {
        log.line("imu_met case=%s op=tmst result=setup_error\n", name);
        return;
    }
    if (!diagnostic_read(spi, 0x54, &before, 1))
        return;
    const std::uint8_t expected =
        static_cast<std::uint8_t>((before & 0xe0u) | 0x10u | 0x01u | (rtc_period ? 0x08u : 0u));
    if (!raw_write(spi, 0x54, expected) || !diagnostic_read(spi, 0x54, &applied, 1)
        || applied != expected) {
        log.line("imu_met case=%s op=tmst result=config_error\n", name);
        return;
    }
    if (!enable_power(spi, log, name, 0x0fu) || !pause_ms(100) || !running)
        return;
    std::uint32_t strobe_first = 0, strobe_second = 0;
    std::array<std::uint8_t, 2> regs_first{}, regs_second{};
    const bool ok = read_tmst_value(spi, strobe_first)
        && diagnostic_read(spi, 0x2b, regs_first.data(), regs_first.size()) && pause_ms(300)
        && running && read_tmst_value(spi, strobe_second)
        && diagnostic_read(spi, 0x2b, regs_second.data(), regs_second.size());
    if (!ok) {
        log.line("imu_met case=%s op=tmst result=read_error\n", name);
        return;
    }
    const std::uint32_t regs_a =
        (static_cast<std::uint32_t>(regs_first[0]) << 8) | regs_first[1];
    const std::uint32_t regs_b =
        (static_cast<std::uint32_t>(regs_second[0]) << 8) | regs_second[1];
    const std::uint32_t expected_ticks = rtc_period ? 9830u : 300000u; // 300 ms
    log.line("imu_met case=%s op=tmst res=%s tmst_cfg=0x%02x strobe_first=%u strobe_second=%u "
             "strobe_delta_300ms=%u regs_first=%u regs_second=%u regs_delta_300ms=%u "
             "expected_ticks=%u intf1=0x%02x power=0x0f\n", name,
             rtc_period ? "rtc_period" : "us", static_cast<unsigned>(applied), strobe_first,
             strobe_second, (strobe_second - strobe_first) & 0x000fffffu, regs_a, regs_b,
             (regs_b - regs_a) & 0x0000ffffu, expected_ticks, static_cast<unsigned>(clock));
    (void)cycle_power_off(spi, log, name);
}

bool metrology_sensor(drv_spi_inst_t spi, logging::Logger& log) noexcept {
    const DiagnosticCase base{"v7_clkin", true, false, true, true, true, true, false};
    log.line("imu_met op=start version=11 phases=5 instrument=none note=software_only\n");
    if (!running)
        return true;

    // Read-only entry state before the first write, so a cold power-up can be
    // told apart from a CPU reset (DS-000440 p62/p68/p80).
    std::uint8_t entry_power = 0xff, entry_intf1 = 0;
    std::array<std::uint8_t, 14> entry_raw{};
    const bool entry_ok = diagnostic_read(spi, 0x4e, &entry_power, 1)
        && diagnostic_read(spi, 0x4d, &entry_intf1, 1)
        && diagnostic_read(spi, 0x1d, entry_raw.data(), entry_raw.size());
    log.line("imu_met op=entry read_ok=%u power=0x%02x intf1=0x%02x temp=%d accel=%d,%d,%d "
             "gyro=%d,%d,%d note=read_only_before_any_write\n",
             static_cast<unsigned>(entry_ok), static_cast<unsigned>(entry_power),
             static_cast<unsigned>(entry_intf1), diagnostic_word(entry_raw.data()),
             diagnostic_word(entry_raw.data() + 2), diagnostic_word(entry_raw.data() + 4),
             diagnostic_word(entry_raw.data() + 6), diagnostic_word(entry_raw.data() + 8),
             diagnostic_word(entry_raw.data() + 10), diagnostic_word(entry_raw.data() + 12));

    // 1. ODR metrology: predicted 1024 Hz with the 32.768 kHz oscillator, else
    //    ~1000 Hz from the internal sources.
    measure_odr(spi, log, "odr_clkin_6axis", 0x05u, 0x0fu);
    measure_odr(spi, log, "odr_internal_auto_6axis", 0x01u, 0x0fu);
    measure_odr(spi, log, "odr_internal_auto_accel", 0x01u, 0x03u);
    measure_odr(spi, log, "odr_internal_rc_6axis", 0x00u, 0x0fu);
    if (!running)
        return true;

    // 2. Sample rate without the FIFO, in both clock modes. The FIFO stayed
    //    empty under RTC_MODE=1, so the rate has to come from DATA_RDY and from
    //    how often the data frame itself changes.
    rate_by_data_ready(spi, log, "dr_rate_clkin", 0x05u, 2000);
    rate_by_data_ready(spi, log, "dr_rate_internal", 0x01u, 2000);
    if (!running)
        return true;

    // 3. PLL_RDY latch window with the internal clock, where sampling works.
    std::uint8_t applied = 0;
    if (!cycle_power_off(spi, log, "status_internal")
        || !select_clock(spi, 0x01u, applied)
        || !raw_write(spi, 0x4f, 0x06) || !raw_write(spi, 0x50, 0x06)
        || !enable_power(spi, log, "status_internal", 0x0fu))
        return false;
    observe_int_status(spi, log, "status_internal", 300);
    if (!running)
        return true;

    // 4. Gyro through the FIFO packet path, in both clock modes.
    fifo_gyro_path(spi, log, "fifo_gyro_internal", 0x01u);
    fifo_gyro_path(spi, log, "fifo_gyro_clkin", 0x05u);
    if (!running)
        return true;

    // 5. TMST_VALUE through both routes with TMST_TO_REGS_EN=1 and a sensor
    //    actually running, in both clock modes.
    tmst_views(spi, log, "tmst_internal_rtc", 0x01u, true);
    tmst_views(spi, log, "tmst_internal_us", 0x01u, false);
    tmst_views(spi, log, "tmst_clkin_rtc", 0x05u, true);
    if (!running)
        return true;

    log.line("imu_met op=complete action=restore_v7_reference\n");
    return diagnostic_setup(spi, log, base);
}

int run(const char* log_path, bool diagnose = false, bool external_clock = false,
        bool selftest = false, bool gyro_core = false, bool audit = false,
        bool metrology = false) noexcept {
    running = 1;
    if (std::signal(SIGINT, stop) == SIG_ERR || std::signal(SIGTERM, stop) == SIG_ERR)
        return 2;
    // RT-Smart printf can bypass dup2/launcher pipes. Write a status file
    // directly, closing it after each line so USB/MTP can read it while running.
    logging::Logger log;
    (void)log.open(log_path);
    log.line("imu_raw op=start bus=%d cs=%d mode=3 bus_hz=%u period_ms=%ld sensor_init=%s\n",
             kBus, kChipSelect, kBusHz, kPeriodMs, metrology ? "metrology_v10"
                 : audit ? "startup_audit_v9"
                 : gyro_core ? "gyro_core_ln_v8"
                 : selftest ? "vendor_selftest_ln_v7"
                 : external_clock ? "external_clock_ln_v7"
                 : diagnose ? "gyro_diagnostic_ln_v7" : "low_noise_poll_v7");
    if (audit)
        log.line("imu_audit op=process pid=%ld ppid=%ld before_spi_create=1\n",
                 static_cast<long>(getpid()), static_cast<long>(getppid()));
    drv_spi_inst_t spi = nullptr;
    while (running) {
        if (drv_spi_inst_create(kBus, true, SPI_HAL_MODE_3, kBusHz, 8,
                                kChipSelect, SPI_HAL_DATA_LINE_1, &spi) == 0)
            break;
        // The SDK frees a failed instance without clearing its output pointer.
        spi = nullptr;
        log.line("imu_raw op=bus result=retry bus=%d cs=%d\n", kBus, kChipSelect);
        if (!pause_ms(1000))
            return 2;
    }
    if (spi == nullptr)
        return 0;
    log.line("imu_raw op=bus result=ready\n");

    // CS stays asserted across command + dummy bytes in each full-duplex call.
    constexpr std::array<std::uint8_t, 2> id_tx{0xf5, 0x00};
    constexpr std::array<std::uint8_t, 15> data_tx{0x9d};
    std::uint64_t cycles = 0, errors = 0;
    int result = (metrology ? metrology_sensor(spi, log)
                  : audit ? startup_audit_sensor(spi, log)
                  : gyro_core ? gyro_core_sensor(spi, log)
                  : selftest ? vendor_selftest_sensor(spi, log)
                  : external_clock ? external_clock_sensor(spi, log)
                  : diagnose ? diagnose_sensor(spi, log) : wake_sensor(spi, log)) ? 0 : 2;
    while (running && result == 0) {
        std::array<std::uint8_t, 2> id_rx{};
        std::array<std::uint8_t, 15> data_rx{};
        const int id_rc = drv_spi_transfer(spi, id_tx.data(), id_rx.data(), id_tx.size(), true);
        const int data_rc = drv_spi_transfer(
            spi, data_tx.data(), data_rx.data(), data_tx.size(), true);
        errors += id_rc != static_cast<int>(id_tx.size());
        errors += data_rc != static_cast<int>(data_tx.size());
        ++cycles;
        // Report the first cycle and about once per second; never gate polling
        // on an ID value, DATA_RDY, or whether the returned data look valid.
        if (cycles == 1 || cycles % 100 == 0) {
            constexpr char hex[] = "0123456789abcdef";
            char raw[29]{};
            for (std::size_t i = 1; i < data_rx.size(); ++i) {
                raw[(i - 1) * 2] = hex[data_rx[i] >> 4];
                raw[(i - 1) * 2 + 1] = hex[data_rx[i] & 0x0f];
            }
            log.line("imu_raw cycles=%llu errors=%llu id_rc=%d id=0x%02x data_rc=%d raw=%s\n",
                     static_cast<unsigned long long>(cycles),
                     static_cast<unsigned long long>(errors), id_rc,
                     static_cast<unsigned>(id_rx[1]), data_rc, raw);
        }
        if (!pause_ms(kPeriodMs)) {
            result = 2;
            break;
        }
    }
    drv_spi_inst_destroy(&spi);
    log.line("imu_raw op=stop cycles=%llu errors=%llu result=%d\n",
             static_cast<unsigned long long>(cycles),
             static_cast<unsigned long long>(errors), result);
    return result;
}

} // namespace dart::imu::raw_probe

int main(int argc, char** argv) {
    if (argc > 3 || (argc == 3 && std::strcmp(argv[2], "--diagnose") != 0
                     && std::strcmp(argv[2], "--external-clock") != 0
                     && std::strcmp(argv[2], "--self-test") != 0
                     && std::strcmp(argv[2], "--gyro-core") != 0
                     && std::strcmp(argv[2], "--startup-audit") != 0
                     && std::strcmp(argv[2], "--metrology") != 0))
        return 2;
    return dart::imu::raw_probe::run(
        argc > 1 ? argv[1] : "/data/app/logs/imu_raw_probe.log",
        argc == 3 && std::strcmp(argv[2], "--diagnose") == 0,
        argc == 3 && std::strcmp(argv[2], "--external-clock") == 0,
        argc == 3 && std::strcmp(argv[2], "--self-test") == 0,
        argc == 3 && std::strcmp(argv[2], "--gyro-core") == 0,
        argc == 3 && std::strcmp(argv[2], "--startup-audit") == 0,
        argc == 3 && std::strcmp(argv[2], "--metrology") == 0);
}
