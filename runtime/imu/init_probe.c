#define _POSIX_C_SOURCE 200809L
/* Run with every other IMU/SPI user stopped. Does not start the camera.
 *
 *   imu_init_probe [--gyro|--gyro2|--cold|--toggle|--int1] [log path]
 *
 * Default mode repeats the IIM-42652 initialization with several pre/post
 * delays. --gyro mode drives the SPI bus directly (bypassing dart_iim42652)
 * and runs the gyro-focused experiment matrix E0..E13; every line is mirrored
 * to stdout so it can be read on the serial console.
 *
 * --gyro2 runs the extended matrix E14..E22 (full bank dumps, factory
 * self-test records, PLL/clock probe, gyro self-test, FIFO capture, most
 * sensitive setting, bus robustness). --cold is the no-soft-reset cold start
 * probe and must be run right after a power cycle. --toggle drives a 10 Hz
 * supply load step for a scope, and --int1 routes UI data ready to INT1 so a
 * scope can show whether any data exists at all.
 *
 * The matrix always starts with a soft reset, so registers written by an
 * experiment never survive into the next run. */
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "drv_spi.h"
#include "spi/iim42652.h"

#define REG_DEVICE_CONFIG 0x11u
#define REG_TEMP_DATA1    0x1du
#define REG_TEMP_DATA2    0x1eu
#define REG_ACCEL_DATA_X1 0x1fu
#define REG_GYRO_DATA_X1  0x25u
#define REG_INT_STATUS    0x2du
#define REG_PWR_MGMT0     0x4eu
#define REG_GYRO_CONFIG0  0x4fu
#define REG_ACCEL_CONFIG0 0x50u
#define REG_INTF_CONFIG0  0x4cu
#define REG_INTF_CONFIG1  0x4du
#define REG_WHO_AM_I      0x75u
#define REG_BANK_SEL      0x76u
#define REG_SENSOR_CFG0   0x03u /* Bank 1 */

/* gyro2 matrix */
#define REG_INT_CONFIG        0x14u
#define REG_FIFO_CONFIG       0x16u
#define REG_FIFO_CONFIG1      0x5fu
#define REG_INT_CONFIG1       0x64u
#define REG_INT_SOURCE0       0x65u
#define REG_SELF_TEST_CONFIG  0x70u
#define REG_TMST_CONFIG       0x54u

static FILE* g_log;

static void emit(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    (void)vfprintf(stdout, format, ap);
    va_end(ap);
    (void)fflush(stdout);
    if (g_log != NULL) {
        va_start(ap, format);
        (void)vfprintf(g_log, format, ap);
        va_end(ap);
        (void)fflush(g_log);
    }
}

static uint64_t mono_ns(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return 0;
    return (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec;
}

static void sleep_ms(unsigned int milliseconds) {
    struct timespec remaining = {
        .tv_sec = (time_t)(milliseconds / 1000u),
        .tv_nsec = (long)(milliseconds % 1000u) * 1000000L};
    while (nanosleep(&remaining, &remaining) != 0 && errno == EINTR) {
    }
}

static void notify_launcher(void) {
    /* RT-Smart waitpid/kill may retain a completed process. Use the same
     * explicit completion protocol as dart_app for finite foreground jobs. */
    const char* path = getenv("DART_EXIT_MARKER");
    if (path == NULL || path[0] != '/' || strlen(path) >= 512u) return;
    const int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        char text[32];
        const int length = snprintf(text, sizeof(text), "%ld\n", (long)getpid());
        if (length > 0 && (size_t)length < sizeof(text)) {
            (void)write(fd, text, (size_t)length);
            (void)fsync(fd);
        }
        close(fd);
    }
}

/* --- raw register access, same framing as dart_iim42652 ------------------- */

static int reg_read(drv_spi_inst_t spi, uint8_t reg, uint8_t* buffer, uint16_t length) {
    uint8_t tx[17];
    uint8_t rx[17];
    if (spi == NULL || buffer == NULL || length == 0u || length > 16u)
        return -1;
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    tx[0] = (uint8_t)(reg | 0x80u);
    if (drv_spi_transfer(spi, tx, rx, (size_t)length + 1u, true) != (int)(length + 1u))
        return -1;
    memcpy(buffer, &rx[1], length);
    return 0;
}

static int reg_write(drv_spi_inst_t spi, uint8_t reg, uint8_t value) {
    const uint8_t bytes[2] = {(uint8_t)(reg & 0x7fu), value};
    if (spi == NULL)
        return -1;
    return drv_spi_write(spi, bytes, sizeof(bytes), true) == 2 ? 0 : -1;
}

static uint8_t reg_read_u8(drv_spi_inst_t spi, uint8_t reg) {
    uint8_t value = 0;
    (void)reg_read(spi, reg, &value, 1u);
    return value;
}

static int read_three(drv_spi_inst_t spi, uint8_t base, int16_t out[3]) {
    uint8_t raw[6];
    out[0] = out[1] = out[2] = 0;
    if (reg_read(spi, base, raw, 6u) != 0)
        return -1;
    for (unsigned int axis = 0; axis < 3u; ++axis) {
        const uint16_t bits = (uint16_t)(((uint16_t)raw[axis * 2u] << 8) | raw[axis * 2u + 1u]);
        memcpy(&out[axis], &bits, sizeof(out[axis]));
    }
    return 0;
}

static void report(const char* tag, drv_spi_inst_t spi) {
    const uint8_t status = reg_read_u8(spi, REG_INT_STATUS); /* read-to-clear */
    int16_t gyro[3];
    int16_t accel[3];
    const int gyro_ok = read_three(spi, REG_GYRO_DATA_X1, gyro);
    const int accel_ok = read_three(spi, REG_ACCEL_DATA_X1, accel);
    emit("exp=%s status=0x%02x gyro_ok=%d gyro=%d,%d,%d accel_ok=%d accel=%d,%d,%d\n",
         tag, status, gyro_ok, gyro[0], gyro[1], gyro[2], accel_ok, accel[0], accel[1],
         accel[2]);
}

/* --- gyro experiment matrix ---------------------------------------------- */

static int gyro_probe(int bus, int cs_pin, uint32_t bus_hz) {
    drv_spi_inst_t spi = NULL;
    int result = 0;

    emit("[gyro-probe] version=1 bus=%d cs=%d hz=%" PRIu32 "\n", bus, cs_pin, bus_hz);
    if (drv_spi_inst_create(
            bus, true, SPI_HAL_MODE_3, bus_hz, 8u, cs_pin, SPI_HAL_DATA_LINE_1,
            &spi) != 0) {
        /* The HAL frees its partial instance but leaves the output pointer
         * dangling, so a failing create is never followed by a transfer. */
        emit("exp=bus_open result=fail bus=%d cs=%d action=abort "
             "(check /dev/spi%d and that no other IMU user is running)\n",
             bus, cs_pin, bus);
        return 1;
    }

    /* E0: soft reset, identity and the key configuration registers. */
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    (void)reg_write(spi, REG_DEVICE_CONFIG, 0x01u);
    sleep_ms(5u);
    emit("exp=E0_reset id=0x%02x intf0=0x%02x intf1=0x%02x power=0x%02x "
         "gyro_cfg=0x%02x accel_cfg=0x%02x\n",
         reg_read_u8(spi, REG_WHO_AM_I), reg_read_u8(spi, REG_INTF_CONFIG0),
         reg_read_u8(spi, REG_INTF_CONFIG1), reg_read_u8(spi, REG_PWR_MGMT0),
         reg_read_u8(spi, REG_GYRO_CONFIG0), reg_read_u8(spi, REG_ACCEL_CONFIG0));

    /* E0b: bank 1 dump; also verifies that BANK_SEL actually latches. */
    (void)reg_write(spi, REG_BANK_SEL, 1u);
    {
        const uint8_t bank_sel = reg_read_u8(spi, REG_BANK_SEL);
        uint8_t dump[16];
        if (reg_read(spi, 0x00u, dump, sizeof(dump)) == 0) {
            emit("exp=E0_bank1 bank_sel_rb=0x%02x reg00_0f=", bank_sel);
            for (unsigned int i = 0; i < sizeof(dump); ++i)
                emit("%02x", dump[i]);
            emit("\n");
        } else {
            emit("exp=E0_bank1 bank_sel_rb=0x%02x read=fail\n", bank_sel);
        }
    }
    (void)reg_write(spi, REG_BANK_SEL, 0u);

    /* E1: sensors still off after reset; the gyro must report 0x8000. */
    report("E1_off", spi);

    /* E2: gyro only, low noise. */
    (void)reg_write(spi, REG_PWR_MGMT0, 0x0cu);
    sleep_ms(200u);
    emit("exp=E2_setup power_w=0x0c power_rb=0x%02x\n", reg_read_u8(spi, REG_PWR_MGMT0));
    report("E2_gyro_only", spi);

    /* E3: accel only, control case. */
    (void)reg_write(spi, REG_PWR_MGMT0, 0x03u);
    sleep_ms(100u);
    emit("exp=E3_setup power_w=0x03 power_rb=0x%02x\n", reg_read_u8(spi, REG_PWR_MGMT0));
    report("E3_accel_only", spi);

    /* E4: both low noise, sweep the gyro ODR. */
    (void)reg_write(spi, REG_PWR_MGMT0, 0x0fu);
    sleep_ms(60u);
    {
        static const struct {
            uint8_t config;
            const char* tag;
        } odr_cases[] = {
            {0x0au, "E4_odr_32hz"},
            {0x08u, "E4_odr_200hz"},
            {0x06u, "E4_odr_1khz"},
            {0x04u, "E4_odr_4khz"},
        };
        for (unsigned int i = 0; i < sizeof(odr_cases) / sizeof(odr_cases[0]); ++i) {
            (void)reg_write(spi, REG_GYRO_CONFIG0, odr_cases[i].config);
            sleep_ms(150u);
            emit("exp=%s cfg_w=0x%02x cfg_rb=0x%02x\n", odr_cases[i].tag,
                 odr_cases[i].config, reg_read_u8(spi, REG_GYRO_CONFIG0));
            report(odr_cases[i].tag, spi);
        }
    }

    /* E5: gyro full scale 500 dps instead of 2000 dps. */
    (void)reg_write(spi, REG_GYRO_CONFIG0, 0x46u);
    sleep_ms(150u);
    emit("exp=E5_setup cfg_w=0x46 cfg_rb=0x%02x\n", reg_read_u8(spi, REG_GYRO_CONFIG0));
    report("E5_fs_500dps", spi);

    /* E6: clear SENSOR_CONFIG0 bit 7 in bank 1, then watch the gyro. */
    (void)reg_write(spi, REG_BANK_SEL, 1u);
    {
        const uint8_t before = reg_read_u8(spi, REG_SENSOR_CFG0);
        (void)reg_write(spi, REG_SENSOR_CFG0, (uint8_t)(before & 0x7fu));
        const uint8_t after = reg_read_u8(spi, REG_SENSOR_CFG0);
        (void)reg_write(spi, REG_BANK_SEL, 0u);
        emit("exp=E6_setup sc0_before=0x%02x sc0_after=0x%02x\n", before, after);
    }
    sleep_ms(150u);
    report("E6_sc0_bit7_clear", spi);

    /* E7: force the clock selector to the internal RC oscillator. */
    {
        const uint8_t before = reg_read_u8(spi, REG_INTF_CONFIG1);
        (void)reg_write(spi, REG_INTF_CONFIG1, (uint8_t)(before & (uint8_t)~0x03u));
        const uint8_t after = reg_read_u8(spi, REG_INTF_CONFIG1);
        emit("exp=E7_setup intf1_before=0x%02x intf1_after=0x%02x clksel=%u\n", before, after,
             (unsigned int)(after & 0x03u));
    }
    sleep_ms(200u);
    report("E7_clksel_rc", spi);

    /* E8: single-transaction latency, relevant for 1 kHz sampling later. */
    {
        const uint64_t start = mono_ns();
        for (unsigned int i = 0; i < 100u; ++i)
            (void)reg_read_u8(spi, REG_WHO_AM_I);
        const uint64_t elapsed = mono_ns() - start;
        emit("exp=E8_timing reads=100 total_us=%" PRIu64 " per_read_us=%" PRIu64 "\n",
             elapsed / 1000ull, (elapsed / 100ull) / 1000ull);
    }

    /* E9/E10: who actually produces data-ready events. Bit 0 and bit 4 are
     * reported separately because the INT_STATUS layout is not yet confirmed
     * for this part; either way a silent gyro must not raise any of them. */
    {
        static const struct {
            uint8_t power;
            const char* tag;
        } ready_cases[] = {
            {0x0cu, "E9_gyro_only_ready"},
            {0x03u, "E10_accel_only_ready"},
        };
        for (unsigned int c = 0; c < sizeof(ready_cases) / sizeof(ready_cases[0]); ++c) {
            (void)reg_write(spi, REG_PWR_MGMT0, ready_cases[c].power);
            sleep_ms(100u);
            unsigned int bit0 = 0, bit4 = 0, other = 0;
            uint8_t seen = 0;
            for (unsigned int i = 0; i < 200u; ++i) {
                const uint8_t status = reg_read_u8(spi, REG_INT_STATUS);
                seen |= status;
                if ((status & 0x01u) != 0u) ++bit0;
                if ((status & 0x10u) != 0u) ++bit4;
                if ((status & 0xeeu) != 0u) ++other;
                sleep_ms(5u);
            }
            emit("exp=%s power=0x%02x polls=200 status_or=0x%02x bit0=%u bit4=%u other=%u\n",
                 ready_cases[c].tag, ready_cases[c].power, seen, bit0, bit4, other);
        }
    }

    /* E11: gyro standby versus low-noise. */
    (void)reg_write(spi, REG_PWR_MGMT0, 0x04u);
    sleep_ms(200u);
    emit("exp=E11_setup power_w=0x04 power_rb=0x%02x\n", reg_read_u8(spi, REG_PWR_MGMT0));
    report("E11_gyro_standby", spi);
    (void)reg_write(spi, REG_PWR_MGMT0, 0x0cu);
    sleep_ms(200u);
    report("E11_gyro_lownoise", spi);

    /* E12: temperature; a sane value proves the analogue front end is alive. */
    (void)reg_write(spi, REG_PWR_MGMT0, 0x0fu);
    sleep_ms(100u);
    {
        const int16_t raw = (int16_t)(((uint16_t)reg_read_u8(spi, REG_TEMP_DATA1) << 8) |
                                      (uint16_t)reg_read_u8(spi, REG_TEMP_DATA2));
        /* Datasheet: T_C = raw / 132.48 + 25. */
        const float celsius = (float)raw / 132.48f + 25.0f;
        emit("exp=E12_temperature raw=%d celsius=%.2f\n", (int)raw, (double)celsius);
    }

    /* E13: bank 0 sweep plus the auxiliary filter configuration registers. */
    {
        uint8_t dump[16];
        emit("exp=E13_bank0_00_0f ");
        if (reg_read(spi, 0x00u, dump, sizeof(dump)) == 0)
            for (unsigned int i = 0; i < sizeof(dump); ++i) emit("%02x", dump[i]);
        else
            emit("read=fail");
        emit("\n");

        emit("exp=E13_bank0_40_4f ");
        if (reg_read(spi, 0x40u, dump, sizeof(dump)) == 0)
            for (unsigned int i = 0; i < sizeof(dump); ++i) emit("%02x", dump[i]);
        else
            emit("read=fail");
        emit("\n");

        emit("exp=E13_bank0_50_5f ");
        if (reg_read(spi, 0x50u, dump, sizeof(dump)) == 0)
            for (unsigned int i = 0; i < sizeof(dump); ++i) emit("%02x", dump[i]);
        else
            emit("read=fail");
        emit("\n");
    }

    /* Leave the device in its reset state so nothing we wrote survives. */
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    (void)reg_write(spi, REG_DEVICE_CONFIG, 0x01u);
    sleep_ms(5u);
    drv_spi_inst_destroy(&spi);
    emit("[gyro-probe] done result=%d\n", result);
    return result;
}

/* --- gyro2: extended register / clock / FIFO / self-test matrix (E14..E24) */

static void emit_hex(const uint8_t* bytes, unsigned int length) {
    for (unsigned int i = 0; i < length; ++i) emit("%02x", bytes[i]);
}

/* Read a whole bank in 16-byte frames and print it as one hex line. */
static int dump_bank(drv_spi_inst_t spi, uint8_t bank, uint8_t last, const char* tag) {
    uint8_t chunk[16];
    unsigned int base;
    (void)reg_write(spi, REG_BANK_SEL, bank);
    emit("exp=%s ", tag);
    for (base = 0u; base <= (unsigned int)last; base += 16u) {
        if (reg_read(spi, (uint8_t)base, chunk, 16u) != 0) {
            emit("read_fail@%02x\n", base);
            (void)reg_write(spi, REG_BANK_SEL, 0u);
            return -1;
        }
        emit_hex(chunk, 16u);
    }
    emit("\n");
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    return 0;
}

/* E14: every user bank, so the undocumented ranges are on record too. */
static void experiment_snapshot(drv_spi_inst_t spi) {
    (void)dump_bank(spi, 0u, 0x70u, "E14_bank0_00_7f");
    (void)dump_bank(spi, 1u, 0x70u, "E14_bank1_00_7f");
    (void)dump_bank(spi, 2u, 0x70u, "E14_bank2_00_7f");
    (void)dump_bank(spi, 3u, 0x00u, "E14_bank3_00_0f");
    (void)dump_bank(spi, 4u, 0x70u, "E14_bank4_00_7f");
}

/* E25: is a 16-byte burst read trustworthy everywhere?  E14 showed a 0xE8 fill
 * from 0x74 in banks 0/1 while a single read of 0x75 still returned 0x6F, so
 * compare burst against 4-byte splits and individual reads at the same spot. */
static void experiment_read_consistency(drv_spi_inst_t spi, uint8_t bank, uint8_t start) {
    uint8_t burst[16];
    uint8_t split[16];
    uint8_t single[16];
    unsigned int i;

    (void)reg_write(spi, REG_BANK_SEL, bank);
    if (reg_read(spi, start, burst, 16u) != 0) {
        emit("exp=E25_burst bank=%u addr=%02x read_fail\n", (unsigned int)bank,
             (unsigned int)start);
        (void)reg_write(spi, REG_BANK_SEL, 0u);
        return;
    }
    for (i = 0u; i < 4u; ++i)
        (void)reg_read(spi, (uint8_t)(start + i * 4u), split + i * 4u, 4u);
    for (i = 0u; i < 16u; ++i)
        (void)reg_read(spi, (uint8_t)(start + i), &single[i], 1u);
    (void)reg_write(spi, REG_BANK_SEL, 0u);

    emit("exp=E25_burst bank=%u addr=%02x ", (unsigned int)bank, (unsigned int)start);
    emit_hex(burst, 16u);
    emit("\n");
    emit("exp=E25_split bank=%u addr=%02x ", (unsigned int)bank, (unsigned int)start);
    emit_hex(split, 16u);
    emit("\n");
    emit("exp=E25_single bank=%u addr=%02x ", (unsigned int)bank, (unsigned int)start);
    emit_hex(single, 16u);
    emit("\n");
    emit("exp=E25_match bank=%u addr=%02x burst_eq_split=%d burst_eq_single=%d\n",
         (unsigned int)bank, (unsigned int)start, memcmp(burst, split, 16u) == 0,
         memcmp(burst, single, 16u) == 0);
}

/* E22: history check.  Turned out 0x46 in an earlier round, so read the
 * undocumented bank-1 alias of 0x75 to see whether that was a bank mixup. */
static void experiment_bank1_alias(drv_spi_inst_t spi) {
    uint8_t alias = 0;
    (void)reg_write(spi, REG_BANK_SEL, 1u);
    const int ok = reg_read(spi, 0x75u, &alias, 1u) == 0;
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    emit("exp=E22_bank1_75 ok=%d value=0x%02x\n", ok, alias);
}

/* E15: factory-recorded self-test output (bank 1 0x5F..0x61 / bank 2 0x3B..0x3D). */
static void experiment_factory_st(drv_spi_inst_t spi) {
    uint8_t gyro[3] = {0, 0, 0};
    uint8_t accel[3] = {0, 0, 0};
    (void)reg_write(spi, REG_BANK_SEL, 1u);
    const int gyro_ok = reg_read(spi, 0x5fu, gyro, 3u) == 0;
    (void)reg_write(spi, REG_BANK_SEL, 2u);
    const int accel_ok = reg_read(spi, 0x3bu, accel, 3u) == 0;
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    emit("exp=E15_factory_st gyro_ok=%d gyro=%02x,%02x,%02x accel_ok=%d accel=%02x,%02x,%02x\n",
         gyro_ok, gyro[0], gyro[1], gyro[2], accel_ok, accel[0], accel[1], accel[2]);
}

/* E17: does the PLL lock?  Default CLKSEL=01 means "PLL when available, else RC",
 * and the PLL is referenced to the gyro MEMS oscillator.  Force RC, clear the
 * latched status, switch to PLL, then watch bit5 (PLL_RDY) / bit3 (DATA_RDY).
 * INT_STATUS latches regardless of INT_SOURCE0 routing (confirmed by E3/E10,
 * where DATA_RDY appeared although INT_SOURCE0 kept its 0x10 default). */
static void experiment_clock(drv_spi_inst_t spi) {
    uint8_t before, cleared, rc_value, pll_value, tmst_cfg, seen = 0;
    unsigned int pll_hits = 0, drdy_hits = 0, reset_hits = 0, other_hits = 0, i;
    uint8_t tmst[3][3];
    uint32_t stamp[3];

    /* DS-000440 12.9: only ODR/FSR/sensor mode may change while a sensor runs,
     * so park both sensors before touching CLKSEL, then start them again with
     * the PLL selected.  A dead MEMS oscillator never locks, and the part then
     * silently stays on RC, which is why everything else keeps working. */
    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    sleep_ms(10u);

    /* Control: the 20-bit timestamp counter must advance if the internal clock
     * domain runs.  TMSTVAL is only a latch (DS-000440 p.96) and that latch only
     * updates with TMST_TO_REGS_EN (TMST_CONFIG bit 4), which is 0 by default. */
    tmst_cfg = reg_read_u8(spi, REG_TMST_CONFIG);
    (void)reg_write(spi, REG_TMST_CONFIG, (uint8_t)(tmst_cfg | 0x10u));
    sleep_ms(2u);
    for (i = 0u; i < 3u; ++i) {
        (void)reg_write(spi, 0x4bu, 0x04u); /* TMST_STROBE, write-one self clearing */
        sleep_ms(2u);
        (void)reg_write(spi, REG_BANK_SEL, 1u);
        tmst[i][0] = tmst[i][1] = tmst[i][2] = 0u;
        (void)reg_read(spi, 0x62u, tmst[i], 3u);
        (void)reg_write(spi, REG_BANK_SEL, 0u);
        stamp[i] = (uint32_t)tmst[i][0] | ((uint32_t)tmst[i][1] << 8) |
                   (((uint32_t)tmst[i][2] & 0x0fu) << 16);
        sleep_ms(5u);
    }
    (void)reg_write(spi, REG_TMST_CONFIG, tmst_cfg);
    emit("exp=E17_tmst tmst_cfg=0x%02x t0=%u t1=%u t2=%u advancing=%d\n", tmst_cfg, stamp[0],
         stamp[1], stamp[2], (stamp[1] != stamp[0] || stamp[2] != stamp[1]) ? 1 : 0);

    before = reg_read_u8(spi, REG_INTF_CONFIG1);

    (void)reg_write(spi, REG_INTF_CONFIG1, (uint8_t)(before & (uint8_t)~0x03u));
    sleep_ms(20u);
    rc_value = reg_read_u8(spi, REG_INTF_CONFIG1);
    cleared = reg_read_u8(spi, REG_INT_STATUS); /* read-to-clear */
    (void)reg_write(spi, REG_INTF_CONFIG1, (uint8_t)((before & (uint8_t)~0x03u) | 0x01u));
    sleep_ms(5u);
    pll_value = reg_read_u8(spi, REG_INTF_CONFIG1);
    (void)reg_write(spi, REG_PWR_MGMT0, 0x0fu);
    sleep_ms(50u);

    for (i = 0u; i < 500u; ++i) {
        const uint8_t status = reg_read_u8(spi, REG_INT_STATUS);
        seen |= status;
        if ((status & 0x20u) != 0u) ++pll_hits;
        if ((status & 0x08u) != 0u) ++drdy_hits;
        if ((status & 0x10u) != 0u) ++reset_hits;
        if ((status & (uint8_t)~0x38u) != 0u) ++other_hits;
        sleep_ms(1u);
    }
    emit("exp=E17_clock intf1_before=0x%02x rc_rb=0x%02x pll_rb=0x%02x cleared=0x%02x "
         "polls=500 pll_rdy=%u data_rdy=%u reset_done=%u other=%u status_or=0x%02x\n",
         before, rc_value, pll_value, cleared, pll_hits, drdy_hits, reset_hits, other_hits, seen);

    /* Put the clock source back, still with the sensors parked (12.9). */
    (void)reg_write(spi, REG_INTF_CONFIG1, before);
}

/* E16: gyro self-test.  The electronics actuate the sense element, so a working
 * gyro channel must show a response even when the normal signal path is quiet. */
static void experiment_self_test(drv_spi_inst_t spi) {
    int16_t before[3] = {0, 0, 0}, during[3] = {0, 0, 0}, after[3] = {0, 0, 0};
    int16_t accel_before[3] = {0, 0, 0}, accel_during[3] = {0, 0, 0};
    uint8_t st[3] = {0, 0, 0};

    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    sleep_ms(10u);
    (void)reg_write(spi, REG_GYRO_CONFIG0, 0x06u);  /* +-2000 dps, 1 kHz */
    (void)reg_write(spi, REG_ACCEL_CONFIG0, 0x06u);
    (void)reg_write(spi, REG_SELF_TEST_CONFIG, 0x00u);
    (void)reg_write(spi, REG_PWR_MGMT0, 0x0fu);     /* both low noise, last write */
    sleep_ms(100u);
    (void)read_three(spi, REG_GYRO_DATA_X1, before);
    (void)read_three(spi, REG_ACCEL_DATA_X1, accel_before);
    emit("exp=E16_baseline gyro=%d,%d,%d accel=%d,%d,%d\n", before[0], before[1], before[2],
         accel_before[0], accel_before[1], accel_before[2]);

    (void)reg_write(spi, REG_SELF_TEST_CONFIG, 0x07u); /* EN_GX|EN_GY|EN_GZ */
    sleep_ms(100u);
    (void)read_three(spi, REG_GYRO_DATA_X1, during);
    (void)read_three(spi, REG_ACCEL_DATA_X1, accel_during);
    (void)reg_write(spi, REG_BANK_SEL, 1u);
    (void)reg_read(spi, 0x5fu, st, 3u);
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    emit("exp=E16_st_on gyro=%d,%d,%d accel=%d,%d,%d bank1_5f_61=%02x,%02x,%02x\n", during[0],
         during[1], during[2], accel_during[0], accel_during[1], accel_during[2], st[0], st[1],
         st[2]);

    (void)reg_write(spi, REG_SELF_TEST_CONFIG, 0x00u);
    sleep_ms(100u);
    (void)read_three(spi, REG_GYRO_DATA_X1, after);
    emit("exp=E16_st_off gyro=%d,%d,%d\n", after[0], after[1], after[2]);

    emit("exp=E16_delta gyro=%d,%d,%d accel=%d,%d,%d\n", (int)during[0] - (int)before[0],
         (int)during[1] - (int)before[1], (int)during[2] - (int)before[2],
         (int)accel_during[0] - (int)accel_before[0],
         (int)accel_during[1] - (int)accel_before[1],
         (int)accel_during[2] - (int)accel_before[2]);
    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
}

/* E18: bypass the sense registers and look at the FIFO directly.  This is the
 * only experiment that can overturn the "no sample ever produced" reading. */
static unsigned int experiment_fifo_case(
    drv_spi_inst_t spi, uint8_t power, uint8_t fifo_config1, const char* tag) {
    uint8_t count[2] = {0, 0};
    uint8_t data[16];
    uint8_t lost[2] = {0, 0};
    unsigned int bytes = 0, taken, offset;

    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    sleep_ms(10u);
    /* FIFO_MODE lives in bits 7:6: 01 = stream-to-FIFO, i.e. 0x40.  Enabling
     * the sensor packet masks only works while the FIFO is still bypassed. */
    (void)reg_write(spi, REG_FIFO_CONFIG1, fifo_config1);
    (void)reg_write(spi, REG_FIFO_CONFIG, 0x40u);
    (void)reg_write(spi, 0x4bu, 0x02u);             /* SIGNAL_PATH_RESET.FIFO_FLUSH */
    sleep_ms(5u);
    (void)reg_write(spi, REG_PWR_MGMT0, power);
    sleep_ms(100u);

    if (reg_read(spi, 0x2eu, count, 2u) == 0)
        bytes = ((unsigned int)count[0] << 8) | (unsigned int)count[1];
    emit("exp=%s power=0x%02x fifo_config1=0x%02x count=%u\n", tag, power, fifo_config1, bytes);

    if (bytes > 0u) {
        taken = bytes > 64u ? 64u : bytes;
        emit("exp=%s_data bytes=%u ", tag, taken);
        for (offset = 0u; offset < taken; offset += 16u) {
            const unsigned int length = (taken - offset) > 16u ? 16u : (taken - offset);
            if (reg_read(spi, 0x30u, data, (uint16_t)length) != 0) {
                emit("read_fail");
                break;
            }
            emit_hex(data, length);
        }
        emit("\n");
    }
    if (reg_read(spi, 0x6cu, lost, 2u) == 0)
        emit("exp=%s_lost count=%u\n", tag, ((unsigned int)lost[0] << 8) | (unsigned int)lost[1]);

    (void)reg_write(spi, REG_FIFO_CONFIG1, 0x00u);
    (void)reg_write(spi, REG_FIFO_CONFIG, 0x00u);   /* back to bypass */
    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    sleep_ms(5u);
    return bytes;
}

static void experiment_fifo(drv_spi_inst_t spi) {
    const unsigned int gyro_bytes = experiment_fifo_case(spi, 0x0cu, 0x02u, "E18_gyro_only");
    const unsigned int accel_bytes = experiment_fifo_case(spi, 0x03u, 0x01u, "E18_accel_only");
    emit("exp=E18_summary gyro_fifo_bytes=%u accel_fifo_bytes=%u\n", gyro_bytes, accel_bytes);
}

/* E20: smallest full scale (most sensitive) and slowest ODR, so any response at
 * all would stay visible in the sense registers for tens of milliseconds. */
static void experiment_most_sensitive(drv_spi_inst_t spi) {
    int16_t first[3] = {0, 0, 0}, accelerometer[3] = {0, 0, 0};
    int16_t previous[3] = {0, 0, 0};
    unsigned int changed = 0, i;

    (void)reg_write(spi, REG_GYRO_CONFIG0, 0xebu); /* +-15.625 dps, 12.5 Hz */
    (void)reg_write(spi, REG_ACCEL_CONFIG0, 0x06u);
    (void)reg_write(spi, REG_PWR_MGMT0, 0x0fu);
    sleep_ms(300u);
    emit("exp=E20_setup gyro_cfg_w=0xeb gyro_cfg_rb=0x%02x power_rb=0x%02x\n",
         reg_read_u8(spi, REG_GYRO_CONFIG0), reg_read_u8(spi, REG_PWR_MGMT0));

    (void)read_three(spi, REG_GYRO_DATA_X1, first);
    (void)read_three(spi, REG_ACCEL_DATA_X1, accelerometer);
    previous[0] = first[0];
    previous[1] = first[1];
    previous[2] = first[2];
    emit("exp=E20_first gyro=%d,%d,%d accel=%d,%d,%d\n", first[0], first[1], first[2],
         accelerometer[0], accelerometer[1], accelerometer[2]);

    for (i = 0u; i < 20u; ++i) {
        int16_t now[3] = {0, 0, 0};
        if (read_three(spi, REG_GYRO_DATA_X1, now) == 0
            && (now[0] != previous[0] || now[1] != previous[1] || now[2] != previous[2])) {
            ++changed;
            previous[0] = now[0];
            previous[1] = now[1];
            previous[2] = now[2];
        }
        sleep_ms(10u);
    }
    emit("exp=E20_change polls=20 changed=%u last=%d,%d,%d\n", changed, previous[0], previous[1],
         previous[2]);
    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
}

/* E21: the same static registers at several clocks and both SPI modes.  Only
 * values that must not change (ID, config block, gyro bytes) are compared. */
static void experiment_bus(int bus, int cs_pin) {
    static const struct {
        int mode;
        uint32_t hz;
    } cases[] = {
        {3, 100000u}, {3, 400000u}, {3, 1000000u}, {3, 2000000u}, {0, 400000u}, {0, 1000000u}};
    uint8_t reference[19];
    unsigned int have_reference = 0, consistent = 0, cases_run = 0, index;

    for (index = 0u; index < sizeof(cases) / sizeof(cases[0]); ++index) {
        drv_spi_inst_t spi = NULL;
        uint8_t frame[19];
        uint8_t id = 0;
        int ok;

        if (drv_spi_inst_create(bus, true, cases[index].mode, cases[index].hz, 8u, cs_pin,
                                SPI_HAL_DATA_LINE_1, &spi)
            != 0) {
            emit("exp=E21_case mode=%d hz=%" PRIu32 " open=fail\n", cases[index].mode,
                 cases[index].hz);
            continue;
        }
        (void)reg_write(spi, REG_BANK_SEL, 0u);
        ok = reg_read(spi, REG_WHO_AM_I, &id, 1u) == 0
             && reg_read(spi, 0x4cu, frame + 1, 12u) == 0
             && reg_read(spi, REG_GYRO_DATA_X1, frame + 13, 6u) == 0;
        frame[0] = id;
        if (!ok) {
            emit("exp=E21_case mode=%d hz=%" PRIu32 " read=fail\n", cases[index].mode,
                 cases[index].hz);
            drv_spi_inst_destroy(&spi);
            continue;
        }
        ++cases_run;
        if (!have_reference) {
            memcpy(reference, frame, sizeof(reference));
            have_reference = 1u;
            ++consistent;
        } else if (memcmp(reference, frame, sizeof(reference)) == 0) {
            ++consistent;
        }
        emit("exp=E21_case mode=%d hz=%" PRIu32 " id=0x%02x frame=", cases[index].mode,
             cases[index].hz, id);
        emit_hex(frame, 19u);
        emit("\n");
        drv_spi_inst_destroy(&spi);
        sleep_ms(5u);
    }
    emit("exp=E21_summary cases=%u consistent=%u reference=", cases_run, consistent);
    if (have_reference) emit_hex(reference, 19u);
    emit("\n");
}

static int gyro_probe2(int bus, int cs_pin, uint32_t bus_hz) {
    drv_spi_inst_t spi = NULL;

    emit("[gyro2] version=2 bus=%d cs=%d hz=%" PRIu32 "\n", bus, cs_pin, bus_hz);
    if (drv_spi_inst_create(bus, true, SPI_HAL_MODE_3, bus_hz, 8u, cs_pin, SPI_HAL_DATA_LINE_1,
                            &spi)
        != 0) {
        emit("exp=bus_open result=fail bus=%d cs=%d action=abort\n", bus, cs_pin);
        return 1;
    }

    (void)reg_write(spi, REG_BANK_SEL, 0u);
    (void)reg_write(spi, REG_DEVICE_CONFIG, 0x01u);
    sleep_ms(5u);
    emit("exp=E14_reset id=0x%02x intf0=0x%02x intf1=0x%02x power=0x%02x gyro_cfg=0x%02x "
         "accel_cfg=0x%02x status=0x%02x\n",
         reg_read_u8(spi, REG_WHO_AM_I), reg_read_u8(spi, REG_INTF_CONFIG0),
         reg_read_u8(spi, REG_INTF_CONFIG1), reg_read_u8(spi, REG_PWR_MGMT0),
         reg_read_u8(spi, REG_GYRO_CONFIG0), reg_read_u8(spi, REG_ACCEL_CONFIG0),
         reg_read_u8(spi, REG_INT_STATUS));

    experiment_snapshot(spi);
    experiment_read_consistency(spi, 0u, 0x30u);
    experiment_read_consistency(spi, 0u, 0x70u);
    experiment_read_consistency(spi, 1u, 0x70u);
    experiment_bank1_alias(spi);
    experiment_factory_st(spi);
    experiment_clock(spi);
    experiment_self_test(spi);
    experiment_fifo(spi);
    experiment_most_sensitive(spi);

    (void)reg_write(spi, REG_BANK_SEL, 0u);
    (void)reg_write(spi, REG_DEVICE_CONFIG, 0x01u);
    sleep_ms(5u);
    drv_spi_inst_destroy(&spi);

    experiment_bus(bus, cs_pin);
    emit("[gyro2] done\n");
    return 0;
}

/* E19: cold start.  Run this right after a real power cycle: no soft reset, no
 * bank 1 writes, just the minimum sequence a working part would need. */
/* Sample both sensors N times and report valid/change counts.  The gyro is
 * the subject; the accelerometer is the positive control for the same run. */
static void cold_sample(drv_spi_inst_t spi, unsigned int polls, const char* tag) {
    int16_t gyro_first[3] = {0, 0, 0}, accel_first[3] = {0, 0, 0};
    int16_t gyro_last[3] = {0, 0, 0}, accel_last[3] = {0, 0, 0};
    unsigned int gyro_distinct = 0, accel_distinct = 0;
    unsigned int gyro_valid = 0, accel_valid = 0, read_fail = 0, have_first = 0, i;

    for (i = 0u; i < polls; ++i) {
        int16_t g[3] = {0, 0, 0};
        int16_t a[3] = {0, 0, 0};
        if (read_three(spi, REG_GYRO_DATA_X1, g) != 0) {
            ++read_fail;
            continue;
        }
        (void)read_three(spi, REG_ACCEL_DATA_X1, a);
        if (!have_first) {
            memcpy(gyro_first, g, sizeof(gyro_first));
            memcpy(accel_first, a, sizeof(accel_first));
            have_first = 1u;
        } else {
            if (g[0] != gyro_first[0] || g[1] != gyro_first[1] || g[2] != gyro_first[2])
                ++gyro_distinct;
            if (a[0] != accel_first[0] || a[1] != accel_first[1] || a[2] != accel_first[2])
                ++accel_distinct;
        }
        if (!(g[0] == INT16_MIN && g[1] == INT16_MIN && g[2] == INT16_MIN)) ++gyro_valid;
        if (!(a[0] == INT16_MIN && a[1] == INT16_MIN && a[2] == INT16_MIN)) ++accel_valid;
        memcpy(gyro_last, g, sizeof(gyro_last));
        memcpy(accel_last, a, sizeof(accel_last));
        sleep_ms(10u);
    }
    emit("exp=%s polls=%u read_fail=%u gyro_valid=%u gyro_distinct=%u gyro=%d,%d,%d "
         "accel_valid=%u accel_distinct=%u accel=%d,%d,%d\n",
         tag, polls, read_fail, gyro_valid, gyro_distinct, gyro_last[0], gyro_last[1],
         gyro_last[2], accel_valid, accel_distinct, accel_last[0], accel_last[1], accel_last[2]);
}

/* E19: cold start.  Run this right after a real power cycle: no soft reset, no
 * bank 1 writes.  Phase 1 isolates the gyro; phase 2 enables both sensors so
 * the accelerometer can prove the part came up correctly at all. */
static int cold_probe(int bus, int cs_pin, uint32_t bus_hz) {
    drv_spi_inst_t spi = NULL;

    emit("[cold] no soft reset -- power-cycle the board immediately before running this\n");
    if (drv_spi_inst_create(bus, true, SPI_HAL_MODE_3, bus_hz, 8u, cs_pin, SPI_HAL_DATA_LINE_1,
                            &spi)
        != 0) {
        emit("exp=bus_open result=fail bus=%d cs=%d action=abort\n", bus, cs_pin);
        return 1;
    }

    (void)reg_write(spi, REG_BANK_SEL, 0u);
    emit("exp=E19_id id=0x%02x\n", reg_read_u8(spi, REG_WHO_AM_I));
    sleep_ms(100u);

    (void)reg_write(spi, REG_PWR_MGMT0, 0x0cu); /* gyro LN only, accel off */
    sleep_ms(200u);
    emit("exp=E19_power_gyro rb=0x%02x\n", reg_read_u8(spi, REG_PWR_MGMT0));
    cold_sample(spi, 50u, "E19_gyro_only");

    (void)reg_write(spi, REG_PWR_MGMT0, 0x0fu); /* both sensors: control phase */
    sleep_ms(200u);
    emit("exp=E19_power_both rb=0x%02x\n", reg_read_u8(spi, REG_PWR_MGMT0));
    cold_sample(spi, 50u, "E19_both");

    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    drv_spi_inst_destroy(&spi);
    emit("[cold] done\n");
    return 0;
}

/* E24: 10 Hz load step for the scope.  No current meter is available, so the
 * only way to see whether the gyro block draws current is the supply response. */
static int toggle_probe(int bus, int cs_pin, uint32_t bus_hz) {
    drv_spi_inst_t spi = NULL;
    uint64_t start;
    unsigned int i;

    emit("[toggle] PWR_MGMT0 0x00 <-> 0x0C every 100 ms, 50 cycles; trigger on VDD\n");
    if (drv_spi_inst_create(bus, true, SPI_HAL_MODE_3, bus_hz, 8u, cs_pin, SPI_HAL_DATA_LINE_1,
                            &spi)
        != 0) {
        emit("exp=bus_open result=fail bus=%d cs=%d action=abort\n", bus, cs_pin);
        return 1;
    }
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    sleep_ms(500u);
    start = mono_ns();
    for (i = 0u; i < 50u; ++i) {
        (void)reg_write(spi, REG_PWR_MGMT0, 0x0cu);
        emit("t_us=%" PRIu64 " event=on power_rb=0x%02x\n", (mono_ns() - start) / 1000ull,
             reg_read_u8(spi, REG_PWR_MGMT0));
        sleep_ms(100u);
        (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
        emit("t_us=%" PRIu64 " event=off power_rb=0x%02x\n", (mono_ns() - start) / 1000ull,
             reg_read_u8(spi, REG_PWR_MGMT0));
        sleep_ms(100u);
    }
    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    drv_spi_inst_destroy(&spi);
    emit("[toggle] done\n");
    return 0;
}

/* E23: route UI data ready to INT1 so a scope can answer "is there any data at
 * all" without trusting the sense registers.  Pin 4 must be reachable. */
static int int1_probe(int bus, int cs_pin, uint32_t bus_hz) {
    drv_spi_inst_t spi = NULL;
    unsigned int phase;

    emit("[int1] scope on pin 4 (INT1): gyro-only 15 s, then accel-only 10 s\n");
    if (drv_spi_inst_create(bus, true, SPI_HAL_MODE_3, bus_hz, 8u, cs_pin, SPI_HAL_DATA_LINE_1,
                            &spi)
        != 0) {
        emit("exp=bus_open result=fail bus=%d cs=%d action=abort\n", bus, cs_pin);
        return 1;
    }
    (void)reg_write(spi, REG_BANK_SEL, 0u);
    (void)reg_write(spi, REG_INT_CONFIG, 0x02u);   /* INT1 push-pull */
    (void)reg_write(spi, REG_INT_CONFIG1, 0x00u);  /* INT_ASYNC_RESET=0, DS-000440 12.6 */
    (void)reg_write(spi, REG_INT_SOURCE0, 0x08u);  /* UI_DRDY -> INT1 */
    emit("exp=E23_config int_config=0x%02x int_config1=0x%02x int_source0=0x%02x\n",
         reg_read_u8(spi, REG_INT_CONFIG), reg_read_u8(spi, REG_INT_CONFIG1),
         reg_read_u8(spi, REG_INT_SOURCE0));

    for (phase = 0u; phase < 2u; ++phase) {
        const uint8_t power = phase == 0u ? 0x0cu : 0x03u;
        const char* tag = phase == 0u ? "gyro_only" : "accel_only";
        const unsigned int polls = (phase == 0u ? 15u : 10u) * 200u;
        unsigned int hits = 0, i;
        uint8_t seen = 0;

        (void)reg_write(spi, REG_PWR_MGMT0, power);
        sleep_ms(100u);
        for (i = 0u; i < polls; ++i) {
            const uint8_t status = reg_read_u8(spi, REG_INT_STATUS);
            seen |= status;
            if ((status & 0x08u) != 0u) ++hits;
            sleep_ms(5u);
        }
        emit("exp=E23_%s power=0x%02x polls=%u data_rdy_hits=%u status_or=0x%02x\n", tag, power,
             polls, hits, seen);
    }
    (void)reg_write(spi, REG_PWR_MGMT0, 0x00u);
    drv_spi_inst_destroy(&spi);
    emit("[int1] done\n");
    return 0;
}

/* --- legacy initialization timing matrix --------------------------------- */

static int timing_probe(FILE* out) {
    const struct {
        const char* name;
        uint32_t pre, post;
    } cases[] = {
        {"baseline", 0, 50}, {"pre200", 200, 50},
        {"post200", 0, 200}, {"both200", 200, 200}, {"post1000", 0, 1000}};
    fprintf(out, "probe=IIM-42652 version=1 ready_timeout_ms=1000 poll_ms=5\n");
    fflush(out);
    for (unsigned int run = 0; run < 2; ++run) {
        for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            dart_iim42652_config_t c = {0};
            c.bus = 1; c.chip_select_pin = 14; c.bus_hz = 1000000; c.timeout_ms = 20;
            c.pre_access_delay_ms = cases[i].pre;
            c.post_enable_delay_ms = cases[i].post;
            c.ready_timeout_ms = 1000;
            dart_iim42652_t* sensor = NULL;
            dart_iim42652_diagnostic_t d;
            const int rc = dart_iim42652_init_with_diagnostic(&c, &sensor, &d);
            fprintf(out,
                "run=%u case=%s pre_ms=%u post_ms=%u result=%d stage=%s "
                "id_read=%u id=0x%02x start_ns=%" PRIu64 " first_access_ns=%" PRIu64
                " reset_ns=%" PRIu64 " configured_ns=%" PRIu64 " enabled_ns=%" PRIu64
                " first_read_ns=%" PRIu64 " ready_ns=%" PRIu64 " finished_ns=%" PRIu64
                " polls=%u config_read=%u intf0=0x%02x intf1=0x%02x power=0x%02x "
                "gyro_cfg=0x%02x accel_cfg=0x%02x status=0x%02x status_seen=0x%02x "
                "axis_config_read=%u sensor_config0=0x%02x accel=%d,%d,%d gyro=%d,%d,%d "
                "individual_read=%u gyro_individual=%d,%d,%d\n",
                run, cases[i].name, c.pre_access_delay_ms, c.post_enable_delay_ms,
                rc, dart_iim42652_stage_name(d.stage), d.device_id_read, d.observed_device_id,
                d.started_ns, d.first_access_ns, d.reset_ns, d.configured_ns, d.enabled_ns,
                d.first_read_ns, d.ready_ns, d.finished_ns, d.readiness_polls, d.registers_read,
                d.interface_config0, d.interface_config1, d.power_config, d.gyro_config,
                d.accel_config, d.interrupt_status, d.interrupt_status_seen,
                d.sensor_config_read, d.sensor_config0, d.last_accelerometer[0],
                d.last_accelerometer[1], d.last_accelerometer[2], d.last_gyroscope[0],
                d.last_gyroscope[1], d.last_gyroscope[2], d.individual_gyroscope_read,
                d.individual_gyroscope[0], d.individual_gyroscope[1], d.individual_gyroscope[2]);
            fflush(out);
            printf("imu_probe run=%u case=%s result=%d polls=%u\n", run, cases[i].name,
                rc, d.readiness_polls);
            fflush(stdout);
            dart_iim42652_deinit(&sensor);
        }
    }
    fprintf(out, "probe=complete\n");
    const int failed = ferror(out);
    if (fclose(out) != 0 || failed) return 3;
    return 0; /* Successful diagnostic execution; per-case results are in the log. */
}

int main(int argc, char** argv) {
    if (atexit(notify_launcher) != 0) return 2;
    int gyro_mode = 0, gyro2_mode = 0, cold_mode = 0, toggle_mode = 0, int1_mode = 0;
    const char* path = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--gyro") == 0)
            gyro_mode = 1;
        else if (strcmp(argv[i], "--gyro2") == 0)
            gyro2_mode = 1;
        else if (strcmp(argv[i], "--cold") == 0)
            cold_mode = 1;
        else if (strcmp(argv[i], "--toggle") == 0)
            toggle_mode = 1;
        else if (strcmp(argv[i], "--int1") == 0)
            int1_mode = 1;
        else if (path == NULL)
            path = argv[i];
    }
    if (path == NULL) path = "/sdcard/app/logs/imu_init_probe.log";

    if (gyro_mode || gyro2_mode || cold_mode || toggle_mode || int1_mode) {
        int rc;
        g_log = fopen(path, "w");
        if (g_log == NULL)
            emit("[gyro-probe] warn=log_open_failed path=%s (stdout only)\n", path);
        if (gyro_mode)
            rc = gyro_probe(1, 14, 1000000u);
        else if (gyro2_mode)
            rc = gyro_probe2(1, 14, 1000000u);
        else if (cold_mode)
            rc = cold_probe(1, 14, 1000000u);
        else if (toggle_mode)
            rc = toggle_probe(1, 14, 1000000u);
        else
            rc = int1_probe(1, 14, 1000000u);
        if (g_log != NULL) {
            (void)fflush(g_log);
            (void)fclose(g_log);
            g_log = NULL;
        }
        return rc;
    }

    FILE* out = fopen(path, "w");
    if (out == NULL) {
        perror(path);
        return 2;
    }
    return timing_probe(out);
}
