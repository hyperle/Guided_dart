#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "drv_spi.h"
#include "spi/iim42652.h"

static struct {
    uint8_t regs[2][128], bank, id, axis_disable;
    int create_failed, fail_read_at, fail_write_at, corrupt_config, change_reset_id;
    int live, destroys, reads, writes, sample_reads, interrupt_sleeps, interrupted;
    int fail_sleep, fail_clock, never_ready, invalid_one_axis;
    int allow_raw_probe, all_reads_ff, all_sample_ff, all_sample_zero;
    int fail_raw_probe_at, raw_probe_reads;
    int require_external_clock, corrupt_pin, corrupt_rtc, corrupt_int1, corrupt_bank, warm_on;
    uint64_t now, reset_at, powered_at, ready_after_ns, interrupt_min_ns, gyro_off_at;
} fake;
static int instance;

/* A controlled monotonic clock lets us check deadlines and interrupted waits
 * without turning the timeout tests into multi-second wall-clock delays. */
int __wrap_clock_gettime(clockid_t id, struct timespec* now) {
    assert(id == CLOCK_MONOTONIC);
    if (fake.fail_clock) { errno = EIO; return -1; }
    now->tv_sec = (time_t)(fake.now / 1000000000u);
    now->tv_nsec = (long)(fake.now % 1000000000u);
    return 0;
}

int __wrap_nanosleep(const struct timespec* request, struct timespec* remaining) {
    const uint64_t ns = (uint64_t)request->tv_sec * 1000000000u + request->tv_nsec;
    if (fake.fail_sleep) { errno = EIO; return -1; }
    if (fake.interrupt_sleeps && ns > 0u && ns >= fake.interrupt_min_ns) {
        const uint64_t left = ns - ns / 2u;
        fake.now += ns / 2u;
        remaining->tv_sec = (time_t)(left / 1000000000u);
        remaining->tv_nsec = (long)(left % 1000000000u);
        --fake.interrupt_sleeps;
        ++fake.interrupted;
        errno = EINTR;
        return -1;
    }
    fake.now += ns;
    return 0;
}

int drv_spi_inst_create(
    int bus, bool active_low, int mode, uint32_t baudrate, uint8_t bits, int cs, uint8_t lines,
    drv_spi_inst_t* out) {
    assert(bus == 1 && active_low && mode == SPI_HAL_MODE_3 && baudrate == 1000000);
    assert(bits == 8 && cs == 14 && lines == SPI_HAL_DATA_LINE_1);
    *out = (drv_spi_inst_t)&instance; /* SDK also leaves this pointer on failure. */
    if (fake.create_failed) return -1;
    fake.live = 1;
    return 0;
}

void drv_spi_inst_destroy(drv_spi_inst_t* handle) {
    assert(fake.live && *handle == (drv_spi_inst_t)&instance);
    ++fake.destroys;
    fake.live = 0;
}

static void check_reset_wait(void) {
    if (fake.reset_at) assert(fake.now - fake.reset_at >= 1000000u);
}

static void sample_bytes(uint8_t bytes[14]) {
    /* 26 C, +1 g X, -0.5 g Y, +16.4 dps X, -32.8 dps Z. */
    const uint8_t valid[] = {0, 132, 8, 0, 0xfc, 0, 0, 0, 1, 13, 0, 0, 0xfd, 0xe6};
    memcpy(bytes, valid, sizeof(valid));
    if (fake.never_ready || fake.now - fake.powered_at < fake.ready_after_ns
        || (fake.require_external_clock && ((fake.regs[1][0x7b] & 6) != 4
                                           || (fake.regs[0][0x4d] & 7) != 5))) {
        for (int axis = 0; axis < 3; ++axis) {
            bytes[8 + axis * 2] = 0x80;
            bytes[9 + axis * 2] = 0;
        }
    } else if (fake.invalid_one_axis) {
        bytes[10] = 0x80;
        bytes[11] = 0;
    }
    if (fake.all_sample_ff) memset(bytes, 0xff, 14);
    if (fake.all_sample_zero) memset(bytes, 0, 14);
}

int drv_spi_transfer(
    drv_spi_inst_t handle, const void* tx, void* rx, size_t length, bool cs_change) {
    assert(fake.live && handle == (drv_spi_inst_t)&instance && cs_change);
    if (++fake.reads == fake.fail_read_at) return -1;
    check_reset_wait();
    const uint8_t command = ((const uint8_t*)tx)[0];
    assert(command & 0x80);
    const uint8_t reg = command & 0x7f;
    uint8_t* out = (uint8_t*)rx + 1;
    memset(rx, 0, length);
    if (fake.all_reads_ff) {
        memset(out, 0xff, length - 1);
        return (int)length;
    }
    if (reg == 0x76) {
        assert(length == 2);
        out[0] = fake.corrupt_bank ? 0xff : fake.bank;
    } else if (reg == 0x75) {
        assert(fake.bank == 0 && length == 2);
        out[0] = fake.change_reset_id && fake.reset_at ? 0xff : fake.id;
    } else if (fake.bank == 0 && reg == 0x2d) {
        /* DATA_RDY can be true even when every gyro axis is invalid. */
        out[0] = fake.regs[0][0x2d] | (fake.powered_at ? 0x08 : 0);
        fake.regs[0][0x2d] = 0; /* Read-to-clear. */
    } else if (fake.bank == 0 && (reg == 0x1d || (reg >= 0x25 && reg <= 0x2a))) {
        uint8_t bytes[14];
        if (!fake.powered_at) {
            assert(fake.allow_raw_probe && reg == 0x1d && length == 15);
        } else {
            assert(fake.now - fake.powered_at >= 50000000u);
        }
        if (reg == 0x1d && ++fake.raw_probe_reads == fake.fail_raw_probe_at) return -1;
        sample_bytes(bytes);
        if (reg == 0x1d) {
            assert(length == 15);
            ++fake.sample_reads;
            memcpy(out, bytes, 14);
        } else {
            assert(length == 2);
            out[0] = bytes[reg - 0x1d];
        }
    } else {
        assert(reg + length - 1 <= 128);
        memcpy(out, fake.regs[fake.bank] + reg, length - 1);
        if (fake.corrupt_config && reg == 0x4e) out[0] = 0;
    }
    return (int)length;
}

int drv_spi_write(drv_spi_inst_t handle, const void* tx, size_t length, bool change) {
    assert(fake.live && handle == (drv_spi_inst_t)&instance && change && length == 2);
    if (++fake.writes == fake.fail_write_at) return -1;
    check_reset_wait();
    const uint8_t* bytes = tx;
    /* Initialization must complete every configuration write before enable. */
    assert(!fake.powered_at);
    if (bytes[0] == 0x76) {
        assert(bytes[1] <= 1);
        fake.bank = bytes[1];
    } else if (bytes[0] == 0x11) {
        assert(fake.bank == 0 && bytes[1] == 1);
        memset(fake.regs, 0, sizeof(fake.regs));
        fake.regs[0][0x14] = 0xa0;
        fake.regs[0][0x4c] = 0x30;
        fake.regs[0][0x4d] = 0x91;
        fake.regs[0][0x56] = 0xfe;
        fake.regs[0][0x57] = 0xab;
        fake.regs[0][0x2d] = 0x10;
        fake.regs[1][0x03] = 0x80 | fake.axis_disable;
        fake.regs[1][0x7b] = 0xab;
        fake.reset_at = fake.now;
    } else if (fake.bank == 1) {
        assert(bytes[0] == 0x7b && bytes[1] == 0xad);
        assert((fake.regs[0][0x4e] & 0x0f) == 0);
        if (!fake.corrupt_pin)
            fake.regs[1][bytes[0]] = bytes[1];
    } else if (bytes[0] == 0x14) {
        assert(bytes[1] == 0xa3 && (fake.regs[0][0x4e] & 0x0f) == 0);
        if (!fake.corrupt_int1)
            fake.regs[0][bytes[0]] = bytes[1];
    } else if (bytes[0] == 0x4d) {
        assert(bytes[1] == 0x95 && (fake.regs[0][0x4e] & 0x0f) == 0);
        if (!fake.corrupt_rtc)
            fake.regs[0][bytes[0]] = bytes[1];
    } else if (bytes[0] == 0x56 || bytes[0] == 0x57) {
        assert(fake.bank == 0 && (fake.regs[0][0x4e] & 0x0f) == 0);
        assert(bytes[1] == (bytes[0] == 0x56 ? 0x0a : 0xa8));
        fake.regs[0][bytes[0]] = bytes[1];
    } else {
        assert(fake.bank == 0);
        assert(bytes[0] == 0x4f || bytes[0] == 0x50 || bytes[0] == 0x4e);
        if (bytes[0] == 0x4e) {
            assert(bytes[1] == 0x0f || (bytes[1] & 0x0f) == 0);
            if (bytes[1] == 0x0f) {
                assert(fake.regs[0][0x4f] == 6 && fake.regs[0][0x50] == 6);
                assert(fake.now - fake.gyro_off_at > 150000000u);
                assert(fake.regs[0][0x14] == 0xa3);
                assert(fake.regs[0][0x56] == 0x0a && fake.regs[0][0x57] == 0xa8);
                fake.powered_at = fake.now;
            } else {
                if (fake.warm_on)
                    assert(fake.now >= 1050000000u);
                fake.gyro_off_at = fake.now;
            }
        }
        fake.regs[0][bytes[0]] = bytes[1];
    }
    return (int)length;
}

static dart_iim42652_config_t config(void) {
    dart_iim42652_config_t c = {0};
    c.bus = 1;
    c.chip_select_pin = 14;
    c.bus_hz = 1000000;
    c.timeout_ms = 20;
    c.ready_timeout_ms = 20;
    return c;
}

static void clear_fake(void) {
    assert(!fake.live);
    memset(&fake, 0, sizeof(fake));
    fake.id = 0x6f;
    fake.now = 1000000000u;
}

static int initialize(dart_iim42652_config_t c, dart_iim42652_diagnostic_t* d) {
    dart_iim42652_t* sensor = NULL;
    fake.allow_raw_probe = c.skip_device_id_check;
    const int result = dart_iim42652_init_with_diagnostic(&c, &sensor, d);
    assert(d->result == result);
    if (result == DART_IIM42652_OK) {
        assert(sensor && fake.live);
        assert(d->ready_ns && d->stage == DART_IIM42652_STAGE_READY);
        assert(d->sensor_config_read && d->sensor_config0 == 0x80);
        assert(d->registers_read && d->power_config == 0x0f && d->gyro_config == 6);
        dart_iim42652_sample_t sample;
        assert(dart_iim42652_read_sample(sensor, &sample) == DART_IIM42652_OK);
        assert(sample.timestamp_ns > 0 && sample.flags == 0);
        assert(sample.accelerometer[0] == 2048 && sample.accelerometer[1] == -1024);
        assert(fabsf(sample.acceleration_m_s2[0] - 9.80665f) < 0.0001f);
        assert(fabsf(sample.angular_velocity_rad_s[2] + 0.57253f) < 0.001f);
        assert(fabsf(sample.temperature_c - 26.0f) < 0.01f);
        /* DS-000440 p62: -32768 is "no sample since reset", not saturation.
         * It must be reported as no data, never scaled into a rate. */
        fake.invalid_one_axis = 1;
        assert(dart_iim42652_read_sample(sensor, &sample) == DART_IIM42652_NO_DATA);
        assert(sample.flags & DART_IIM42652_SAMPLE_GYRO_NO_DATA);
        assert(!(sample.flags & DART_IIM42652_SAMPLE_GYRO_SATURATED));
        assert(sample.gyroscope[1] == INT16_MIN && sample.gyroscope[0] == 269);
        assert(sample.angular_velocity_rad_s[0] == 0.0f
               && sample.angular_velocity_rad_s[1] == 0.0f
               && sample.angular_velocity_rad_s[2] == 0.0f);
        fake.invalid_one_axis = 0;
        assert(dart_iim42652_read_sample(sensor, &sample) == DART_IIM42652_OK);
        assert(sample.flags == 0);
        if (c.skip_device_id_check) {
            /* A valid first sample cannot hide a later disconnected bus. */
            fake.all_sample_ff = 1;
            assert(dart_iim42652_read_sample(sensor, &sample) == DART_IIM42652_BUS_ERROR);
            fake.all_sample_ff = 0;
            fake.all_sample_zero = 1;
            assert(dart_iim42652_read_sample(sensor, &sample) == DART_IIM42652_BUS_ERROR);
            fake.all_sample_zero = 0;
        }
        dart_iim42652_deinit(&sensor);
        dart_iim42652_deinit(&sensor);
    }
    assert(!sensor && !fake.live);
    return result;
}

int main(void) {
    dart_iim42652_diagnostic_t d;
    clear_fake();
    assert(initialize(config(), &d) == DART_IIM42652_OK);
    assert(d.first_read_ns - d.enabled_ns == 50000000u);
    assert(d.interrupt_status_seen == 0x18 && d.interrupt_status == 0x08);
    assert(d.individual_gyroscope_read && d.individual_gyroscope[2] == -538);
    /* Exclude the consumer's three sample reads: valid, no-data, valid. */
    const int normal_reads = fake.reads - 3;
    const int normal_writes = fake.writes;

    clear_fake(); fake.corrupt_bank = 1;
    assert(initialize(config(), &d) == DART_IIM42652_BUS_ERROR);
    assert(!fake.powered_at && fake.bank == 0);

    /* Both CLKIN mux and RTC_MODE are required, and all reserved fields survive.
     * A successful transfer without matching readback must never enable sensors. */
    dart_iim42652_config_t external = config();
    external.use_external_clock = 1;
    clear_fake(); fake.require_external_clock = 1;
    assert(initialize(external, &d) == DART_IIM42652_OK);
    assert(d.external_clock_config_read && d.interface_config5 == 0xad);
    assert(d.interface_config1 == 0x95 && d.first_read_ns - d.enabled_ns == 50000000u);
    /* Three consumer sample reads are appended after initialization. */
    const int external_reads = fake.reads - 3, external_writes = fake.writes;
    clear_fake(); fake.require_external_clock = 1; fake.warm_on = 1; fake.regs[0][0x4e] = 0x0f;
    assert(initialize(external, &d) == DART_IIM42652_OK);
    for (int reg = 0; reg < 2; ++reg) {
        clear_fake(); fake.corrupt_pin = reg == 0; fake.corrupt_rtc = reg == 1;
        assert(initialize(external, &d) == DART_IIM42652_CONFIGURATION_MISMATCH);
        assert(!fake.powered_at && fake.bank == 0);
    }
    for (int read = 1; read <= external_reads; ++read) {
        clear_fake(); fake.require_external_clock = 1; fake.fail_read_at = read;
        assert(initialize(external, &d)
            == (read <= external_reads - 6 ? DART_IIM42652_BUS_ERROR : DART_IIM42652_OK));
        assert(fake.destroys == 1 && fake.bank == 0);
    }
    for (int write = 1; write <= external_writes; ++write) {
        clear_fake(); fake.fail_write_at = write;
        assert(initialize(external, &d) == DART_IIM42652_BUS_ERROR);
        assert(fake.destroys == 1);
    }
    clear_fake(); external.use_external_clock = 2;
    assert(initialize(external, &d) == DART_IIM42652_INVALID_ARGUMENT);
    assert(!fake.reads && !fake.writes);

    /* All mandatory transfer failures release the handle. Optional byte
     * diagnostics cannot turn success into failure. Bank is restored on a
     * failed Bank 1 read. */
    for (int read = 1; read <= normal_reads; ++read) {
        clear_fake();
        fake.fail_read_at = read;
        const int rc = initialize(config(), &d);
        assert(rc == (read <= normal_reads - 6 ? DART_IIM42652_BUS_ERROR : DART_IIM42652_OK));
        assert(fake.destroys == 1 && fake.bank == 0);
    }
    for (int write = 1; write <= normal_writes; ++write) {
        clear_fake();
        fake.fail_write_at = write;
        assert(initialize(config(), &d) == DART_IIM42652_BUS_ERROR);
        assert(fake.destroys == 1);
    }
    clear_fake(); fake.create_failed = 1;
    assert(initialize(config(), &d) == DART_IIM42652_BUS_ERROR && fake.destroys == 0);
    const uint8_t unsupported[] = {0, 0xff, 0x46, 0x47};
    for (unsigned int i = 0; i < sizeof(unsupported); ++i) {
        clear_fake(); fake.id = unsupported[i];
        assert(initialize(config(), &d) == DART_IIM42652_UNEXPECTED_DEVICE_ID);
        assert(d.device_id_read && d.observed_device_id == unsupported[i]);
    }
    clear_fake(); fake.change_reset_id = 1;
    assert(initialize(config(), &d) == DART_IIM42652_UNEXPECTED_DEVICE_ID);
    clear_fake(); fake.corrupt_int1 = 1;
    assert(initialize(config(), &d) == DART_IIM42652_CONFIGURATION_MISMATCH);
    assert(!fake.powered_at);
    clear_fake(); fake.corrupt_config = 1;
    assert(initialize(config(), &d) == DART_IIM42652_CONFIGURATION_MISMATCH);
    clear_fake(); fake.axis_disable = 0x08;
    assert(initialize(config(), &d) == DART_IIM42652_CONFIGURATION_MISMATCH);
    assert(d.sensor_config_read && d.sensor_config0 == 0x88 && !fake.powered_at);

    clear_fake(); fake.never_ready = 1;
    assert(initialize(config(), &d) == DART_IIM42652_READY_TIMEOUT);
    assert(d.stage == DART_IIM42652_STAGE_WAIT_DATA && !d.ready_ns);
    assert(d.readiness_polls == 5 && d.finished_ns - d.first_read_ns == 20000000u);
    assert(d.last_gyroscope[0] == INT16_MIN && d.individual_gyroscope[2] == INT16_MIN);
    assert(d.interrupt_status_seen == 0x18); /* Status bits do not imply valid gyro. */
    clear_fake(); fake.invalid_one_axis = 1;
    assert(initialize(config(), &d) == DART_IIM42652_READY_TIMEOUT);
    clear_fake(); fake.ready_after_ns = 65000000u;
    assert(initialize(config(), &d) == DART_IIM42652_OK);
    assert(d.readiness_polls == 4 && d.ready_ns - d.enabled_ns == 65000000u);

    clear_fake(); fake.interrupt_sleeps = 3;
    dart_iim42652_config_t c = config();
    c.pre_access_delay_ms = 200;
    c.post_enable_delay_ms = 200;
    assert(initialize(c, &d) == DART_IIM42652_OK);
    assert(fake.interrupted == 3);
    assert(d.first_access_ns - d.started_ns == 200000000u);
    assert(d.first_read_ns - d.enabled_ns == 200000000u);
    clear_fake(); fake.interrupt_sleeps = 1; fake.interrupt_min_ns = 50000000u;
    assert(initialize(config(), &d) == DART_IIM42652_OK);
    assert(fake.interrupted == 1 && d.first_read_ns - d.enabled_ns == 50000000u);
    clear_fake(); fake.fail_sleep = 1;
    assert(initialize(config(), &d) == DART_IIM42652_CLOCK_ERROR);
    clear_fake(); fake.fail_clock = 1;
    assert(initialize(config(), &d) == DART_IIM42652_CLOCK_ERROR && fake.destroys == 0);
    clear_fake(); c = config(); c.post_enable_delay_ms = 1;
    assert(initialize(c, &d) == DART_IIM42652_INVALID_ARGUMENT);
    assert(!fake.reads && !fake.writes);
    assert(strcmp(dart_iim42652_device_name(0x6f), "IIM-42652") == 0);
    assert(strcmp(dart_iim42652_device_name(0x46), "unknown") == 0);

    /* Bypass both identity comparisons, but keep the real IDs and raw bytes. */
    c = config(); c.skip_device_id_check = 1;
    for (unsigned int i = 0; i < sizeof(unsupported); ++i) {
        clear_fake(); fake.id = unsupported[i];
        assert(initialize(c, &d) == DART_IIM42652_OK);
        assert(d.device_id_check_skipped && d.observed_device_id == unsupported[i]);
        assert(d.reset_device_id_read && d.reset_device_id == unsupported[i]);
        assert(d.raw_probe_read && d.raw_probe_stage == DART_IIM42652_STAGE_WAIT_DATA);
        assert(d.raw_probe[2] == 8 && d.raw_probe[8] == 1 && d.raw_probe[13] == 0xe6);
    }
    clear_fake(); fake.change_reset_id = 1;
    assert(initialize(c, &d) == DART_IIM42652_OK);
    assert(d.observed_device_id == 0x6f && d.reset_device_id == 0xff);

    /* ID transport errors still fail, after the direct diagnostic burst. */
    for (int read = 3; read <= 4; ++read) {
        clear_fake(); fake.fail_read_at = read;
        assert(initialize(c, &d) == DART_IIM42652_BUS_ERROR);
        assert(d.raw_probe_read && d.raw_probe_stage == DART_IIM42652_STAGE_IDENTIFY);
        assert(d.device_id_read == (read == 4) && !d.reset_device_id_read);
    }
    clear_fake(); fake.fail_raw_probe_at = 1;
    assert(initialize(c, &d) == DART_IIM42652_OK); /* Optional probe is best effort. */
    clear_fake(); fake.corrupt_config = 1;
    assert(initialize(c, &d) == DART_IIM42652_CONFIGURATION_MISMATCH);
    assert(d.raw_probe_read && d.raw_probe_stage == DART_IIM42652_STAGE_VERIFY);
    clear_fake(); fake.corrupt_config = 1; fake.fail_raw_probe_at = 2;
    assert(initialize(c, &d) == DART_IIM42652_CONFIGURATION_MISMATCH);
    assert(d.raw_probe_read && d.raw_probe_stage == DART_IIM42652_STAGE_IDENTIFY);
    clear_fake(); fake.axis_disable = 0x08;
    assert(initialize(c, &d) == DART_IIM42652_CONFIGURATION_MISMATCH);
    assert(d.raw_probe_read && !fake.powered_at);

    /* A floating bus is observable, never promoted to a usable device. */
    clear_fake(); fake.all_reads_ff = 1;
    assert(initialize(c, &d) == DART_IIM42652_BUS_ERROR);
    assert(d.device_id_check_skipped && !d.device_id_read);
    assert(!d.reset_device_id_read && !d.raw_probe_read && !d.ready_ns);
    assert(d.stage == DART_IIM42652_STAGE_IDENTIFY);
    for (int bypass = 0; bypass <= 1; ++bypass) {
        c.skip_device_id_check = (uint8_t)bypass;
        for (int zero = 0; zero <= 1; ++zero) {
            clear_fake(); fake.all_sample_zero = zero; fake.all_sample_ff = !zero;
            assert(initialize(c, &d) == DART_IIM42652_READY_TIMEOUT);
            assert(!d.ready_ns && d.readiness_polls == 5);
            assert(d.raw_probe_read == bypass);
            if (bypass) {
                for (unsigned int i = 0; i < sizeof(d.raw_probe); ++i)
                    assert(d.raw_probe[i] == (zero ? 0 : 0xff));
            }
        }
    }
    clear_fake(); c.skip_device_id_check = 2;
    assert(initialize(c, &d) == DART_IIM42652_INVALID_ARGUMENT);
    assert(!fake.reads && !fake.writes);
    return 0;
}
