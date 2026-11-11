#define _POSIX_C_SOURCE 200809L

#include "camera_timing_config.h"
#include "camera/k230/internal.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "mpi_sensor_api.h"
#include "mpi_vicap_api.h"

/* This probe targets the previously verified Dart board wiring (CSI2). The
 * shared SDK's generated config can be changed by builds for other boards. */
enum { PROBE_CSI_PORT = 2 };
_Static_assert(SC038HGS_MIPI_CSI2_640X480_10BIT_LINEAR == 2,
               "Sensor mode ABI must match the known-good board executable");

/* Exclusive, finite board diagnostic. Only documented HTS/VTS are changed,
 * towards longer periods, then restored before the normal application runs.
 * No PLL, analogue, output-window or receiver-clock settings are written. */
typedef struct {
    FILE* log;
    dart_media_t* media;
    dart_camera_t* camera;
    uint32_t original_hts, original_vts;
    int timing_saved;
    int drop_saved, original_drop_enabled;
    k_vicap_drop_frame original_drop;
} Probe;

static uint64_t now_us(void) {
    struct timespec ts = {0};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0;
    return (uint64_t)ts.tv_sec * UINT64_C(1000000) + (uint64_t)ts.tv_nsec / 1000u;
}

static int checked(Probe* p, const char* operation, int rc) {
    fprintf(p->log, "op=call name=%s rc=%d errno=%d\n", operation, rc, rc ? errno : 0);
    return rc;
}

static int read16(Probe* p, uint32_t address, uint32_t* value) {
    uint32_t hi = 0, lo = 0;
    int rc = kd_mpi_sensor_reg_read(p->camera->sensor_fd, address, &hi);
    if (rc == 0)
        rc = kd_mpi_sensor_reg_read(p->camera->sensor_fd, address + 1u, &lo);
    if (rc == 0)
        *value = (hi << 8u) | lo;
    return rc;
}

static int sensor_snapshot(Probe* p, const char* stage) {
    static const uint32_t registers[] = {
        0x3107, 0x3108, 0x2100, 0x3031, 0x3037, 0x4b01, 0x4603,
        0x3222, 0x3225, 0x3223, 0x3282, 0x4501,
        0x3208, 0x3209, 0x320a, 0x320b, 0x320c, 0x320d, 0x320e, 0x320f,
        0x3e00, 0x3e01, 0x3e02,
    };
    int ok = 1;
    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); ++i) {
        uint32_t value = 0;
        const int rc = kd_mpi_sensor_reg_read(p->camera->sensor_fd, registers[i], &value);
        fprintf(p->log, "op=sensor stage=%s reg=0x%04x value=0x%02x rc=%d\n",
                stage, registers[i], value, rc);
        ok = ok && rc == 0;
    }
    return ok;
}

static void receiver_snapshot(Probe* p, const char* stage) {
    static const uint32_t vi[] = {
        0x0010, 0x0060, 0x007c, 0x0080, 0x0084, 0x0088, 0x008c, 0x0090, 0x0094,
        0x00ac, 0x1800, 0x1804, 0x1808, 0x180c, 0x1840, 0x1844, 0x1848,
        0x184c, 0x1880, 0x1884, 0x1888, 0x1890, 0x18e0, 0x18f0,
    };
    static const uint32_t mcm[] = {
        0x0400, 0x0404, 0x1200, 0x1204, 0x1300, 0x1600, 0x1604, 0x1618,
        0x1620, 0x16c0, 0x16c4, 0x16c8, 0x16d0, 0x16e8,
    };
    const int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        checked(p, "receiver_open", -1);
        return;
    }
    for (unsigned block = 0; block < 2; ++block) {
        const off_t base = block == 0 ? (off_t)0x90009000ULL : (off_t)0x90000000ULL;
        const uint32_t* offsets = block == 0 ? vi : mcm;
        const size_t count = block == 0 ? sizeof(vi) / sizeof(vi[0])
                                       : sizeof(mcm) / sizeof(mcm[0]);
        volatile uint32_t* mapping = mmap(NULL, 0x2000, PROT_READ, MAP_SHARED, fd, base);
        if (mapping == MAP_FAILED || mapping == NULL) {
            checked(p, "receiver_map", -1);
            continue;
        }
        if (block == 0 && !p->drop_saved) {
            const uint32_t mode = (mapping[0x00acu / 4u] >> 4u) & 3u;
            p->original_drop_enabled = (mapping[0x0094u / 4u] & 4u) != 0;
            p->original_drop.m = (uint8_t)mapping[0x008cu / 4u];
            p->original_drop.n = (uint8_t)mapping[0x0090u / 4u];
            p->original_drop.mode = mode == 1u ? VICAP_VCID_HDR_2FRAME
                                             : mode == 2u ? VICAP_VCID_HDR_3FRAME : VICAP_LINERA_MODE;
            p->drop_saved = mode == 1u || mode == 2u || !p->original_drop_enabled;
        }
        for (size_t i = 0; i < count; ++i)
            fprintf(p->log, "op=receiver stage=%s block=%s reg=0x%04x value=0x%08x\n",
                    stage, block == 0 ? "vi_csi2" : "mcm", offsets[i], mapping[offsets[i] / 4u]);
        checked(p, "receiver_unmap", munmap((void*)mapping, 0x2000));
    }
    close(fd);
}

static int set_timing(Probe* p, uint32_t hts, uint32_t vts) {
    if (hts < p->original_hts || hts > 0xfffu || vts < p->original_vts || vts > 0xfff0u)
        return 0;
    const int fd = p->camera->sensor_fd;
    uint32_t high = 0;
    if (checked(p, "hts_high_read", kd_mpi_sensor_reg_read(fd, 0x320c, &high)) != 0)
        return 0;
    if (checked(p, "group_begin", kd_mpi_sensor_reg_write(fd, 0x3800, 0x00)) != 0)
        return 0;
    const uint32_t addresses[] = {0x320c, 0x320d, 0x320e, 0x320f};
    const uint32_t values[] = {(high & 0xf0u) | (hts >> 8u), hts & 0xffu, vts >> 8u, vts & 0xffu};
    int rc = 0;
    for (size_t i = 0; i < 4 && rc == 0; ++i)
        rc = checked(p, "timing_write", kd_mpi_sensor_reg_write(fd, addresses[i], values[i]));
    const int end_rc = checked(p, "group_end", kd_mpi_sensor_reg_write(fd, 0x3800, 0x10));
    if (rc != 0 || end_rc != 0)
        return 0;
    if (checked(p, "group_launch", kd_mpi_sensor_reg_write(fd, 0x3800, 0x60)) != 0)
        return 0;
    /* Allow the documented grouped update and buffered old frames to settle. */
    const struct timespec delay = {.tv_sec = 0, .tv_nsec = 150000000};
    nanosleep(&delay, NULL);
    uint32_t actual_hts = 0, actual_vts = 0;
    rc = read16(p, 0x320c, &actual_hts);
    if (rc == 0)
        rc = read16(p, 0x320e, &actual_vts);
    actual_hts &= 0xfffu;
    fprintf(p->log, "op=timing requested_hts=%u requested_vts=%u hts=%u vts=%u rc=%d\n",
            hts, vts, actual_hts, actual_vts, rc);
    return rc == 0 && hts == actual_hts && vts == actual_vts;
}

static int measure(Probe* p, const char* name, int convert) {
    enum { WARMUP = 24, FRAMES = 192, TRACE = 8 };
    struct { uint64_t received, pts; uint32_t reference; } trace[TRACE] = {0};
    uint64_t first = 0, last = 0, previous_pts = 0, first_pts = 0;
    uint64_t work_sum = 0, work_max = 0, pts_max_delta = 0;
    uint32_t nonincreasing_pts = 0;
    uint32_t hts = 0, vts = 0;
    if (read16(p, 0x320c, &hts) != 0 || read16(p, 0x320e, &vts) != 0)
        return 0;
    hts &= 0xfffu;
    fprintf(p->log, "op=case_start case=%s convert=%d hts=%u vts=%u\n", name, convert, hts, vts);
    fflush(p->log);
    for (unsigned i = 0; i < WARMUP + FRAMES; ++i) {
        const uint64_t begin = now_us();
        uint64_t pts = 0;
        uint32_t reference = 0;
        int rc = 0;
        if (convert) {
            dart_camera_frame_t frame = {0};
            rc = dart_camera_acquire_frame(p->camera, 500, &frame);
            if (rc == 0) {
                pts = frame.source_pts;
                rc = dart_camera_release_frame(p->camera, &frame);
            }
        } else {
            k_video_frame_info frame = {0};
            rc = kd_mpi_vicap_dump_frame(VICAP_DEV_ID_0, VICAP_CHN_ID_0,
                                        VICAP_DUMP_YUV444, &frame, 500);
            if (rc == 0) {
                pts = frame.v_frame.pts;
                reference = frame.v_frame.time_ref;
                rc = kd_mpi_vicap_dump_release(VICAP_DEV_ID_0, VICAP_CHN_ID_0, &frame);
            }
        }
        const uint64_t received = now_us();
        if (rc != 0) {
            checked(p, "capture_or_release", rc);
            return 0;
        }
        if (i < WARMUP)
            continue;
        const unsigned index = i - WARMUP;
        const uint64_t work = received - begin;
        work_sum += work;
        if (work > work_max) work_max = work;
        if (index == 0) {
            first = received;
            first_pts = pts;
        } else if (pts > previous_pts) {
            const uint64_t delta = pts - previous_pts;
            if (delta > pts_max_delta) pts_max_delta = delta;
        } else {
            ++nonincreasing_pts;
        }
        previous_pts = pts;
        last = received;
        if (index < TRACE) {
            trace[index].received = received;
            trace[index].pts = pts;
            trace[index].reference = reference;
        }
    }
    if (last <= first)
        return 0;
    const double fps = (FRAMES - 1) * 1000000.0 / (double)(last - first);
    fprintf(p->log, "op=result case=%s frames=%u span_us=%" PRIu64 " fps=%.6f "
            "mean_step_us=%" PRIu64 " max_step_us=%" PRIu64 " hts=%u vts=%u "
            "inferred_pixel_clock_hz=%.0f pts_span_raw=%" PRIu64 " pts_max_delta_raw=%" PRIu64
            " pts_nonincreasing=%u pts_timebase_validated=0\n",
            name, FRAMES, last - first, fps, work_sum / FRAMES, work_max, hts, vts,
            fps * hts * vts, previous_pts - first_pts, pts_max_delta, nonincreasing_pts);
    for (unsigned i = 0; i < TRACE; ++i)
        fprintf(p->log, "op=sample case=%s index=%u received_us=%" PRIu64 " pts_raw=%" PRIu64
                " time_ref=%u\n", name, i, trace[i].received, trace[i].pts, trace[i].reference);
    fflush(p->log);
    return nonincreasing_pts == 0;
}

static int run(Probe* p) {
    dart_camera_config_t config = {0};
    dart_camera_config_init(&config);
    config.csi_port = PROBE_CSI_PORT;
    config.channel_buffers = 6;
    config.max_outstanding_frames = 4;
    config.initial_exposure_lines = 250;
    if (checked(p, "media_create", dart_media_create(&p->media)) != 0
        || checked(p, "camera_create", dart_camera_create(&p->camera)) != 0
        || checked(p, "camera_init", dart_camera_init(p->camera, p->media, &config)) != 0
        || checked(p, "camera_start", dart_camera_start(p->camera)) != 0)
        return 0;
    if (!sensor_snapshot(p, "baseline")) return 0;
    receiver_snapshot(p, "baseline");
    if (read16(p, 0x320c, &p->original_hts) != 0
        || read16(p, 0x320e, &p->original_vts) != 0)
        return 0;
    p->original_hts &= 0xfffu;
    p->timing_saved = 1;
    /* Refuse timing experiments with an unexpected output mode or chip. */
    uint32_t id = 0;
    if (read16(p, 0x3107, &id) != 0 || id != 0x1a79u
        || p->camera->config.output_width != 640 || p->camera->config.output_height != 480
        || p->original_hts != 750 || p->original_vts != 600)
        return 0;
    if (!measure(p, "gray_baseline", 1) || !measure(p, "raw_dump_baseline", 0))
        return 0;
    if (!p->drop_saved) return 0;
    k_vicap_drop_frame drop = {0};
    if (checked(p, "disable_vi_drop", kd_mpi_vicap_set_vi_drop_frame(VICAP_CSI2, &drop, K_FALSE)) != 0)
        return 0;
    receiver_snapshot(p, "drop_disabled");
    if (!measure(p, "raw_drop_disabled", 0)) return 0;
    if (!set_timing(p, 750, 1200) || !measure(p, "vts_double", 0)) return 0;
    if (!set_timing(p, 1500, 600) || !measure(p, "hts_double", 0)) return 0;
    if (!set_timing(p, p->original_hts, p->original_vts)
        || !measure(p, "raw_restored", 0))
        return 0;
    sensor_snapshot(p, "restored");
    receiver_snapshot(p, "restored");
    return 1;
}

static int cleanup(Probe* p) {
    int ok = 1;
    if (p->camera != NULL) {
        if (p->timing_saved && !set_timing(p, p->original_hts, p->original_vts))
            ok = 0;
        if (p->drop_saved && checked(p, "restore_vi_drop", kd_mpi_vicap_set_vi_drop_frame(
                VICAP_CSI2, &p->original_drop, p->original_drop_enabled ? K_TRUE : K_FALSE)) != 0)
            ok = 0;
        /* init() rolls back to EMPTY on sensor-probe failure. Deinit rejects
         * EMPTY, but destroy and media_destroy must still release ownership. */
        if (p->camera->state != DART_CAMERA_STATE_EMPTY
            && checked(p, "camera_deinit", dart_camera_deinit(p->camera)) != 0)
            return 0;
        if (checked(p, "camera_destroy", dart_camera_destroy(p->camera)) != 0)
            return 0;
        p->camera = NULL;
    }
    if (p->media != NULL) {
        if (checked(p, "media_destroy", dart_media_destroy(p->media)) != 0)
            return 0;
        p->media = NULL;
    }
    return ok;
}

static void notify_launcher_exit(void) {
    const char* path = getenv("DART_EXIT_MARKER");
    if (path == NULL || path[0] == '\0') return;
    FILE* marker = fopen(path, "w");
    if (marker == NULL) return;
    fprintf(marker, "%ld\n", (long)getpid());
    fflush(marker);
    fsync(fileno(marker));
    fclose(marker);
}

int main(void) {
    Probe probe = {0};
    probe.log = fopen("/data/app/logs/camera_timing_probe.log", "a");
    if (probe.log == NULL) {
        notify_launcher_exit();
        return 1;
    }
    fprintf(probe.log, "op=start version=3 mono_us=%" PRIu64 " csi_port=%u exposure_lines=250 "
            "record=0 pwm=0 writes=longer_hts_vts_only\n", now_us(), PROBE_CSI_PORT);
    const int ok = run(&probe);
    const int cleaned = cleanup(&probe);
    fprintf(probe.log, "op=complete result=%s cleanup=%s mono_us=%" PRIu64 "\n",
            ok ? "ok" : "failed", cleaned ? "ok" : "failed", now_us());
    fflush(probe.log);
    fsync(fileno(probe.log));
    fclose(probe.log);
    notify_launcher_exit();
    return ok && cleaned ? 0 : 1;
}
