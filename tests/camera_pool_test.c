#include <assert.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>

#include "camera/k230/internal.h"
#include "mpi_sensor_api.h"
#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_vicap_api.h"

enum {
    POOL_ID = 7,
    BUFFERS = 4,
    WIDTH = 640,
    HEIGHT = 480,
    BLOCK_BYTES = WIDTH * HEIGHT * 6,
    FAKE_FAILURE = -900
};
static uint8_t pixels[BUFFERS][BLOCK_BYTES];
static k_vicap_chn_attr channel;
static k_vicap_dev_attr device;
static uint32_t map_calls, unmap_calls, live_maps, dump_calls, dumps_held, next_block;
static uint32_t deinit_calls, invalidate_calls, fail_map_call;
static int wrong_pool, bad_address, bad_stride, fail_release, fail_unmap, fail_deinit;
static int vb_live, vicap_live, streaming, not_ready_count;
static int probe_result, sensor_bad_size, sensor_bad_csi, info_changes_mode, sensor_unknown_fps;
static int mono, fail_cache, bad_format, fail_init;
static int pool_live, fail_pool_create, fail_pool_destroy;
static int sensor_powered, sensor_streaming, fail_sensor_stage, fail_sensor_stop;
static int mcm_configured, fail_mcm;
static uint32_t mcm_registers[0x2000 / 4];
static int fail_mcm_map, wrong_mcm_registers;
static uint32_t exposure_calls;
static k_u32 sensor_registers[65536];
static struct {
    k_u32 address, value;
} pending_registers[5];
static unsigned pending_count, reg_write_calls;
static int group_holding, fail_reg_write_call, fail_reg_read_addr, bad_readback;

static void reset_sensor_registers(void) {
    memset(sensor_registers, 0, sizeof(sensor_registers));
    sensor_registers[0x320e] = 0x02;
    sensor_registers[0x320f] = 0x58;
    sensor_registers[0x3e00] = 0xa0; /* Reserved bits must survive exposure writes. */
    sensor_registers[0x3e01] = 0x08;
    pending_count = 0;
    group_holding = 0;
}

static uint64_t physical(uint32_t index) {
    return UINT64_C(0x10000000) + (uint64_t)index * BLOCK_BYTES;
}

static void reset_fake(void) {
    assert(!vb_live && !vicap_live && !streaming && !live_maps && !dumps_held);
    assert(!pool_live && !sensor_powered && !sensor_streaming);
    channel = (k_vicap_chn_attr){0};
    map_calls = unmap_calls = dump_calls = next_block = 0;
    deinit_calls = invalidate_calls = fail_map_call = 0;
    wrong_pool = bad_address = bad_stride = fail_release = fail_unmap = fail_deinit = 0;
    not_ready_count = 0;
    probe_result = sensor_bad_size = sensor_bad_csi = info_changes_mode = 0;
    sensor_unknown_fps = 0;
    mono = 1;
    fail_cache = bad_format = fail_init = 0;
    fail_pool_create = fail_pool_destroy = 0;
    fail_sensor_stage = fail_sensor_stop = 0;
    mcm_configured = fail_mcm = 0;
    fail_mcm_map = wrong_mcm_registers = 0;
    memset(mcm_registers, 0, sizeof(mcm_registers));
    memset(pixels, 0, sizeof(pixels));
    exposure_calls = 0;
    reg_write_calls = 0;
    fail_reg_write_call = bad_readback = 0;
    fail_reg_read_addr = -1;
    reset_sensor_registers();
}

k_s32 kd_mpi_vb_set_config(const k_vb_config* config) {
    assert(!vb_live && config->comm_pool[0].blk_cnt == 0);
    return 0;
}
k_s32 kd_mpi_vb_init(void) {
    vb_live = 1;
    return 0;
}
k_s32 kd_mpi_vb_exit(void) {
    assert(!vicap_live && !pool_live && !live_maps && !dumps_held);
    vb_live = 0;
    return 0;
}
k_s32 kd_mpi_vb_handle_to_pool_id(k_vb_blk_handle block) {
    return pool_live && (block >> 16u) == POOL_ID && (block & 0xffffu) < BUFFERS
             ? POOL_ID
             : (k_s32)VB_INVALID_POOLID;
}
k_u64 kd_mpi_vb_handle_to_phyaddr(k_vb_blk_handle block) {
    return kd_mpi_vb_handle_to_pool_id(block) == POOL_ID ? physical(block & 0xffffu) : 0;
}
k_vb_blk_handle kd_mpi_vb_phyaddr_to_handle(k_u64 address) {
    for (uint32_t i = 0; i < BUFFERS && pool_live; ++i) {
        if (address == physical(i)) {
            return ((wrong_pool ? POOL_ID + 1u : POOL_ID) << 16u) | i;
        }
    }
    return VB_INVALID_HANDLE;
}
/* The camera owns the RAW pool; only MCM acquires/releases its VB blocks. */
k_s32 kd_mpi_vb_create_pool(k_vb_pool_config* config) {
    assert(vb_live && !pool_live && config->blk_size == BLOCK_BYTES);
    assert(config->blk_cnt == BUFFERS);
    if (fail_pool_create)
        return FAKE_FAILURE;
    pool_live = 1;
    return POOL_ID;
}
k_s32 kd_mpi_vb_destory_pool(k_u32 pool) {
    assert(pool == POOL_ID && pool_live && !vicap_live && !live_maps && !dumps_held);
    if (fail_pool_destroy)
        return FAKE_FAILURE;
    pool_live = 0;
    return 0;
}
k_vb_blk_handle kd_mpi_vb_get_block(k_u32 pool, k_u64 bytes, const k_char* name) {
    (void)pool;
    (void)bytes;
    (void)name;
    assert(0);
    return VB_INVALID_HANDLE;
}
k_s32 kd_mpi_vb_release_block(k_vb_blk_handle block) {
    (void)block;
    assert(0);
    return FAKE_FAILURE;
}
void* kd_mpi_sys_mmap_cached(k_u64 address, k_u32 size) {
    assert(vicap_live && size == BLOCK_BYTES);
    ++map_calls;
    if (map_calls == fail_map_call)
        return NULL;
    for (uint32_t i = 0; i < BUFFERS; ++i) {
        if (address == physical(i)) {
            ++live_maps;
            return pixels[i];
        }
    }
    assert(0);
    return NULL;
}
k_s32 kd_mpi_sys_munmap(void* address, k_u32 size) {
    assert(vicap_live && size == BLOCK_BYTES && address != NULL && live_maps > 0);
    ++unmap_calls;
    if (fail_unmap)
        return FAKE_FAILURE;
    --live_maps;
    return 0;
}
k_s32 kd_mpi_sys_mmz_invalidate_cache(k_u64 address, void* mapped, k_u32 size) {
    const uint32_t index = (uint32_t)((address - physical(0)) / BLOCK_BYTES);
    assert(index < BUFFERS && mapped == pixels[index]);
    assert(size == WIDTH * HEIGHT * 2u || size == WIDTH / 4u * 5u * HEIGHT);
    ++invalidate_calls;
    return fail_cache ? FAKE_FAILURE : 0;
}
k_s32 kd_mpi_sensor_adapt_get(k_vicap_probe_config* probe, k_vicap_sensor_info* info) {
    if (probe_result)
        return probe_result;
    *info = (k_vicap_sensor_info){0};
    info->width = sensor_bad_size ? 99999u : WIDTH;
    info->height = HEIGHT;
    info->fps = sensor_unknown_fps ? 0 : 30;
    info->csi_num = sensor_bad_csi ? 0u : probe->csi_num + 1u;
    info->hdr_mode = VICAP_LINERA_MODE;
    info->sensor_name = mono ? "sc038hgs_csi2" : "legacy_csi2";
    info->data_type = VICAP_CSI_DATA_TYPE_RAW10;
    return 0;
}
k_s32 kd_mpi_vicap_get_sensor_info(k_vicap_sensor_type type, k_vicap_sensor_info* info) {
    (void)type;
    if (info_changes_mode)
        info->fps = 60;
    return 0;
}
k_s32 kd_mpi_vicap_set_dev_attr(k_vicap_dev dev, k_vicap_dev_attr attr) {
    assert(dev == VICAP_DEV_ID_0 && attr.acq_win.width == WIDTH);
    assert(attr.mode == VICAP_WORK_ONLY_MCM_MODE);
    assert(attr.pipe_ctrl.data == 0 && attr.buffer_pool_id == POOL_ID);
    device = attr;
    device.buffer_size = BLOCK_BYTES;
    return 0;
}
k_s32 kd_mpi_vicap_get_dev_attr(k_vicap_dev dev, k_vicap_dev_attr* attr) {
    (void)dev;
    *attr = device;
    return 0;
}
int __wrap_ioctl(int fd, int request, ...);

int __wrap_open(const char* path, int flags, ...) {
    assert(strcmp(path, "/dev/mem") == 0 && flags == (O_RDWR | O_SYNC));
    assert(!streaming);
    return 98;
}
int __real_close(int fd);
int __wrap_close(int fd) { return fd == 98 ? 0 : __real_close(fd); }
void* __wrap_mmap(void* addr, size_t length, int prot, int flags, int fd, off_t offset) {
    assert(addr == NULL && length == sizeof(mcm_registers) && fd == 98);
    assert(prot == (PROT_READ | PROT_WRITE) && flags == MAP_SHARED);
    assert((uint64_t)offset == UINT64_C(0x90000000));
    return fail_mcm_map ? MAP_FAILED : mcm_registers;
}
int __wrap_munmap(void* addr, size_t length) {
    assert(addr == mcm_registers && length == sizeof(mcm_registers));
    /* Only the input width changes; DMA pitch/limits and formats survive. */
    assert(
        mcm_registers[0x1204 / 4] == ((HEIGHT << 16) | (wrong_mcm_registers ? WIDTH * 2 : WIDTH)));
    assert(mcm_registers[0x1200 / 4] == (wrong_mcm_registers ? 0x20 : 0x20020));
    assert(mcm_registers[0x1604 / 4] == 0x10);
    assert(mcm_registers[0x1618 / 4] == WIDTH * HEIGHT * 2);
    assert(mcm_registers[0x1620 / 4] == WIDTH * 2);
    return 0;
}

int __real_ioctl(int fd, int request, ...) {
    va_list args;
    va_start(args, request);
    const k_vicap_mcm_chn_attr* attr = va_arg(args, const k_vicap_mcm_chn_attr*);
    va_end(args);
    if (request == 1234) {
        assert(fd == 77 && attr == NULL);
        return 27;
    }
    assert(fd == 9 && request == 0x40204315 && attr != NULL);
    assert(attr->cfg.format == VICAP_MCM_RAW10 && attr->cfg.stride == WIDTH * 2);
    assert(attr->buffer_pool_id == POOL_ID && attr->buff_num == BUFFERS);
    assert(attr->work_mode == VICAP_WORK_ONLY_MCM_MODE);
    mcm_registers[0x1204 / 4] = (HEIGHT << 16) | (WIDTH * 2);
    mcm_registers[0x1200 / 4] = wrong_mcm_registers ? 0x20 : 0x20020;
    mcm_registers[0x1604 / 4] = 0x10;
    mcm_registers[0x1618 / 4] = WIDTH * HEIGHT * 2;
    mcm_registers[0x1620 / 4] = WIDTH * 2;
    ++mcm_configured;
    return 0;
}

k_s32 kd_mpi_vicap_set_chn_attr(k_vicap_dev dev, k_vicap_chn chn, k_vicap_chn_attr attr) {
    assert(dev == VICAP_DEV_ID_0 && chn == VICAP_CHN_ID_0);
    assert(attr.pix_format == PIXEL_FORMAT_RGB_BAYER_10BPP);
    assert(!attr.scale_enable && attr.buffer_pool_id == POOL_ID);
    channel = attr;
    channel.buffer_pool_id = (k_s32)VB_INVALID_POOLID;
    channel.buffer_size = BLOCK_BYTES;
    /* Reproduce the checked MPI's incorrect defaults. The real adapter must
     * correct this request before it reaches the simulated kernel. */
    k_vicap_mcm_chn_attr mcm = {0};
    mcm.cfg.width = WIDTH;
    mcm.cfg.height = HEIGHT;
    mcm.cfg.stride = WIDTH * 3;
    mcm.work_mode = VICAP_WORK_ONLY_MCM_MODE;
    if (fail_mcm)
        mcm.cfg.width = 1;
    return __wrap_ioctl(9, 0x40204315, &mcm);
}
k_s32 kd_mpi_vicap_get_chn_attr(k_vicap_dev dev, k_vicap_chn chn, k_vicap_chn_attr* attr) {
    (void)dev;
    (void)chn;
    *attr = channel;
    return 0;
}
k_s32 kd_mpi_vicap_init(k_vicap_dev dev) {
    (void)dev;
    assert(vb_live);
    assert(mcm_configured == 1);
    vicap_live = 1;
    return fail_init ? FAKE_FAILURE : 0;
}
k_s32 kd_mpi_vicap_deinit(k_vicap_dev dev) {
    (void)dev;
    assert(!streaming && !dumps_held && !live_maps);
    ++deinit_calls;
    if (fail_deinit)
        return FAKE_FAILURE;
    vicap_live = mcm_configured = 0;
    return 0;
}
k_s32 kd_mpi_vicap_start_stream(k_vicap_dev dev) {
    (void)dev;
    assert(vicap_live && sensor_powered);
    if (fail_sensor_stage == 4)
        return FAKE_FAILURE;
    streaming = 1;
    return 0;
}
k_s32 kd_mpi_vicap_stop_stream(k_vicap_dev dev) {
    (void)dev;
    assert(!dumps_held);
    streaming = 0;
    return 0;
}
k_s32 kd_mpi_vicap_get_sensor_fd(k_vicap_sensor_attr* attr) {
    attr->sensor_fd = 9;
    return 0;
}
k_s32 kd_mpi_vicap_register_scene(const char* name, const char* path) {
    (void)name;
    (void)path;
    assert(0);
    return FAKE_FAILURE;
}
const char* kd_mpi_vicap_get_scene(void) {
    assert(0);
    return NULL;
}
k_s32 kd_mpi_vicap_load_scene(const char* name) {
    (void)name;
    assert(0);
    return FAKE_FAILURE;
}
k_s32 kd_mpi_vicap_set_database_parse_mode(k_vicap_dev dev, k_vicap_database_parse_mode mode) {
    (void)dev;
    (void)mode;
    assert(0);
    return FAKE_FAILURE;
}
k_s32 kd_mpi_sensor_power_set(k_s32 fd, k_bool on) {
    assert(fd == 9 && !sensor_streaming);
    if (on && fail_sensor_stage == 1)
        return FAKE_FAILURE;
    sensor_powered = on;
    return 0;
}
k_s32 kd_mpi_sensor_init(k_s32 fd, k_sensor_mode mode) {
    (void)mode;
    assert(fd == 9 && sensor_powered && !streaming);
    reset_sensor_registers();
    return fail_sensor_stage == 2 ? FAKE_FAILURE : 0;
}
k_s32 kd_mpi_sensor_tpg_set(k_s32 fd, k_sensor_test_pattern pattern) {
    assert(fd == 9 && sensor_powered && !pattern.enable);
    return fail_sensor_stage == 3 ? FAKE_FAILURE : 0;
}
k_s32 kd_mpi_sensor_stream_enable(k_s32 fd, k_s32 on) {
    assert(fd == 9 && sensor_powered);
    if ((on && fail_sensor_stage == 5) || (!on && fail_sensor_stop))
        return FAKE_FAILURE;
    sensor_streaming = on;
    return 0;
}
k_s32 kd_mpi_sensor_get_exposure_time_range(k_s32 fd, k_sensor_exposure_time_range* range) {
    (void)fd;
    (void)range;
    assert(0);
    return FAKE_FAILURE;
}
k_s32 kd_mpi_sensor_intg_time_set(k_s32 fd, k_sensor_intg_time integration) {
    (void)fd;
    (void)integration;
    assert(0);
    return FAKE_FAILURE;
}
/* SC132GS (ISP) controls; the SC038HGS RAW path must never reach them. */
k_s32 kd_mpi_sensor_intg_time_get(k_s32 fd, k_sensor_intg_time* integration) {
    (void)fd;
    (void)integration;
    assert(0);
    return FAKE_FAILURE;
}
k_s32 kd_mpi_sensor_mode_get(k_s32 fd, k_sensor_mode* mode) {
    (void)fd;
    (void)mode;
    assert(0);
    return FAKE_FAILURE;
}
k_s32 kd_mpi_sensor_again_set(k_s32 fd, k_sensor_gain gain) {
    (void)fd;
    (void)gain;
    assert(0);
    return FAKE_FAILURE;
}
k_s32 kd_mpi_sensor_reg_read(k_s32 fd, k_u32 address, k_u32* value) {
    assert(fd == 9 && sensor_powered && address < 65536);
    if ((int)address == fail_reg_read_addr)
        return FAKE_FAILURE;
    *value = sensor_registers[address];
    if (bad_readback && exposure_calls && address == 0x3e01)
        *value ^= 1u;
    return 0;
}
k_s32 kd_mpi_sensor_reg_write(k_s32 fd, k_u32 address, k_u32 value) {
    assert(fd == 9 && sensor_powered && value <= 255);
    if ((int)++reg_write_calls == fail_reg_write_call)
        return FAKE_FAILURE;
    if (address == 0x3800) {
        if (value == 0x00) {
            group_holding = 1;
            pending_count = 0;
        } else if (value == 0x10) {
            group_holding = 0;
        } else {
            assert(value == 0x60 && !group_holding && pending_count == 5);
            for (unsigned i = 0; i < pending_count; ++i)
                sensor_registers[pending_registers[i].address] = pending_registers[i].value;
            ++exposure_calls;
        }
    } else {
        assert(group_holding && pending_count < 5);
        pending_registers[pending_count].address = address;
        pending_registers[pending_count++].value = value;
    }
    return 0;
}
k_s32 kd_mpi_vicap_dump_frame(
    k_vicap_dev dev, k_vicap_chn chn, k_vicap_dump_format format, k_video_frame_info* info,
    k_u32 timeout) {
    (void)dev;
    (void)chn;
    (void)timeout;
    assert(streaming && sensor_streaming);
    ++dump_calls;
    assert(format == VICAP_DUMP_YUV444);
    if (not_ready_count > 0) {
        --not_ready_count;
        return K_ERR_VICAP_NOTREADY;
    }
    *info = (k_video_frame_info){0};
    info->pool_id = wrong_pool ? POOL_ID + 1 : POOL_ID;
    info->v_frame.width = WIDTH;
    info->v_frame.height = HEIGHT;
    /* Driver stride is deliberately garbage: configured DMA pitch owns layout. */
    info->v_frame.stride[0] = 0xdeadbeef;
    if (bad_stride)
        info->v_frame.width = WIDTH - 1;
    info->v_frame.pixel_format =
        bad_format ? PIXEL_FORMAT_RGB_BAYER_12BPP : PIXEL_FORMAT_YVU_SEMIPLANAR_444;
    info->v_frame.compress_mode = COMPRESS_MODE_NONE;
    info->v_frame.phys_addr[0] =
        bad_address ? UINT64_C(0x90000000) : physical(next_block++ % BUFFERS);
    ++dumps_held;
    return 0;
}
k_s32 kd_mpi_vicap_dump_release(k_vicap_dev dev, k_vicap_chn chn, const k_video_frame_info* info) {
    (void)dev;
    (void)chn;
    (void)info;
    assert(dumps_held);
    if (fail_release)
        return FAKE_FAILURE;
    --dumps_held;
    return 0;
}
static dart_camera_t* initialize(dart_media_t** media) {
    dart_camera_config_t config;
    dart_camera_config_init(&config);
    config.csi_port = 2;
    config.max_outstanding_frames = 2;
    config.exposure_mode = DART_CAMERA_EXPOSURE_FIXED;
    dart_camera_t* camera = NULL;
    assert(dart_media_create(media) == 0);
    assert(dart_camera_create(&camera) == 0);
    assert(dart_camera_init(camera, *media, &config) == 0);
    dart_camera_caps_t caps;
    assert(dart_camera_get_caps(camera, &caps) == 0);
    assert(caps.output_width == WIDTH && caps.output_height == HEIGHT);
    assert(caps.sensor_fps == (sensor_unknown_fps ? 0u : 30u));
    assert(camera->config.output_width == WIDTH && camera->config.output_height == HEIGHT);
    assert(map_calls == 0 && channel.buffer_pool_id == (k_s32)VB_INVALID_POOLID);
    return camera;
}

static void destroy(dart_camera_t* camera, dart_media_t* media) {
    assert(dart_camera_deinit(camera) == 0);
    assert(dart_camera_destroy(camera) == 0);
    assert(dart_media_destroy(media) == 0);
}

static void invalid_modes(void) {
    for (int test = 0; test < 5; ++test) {
        reset_fake();
        dart_media_t* media = NULL;
        dart_camera_t* camera = NULL;
        dart_camera_config_t config;
        dart_camera_config_init(&config);
        config.csi_port = 2;
        if (test == 0)
            probe_result = 1;
        if (test == 1)
            sensor_bad_size = 1;
        if (test == 2)
            sensor_bad_csi = 1;
        if (test == 3)
            info_changes_mode = 1;
        if (test == 4) {
            config.sensor_width = WIDTH;
            config.sensor_height = HEIGHT;
            config.sensor_fps = 60;
        }
        assert(dart_media_create(&media) == 0);
        assert(dart_camera_create(&camera) == 0);
        const int expected = test == 0 ? DART_CAMERA_E_SENSOR_PROBE_FAILED
                           : test == 1 ? DART_CAMERA_E_CONFIG_INVALID
                                       : DART_CAMERA_E_SENSOR_MODE_MISMATCH;
        assert(dart_camera_init(camera, media, &config) == expected);
        assert(!vicap_live && !streaming && !live_maps && !dumps_held);
        assert(camera->state == DART_CAMERA_STATE_EMPTY);
        assert(dart_camera_destroy(camera) == 0);
        assert(dart_media_destroy(media) == 0);
    }
}

static void raw_gray_frames(void) {
    for (uint32_t layout = 0; layout <= DART_CAMERA_RAW10_PACKED_LSB; ++layout) {
        reset_fake();
        sensor_unknown_fps = 1;
        const uint32_t stride = layout == DART_CAMERA_RAW10_PACKED_LSB ? WIDTH / 4 * 5 : WIDTH * 2;
        for (uint32_t block = 0; block < BUFFERS; ++block) {
            for (uint32_t y = 0; y < HEIGHT; ++y) {
                for (uint32_t x = 0; x < WIDTH; ++x) {
                    uint8_t* row = pixels[block] + y * stride;
                    if (layout == DART_CAMERA_RAW10_MIPI) {
                        row[x / 4 * 5 + x % 4] = (uint8_t)(x >> 2);
                        row[x / 4 * 5 + 4] |= (uint8_t)((x & 3) << (2 * (x % 4)));
                    } else if (layout == DART_CAMERA_RAW10_PACKED_LSB) {
                        const uint64_t bits = (uint64_t)x << (10u * (x % 4u));
                        for (unsigned byte = 0; byte < 5; ++byte)
                            row[x / 4 * 5 + byte] |= (uint8_t)(bits >> (byte * 8u));
                    } else {
                        const uint32_t word = layout == DART_CAMERA_RAW10_LE16_MSB ? x << 6 : x;
                        row[2 * x] = (uint8_t)word;
                        row[2 * x + 1] = (uint8_t)(word >> 8);
                    }
                }
            }
        }
        dart_media_t* media = NULL;
        dart_camera_t* camera = initialize(&media);
        camera->config.raw10_layout = layout;
        camera->raw_stride = stride;
        assert(dart_camera_start(camera) == 0 && exposure_calls == 0);
        dart_camera_frame_t first, second, extra;
        assert(dart_camera_acquire_frame(camera, 20, &first) == 0);
        assert(dumps_held == 0);           /* Recorder holds only grayscale CPU pixels. */
        assert(first.stride == WIDTH && first.data[199] == 49 && first.data[200] == 50);
        assert(first.data[(HEIGHT - 1) * WIDTH + 199] == 49);
        assert(first.data[(HEIGHT - 1) * WIDTH + 200] == 50);
        assert(first.data != pixels[1]);
        assert(dart_camera_acquire_frame(camera, 20, &second) == 0);
        assert(first.data != second.data && first.data[199] == 49 && first.data[200] == 50);
        memset(pixels, 0, sizeof(pixels)); /* DMA reuse cannot change leased pixels. */
        assert(first.data[200] == 50 && second.data[200] == 50);
        assert(dart_camera_acquire_frame(camera, 0, &extra) == DART_CAMERA_E_LEASE_EXHAUSTED);
        assert(dart_camera_stop(camera) == DART_CAMERA_E_BUSY);
        assert(dart_camera_release_frame(camera, &second) == 0);
        assert(dart_camera_release_frame(camera, &first) == 0);
        uint32_t min_lines, max_lines;
        assert(dart_camera_get_exposure_range(camera, &min_lines, &max_lines) == 0);
        assert(min_lines == 4 && max_lines == 65509);
        bad_format = 1;
        assert(
            dart_camera_acquire_frame(camera, 0, &extra) == DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH);
        bad_format = 0;
        fail_cache = 1;
        assert(
            dart_camera_acquire_frame(camera, 0, &extra) == DART_CAMERA_E_CACHE_INVALIDATE_FAILED);
        assert(extra.data == NULL && !dumps_held);
        fail_cache = 0;
        assert(dart_camera_stop(camera) == 0 && dart_camera_start(camera) == 0);
        assert(map_calls == BUFFERS);
        destroy(camera, media);
    }
}

static void raw_rollback(void) {
    for (int failure = 0; failure < 2; ++failure) {
        reset_fake();
        fail_mcm_map = failure == 0;
        wrong_mcm_registers = failure == 1;
        dart_media_t* media = NULL;
        dart_camera_t* camera = NULL;
        dart_camera_config_t config;
        dart_camera_config_init(&config);
        config.csi_port = 2;
        assert(dart_media_create(&media) == 0 && dart_camera_create(&camera) == 0);
        assert(
            dart_camera_init(camera, media, &config)
            == (failure == 0 ? DART_CAMERA_E_MAP_FAILED : DART_CAMERA_E_NOT_SUPPORTED));
        assert(!streaming && !pool_live && !vicap_live && !live_maps);
        assert(dart_camera_destroy(camera) == 0 && dart_media_destroy(media) == 0);
    }
    for (int stage = 0; stage < 4; ++stage) {
        reset_fake();
        fail_pool_create = stage == 0;
        fail_init = stage != 0;
        fail_deinit = stage == 2;
        fail_pool_destroy = stage == 3;
        dart_media_t* media = NULL;
        dart_camera_t* camera = NULL;
        dart_camera_config_t config;
        dart_camera_config_init(&config);
        config.csi_port = 2;
        assert(dart_media_create(&media) == 0 && dart_camera_create(&camera) == 0);
        assert(dart_camera_init(camera, media, &config) == FAKE_FAILURE);
        if (stage >= 2) {
            assert(camera->state == DART_CAMERA_STATE_FAULTED && pool_live);
            assert(dart_media_destroy(media) == DART_CAMERA_E_BUSY);
            fail_deinit = fail_pool_destroy = 0;
            destroy(camera, media);
        } else {
            assert(camera->state == DART_CAMERA_STATE_EMPTY && !pool_live && !vicap_live);
            assert(dart_camera_destroy(camera) == 0 && dart_media_destroy(media) == 0);
        }
    }
    reset_fake();
    dart_camera_config_t config;
    dart_camera_config_init(&config);
    config.csi_port = 2;
    config.exposure_mode = DART_CAMERA_EXPOSURE_AUTO;
    dart_media_t* media = NULL;
    dart_camera_t* camera = NULL;
    assert(dart_media_create(&media) == 0 && dart_camera_create(&camera) == 0);
    assert(dart_camera_init(camera, media, &config) == DART_CAMERA_E_NOT_SUPPORTED);
    assert(!pool_live && !vicap_live);
    assert(dart_camera_destroy(camera) == 0 && dart_media_destroy(media) == 0);

    reset_fake();
    mono = sensor_unknown_fps = 1;
    camera = initialize(&media);
    camera->config.initial_exposure_lines = 250;
    assert(dart_camera_start(camera) == 0);
    assert(streaming && exposure_calls == 1);
    assert(sensor_registers[0x3e01] == 0x0f && sensor_registers[0x3e02] == 0xa0);
    destroy(camera, media);

    for (int stage = 1; stage <= 5; ++stage) {
        reset_fake();
        camera = initialize(&media);
        fail_sensor_stage = stage;
        assert(dart_camera_start(camera) == FAKE_FAILURE);
        assert(!streaming && !sensor_powered && !sensor_streaming);
        fail_sensor_stage = 0;
        assert(dart_camera_start(camera) == 0);
        destroy(camera, media);
    }

    reset_fake();
    camera = initialize(&media);
    assert(dart_camera_start(camera) == 0);
    fail_sensor_stop = 1;
    assert(dart_camera_deinit(camera) == FAKE_FAILURE);
    assert(pool_live && streaming && camera->state == DART_CAMERA_STATE_FAULTED);
    fail_sensor_stop = 0;
    destroy(camera, media);

    reset_fake();
    config.exposure_mode = DART_CAMERA_EXPOSURE_FIXED;
    assert(dart_media_create(&media) == 0 && dart_camera_create(&camera) == 0);
    fail_mcm = 1;
    assert(dart_camera_init(camera, media, &config) != 0);
    assert(!pool_live && !vicap_live && !mcm_configured);
    assert(dart_camera_destroy(camera) == 0 && dart_media_destroy(media) == 0);
}

static uint32_t exposure_lines(void) {
    return (((sensor_registers[0x3e00] & 0x0f) << 16) | (sensor_registers[0x3e01] << 8)
            | sensor_registers[0x3e02])
         / 16u;
}

static uint32_t frame_lines(void) {
    return (sensor_registers[0x320e] << 8) | sensor_registers[0x320f];
}

static void line_exposure(void) {
    reset_fake();
    sensor_unknown_fps = 1;
    dart_media_t* media = NULL;
    dart_camera_t* camera = initialize(&media);
    assert(dart_camera_set_exposure_lines(camera, 2560) == DART_CAMERA_E_INVALID_STATE);
    camera->config.initial_exposure_lines = 2560;
    assert(dart_camera_start(camera) == 0);
    assert(exposure_lines() == 2560 && frame_lines() == 2571);
    assert(sensor_registers[0x3e00] == 0xa0 && exposure_calls == 1);
    assert(dart_camera_set_exposure_lines(camera, 65509) == 0);
    assert(exposure_lines() == 65509 && frame_lines() == 65520);
    assert(sensor_registers[0x3e00] == 0xaf);
    assert(dart_camera_set_exposure_lines(camera, 4) == 0);
    assert(exposure_lines() == 4 && frame_lines() == 600);
    const unsigned before = reg_write_calls;
    assert(dart_camera_set_exposure_lines(camera, 3) == DART_CAMERA_E_OUT_OF_RANGE);
    assert(dart_camera_set_exposure_lines(camera, 65510) == DART_CAMERA_E_OUT_OF_RANGE);
    assert(reg_write_calls == before && exposure_calls == 3);
    destroy(camera, media);
}

static void line_exposure_failures(void) {
    /* Each of the eight group transfers, pre/post reads, and readback mismatch. */
    for (int failure = 1; failure <= 11; ++failure) {
        reset_fake();
        sensor_unknown_fps = 1;
        dart_media_t* media = NULL;
        dart_camera_t* camera = initialize(&media);
        assert(dart_camera_start(camera) == 0);
        if (failure <= 8)
            fail_reg_write_call = failure;
        if (failure == 9)
            fail_reg_read_addr = 0x3e00;
        if (failure == 10)
            fail_reg_read_addr = 0x3e01;
        bad_readback = failure == 11;
        const int rc = dart_camera_set_exposure_lines(camera, 2560);
        assert(rc == (failure == 11 ? DART_CAMERA_E_EXPOSURE_WRITE_FAILED : FAKE_FAILURE));
        assert(
            camera->state
            == (failure == 9 ? DART_CAMERA_STATE_STARTED : DART_CAMERA_STATE_FAULTED));
        if (failure <= 9) {
            assert(exposure_calls == 0);
            assert(exposure_lines() == 128 && frame_lines() == 600);
        }
        fail_reg_write_call = bad_readback = 0;
        fail_reg_read_addr = -1;
        const dart_camera_config_t config = camera->config;
        assert(dart_camera_deinit(camera) == 0);
        assert(dart_camera_init(camera, media, &config) == 0);
        assert(dart_camera_start(camera) == 0);
        assert(dart_camera_set_exposure_lines(camera, 2560) == 0);
        assert(exposure_lines() == 2560 && frame_lines() == 2571);
        destroy(camera, media);
    }

    reset_fake();
    dart_media_t* media = NULL;
    dart_camera_t* camera = initialize(&media);
    camera->config.initial_exposure_lines = 2560;
    fail_reg_write_call = 3;
    assert(dart_camera_start(camera) == FAKE_FAILURE);
    assert(camera->state == DART_CAMERA_STATE_FAULTED);
    assert(!streaming && !sensor_powered && !sensor_streaming);
    fail_reg_write_call = 0;
    const dart_camera_config_t config = camera->config;
    assert(dart_camera_deinit(camera) == 0);
    assert(dart_camera_init(camera, media, &config) == 0);
    assert(dart_camera_start(camera) == 0);
    assert(exposure_lines() == 2560 && frame_lines() == 2571);
    destroy(camera, media);
}

int main(void) {
    assert(__wrap_ioctl(77, 1234, (void*)NULL) == 27);
    line_exposure();
    line_exposure_failures();
    raw_gray_frames();
    raw_rollback();
    invalid_modes();
    dart_media_t* media = NULL;
    reset_fake();
    sensor_unknown_fps = 1;
    dart_camera_t* unknown = initialize(&media);
    assert(dart_camera_start(unknown) == 0);
    dart_camera_frame_t frame;
    assert(dart_camera_acquire_frame(unknown, 20, &frame) == 0);
    assert(frame.width == WIDTH && frame.height == HEIGHT);
    assert(dart_camera_release_frame(unknown, &frame) == 0);
    destroy(unknown, media);
    reset_fake();
    dart_camera_t* camera = initialize(&media);
    not_ready_count = 1;
    assert(dart_camera_start(camera) == 0);
    assert(map_calls == BUFFERS && dumps_held == 0 && dump_calls == 2);
    assert(camera->output_pool.pool_id == POOL_ID);
    for (uint32_t i = 0; i < 8; ++i) {
        dart_camera_frame_t frame;
        assert(dart_camera_acquire_frame(camera, 20, &frame) == 0);
        assert(frame.data != NULL && frame.width == WIDTH && frame.stride == WIDTH);
        assert(dart_camera_release_frame(camera, &frame) == 0);
    }
    assert(invalidate_calls == 8 && map_calls == BUFFERS && unmap_calls == 0);
    dart_camera_frame_t first, second, extra;
    assert(dart_camera_acquire_frame(camera, 20, &first) == 0);
    assert(dart_camera_acquire_frame(camera, 20, &second) == 0);
    const uint32_t previous_dumps = dump_calls;
    assert(dart_camera_acquire_frame(camera, 20, &extra) == DART_CAMERA_E_LEASE_EXHAUSTED);
    assert(dump_calls == previous_dumps && dart_camera_stop(camera) == DART_CAMERA_E_BUSY);
    assert(dart_camera_release_frame(camera, &second) == 0);
    assert(dart_camera_release_frame(camera, &first) == 0);
    wrong_pool = 1;
    assert(dart_camera_acquire_frame(camera, 20, &extra) == DART_CAMERA_E_FRAME_POOL_MISMATCH);
    assert(extra.data == NULL && dumps_held == 0 && dart_frame_lease_active(&camera->leases) == 0);
    wrong_pool = 0;
    bad_address = 1;
    assert(dart_camera_acquire_frame(camera, 20, &extra) == DART_CAMERA_E_FRAME_POOL_MISMATCH);
    bad_address = 0;
    bad_stride = 1;
    assert(dart_camera_acquire_frame(camera, 20, &extra) == DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH);
    bad_stride = 0;
    assert(dart_camera_stop(camera) == 0 && dart_camera_start(camera) == 0);
    assert(map_calls == BUFFERS);
    fail_unmap = 1;
    assert(dart_camera_deinit(camera) == FAKE_FAILURE);
    assert(vicap_live && deinit_calls == 0 && live_maps == BUFFERS);
    assert(dart_media_destroy(media) == DART_CAMERA_E_BUSY);
    fail_unmap = 0;
    fail_deinit = 1;
    assert(dart_camera_deinit(camera) == FAKE_FAILURE);
    assert(vicap_live && live_maps == 0 && camera->state == DART_CAMERA_STATE_FAULTED);
    fail_deinit = 0;
    destroy(camera, media);

    reset_fake();
    camera = initialize(&media);
    fail_map_call = 2;
    assert(dart_camera_start(camera) == DART_CAMERA_E_MAP_FAILED);
    assert(!streaming && !dumps_held && !live_maps && vicap_live);
    fail_map_call = 0;
    assert(dart_camera_start(camera) == 0);
    destroy(camera, media);

    reset_fake();
    camera = initialize(&media);
    fail_map_call = 2;
    fail_unmap = 1;
    assert(dart_camera_start(camera) == DART_CAMERA_E_MAP_FAILED);
    assert(camera->state == DART_CAMERA_STATE_FAULTED && live_maps == 1 && !dumps_held);
    fail_unmap = 0;
    destroy(camera, media);

    reset_fake();
    camera = initialize(&media);
    fail_release = 1;
    assert(dart_camera_start(camera) == FAKE_FAILURE);
    assert(streaming && dumps_held == 1 && camera->state == DART_CAMERA_STATE_FAULTED);
    fail_release = 0;
    destroy(camera, media);

    reset_fake();
    camera = initialize(&media);
    assert(dart_camera_start(camera) == 0);
    fail_release = 1;
    assert(dart_camera_acquire_frame(camera, 20, &extra) == FAKE_FAILURE);
    assert(extra.data == NULL && dumps_held == 1);
    assert(dart_frame_lease_active(&camera->leases) == 0);
    assert(camera->state == DART_CAMERA_STATE_FAULTED);
    fail_release = 0;
    destroy(camera, media);

    reset_fake();
    camera = initialize(&media);
    wrong_pool = 1;
    assert(dart_camera_start(camera) == DART_CAMERA_E_FRAME_POOL_MISMATCH);
    assert(!streaming && !dumps_held && map_calls == 0);
    destroy(camera, media);

    reset_fake();
    camera = initialize(&media);
    not_ready_count = 4;
    assert(dart_camera_start(camera) == DART_CAMERA_E_NO_FRAME);
    assert(dump_calls == 3 && !streaming && !dumps_held && map_calls == 0);
    destroy(camera, media);
    puts("camera pool regression passed: startup, leases, validation, rollback and cleanup");
    return 0;
}
