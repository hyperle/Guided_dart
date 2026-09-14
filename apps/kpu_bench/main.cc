/*
 * kpu_bench —— 「模型无关」的 AI2D + KPU 乒乓流水计时器
 *
 * 目的：在还没有绿灯 kmodel 的情况下，先把**真实硬件链路**的耗时测出来，回答
 * 「单帧处理能不能压进 5ms / 200fps 够不够」这个问题。整条链路与将来正式实现
 * 完全一致，只是暂时不接后处理解码：
 *
 *   VICAP(dev0, OFFLINE) CHN0
 *     └ RGB_888_PLANAR 640x360（ISP 硬件完成 NV12->RGB 平面，CPU 不做色彩转换）
 *          └ pre 线程: dump 帧 -> 用帧的**物理地址**直接建 AI2D 输入 tensor
 *                      （零拷贝，不 memcpy）-> AI2D 硬件 resize+pad（等 AI2D 中断）
 *                      -> 立刻归还 VICAP 帧（AI2D 已读完）
 *          └ kpu 线程: interp.run()（nncase 内部 poll 等 KPU 完成中断）-> 释放该缓冲
 *
 * 两种模式各自计时并对照：
 *   serial —— 单线程：AI2D(N) -> KPU(N) -> AI2D(N+1) ...（每帧 = 两个引擎耗时之和）
 *   pipe   —— 双缓冲乒乓：AI2D(N+1) 与 KPU(N) 重叠（每帧 ≈ max(两者)）
 * 一旦 pipe 的单帧耗时 < 5000us，就说明「等换高速相机后」这条链路能支撑 200fps。
 *
 * 用法（板端 msh）:
 *   /sdcard/app/kpu_bench --kmodel /sdcard/app/best.kmodel --mode both --frames 200
 *   /sdcard/app/kpu_bench --kmodel /sdcard/app/best.kmodel --mode pipe --frames 500 --vis-out 640x360
 *
 * 日志: /sdcard/app/logs/kpu_bench.log（与业务应用同一套 open/write/fsync 模式）
 *
 * 注意：它与 green_led_ai 都要独占 VICAP，同一时刻只能跑一个。
 */
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>

#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/runtime_tensor.h>
#include <nncase/functional/ai2d/ai2d_builder.h>

#include "mpi_sensor_api.h"
#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_vicap_api.h"

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::F::k230;

#define BENCH_LOG "/sdcard/app/logs/kpu_bench.log"
#define DEV       VICAP_DEV_ID_0
#define CHN_AI    VICAP_CHN_ID_0
#define SLOTS     2              /* 乒乓缓冲数 */

/* ============================ 日志 ============================ */

static int g_trace = 0;

static void blog(const char *fmt, ...)
{
    char buf[640];
    va_list ap;
    int n;
    static int dir_ok;

    if (!dir_ok) {
        mkdir("/sdcard/app/logs", 0777);
        dir_ok = 1;
    }
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if (n > (int)sizeof(buf) - 1)
        n = (int)sizeof(buf) - 1;

    fwrite(buf, 1, (size_t)n, stdout);
    fflush(stdout);

    int fd = open(BENCH_LOG, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd >= 0) {
        ssize_t w = write(fd, buf, (size_t)n);
        fsync(fd);
        close(fd);
        (void)w;
    }
}

static uint64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/* ============================ MPP 串行化 ============================ */

/*
 * 与 green_led_ai 同样的纪律：MPP 调用（dump/release/mmap/munmap）全进程串行，
 * 映射一次缓存起来、不逐帧 map/unmap。本程序里 MPP 调用只发生在 pre 线程，
 * 这里的锁是防御性的（将来加线程也不会踩坑）。
 */
static pthread_mutex_t g_mpp_lock = PTHREAD_MUTEX_INITIALIZER;

static void mpp_enter(void) { pthread_mutex_lock(&g_mpp_lock); }
static void mpp_leave(void) { pthread_mutex_unlock(&g_mpp_lock); }

struct MapEnt { uint64_t phys; uint32_t size; uint8_t *va; };
static std::vector<MapEnt> g_maps;

static uint8_t *map_persist(uint64_t phys, uint32_t size)
{
    for (auto &e : g_maps) {
        if (e.phys == phys)
            return e.size >= size ? e.va : nullptr;
    }
    uint8_t *va = (uint8_t *)kd_mpi_sys_mmap(phys, size);
    if (!va)
        return nullptr;
    g_maps.push_back({ phys, size, va });
    return va;
}

static void unmap_all(void)
{
    mpp_enter();
    for (auto &e : g_maps)
        kd_mpi_sys_munmap(e.va, e.size);
    g_maps.clear();
    mpp_leave();
}

/* ============================ 配置 ============================ */

struct cfg_t {
    const char *kmodel;
    int         csi;
    uint32_t    acq_w, acq_h;
    int         probe_fps;
    uint32_t    src_w, src_h;
    int         frames;
    int         mode;          /* 0=serial 1=pipe 2=both */
    int         sync_in;       /* 是否对输入 tensor 做 sync_write_back */
    int         exposure_us;
    int         ae;
} g_cfg = {
    .kmodel = nullptr,
    .csi = 2,
    .acq_w = 1920,
    .acq_h = 1080,
    .probe_fps = 30,
    .src_w = 640,
    .src_h = 360,
    .frames = 200,
    .mode = 2,
    .sync_in = 0,
    .exposure_us = 200,
    .ae = 0,
};

/* ============================ VICAP ============================ */

static k_vicap_sensor_info g_sensor_info;
static uint32_t            g_acq_w, g_acq_h, g_acq_fps;
static int                 g_vicap_ok;

static int vb_init(void)
{
    k_vb_config config;
    memset(&config, 0, sizeof(config));
    config.max_pool_cnt = 64;
    if (kd_mpi_vb_set_config(&config) != K_SUCCESS)
        return -1;
    k_vb_supplement_config sup;
    memset(&sup, 0, sizeof(sup));
    sup.supplement_config |= VB_SUPPLEMENT_JPEG_MASK;
    if (kd_mpi_vb_set_supplement_config(&sup) != K_SUCCESS)
        return -1;
    return kd_mpi_vb_init() == K_SUCCESS ? 0 : -1;
}

static int vicap_init(void)
{
    k_vicap_probe_config probe;
    memset(&probe, 0, sizeof(probe));
    probe.csi_num = (k_u32)g_cfg.csi;
    probe.width = g_cfg.acq_w;
    probe.height = g_cfg.acq_h;
    probe.fps = (k_u32)g_cfg.probe_fps;
    if (kd_mpi_sensor_adapt_get(&probe, &g_sensor_info) != 0) {
        blog("kpu_bench: sensor probe failed csi=%d %ux%u@%d\n", g_cfg.csi, g_cfg.acq_w,
             g_cfg.acq_h, g_cfg.probe_fps);
        return -1;
    }
    if (kd_mpi_vicap_get_sensor_info(g_sensor_info.sensor_type, &g_sensor_info) != K_SUCCESS)
        return -1;
    g_acq_w = g_sensor_info.width;
    g_acq_h = g_sensor_info.height;
    g_acq_fps = g_sensor_info.fps;

    k_vicap_dev_attr dev_attr;
    memset(&dev_attr, 0, sizeof(dev_attr));
    dev_attr.acq_win.width = g_acq_w;
    dev_attr.acq_win.height = g_acq_h;
    dev_attr.mode = VICAP_WORK_OFFLINE_MODE;
    dev_attr.buffer_num = 6;
    dev_attr.buffer_size = VB_ALIGN_UP((uint64_t)g_acq_w * g_acq_h * 2, 4096);
    dev_attr.buffer_pool_id = VB_INVALID_POOLID;
    dev_attr.pipe_ctrl.data = 0xFFFFFFFF;
    dev_attr.pipe_ctrl.bits.ae_enable = g_cfg.ae ? K_TRUE : K_FALSE;
    dev_attr.pipe_ctrl.bits.awb_enable = K_TRUE;
    dev_attr.pipe_ctrl.bits.ahdr_enable = K_FALSE;
    dev_attr.pipe_ctrl.bits.dnr3_enable = K_TRUE;
    memcpy(&dev_attr.sensor_info, &g_sensor_info, sizeof(g_sensor_info));
    if (kd_mpi_vicap_set_dev_attr(DEV, dev_attr) != K_SUCCESS) {
        blog("kpu_bench: set_dev_attr failed\n");
        return -1;
    }

    /*
     * AI 通道用 RGB_888_PLANAR（官方 usage_kpu 示例同款）：ISP 硬件就把色彩空间
     * 转好了，AI2D 只需要 resize+pad，CPU 完全不碰像素。
     */
    k_vicap_chn_attr chn;
    memset(&chn, 0, sizeof(chn));
    chn.out_win.width = g_cfg.src_w;
    chn.out_win.height = g_cfg.src_h;
    chn.crop_win.width = g_acq_w;
    chn.crop_win.height = g_acq_h;
    chn.scale_win = chn.out_win;
    chn.crop_enable = K_FALSE;
    chn.scale_enable = K_TRUE;
    chn.chn_enable = K_TRUE;
    chn.pix_format = PIXEL_FORMAT_RGB_888_PLANAR;
    chn.buffer_num = 6;
    chn.buffer_size = VB_ALIGN_UP((uint64_t)g_cfg.src_w * g_cfg.src_h * 3, 4096);
    chn.alignment = 12;
    chn.buffer_pool_id = VB_INVALID_POOLID;
    if (kd_mpi_vicap_set_chn_attr(DEV, CHN_AI, chn) != K_SUCCESS) {
        blog("kpu_bench: set_chn_attr failed\n");
        return -1;
    }
    if (kd_mpi_vicap_init(DEV) != K_SUCCESS) {
        blog("kpu_bench: vicap_init failed\n");
        return -1;
    }
    if (kd_mpi_vicap_start_stream(DEV) != K_SUCCESS) {
        blog("kpu_bench: start_stream failed\n");
        return -1;
    }

    /* 固定曝光/最低增益：和主应用一致，保证画面亮度可比 */
    k_vicap_sensor_attr sattr;
    memset(&sattr, 0, sizeof(sattr));
    sattr.dev_num = DEV;
    if (kd_mpi_vicap_get_sensor_fd(&sattr) == 0 && sattr.sensor_fd >= 0 && !g_cfg.ae) {
        if (g_cfg.exposure_us > 0) {
            k_sensor_exposure_time_range range;
            memset(&range, 0, sizeof(range));
            if (kd_mpi_sensor_get_exposure_time_range(sattr.sensor_fd, &range) == 0) {
                /* 单位统一：intg_time 用秒，range 用微秒（别混比） */
                float min_s = range.min_intg_time_us / 1000000.0f;
                float max_s = range.max_intg_time_us / 1000000.0f;
                float want = (float)g_cfg.exposure_us / 1000000.0f;
                if (want < min_s) want = min_s;
                if (want > max_s) want = max_s;
                k_sensor_intg_time t;
                memset(&t, 0, sizeof(t));
                t.intg_time[0] = want;
                blog("kpu_bench: exposure fixed %.0f us (range %.0f-%.0f us) rc=%d\n",
                     want * 1000000.0f, range.min_intg_time_us, range.max_intg_time_us,
                     (int)kd_mpi_sensor_intg_time_set(sattr.sensor_fd, t));
            }
        }
        k_sensor_gain_info gr;
        memset(&gr, 0, sizeof(gr));
        if (kd_mpi_sensor_get_gain_range(sattr.sensor_fd, &gr) == 0) {
            k_sensor_gain gain;
            memset(&gain, 0, sizeof(gain));
            gain.gain[0] = gr.min;
            blog("kpu_bench: gain fixed %.3f rc=%d\n", gr.min,
                 (int)kd_mpi_sensor_again_set(sattr.sensor_fd, gain));
        }
    }

    blog("kpu_bench: vicap ok acq=%ux%u@%u -> CHN0 RGB888_PLANAR %ux%u buffers=6\n",
         g_acq_w, g_acq_h, g_acq_fps, g_cfg.src_w, g_cfg.src_h);
    g_vicap_ok = 1;
    return 0;
}

/* ============================ 流水 ============================ */

struct slot_t {
    runtime_tensor              in;     /* AI2D 输出 = KPU 输入（pool_shared） */
    std::vector<runtime_tensor> out;    /* KPU 输出 */
    int                         state;  /* 0=空闲 1=AI2D 完成待推理 2=KPU 在用 */
    uint64_t                    t_ai2d_us;
};

static interpreter              g_interp;
static ai2d_builder            *g_builder = nullptr;
static slot_t                   g_slot[SLOTS];
static dims_t                   g_in_shape;      /* AI2D 输入 NCHW（源码尺寸） */
static dims_t                   g_model_in_shape;/* 模型输入 = AI2D 输出 */
static uint32_t                 g_in_bytes;
static int                      g_src_channels = 3;

static pthread_mutex_t          g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t           g_cond = PTHREAD_COND_INITIALIZER;
static volatile int             g_run;
static volatile int             g_frames_left;
static int                      g_mode_pipe;
static uint64_t                 g_t_ai2d_sum, g_t_ai2d_max;
static uint64_t                 g_t_kpu_sum, g_t_kpu_max;
static uint64_t                 g_frames_done;
static uint64_t                 g_dump_fail;

static int ai2d_in_bytes(void)
{
    return (int)(g_in_shape[0] * g_in_shape[1] * g_in_shape[2] * g_in_shape[3]);
}

/* 取一个空闲 slot；返回下标或 -1（退出） */
static int slot_wait_free(void)
{
    int idx = -1;
    pthread_mutex_lock(&g_lock);
    while (g_run) {
        for (int i = 0; i < SLOTS; ++i) {
            if (g_slot[i].state == 0) {
                idx = i;
                g_slot[i].state = 2;   /* 先占住，AI2D 期间不让别人用 */
                break;
            }
        }
        if (idx >= 0)
            break;
        pthread_cond_wait(&g_cond, &g_lock);
    }
    pthread_mutex_unlock(&g_lock);
    return idx;
}

static int slot_wait_ready(void)
{
    int idx = -1;
    pthread_mutex_lock(&g_lock);
    for (;;) {
        for (int i = 0; i < SLOTS; ++i) {
            if (g_slot[i].state == 1) {
                idx = i;
                g_slot[i].state = 2;
                break;
            }
        }
        if (idx >= 0)
            break;
        if (!g_run)          /* 采集侧已收工且没有就绪缓冲：结束 */
            break;
        pthread_cond_wait(&g_cond, &g_lock);
    }
    pthread_mutex_unlock(&g_lock);
    return idx;
}

static void slot_set_state(int idx, int st)
{
    pthread_mutex_lock(&g_lock);
    g_slot[idx].state = st;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
}

static int frames_left_dec(void)
{
    int v;
    pthread_mutex_lock(&g_lock);
    v = g_frames_left;
    if (v > 0)
        g_frames_left = v - 1;
    pthread_mutex_unlock(&g_lock);
    return v;
}

/* AI2D 一步：dump 一帧 -> 零拷贝绑定 -> invoke -> 归还帧 */
static int stage_ai2d(int slot_idx)
{
    k_video_frame_info frame;
    memset(&frame, 0, sizeof(frame));

    mpp_enter();
    k_s32 ret = kd_mpi_vicap_dump_frame(DEV, CHN_AI, VICAP_DUMP_YUV, &frame, 200);
    mpp_leave();
    if (ret != K_SUCCESS) {
        g_dump_fail++;
        if (g_dump_fail % 50u == 1u)
            blog("kpu_bench: dump failed ret=0x%08x (fail=%llu)\n", (unsigned)ret,
                (unsigned long long)g_dump_fail);
        usleep(2000);
        return -1;
    }

    uint32_t w = frame.v_frame.width;
    uint32_t h = frame.v_frame.height;
    if (w != g_cfg.src_w || h != g_cfg.src_h) {
        blog("kpu_bench: frame %ux%u != config %ux%u, skip\n", w, h, g_cfg.src_w,
             g_cfg.src_h);
        mpp_enter();
        kd_mpi_vicap_dump_release(DEV, CHN_AI, &frame);
        mpp_leave();
        return -1;
    }
    uint32_t stride0 = frame.v_frame.stride[0] ? frame.v_frame.stride[0] : w;
    uint64_t phys0 = frame.v_frame.phys_addr[0];

    /* 平面必须连续且行跨度==宽，才能当成一个 NCHW uint8 tensor 用 */
    int contiguous = (stride0 == w) &&
                     (frame.v_frame.phys_addr[1] == phys0 + (uint64_t)w * h) &&
                     (frame.v_frame.phys_addr[2] == phys0 + 2ull * w * h);

    if (g_frames_done < 3 || g_trace) {
        blog("kpu_bench: frame %ux%u stride=%u/%u/%u phys=0x%llx,0x%llx,0x%llx contiguous=%d\n",
             w, h, frame.v_frame.stride[0], frame.v_frame.stride[1], frame.v_frame.stride[2],
             (unsigned long long)frame.v_frame.phys_addr[0],
             (unsigned long long)frame.v_frame.phys_addr[1],
             (unsigned long long)frame.v_frame.phys_addr[2], contiguous);
    }

    uint64_t t0 = mono_us();
    if (!contiguous) {
        blog("kpu_bench: 平面不连续，无法零拷贝绑定（需要 phys1=phys0+w*h 且 phys2=phys1+w*h）\n");
        mpp_enter();
        kd_mpi_vicap_dump_release(DEV, CHN_AI, &frame);
        mpp_leave();
        return -1;
    }

    uint8_t *va = nullptr;
    mpp_enter();
    va = map_persist(phys0, (uint32_t)((uint64_t)w * h * 3));
    mpp_leave();
    if (!va) {
        mpp_enter();
        kd_mpi_vicap_dump_release(DEV, CHN_AI, &frame);
        mpp_leave();
        return -1;
    }

    /* 零拷贝：直接用 VICAP 帧的物理地址当 AI2D 输入 tensor（不 memcpy） */
    dims_t shape{ 1, (size_t)g_src_channels, (size_t)h, (size_t)w };
    auto in_res = host_runtime_tensor::create(
        typecode_t::dt_uint8, shape,
        { (gsl::byte *)va, (size_t)ai2d_in_bytes() },
        false, hrt::pool_shared, (uintptr_t)phys0);
    if (!in_res.is_ok()) {
        blog("kpu_bench: hrt::create over vicap frame failed\n");
        mpp_enter();
        kd_mpi_vicap_dump_release(DEV, CHN_AI, &frame);
        mpp_leave();
        return -1;
    }
    runtime_tensor in_tensor = in_res.unwrap();

    if (g_cfg.sync_in)
        (void)hrt::sync(in_tensor, sync_op_t::sync_write_back, true);

    auto r = g_builder->invoke(in_tensor, g_slot[slot_idx].in);
    uint64_t t1 = mono_us();

    /* AI2D invoke 内部等硬件中断返回 -> 输入帧已经读完，可以立即归还 VICAP */
    mpp_enter();
    kd_mpi_vicap_dump_release(DEV, CHN_AI, &frame);
    mpp_leave();

    if (!r.is_ok()) {
        blog("kpu_bench: ai2d invoke failed\n");
        return -1;
    }
    (void)t0;
    g_slot[slot_idx].t_ai2d_us = t1 - t0;

    pthread_mutex_lock(&g_lock);
    g_t_ai2d_sum += (t1 - t0);
    if (t1 - t0 > g_t_ai2d_max)
        g_t_ai2d_max = t1 - t0;
    pthread_mutex_unlock(&g_lock);
    return 0;
}

/* KPU 一步：用已就绪的输入 tensor 跑模型 */
static int stage_kpu(int slot_idx)
{
    auto iset = g_interp.input_tensor(0, g_slot[slot_idx].in);
    if (!iset.is_ok()) {
        blog("kpu_bench: set input tensor failed\n");
        return -1;
    }
    for (size_t i = 0; i < g_slot[slot_idx].out.size(); ++i) {
        auto os = g_interp.output_tensor(i, g_slot[slot_idx].out[i]);
        if (!os.is_ok()) {
            blog("kpu_bench: set output tensor %u failed\n", (unsigned)i);
            return -1;
        }
    }

    uint64_t t0 = mono_us();
    auto r = g_interp.run();      /* 内部 poll 等 KPU 完成中断 */
    uint64_t t1 = mono_us();
    if (!r.is_ok()) {
        blog("kpu_bench: interp.run failed\n");
        return -1;
    }

    pthread_mutex_lock(&g_lock);
    g_t_kpu_sum += (t1 - t0);
    if (t1 - t0 > g_t_kpu_max)
        g_t_kpu_max = t1 - t0;
    g_frames_done++;
    pthread_mutex_unlock(&g_lock);
    return (int)(t1 - t0);
}

static void *pre_thread(void *arg)
{
    (void)arg;
    while (g_run) {
        if (frames_left_dec() <= 0)
            break;
        int s = slot_wait_free();
        if (s < 0)
            break;
        if (stage_ai2d(s) == 0)
            slot_set_state(s, 1);      /* 交 KPU */
        else
            slot_set_state(s, 0);
    }
    /* 收工：置 g_run=0 并唤醒 kpu 线程，让它把最后一个就绪缓冲处理完 */
    pthread_mutex_lock(&g_lock);
    g_run = 0;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
    return nullptr;
}

static void *kpu_thread(void *arg)
{
    (void)arg;
    for (;;) {
        int s = slot_wait_ready();
        if (s < 0)
            break;
        stage_kpu(s);
        slot_set_state(s, 0);
    }
    return nullptr;
}

/* 串行模式：单线程 AI2D -> KPU -> 下一帧（无重叠） */
static void run_serial(int frames, uint64_t *ai2d_avg, uint64_t *kpu_avg, uint64_t *frame_avg)
{
    g_t_ai2d_sum = g_t_kpu_sum = g_frames_done = 0;
    g_t_ai2d_max = g_t_kpu_max = 0;
    g_dump_fail = 0;

    uint64_t t_start = mono_us();
    for (int i = 0; i < frames; ++i) {
        if (stage_ai2d(0) != 0)
            continue;
        int k = stage_kpu(0);
        (void)k;
        g_slot[0].state = 0;
    }
    uint64_t t_end = mono_us();

    uint64_t n = g_frames_done ? g_frames_done : 1;
    *ai2d_avg = g_t_ai2d_sum / n;
    *kpu_avg = g_t_kpu_sum / n;
    *frame_avg = (t_end - t_start) / n;
}

static void run_pipe(int frames, uint64_t *ai2d_avg, uint64_t *kpu_avg, uint64_t *frame_avg)
{
    g_t_ai2d_sum = g_t_kpu_sum = g_frames_done = 0;
    g_t_ai2d_max = g_t_kpu_max = 0;
    g_dump_fail = 0;
    for (int i = 0; i < SLOTS; ++i)
        g_slot[i].state = 0;

    g_run = 1;
    g_frames_left = frames;
    g_mode_pipe = 1;

    pthread_t pre, kpu;
    uint64_t t_start = mono_us();
    pthread_create(&pre, nullptr, pre_thread, nullptr);
    pthread_create(&kpu, nullptr, kpu_thread, nullptr);
    pthread_join(pre, nullptr);
    pthread_join(kpu, nullptr);
    uint64_t t_end = mono_us();

    uint64_t n = g_frames_done ? g_frames_done : 1;
    *ai2d_avg = g_t_ai2d_sum / n;
    *kpu_avg = g_t_kpu_sum / n;
    *frame_avg = (t_end - t_start) / n;
    (void)g_mode_pipe;
}

/* ============================ 初始化 nncase ============================ */

static int nncase_init(const char *kmodel_path)
{
    std::ifstream ifs(kmodel_path, std::ios::binary);
    if (!ifs.good()) {
        blog("kpu_bench: 打不开 kmodel: %s\n", kmodel_path);
        return -1;
    }
    auto lr = g_interp.load_model(ifs);
    if (!lr.is_ok()) {
        blog("kpu_bench: load_model failed: %s\n", kmodel_path);
        return -1;
    }

    blog("kpu_bench: kmodel=%s inputs=%u outputs=%u\n", kmodel_path,
         (unsigned)g_interp.inputs_size(), (unsigned)g_interp.outputs_size());

    for (size_t i = 0; i < g_interp.inputs_size(); ++i) {
        auto sh = g_interp.input_shape(i);
        auto desc = g_interp.input_desc(i);
        std::string s;
        for (size_t j = 0; j < sh.size(); ++j)
            s += std::to_string(sh[j]) + (j + 1 < sh.size() ? "x" : "");
        blog("kpu_bench: input[%u] shape=%s dtype=%d\n", (unsigned)i, s.c_str(),
             (int)desc.datatype);
        if (i == 0)
            g_model_in_shape = sh;
    }
    for (size_t i = 0; i < g_interp.outputs_size(); ++i) {
        auto sh = g_interp.output_shape(i);
        auto desc = g_interp.output_desc(i);
        std::string s;
        for (size_t j = 0; j < sh.size(); ++j)
            s += std::to_string(sh[j]) + (j + 1 < sh.size() ? "x" : "");
        blog("kpu_bench: output[%u] shape=%s dtype=%d\n", (unsigned)i, s.c_str(),
             (int)desc.datatype);
    }

    if (g_model_in_shape.size() != 4) {
        blog("kpu_bench: 只支持 4 维输入模型\n");
        return -1;
    }

    /* AI2D 输入：源码尺寸的 NCHW uint8（RGB888 planar 就是一个 NCHW 张量） */
    g_in_shape = dims_t{ 1, (size_t)g_src_channels, (size_t)g_cfg.src_h, (size_t)g_cfg.src_w };
    g_in_bytes = (uint32_t)ai2d_in_bytes();

    /* letterbox 参数（与官方示例同一算法） */
    int in_w = (int)g_model_in_shape[3];
    int in_h = (int)g_model_in_shape[2];
    float ratiow = (float)in_w / (float)g_cfg.src_w;
    float ratioh = (float)in_h / (float)g_cfg.src_h;
    float ratio = ratiow < ratioh ? ratiow : ratioh;
    int new_w = (int)(ratio * g_cfg.src_w);
    int new_h = (int)(ratio * g_cfg.src_h);
    float dw = (float)(in_w - new_w) / 2;
    float dh = (float)(in_h - new_h) / 2;
    int top = (int)(roundf(0));
    int bottom = (int)(roundf(dh * 2 + 0.1f));
    int left = (int)(roundf(0));
    int right = (int)(roundf(dw * 2 - 0.1f));

    ai2d_datatype_t ai2d_dtype{ ai2d_format::NCHW_FMT, ai2d_format::NCHW_FMT,
                                typecode_t::dt_uint8, typecode_t::dt_uint8 };
    ai2d_crop_param_t crop_param{ false, 0, 0, 0, 0 };
    ai2d_shift_param_t shift_param{ false, 0 };
    ai2d_pad_param_t pad_param{ true,
                                { { 0, 0 }, { 0, 0 }, { top, bottom }, { left, right } },
                                ai2d_pad_mode::constant, { 114, 114, 114 } };
    ai2d_resize_param_t resize_param{ true, ai2d_interp_method::tf_bilinear,
                                      ai2d_interp_mode::half_pixel };
    ai2d_affine_param_t affine_param{ false, ai2d_interp_method::cv2_bilinear, 0, 0, 127, 1,
                                      { 0.5f, 0.1f, 0.0f, 0.1f, 0.5f, 0.0f } };

    g_builder = new ai2d_builder(g_in_shape, g_model_in_shape, ai2d_dtype, crop_param,
                                 shift_param, pad_param, resize_param, affine_param);
    auto br = g_builder->build_schedule();
    if (!br.is_ok()) {
        blog("kpu_bench: ai2d build_schedule failed\n");
        return -1;
    }
    blog("kpu_bench: ai2d %ux%ux%d -> %ux%ux%d (resize+pad letterbox, 硬件)\n",
         g_cfg.src_w, g_cfg.src_h, g_src_channels, in_w, in_h, (int)g_model_in_shape[1]);

    /* 乒乓缓冲：AI2D 输出与 KPU 输入是同一个 tensor（零拷贝共用一份内存） */
    for (int i = 0; i < SLOTS; ++i) {
        auto res = host_runtime_tensor::create(typecode_t::dt_uint8, g_model_in_shape,
                                              hrt::pool_shared);
        if (!res.is_ok()) {
            blog("kpu_bench: create input tensor failed\n");
            return -1;
        }
        g_slot[i].in = res.unwrap();
        g_slot[i].state = 0;
        g_slot[i].t_ai2d_us = 0;

        for (size_t k = 0; k < g_interp.outputs_size(); ++k) {
            auto od = host_runtime_tensor::create(g_interp.output_desc(k).datatype,
                                                 g_interp.output_shape(k), hrt::pool_shared);
            if (!od.is_ok()) {
                blog("kpu_bench: create output tensor %u failed\n", (unsigned)k);
                return -1;
            }
            g_slot[i].out.push_back(od.unwrap());
        }
    }
    return 0;
}

/* ============================ 收尾 ============================ */

static void bench_cleanup(void)
{
    unmap_all();
    mpp_enter();
    kd_mpi_vicap_stop_stream(DEV);
    kd_mpi_vicap_deinit(DEV);
    kd_mpi_vb_exit();
    mpp_leave();
}

/* ============================ main ============================ */

static void usage(void)
{
    blog("usage: kpu_bench --kmodel <path> [options]\n");
    blog("  --kmodel <path>      kmodel 路径（必填）\n");
    blog("  --csi <0-2>          CSI 号 (默认 2)\n");
    blog("  --vis-out <WxH>      AI 通道尺寸 (默认 640x360，必须 8 对齐)\n");
    blog("  --frames <n>         每种模式测多少帧 (默认 200)\n");
    blog("  --mode <serial|pipe|both>  默认 both\n");
    blog("  --sync <0|1>         是否对输入 tensor 做 sync_write_back (默认 0)\n");
    blog("  --expo-us <us>       固定曝光 (默认 200)\n");
    blog("  --ae                 自动曝光\n");
    blog("  --trace              打印每帧平面地址等细节\n");
}

int main(int argc, char **argv)
{
    static const struct option opts[] = {
        { "kmodel",   required_argument, 0, 'k' },
        { "csi",      required_argument, 0, 'c' },
        { "vis-out",  required_argument, 0, 'v' },
        { "frames",   required_argument, 0, 'f' },
        { "mode",     required_argument, 0, 'm' },
        { "sync",     required_argument, 0, 's' },
        { "expo-us",  required_argument, 0, 'e' },
        { "ae",       no_argument,       0, 'A' },
        { "trace",    no_argument,       0, 't' },
        { "help",     no_argument,       0, 'h' },
        { 0, 0, 0, 0 }
    };

    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, SIG_IGN);
    signal(SIGPIPE, SIG_IGN);

    int c;
    while ((c = getopt_long(argc, argv, "", opts, NULL)) != -1) {
        switch (c) {
        case 'k': g_cfg.kmodel = optarg; break;
        case 'c': g_cfg.csi = atoi(optarg); break;
        case 'v': {
            unsigned w = 0, h = 0;
            if (sscanf(optarg, "%ux%u", &w, &h) == 2) {
                g_cfg.src_w = w & ~7u;
                g_cfg.src_h = h & ~1u;
            }
            break;
        }
        case 'f': g_cfg.frames = atoi(optarg); break;
        case 'm':
            g_cfg.mode = strcmp(optarg, "serial") == 0 ? 0
                        : strcmp(optarg, "pipe") == 0   ? 1
                                                        : 2;
            break;
        case 's': g_cfg.sync_in = atoi(optarg); break;
        case 'e': g_cfg.exposure_us = atoi(optarg); break;
        case 'A': g_cfg.ae = 1; break;
        case 't': g_trace = 1; break;
        case 'h': usage(); return 0;
        default: usage(); return 1;
        }
    }

    blog("\n==== kpu_bench start pid=%d ====\n", (int)getpid());
    if (!g_cfg.kmodel) {
        blog("kpu_bench: 缺少 --kmodel\n");
        usage();
        return 1;
    }
    if (g_cfg.frames <= 0)
        g_cfg.frames = 200;

    if (vb_init() != 0) {
        blog("kpu_bench: vb init failed\n");
        return 1;
    }
    if (vicap_init() != 0) {
        mpp_enter();
        kd_mpi_vb_exit();
        mpp_leave();
        blog("==== kpu_bench done ====\n");
        return 1;
    }

    if (nncase_init(g_cfg.kmodel) != 0) {
        bench_cleanup();
        blog("==== kpu_bench done ====\n");
        return 1;
    }

    /* 丢掉前几帧，等 ISP/曝光稳定 */
    for (int i = 0; i < 8; ++i) {
        k_video_frame_info f;
        memset(&f, 0, sizeof(f));
        mpp_enter();
        k_s32 r = kd_mpi_vicap_dump_frame(DEV, CHN_AI, VICAP_DUMP_YUV, &f, 200);
        mpp_leave();
        if (r == K_SUCCESS) {
            mpp_enter();
            kd_mpi_vicap_dump_release(DEV, CHN_AI, &f);
            mpp_leave();
        }
    }

    uint64_t a_s = 0, k_s = 0, f_s = 0;
    uint64_t a_p = 0, k_p = 0, f_p = 0;

    if (g_cfg.mode == 0 || g_cfg.mode == 2) {
        run_serial(g_cfg.frames, &a_s, &k_s, &f_s);
        blog("kpu_bench: [serial] frames=%llu ai2d=%llu us(max %llu) kpu=%llu us(max %llu) "
             "per_frame=%llu us -> %.1f fps\n",
             (unsigned long long)g_frames_done, (unsigned long long)a_s,
             (unsigned long long)g_t_ai2d_max, (unsigned long long)k_s,
             (unsigned long long)g_t_kpu_max, (unsigned long long)f_s,
             f_s ? 1000000.0 / (double)f_s : 0.0);
    }
    if (g_cfg.mode == 1 || g_cfg.mode == 2) {
        run_pipe(g_cfg.frames, &a_p, &k_p, &f_p);
        blog("kpu_bench: [pipe  ] frames=%llu ai2d=%llu us(max %llu) kpu=%llu us(max %llu) "
             "per_frame=%llu us -> %.1f fps\n",
             (unsigned long long)g_frames_done, (unsigned long long)a_p,
             (unsigned long long)g_t_ai2d_max, (unsigned long long)k_p,
             (unsigned long long)g_t_kpu_max, (unsigned long long)f_p,
             f_p ? 1000000.0 / (double)f_p : 0.0);
    }

    if (g_cfg.mode == 2 && f_p) {
        uint64_t serial_sum = a_s + k_s;
        blog("kpu_bench: 结论 serial(ai2d+kpu)=%llu us, pipe=%llu us, 重叠收益=%lld us(%lld%%)\n",
             (unsigned long long)serial_sum, (unsigned long long)f_p,
             (long long)serial_sum - (long long)f_p,
             serial_sum ? (long long)((serial_sum - f_p) * 100 / serial_sum) : 0);
        blog("kpu_bench: 200fps 需要单帧 <= 5000 us -> pipe %s\n",
             f_p <= 5000 ? "PASS" : "FAIL(需要更小模型/更低输入分辨率)");
    }

    bench_cleanup();
    blog("==== kpu_bench done ====\n");
    (void)g_vicap_ok;
    return 0;
}
