/*
 * vision —— 实现
 */
#include "vision.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "app.h"
#include "detect.h"
#include "mpi_vb_api.h"
#include <time.h>

#include "record.h"
#include "vicap_src.h"

/* ============================ 状态 ============================ */

static volatile int g_run;          /* 线程存活 */
static volatile int g_enabled;      /* 视觉启用（运行期可切） */

static pthread_t       g_cap_tid, g_proc_tid;
static int             g_cap_started, g_proc_started;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond = PTHREAD_COND_INITIALIZER;

/* 单槽信箱：新帧优先（旧帧立即归还，避免引入延迟） */
static k_video_frame_info g_slot;
static uint64_t           g_slot_seq;
static uint64_t           g_slot_mono_us;
static int                g_slot_have;

static const detector_t *g_det;
static vision_stats_t    g_stats;
static uint64_t          g_produced;      /* 已投递帧数（含丢弃） */
static uint64_t          g_frame_seq;

/* 最近几次 dump 成功的物理地址：停流诊断时用来看 VB 块的引用计数是否卡住 */
#define DIAG_PHYS_N 6
static uint64_t g_diag_phys[DIAG_PHYS_N];
static uint32_t g_diag_phys_n;
static uint64_t g_fail_dump_us_sum;   /* 失败 dump 的耗时累计（区分「等到超时」与「立刻报错」） */
static uint64_t g_fail_dump_cnt;
static uint64_t g_last_dump_us;
static uint32_t g_dump_call_us;       /* 最近一次 dump 调用耗时（阻塞等帧的时长） */
static uint32_t g_dump_timeouts;      /* dump 等不到帧的次数（流水异常指标，稳态应不增长） */
static int      g_stall_reported;

/*
 * 归还环：**VICAP 只由采集线程调用** —— dump 和 release 都在它手里，因此不需要
 * 任何锁；上一轮"阻塞等帧占着全局 MPP 锁"把整链压到 13fps 的根因就此消失。
 * 处理线程 / 异步后端 / 录像回调用完一帧后推进这个环，由采集线程统一 release。
 */
#define RECYCLE_N 8
static k_video_frame_info g_recyc[RECYCLE_N];
static int      g_recyc_n;
static uint64_t g_recycled;           /* 已归还帧数 */
static uint64_t g_recycle_leak;       /* 环满被丢弃（会漏一个 VB 块，必须为 0） */


static int      g_found_prev;
static detect_out_t g_last;          /* 最近一帧的检测明细（探针/亮度） */
static int      g_last_valid;
static uint32_t g_misses;
static uint64_t g_period_start_us;
static uint32_t g_period_frames;
static uint64_t g_period_detect_us;
static uint32_t g_period_detect_max;
static uint64_t g_period_lat_sum;
static uint32_t g_period_lat_max;
static uint32_t g_lat_max_all;

/* ============================ 平面映射 ============================ */

typedef struct {
    const uint8_t *y;
    const uint8_t *uv;
} plane_map_t;

/*
 * 把一帧映射成可读的虚拟地址。规则（旧工程就是在这里打挂内核的）：
 *   1) 映射长度必须覆盖**实际会读到的全部字节**，绝不能只映射 Y 平面；
 *   2) 连续布局一次映射整帧，分段布局才分别映射；
 *   3) 映射走 mpp_map_persist（按物理地址缓存、只映射一次、永不逐帧解映射）。
 *
 * 两种像素格式的帧大小：
 *   NV12 (PIXEL_FORMAT_YUV_SEMIPLANAR_420): Y = stride*height，UV = Y/2
 *   RGB_888_PLANAR                        : 三个平面各 stride*height（kpu 后端用）
 */
static int plane_map(const k_video_frame_info *f, uint32_t stride, uint32_t height,
                     plane_map_t *m)
{
    uint32_t ysize = stride * height;
    uint64_t y_phys = f->v_frame.phys_addr[0];
    uint64_t uv_phys = f->v_frame.phys_addr[1];

    m->y = NULL;
    m->uv = NULL;

    if (f->v_frame.pixel_format == PIXEL_FORMAT_RGB_888_PLANAR) {
        /* 平面张量：AI2D 的 NCHW uint8 输入要一整块 3*W*H，映射也必须给够 */
        mpp_enter();
        m->y = (const uint8_t *)mpp_map_persist_cached(y_phys, ysize * 3u);
        if (m->y)
            mpp_invalidate(y_phys, (void *)m->y, ysize * 3u);
        mpp_leave();
        return m->y ? 0 : -1;
    }

    uint32_t usize = ysize / 2;
    int      contiguous = (!uv_phys || uv_phys == y_phys + ysize);

    mpp_enter();
    uint8_t *y = (uint8_t *)mpp_map_persist_cached(y_phys,
                                                   contiguous ? (ysize + usize) : ysize);
    if (y) {
        m->y = y;
        if (contiguous) {
            m->uv = y + ysize;      /* 与 Y 同一次映射之内，不会越界 */
            /* DMA 刚写完这一帧：先把旧 cache 行作废，CPU 才能读到新数据 */
            mpp_invalidate(y_phys, y, ysize + usize);
        } else {
            m->uv = (const uint8_t *)mpp_map_persist_cached(uv_phys, usize);
            mpp_invalidate(y_phys, y, ysize);
            if (m->uv)
                mpp_invalidate(uv_phys, (void *)m->uv, usize);
        }
    }
    mpp_leave();

    return (m->y && m->uv) ? 0 : -1;
}

/* 停流诊断（定义在文件后半部分，采集线程里要用） */
static void vision_diag_stall(int raw, uint64_t stall_us);

/* ============================ 采集线程 ============================ */

/* 帧用完：推进归还环（采集线程负责真正 release）。任何线程都可以调，不阻塞。 */
void vision_recycle_frame(const k_video_frame_info *f)
{
    pthread_mutex_lock(&g_lock);
    if (g_recyc_n < RECYCLE_N) {
        g_recyc[g_recyc_n++] = *f;
        g_recycled++;
        pthread_cond_broadcast(&g_cond);
    } else {
        g_recycle_leak++;      /* 设计上不会发生：在途帧最多 2~3 个 */
    }
    pthread_mutex_unlock(&g_lock);
}

/* 录像回调：copy 模式拷完立刻回调；borrow 模式送帧返回后回调 */
static void vision_rec_done(const k_video_frame_info *f, void *ctx)
{
    (void)ctx;
    vision_recycle_frame(f);
}

/* 等 ms 毫秒：只在流水异常（dump 立刻报错）时退避，正常路径完全不用 */
static void vision_wait_ms(int ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += ms / 1000;
    ts.tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_nsec -= 1000000000L; ts.tv_sec++; }
    pthread_mutex_lock(&g_lock);
    if (g_run)
        (void)pthread_cond_timedwait(&g_cond, &g_lock, &ts);
    pthread_mutex_unlock(&g_lock);
}

static void *vis_cap_thread(void *arg)
{
    (void)arg;

    while (g_run) {
        /*
         * 1) 先把处理线程用完的帧统一归还 —— VICAP 的 dump 与 release 都在本线程，
         *    既不需要锁，也不会出现"别人拿着锁等帧"的互相阻塞。
         */
        k_video_frame_info rel[RECYCLE_N];
        int nrel = 0;
        pthread_mutex_lock(&g_lock);
        while (g_recyc_n > 0 && nrel < RECYCLE_N)
            rel[nrel++] = g_recyc[--g_recyc_n];
        pthread_mutex_unlock(&g_lock);
        for (int i = 0; i < nrel; ++i)
            vicap_release(VICAP_CHN_VISION, &rel[i]);

        /* 2) 识别关掉时不空转：等"开关变了"这个事件（不是 sleep 轮询）。
         *    但归还环一有新帧就先回顶部把它 release 掉，别压着 VICAP 缓冲。 */
        pthread_mutex_lock(&g_lock);
        while (g_run && !g_enabled && g_recyc_n == 0)
            pthread_cond_wait(&g_cond, &g_lock);
        int still_off = !g_enabled;
        pthread_mutex_unlock(&g_lock);
        if (!g_run)
            break;
        if (still_off)
            continue;

        /*
         * 3) 取帧：dump 本身就是"阻塞等下一帧"，这就是事件驱动 —— 不 sleep、
         *    不轮询，也不拿超时当节流用。g_cfg.dump_timeout_ms 只是"进程能停
         *    下来 / 流水真出事时能报出来"的兜底（默认 200ms）；稳态下一帧一返回。
         */
        k_video_frame_info f;
        memset(&f, 0, sizeof(f));
        int raw = 0;
        uint64_t t_dump0 = mono_us();
        int rc = vicap_dump_soft(VICAP_CHN_VISION, &f, g_cfg.dump_timeout_ms, &raw);
        g_dump_call_us = (uint32_t)(mono_us() - t_dump0);
        if (rc != 0) {
            g_fail_dump_us_sum += mono_us() - t_dump0;
            g_fail_dump_cnt++;
            if (rc > 0) {
                /* rc==1: BUF_EMPTY（此刻没帧，常态）
                 * rc==2: NOTREADY（流水未就绪/被反压卡住，必须单独计数并报警） */
                if (rc == 2) {
                    g_stats.notready++;
                    if (g_stats.notready % 100u == 1u)
                        app_log(APP_NAME_STR ": vision dump 返回 NOTREADY（流水未就绪，"
                                "累计 %llu 次）—— 通常是录像/编码侧反压把 ISP 卡住了\n",
                                (unsigned long long)g_stats.notready);
                } else {
                    g_stats.noframe++;
                }
                /*
                 * 只有「曾经成功出过帧」并且「连续 3 秒没有新帧」才算停流。
                 * （上一版写成 dumped==0 也算，结果启动瞬间第一次空转就误触发，
                 *   直接把录像解绑了 —— 那次的"问题在绑定"结论是假的。）
                 */
                uint64_t now_us = mono_us();
                if (g_last_dump_us && now_us - g_last_dump_us > 3000000ull &&
                    !g_stall_reported) {
                    g_stall_reported = 1;
                    vision_diag_stall(raw, now_us - g_last_dump_us);
                }
            }
            if (rc < 0) {
                g_stats.dump_fail++;
                if (g_stats.dump_fail % 50u == 1u)
                    app_log(APP_NAME_STR ": vision dump failed ret=0x%08x (fail=%llu)\n",
                            (unsigned)raw, (unsigned long long)g_stats.dump_fail);
            }
            g_dump_timeouts++;
            g_stats.dump_timeouts = g_dump_timeouts;
            if (g_dump_timeouts % 100u == 1u)
                app_log(APP_NAME_STR ": vision dump 等不到帧（累计 %u 次，call=%uus）—— "
                        "只有流水异常才会走到这里\n", g_dump_timeouts, g_dump_call_us);
            /*
             * 只有"立刻返回的错误"才需要退避（每秒上千次空转会烧 CPU）；
             * dump 自己等到超时的情况直接重试 —— 它已经等过了，不需要再睡。
             */
            if (g_dump_call_us < 1000u)
                vision_wait_ms(5);
            continue;
        }
        g_last_dump_us = mono_us();
        g_stall_reported = 0;
        g_stats.dumped++;
        g_diag_phys[g_diag_phys_n % DIAG_PHYS_N] = f.v_frame.phys_addr[0];
        g_diag_phys_n++;

        uint64_t seq = ++g_frame_seq;

        k_video_frame_info old;
        int have_old = 0;
        pthread_mutex_lock(&g_lock);
        if (!g_run || !g_enabled) {
            pthread_mutex_unlock(&g_lock);
            vicap_release(VICAP_CHN_VISION, &f);   /* 关停瞬间：归还本帧 */
            continue;
        }
        if (g_slot_have) {
            /* 处理器还没取走上一帧：作废旧帧（新帧优先），绝不阻塞采集 */
            old = g_slot;
            g_slot_have = 0;
            have_old = 1;
            g_stats.dropped++;
        }
        g_slot = f;
        g_slot_seq = seq;
        g_slot_mono_us = mono_us();
        g_slot_have = 1;
        g_produced++;
        pthread_cond_signal(&g_cond);
        pthread_mutex_unlock(&g_lock);
        if (have_old)
            vicap_release(VICAP_CHN_VISION, &old);

        trace_log(seq, "cap dump ok w=%u h=%u stride=%u phys0=0x%llx",
                  f.v_frame.width, f.v_frame.height, f.v_frame.stride[0],
                  (unsigned long long)f.v_frame.phys_addr[0]);
    }
    return NULL;
}

/* ============================ 停流诊断 ============================ */

/*
 * 视觉通道连续拿不到帧时调用：把判定所需的信息一次性打进日志。
 * 只报告事实、不做任何自动处置（--stall-fallback 那套"自动解绑录像"已经删掉：
 * 解绑后 CHN1 仍使能且无人消费，ISP 照样停，那次的结论是假的）。
 */
static void vision_diag_stall(int raw, uint64_t stall_us)
{
    char blk[320];
    int  off = 0;
    blk[0] = 0;

    mpp_enter();
    uint32_t n = g_diag_phys_n < DIAG_PHYS_N ? g_diag_phys_n : DIAG_PHYS_N;
    for (uint32_t i = 0; i < n && off < (int)sizeof(blk) - 48; ++i) {
        uint64_t phys = g_diag_phys[i];
        k_vb_blk_handle h = kd_mpi_vb_phyaddr_to_handle(phys);
        int ref = -1, usr = -1;
        if (h != VB_INVALID_HANDLE) {
            ref = (int)kd_mpi_vb_get_block_refcnt(h);
            usr = (int)kd_mpi_vb_inquire_user_cnt(h);
        }
        off += snprintf(blk + off, sizeof(blk) - (size_t)off, " 0x%llx(ref=%d,usr=%d)",
                        (unsigned long long)phys, ref, usr);
    }
    k_vicap_chn_attr ca;
    memset(&ca, 0, sizeof(ca));
    int got_attr = (kd_mpi_vicap_get_chn_attr(VICAP_DEV_ID_0, VICAP_CHN_VISION, &ca)
                    == K_SUCCESS);
    mpp_leave();
    if (got_attr) {
        static char attr_buf[160];
        snprintf(attr_buf, sizeof(attr_buf),
                 "en=%d out=%ux%u fmt=%d buf=%u sz=%u align=%u",
                 (int)ca.chn_enable, ca.out_win.width, ca.out_win.height,
                 (int)ca.pix_format, ca.buffer_num, ca.buffer_size, ca.alignment);
        app_log(APP_NAME_STR ": 视觉停流诊断 CHN0 属性: %s\n", attr_buf);
    }

    record_stats_t rs;
    record_get_stats(&rs);

    app_log(APP_NAME_STR ": 视觉停流诊断: raw=0x%08x 已停 %.1fs dumped=%llu noframe=%llu "
            "notready=%llu rel_fail=%llu recycled=%llu leak=%llu dump_timeout=%u call=%uus | "
            "失败 dump 平均耗时 %llu us（≈超时上限说明是在等帧，≈0 说明通道立刻报错）| "
            "CHN0 attr: %s | CHN0 缓冲引用:%s | rec feed=%s sess=%u streams=%llu\n",
            (unsigned)raw, (double)stall_us / 1e6,
            (unsigned long long)g_stats.dumped, (unsigned long long)g_stats.noframe,
            (unsigned long long)g_stats.notready,
            (unsigned long long)vicap_release_fail_count(),
            (unsigned long long)g_recycled, (unsigned long long)g_recycle_leak,
            g_dump_timeouts, g_dump_call_us,
            (unsigned long long)(g_fail_dump_cnt ? g_fail_dump_us_sum / g_fail_dump_cnt : 0),
            got_attr ? "ok" : "get_chn_attr 失败",
            blk[0] ? blk : "(还没有成功 dump 过)",
            rs.feed ? "borrow" : "copy", rs.session_index,
            (unsigned long long)rs.streams);

    /*
     * 这里刻意**不做任何自动处置**。上一版会"自动解绑录像再看视觉是否恢复"，
     * 但解绑后 CHN1 仍处于使能且无人消费的状态，ISP 照样停 —— 于是它给出了
     * "问题在 CHN0 自身"的假结论。诊断只负责把事实打进日志，处置由人决定。
     */

}

/* ============================ 处理线程 ============================ */

static void vision_log_result(const detect_out_t *d);

/* 结果记账：统计 + 发布 + FOUND/LOST 日志（同步/异步后端共用） */
int vision_get_last(detect_out_t *out)
{
    if (!out)
        return 0;
    pthread_mutex_lock(&g_lock);
    *out = g_last;
    int v = g_last_valid;
    pthread_mutex_unlock(&g_lock);
    return v;
}

static void vision_note_result(const detect_out_t *d, uint32_t latency_us)
{
    pthread_mutex_lock(&g_lock);
    g_last = *d;
    g_last_valid = 1;
    g_stats.frames++;
    if (d->cx < 0)
        g_stats.misses++;
    if (d->t_us > g_period_detect_max)
        g_period_detect_max = d->t_us;
    g_period_detect_us += d->t_us;
    g_period_frames++;
    /* 端到端延迟：周均值/周峰值 + 全程最大（三者语义分开，别再混） */
    g_period_lat_sum += latency_us;
    if (latency_us > g_period_lat_max)
        g_period_lat_max = latency_us;
    if (latency_us > g_lat_max_all)
        g_lat_max_all = latency_us;
    uint32_t fps_now = g_stats.fps;
    pthread_mutex_unlock(&g_lock);

    result_publish(d->cx, d->cy, d->seq, fps_now);
    vision_log_result(d);

    uint64_t now = mono_us();
    if (now - g_period_start_us >= (uint64_t)g_cfg.status_period_s * 1000000ull) {
        pthread_mutex_lock(&g_lock);
        g_stats.fps = (uint32_t)((g_period_frames * 1000000ull) /
                                 (now - g_period_start_us ? (now - g_period_start_us) : 1));
        g_stats.t_detect_avg_us = g_period_frames
                                      ? (uint32_t)(g_period_detect_us / g_period_frames)
                                      : 0;
        g_stats.t_detect_max_us = g_period_detect_max;
        g_stats.t_lat_avg_us = g_period_frames
                                   ? (uint32_t)(g_period_lat_sum / g_period_frames)
                                   : 0;
        g_stats.t_lat_max_us = g_period_lat_max;
        g_stats.t_lat_max_all_us = g_lat_max_all;
        uint32_t fps_meas = g_stats.fps;
        pthread_mutex_unlock(&g_lock);
        /*
         * 把**实测处理帧率**告诉录像抽样器：本板请求 30fps、实测只有 28.5fps，
         * 抽样器若还按 30 算，25/30 只能录到 23.8fps。用实测值才能真的落到 25fps。
         */
        if (fps_meas)
            record_set_src_fps(fps_meas);
        g_period_start_us = now;
        g_period_frames = 0;
        g_period_detect_us = 0;
        g_period_detect_max = 0;
        g_period_lat_sum = 0;
        g_period_lat_max = 0;
    }
}

/* 把后端自己的耗时分解写进一行（kpu 后端才有） */
void vision_backend_stats(char *buf, size_t n)
{
    if (g_det && g_det->stats_line)
        g_det->stats_line(buf, n);
    else if (n)
        buf[0] = '\0';
}

static void vision_log_result(const detect_out_t *d)
{
    int found = (d->cx >= 0);
    if (found == g_found_prev)
        return;

    if (found) {
        app_log(APP_NAME_STR ": detect FOUND center=(%d,%d) px=%u blobs=%u scan=%s "
                "(lost %u frames before)\n",
                d->cx, d->cy, d->px, d->blobs, d->full_scan ? "full" : "roi",
                g_misses ? g_misses - 1 : 0);
        g_stats.found++;
        g_stats.found_total++;
    } else {
        app_log(APP_NAME_STR ": detect LOST (green_px=%u probe_lab=(%d,%d,%d) "
                "probe_rgb=(%d,%d,%d))\n",
                d->green_px, d->probe_L, d->probe_A, d->probe_B,
                d->probe_r, d->probe_g, d->probe_b);
        g_stats.lost++;
        g_stats.lost_total++;
    }
    g_found_prev = found;
}

/*
 * 处理线程。两种后端：
 *   同步（color）：一次 run() 跑完全部并直接拿到结果；
 *   异步（kpu）  ：submit() 只投递（帧所有权交给后端，由后端在 AI2D 读完硬件后
 *                 归还），结果用 collect() 取回。异步模式下本线程用很短的
 *                 轮询周期看自己的信箱与结果环——注意真正等 KPU 的地方不在
 *                 这里，而在后端自己的线程里由 KPU 完成中断唤醒（见
 *                 detect_kpu.cpp 的 interp.run()）。
 */
static void *vis_proc_thread(void *arg)
{
    (void)arg;
    const int is_async = (g_det && g_det->submit != NULL);

    while (g_run) {
        k_video_frame_info frame;
        uint64_t seq = 0, dumped_us = 0;
        int have = 0;

        pthread_mutex_lock(&g_lock);
        if (g_slot_have) {
            frame = g_slot;
            seq = g_slot_seq;
            dumped_us = g_slot_mono_us;
            g_slot_have = 0;
            have = 1;
        } else {
            /* 同步/异步后端都在这里等"下一帧到了"这个事件（异步后端的结果在下面
             * 每次提交后统一取回）—— 不 sleep、不轮询 */
            while (g_run && !g_slot_have)
                pthread_cond_wait(&g_cond, &g_lock);
            if (g_run && g_slot_have) {
                frame = g_slot;
                seq = g_slot_seq;
                dumped_us = g_slot_mono_us;
                g_slot_have = 0;
                have = 1;
            }
        }
        pthread_mutex_unlock(&g_lock);

        if (!g_run && !have)
            break;

        if (have) {
            if (!g_enabled) {
                vision_recycle_frame(&frame);   /* VICAP 只由采集线程 release */
            } else {
                uint32_t stride = frame.v_frame.stride[0] ? frame.v_frame.stride[0]
                                                          : frame.v_frame.width;
                uint32_t width = frame.v_frame.width;
                uint32_t height = frame.v_frame.height;
                int full_scan = (g_cfg.miss_full_scan > 0 &&
                                 g_misses >= (uint32_t)g_cfg.miss_full_scan);

                /* 一次性打印帧布局：板上核对「映射长度 / 平面是否连续」的第一手证据 */
                static int layout_logged;
                if (!layout_logged) {
                    layout_logged = 1;
                    app_log(APP_NAME_STR ": frame %ux%u stride=%u/%u/%u fmt=%d "
                            "phys1-phys0=%lld phys2-phys0=%lld stride*h=%u "
                            "(RGB_PLANAR 期望 phys 差 = %u / %u) 映射=cached+invalidate\n",
                            width, height, frame.v_frame.stride[0], frame.v_frame.stride[1],
                            frame.v_frame.stride[2], (int)frame.v_frame.pixel_format,
                            (long long)(frame.v_frame.phys_addr[1] - frame.v_frame.phys_addr[0]),
                            (long long)(frame.v_frame.phys_addr[2] - frame.v_frame.phys_addr[0]),
                            stride * height, stride * height, stride * height * 2);
                }

                mpp_stats_t ms0, ms1;
                mpp_get_stats(&ms0);

                plane_map_t pm;
                if (plane_map(&frame, stride, height, &pm) == 0) {
                    frame_view_t view;
                    memset(&view, 0, sizeof(view));
                    view.y = pm.y;
                    view.uv = pm.uv;
                    view.stride = stride;
                    view.width = width;
                    view.height = height;
                    view.seq = seq;
                    view.dump_us = dumped_us;

                    trace_log(seq, "proc mapped y=%p uv=%p begin", (const void *)pm.y,
                              (const void *)pm.uv);

                    if (is_async) {
                        /* 所有权转交后端：成功时本线程绝不能再 release 这帧 */
                        int rc = g_det->submit(&frame, &view, seq);
                        if (rc != 0) {
                            vision_recycle_frame(&frame);   /* VICAP 只由采集线程 release */
                            if (rc > 0)
                                g_stats.dropped++;
                            else
                                g_stats.dump_fail++;
                        }
                    } else {
                        detect_out_t d;
                        memset(&d, 0, sizeof(d));
                        g_det->run(&view, full_scan, &d);
                        d.seq = seq;

                        /*
                         * 交给录像：copy 模式会把这一帧拷进录像私有的 VB 块，拷完
                         * **立刻**回调 vision_rec_done 把帧推进归还环；borrow 模式则
                         * 在编码器收下之后回调。两种情况本线程都不再碰这帧。
                         * 录像不要（没开/抽样丢掉/没空闲块）就自己交回归还环。
                         */
                        rec_frame_t rf;
                        memset(&rf, 0, sizeof(rf));
                        rf.f = &frame;
                        rf.stride = stride;
                        rf.width = width;
                        rf.height = height;
                        if (frame.v_frame.pixel_format == PIXEL_FORMAT_RGB_888_PLANAR) {
                            rf.np = 3;
                            rf.p[0] = pm.y;
                            rf.p[1] = pm.y + (size_t)stride * height;
                            rf.p[2] = pm.y + (size_t)stride * height * 2;
                        } else {
                            rf.np = 2;
                            rf.p[0] = pm.y;
                            rf.p[1] = pm.uv;
                        }
                        if (!g_cfg.record_on ||
                            record_take_frame(&rf, seq, vision_rec_done, NULL) != 0)
                            vision_recycle_frame(&frame);
                        trace_log(seq, "proc detect-done cx=%d px=%u", d.cx, d.px);

                        if (d.cx >= 0)
                            g_misses = 0;
                        else
                            g_misses++;

                        uint32_t latency_us = (uint32_t)(mono_us() - dumped_us);
                        vision_note_result(&d, latency_us);
                    }
                } else {
                    app_log(APP_NAME_STR ": frame map failed seq=%llu\n",
                            (unsigned long long)seq);
                    vision_recycle_frame(&frame);   /* VICAP 只由采集线程 release */
                }

                mpp_get_stats(&ms1);
                pthread_mutex_lock(&g_lock);
                g_stats.map_calls += (uint32_t)(ms1.maps - ms0.maps);
                pthread_mutex_unlock(&g_lock);
            }
        }

        /*
         * 异步后端：把已完成的结果全部取回来。这里不 sleep 也不轮询 —— 本轮提交
         * 完成后立刻收一次，然后回到上面等下一帧的阻塞等待里（后端有 2 个乒乓槽，
         * 上上帧的结果最迟在下一帧到达时被收走）。
         */
        if (is_async && g_det->collect) {
            detect_out_t d;
            while (g_det->collect(&d)) {
                if (d.cx >= 0)
                    g_misses = 0;
                else
                    g_misses++;
                vision_note_result(&d, d.lat_us);
            }
        }
    }
    return NULL;
}

/* ============================ 对外接口 ============================ */

int vision_start(void)
{
    g_run = 1;
    g_enabled = g_cfg.vision_on ? 1 : 0;
    g_found_prev = 0;
    g_misses = 0;
    g_period_start_us = mono_us();

    {
        uint32_t afps = vicap_acq_fps();
        uint32_t frame_us = afps ? (1000000u / afps) : 33000u;
        app_log(APP_NAME_STR ": vision 取帧=阻塞等帧（事件驱动，不 sleep、不轮询；帧间隔 %u us）"
                "兜底超时 %dms；VICAP 只由采集线程调用\n", frame_us, g_cfg.dump_timeout_ms);
    }

    g_det = detector_get(g_cfg.detector);
    if (!g_det || g_det->init() != 0) {
        app_log(APP_NAME_STR ": detector init failed\n");
        g_run = 0;
        return -1;
    }

    if (pthread_create(&g_cap_tid, NULL, vis_cap_thread, NULL) != 0) {
        app_log(APP_NAME_STR ": vision capture thread create failed\n");
        g_run = 0;
        return -1;
    }
    g_cap_started = 1;

    if (pthread_create(&g_proc_tid, NULL, vis_proc_thread, NULL) != 0) {
        app_log(APP_NAME_STR ": vision process thread create failed\n");
        g_run = 0;
        pthread_cond_broadcast(&g_cond);
        pthread_join(g_cap_tid, NULL);
        g_cap_started = 0;
        return -1;
    }
    g_proc_started = 1;

    app_log(APP_NAME_STR ": vision thread started (chn=CHN%d %ux%u) enabled=%d\n",
            (int)VICAP_CHN_VISION, vicap_chn_width(VICAP_CHN_VISION),
            vicap_chn_height(VICAP_CHN_VISION), g_enabled);
    return 0;
}

void vision_stop(void)
{
    if (!g_cap_started && !g_proc_started)
        return;

    g_run = 0;
    g_enabled = 0;
    pthread_mutex_lock(&g_lock);
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);

    if (g_cap_started) {
        pthread_join(g_cap_tid, NULL);
        g_cap_started = 0;
    }
    if (g_proc_started) {
        pthread_join(g_proc_tid, NULL);
        g_proc_started = 0;
    }

    /* 收尾：把信箱槽和归还环里残留的帧还回去（join 之后本线程独用 VICAP） */
    k_video_frame_info left[RECYCLE_N + 1];
    int nleft = 0;
    pthread_mutex_lock(&g_lock);
    if (g_slot_have) {
        left[nleft++] = g_slot;
        g_slot_have = 0;
    }
    while (g_recyc_n > 0 && nleft < RECYCLE_N + 1)
        left[nleft++] = g_recyc[--g_recyc_n];
    pthread_mutex_unlock(&g_lock);
    for (int i = 0; i < nleft; ++i)
        vicap_release(VICAP_CHN_VISION, &left[i]);

    if (g_det && g_det->deinit)
        g_det->deinit();
    g_det = NULL;
}

int vision_is_started(void)
{
    return g_cap_started && g_proc_started;
}

void vision_set_enabled(int on)
{
    g_enabled = on ? 1 : 0;
    /* 两个方向都要唤醒：关掉时让处理线程收尾，打开时让采集线程从"等开关"里醒来 */
    pthread_mutex_lock(&g_lock);
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
    app_log(APP_NAME_STR ": vision %s\n", on ? "enabled" : "disabled");
}

int vision_enabled(void)
{
    return g_enabled ? 1 : 0;
}

void vision_get_stats(vision_stats_t *st)
{
    if (!st)
        return;
    pthread_mutex_lock(&g_lock);
    *st = g_stats;
    /* 这三个量在采集线程的静态变量里（不在 g_stats）：必须显式搬过来，
     * 否则状态行会一直显示 0（2025-09 板端就是这样骗过一轮的） */
    st->recycled     = g_recycled;
    st->recycle_leak = g_recycle_leak;
    st->dump_call_us = g_dump_call_us;
    pthread_mutex_unlock(&g_lock);
}
