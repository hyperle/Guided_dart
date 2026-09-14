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
/*
 * 采集节流（自适应）：
 * 板端实测：dump 的**超时路径**会让 VICAP 通道逐渐劣化（超时攒到约千次后彻底
 * 不出帧，表现为 dumped 每 2 秒增量 58→37→29→6→0 的衰减）。旧工程用
 * 「连续 dump + 200ms 超时」几乎从不超时，单通道稳定跑了 40079 帧。
 * 这里改成：节流起点取 105% 帧间隔（比一帧稍长，醒来时帧一定已就绪），
 * 万一还是超时就**自动加大节流**，把超时次数压到 0 附近。
 */
static uint32_t g_dump_interval_us;   /* 当前节流间隔（动态调整） */
static uint32_t g_dump_interval_base; /* 起始间隔 = 105% 帧间隔 */
static uint32_t g_dump_timeouts;      /* 本进程内 dump 超时累计（要盯住 = 0） */
static int      g_stall_reported;

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

static void *vis_cap_thread(void *arg)
{
    (void)arg;
    int empty_runs = 0;

    while (g_run) {
        if (!g_enabled) {
            usleep(20000);
            continue;
        }

        /*
         * 按采集帧间隔节流：传感器 30fps 时每 33ms 才有一帧，若在两次成功之间
         * 用 5ms 超时反复重试（每秒上百次超时），一是白占 MPP 锁，二是板端实测
         * 长时间大量超时后通道会彻底不出帧。这里先睡到「下一帧该到了」再取，
         * 取的时候帧基本已经就绪，超时次数从每帧 2~3 次降到接近 0。
         */

        k_video_frame_info f;
        memset(&f, 0, sizeof(f));
        int raw = 0;
        uint64_t t_dump0 = mono_us();
        /* 短超时（5ms）快速失败：不让「等新帧」长时间占着 MPP 锁 */
        int rc = vicap_dump_soft(VICAP_CHN_VISION, &f, g_cfg.dump_timeout_ms, &raw);
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
            /* 锁外退避；连续空转时退得更久一点，减少对录像侧的锁竞争 */
            /*
             * 连续取帧（不节流）：dump 的语义就是"阻塞等下一帧"，正常情况几乎不会
             * 走到这里。真走到这里说明帧率低于预期或流水异常，给个短退避避免空转。
             */
            g_dump_timeouts++;
            g_stats.dump_timeouts = g_dump_timeouts;
            if (g_dump_timeouts % 100u == 1u)
                app_log(APP_NAME_STR ": vision dump no-frame (total %u)\n", g_dump_timeouts);
            (void)empty_runs;
            usleep(1000);
            continue;
        }
        empty_runs = 0;
        g_last_dump_us = mono_us();
        g_stall_reported = 0;
        g_stats.dumped++;
        g_diag_phys[g_diag_phys_n % DIAG_PHYS_N] = f.v_frame.phys_addr[0];
        g_diag_phys_n++;

        uint64_t seq = ++g_frame_seq;

        pthread_mutex_lock(&g_lock);
        if (!g_run || !g_enabled) {
            pthread_mutex_unlock(&g_lock);
            vicap_release(VICAP_CHN_VISION, &f);   /* 关停瞬间：归还本帧 */
            continue;
        }
        if (g_slot_have) {
            /* 处理器还没取走上一帧：作废旧帧（新帧优先），绝不阻塞采集 */
            k_video_frame_info old = g_slot;
            g_slot_have = 0;
            g_stats.dropped++;
            pthread_mutex_unlock(&g_lock);
            vicap_release(VICAP_CHN_VISION, &old);
            pthread_mutex_lock(&g_lock);
        }
        g_slot = f;
        g_slot_seq = seq;
        g_slot_mono_us = mono_us();
        g_slot_have = 1;
        g_produced++;
        pthread_cond_signal(&g_cond);
        pthread_mutex_unlock(&g_lock);

        trace_log(seq, "cap dump ok w=%u h=%u stride=%u phys0=0x%llx",
                  f.v_frame.width, f.v_frame.height, f.v_frame.stride[0],
                  (unsigned long long)f.v_frame.phys_addr[0]);
    }
    return NULL;
}

/* ============================ 停流诊断 ============================ */

/*
 * 视觉通道连续拿不到帧时调用：把判定所需的信息一次性打进日志，
 * 并在开启 --stall-fallback（默认开）时**自动解绑录像**验证假设 ——
 * 视觉是主功能，录像可降级；同时这次结果直接告诉我们问题出在哪：
 *   解绑后立刻恢复  -> CHN1/绑定 与 CHN0 的相互影响（两条通道不能这么配）
 *   解绑后仍不恢复  -> CHN0 自身的问题（映射/释放/配置）
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
            "notready=%llu rel_fail=%llu dump_timeout=%u pace=%uus | "
            "失败 dump 平均耗时 %llu us（≈超时上限说明是在等帧，≈0 说明通道立刻报错）| "
            "CHN0 attr: %s | CHN0 缓冲引用:%s | rec bound=%d streams=%llu\n",
            (unsigned)raw, (double)stall_us / 1e6,
            (unsigned long long)g_stats.dumped, (unsigned long long)g_stats.noframe,
            (unsigned long long)g_stats.notready,
            (unsigned long long)vicap_release_fail_count(),
            g_dump_timeouts, g_dump_interval_us,
            (unsigned long long)(g_fail_dump_cnt ? g_fail_dump_us_sum / g_fail_dump_cnt : 0),
            got_attr ? "ok" : "get_chn_attr 失败",
            blk[0] ? blk : "(还没有成功 dump 过)",
            rs.bound, (unsigned long long)rs.streams);

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
        pthread_mutex_unlock(&g_lock);
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
        } else if (!is_async) {
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
                vicap_release(VICAP_CHN_VISION, &frame);
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
                            vicap_release(VICAP_CHN_VISION, &frame);
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
                         * 交棒给录像（shared 模式）：收下则不归还 —— 由录像取流线程
                         * 在取到对应码流后归还（唯一归还者）。拒收/录像关闭则自己归还。
                         */
                        if (!g_cfg.record_on || record_offer_frame(&frame, seq) != 0)
                            vicap_release(VICAP_CHN_VISION, &frame);
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
                    vicap_release(VICAP_CHN_VISION, &frame);
                }

                mpp_get_stats(&ms1);
                pthread_mutex_lock(&g_lock);
                g_stats.map_calls += (uint32_t)(ms1.maps - ms0.maps);
                pthread_mutex_unlock(&g_lock);
            }
        }

        /* 异步后端：把已完成的结果全部取回来 */
        if (is_async && g_det->collect) {
            detect_out_t d;
            while (g_det->collect(&d)) {
                if (d.cx >= 0)
                    g_misses = 0;
                else
                    g_misses++;
                vision_note_result(&d, d.lat_us);
            }
            usleep(500);
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
        g_dump_interval_base = frame_us;
        g_dump_interval_us = frame_us;
        app_log(APP_NAME_STR ": vision dump 策略=continuous timeout=%dms（帧间隔 %u us）\n",
                g_cfg.dump_timeout_ms, frame_us);
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

    /* 收尾：把信箱里可能残留的帧还回去（必须在 join 之后） */
    pthread_mutex_lock(&g_lock);
    if (g_slot_have) {
        k_video_frame_info f = g_slot;
        g_slot_have = 0;
        pthread_mutex_unlock(&g_lock);
        vicap_release(VICAP_CHN_VISION, &f);
    } else {
        pthread_mutex_unlock(&g_lock);
    }

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
    if (!on) {
        /* 关掉时唤醒处理线程，让它把在途帧归还掉 */
        pthread_mutex_lock(&g_lock);
        pthread_cond_broadcast(&g_cond);
        pthread_mutex_unlock(&g_lock);
    }
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
    pthread_mutex_unlock(&g_lock);
}
