/*
 * green_led_rtos 录像模块：H.264 硬件编码 + 落盘 + recording/ 容量管理。
 *
 * 线程结构（全部在 rec_init 创建）：
 *   - feed 线程：  把采集线程拷入环形缓冲的 NV12 帧送入 VENC(H.264)，同时写 .csv
 *   - drain 线程： 从 VENC 取编码流写入 .h264，并把对应的输入缓冲归还环形缓冲
 *   - control 线程：轮询信号标志 / 控制 socket，执行 start/stop/status 等命令
 *
 * 环形缓冲是"录像自有的 VB 内存"：采集线程只做 memcpy 入队，满则丢帧，
 * 从不等待编码器/SD —— 因此录像不影响识别帧率（rec_feed_frame 的时间有界）。
 *
 * 编码输入缓冲生命周期：FREE -> QUEUED(采集已拷入) -> SENT(已送编码器，等待
 * 输出) -> FREE(输出帧被 drain 取走)。SENT 缓冲必须等编码器输出完对应帧才能
 * 复用，因此用 in-flight FIFO 按输出顺序归还，避免硬件还在读就被覆盖。
 */
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_venc_api.h"

#include "recorder.h"

#define VENC_CHN       0
#define IN_RING_N      6    /* NV12 输入环形缓冲数（约 2.8MB VB） */
#define OUT_BUF_N      8    /* VENC 输出流缓冲数（约 2.5MB VB） */
#define INFLIGHT_MAX   128
#define MAX_DIR_FILES  512
#define REC_ROLL_BYTES (256ull << 20) /* 常录模式下每段会话的字节上限：到点自动滚到下一段 */

enum { SLOT_FREE = 0, SLOT_RESERVED, SLOT_QUEUED, SLOT_SENT };

typedef struct {
    uint8_t     state;
    k_vb_blk_handle handle;
    uint64_t    phys;
    uint8_t    *va;
    uint32_t    stride;   /* 输入帧行距（拷贝与 VENC stride 一致） */
    /* 元数据（入队时快照） */
    uint64_t    frame_seq;
    uint64_t    mono_us;
    uint64_t    result_seq;
    int32_t     center_x;
    int32_t     center_y;
    uint32_t    detect_fps;
} rec_slot_t;

typedef struct {
    uint64_t seq;
    int32_t  center_x;
    int32_t  center_y;
    uint32_t fps;
} rec_result_t;

/* ---------------- 全局状态 ---------------- */
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t g_op   = PTHREAD_MUTEX_INITIALIZER; /* start/stop 串行化 */

static volatile int g_init     = 0;
static volatile int g_exit     = 0;
static volatile int g_active   = 0;   /* 会话进行中（无锁读，写有锁） */
static volatile int g_feed_idle = 1;  /* feed 已无输入可送 */
static volatile int g_drain_idle = 1; /* drain 已取完输出 */
static volatile int g_auto_stop = 0;  /* 会话超总容量上限 */
static volatile int g_auto_roll = 0;  /* 会话到滚动字节上限 */
static volatile int g_auto_mode = 1;  /* 常录模式：开机即录 + 自动滚动续录 */
static volatile int g_start_failed_logged = 0;

static recorder_config_t g_cfg;
static char g_dir[256];
static uint64_t g_cap_bytes = REC_DEFAULT_CAP_BYTES;

/* VB 与编码器 */
static int g_pools_ok = 0;
static k_u32 g_in_pool = VB_INVALID_POOLID;
static k_u32 g_out_pool = VB_INVALID_POOLID;
static uint32_t g_in_blk = 0;
static int g_chn_ready = 0;

static rec_slot_t g_slots[IN_RING_N];
static uint32_t g_inflight_idx[INFLIGHT_MAX];
static uint32_t g_inflight_n = 0;

/* 会话文件 */
static FILE *g_fh264 = NULL;
static FILE *g_fcsv = NULL;
static char g_h264_path[320];
static char g_csv_path[320];
static uint32_t g_sess_idx = 0;
static uint64_t g_sess_start_us = 0;

/* 统计 */
static recorder_stats_t g_stats;

/* 识别结果（detect 线程写，采集入队时读，均在 g_lock 下） */
static rec_result_t g_last_result;

/* 控制通道 */
static int g_listen_fd = -1;
static volatile int g_sig_start = 0;
static volatile int g_sig_stop = 0;

/* 前向声明（会话函数定义在 control/socket 段之后） */
static int  rec_session_start(void);
static void rec_session_stop(void);

static pthread_t g_feed_tid, g_drain_tid, g_ctrl_tid;

/* ---------------- 小工具 ---------------- */

static uint64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

static void rec_log(const char *fmt, ...)
{
    static int dir_ok;
    if (!dir_ok) {
        mkdir("/sdcard/app/logs", 0777);
        dir_ok = 1;
    }
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    int fd = open("/sdcard/app/logs/green_led_recorder.log",
                  O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0) {
        fprintf(stderr, "recorder: %s", buf);
        return;
    }
    ssize_t w = write(fd, buf, (size_t)n);
    fsync(fd);
    close(fd);
    (void)w;
}

static int rec_dir_mkdir(void)
{
    /* 只需创建最后一层；父目录 /sdcard/app 由部署保证存在 */
    return mkdir(g_dir, 0777) == 0 || errno == EEXIST ? 0 : -1;
}

/* 扫描 dir，返回下一个会话序号（rec_<idx>.xxx 的最大 idx + 1） */
static uint32_t rec_dir_next_index(void)
{
    uint32_t max_idx = 0;
    DIR *d = opendir(g_dir);
    if (!d)
        return 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, "rec_", 4) != 0)
            continue;
        unsigned long idx = strtoul(e->d_name + 4, NULL, 10);
        if (idx > max_idx)
            max_idx = (uint32_t)idx;
    }
    closedir(d);
    return max_idx + 1;
}

/* 删除最旧会话，使目录总大小 <= cap（按 rec_<idx> 序号从旧到新删） */
static void rec_evict_old(void)
{
    typedef struct { uint32_t idx; uint64_t size; } fent_t;
    static fent_t ents[MAX_DIR_FILES];
    int n = 0;
    uint64_t total = 0;

    DIR *d = opendir(g_dir);
    if (!d)
        return;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        unsigned long idx;
        if (strncmp(e->d_name, "rec_", 4) != 0)
            continue;
        if (sscanf(e->d_name + 4, "%lu", &idx) != 1)
            continue;
        if (strstr(e->d_name, ".h264") == NULL && strstr(e->d_name, ".csv") == NULL)
            continue;
        char path[560];
        snprintf(path, sizeof(path), "%s/%s", g_dir, e->d_name);
        struct stat st;
        if (stat(path, &st) != 0)
            continue;
        if (n < MAX_DIR_FILES) {
            ents[n].idx = (uint32_t)idx;
            ents[n].size = (uint64_t)st.st_size;
            n++;
        }
        total += (uint64_t)st.st_size;
    }
    closedir(d);
    if (n == 0 || total <= g_cap_bytes)
        return;

    /* 按 idx 升序插入排序 */
    for (int i = 1; i < n; i++) {
        fent_t t = ents[i];
        int j = i - 1;
        while (j >= 0 && ents[j].idx > t.idx) {
            ents[j + 1] = ents[j];
            j--;
        }
        ents[j + 1] = t;
    }
    /* 归并同名会话的两个文件为一个条目，便于按会话删 */
    int m = 0;
    for (int i = 0; i < n; i++) {
        if (m > 0 && ents[m - 1].idx == ents[i].idx) {
            ents[m - 1].size += ents[i].size;
        } else {
            ents[m++] = ents[i];
        }
    }
    for (int i = 0; i < m && total > g_cap_bytes; i++) {
        char h[560], c[560];
        snprintf(h, sizeof(h), "%s/rec_%04u.h264", g_dir, ents[i].idx);
        snprintf(c, sizeof(c), "%s/rec_%04u.csv", g_dir, ents[i].idx);
        unlink(h);
        unlink(c);
        total -= ents[i].size;
        rec_log("recorder: evict session rec_%04u (%" PRIu64 " bytes)\n",
                ents[i].idx, ents[i].size);
    }
}

/* 轮询等待 idle 标志（避免 cond timedwait 依赖实时时钟） */
static int rec_wait_idle(volatile int *flag, int timeout_ms)
{
    for (int waited = 0; waited < timeout_ms; waited += 10) {
        pthread_mutex_lock(&g_lock);
        int v = *flag;
        pthread_mutex_unlock(&g_lock);
        if (v)
            return 1;
        usleep(10000);
    }
    return 0;
}

/* ---------------- VB / 编码器 ---------------- */

static int rec_ensure_pools(void)
{
    if (g_pools_ok)
        return 0;
    uint32_t ysize = g_cfg.width * g_cfg.height;
    g_in_blk = VB_ALIGN_UP((uint64_t)ysize * 3u / 2u, 4096);
    g_in_pool = kd_mpi_vb_create_pool_ex(g_in_blk, IN_RING_N,
                                         VB_REMAP_MODE_NOCACHE);
    if (g_in_pool == VB_INVALID_POOLID) {
        rec_log("recorder: input vb pool create failed\n");
        return -1;
    }
    k_u32 oblk = VB_ALIGN_UP((uint64_t)ysize, 4096);
    g_out_pool = kd_mpi_vb_create_pool_ex(oblk, OUT_BUF_N,
                                          VB_REMAP_MODE_NOCACHE);
    if (g_out_pool == VB_INVALID_POOLID) {
        kd_mpi_vb_destory_pool(g_in_pool);
        g_in_pool = VB_INVALID_POOLID;
        rec_log("recorder: output vb pool create failed\n");
        return -1;
    }
    for (int i = 0; i < IN_RING_N; i++) {
        rec_slot_t *s = &g_slots[i];
        s->handle = kd_mpi_vb_get_block(g_in_pool, g_in_blk, NULL);
        if (s->handle == 0) {
            rec_log("recorder: get input block %d failed\n", i);
            s->handle = 0;
            s->state = SLOT_FREE;
            continue;
        }
        s->phys = kd_mpi_vb_handle_to_phyaddr(s->handle);
        s->va = (uint8_t *)kd_mpi_sys_mmap(s->phys, g_in_blk);
        if (!s->va || s->phys == 0) {
            rec_log("recorder: map input block %d failed\n", i);
            s->va = NULL;
            s->handle = 0;
            continue;
        }
        s->state = SLOT_FREE;
    }
    g_pools_ok = 1;
    rec_log("recorder: vb pools ok (in=%u blk=%u, out=%u)\n",
            g_in_pool, g_in_blk, g_out_pool);
    return 0;
}

static int rec_venc_start(void)
{
    if (g_chn_ready)
        return 0;
    if (kd_mpi_venc_attach_vb_pool(VENC_CHN, g_out_pool) != K_SUCCESS) {
        rec_log("recorder: venc attach vb pool failed\n");
        return -1;
    }
    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.type = K_PT_H264;
    attr.venc_attr.pic_width = g_cfg.width;
    attr.venc_attr.pic_height = g_cfg.height;
    attr.venc_attr.profile = VENC_PROFILE_H264_HIGH;
    attr.rc_attr.rc_mode = K_VENC_RC_MODE_CBR;
    attr.rc_attr.cbr.gop = g_cfg.nominal_fps;      /* 1s 一个 I 帧 */
    attr.rc_attr.cbr.src_frame_rate = g_cfg.nominal_fps;
    attr.rc_attr.cbr.dst_frame_rate = g_cfg.nominal_fps;
    attr.rc_attr.cbr.bit_rate = g_cfg.bitrate_kbps;
    if (kd_mpi_venc_create_chn(VENC_CHN, &attr) != K_SUCCESS) {
        rec_log("recorder: venc create chn failed\n");
        kd_mpi_venc_detach_vb_pool(VENC_CHN);
        return -1;
    }
    if (kd_mpi_venc_start_chn(VENC_CHN) != K_SUCCESS) {
        rec_log("recorder: venc start chn failed\n");
        kd_mpi_venc_destroy_chn(VENC_CHN);
        kd_mpi_venc_detach_vb_pool(VENC_CHN);
        return -1;
    }
    g_chn_ready = 1;
    return 0;
}

static void rec_venc_stop(void)
{
    if (!g_chn_ready)
        return;
    /* stop_chn 由会话停止流程在 drain 收尾前显式调用，这里只销毁通道 */
    kd_mpi_venc_destroy_chn(VENC_CHN);
    kd_mpi_venc_detach_vb_pool(VENC_CHN);
    g_chn_ready = 0;
}

/* ---------------- feed 线程 ---------------- */

static void *rec_feed_thread(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_lock);
    for (;;) {
        /* 无排队帧时休息；会话结束(active=0)后若队列也空则置 feed_idle */
        while (!g_exit) {
            int any = 0;
            for (int i = 0; i < IN_RING_N; i++)
                if (g_slots[i].state == SLOT_QUEUED) { any = 1; break; }
            if (any)
                break;
            g_feed_idle = g_active ? 0 : 1;
            pthread_cond_broadcast(&g_cond);
            pthread_cond_wait(&g_cond, &g_lock);
        }
        if (g_exit) {
            int any = 0;
            for (int i = 0; i < IN_RING_N; i++)
                if (g_slots[i].state == SLOT_QUEUED) { any = 1; break; }
            if (!any) {
                g_feed_idle = 1;
                pthread_cond_broadcast(&g_cond);
                break; /* deinit：队列已空，退出 */
            }
        }
        /* 取一个 QUEUED 槽送出 */
        rec_slot_t *s = NULL;
        for (int i = 0; i < IN_RING_N; i++)
            if (g_slots[i].state == SLOT_QUEUED) { s = &g_slots[i]; break; }
        if (!s)
            continue;
        g_feed_idle = 0;

        k_video_frame_info fi;
        memset(&fi, 0, sizeof(fi));
        fi.v_frame.width = g_cfg.width;
        fi.v_frame.height = g_cfg.height;
        fi.v_frame.pixel_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        fi.v_frame.stride[0] = s->stride;
        fi.v_frame.stride[1] = s->stride;
        fi.v_frame.phys_addr[0] = s->phys;
        fi.v_frame.phys_addr[1] = s->phys + (uint64_t)s->stride * g_cfg.height;
        pthread_mutex_unlock(&g_lock);

        k_s32 ret = kd_mpi_venc_send_frame(VENC_CHN, &fi, 80);

        pthread_mutex_lock(&g_lock);
        if (ret == K_SUCCESS) {
            s->state = SLOT_SENT;
            if (g_inflight_n < INFLIGHT_MAX) {
                g_inflight_idx[g_inflight_n++] = (uint32_t)(s - g_slots);
            } else {
                /* in-flight 队列异常满：释放缓冲并计数 */
                s->state = SLOT_FREE;
                g_stats.frames_dropped++;
                rec_log("recorder: in-flight overflow\n");
            }
            g_stats.frames_sent++;
            if (g_fcsv)
                fprintf(g_fcsv, "%" PRIu64 ",%" PRIu64 ",%" PRIu64
                                ",%d,%d,%u\n",
                        s->frame_seq, s->result_seq, s->mono_us,
                        s->center_x, s->center_y, s->detect_fps);
        } else {
            s->state = SLOT_FREE;
            g_stats.frames_dropped++;
        }
        pthread_cond_broadcast(&g_cond);
    }
    pthread_mutex_unlock(&g_lock);
    return NULL;
}

/* ---------------- drain 线程 ---------------- */

static void *rec_drain_thread(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&g_lock);
    for (;;) {
        while (!g_exit && g_inflight_n == 0) {
            g_drain_idle = g_active ? 0 : 1;
            pthread_cond_broadcast(&g_cond);
            pthread_cond_wait(&g_cond, &g_lock);
        }
        if (g_exit && g_inflight_n == 0) {
            g_drain_idle = 1;
            pthread_cond_broadcast(&g_cond);
            break;
        }
        pthread_mutex_unlock(&g_lock);

        /* 取编码流（在锁外做，get_stream 会等待） */
        k_venc_chn_status status;
        memset(&status, 0, sizeof(status));
        if (kd_mpi_venc_query_status(VENC_CHN, &status) != K_SUCCESS)
            status.cur_packs = 0;
        k_venc_stream st;
        memset(&st, 0, sizeof(st));
        st.pack_cnt = status.cur_packs ? status.cur_packs : 1;
        st.pack = (k_venc_pack *)malloc(sizeof(k_venc_pack) * st.pack_cnt);
        if (!st.pack) {
            usleep(10000);
            pthread_mutex_lock(&g_lock);
            continue;
        }
        k_s32 ret = kd_mpi_venc_get_stream(VENC_CHN, &st, 200);
        if (ret != K_SUCCESS) {
            free(st.pack);
            pthread_mutex_lock(&g_lock);
            continue;
        }

        uint64_t added = 0;
        uint32_t video = 0;
        for (k_u32 i = 0; i < st.pack_cnt; i++) {
            if (st.pack[i].len == 0)
                continue;
            k_u8 *p = (k_u8 *)kd_mpi_sys_mmap(st.pack[i].phys_addr,
                                              st.pack[i].len);
            if (p) {
                if (g_fh264 && fwrite(p, 1, st.pack[i].len, g_fh264) == st.pack[i].len)
                    added += st.pack[i].len;
                kd_mpi_sys_munmap(p, st.pack[i].len);
            }
            if (st.pack[i].type != K_VENC_HEADER)
                video++;
        }
        kd_mpi_venc_release_stream(VENC_CHN, &st);
        free(st.pack);

        pthread_mutex_lock(&g_lock);
        uint32_t pops = video < g_inflight_n ? video : g_inflight_n;
        for (uint32_t k = 0; k < pops; k++) {
            rec_slot_t *s = &g_slots[g_inflight_idx[0]];
            /* 队列前移 */
            for (uint32_t j = 0; j + 1 < g_inflight_n; j++)
                g_inflight_idx[j] = g_inflight_idx[j + 1];
            g_inflight_n--;
            s->state = SLOT_FREE;
        }
        g_stats.frames_written += pops;
        g_stats.bytes += added;
        if (video > pops)
            rec_log("recorder: pack/frame mismatch video=%u pops=%u\n", video, pops);
        if (REC_ROLL_BYTES && g_stats.bytes >= REC_ROLL_BYTES && !g_auto_roll) {
            g_auto_roll = 1;   /* 常录滚动点：交给控制线程滚动到下一段 */
            rec_log("recorder: session roll point reached\n");
        }
        if (g_cap_bytes && g_stats.bytes > g_cap_bytes && !g_auto_stop) {
            g_auto_stop = 1;   /* 单会话超总容量上限：交给控制线程停止 */
            rec_log("recorder: session exceeds cap, auto stop\n");
        }
        pthread_cond_broadcast(&g_cond);
    }
    pthread_mutex_unlock(&g_lock);
    return NULL;
}

/* ---------------- 控制 socket ---------------- */

static void rec_socket_reply(int fd, const char *msg)
{
    if (fd >= 0) {
        (void)send(fd, msg, strlen(msg), MSG_NOSIGNAL);
    }
}

/* 读取一行命令并执行（在 control 线程内） */
static void rec_socket_handle_client(int fd)
{
    struct timeval tv = {0, 300000}; /* 300ms 读超时，防恶意/异常客户端卡死控制线程 */
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    char buf[512];
    ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0)
        return;
    buf[n] = '\0';
    /* 去掉首尾空白 */
    char *s = buf;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')
        s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n'))
        *--e = '\0';

    if (strcmp(s, "start") == 0) {
        /* 进入常录模式并立即开录（若已在录则为 no-op） */
        if (rec_start() == 0)
            rec_socket_reply(fd, "start ok (auto=1)\n");
        else
            rec_socket_reply(fd, "start failed\n");
    } else if (strcmp(s, "stop") == 0) {
        rec_stop(); /* 停止并退出常录模式（暂停，需 start/USR1 恢复） */
        rec_socket_reply(fd, "stop ok (auto=0)\n");
    } else if (strcmp(s, "status") == 0) {
        recorder_stats_t st;
        pthread_mutex_lock(&g_lock);
        st = g_stats;
        pthread_mutex_unlock(&g_lock);
        char out[512];
        snprintf(out, sizeof(out),
                 "active=%d auto=%d roll_bytes=%" PRIu64
                 " dir=%.140s cap=%" PRIu64
                 " frames_in=%u sent=%u dropped=%u written=%u bytes=%" PRIu64
                 " sessions=%u\n",
                 g_active ? 1 : 0, g_auto_mode ? 1 : 0,
                 (uint64_t)REC_ROLL_BYTES, g_dir, g_cap_bytes,
                 st.frames_in, st.frames_sent, st.frames_dropped,
                 st.frames_written, st.bytes, st.sessions);
        rec_socket_reply(fd, out);
    } else if (strncmp(s, "cap ", 4) == 0) {
        rec_set_cap_bytes(strtoull(s + 4, NULL, 10));
        rec_socket_reply(fd, "cap ok\n");
    } else if (strncmp(s, "dir ", 4) == 0) {
        pthread_mutex_lock(&g_op);
        snprintf(g_dir, sizeof(g_dir), "%s", s + 4);
        pthread_mutex_unlock(&g_op);
        rec_socket_reply(fd, "dir ok\n");
    } else {
        rec_socket_reply(fd, "usage: start|stop|status|cap <bytes>|dir <path>\n");
    }
}

/* ---------------- control 线程 ---------------- */

/* 停止当前会话但保留常录模式（滚动/超容量用，之后自动续录） */
static void rec_stop_keep_auto(void)
{
    pthread_mutex_lock(&g_op);
    rec_session_stop();
    pthread_mutex_unlock(&g_op);
}

static void *rec_ctrl_thread(void *arg)
{
    (void)arg;
    while (!g_exit) {
        usleep(50000);

        /* 信号请求 */
        int want_start = 0, want_stop = 0, want_rollstop = 0, want_capstop = 0;
        pthread_mutex_lock(&g_lock);
        if (g_sig_start) { g_sig_start = 0; want_start = 1; }
        if (g_sig_stop)  { g_sig_stop = 0;  want_stop = 1; }
        if (g_auto_roll) { g_auto_roll = 0; want_rollstop = 1; }
        if (g_auto_stop) { g_auto_stop = 0; want_capstop = 1; }
        pthread_mutex_unlock(&g_lock);
        if (want_start) {
            rec_log("recorder: manual start (auto mode on)\n");
            (void)rec_start(); /* 内部会置 auto_mode=1 */
        }
        if (want_stop) {
            rec_log("recorder: manual stop (auto mode off)\n");
            rec_stop(); /* 内部会置 auto_mode=0 */
        }
        if (want_rollstop) {
            rec_log("recorder: session rolled\n");
            rec_stop_keep_auto(); /* 保持常录，下面自动开下一段 */
        }
        if (want_capstop) {
            rec_log("recorder: cap reached, stop session\n");
            rec_stop_keep_auto();
        }

        /* 常录模式：没有会话在跑就立刻开一段（开机自动开始 + 滚动/超容量后续录） */
        if (!g_exit && g_auto_mode && !rec_is_active()) {
            if (rec_start() != 0) {
                if (!g_start_failed_logged) {
                    g_start_failed_logged = 1;
                    rec_log("recorder: auto start failed (will retry)\n");
                }
                usleep(1000000); /* 失败退避 1s，避免空转 */
            } else {
                g_start_failed_logged = 0;
            }
        }

        /* 控制 socket 连接 */
        if (g_listen_fd >= 0) {
            int cfd = accept(g_listen_fd, NULL, NULL);
            if (cfd >= 0) {
                rec_socket_handle_client(cfd);
                close(cfd);
            }
        }
    }
    if (g_listen_fd >= 0) {
        close(g_listen_fd);
        g_listen_fd = -1;
        unlink(REC_CTRL_SOCKET);
    }
    return NULL;
}

/* ---------------- 会话开始 / 结束 ---------------- */

static int rec_session_start(void)
{
    if (g_exit)
        return -1;
    if (g_active)
        return 0;
    if (rec_ensure_pools() != 0)
        return -1;
    if (rec_dir_mkdir() != 0) {
        rec_log("recorder: mkdir %s failed\n", g_dir);
        return -1;
    }
    uint32_t idx = rec_dir_next_index();
    snprintf(g_h264_path, sizeof(g_h264_path), "%s/rec_%04u.h264", g_dir, idx);
    snprintf(g_csv_path, sizeof(g_csv_path), "%s/rec_%04u.csv", g_dir, idx);
    g_fh264 = fopen(g_h264_path, "wb");
    if (!g_fh264) {
        rec_log("recorder: open %s failed\n", g_h264_path);
        return -1;
    }
    g_fcsv = fopen(g_csv_path, "w");
    if (!g_fcsv) {
        fclose(g_fh264);
        g_fh264 = NULL;
        rec_log("recorder: open %s failed\n", g_csv_path);
        return -1;
    }
    if (rec_venc_start() != 0) {
        fclose(g_fh264); g_fh264 = NULL;
        fclose(g_fcsv);  g_fcsv = NULL;
        unlink(g_h264_path);
        unlink(g_csv_path);
        return -1;
    }

    pthread_mutex_lock(&g_lock);
    g_sess_idx = idx;
    g_sess_start_us = mono_us();
    memset(&g_stats, 0, sizeof(g_stats));
    /* 兜底：上一会话异常残留的槽位/在途帧一律作废（本会话编码器是新建的） */
    for (int i = 0; i < IN_RING_N; i++)
        g_slots[i].state = SLOT_FREE;
    g_inflight_n = 0;
    g_auto_roll = 0;
    g_auto_stop = 0;
    g_feed_idle = 0;
    g_drain_idle = 0;
    g_active = 1;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);

    fprintf(g_fcsv, "# session rec_%04u start_mono_us=%" PRIu64 "\n",
            idx, g_sess_start_us);
    fprintf(g_fcsv, "frame_seq,result_seq,mono_us,center_x,center_y,detect_fps\n");
    rec_log("recorder: session rec_%04u start (dir=%s cap=%" PRIu64 ")\n",
            idx, g_dir, g_cap_bytes);
    return 0;
}

static void rec_session_stop(void)
{
    pthread_mutex_lock(&g_lock);
    if (!g_active) {
        pthread_mutex_unlock(&g_lock);
        return;
    }
    g_active = 0;
    g_feed_idle = 0;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);

    rec_log("recorder: session rec_%04u stopping\n", g_sess_idx);
    if (!rec_wait_idle(&g_feed_idle, 2000))
        rec_log("recorder: warning feed not idle\n");
    if (g_fcsv) {
        fflush(g_fcsv);
        fclose(g_fcsv);
        g_fcsv = NULL;
    }
    /* 停编码器让尾部帧输出，drain 取完 in-flight 后自然空闲 */
    kd_mpi_venc_stop_chn(VENC_CHN);
    if (!rec_wait_idle(&g_drain_idle, 3000)) {
        rec_log("recorder: warning drain not idle\n");
        pthread_mutex_lock(&g_lock);
        for (int i = 0; i < IN_RING_N; i++)
            if (g_slots[i].state == SLOT_SENT) {
                g_slots[i].state = SLOT_FREE;
                g_stats.frames_dropped++;
            }
        g_inflight_n = 0;
        pthread_cond_broadcast(&g_cond);
        pthread_mutex_unlock(&g_lock);
        /* drain 看到 in-flight 清空后应立即空闲；等它停下再关文件 */
        rec_wait_idle(&g_drain_idle, 1000);
    }
    if (g_fh264) {
        fflush(g_fh264);
        int fd = fileno(g_fh264);
        if (fd >= 0)
            fsync(fd);
        fclose(g_fh264);
        g_fh264 = NULL;
    }
    rec_venc_stop();
    rec_evict_old();

    pthread_mutex_lock(&g_lock);
    g_stats.sessions++;
    uint32_t written = g_stats.frames_written;
    pthread_mutex_unlock(&g_lock);
    rec_log("recorder: session rec_%04u done in=%u sent=%u dropped=%u "
            "written=%u bytes=%" PRIu64 "\n",
            g_sess_idx, g_stats.frames_in, g_stats.frames_sent,
            g_stats.frames_dropped, g_stats.frames_written, g_stats.bytes);
    if (written == 0) {
        /* 空会话（如误开即停）：不留下空文件 */
        unlink(g_h264_path);
        unlink(g_csv_path);
        rec_log("recorder: empty session removed\n");
    }
}

/* ---------------- 公共 API ---------------- */

int rec_init(const recorder_config_t *cfg)
{
    if (g_init)
        return 0;
    memset(&g_cfg, 0, sizeof(g_cfg));
    if (cfg)
        g_cfg = *cfg;
    if (!g_cfg.record_dir)
        snprintf(g_dir, sizeof(g_dir), "%s", REC_DEFAULT_DIR);
    else
        snprintf(g_dir, sizeof(g_dir), "%s", g_cfg.record_dir);
    g_cfg.record_dir = NULL; /* 只留 g_dir 内拷贝，避免悬垂指针 */
    if (g_cfg.cap_bytes)
        g_cap_bytes = g_cfg.cap_bytes;
    if (!g_cfg.width)  g_cfg.width = 640;
    if (!g_cfg.height) g_cfg.height = 480;
    if (!g_cfg.nominal_fps) g_cfg.nominal_fps = 120;
    if (!g_cfg.bitrate_kbps) g_cfg.bitrate_kbps = 12000;

    /* 控制 socket（/sdcard/app 支持 unix socket，与旧版一致） */
    g_listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (g_listen_fd >= 0) {
        struct sockaddr_un addr;
        unlink(REC_CTRL_SOCKET);
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, REC_CTRL_SOCKET, sizeof(addr.sun_path) - 1);
        if (bind(g_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
            listen(g_listen_fd, 2) != 0) {
            close(g_listen_fd);
            g_listen_fd = -1;
        } else {
            fcntl(g_listen_fd, F_SETFL, O_NONBLOCK);
        }
    }
    if (g_listen_fd < 0)
        rec_log("recorder: control socket unavailable\n");

    if (pthread_create(&g_feed_tid, NULL, rec_feed_thread, NULL) != 0 ||
        pthread_create(&g_drain_tid, NULL, rec_drain_thread, NULL) != 0 ||
        pthread_create(&g_ctrl_tid, NULL, rec_ctrl_thread, NULL) != 0) {
        rec_log("recorder: thread create failed\n");
        return -1;
    }
    g_init = 1;
    rec_log("recorder: init ok dir=%s cap=%" PRIu64 " bitrate=%u kbps\n",
            g_dir, g_cap_bytes, g_cfg.bitrate_kbps);
    return 0;
}

void rec_deinit(void)
{
    if (!g_init)
        return;
    pthread_mutex_lock(&g_op); /* 与 control 线程的 start/stop/常录续录串行 */
    g_exit = 1;                /* 先置退出：control 随后的自动开录会失败退出 */
    rec_session_stop();
    pthread_mutex_unlock(&g_op);
    pthread_cond_broadcast(&g_cond);
    pthread_join(g_feed_tid, NULL);
    pthread_join(g_drain_tid, NULL);
    pthread_join(g_ctrl_tid, NULL);
    if (g_pools_ok) {
        for (int i = 0; i < IN_RING_N; i++) {
            if (g_slots[i].va) {
                kd_mpi_sys_munmap(g_slots[i].va, g_in_blk);
                g_slots[i].va = NULL;
            }
            if (g_slots[i].handle) {
                kd_mpi_vb_release_block(g_slots[i].handle);
                g_slots[i].handle = 0;
            }
        }
        if (g_in_pool != VB_INVALID_POOLID)
            kd_mpi_vb_destory_pool(g_in_pool);
        if (g_out_pool != VB_INVALID_POOLID)
            kd_mpi_vb_destory_pool(g_out_pool);
        g_in_pool = g_out_pool = VB_INVALID_POOLID;
        g_pools_ok = 0;
    }
    g_init = 0;
    rec_log("recorder: deinit done\n");
}

int rec_start(void)
{
    pthread_mutex_lock(&g_op);
    g_auto_mode = 1; /* 手动/外部开启 = 进入常录模式 */
    int rc = rec_session_start();
    pthread_mutex_unlock(&g_op);
    return rc;
}

int rec_stop(void)
{
    pthread_mutex_lock(&g_op);
    g_auto_mode = 0; /* 手动/外部停止 = 退出常录模式（暂停，直到再次 start） */
    rec_session_stop();
    pthread_mutex_unlock(&g_op);
    return 0;
}

int rec_is_active(void)
{
    return g_active ? 1 : 0;
}

void rec_note_result(uint64_t frame_seq, int32_t center_x, int32_t center_y,
                     uint32_t detect_fps)
{
    pthread_mutex_lock(&g_lock);
    g_last_result.seq = frame_seq;
    g_last_result.center_x = center_x;
    g_last_result.center_y = center_y;
    g_last_result.fps = detect_fps;
    pthread_mutex_unlock(&g_lock);
}

int rec_feed_frame(uint64_t phys_y, uint32_t stride, uint32_t width,
                   uint32_t height, uint64_t frame_seq, uint64_t t_us)
{
    (void)width;
    if (!g_active || !g_pools_ok)
        return 0;
    uint64_t total = (uint64_t)stride * height * 3u / 2u;
    if (total > g_in_blk) {
        static int warned;
        if (!warned++)
            rec_log("recorder: frame bigger than block, dropped\n");
        return 0;
    }
    uint64_t ysize = (uint64_t)stride * height;

    pthread_mutex_lock(&g_lock);
    if (!g_active) {
        pthread_mutex_unlock(&g_lock);
        return 0;
    }
    rec_slot_t *s = NULL;
    for (int i = 0; i < IN_RING_N; i++)
        if (g_slots[i].state == SLOT_FREE) { s = &g_slots[i]; break; }
    if (!s) {
        g_stats.frames_dropped++; /* 环形满：丢帧，不影响采集 */
        pthread_mutex_unlock(&g_lock);
        return 0;
    }
    s->state = SLOT_RESERVED; /* feed 只取 QUEUED，避免拷到一半被送编码 */
    s->stride = stride;
    s->frame_seq = frame_seq;
    s->mono_us = t_us;
    s->result_seq = g_last_result.seq;
    s->center_x = g_last_result.center_x;
    s->center_y = g_last_result.center_y;
    s->detect_fps = g_last_result.fps;
    g_stats.frames_in++;
    pthread_mutex_unlock(&g_lock);

    /* 拷帧（源是采集线程当前持有的 VICAP dump buffer，本调用期间有效） */
    uint8_t *src = (uint8_t *)kd_mpi_sys_mmap(phys_y, total);
    if (!src) {
        pthread_mutex_lock(&g_lock);
        if (s->state == SLOT_RESERVED)
            s->state = SLOT_FREE;
        pthread_mutex_unlock(&g_lock);
        return 0;
    }
    memcpy(s->va, src, ysize);
    memcpy(s->va + ysize, src + ysize, total - ysize);
    kd_mpi_sys_munmap(src, total);
    pthread_mutex_lock(&g_lock);
    if (s->state == SLOT_RESERVED)
        s->state = SLOT_QUEUED; /* 拷贝完成，feed 才可见 */
    pthread_cond_broadcast(&g_cond); /* 唤醒 feed */
    pthread_mutex_unlock(&g_lock);
    return 1;
}

void rec_signal_start(void)
{
    g_sig_start = 1; /* 信号上下文：仅置位 */
}

void rec_signal_stop(void)
{
    g_sig_stop = 1;
}

void rec_set_cap_bytes(uint64_t cap)
{
    pthread_mutex_lock(&g_op);
    g_cap_bytes = cap ? cap : REC_DEFAULT_CAP_BYTES;
    pthread_mutex_unlock(&g_op);
    rec_log("recorder: cap set to %" PRIu64 "\n", g_cap_bytes);
}

void rec_get_stats(recorder_stats_t *stats)
{
    if (!stats)
        return;
    pthread_mutex_lock(&g_lock);
    *stats = g_stats;
    pthread_mutex_unlock(&g_lock);
}
