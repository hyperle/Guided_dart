/*
 * ============================================================================
 * record —— 录像子系统（v4：应用层拷贝 + 私有 VB 块喂编码器）
 *
 * 【为什么不再"零拷贝交棒"】
 *   前几轮的形态是：识别线程 dump 出 CHN0 的一帧 -> 处理 -> 把**同一个 VICAP
 *   缓冲**交棒给编码器 -> 等取到码流后才归还。板端实测这套形态有两个后果：
 *     1) VICAP 的缓冲区被编码器占着，识别侧的缓冲/锁全被拖着走；
 *     2) bind 双通道更直接：CHN1 直连 VENC 会把 CHN0 拖死（4~147 帧后永久
 *        NOTREADY），因为编码器持有 CHN1 帧形成反压，把 ISP 两路输出一起卡住。
 *   所以改成：**识别线程把帧拷一份到录像自己的 VB 块**，拷完立刻把 VICAP 帧
 *   交回给采集线程归还；编码器只碰录像自己的块。代价是每帧一次 640x480 NV12
 *   的 memcpy（460800 字节，实测 copy= 字段有数），换来的是三条流水彻底解耦。
 *
 * 【三条流水，各自只碰自己的东西】
 *   识别：  采集线程 dump/release VICAP（不加锁，见 vicap_src.c 注释）
 *   录像：  本文件的 VENC 线程 send_frame/get_stream（venc_enter 专用锁）
 *   写盘：  writer 线程只碰 RAM 环和文件系统
 *
 * 【没有 sleep，也不靠超时做调度】
 *   · VENC 线程永远阻塞在"等下一个拷贝块"的条件变量上，或阻塞在
 *     send_frame / get_stream 上 —— 都是**等事件**，不是轮询。
 *   · send_frame 用 -1（阻塞到编码器接收），get_stream 用 1000ms 只为进程能
 *     停下来兜底；稳态下"送一帧必出一包"，两个调用都是立刻返回。
 *   · 录像跟不上时**只丢录像**：没有空闲块就直接放弃这一帧（blk_empty++），
 *     识别侧一秒都不会等它。
 *
 * 【块所有权】
 *   free ──take_frame(拷贝)──> queue ──VENC线程send──> inflight ──取到码流──> free
 *   borrow 模式（对照用）不拷贝：直接把 VICAP 帧送编码器，send 返回后立刻通过
 *   done() 交回识别线程去 release —— 这是官方 sample_uvc_dev_vicap 的写法。
 * ============================================================================
 */
#include "record.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "app.h"
#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_venc_api.h"
#include "rec_dir.h"
#include "vicap_src.h"

#define VENC_CHN        0
#define OUT_BUF_N       8          /* VENC 码流输出池块数 */
#define MAX_PACKS       32         /* 单次取流最大包数（预分配） */
#define BLK_N           6          /* 拷贝块数：6 x 451KB ≈ 2.7MB */
#define RING_BYTES      (2u << 20) /* 码流 RAM 环：编码后几 Mbps，2MB 可吸收数秒卡顿 */
#define WRITE_MAX       (256u << 10)
#define FS_FLUSH_MS     1000ull    /* 码流/CSV fsync 周期（沿用旧工程 1 秒 1 次） */

#define VENC_SEND_TO_MS (-1)       /* 阻塞到编码器接收（官方 smart_ipc 同款语义） */
#define VENC_GET_TO_MS  1000       /* 只为关停兜底；稳态立刻返回 */

typedef struct { uint32_t len; uint32_t kind; } ring_hdr_t;
#define RING_KIND_H264     0u   /* 视频包（计一帧） */
#define RING_KIND_CSV      1u
#define RING_KIND_H264_HDR 2u   /* SPS/PPS：要写进文件，但不算一帧 */

#define SPS_PPS_MAX 1024u       /* 参数集缓存上限（实际 SPS+PPS 只有几十字节） */

/* ============================ 状态 ============================ */

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond = PTHREAD_COND_INITIALIZER;

static volatile int g_run;            /* VENC 线程存活 */
static volatile int g_want;           /* 是否落盘（常录开关） */
static volatile int g_writer_run;     /* writer 线程存活 */

static pthread_t    g_venc_tid, g_writer_tid;
static int          g_venc_started, g_writer_started;

static char         g_dir[256];
static uint64_t     g_cap_bytes = REC_DEFAULT_CAP_BYTES;

static int          g_out_pools_ok;
static k_u32        g_out_pool = VB_INVALID_POOLID;
static k_u32        g_out_blk_size;
static int          g_venc_ready;

/* 拷贝块池（懒建：第一帧到了才知道真实的 stride/格式） */
typedef struct {
    k_vb_blk_handle h;
    k_u64           phys;
    uint8_t        *va;            /* 非 cache 映射（官方 OSD 拷贝同款） */
    k_video_frame_info f;          /* 构造好的帧描述，直接送编码器 */
} rec_blk_t;

static rec_blk_t    g_blk[BLK_N];
static int          g_blk_ready;
static k_u32        g_in_pool = VB_INVALID_POOLID;
static uint32_t     g_blk_stride, g_blk_w, g_blk_h, g_blk_fmt;

/* 块状态机（全部在 g_lock 下操作，锁内只做数组操作，绝不阻塞） */
typedef struct {
    int                blk;        /* >=0: 拷贝块索引；-1: borrow 的 VICAP 帧 */
    k_video_frame_info f;          /* 送给编码器的帧描述 */
    rec_done_fn        done;       /* borrow 模式：编码器用完/送完后的回调 */
    void              *ctx;
} venc_job_t;

static int          g_free[BLK_N], g_free_n;
static venc_job_t   g_queue[BLK_N];
static int          g_queue_n;
static venc_job_t   g_inflight[BLK_N];
static int          g_inflight_n;

/* 抽样（30->25 每 6 帧丢 1，确定性） */
static uint32_t     g_src_fps = 30, g_dst_fps = 25;
static uint32_t     g_samp_acc;

/* 会话（只在 writer 线程里打开/关闭） */
static FILE        *g_fh264;
static FILE        *g_fcsv;
static uint32_t     g_session_idx;
static volatile int g_session_open;
static uint64_t     g_session_bytes;
static uint64_t     g_session_frames;
static uint64_t     g_csv_index;
static uint64_t     g_session_start_us;

/* 码流 RAM 环 */
static uint8_t     *g_ring;
static uint8_t     *g_wscratch;
static uint32_t     g_ring_w, g_ring_r, g_ring_used;
static pthread_mutex_t g_ring_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_ring_cond = PTHREAD_COND_INITIALIZER;

static k_venc_pack  g_packs[MAX_PACKS];
static uint64_t     g_last_flush_ms;

/*
 * SPS/PPS（K_VENC_HEADER）缓存。
 *
 * 【板端实测的病历】K230 的 VENC 把参数集作为**独立的 header 包**、**只在码流开始时
 * 出现一次**（SDK 的 sample_webrtc 注释原话："typically once at stream start"）。
 * 我们原来的写法是"只在 send_frame 之后才去取流"，等第一次 get_stream 时那个 header
 * 包早就不在了 —— 于是**所有**录下来的 .h264 里一个 SPS/PPS 都没有，主机播放就是
 *   non-existing PPS 1 referenced / decode_slice_header error / no frame!
 * （旧版本 bind 模式的录像也一样缺，属于一直存在、只是没人真正解码播放过的问题。）
 *
 * 修法（三条一起上）：
 *   1) start_chn 之后**立刻**取一次流，把 header 包抓住缓存；
 *   2) 正常取流路径里只要再出现 header 包，同样缓存；
 *   3) 每个会话文件开头先写缓存的参数集，并且每个 I 帧前面再重复一次
 *      （和 sample_webrtc 的做法一致：每次 I 帧前都补 SPS/PPS）。
 */
static uint8_t      g_sps_pps[SPS_PPS_MAX];
static uint32_t     g_sps_pps_len;
static uint32_t     g_hdr_packs;      /* 收到过多少个 header 包 */
static uint32_t     g_dbg_batches;    /* 前几批码流打印包类型（定位用） */

static record_stats_t g_stats;

/* ============================ 小工具 ============================ */

static void rec_mkdir(void)
{
    (void)mkdir(g_dir, 0777);
}

/* ============================ RAM 环 ============================ */

static int ring_init(void)
{
    if (g_ring)
        return 0;
    g_ring = (uint8_t *)malloc(RING_BYTES);
    g_wscratch = (uint8_t *)malloc(WRITE_MAX);
    if (!g_ring || !g_wscratch) {
        free(g_ring); free(g_wscratch);
        g_ring = NULL; g_wscratch = NULL;
        app_log(APP_NAME_STR ": rec RAM 环分配失败（录像会退回直写 SD，有反压风险）\n");
        return -1;
    }
    g_ring_w = g_ring_r = g_ring_used = 0;
    return 0;
}

/* 生产者：非阻塞；装不下返回 -1（丢录像帧，绝不等待） */
static int ring_write(uint32_t kind, const uint8_t *data, uint32_t len)
{
    if (!g_ring || len == 0 || len > WRITE_MAX)
        return -1;

    pthread_mutex_lock(&g_ring_lock);
    uint32_t need = (uint32_t)sizeof(ring_hdr_t) + len;
    if (RING_BYTES - g_ring_used < need + sizeof(ring_hdr_t)) {
        pthread_mutex_unlock(&g_ring_lock);
        g_stats.ring_dropped += len;
        return -1;
    }
    uint32_t tail = RING_BYTES - g_ring_w;
    if (tail < need) {                       /* 尾部放不下：填充头 + 绕回（读写两侧对称记账） */
        ring_hdr_t pad = { 0u, kind };
        memcpy(g_ring + g_ring_w, &pad, sizeof(pad));
        g_ring_used += tail;
        g_ring_w = 0;
    }
    ring_hdr_t h = { len, kind };
    memcpy(g_ring + g_ring_w, &h, sizeof(h));
    uint32_t p = g_ring_w + (uint32_t)sizeof(h);
    uint32_t first = RING_BYTES - p;
    if (first >= len) {
        memcpy(g_ring + p, data, len);
    } else {
        memcpy(g_ring + p, data, first);
        memcpy(g_ring, data + first, len - first);
    }
    g_ring_w = (p + len) % RING_BYTES;
    g_ring_used += need;
    pthread_cond_broadcast(&g_ring_cond);
    pthread_mutex_unlock(&g_ring_lock);
    return 0;
}

/* 消费者（只在 writer 线程调用）：取一条记录到 scratch。环空时**阻塞等**，不轮询 */
static uint32_t ring_take(uint32_t *kind)
{
    pthread_mutex_lock(&g_ring_lock);
    while (g_writer_run && g_ring_used == 0)
        pthread_cond_wait(&g_ring_cond, &g_ring_lock);
    if (g_ring_used == 0) {
        pthread_mutex_unlock(&g_ring_lock);
        return 0;
    }
    ring_hdr_t h;
    memcpy(&h, g_ring + g_ring_r, sizeof(h));
    if (h.len == 0) {                        /* 填充头：跳到环首（与写端对称） */
        g_ring_used -= (RING_BYTES - g_ring_r);
        g_ring_r = 0;
        pthread_mutex_unlock(&g_ring_lock);
        return 0;
    }
    if (h.len > WRITE_MAX) {                 /* 异常长度：丢弃该记录自保 */
        g_ring_used -= (uint32_t)sizeof(h) + h.len;
        g_ring_r = (g_ring_r + (uint32_t)sizeof(h) + h.len) % RING_BYTES;
        pthread_mutex_unlock(&g_ring_lock);
        return 0;
    }
    uint32_t p = g_ring_r + (uint32_t)sizeof(h);
    uint32_t first = RING_BYTES - p;
    if (first >= h.len) {
        memcpy(g_wscratch, g_ring + p, h.len);
    } else {
        memcpy(g_wscratch, g_ring + p, first);
        memcpy(g_wscratch + first, g_ring, h.len - first);
    }
    g_ring_r = (p + h.len) % RING_BYTES;
    g_ring_used -= (uint32_t)sizeof(h) + h.len;
    *kind = h.kind;
    pthread_mutex_unlock(&g_ring_lock);
    return h.len;
}

/* ============================ 会话（只在 writer 线程里调用） ============================ */

static int rec_session_open(void)
{
    rec_mkdir();
    int evicted = rec_dir_evict(g_dir, g_cap_bytes);
    if (evicted > 0)
        app_log(APP_NAME_STR ": rec evicted %d oldest session(s) to stay under cap=%llu\n",
                evicted, (unsigned long long)g_cap_bytes);
    else if (evicted < 0)
        app_log(APP_NAME_STR ": rec dir %s 打不开，跳过容量淘汰\n", g_dir);

    g_session_idx = rec_dir_next_index(g_dir);
    char path[600];
    snprintf(path, sizeof(path), "%s/rec_%04u.h264", g_dir, g_session_idx);
    g_fh264 = fopen(path, "wb");
    if (!g_fh264) {
        app_log(APP_NAME_STR ": rec open %s failed: %s\n", path, strerror(errno));
        return -1;
    }
    snprintf(path, sizeof(path), "%s/rec_%04u.csv", g_dir, g_session_idx);
    g_fcsv = fopen(path, "w");
    if (g_fcsv)
        fprintf(g_fcsv, "frame_seq,result_seq,mono_us,center_x,center_y,detect_fps\n");

    g_session_bytes = 0;
    g_session_frames = 0;
    g_csv_index = 0;

    /* 每个文件都自带 SPS/PPS（录像会滚动成多个文件，谁都不能依赖别人） */
    uint32_t hdr_len = 0;
    uint8_t  hdr_copy[SPS_PPS_MAX];
    pthread_mutex_lock(&g_lock);
    hdr_len = g_sps_pps_len;
    if (hdr_len)
        memcpy(hdr_copy, g_sps_pps, hdr_len);
    pthread_mutex_unlock(&g_lock);
    if (hdr_len && fwrite(hdr_copy, 1, hdr_len, g_fh264) == hdr_len)
        g_session_bytes += hdr_len;

    g_session_start_us = mono_us();
    g_session_open = 1;
    pthread_mutex_lock(&g_lock);
    g_stats.sessions++;
    pthread_mutex_unlock(&g_lock);

    app_log(APP_NAME_STR ": rec session rec_%04u start (dir=%s cap=%llu 头部=%u 字节)\n",
            g_session_idx, g_dir, (unsigned long long)g_cap_bytes, hdr_len);
    return 0;
}

static void rec_session_close(void)
{
    if (!g_session_open)
        return;
    if (g_fh264) {
        fflush(g_fh264);
        fsync(fileno(g_fh264));
        fclose(g_fh264);
        g_fh264 = NULL;
    }
    if (g_fcsv) {
        fflush(g_fcsv);
        fsync(fileno(g_fcsv));
        fclose(g_fcsv);
        g_fcsv = NULL;
    }
    app_log(APP_NAME_STR ": rec session rec_%04u stop (frames=%llu bytes=%llu)\n",
            g_session_idx, (unsigned long long)g_session_frames,
            (unsigned long long)g_session_bytes);
    g_session_open = 0;
    g_session_idx = 0;
}

/* 把环里剩余记录写完（收会话/滚动前调用；只在 writer 线程） */
static void ring_flush_to_files(void)
{
    for (int guard = 0; guard < 4096; ++guard) {
        uint32_t kind = 0;
        uint32_t len = ring_take(&kind);
        if (len == 0)
            break;
        if (kind == RING_KIND_H264) {
            if (g_fh264 && fwrite(g_wscratch, 1, len, g_fh264) == len) {
                g_session_bytes += len;
                g_session_frames++;
                g_stats.written++;
            }
        } else if (kind == RING_KIND_H264_HDR) {
            if (g_fh264 && fwrite(g_wscratch, 1, len, g_fh264) == len)
                g_session_bytes += len;      /* 参数集：写文件但不计帧 */
        } else if (g_fcsv) {
            fwrite(g_wscratch, 1, len, g_fcsv);
        }
    }
    if (g_fh264) { fflush(g_fh264); fsync(fileno(g_fh264)); }
    if (g_fcsv)  { fflush(g_fcsv);  fsync(fileno(g_fcsv)); }
}

static void *rec_writer_thread(void *arg)
{
    (void)arg;
    for (;;) {
        /* 会话状态机：开会话 / 收会话 / 到量滚动 —— 全部在 writer 线程，SD I/O 不碰取流线程 */
        if (g_want && !g_session_open) {
            if (rec_session_open() != 0)
                g_want = 0;
        } else if (!g_want && g_session_open) {
            ring_flush_to_files();
            rec_session_close();
        } else if (g_session_open && g_session_bytes >= REC_ROLL_BYTES) {
            app_log(APP_NAME_STR ": rec roll at %llu bytes\n",
                    (unsigned long long)g_session_bytes);
            ring_flush_to_files();
            rec_session_close();
        }

        uint32_t kind = 0;
        uint32_t len = ring_take(&kind);      /* 环空则阻塞等（不 sleep、不轮询） */
        if (len == 0) {
            if (!g_writer_run && g_ring_used == 0)
                break;
            continue;
        }
        if (kind == RING_KIND_H264) {
            if (g_fh264 && fwrite(g_wscratch, 1, len, g_fh264) == len) {
                g_session_bytes += len;
                g_session_frames++;    /* 会话实测帧率 = 它 / 会话时长（fps= 字段） */
                g_stats.written++;
            }
        } else if (kind == RING_KIND_H264_HDR) {
            if (g_fh264 && fwrite(g_wscratch, 1, len, g_fh264) == len)
                g_session_bytes += len;      /* 参数集：写文件但不计帧 */
        } else if (g_fcsv) {
            fwrite(g_wscratch, 1, len, g_fcsv);
        }

        uint64_t now_ms = mono_us() / 1000ull;
        if (now_ms - g_last_flush_ms >= FS_FLUSH_MS) {
            g_last_flush_ms = now_ms;
            if (g_fh264) { fflush(g_fh264); fsync(fileno(g_fh264)); }
            if (g_fcsv)  { fflush(g_fcsv);  fsync(fileno(g_fcsv)); }
        }
    }
    if (g_fh264) { fflush(g_fh264); fsync(fileno(g_fh264)); }
    if (g_fcsv)  { fflush(g_fcsv);  fsync(fileno(g_fcsv)); }
    return NULL;
}

/* ============================ VENC 与拷贝块 ============================ */

/* 块的平面布局：NV12 = Y + UV(半高)，RGB_888_PLANAR = 三平面各 stride*h */
static void blk_setup_frame(rec_blk_t *b, uint32_t w, uint32_t h, uint32_t stride,
                            uint32_t fmt, k_mod_id mod_id)
{
    memset(&b->f, 0, sizeof(b->f));
    b->f.v_frame.width        = w;
    b->f.v_frame.height       = h;
    b->f.v_frame.stride[0]    = stride;
    b->f.v_frame.pixel_format = fmt;
    b->f.v_frame.phys_addr[0] = b->phys;
    b->f.pool_id              = g_in_pool;
    b->f.mod_id               = mod_id;   /* 照抄源帧：VENC 收 VICAP 帧时就是这个值 */
    if (fmt == PIXEL_FORMAT_RGB_888_PLANAR) {
        b->f.v_frame.stride[1] = stride;
        b->f.v_frame.stride[2] = stride;
        b->f.v_frame.phys_addr[1] = b->phys + (k_u64)stride * h;
        b->f.v_frame.phys_addr[2] = b->phys + (k_u64)stride * h * 2;
    } else {                                   /* NV12 */
        b->f.v_frame.phys_addr[1] = b->phys + (k_u64)stride * h;
    }
}

static uint32_t blk_plane_bytes(uint32_t w, uint32_t h, uint32_t stride, uint32_t fmt)
{
    (void)w;
    if (fmt == PIXEL_FORMAT_RGB_888_PLANAR)
        return stride * h * 3u;
    return stride * h * 3u / 2u;               /* NV12 */
}

/*
 * 懒建输入块池：第一帧到了才知道真实的 stride 与像素格式。
 * 非 cache 映射（与官方 OSD 拷贝路径一致），CPU 写完硬件立刻可见，不需要 flush。
 */
static int blk_pool_init(uint32_t w, uint32_t h, uint32_t stride, uint32_t fmt,
                         k_mod_id mod_id)
{
    if (g_blk_ready)
        return 0;
    if (!w || !h || !stride)
        return -1;

    uint32_t bytes = blk_plane_bytes(w, h, stride, fmt);
    uint64_t blk_size = VB_ALIGN_UP(bytes, 4096);

    mpp_enter();
    g_in_pool = kd_mpi_vb_create_pool_ex(blk_size, BLK_N, VB_REMAP_MODE_NOCACHE);
    if (g_in_pool == VB_INVALID_POOLID) {
        mpp_leave();
        app_log(APP_NAME_STR ": rec 拷贝块池创建失败 (%u 字节 x %d)\n", (unsigned)blk_size, BLK_N);
        return -1;
    }
    for (int i = 0; i < BLK_N; ++i) {
        g_blk[i].h = kd_mpi_vb_get_block(g_in_pool, blk_size, NULL);
        if (g_blk[i].h == VB_INVALID_HANDLE) {
            mpp_leave();
            app_log(APP_NAME_STR ": rec 拷贝块 %d 申请失败\n", i);
            return -1;
        }
        g_blk[i].phys = kd_mpi_vb_handle_to_phyaddr(g_blk[i].h);
        g_blk[i].va = (uint8_t *)mpp_map_persist(g_blk[i].phys, (k_u32)blk_size);
        if (!g_blk[i].phys || !g_blk[i].va) {
            mpp_leave();
            app_log(APP_NAME_STR ": rec 拷贝块 %d 映射失败\n", i);
            return -1;
        }
        blk_setup_frame(&g_blk[i], w, h, stride, fmt, mod_id);
    }
    mpp_leave();

    g_blk_stride = stride;
    g_blk_w = w;
    g_blk_h = h;
    g_blk_fmt = fmt;
    g_blk_ready = 1;

    pthread_mutex_lock(&g_lock);
    g_free_n = 0;
    for (int i = 0; i < BLK_N; ++i)
        g_free[g_free_n++] = i;
    pthread_mutex_unlock(&g_lock);

    app_log(APP_NAME_STR ": rec 拷贝块池就绪 %dx%d stride=%u fmt=%d 块=%d x %u 字节"
            "（非 cache 映射；编码器只碰这里，不再持有 VICAP 缓冲）\n",
            w, h, stride, (int)fmt, BLK_N, (unsigned)blk_size);
    return 0;
}

static void blk_pool_destroy(void)
{
    mpp_enter();
    for (int i = 0; i < BLK_N; ++i) {
        if (g_blk[i].h != VB_INVALID_HANDLE) {
            kd_mpi_vb_release_block(g_blk[i].h);
            g_blk[i].h = VB_INVALID_HANDLE;
        }
    }
    if (g_in_pool != VB_INVALID_POOLID) {
        kd_mpi_vb_destory_pool(g_in_pool);
        g_in_pool = VB_INVALID_POOLID;
    }
    mpp_leave();
    g_blk_ready = 0;
}

static void blk_push_free(int bi)
{
    pthread_mutex_lock(&g_lock);
    if (g_free_n < BLK_N)
        g_free[g_free_n++] = bi;
    pthread_mutex_unlock(&g_lock);
}

/* 一帧拷贝：源是识别侧已映射（cached+invalidate）的平面指针 */
static uint32_t rec_copy_frame(const rec_frame_t *rf, int bi)
{
    uint8_t *dst = g_blk[bi].va;
    uint32_t ysz = rf->stride * rf->height;
    uint64_t t0 = mono_us();

    if (rf->np == 3) {                       /* RGB_888_PLANAR：三平面各 stride*h */
        for (int p = 0; p < 3; ++p)
            memcpy(dst + (size_t)ysz * p, rf->p[p], ysz);
    } else {
        if (rf->stride == g_blk_stride) {
            memcpy(dst, rf->p[0], ysz);      /* Y */
            memcpy(dst + ysz, rf->p[1], ysz / 2u);
        } else {
            /* stride 不一致（理论上不会）：逐行拷，长度取两者较小值，绝不越界 */
            uint32_t row = (g_blk_stride < rf->stride) ? g_blk_stride : rf->stride;
            for (uint32_t y = 0; y < rf->height; ++y)
                memcpy(dst + (size_t)y * g_blk_stride,
                       rf->p[0] + (size_t)y * rf->stride, row);
            for (uint32_t y = 0; y < rf->height / 2u; ++y)
                memcpy(dst + (size_t)ysz + (size_t)y * g_blk_stride,
                       rf->p[1] + (size_t)y * rf->stride, row);
        }
    }
    return (uint32_t)(mono_us() - t0);
}

/* 缓存 header 包内容（VENC 线程内部调用；读写用 g_lock 保护一小下） */
static void rec_cache_header(const uint8_t *data, uint32_t len)
{
    if (!data || !len || len > SPS_PPS_MAX)
        return;
    pthread_mutex_lock(&g_lock);
    memcpy(g_sps_pps, data, len);
    g_sps_pps_len = len;
    g_hdr_packs++;
    pthread_mutex_unlock(&g_lock);
}

/* ============================ 送帧 / 取流 ============================ */

/*
 * 识别线程调用（处理完一帧、归还 VICAP 帧之前）。
 *   返回 0 = 录用（copy：已拷完，录像随后自行编码；borrow：编码器用完才回调 done）
 *   返回 1 = 不录（没开录像 / 抽样丢掉 / 没有空闲块）——调用方照常归还自己的帧
 * 本函数**绝不阻塞**：块不够就丢录像帧（best-effort），识别优先。
 */
int record_take_frame(const rec_frame_t *rf, uint64_t seq, rec_done_fn done, void *ctx)
{
    (void)seq;      /* 帧号只用于日志对齐；CSV 里的 result_seq 已经能对回视觉帧 */
    if (!rf || !rf->f)
        return 1;
    if (!g_run || !g_venc_ready || !g_want)
        return 1;

    /* 抽样：src->dst（30->25 即每 6 帧丢 1，确定性） */
    g_samp_acc += g_dst_fps;
    if (g_samp_acc < g_src_fps) {
        g_stats.sampled_out++;
        return 1;
    }
    g_samp_acc -= g_src_fps;

    if (g_cfg.rec_feed == REC_FEED_BORROW) {
        pthread_mutex_lock(&g_lock);
        if (g_queue_n >= BLK_N) {
            g_stats.queue_full++;
            pthread_mutex_unlock(&g_lock);
            return 1;
        }
        venc_job_t *j = &g_queue[g_queue_n++];
        memset(j, 0, sizeof(*j));
        j->blk = -1;
        j->f = *rf->f;
        j->done = done;
        j->ctx = ctx;
        pthread_cond_broadcast(&g_cond);
        pthread_mutex_unlock(&g_lock);
        g_stats.borrowed++;
        return 0;      /* done() 由 VENC 线程在 send_frame 返回后调用 */
    }

    /* copy：先抢块，再拷贝（拷贝不占任何锁） */
    if (!g_blk_ready) {
        if (blk_pool_init(rf->width, rf->height, rf->stride,
                          rf->f->v_frame.pixel_format, rf->f->mod_id) != 0)
            return 1;
    }
    pthread_mutex_lock(&g_lock);
    if (g_free_n == 0) {
        g_stats.blk_empty++;
        pthread_mutex_unlock(&g_lock);
        return 1;
    }
    int bi = g_free[--g_free_n];
    pthread_mutex_unlock(&g_lock);

    uint32_t us = rec_copy_frame(rf, bi);
    g_stats.copy_sum_us += us;
    if (us > g_stats.copy_max_us)
        g_stats.copy_max_us = us;
    g_stats.copied++;

    pthread_mutex_lock(&g_lock);
    venc_job_t *j = &g_queue[g_queue_n++];
    memset(j, 0, sizeof(*j));
    j->blk = bi;
    j->f = g_blk[bi].f;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);

    if (done)                       /* 拷完就把 VICAP 帧交回采集线程归还 */
        done(rf->f, ctx);
    return 0;
}

/* 一帧彻底用完：拷贝块回到 free；borrow 帧不需要做什么（done 已回调） */
static void job_finish(const venc_job_t *job)
{
    if (job->blk >= 0)
        blk_push_free(job->blk);
}

/* 取一次码流：把包拷进 RAM 环，并回收对应的在途帧。返回 1=取到，0=暂无 */
static int rec_take_stream_once(void)
{
    k_venc_stream st;
    uint32_t nvideo = 0;

    memset(&st, 0, sizeof(st));
    st.pack_cnt = MAX_PACKS;   /* 同 rec_capture_header：长度以返回值为准，别用 cur_packs 截 */
    st.pack = g_packs;
    venc_enter();
    k_s32 ret = kd_mpi_venc_get_stream(VENC_CHN, &st, VENC_GET_TO_MS);
    venc_leave();
    if (ret != K_SUCCESS)
        return 0;                        /* 编码器还没交付：这次没取到（本函数只在送帧后被调用） */

    const uint8_t *vas[MAX_PACKS];
    uint32_t lens[MAX_PACKS];
    k_u32 pack_cnt = st.pack_cnt > MAX_PACKS ? MAX_PACKS : st.pack_cnt;
    for (k_u32 i = 0; i < pack_cnt; i++) {
        vas[i] = NULL;
        lens[i] = 0;
        if (st.pack[i].len == 0)
            continue;
        /* 码流包在输出池的某个块里、每包长度不同：先归一化到块基址再用块大小映射 */
        uint64_t base = st.pack[i].phys_addr;
        uint32_t mlen = st.pack[i].len;
        k_vb_blk_handle h = kd_mpi_vb_phyaddr_to_handle(st.pack[i].phys_addr);
        if (h != VB_INVALID_HANDLE) {
            k_u64 b = kd_mpi_vb_handle_to_phyaddr(h);
            uint32_t blk = g_out_blk_size ? g_out_blk_size : mlen;
            if (b && b <= st.pack[i].phys_addr &&
                (st.pack[i].phys_addr - b) + st.pack[i].len <= blk) {
                base = b;
                mlen = blk;
            }
        }
        uint8_t *p = (uint8_t *)mpp_map_persist_cached(base, mlen);
        if (p) {
            mpp_invalidate(base, p, mlen);
            vas[i] = p + (size_t)(st.pack[i].phys_addr - base);
            lens[i] = st.pack[i].len;
        }
        if (st.pack[i].type != K_VENC_HEADER)
            nvideo++;
    }
    venc_leave();

    /* 前几批把包类型打出来：一眼能看出 header 包到底有没有来过 */
    if (g_dbg_batches < 6u) {
        g_dbg_batches++;
        char line[256];
        int off = 0;
        off += snprintf(line + off, sizeof(line) - (size_t)off, "rec 码流批 #%u packs=%u:",
                        g_dbg_batches, (unsigned)pack_cnt);
        for (k_u32 i = 0; i < pack_cnt && off < (int)sizeof(line) - 56; i++)
            off += snprintf(line + off, sizeof(line) - (size_t)off,
                            " [type=%d len=%u h=%02x%02x%02x%02x%s]",
                            (int)st.pack[i].type, (unsigned)st.pack[i].len,
                            vas[i] ? vas[i][0] : 0, vas[i] ? vas[i][1] : 0,
                            vas[i] ? vas[i][2] : 0, vas[i] ? vas[i][3] : 0,
                            vas[i] ? "" : " map失败");
        app_log(APP_NAME_STR ": %s\n", line);
    }

    /* 已取到码流：回收对应数量的在途帧（送进去的帧与出来的包严格 1:1） */
    if (nvideo) {
        venc_job_t done_jobs[BLK_N];
        uint32_t nfin = 0;
        pthread_mutex_lock(&g_lock);
        if (nvideo > (uint32_t)g_inflight_n)
            g_stats.fifo_underflow++;    /* 输出多于输入：编码器行为与预期不符 */
        while (nfin < nvideo && g_inflight_n > 0) {
            done_jobs[nfin++] = g_inflight[0];
            for (int i = 1; i < g_inflight_n; ++i)
                g_inflight[i - 1] = g_inflight[i];
            g_inflight_n--;
        }
        pthread_mutex_unlock(&g_lock);
        for (uint32_t i = 0; i < nfin; ++i)
            job_finish(&done_jobs[i]);
    }

    /* 码流进 RAM 环（不碰 SD） */
    int writing = g_session_open;
    uint32_t hdr_len, hdr_len_now;
    uint8_t  hdr_copy[SPS_PPS_MAX];
    pthread_mutex_lock(&g_lock);
    hdr_len = g_sps_pps_len;
    if (hdr_len)
        memcpy(hdr_copy, g_sps_pps, hdr_len);
    pthread_mutex_unlock(&g_lock);

    for (k_u32 i = 0; i < pack_cnt; i++) {
        if (!vas[i] || !lens[i])
            continue;
        if (st.pack[i].type == K_VENC_HEADER) {
            /* 参数集：缓存下来（会话开头 + 每个 I 帧前都要用），同时照写进文件 */
            rec_cache_header(vas[i], lens[i]);
            hdr_len = g_sps_pps_len;
            if (hdr_len)
                memcpy(hdr_copy, g_sps_pps, hdr_len);
            if (writing && ring_write(RING_KIND_H264_HDR, vas[i], lens[i]) != 0)
                g_stats.ring_dropped_frames++;
            continue;
        }
        if (writing) {
            /* I 帧前重复一次参数集：单个文件里从这里开始也能解（对齐 sample_webrtc 的做法） */
            if (st.pack[i].type == K_VENC_I_FRAME) {
                pthread_mutex_lock(&g_lock);
                hdr_len_now = g_sps_pps_len;
                if (hdr_len_now)
                    memcpy(hdr_copy, g_sps_pps, hdr_len_now);
                pthread_mutex_unlock(&g_lock);
                if (hdr_len_now) {
                    hdr_len = hdr_len_now;
                    (void)ring_write(RING_KIND_H264_HDR, hdr_copy, hdr_len);
                }
            }
            if (ring_write(RING_KIND_H264, vas[i], lens[i]) != 0)
                g_stats.ring_dropped_frames++;
        }
    }
    if (writing && nvideo) {
        char csv[96];
        result_t r;
        result_get(&r);
        int n = snprintf(csv, sizeof(csv), "%llu,%llu,%llu,%d,%d,%u\n",
                         (unsigned long long)(++g_csv_index), (unsigned long long)r.seq,
                         (unsigned long long)mono_us(), r.center_x, r.center_y, r.fps);
        if (n > 0)
            (void)ring_write(RING_KIND_CSV, (const uint8_t *)csv, (uint32_t)n);
    }

    venc_enter();
    kd_mpi_venc_release_stream(VENC_CHN, &st);
    venc_leave();

    g_stats.streams += nvideo;
    return 1;
}

/*
 * VENC 线程：唯一的编码器使用者。
 *   · 没活干时阻塞在条件变量上（等识别侧拷好一帧），不是 sleep 也不是轮询；
 *   · 送帧用 -1 阻塞到编码器接收；取流只在"确实有包"时才调用。
 */
static void *rec_venc_thread(void *arg)
{
    (void)arg;
    while (1) {
        venc_job_t job;

        pthread_mutex_lock(&g_lock);
        while (g_run && g_queue_n == 0)
            pthread_cond_wait(&g_cond, &g_lock);
        if (g_queue_n == 0) {
            pthread_mutex_unlock(&g_lock);
            if (!g_run)
                break;
            continue;
        }
        job = g_queue[0];
        for (int i = 1; i < g_queue_n; ++i)
            g_queue[i - 1] = g_queue[i];
        g_queue_n--;
        pthread_mutex_unlock(&g_lock);

        venc_enter();
        k_s32 ret = kd_mpi_venc_send_frame(VENC_CHN, &job.f, VENC_SEND_TO_MS);
        venc_leave();

        if (job.blk < 0) {
            /* borrow 模式：借来的 VICAP 帧用完（无论成败）都要交回识别去 release */
            if (job.done)
                job.done(&job.f, job.ctx);
        }
        if (ret != K_SUCCESS) {
            g_stats.send_fail++;
            job_finish(&job);
            continue;
        }
        g_stats.sent++;
        pthread_mutex_lock(&g_lock);
        if (g_inflight_n < BLK_N) {
            g_inflight[g_inflight_n++] = job;
        } else {
            g_stats.fifo_overflow++;     /* 理论上 1:1 不会发生 */
            pthread_mutex_unlock(&g_lock);
            job_finish(&job);
            continue;
        }
        pthread_mutex_unlock(&g_lock);

        (void)rec_take_stream_once();
    }
    return NULL;
}

/* ============================ 对外接口 ============================ */

/*
 * start_chn 之后立刻把参数集抓出来。
 * 返回 0=拿到了；<0=这次没有（那就等正常取流路径里再抓，不致命）。
 */
static int rec_capture_header(int timeout_ms)
{
    k_venc_stream st;
    memset(&st, 0, sizeof(st));

    /*
     * 注意：这里**故意不用 query_status.cur_packs 做预检/限长**。
     * SDK 文档说 get_stream "Stream type can be frame or header"，而 header 包
     * 有可能不计进 cur_packs —— 按 cur_packs 申请长度就会把参数集截掉，
     * 这正是"文件里永远没有 SPS/PPS"的一种可能。直接申请整批，长度以返回值为准。
     */
    st.pack_cnt = MAX_PACKS;
    st.pack = g_packs;
    venc_enter();
    if (kd_mpi_venc_get_stream(VENC_CHN, &st, timeout_ms) != K_SUCCESS) {
        venc_leave();
        return -1;
    }
    int got = 0;
    char line[256];
    int off = snprintf(line, sizeof(line), "rec start_chn 首批码流 packs=%u:", (unsigned)st.pack_cnt);
    for (k_u32 i = 0; i < st.pack_cnt && i < MAX_PACKS; i++) {
        if (st.pack[i].len == 0)
            continue;
        uint8_t *p = (uint8_t *)mpp_map_persist(st.pack[i].phys_addr, st.pack[i].len);
        if (off < (int)sizeof(line) - 48)
            off += snprintf(line + off, sizeof(line) - (size_t)off,
                            " [type=%d len=%u h=%02x%02x%02x%02x]", (int)st.pack[i].type,
                            (unsigned)st.pack[i].len, p ? p[0] : 0, p ? p[1] : 0,
                            p ? p[2] : 0, p ? p[3] : 0);
        if (!p)
            continue;
        if (st.pack[i].type == K_VENC_HEADER) {
            rec_cache_header(p, st.pack[i].len);
            got = 1;
        }
        /* 此刻还没有任何帧送进编码器，理论上不会出现视频包；真出现了也只可能是
         * 空跑产生的，丢弃不影响录像（会话还没开）。 */
    }
    kd_mpi_venc_release_stream(VENC_CHN, &st);
    venc_leave();
    app_log(APP_NAME_STR ": %s%s\n", line, got ? "  <= 含参数集，已缓存" : "");
    return got ? 0 : -1;
}

int record_setup(void)
{
    uint32_t w = vicap_chn_width(VICAP_CHN_VISION);
    uint32_t h = vicap_chn_height(VICAP_CHN_VISION);
    if (!w || !h) {
        app_log(APP_NAME_STR ": rec 编码尺寸无效（通道未配置？）\n");
        return -1;
    }
    g_out_blk_size = VB_ALIGN_UP((uint64_t)w * h, 4096);

    mpp_enter();
    g_out_pool = kd_mpi_vb_create_pool_ex(g_out_blk_size, OUT_BUF_N, VB_REMAP_MODE_NOCACHE);
    mpp_leave();
    if (g_out_pool == VB_INVALID_POOLID) {
        app_log(APP_NAME_STR ": rec 输出 vb 池创建失败 (%u 字节 x %d)\n",
                g_out_blk_size, OUT_BUF_N);
        return -1;
    }
    g_out_pools_ok = 1;

    /*
     * 源帧率填"实际喂进编码器的帧率"：应用已按 rec_fps 抽样，编码器按 src=dst
     * 配置不做内部丢帧 —— 这样"送进去的帧数"与"取出来的包数"才是严格 1:1。
     */
    uint32_t fps = (uint32_t)(g_cfg.rec_fps > 0 ? g_cfg.rec_fps : 25);

    mpp_enter();
    if (kd_mpi_venc_attach_vb_pool(VENC_CHN, g_out_pool) != K_SUCCESS) {
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc attach vb pool 失败\n");
        return -1;
    }
    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.type = K_PT_H264;
    attr.venc_attr.pic_width = w;
    attr.venc_attr.pic_height = h;
    attr.venc_attr.profile = VENC_PROFILE_H264_HIGH;
    attr.rc_attr.rc_mode = K_VENC_RC_MODE_CBR;
    attr.rc_attr.cbr.src_frame_rate = fps;
    attr.rc_attr.cbr.dst_frame_rate = fps;
    attr.rc_attr.cbr.gop = fps;
    attr.rc_attr.cbr.bit_rate = g_cfg.bitrate_kbps;
    if (kd_mpi_venc_create_chn(VENC_CHN, &attr) != K_SUCCESS) {
        kd_mpi_venc_detach_vb_pool(VENC_CHN);
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc create chn 失败\n");
        return -1;
    }
    g_venc_ready = 1;
    if (kd_mpi_venc_start_chn(VENC_CHN) != K_SUCCESS) {
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc start chn 失败\n");
        return -1;
    }
    mpp_leave();

    /* 参数集只在码流开始时出现一次：start_chn 之后立刻抓，抓不到就在正常路径里补抓 */
    if (rec_capture_header(200) == 0)
        app_log(APP_NAME_STR ": rec 已抓到 SPS/PPS %u 字节（每个会话文件开头都会写上）\n",
                g_sps_pps_len);
    else
        app_log(APP_NAME_STR ": rec start_chn 后暂无 header 包，改在取流路径里补抓\n");

    app_log(APP_NAME_STR ": rec 模式=%s（%s）%ux%u @%u fps bitrate=%u kbps；"
            "编码器只吃录像私有块，VICAP 通道只有 CHN%d 一条\n",
            g_cfg.rec_feed == REC_FEED_BORROW ? "borrow" : "copy",
            g_cfg.rec_feed == REC_FEED_BORROW
                ? "官方 uvc 样例写法：直接送 VICAP 帧，送完立刻归还，零拷贝"
                : "应用层拷贝：拷进录像私有 VB 块，VICAP 帧拷完即归还",
            w, h, fps, g_cfg.bitrate_kbps, (int)VICAP_CHN_VISION);
    return 0;
}

int record_run(void)
{
    snprintf(g_dir, sizeof(g_dir), "%s", g_cfg.rec_dir ? g_cfg.rec_dir : REC_DEFAULT_DIR);
    g_cap_bytes = g_cfg.cap_bytes ? g_cfg.cap_bytes : REC_DEFAULT_CAP_BYTES;
    g_want = g_cfg.record_on ? 1 : 0;
    g_src_fps = vicap_acq_fps() ? vicap_acq_fps() : 30u;
    g_dst_fps = (uint32_t)(g_cfg.rec_fps > 0 ? g_cfg.rec_fps : 25);
    if (!g_dst_fps || g_dst_fps > g_src_fps)
        g_dst_fps = g_src_fps;

    g_stats.enabled = g_want;
    g_stats.feed = (g_cfg.rec_feed == REC_FEED_BORROW) ? 1 : 0;
    g_stats.hdr_bytes = g_sps_pps_len;
    g_stats.dir = g_dir;
    g_stats.cap_bytes = g_cap_bytes;

    ring_init();
    g_writer_run = 1;
    if (pthread_create(&g_writer_tid, NULL, rec_writer_thread, NULL) == 0)
        g_writer_started = 1;

    g_run = 1;
    if (pthread_create(&g_venc_tid, NULL, rec_venc_thread, NULL) == 0)
        g_venc_started = 1;

    if (!g_venc_started)
        app_log(APP_NAME_STR ": rec VENC 线程创建失败\n");
    else
        app_log(APP_NAME_STR ": rec 线程就绪（VENC=%d 写盘=%d）源=%u fps 目标=%u fps"
                "（确定性抽样，不 sleep）\n", g_venc_started, g_writer_started,
                g_src_fps, g_dst_fps);
    return g_venc_started ? 0 : -1;
}

void record_deinit(void)
{
    g_run = 0;
    g_want = 0;

    pthread_mutex_lock(&g_lock);
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
    pthread_mutex_lock(&g_ring_lock);
    pthread_cond_broadcast(&g_ring_cond);
    pthread_mutex_unlock(&g_ring_lock);

    if (g_venc_started) {
        pthread_join(g_venc_tid, NULL);
        g_venc_started = 0;
    }
    /* 只剩本线程：把编码器里剩的码流取完，并回收所有在途块 */
    for (int i = 0; i < 200; ++i) {
        if (!rec_take_stream_once())
            break;
    }

    pthread_mutex_lock(&g_lock);
    venc_job_t rel[BLK_N];
    int nrel = g_inflight_n;
    for (int i = 0; i < nrel; ++i)
        rel[i] = g_inflight[i];
    g_inflight_n = 0;
    int nq = g_queue_n;
    for (int i = 0; i < nq && i < BLK_N; ++i)
        rel[nrel + i] = g_queue[i];
    g_queue_n = 0;
    pthread_mutex_unlock(&g_lock);
    for (int i = 0; i < nrel + nq; ++i) {
        if (rel[i].blk < 0 && rel[i].done)      /* borrow 帧：交回识别去 release */
            rel[i].done(&rel[i].f, rel[i].ctx);
        else
            job_finish(&rel[i]);
    }

    if (g_writer_started) {
        g_writer_run = 0;
        pthread_mutex_lock(&g_ring_lock);
        pthread_cond_broadcast(&g_ring_cond);
        pthread_mutex_unlock(&g_ring_lock);
        pthread_join(g_writer_tid, NULL);
        g_writer_started = 0;
    }

    blk_pool_destroy();

    venc_enter();
    if (g_venc_ready) {
        kd_mpi_venc_stop_chn(VENC_CHN);
        kd_mpi_venc_destroy_chn(VENC_CHN);
        kd_mpi_venc_detach_vb_pool(VENC_CHN);
        g_venc_ready = 0;
    }
    if (g_out_pools_ok) {
        kd_mpi_vb_destory_pool(g_out_pool);
        g_out_pool = VB_INVALID_POOLID;
        g_out_pools_ok = 0;
    }
    venc_leave();

    free(g_ring);       g_ring = NULL;
    free(g_wscratch);   g_wscratch = NULL;
}

void record_set_enabled(int on)
{
    pthread_mutex_lock(&g_lock);
    g_want = on ? 1 : 0;
    g_stats.enabled = g_want;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
}

int record_enabled(void)
{
    return g_want;
}

int record_is_active(void)
{
    return g_session_open;
}

int record_is_started(void)
{
    return g_venc_started;
}

/* 识别侧上报**实测**处理帧率，供抽样器当分母用（比 vicap 请求值准） */
void record_set_src_fps(uint32_t fps)
{
    if (fps < 1 || fps > 1000)
        return;
    g_src_fps = fps;
    if (g_samp_acc > g_src_fps)
        g_samp_acc = 0;
}

void record_set_cap(uint64_t bytes)
{
    g_cap_bytes = bytes;
}

int record_set_dir(const char *dir)
{
    if (!dir)
        return -1;
    snprintf(g_dir, sizeof(g_dir), "%s", dir);
    return 0;
}

void record_get_stats(record_stats_t *st)
{
    if (!st)
        return;
    pthread_mutex_lock(&g_lock);
    *st = g_stats;
    st->active = g_session_open;
    st->session_index = g_session_idx;
    st->bytes = g_session_bytes;
    st->hdr_bytes = g_sps_pps_len;
    st->free_blocks = g_free_n;
    st->queue_blocks = g_queue_n;
    st->inflight_blocks = g_inflight_n;
    uint64_t dur = mono_us() - g_session_start_us;
    st->real_fps = (g_session_open && dur > 0)
                       ? (uint32_t)((g_session_frames * 1000000ull) / dur)
                       : 0;
    pthread_mutex_unlock(&g_lock);
}
