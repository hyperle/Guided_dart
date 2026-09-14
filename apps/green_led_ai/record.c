/*
 * record —— 录像子系统（与识别共享同一个 VICAP 通道，取流侧唯一归还）
 *
 * ============================================================================
 * 设计依据（全部来自本板实测，不是推测）
 *
 * 1) 采集通道只能有一个：本板四次运行里，"双通道 + CHN1 硬件绑定 VENC" 的形态
 *    都在 200~400 帧后衰减到停（dumped 每 2 秒增量 58→37→29→6→0），而且 CHN1 的
 *    streams 也同时冻结；相反，旧工程 green_led_rtos 的 "单通道 + OFFLINE +
 *    连续长超时 dump + send_frame" 在同一块板上稳定跑了 40079 帧。
 *    => 因此默认走单通道共享形态（rec_mode=shared），双通道绑定只作为可选实验。
 *
 * 2) 帧的所有权必须唯一且明确。旧工程的恶性 bug 是：同一帧既送 VENC 又由识别
 *    线程 kd_mpi_vicap_dump_release，VB 引用计数被打崩（读到全黑帧 / 通道饿死）。
 *    这里的纪律：
 *      vision 线程 dump 出帧 -> 处理完 -> record_offer_frame()
 *          ├─ 收下(0)：所有权交给录像子系统，vision 绝不再碰
 *          └─ 拒收(1)：vision 自己归还
 *      录像送帧线程：抽样丢的 / send_frame 失败的 -> 自己归还
 *                    send_frame 成功的 -> 放进"在途 FIFO"（编码器在读）
 *      取流线程：每取到一个视频包 -> 从在途 FIFO 弹出对应帧并归还（唯一归还者）
 *    因为抽样比例是确定的（30->25 即每 6 帧丢 1 帧）、且编码器按 src=dst 配置
 *    不做内部丢帧，"送进去的帧数"与"取出来的包数"严格 1:1，不会有偏差。
 *
 * 3) SD 卡绝不能出现在编码/采集的关键路径上。取流线程只把码流拷进 2MB RAM 环
 *    （编码后才几 Mbps，拷贝可忽略）并立刻 release_stream；真正的写盘、会话开关、
 *    滚动、容量淘汰全在独立的 writer 线程里，且 fsync 限 1 秒 1 次。
 *    这样 SD 抖动最多让 RAM 环溢出丢录像帧，绝不会反压到编码器/VICAP。
 *
 * 4) 运行期 record on/off 只控制"要不要落盘"：编码器保持运行（空转），
 *    想彻底不建 VENC/不占资源就用启动参数 --record off。
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
#define OUT_BUF_N       8         /* VENC 输出缓冲块数 */
#define MAX_PACKS       32        /* 单次取流最大包数（预分配，锁内不 malloc） */
#define INFLIGHT_MAX    4         /* 已送编码器、尚未从码流取回的帧数上限 */
#define RING_BYTES      (2u << 20)/* 码流 RAM 环：编码后 6Mbps，2MB 可吸收约 3 秒卡顿 */
#define WRITE_MAX       (256u << 10)
#define FS_FLUSH_MS     1000ull   /* 码流/CSV fsync 周期（旧工程就是 1 秒 1 次） */

typedef struct { uint32_t len; uint32_t kind; } ring_hdr_t;
#define RING_KIND_H264  0u
#define RING_KIND_CSV   1u

/* ============================ 状态 ============================ */

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cond = PTHREAD_COND_INITIALIZER;

static volatile int g_run;            /* 送帧线程 + 取流线程存活 */
static volatile int g_want;           /* 是否落盘（常录开关） */
static volatile int g_writer_run;     /* writer 线程存活 */

static pthread_t    g_send_tid, g_drain_tid, g_writer_tid;
static int          g_send_started, g_drain_started, g_writer_started;

static char         g_dir[256];
static uint64_t     g_cap_bytes = REC_DEFAULT_CAP_BYTES;

static int          g_pools_ok;
static k_u32        g_out_pool = VB_INVALID_POOLID;
static k_u32        g_out_blk_size;
static int          g_venc_ready;
static int          g_bound;
static int          g_shared;         /* 1=与识别共享通道（默认） */

/* 会话（只在 writer 线程里打开/关闭） */
static FILE        *g_fh264;
static FILE        *g_fcsv;
static uint32_t     g_session_idx;
static volatile int g_session_open;
static uint64_t     g_session_bytes;
static uint64_t     g_session_frames;
static uint64_t     g_csv_index;
static uint64_t     g_session_start_us;

/* 识别 -> 录像 的交接槽（帧所有权在此转移） */
static k_video_frame_info g_offer;
static uint64_t     g_offer_seq;
static int          g_offer_have;
static uint32_t     g_samp_acc;       /* 抽样累加器（30->25 即每 6 帧丢 1） */

/* 在途 FIFO：已送编码器、等待从码流取回后归还的帧 */
static k_video_frame_info g_inflight[INFLIGHT_MAX];
static int          g_inflight_cnt;

/* 码流 RAM 环 */
static uint8_t     *g_ring;
static uint8_t     *g_wscratch;
static uint32_t     g_ring_w, g_ring_r, g_ring_used;
static pthread_mutex_t g_ring_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_ring_cond = PTHREAD_COND_INITIALIZER;

static k_venc_pack  g_packs[MAX_PACKS];
static uint64_t     g_last_flush_ms;

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

/* 消费者（只在 writer 线程调用）：取一条记录到 scratch，返回长度 */
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
    g_session_start_us = mono_us();
    g_session_open = 1;
    pthread_mutex_lock(&g_lock);
    g_stats.sessions++;
    pthread_mutex_unlock(&g_lock);

    app_log(APP_NAME_STR ": rec session rec_%04u start (dir=%s cap=%llu)\n",
            g_session_idx, g_dir, (unsigned long long)g_cap_bytes);
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
                g_stats.written++;
            }
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
        uint32_t len = ring_take(&kind);
        if (len == 0) {
            if (!g_writer_run && g_ring_used == 0)
                break;
            continue;
        }
        if (kind == RING_KIND_H264) {
            if (g_fh264 && fwrite(g_wscratch, 1, len, g_fh264) == len) {
                g_session_bytes += len;
                g_stats.written++;
            }
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

/* ============================ VENC ============================ */

static void rec_release_hw(void)
{
    mpp_enter();
    if (g_bound) {
        k_mpp_chn vi, venc;
        memset(&vi, 0, sizeof(vi));
        memset(&venc, 0, sizeof(venc));
        vi.mod_id = K_ID_VI;              /* K230 无 VDSS：VICAP 通道按 K_ID_VI 寻址 */
        vi.dev_id = VICAP_DEV_ID_0;
        vi.chn_id = VICAP_CHN_RECORD;
        venc.mod_id = K_ID_VENC;
        venc.dev_id = 0;
        venc.chn_id = VENC_CHN;
        (void)kd_mpi_sys_unbind(&vi, &venc);
        g_bound = 0;
    }
    if (g_venc_ready) {
        kd_mpi_venc_stop_chn(VENC_CHN);
        kd_mpi_venc_destroy_chn(VENC_CHN);
        kd_mpi_venc_detach_vb_pool(VENC_CHN);
        g_venc_ready = 0;
    }
    if (g_pools_ok) {
        kd_mpi_vb_destory_pool(g_out_pool);
        g_out_pool = VB_INVALID_POOLID;
        g_pools_ok = 0;
    }
    mpp_leave();
}

int record_bind(void)
{
    /* 必须先确定模式：record_bind 在 record_run 之前被调用 */
    g_shared = (g_cfg.rec_mode == 0);

    /* 输出码流池：输入帧不经过应用（shared 由识别交棒 / bind 由硬件直连） */
    uint32_t ow = 0, oh = 0;
    if (g_shared) {
        ow = vicap_chn_width(VICAP_CHN_VISION);
        oh = vicap_chn_height(VICAP_CHN_VISION);
    } else {
        ow = vicap_chn_width(VICAP_CHN_RECORD);
        oh = vicap_chn_height(VICAP_CHN_RECORD);
    }
    if (!ow || !oh) {
        app_log(APP_NAME_STR ": rec 编码尺寸无效（通道未配置？）\n");
        return -1;
    }
    g_out_blk_size = VB_ALIGN_UP((uint64_t)ow * oh, 4096);

    mpp_enter();
    g_out_pool = kd_mpi_vb_create_pool_ex(g_out_blk_size, OUT_BUF_N, VB_REMAP_MODE_NOCACHE);
    mpp_leave();
    if (g_out_pool == VB_INVALID_POOLID) {
        app_log(APP_NAME_STR ": rec 输出 vb 池创建失败 (%u 字节 x %d)\n",
                g_out_blk_size, OUT_BUF_N);
        return -1;
    }
    g_pools_ok = 1;

    /*
     * 源帧率填"实际喂进编码器的帧率"：shared 模式应用已按 rec_fps 抽样，
     * bind 模式由硬件直连（源=采集帧率，目标=rec_fps，但本板实测编码器不按
     * dst_frame_rate 丢帧，所以 bind 模式录到的就是采集帧率）。
     */
    uint32_t src_fps = (uint32_t)g_cfg.rec_fps;
    uint32_t dst_fps = (uint32_t)g_cfg.rec_fps;
    if (!g_shared)
        src_fps = vicap_acq_fps() ? vicap_acq_fps() : dst_fps;

    mpp_enter();
    if (kd_mpi_venc_attach_vb_pool(VENC_CHN, g_out_pool) != K_SUCCESS) {
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc attach vb pool 失败\n");
        rec_release_hw();
        return -1;
    }
    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.type = K_PT_H264;
    attr.venc_attr.pic_width = ow;
    attr.venc_attr.pic_height = oh;
    attr.venc_attr.profile = VENC_PROFILE_H264_HIGH;
    attr.rc_attr.rc_mode = K_VENC_RC_MODE_CBR;
    attr.rc_attr.cbr.src_frame_rate = src_fps;
    attr.rc_attr.cbr.dst_frame_rate = dst_fps;
    attr.rc_attr.cbr.gop = dst_fps;
    attr.rc_attr.cbr.bit_rate = g_cfg.bitrate_kbps;
    if (kd_mpi_venc_create_chn(VENC_CHN, &attr) != K_SUCCESS) {
        kd_mpi_venc_detach_vb_pool(VENC_CHN);
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc create chn 失败\n");
        rec_release_hw();
        return -1;
    }
    g_venc_ready = 1;
    if (kd_mpi_venc_start_chn(VENC_CHN) != K_SUCCESS) {
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc start chn 失败\n");
        rec_release_hw();
        return -1;
    }

    if (!g_shared) {
        k_mpp_chn vi, venc;
        memset(&vi, 0, sizeof(vi));
        memset(&venc, 0, sizeof(venc));
        vi.mod_id = K_ID_VI;
        vi.dev_id = VICAP_DEV_ID_0;
        vi.chn_id = VICAP_CHN_RECORD;
        venc.mod_id = K_ID_VENC;
        venc.dev_id = 0;
        venc.chn_id = VENC_CHN;
        k_s32 ret = kd_mpi_sys_bind(&vi, &venc);
        mpp_leave();
        if (ret != K_SUCCESS) {
            app_log(APP_NAME_STR ": rec bind VICAP CHN%d -> VENC%d 失败 ret=0x%08x\n",
                    (int)VICAP_CHN_RECORD, VENC_CHN, (unsigned)ret);
            rec_release_hw();
            return -1;
        }
        g_bound = 1;
        app_log(APP_NAME_STR ": rec 模式=bind（双通道硬件直连）VICAP CHN%d -> VENC%d "
                "%ux%u src=%u dst=%u\n", (int)VICAP_CHN_RECORD, VENC_CHN, ow, oh,
                src_fps, dst_fps);
    } else {
        mpp_leave();
        app_log(APP_NAME_STR ": rec 模式=shared（与识别共享 VICAP CHN%d，应用交棒）"
                "%ux%u @%u fps bitrate=%u kbps\n",
                (int)VICAP_CHN_VISION, ow, oh, dst_fps, g_cfg.bitrate_kbps);
    }
    return 0;
}

/* ============================ 送帧（shared 模式） ============================ */

static int rec_send_one(const k_video_frame_info *frame);

/*
 * 识别线程处理完一帧后调用（必须在 release 之前）。
 * 返回 0 = 已接管（后端负责归还），1 = 不需要/忙（调用方自己归还）。
 */
int record_offer_frame(const k_video_frame_info *frame, uint64_t seq)
{
    if (!g_shared || !g_want || !g_run)
        return 1;

    int rc = 1;
    pthread_mutex_lock(&g_lock);
    if (!g_offer_have) {
        g_offer = *frame;
        g_offer_seq = seq;
        g_offer_have = 1;
        pthread_cond_broadcast(&g_cond);
        rc = 0;
    } else {
        g_stats.offer_busy++;
    }
    pthread_mutex_unlock(&g_lock);
    return rc;
}

static void *rec_send_thread(void *arg)
{
    (void)arg;
    uint32_t src_fps = vicap_acq_fps() ? vicap_acq_fps() : 30u;
    uint32_t dst_fps = (uint32_t)(g_cfg.rec_fps > 0 ? g_cfg.rec_fps : 25);

    while (g_run) {
        k_video_frame_info f;

        pthread_mutex_lock(&g_lock);
        while (g_run && !g_offer_have)
            pthread_cond_wait(&g_cond, &g_lock);
        if (!g_run && !g_offer_have) {
            pthread_mutex_unlock(&g_lock);
            break;
        }
        f = g_offer;
        g_offer_have = 0;
        pthread_mutex_unlock(&g_lock);

        /* 抽样：30->25 即每 6 帧丢 1 帧（确定性，保证送帧与取包 1:1） */
        g_samp_acc += dst_fps;
        int keep = 0;
        if (g_samp_acc >= src_fps) {
            g_samp_acc -= src_fps;
            keep = 1;
        }
        if (!g_want || !keep) {
            vicap_release(VICAP_CHN_VISION, &f);
            if (keep)
                g_stats.sent_skipped++;
            else
                g_stats.sampled_out++;
            continue;
        }

        if (rec_send_one(&f) != 0) {
            /* 编码器忙/出错：自己归还，绝不堆积（这是"不反压"的关键） */
            vicap_release(VICAP_CHN_VISION, &f);
            g_stats.send_fail++;
        }
    }
    return NULL;
}

/*
 * 送一帧给编码器，成功后放进"在途 FIFO"（等待取流线程归还）。
 * 返回 0 = 成功（编码器在读该帧），-1 = 失败（调用方负责归还）。
 */
static int rec_send_one(const k_video_frame_info *frame)
{
    mpp_enter();
    k_s32 ret = kd_mpi_venc_send_frame(VENC_CHN, (k_video_frame_info *)frame, 20);
    mpp_leave();
    if (ret != K_SUCCESS)
        return -1;

    pthread_mutex_lock(&g_lock);
    if (g_inflight_cnt >= INFLIGHT_MAX) {
        /* 理论上 1:1 不会发生；真发生了说明编码器侧异常，丢最旧的避免泄漏 */
        vicap_release(VICAP_CHN_VISION, &g_inflight[0]);
        for (int i = 1; i < g_inflight_cnt; ++i)
            g_inflight[i - 1] = g_inflight[i];
        g_inflight_cnt--;
        g_stats.fifo_overflow++;
    }
    g_inflight[g_inflight_cnt++] = *frame;
    g_stats.sent++;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
    return 0;
}

/* ============================ 取流 ============================ */

/* 取一次码流：把包拷进 RAM 环，并归还对应的在途帧。返回 1=取到，0=暂无 */
static int rec_drain_once(void)
{
    k_venc_chn_status status;
    k_venc_stream st;
    uint32_t nvideo = 0;

    memset(&status, 0, sizeof(status));
    mpp_enter();
    if (kd_mpi_venc_query_status(VENC_CHN, &status) != K_SUCCESS || status.cur_packs == 0) {
        mpp_leave();
        return 0;
    }
    uint32_t cnt = status.cur_packs > MAX_PACKS ? MAX_PACKS : status.cur_packs;
    memset(&st, 0, sizeof(st));
    st.pack_cnt = cnt;
    st.pack = g_packs;
    k_s32 ret = kd_mpi_venc_get_stream(VENC_CHN, &st, 3);
    if (ret != K_SUCCESS) {
        mpp_leave();
        return 0;
    }

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
    mpp_leave();

    /* 已取到码流：立刻归还在途帧（编码器已产出对应输出）—— 唯一归还者 */
    if (g_shared && nvideo) {
        pthread_mutex_lock(&g_lock);
        if (nvideo > (uint32_t)g_inflight_cnt)
            g_stats.fifo_underflow++;      /* 输出多于输入：说明编码器行为与预期不符 */
        k_video_frame_info rel[INFLIGHT_MAX];
        uint32_t nrel = 0;
        while (nrel < nvideo && g_inflight_cnt > 0) {
            rel[nrel++] = g_inflight[0];
            for (int i = 1; i < g_inflight_cnt; ++i)
                g_inflight[i - 1] = g_inflight[i];
            g_inflight_cnt--;
        }
        pthread_mutex_unlock(&g_lock);
        for (uint32_t i = 0; i < nrel; ++i)
            vicap_release(VICAP_CHN_VISION, &rel[i]);
    }

    /* 码流进 RAM 环（不碰 SD） */
    int writing = g_session_open;
    for (k_u32 i = 0; i < pack_cnt; i++) {
        if (!vas[i] || !lens[i])
            continue;
        if (writing) {
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

    mpp_enter();
    kd_mpi_venc_release_stream(VENC_CHN, &st);
    mpp_leave();

    g_stats.streams += nvideo;
    return 1;
}

static void *rec_drain_thread(void *arg)
{
    (void)arg;
    /* 本线程只做「取流 + 推环 + 归还在途帧」，绝不做文件系统操作 */
    while (g_run) {
        if (!rec_drain_once())
            usleep(2000);
    }
    return NULL;
}

/* ============================ 对外接口 ============================ */

int record_run(void)
{
    snprintf(g_dir, sizeof(g_dir), "%s", g_cfg.rec_dir ? g_cfg.rec_dir : REC_DEFAULT_DIR);
    g_cap_bytes = g_cfg.cap_bytes ? g_cfg.cap_bytes : REC_DEFAULT_CAP_BYTES;
    g_shared = (g_cfg.rec_mode == 0);
    g_want = g_cfg.record_on ? 1 : 0;
    g_stats.enabled = g_want;
    g_stats.shared = g_shared;
    g_stats.bound = g_bound;
    g_stats.dir = g_dir;
    g_stats.cap_bytes = g_cap_bytes;

    ring_init();
    g_writer_run = 1;
    if (pthread_create(&g_writer_tid, NULL, rec_writer_thread, NULL) == 0)
        g_writer_started = 1;

    g_run = 1;
    if (pthread_create(&g_drain_tid, NULL, rec_drain_thread, NULL) == 0)
        g_drain_started = 1;
    if (g_shared && pthread_create(&g_send_tid, NULL, rec_send_thread, NULL) == 0)
        g_send_started = 1;

    if (!g_drain_started)
        app_log(APP_NAME_STR ": rec 取流线程创建失败\n");
    app_log(APP_NAME_STR ": rec 线程就绪（取流=%d 送帧=%d 写盘=%d）\n",
            g_drain_started, g_send_started, g_writer_started);
    return g_drain_started ? 0 : -1;
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

    if (g_send_started) {
        pthread_join(g_send_tid, NULL);
        g_send_started = 0;
    }
    if (g_drain_started) {
        /* 先让取流线程退出：rec_drain_once 用静态 g_packs，绝不能两个线程同时跑 */
        pthread_join(g_drain_tid, NULL);
        g_drain_started = 0;
    }
    /* 只剩本线程：把编码器里剩的码流取完，并归还所有在途帧 */
    for (int i = 0; i < 200; ++i) {
        if (!rec_drain_once())
            break;
        usleep(2000);
    }

    pthread_mutex_lock(&g_lock);
    k_video_frame_info rel[INFLIGHT_MAX];
    int nrel = g_inflight_cnt;
    for (int i = 0; i < nrel; ++i)
        rel[i] = g_inflight[i];
    g_inflight_cnt = 0;
    if (g_offer_have) {
        rel[nrel < INFLIGHT_MAX ? nrel : INFLIGHT_MAX - 1] = g_offer;
        if (nrel < INFLIGHT_MAX)
            nrel++;
        g_offer_have = 0;
    }
    pthread_mutex_unlock(&g_lock);
    for (int i = 0; i < nrel; ++i) {
        if (g_shared)
            vicap_release(VICAP_CHN_VISION, &rel[i]);
    }

    if (g_writer_started) {
        g_writer_run = 0;
        pthread_mutex_lock(&g_ring_lock);
        pthread_cond_broadcast(&g_ring_cond);
        pthread_mutex_unlock(&g_ring_lock);
        pthread_join(g_writer_tid, NULL);
        g_writer_started = 0;
    }
    rec_session_close();
    rec_release_hw();
    free(g_ring);
    free(g_wscratch);
    g_ring = NULL;
    g_wscratch = NULL;
}

void record_set_enabled(int on)
{
    g_want = on ? 1 : 0;
    g_stats.enabled = g_want;
    app_log(APP_NAME_STR ": rec %s\n",
            on ? "落盘 enabled" : "落盘 disabled（当前会话收尾，编码流水继续空转）");
}

int record_enabled(void)
{
    return g_want ? 1 : 0;
}

int record_is_active(void)
{
    return g_session_open;
}

int record_is_started(void)
{
    return g_drain_started;
}

void record_set_cap(uint64_t bytes)
{
    g_cap_bytes = bytes ? bytes : REC_DEFAULT_CAP_BYTES;
    g_stats.cap_bytes = g_cap_bytes;
    app_log(APP_NAME_STR ": rec cap set to %llu\n", (unsigned long long)g_cap_bytes);
}

int record_set_dir(const char *dir)
{
    if (!dir || !dir[0])
        return -1;
    if (g_session_open) {
        app_log(APP_NAME_STR ": rec dir change rejected (session active)\n");
        return -1;
    }
    snprintf(g_dir, sizeof(g_dir), "%s", dir);
    g_stats.dir = g_dir;
    app_log(APP_NAME_STR ": rec dir set to %s\n", g_dir);
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
    uint64_t dur = mono_us() - g_session_start_us;
    st->real_fps = (g_session_open && dur > 0)
                       ? (uint32_t)((g_session_frames * 1000000ull) / dur)
                       : 0;
    pthread_mutex_unlock(&g_lock);
}
