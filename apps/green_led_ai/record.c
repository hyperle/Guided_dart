/*
 * record —— 录像子系统（VICAP CHN1 硬件绑定到 VENC，与视觉完全解耦）
 *
 * ★ 为什么用 kd_mpi_sys_bind 而不是 kd_mpi_venc_send_frame
 *
 * 板端实测（logs/green_led_ai.log，2025-09）：用 send_frame 手动喂编码器时，
 * 十几帧之后**两条通道全部停止出帧**（vis fps=0、rec in=13 不再增长）。原因：
 * VICAP 帧送进 VENC 后没有任何人归还它（MPP 里没有 venc release_frame），
 * CHN1 的 6 个 VB 缓冲被编码器攥死 -> 该通道饿死 -> 挂在同一个 VICAP/ISP
 * 流水上的 CHN0 也跟着停。旧工程 green_led_rtos 之所以还能持续出帧，是因为
 * 识别线程"顺手"把同一帧 release 了 —— 那正是它 VB 引用计数崩掉、读到全黑帧
 * 的原因（同一个 bug 的两面）。
 *
 * 官方 sample_venc 里**根本没有 send_frame**：它把 VICAP 通道 kd_mpi_sys_bind
 * 到 VENC 通道，帧由硬件直接搬运、编码器内部自己回收，应用只负责取码流落盘。
 * 官方顺序（照抄，绑定必须在 start_stream 之前）：
 *     venc 输出池 -> attach_vb_pool -> create_chn -> start_chn
 *       -> vicap set_dev_attr/chn_attr/init -> **bind** -> start_stream
 * 所以本模块拆成两个入口：
 *     record_bind() —— 在 vicap_start() 之前调用（建 VENC + 绑定）
 *     record_run()  —— 在 start_stream 之后调用（起取流/落盘线程）
 *
 * ★ 运行期 record on/off 的语义
 *   编码器始终在跑（否则 VICAP 通道没人消费又可能把 ISP 卡死），on/off 只控制
 *   "要不要把码流落盘"：
 *     record on  -> 开会话，写 rec_XXXX.h264/.csv
 *     record off -> 当前会话收尾，之后码流丢弃（计入 discarded）
 *   想彻底不占 CHN1/VENC 的资源，用启动参数 --record off。
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

#define VENC_CHN      0
#define OUT_BUF_N     8          /* VENC 输出缓冲数 */
#define MAX_PACKS     32         /* 单次取流最大包数（预分配，锁内不 malloc） */

/* ============================ 状态 ============================ */

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

static volatile int g_run;
static volatile int g_want;         /* 是否落盘（常录开关） */
static pthread_t    g_drain_tid;
static int          g_drain_started;

static char         g_dir[256];
static uint64_t     g_cap_bytes = REC_DEFAULT_CAP_BYTES;

static int          g_pools_ok;
static k_u32        g_out_pool = VB_INVALID_POOLID;
static int          g_venc_ready;
static int          g_bound;

static FILE        *g_fh264;
static FILE        *g_fcsv;
static uint32_t     g_session_idx;
static volatile int g_session_open;
static uint64_t     g_session_bytes;
static uint64_t     g_session_frames;
static uint64_t     g_csv_index;
static uint64_t     g_session_start_us;

static k_venc_pack  g_packs[MAX_PACKS];
static k_u32        g_out_blk_size;    /* VENC 输出池的块大小（码流包映射按块归一化） */
static uint64_t     g_last_csv_flush_ms;
static record_stats_t g_stats;

static void rec_mkdir(void)
{
    (void)mkdir(g_dir, 0777);
}

/* ============================ VENC + 绑定 ============================ */

static int rec_ensure_pool(void)
{
    if (g_pools_ok)
        return 0;

    uint32_t w = vicap_chn_width(VICAP_CHN_RECORD);
    uint32_t h = vicap_chn_height(VICAP_CHN_RECORD);
    if (!w || !h) {
        app_log(APP_NAME_STR ": rec 通道尺寸无效（CHN%d 未配置？）\n",
                (int)VICAP_CHN_RECORD);
        return -1;
    }
    /* 只需要编码器**输出**码流池：输入帧由 VICAP 通过绑定直接供给编码器，
     * 应用既不申请输入池、也不拷帧、更不碰输入帧所有权。 */
    k_u32 oblk = VB_ALIGN_UP((uint64_t)w * h, 4096);

    mpp_enter();
    g_out_pool = kd_mpi_vb_create_pool_ex(oblk, OUT_BUF_N, VB_REMAP_MODE_NOCACHE);
    mpp_leave();

    if (g_out_pool == VB_INVALID_POOLID) {
        app_log(APP_NAME_STR ": rec 输出 vb 池创建失败 (%u 字节 x %d)\n", oblk, OUT_BUF_N);
        return -1;
    }
    g_pools_ok = 1;
    g_out_blk_size = oblk;
    app_log(APP_NAME_STR ": rec vb pool ok (id=%u blk=%u x %d)\n",
            g_out_pool, oblk, OUT_BUF_N);
    return 0;
}

static void rec_release_all(void)
{
    mpp_enter();
    if (g_bound) {
        k_mpp_chn vi, venc;
        memset(&vi, 0, sizeof(vi));
        memset(&venc, 0, sizeof(venc));
        vi.mod_id = K_ID_VI;          /* K230 无 VDSS：VICAP 通道用 K_ID_VI 寻址 */
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

/*
 * 建立 VENC 通道并把 VICAP 录像通道绑上去，必须在 kd_mpi_vicap_start_stream
 * 之前调用（官方 sample_venc 的顺序）。
 */
int record_bind(void)
{
    if (rec_ensure_pool() != 0)
        return -1;

    uint32_t w = vicap_chn_width(VICAP_CHN_RECORD);
    uint32_t h = vicap_chn_height(VICAP_CHN_RECORD);
    uint32_t src_fps = vicap_acq_fps() ? vicap_acq_fps() : (uint32_t)g_cfg.rec_fps;
    uint32_t dst_fps = (uint32_t)(g_cfg.rec_fps > 0 ? g_cfg.rec_fps : 25);

    mpp_enter();
    if (kd_mpi_venc_attach_vb_pool(VENC_CHN, g_out_pool) != K_SUCCESS) {
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc attach vb pool 失败\n");
        rec_release_all();
        return -1;
    }

    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.type = K_PT_H264;
    attr.venc_attr.pic_width = w;
    attr.venc_attr.pic_height = h;
    attr.venc_attr.profile = VENC_PROFILE_H264_HIGH;
    attr.rc_attr.rc_mode = K_VENC_RC_MODE_CBR;
    /* 源帧率=采集实际帧率、目标帧率=想要的录像帧率：掉帧在**编码器硬件**里完成，
     * 应用侧不做抽样、也不消耗输入帧 */
    attr.rc_attr.cbr.src_frame_rate = src_fps;
    attr.rc_attr.cbr.dst_frame_rate = dst_fps;
    attr.rc_attr.cbr.gop = dst_fps;                 /* 1 秒一个 I 帧 */
    attr.rc_attr.cbr.bit_rate = g_cfg.bitrate_kbps;

    if (kd_mpi_venc_create_chn(VENC_CHN, &attr) != K_SUCCESS) {
        kd_mpi_venc_detach_vb_pool(VENC_CHN);
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc create chn 失败\n");
        rec_release_all();
        return -1;
    }
    g_venc_ready = 1;

    if (kd_mpi_venc_start_chn(VENC_CHN) != K_SUCCESS) {
        mpp_leave();
        app_log(APP_NAME_STR ": rec venc start chn 失败\n");
        rec_release_all();
        return -1;
    }

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
        rec_release_all();
        return -1;
    }
    g_bound = 1;

    app_log(APP_NAME_STR ": rec bound VICAP CHN%d(%ux%u) -> VENC%d H.264 %ux%u "
            "src_fps=%u dst_fps=%u bitrate=%u kbps (零拷贝)\n",
            (int)VICAP_CHN_RECORD, w, h, VENC_CHN, w, h, src_fps, dst_fps,
            g_cfg.bitrate_kbps);
    return 0;
}

/* ============================ 会话 ============================ */

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

    pthread_mutex_lock(&g_lock);
    g_stats.bytes = g_session_bytes;
    pthread_mutex_unlock(&g_lock);

    g_session_open = 0;
    g_session_idx = 0;
}

/* ============================ 取流 / 落盘 ============================ */

/* 取一次码流；返回 1=取到，0=暂无 */
static int rec_drain_once(void)
{
    k_venc_chn_status status;
    k_venc_stream st;
    uint32_t nread = 0;
    uint64_t added = 0;

    memset(&status, 0, sizeof(status));
    mpp_enter();
    if (kd_mpi_venc_query_status(VENC_CHN, &status) != K_SUCCESS)
        status.cur_packs = 0;

    if (status.cur_packs == 0) {
        mpp_leave();
        return 0;
    }

    uint32_t cnt = status.cur_packs;
    if (cnt > MAX_PACKS)
        cnt = MAX_PACKS;
    memset(&st, 0, sizeof(st));
    st.pack_cnt = cnt;
    st.pack = g_packs;

    k_s32 ret = kd_mpi_venc_get_stream(VENC_CHN, &st, 10);
    if (ret != K_SUCCESS) {
        mpp_leave();
        return 0;
    }

    /* 锁内只做「拿码流 + 拿可读虚拟地址」（持久映射，稳态不再 mmap） */
    const uint8_t *vas[MAX_PACKS];
    uint32_t       lens[MAX_PACKS];
    k_u32          pack_cnt = st.pack_cnt;
    if (pack_cnt > MAX_PACKS)
        pack_cnt = MAX_PACKS;
    for (k_u32 i = 0; i < pack_cnt; i++) {
        vas[i] = NULL;
        lens[i] = 0;
        if (st.pack[i].len == 0)
            continue;

        /*
         * 码流包在编码器输出池的某个块里，且**每包长度都不同**（板端实测
         * 24~55 字节，图像静止时尤其小）。如果直接按 pack.phys_addr + len
         * 去映射，同一块会被以不同长度反复请求；这里先归一化到 VB 块基址，
         * 用**池块大小**映射一次，之后所有包都命中同一映射。
         */
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
        /* 码流包由编码器 DMA 写入，CPU 只读：cached 映射 + 读前失效 */
        uint8_t *p = (uint8_t *)mpp_map_persist_cached(base, mlen);
        if (p) {
            mpp_invalidate(base, p, mlen);
            vas[i] = p + (size_t)(st.pack[i].phys_addr - base);
            lens[i] = st.pack[i].len;
        }
        if (st.pack[i].type != K_VENC_HEADER)
            nread++;
    }
    mpp_leave();

    /* ---- 锁外写盘（SD 慢不占 MPP 锁）---- */
    int writing = g_session_open;
    for (k_u32 i = 0; i < pack_cnt; i++) {
        if (!vas[i] || !lens[i])
            continue;
        if (writing && g_fh264 && fwrite(vas[i], 1, lens[i], g_fh264) == lens[i])
            added += lens[i];
    }
    if (writing && g_fcsv && nread) {
        result_t r;
        result_get(&r);
        fprintf(g_fcsv, "%llu,%llu,%llu,%d,%d,%u\n",
                (unsigned long long)(++g_csv_index), (unsigned long long)r.seq,
                (unsigned long long)mono_us(), r.center_x, r.center_y, r.fps);
    }
    /* 每秒把码流与 CSV 都刷一次盘：异常掉电也能留下已录内容与逐帧记录 */
    if (added) {
        fsync(fileno(g_fh264));
        uint64_t now_ms = mono_us() / 1000ull;
        if (now_ms - g_last_csv_flush_ms >= 1000ull) {
            g_last_csv_flush_ms = now_ms;
            if (g_fcsv) {
                fflush(g_fcsv);
                fsync(fileno(g_fcsv));
            }
        }
    }

    mpp_enter();
    kd_mpi_venc_release_stream(VENC_CHN, &st);
    mpp_leave();

    pthread_mutex_lock(&g_lock);
    g_stats.streams += nread;
    if (writing) {
        g_stats.written += nread;
        g_session_bytes += added;
        g_session_frames += nread;
        g_stats.bytes = g_session_bytes;
    } else {
        g_stats.discarded += nread;
    }
    int roll = (g_session_open && g_session_bytes >= REC_ROLL_BYTES);
    pthread_mutex_unlock(&g_lock);

    if (roll) {
        app_log(APP_NAME_STR ": rec roll at %llu bytes\n",
                (unsigned long long)g_session_bytes);
        rec_session_close();
    }
    return 1;
}

static void *rec_drain_thread(void *arg)
{
    (void)arg;

    while (g_run) {
        /* 会话开关：纯文件操作，不碰 MPP */
        if (g_want && !g_session_open) {
            if (rec_session_open() != 0)
                g_want = 0;     /* 开会话失败（目录不可写等）：撤销落盘请求 */
        } else if (!g_want && g_session_open) {
            rec_session_close();
        }

        if (!rec_drain_once())
            usleep(2000);
    }

    rec_session_close();
    return NULL;
}

/* ============================ 对外接口 ============================ */

int record_run(void)
{
    snprintf(g_dir, sizeof(g_dir), "%s", g_cfg.rec_dir ? g_cfg.rec_dir : REC_DEFAULT_DIR);
    g_cap_bytes = g_cfg.cap_bytes ? g_cfg.cap_bytes : REC_DEFAULT_CAP_BYTES;
    g_want = g_cfg.record_on ? 1 : 0;
    g_stats.enabled = g_want;
    g_stats.bound = g_bound;
    g_stats.dir = g_dir;
    g_stats.cap_bytes = g_cap_bytes;

    if (!g_bound) {
        app_log(APP_NAME_STR ": rec 未绑定（record_bind 失败），不启动取流线程\n");
        return -1;
    }

    g_run = 1;
    if (pthread_create(&g_drain_tid, NULL, rec_drain_thread, NULL) != 0) {
        app_log(APP_NAME_STR ": rec drain 线程创建失败\n");
        g_run = 0;
        return -1;
    }
    g_drain_started = 1;

    app_log(APP_NAME_STR ": rec thread started (常录 %s；编码流水始终在跑，"
            "record off 只停落盘)\n", g_want ? "on" : "off");
    return 0;
}

void record_deinit(void)
{
    if (g_drain_started) {
        g_run = 0;
        pthread_join(g_drain_tid, NULL);
        g_drain_started = 0;
    }
    rec_session_close();
    rec_release_all();
}

int record_force_unbind(void)
{
    if (!g_bound)
        return 0;
    g_run = 0;
    if (g_drain_started) {
        pthread_join(g_drain_tid, NULL);
        g_drain_started = 0;
    }
    rec_session_close();
    rec_release_all();          /* 内部先 unbind，再停/销毁 VENC，再销毁池 */
    g_want = 0;
    g_stats.enabled = 0;
    g_stats.bound = 0;
    app_log(APP_NAME_STR ": rec 已解绑 VICAP CHN%d -> VENC（降级为仅识别）\n",
            (int)VICAP_CHN_RECORD);
    return 1;
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
    return g_drain_started && g_bound;
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
