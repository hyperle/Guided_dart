/*
 * record —— 录像子系统（v4：单通道 CHN0 + 应用层拷贝喂编码器）
 *
 * 数据流：
 *   识别线程 dump CHN0 -> 检测 -> record_take_frame()
 *        copy  模式：拷进录像私有 VB 块（拷完即回调 done()，识别侧马上归还 VICAP 帧）
 *        borrow 模式：直接送 VICAP 帧，send_frame 返回后回调 done()（官方 uvc 样例写法）
 *   VENC 线程：free 块 -> send_frame(-1 阻塞) -> get_stream -> RAM 环 -> release_stream -> 块回 free
 *   writer 线程：RAM 环 -> SD（会话开关/滚动/容量淘汰/1 秒 1 次 fsync）
 *
 * 三条流水互不阻塞：识别只碰 VICAP（采集线程独用）和自己私有的拷贝缓冲；
 * VENC 有专用锁（只有 VENC 线程用）；写盘完全不碰 MPP。全程没有 sleep/poll，
 * 等待一律是"等事件"（条件变量 / 编码器）。
 *
 * 调用顺序：
 *     vicap_setup()      // 只配 CHN0
 *     record_setup()     // 输出池 + VENC 通道（拷贝块池在第一帧时懒建）
 *     vicap_start()
 *     vision_start(); record_run();
 *     ...
 *     record_deinit(); vicap_stop();
 *
 * 运行期 record on/off 只控制"要不要拷贝/落盘"：编码器保持空转，识别侧一秒都不等。
 */
#ifndef GREEN_LED_AI_RECORD_H
#define GREEN_LED_AI_RECORD_H

#include <stdint.h>

#include "mpi_vicap_api.h"   /* k_video_frame_info */

#ifdef __cplusplus
extern "C" {
#endif

#define REC_DEFAULT_DIR       "/sdcard/app/recording"
#define REC_DEFAULT_CAP_BYTES (8ull << 30)   /* recording/ 目录总容量上限 8GB */
#define REC_ROLL_BYTES        (256ull << 20) /* 每段会话字节上限，到点自动滚动 */

#define REC_FEED_COPY   0    /* 应用层拷贝到私有 VB 块（默认，最稳） */
#define REC_FEED_BORROW 1    /* 直接把 VICAP 帧借给编码器（官方样例写法，零拷贝对照） */

typedef struct {
    uint64_t sessions;          /* 已开始的会话数 */
    uint64_t copied;            /* copy 模式：已拷贝并交给编码器的帧数 */
    uint64_t borrowed;          /* borrow 模式：直接借给编码器的帧数 */
    uint64_t sampled_out;       /* 按 rec_fps 抽样主动丢掉的帧数 */
    uint64_t blk_empty;         /* 没有空闲拷贝块而丢掉的帧数（录像尽力而为） */
    uint64_t queue_full;        /* 待送队列满而丢掉的帧数 */
    uint64_t send_fail;         /* send_frame 失败 */
    uint64_t sent;              /* 成功送进编码器的帧数（copy+borrow） */
    uint64_t fifo_overflow;     /* 在途帧溢出（正常应恒为 0） */
    uint64_t fifo_underflow;    /* 输出包多于在途帧（正常应恒为 0） */
    uint64_t streams;           /* 从编码器取回的码流帧数 */
    uint64_t written;           /* 已落盘帧数 */
    uint64_t ring_dropped_frames;/* 因 RAM 环满丢弃的帧数（只影响录像） */
    uint64_t ring_dropped;      /* 因 RAM 环满丢弃的字节数 */
    uint64_t bytes;             /* 当前会话已写字节 */
    uint64_t copy_sum_us;       /* 拷贝耗时累计（算平均值用） */
    uint32_t hdr_bytes;         /* 已缓存的 SPS/PPS 字节数（0 = 文件会缺参数集，必须 >0） */
    uint32_t copy_max_us;       /* 单帧拷贝最长耗时 */
    uint32_t real_fps;          /* 当前会话实测落盘帧率 */
    uint32_t session_index;     /* 当前会话编号（0=未开） */
    int      free_blocks;       /* 空闲拷贝块 */
    int      queue_blocks;      /* 已拷好、待送编码器 */
    int      inflight_blocks;   /* 已送编码器、等取流 */
    int      active;            /* 当前是否在落盘 */
    int      enabled;           /* 常录开关 */
    int      feed;              /* 1=borrow, 0=copy */
    const char *dir;
    uint64_t cap_bytes;
} record_stats_t;

/*
 * 识别侧交帧用的视图：帧本体 + 已经映射（cached+invalidated）好的平面指针。
 * copy 模式只用平面指针；borrow 模式只用帧本体。
 */
typedef struct {
    const k_video_frame_info *f;
    const uint8_t            *p[3];
    int                       np;      /* 平面数：NV12=2，RGB_888_PLANAR=3 */
    uint32_t                  stride, width, height;
} rec_frame_t;

/* 帧用完后由录像回调（copy 模式在拷贝完成后立刻回调；borrow 模式在送帧返回后回调） */
typedef void (*rec_done_fn)(const k_video_frame_info *f, void *ctx);

/* 建输出池与 VENC 通道；在 vicap_start 之前调用（拷贝块池在第一帧时懒建） */
int  record_setup(void);
/* 起 VENC/写盘线程；在 vicap_start 之后调用 */
int  record_run(void);
void record_deinit(void);

/*
 * 识别线程把处理完的一帧交给录像（在归还 VICAP 帧之前调用）。**绝不阻塞**：
 *   0 = 录用（copy 已拷完 / borrow 已排队；done() 由录像在合适时机回调）
 *   1 = 不录（没开录像 / 抽样丢掉 / 没空闲块）——调用方照常归还自己的帧
 */
int  record_take_frame(const rec_frame_t *rf, uint64_t seq, rec_done_fn done, void *ctx);

void record_get_stats(record_stats_t *st);

void record_set_enabled(int on);   /* 常录/落盘开关 */
int  record_enabled(void);
int  record_is_active(void);
int  record_is_started(void);

/* 识别侧上报实测处理帧率（抽样器用它当分母；vicap 的请求值只是 30） */
void record_set_src_fps(uint32_t fps);

void record_set_cap(uint64_t bytes);
int  record_set_dir(const char *dir);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_RECORD_H */
