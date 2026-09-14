/*
 * record —— 录像子系统（默认与识别共享同一个 VICAP 通道）
 *
 * 职责：
 *   1) 把识别线程处理完的帧**交棒**给编码器（shared 模式，默认）
 *      或把独立的录像通道硬件直连给编码器（bind 模式，可选实验）
 *   2) 取 H.264 码流 -> RAM 环 -> 独立 writer 线程落盘
 *   3) 会话滚动、目录容量淘汰
 *
 * 调用顺序（bind 模式要求绑定在 start_stream 之前）：
 *     vicap_setup()             // set_dev_attr / chn_attr / vicap_init
 *     record_bind()             // 输出池 + VENC 通道（+ bind）
 *     vicap_start()             // start_stream
 *     vision_start(); record_run();
 *     ...
 *     record_deinit(); vicap_stop();
 *
 * 帧所有权（唯一归还者原则，详见 record.c 头注释）：
 *     识别 dump -> 处理 -> record_offer_frame()
 *          收下 -> 送帧线程（抽样丢/送失败则自己归还）-> 在途 FIFO -> 取流线程归还
 *          拒收 -> 识别线程自己归还
 *
 * 运行期 record on/off 只控制落盘（编码器保持空转，避免 VICAP 通道无人消费）。
 * 想彻底不占 CHN1/VENC 资源，用启动参数 --record off。
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

typedef struct {
    uint64_t sessions;          /* 已开始的会话数 */
    uint64_t sent;              /* 成功送进编码器的帧数 */
    uint64_t sampled_out;       /* 按 rec_fps 抽样主动丢掉的帧数 */
    uint64_t sent_skipped;      /* record off 期间丢掉的帧数 */
    uint64_t send_fail;         /* send_frame 失败（已自行归还） */
    uint64_t offer_busy;        /* 交接槽已占用（识别自己归还） */
    uint64_t fifo_overflow;     /* 在途 FIFO 溢出（正常应恒为 0） */
    uint64_t fifo_underflow;    /* 输出包多于在途帧（正常应恒为 0） */
    uint64_t streams;           /* 从编码器取回的码流帧数 */
    uint64_t written;           /* 已落盘帧数 */
    uint64_t ring_dropped_frames;/* 因 RAM 环满丢弃的帧数（只影响录像） */
    uint64_t ring_dropped;      /* 因 RAM 环满丢弃的字节数 */
    uint64_t bytes;             /* 当前会话已写字节 */
    uint32_t real_fps;          /* 当前会话实测落盘帧率 */
    uint32_t session_index;     /* 当前会话编号（0=未开） */
    int      active;            /* 当前是否在落盘 */
    int      enabled;           /* 常录开关 */
    int      bound;             /* bind 模式是否绑定成功 */
    int      shared;            /* 1=与识别共享通道 */
    const char *dir;
    uint64_t cap_bytes;
} record_stats_t;

/* 建输出池与 VENC 通道（bind 模式还会做 VICAP->VENC 绑定）；在 vicap_start 之前调用 */
int  record_bind(void);
/* 起 送帧/取流/写盘 线程；在 vicap_start 之后调用 */
int  record_run(void);
void record_deinit(void);

/* 识别线程把一帧交棒给录像（在 release 之前调用）：
 *   0 = 已接管（后端负责归还，调用方不要再动这帧）
 *   1 = 不需要/忙（调用方自己归还） */
int  record_offer_frame(const k_video_frame_info *frame, uint64_t seq);

void record_get_stats(record_stats_t *st);

void record_set_enabled(int on);   /* 常录/落盘开关 */
int  record_enabled(void);
int  record_is_active(void);
int  record_is_started(void);

void record_set_cap(uint64_t bytes);
int  record_set_dir(const char *dir);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_RECORD_H */
