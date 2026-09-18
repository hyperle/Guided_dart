/*
 * vision —— 视觉子系统（CHN0 独占）
 *
 * 线程模型（两级解耦，视觉链路永不等待录像链路）：
 *
 *   vis_cap 线程: dump CHN0 -> 投递到「单槽信箱」（新帧优先，旧帧立即归还）
 *        |  信箱满时不阻塞采集，直接把旧帧 release 掉并计数
 *        v
 *   vis_proc 线程: 取帧（成为该帧唯一所有者）-> 持久映射取 VA -> detect
 *                  -> 归还帧一次（kd_mpi_vicap_dump_release）
 *
 * 帧所有权：dump 出来的帧由 vis_cap 交给 vis_proc，只有 vis_proc 会 release；
 * 视觉通道的帧从不送编码器，录像通道(CHN1)的帧从不进检测器 —— 这就是旧
 * 工程「同一帧既送 VENC 又 release」导致内核/VICAP 卡死的根因修法。
 *
 * 运行期 vision on/off 只控制「是否 dump / 是否继续处理」，不重建通道，
 * 因此启停视觉不影响录像（反之亦然）。
 */
#ifndef GREEN_LED_AI_VISION_H
#define GREEN_LED_AI_VISION_H

#include <stddef.h>
#include <stdint.h>

#include "detect.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t frames;          /* 处理完成的视觉帧数 */
    uint64_t dumped;          /* CHN0 dump 成功总帧数（判断采集是否在出帧） */
    uint64_t noframe;         /* dump 返回 BUF_EMPTY（此刻没帧，识别比采集快时常见） */
    uint64_t notready;        /* dump 返回 NOTREADY（流水被反压卡住，>0 要查录像侧） */
    uint64_t misses;          /* 未命中的帧数（cx<0） */
    uint64_t dropped;         /* 信箱满被丢弃的帧数（新帧优先） */
    uint64_t dump_fail;       /* dump 真错误次数 */
    uint64_t dump_timeouts;   /* dump 等不到帧的次数（稳态不增长） */
    uint64_t recycled;        /* 已交回购还环、由采集线程 release 的帧数（≈dumped） */
    uint64_t recycle_leak;    /* 归还环满被丢弃的帧（会漏 VB 块，必须恒为 0） */
    uint32_t dump_call_us;    /* 最近一次 dump 调用耗时（≈帧间隔=在等下一帧） */
    uint64_t found, lost;     /* 命中/丢失累计 */
    uint64_t found_total, lost_total;
    uint32_t fps;             /* 统计周期内的处理帧率 */
    uint32_t t_detect_avg_us; /* 纯检测耗时（周均值/周峰值） */
    uint32_t t_detect_max_us;
    uint32_t t_lat_avg_us;    /* 端到端延迟：dump 成功 -> 结果产生（周均值/周峰值） */
    uint32_t t_lat_max_us;
    uint32_t t_lat_max_all_us;/* 端到端延迟：进程启动以来的最大值 */
    uint64_t map_calls;       /* 本周期内 mmap 次数（稳态应为 0） */
} vision_stats_t;

/* 创建线程（不 dump，直到 vision_set_enabled(1)） */
int  vision_start(void);
void vision_stop(void);

void vision_set_enabled(int on);
int  vision_enabled(void);
int  vision_is_started(void);    /* 线程是否已创建（--vision off 时为 0） */

/* 帧用完：推进归还环（由采集线程统一 release；任何线程都可调，不阻塞、不碰 MPP） */
void vision_recycle_frame(const k_video_frame_info *f);

void vision_get_stats(vision_stats_t *st);

/* 后端自己的耗时分解（kpu 后端有：ai2d/kpu/post/latency/fps），无则空串 */
void vision_backend_stats(char *buf, size_t n);

/* 最近一帧的检测明细（探针像素/亮度统计），供状态行打印、照日志调阈值 */
int  vision_get_last(detect_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_VISION_H */
