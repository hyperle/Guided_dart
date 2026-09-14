/*
 * vicap_src —— VICAP/ISP/Sensor 初始化与帧 dump/release 封装
 *
 * 通道规划（这是本工程修复“帧双重所有权”恶性 bug 的核心）：
 *   CHN0 (VICAP_CHN_ID_0) -> 视觉子系统（只有视觉线程 dump/release）
 *   CHN1 (VICAP_CHN_ID_1) -> 录像子系统（只有录像线程 dump，成功后交给 VENC）
 * 两条通道各自持有自己的 VB 环形缓冲，互不共享帧，因此每个帧只有一个
 * 所有者、只有一条释放路径。
 *
 * 所有 dump/release 都在 mpp 临界区内执行（见 app.h 说明 2）。
 */
#ifndef GREEN_LED_AI_VICAP_SRC_H
#define GREEN_LED_AI_VICAP_SRC_H

#include <stdint.h>

#include "mpi_vicap_api.h"   /* k_video_frame_info / k_vicap_chn */

#ifdef __cplusplus
extern "C" {
#endif

#define VICAP_CHN_VISION   VICAP_CHN_ID_0
#define VICAP_CHN_RECORD   VICAP_CHN_ID_1

/*
 * 采集侧的三个步骤（拆开是为了让录像子系统能在 start_stream 之前完成
 * VICAP->VENC 绑定，官方 sample_venc 就是这个顺序）：
 *   vicap_setup()  —— VB 初始化、探测 sensor、配置 dev/两条通道、vicap_init
 *   vicap_start()  —— start_stream + 应用固定曝光/增益
 *   vicap_stop()   —— stop_stream + deinit + VB exit
 * 返回 0 成功；失败时日志里已有明确原因。
 */
int  vicap_setup(void);
int  vicap_start(void);
void vicap_stop(void);

/* dump / release 封装（内部自动进 MPP 临界区，调用方不要持锁） */
int  vicap_dump(int chn, k_video_frame_info *frame, int timeout_ms);
int  vicap_release(int chn, const k_video_frame_info *frame);

/* 归还失败的累计次数（>0 说明 VB 块在泄漏，攒够通道缓冲数就会停流） */
uint64_t vicap_release_fail_count(void);

/*
 * 「软」dump：把「暂时还没有帧」和「真错误」分开，返回值：
 *   0 = 拿到帧；1 = 暂时没帧（NOTREADY/BUF_EMPTY，别当错误、别刷日志）；
 *  -1 = 真错误（带上 *ret 原始错误码）。
 *
 * 为什么必须区分：dump 是在 MPP 临界区内阻塞等待的。视觉比采集快是稳态
 * （传感器 30fps，处理只要几百 us），如果每次都用长超时硬等，就会长时间
 * 持锁把录像侧（dump/编码取流）一起拖住。所以这里用很短的超时快速失败，
 * 由调用方在**锁外**睡一小会再试。
 */
int  vicap_dump_soft(int chn, k_video_frame_info *frame, int timeout_ms, int *ret);

/* 采集侧实际命中的 sensor 模式（日志用） */
uint32_t vicap_acq_width(void);
uint32_t vicap_acq_height(void);
uint32_t vicap_acq_fps(void);
uint32_t vicap_chn_width(int chn);
uint32_t vicap_chn_height(int chn);
int      vicap_is_open(void);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_VICAP_SRC_H */
