/*
 * record —— 录像子系统（VICAP CHN1 硬件绑定到 VENC）
 *
 * 职责：把 VICAP 的录像通道绑到编码器（零拷贝、编码器自己回收输入帧）、
 * 取 H.264 码流落盘、按容量滚动与淘汰旧会话。识别代码不依赖本模块任何实现。
 *
 * 调用顺序（官方 sample_venc 的顺序，绑定必须在 start_stream 之前）：
 *     vicap_setup()            // set_dev_attr / chn_attr / vicap_init
 *     record_bind()            // venc 池/通道/start + kd_mpi_sys_bind  ← 本模块
 *     vicap_start()            // kd_mpi_vicap_start_stream
 *     record_run()             // 起取流+落盘线程                      ← 本模块
 *     ...
 *     record_deinit(); vicap_close();
 *
 * 输出：每段会话在 <record_dir>/（默认 /sdcard/app/recording）生成一对文件
 *     rec_%04u.h264   —— Annex-B H.264 裸流（含 SPS/PPS，可直接 ffmpeg 封装）
 *     rec_%04u.csv    —— 逐帧识别结果（frame_seq,result_seq,时间戳,center,fps）
 * 目录总量受 cap_bytes 约束（默认 8GB），超限自动删除最旧会话（成对删除）。
 *
 * 启停（视觉与录像互不影响）：
 *   1) 启动参数：--record on|off（off = 连 CHN1/VENC 都不建，最省资源）
 *   2) API / socket：record_set_enabled(1|0)  → record on|off
 *      运行期 off = 当前会话收尾、之后码流丢弃（编码器继续空转，
 *      以保证 VICAP 通道始终有人消费，不会把 ISP 卡住）
 *   3) 信号：SIGUSR1 开始落盘、SIGUSR2 停止落盘
 */
#ifndef GREEN_LED_AI_RECORD_H
#define GREEN_LED_AI_RECORD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REC_DEFAULT_DIR       "/sdcard/app/recording"
#define REC_DEFAULT_CAP_BYTES (8ull << 30)   /* recording/ 目录总容量上限 8GB */
#define REC_ROLL_BYTES        (256ull << 20) /* 每段会话字节上限，到点自动滚动 */

typedef struct {
    uint64_t sessions;        /* 已开始的会话数（当前实现按 session_index 观察） */
    uint64_t streams;         /* 从编码器取回的码流帧数 */
    uint64_t written;         /* 落盘帧数（当前会话） */
    uint64_t discarded;       /* record off 期间丢弃的码流帧数 */
    uint64_t bytes;           /* 当前会话已写字节 */
    uint32_t real_fps;        /* 当前会话实际写入帧率 */
    uint32_t session_index;   /* 当前会话编号（0=未开） */
    int      active;          /* 当前是否在落盘 */
    int      enabled;         /* 常录开关 */
    int      bound;           /* VICAP->VENC 绑定是否成功 */
    const char *dir;
    uint64_t cap_bytes;
} record_stats_t;

/* 建 VENC 通道并绑定 VICAP 的录像通道（必须在 vicap_start 之前） */
int  record_bind(void);
/* 起取流/落盘线程（必须在 vicap_start 之后） */
int  record_run(void);
void record_deinit(void);

void record_get_stats(record_stats_t *st);

void record_set_enabled(int on);   /* 常录/落盘开关 */
int  record_enabled(void);
int  record_is_active(void);
int  record_is_started(void);

/* 诊断/降级用：停取流线程 + 解除绑定 + 释放 VENC 与输出池。
 * 视觉停流时用来判断「是不是 CHN1/绑定 影响了 CHN0」，返回 1=做过解绑 */
int  record_force_unbind(void);

void record_set_cap(uint64_t bytes);
int  record_set_dir(const char *dir);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_RECORD_H */
