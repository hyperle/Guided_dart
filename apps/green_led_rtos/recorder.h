/*
 * green_led_rtos 录像模块接口（与绿灯识别解耦）。
 *
 * 职责：识别线程/采集线程只通过下面两个入口喂数据，本模块负责 H.264 编码、
 * 落盘与 recording/ 目录容量管理。识别代码不依赖本模块的任何内部实现。
 *
 * 默认行为 = 常录：进程启动后自动开启录像，每段会话默认滚到 256MB 自动续录
 * （recorder.c 的 REC_ROLL_BYTES），直到进程退出；总容量上限默认 8GB
 * （REC_DEFAULT_CAP_BYTES），超限自动删除最旧会话。
 *
 * 录像启停（外部其它模块/文件可任选其一）：
 *   1) API：      rec_start() / rec_stop()（同进程内其它模块直接调用，同步执行）
 *   2) 信号：      SIGUSR1 开启(恢复常录)、SIGUSR2 停止并暂停常录（pid 写入
 *                 /sdcard/app/green_led_rtos.pid，外部进程可 kill 该 pid）
 *   3) Unix socket：/sdcard/app/green_led_rtos.ctl，发送命令：
 *                 start / stop / status / cap <bytes> / dir <path>
 *   说明：手动 stop 后进入暂停（不再自动续录），再次 start/USR1 恢复常录。
 *
 * 输出：每次会话在 <record_dir>/（默认 /sdcard/app/recording）生成一对文件
 *     rec_%04u.h264   —— Annex-B H.264 裸流（含 SPS/PPS，可直接 ffmpeg 封装）
 *     rec_%04u.csv    —— 逐帧识别结果（frame_seq,result_seq,时间戳,center,fps）
 * recording/ 目录总大小受 cap_bytes 约束（默认 8GB），超限自动删除最旧会话。
 *
 * 保证：rec_feed_frame() 在录像开启时同步拷帧入环形缓冲（满则丢），
 * 不会等待编码器/SD，因此不影响识别帧率。
 */
#ifndef GREEN_LED_RECORDER_H
#define GREEN_LED_RECORDER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REC_CTRL_SOCKET      "/sdcard/app/green_led_rtos.ctl"
#define REC_PID_FILE         "/sdcard/app/green_led_rtos.pid"
#define REC_DEFAULT_DIR      "/sdcard/app/recording"
#define REC_DEFAULT_CAP_BYTES (8ull << 30) /* 8GB：recording/ 目录总容量上限 */

typedef struct {
    const char *record_dir;   /* recording 目录，NULL = 用默认 */
    uint64_t    cap_bytes;    /* 目录总容量上限（字节），0 = 用默认 */
    uint32_t    width;        /* 输入帧宽（640） */
    uint32_t    height;       /* 输入帧高（480） */
    uint32_t    nominal_fps;  /* 标称帧率，用于 H.264 RC（120） */
    uint32_t    bitrate_kbps; /* H.264 目标码率（默认 12000） */
} recorder_config_t;

/* 会话统计（rec_get_stats 返回） */
typedef struct {
    uint32_t sessions;        /* 已完成的会话数 */
    uint32_t frames_in;       /* 本次会话采集线程送入的帧数 */
    uint32_t frames_sent;     /* 本次会话成功送入编码器的帧数 */
    uint32_t frames_dropped;  /* 本次会话丢弃帧数（环形满 / 编码器忙） */
    uint32_t frames_written;  /* 本次会话实际写入 .h264 的帧数 */
    uint64_t bytes;           /* 本次会话 .h264 已写字节 */
} recorder_stats_t;

int  rec_init(const recorder_config_t *cfg);   /* 返回 0 成功；失败不影响识别 */
void rec_deinit(void);                          /* 停会话、关线程、释放编码器 */

/* 开启/停止录像并切换常录模式：
 *   rec_start() = 进入常录模式并立即开录（已在录则为 no-op，返回 0）；
 *   rec_stop()  = 停止当前会话并退出常录（暂停；start/USR1 恢复）。
 * 直接调用（任意线程）为同步执行，结束后返回；来自信号 SIGUSR1/SIGUSR2 与
 * socket 命令时由模块内部控制线程代为执行（信号处理函数只置位标志，安全）。
 * 可用 rec_is_active 查询当前是否有会话在跑。 */
int  rec_start(void);
int  rec_stop(void);
int  rec_is_active(void);

/* 识别线程每完成一帧检测调用一次：登记结果供录像元数据按帧对齐 */
void rec_note_result(uint64_t frame_seq, int32_t center_x, int32_t center_y,
                     uint32_t detect_fps);

/* 采集线程每帧调用一次：录像开启时拷贝该帧入队；返回 1=已入队，0=未录/丢弃。
 * phys_y: NV12 Y 平面物理地址（UV 紧跟其后，stride*height 起）；调用方须保证
 * 该 buffer 在调用期间有效。 */
int  rec_feed_frame(uint64_t phys_y, uint32_t stride, uint32_t width,
                    uint32_t height, uint64_t frame_seq, uint64_t mono_us);

/* 信号处理上下文安全：仅置位标志，由内部控制线程执行（异步） */
void rec_signal_start(void);
void rec_signal_stop(void);

void rec_set_cap_bytes(uint64_t cap);
void rec_get_stats(recorder_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_RECORDER_H */
