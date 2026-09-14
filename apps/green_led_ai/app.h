/*
 * green_led_ai —— 公共定义：配置、日志、运行结果、MPP 串行化与物理内存映射
 *
 * 设计要点（都是踩过的坑，详见 README.md「为什么这样设计」）：
 *
 * 1) VICAP 帧“单一所有者”原则
 *    ★ 一个 dump 出来的帧在同一时刻只能有一个所有者，且只有一条释放路径：
 *        - 送入 VENC（kd_mpi_venc_send_frame 返回 K_SUCCESS）后，帧归编码器所有，
 *          **应用层绝不能再调用 kd_mpi_vicap_dump_release**（MPP 没有
 *          kd_mpi_venc_release_frame，编码器用完自行归还）。
 *        - 未送编码器 / 送失败 / 被丢弃的帧，必须由应用层释放一次。
 *    旧工程 green_led_rtos 的恶性 bug 就是把同一帧既送 VENC 又由识别线程
 *    dump_release：VB 引用计数被打崩，几个帧后 6 个 buffer 耗光，
 *    kd_mpi_vicap_dump_frame 永久返回 0xa0158010（VICAP + K_ERR_NOTREADY）。
 *    本工程用“视觉 CHN0 / 录像 CHN1”两条独立通道从根上隔离所有权。
 *
 * 2) MPP 调用全进程串行
 *    实测 kd_mpi_sys_mmap/munmap 与 kd_mpi_venc_send_frame / get_stream /
 *    release_stream 在**多线程并发**时会把 RT-Smart 打挂（整机重启）。
 *    因此所有 MPP 调用（含 mmap）都必须包在 mpp_enter()/mpp_leave() 之间。
 *    锁内只做 MPP 调用，不做任何可能长阻塞的业务（写 SD 等）。
 *
 * 3) 物理内存映射“一次映射、永不逐帧解映射”
 *    mpp_map_persist() 按物理地址缓存映射：整个进程生命周期内每个物理块只
 *    mmap 一次，且只在 mpp_enter() 内调用（即单线程化）；映射直到
 *    mpp_shutdown()（所有工作线程 join 之后）才统一 munmap。
 *    旧工程“每帧 map/unmap + 长度不匹配”直接把内核打挂（实测 29 次重启循环）。
 */
#ifndef GREEN_LED_AI_APP_H
#define GREEN_LED_AI_APP_H

#include <stdint.h>

#include "mpi_sys_api.h"     /* k_u32 / k_u64 / kd_mpi_sys_mmap */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 路径约定 ============================ */

#define APP_NAME_STR        "green_led_ai"
#define APP_LOG_PATH        "/sdcard/app/logs/green_led_ai.log"
#define APP_LOG_DIR         "/sdcard/app/logs"
#define APP_PID_FILE        "/sdcard/app/green_led_ai.pid"
#define APP_CTL_SOCKET      "/sdcard/app/green_led_ai.ctl"

/* ============================ 配置 ============================ */

typedef struct {
    /* 采集 */
    int      csi;              /* CSI 号，庐山派摄像头在 CSI2 */
    int      probe_fps;        /* 探测请求帧率（必须与 1920x1080 成对出现） */
    uint32_t acq_w, acq_h;     /* 探测请求分辨率 */
    int      ae_enable;        /* 1=自动曝光；默认 0=固定曝光 */
    int      exposure_us;      /* 固定曝光(us)，0=不设 */
    int      fix_gain;         /* 1=把增益压到最低 */

    /* 子系统开关（可分别单独启停） */
    int      vision_on;
    int      record_on;
    int      vicap_online;      /* 1=ONLINE 模式（双通道走官方组合）0=OFFLINE */

    /* 双通道输出尺寸 */
    uint32_t vis_w, vis_h;     /* 视觉通道 CHN0 */
    uint32_t rec_w, rec_h;     /* 录像通道 CHN1 */

    /* 视觉检测（color 检测器参数，与旧工程 CLIELAB 口径一致） */
    uint32_t roi_x, roi_y, roi_w, roi_h;
    uint32_t step_x, step_y;
    int      lab_l_min, lab_a_max, lab_b_min;
    uint32_t pix_min;
    int      miss_full_scan;

    /* 录像 */
    int         rec_fps;         /* 录像抽样帧率（视觉通道可以远高于它） */
    uint32_t    bitrate_kbps;
    const char *rec_dir;
    uint64_t    cap_bytes;

    /* 检测器后端 */
    const char *detector;        /* color | kpu */
    const char *kmodel;          /* kpu 后端模型路径 */
    int         vis_rgb_planar;  /* 1 = 视觉通道输出 RGB_888_PLANAR（kpu 后端需要） */
    float       kpu_conf;        /* kpu: 置信度阈值 */
    float       kpu_iou;         /* kpu: NMS IoU 阈值 */
    int         kpu_class;       /* kpu: 只认这个类别（-1=任意类别取最高分） */
    int         kpu_nc;          /* kpu: 类别数（0=从输出形状推断） */
    const char *kpu_nms;         /* kpu: own | librvv */
    int         kpu_sync;        /* kpu: 是否对输入 tensor 做 sync_write_back */

    /* 运维 */
    int pm_perf;
    int      rec_mode;          /* 0=shared（默认：与识别共享通道，应用交棒）
                                 * 1=bind（双通道硬件直连，实验特性） */
    int      dump_timeout_ms;   /* dump 等待上限(ms)；dump 是"阻塞等下一帧"，给足即可 */
    int status_period_s;
    int trace_frames;            /* 前 N 帧逐帧 trace（定位板子卡在哪一步） */
} cfg_t;

extern cfg_t g_cfg;

/* ============================ 日志 ============================ */

/*
 * 与旧工程完全一致的日志模式：每行 = 一次 open/write/fsync/close，
 * 因此 MTP/文件管理器随时能读到；同时写 stdout（launcher 会重定向到
 * /sdcard/app/logs/launcher-child.log，msh 手动跑也直接可见）。
 */
void app_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* 前 N 帧的逐帧 trace（g_cfg.trace_frames 控制；0=关闭） */
void trace_log(uint64_t seq, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* ============================ 时间 ============================ */

uint64_t mono_us(void);

/* ============================ 最近一次识别结果 ============================ */

typedef struct {
    int32_t  center_x;
    int32_t  center_y;
    uint64_t seq;      /* 产生该结果的视觉帧号 */
    uint32_t fps;      /* 产生结果时的视觉帧率 */
    int      found;
} result_t;

void result_publish(int32_t cx, int32_t cy, uint64_t seq, uint32_t fps);
void result_get(result_t *out);

/* ============================ MPP 串行化 ============================ */

/* 进入/离开 MPP 临界区（不可嵌套；锁内不要做业务，尤其不要写文件） */
void mpp_enter(void);
void mpp_leave(void);

/*
 * 物理地址 → 用户态虚拟地址（进程内缓存，只映射一次；必须在 mpp_enter/leave
 * 之间调用）。返回 NULL 表示失败。
 *
 * 默认 mpp_map_persist() 用**非 cache** 映射（安全但慢：每个字节都是一次强序
 * 总线访问，逐像素扫描时实测能吃掉几十毫秒）。
 * mpp_map_persist_cached() 用 cache 映射：CPU 读得快，但必须在**每次读之前**
 * 调用 mpp_invalidate() 把 DMA 刚写入的新数据对应的旧 cache 行作废，
 * 否则会读到上一帧的残留（这正是老工程"读到全黑帧"的另一条可能路径）。
 */
void *mpp_map_persist(k_u64 phys, k_u32 size);
void *mpp_map_persist_cached(k_u64 phys, k_u32 size);

/* 让 [phys, va, size) 这段 cache 作废（DMA 写完、CPU 读之前调用） */
void  mpp_invalidate(k_u64 phys, void *va, k_u32 size);

/* 统一释放所有缓存映射（必须在所有工作线程 join 之后调用） */
void mpp_shutdown(void);

typedef struct {
    uint64_t calls;        /* 进入临界区次数 */
    uint64_t contended;    /* trylock 失败（真的发生过并发）次数 */
    uint64_t max_hold_us;  /* 单次持锁最长时间 */
    uint32_t maps;         /* 累计 mmap 次数（应远小于帧数） */
} mpp_stats_t;

void mpp_get_stats(mpp_stats_t *st);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_APP_H */
