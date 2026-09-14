/*
 * detect —— 检测器接口：视觉子系统只依赖这个接口，后端可插拔
 *
 * 后端：
 *   color —— 纯 CPU/RVV：NV12 -> RGB565 -> 65536 项 CIELAB 查表 -> 阈值掩码
 *            -> 连通域筛选 -> 最大 blob 包围盒中心（无模型，立即可用）
 *   kpu   —— AI2D 硬件预处理 + KPU(NanoDet-Plus) + RVV 后处理（需要 kmodel）
 *
 * 统一的输出 detect_out_t 同时携带「结果」和「调参用探针」，日志口径不变。
 */
#ifndef GREEN_LED_AI_DETECT_H
#define GREEN_LED_AI_DETECT_H

#include <stddef.h>
#include <stdint.h>

#include "mpi_vicap_api.h"   /* k_video_frame_info（异步后端接管帧所有权要用） */

#ifdef __cplusplus
extern "C" {
#endif

/* 一帧的只读视图（Y/UV 平面已映射好；stride 为硬件行跨度） */
typedef struct {
    const uint8_t *y;        /* NV12: Y 平面；RGB_888_PLANAR: R 平面（三平面连续） */
    const uint8_t *uv;       /* NV12: UV 平面；RGB_882_PLANAR: 未用 */
    uint32_t       stride;
    uint32_t       width;
    uint32_t       height;
    uint64_t       seq;
    uint64_t       dump_us;  /* 该帧 dump 出来的时刻（算端到端延迟用） */
} frame_view_t;

typedef struct {
    uint64_t seq;           /* 产生该结果的那一帧序号（由调用方/后端填） */
    int32_t  cx, cy;        /* 最大 blob 包围盒中心；-1 = 未命中 */
    uint32_t px;            /* 该 blob 的估算图像像素数 */
    uint32_t blobs;         /* 通过形状筛选的 blob 数 */
    uint32_t green_px;      /* 落在阈值内的采样点数 */
    uint32_t roi_x, roi_y, roi_w, roi_h;
    int      full_scan;
    /* 探针：窗口内「最绿」像素（对照日志调阈值用） */
    int      probe_ok;
    int      probe_x, probe_y;
    int      probe_r, probe_g, probe_b;
    int      probe_L, probe_A, probe_B;
    /* 窗口内 Y 统计：判断画面是不是全黑/曝光够不够 */
    int      y_min, y_max;
    uint32_t y_mean;
    uint32_t t_us;          /* 本帧检测耗时(us)：同步后端=纯检测；异步后端=后处理耗时 */
    uint32_t lat_us;        /* 端到端延迟：帧 dump -> 结果产生（异步后端填） */
    float    score;         /* 置信度（kpu 后端；color 后端为 0） */
    int      cls;           /* 类别（kpu 后端；color 后端为 -1） */
} detect_out_t;

typedef struct detector {
    const char *name;

    /* 同步后端（color）：一次调用跑完全部并返回结果 */
    int  (*init)(void);
    void (*run)(const frame_view_t *f, int full_scan, detect_out_t *out);

    /*
     * 异步后端（kpu）：submit 立即返回，结果由 collect 取出。
     *   submit 接管该帧的**所有权**（后端在 AI2D 读完硬件之后负责
     *   kd_mpi_vicap_dump_release），返回值：
     *     0  = 已接管（调用方不要再动这帧）
     *     1  = 忙（调用方自己 release 并计丢帧）
     *    -1  = 错误（调用方自己 release）
     *   collect：1=取到一条结果并填好 *out；0=暂时没有
     *   stats_line：把本后端的耗时分解写进一行日志（可为 NULL）
     */
    int  (*submit)(const k_video_frame_info *frame, const frame_view_t *view, uint64_t seq);
    int  (*collect)(detect_out_t *out);
    void (*stats_line)(char *buf, size_t n);

    void (*deinit)(void);
} detector_t;

const detector_t *detector_color(void);
#ifdef GREEN_LED_WITH_KPU
const detector_t *detector_kpu(void);      /* 仅 KPU 目标里存在（detect_kpu.cpp） */
#endif
const detector_t *detector_get(const char *name);   /* 未知名 -> 返回 color */

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_DETECT_H */
