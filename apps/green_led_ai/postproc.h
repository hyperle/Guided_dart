/*
 * postproc —— 检测后处理（纯逻辑，无 nncase/MPP 依赖，可在宿主机单元测试）
 *
 * 分工：
 *   - 本文件负责「模型原始输出 -> 检测框 -> NMS -> 选目标」这段纯数学，
 *     不碰任何硬件，因此能用合成数据在主机上跑测试（apps/green_led_ai/test/）。
 *   - 硬件侧的 AI2D/KPU 调度在 detect_kpu.cpp（那里也提供了用 SDK 自带
 *     librvv 的 nms() 的可选路径，见 --kpu-nms）。
 *
 * 当前实现的是 **YOLOv8/v11 风格 anchor-free 输出**：
 *   单输出张量，每个候选 (cx, cy, w, h, cls0..clsN-1)，坐标在「模型输入像素」
 *   空间（letterbox 后），需要除以 ratio 换回源码像素空间。
 *   支持两种排布：
 *     c_major=1: out[c * n_box + i]   （官方示例里 output0 的原始排布，C x N）
 *     c_major=0: out[i * n_ch  + c]   （N x C，部分导出/量化模型如此）
 *
 * NanoDet-Plus（GFL + softmax 分布式回归）的解码不同，后续换模型时只需在本文件
 * 加一个 yolov8_decode() 的同级函数，NMS/选目标逻辑完全复用。
 */
#ifndef GREEN_LED_AI_POSTPROC_H
#define GREEN_LED_AI_POSTPROC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x1, y1, x2, y2;   /* 源码像素坐标（已除 letterbox ratio） */
    float score;
    int   cls;
} det_box_t;

/*
 * YOLOv8 风格解码。
 *   out      模型输出首地址（float）
 *   n_box    候选框数量
 *   n_ch     每行通道数（= 4 + 类别数）
 *   c_major  1 = out[c*n_box+i]；0 = out[i*n_ch+c]
 *   nc       类别数
 *   conf_thr 置信度阈值
 *   ratio    模型输入 / 源码 的缩放比（letterbox 后）
 *   boxes/max_boxes  输出缓冲与容量
 * 返回实际写入的框数（不超过 max_boxes）。
 */
int yolov8_decode(const float *out, int n_box, int n_ch, int c_major,
                  int nc, float conf_thr, float ratio,
                  det_box_t *boxes, int max_boxes);

/*
 * 贪心 NMS（按类别抑制：不同类别互不压制；同类别 IoU > iou_thr 则抑制低分框）。
 * boxes 原地重排为「保留的框在前」，返回保留数量。keep 可为 NULL。
 */
int nms_boxes(det_box_t *boxes, int n, float iou_thr);

/*
 * 选目标：返回得分最高的框下标；cls_filter >= 0 时只看该类。
 * 没有可用框返回 -1。
 */
int pick_best(const det_box_t *boxes, int n, int cls_filter);

/* 单框 IoU（导出给测试用） */
float box_iou(const det_box_t *a, const det_box_t *b);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_POSTPROC_H */
