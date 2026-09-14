/*
 * postproc —— 实现（纯逻辑，无硬件依赖）
 */
#include "postproc.h"

#include <math.h>
#include <string.h>

float box_iou(const det_box_t *a, const det_box_t *b)
{
    float xx1 = a->x1 > b->x1 ? a->x1 : b->x1;
    float yy1 = a->y1 > b->y1 ? a->y1 : b->y1;
    float xx2 = a->x2 < b->x2 ? a->x2 : b->x2;
    float yy2 = a->y2 < b->y2 ? a->y2 : b->y2;

    float w = xx2 - xx1;
    float h = yy2 - yy1;
    if (w <= 0.0f || h <= 0.0f)
        return 0.0f;

    float inter = w * h;
    float area_a = (a->x2 - a->x1) * (a->y2 - a->y1);
    float area_b = (b->x2 - b->x1) * (b->y2 - b->y1);
    float uni = area_a + area_b - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

int yolov8_decode(const float *out, int n_box, int n_ch, int c_major,
                  int nc, float conf_thr, float ratio,
                  det_box_t *boxes, int max_boxes)
{
    int n = 0;
    if (!out || !boxes || n_box <= 0 || nc <= 0 || n_ch < 4 + nc || max_boxes <= 0)
        return 0;
    if (ratio <= 0.0f)
        ratio = 1.0f;

    for (int i = 0; i < n_box; ++i) {
        /* 取该候选里得分最高的类别 */
        float best = -1e30f;
        int   best_c = -1;
        for (int c = 0; c < nc; ++c) {
            float v = c_major ? out[(size_t)(4 + c) * n_box + i]
                              : out[(size_t)i * n_ch + (4 + c)];
            if (v > best) {
                best = v;
                best_c = c;
            }
        }
        if (best < conf_thr || best_c < 0)
            continue;

        float cx = c_major ? out[(size_t)0 * n_box + i] : out[(size_t)i * n_ch + 0];
        float cy = c_major ? out[(size_t)1 * n_box + i] : out[(size_t)i * n_ch + 1];
        float bw = c_major ? out[(size_t)2 * n_box + i] : out[(size_t)i * n_ch + 2];
        float bh = c_major ? out[(size_t)3 * n_box + i] : out[(size_t)i * n_ch + 3];

        if (!(bw > 0.0f) || !(bh > 0.0f))
            continue;

        /* 模型输入像素 -> 源码像素（与官方示例同一口径：除以 letterbox ratio） */
        float x = cx / ratio;
        float y = cy / ratio;
        float w = bw / ratio;
        float h = bh / ratio;

        boxes[n].x1 = x - 0.5f * w;
        boxes[n].y1 = y - 0.5f * h;
        boxes[n].x2 = x + 0.5f * w;
        boxes[n].y2 = y + 0.5f * h;
        boxes[n].score = best;
        boxes[n].cls = best_c;
        n++;
        if (n >= max_boxes)
            break;
    }
    return n;
}

int nms_boxes(det_box_t *boxes, int n, float iou_thr)
{
    if (!boxes || n <= 1)
        return n < 0 ? 0 : n;

    /* 1) 按分数降序做插入排序（候选数量不大，避免依赖 qsort 的比较器开销） */
    for (int i = 1; i < n; ++i) {
        det_box_t t = boxes[i];
        int j = i - 1;
        while (j >= 0 && boxes[j].score < t.score) {
            boxes[j + 1] = boxes[j];
            j--;
        }
        boxes[j + 1] = t;
    }

    /* 2) 贪心抑制：被抑制的框标记后压缩到尾部 */
    uint8_t dead[512];
    if (n > (int)sizeof(dead))
        n = (int)sizeof(dead);           /* 上限保护（正常远小于此） */
    memset(dead, 0, (size_t)n);

    int kept = 0;
    for (int i = 0; i < n; ++i) {
        if (dead[i])
            continue;
        for (int j = i + 1; j < n; ++j) {
            if (dead[j] || boxes[j].cls != boxes[i].cls)
                continue;
            if (box_iou(&boxes[i], &boxes[j]) > iou_thr)
                dead[j] = 1;
        }
        boxes[kept++] = boxes[i];
    }
    return kept;
}

int pick_best(const det_box_t *boxes, int n, int cls_filter)
{
    int best = -1;
    float best_score = -1.0f;
    for (int i = 0; i < n; ++i) {
        if (cls_filter >= 0 && boxes[i].cls != cls_filter)
            continue;
        if (boxes[i].score > best_score) {
            best_score = boxes[i].score;
            best = i;
        }
    }
    return best;
}
