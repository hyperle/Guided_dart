/*
 * postproc 宿主机单元测试：用合成 YOLOv8 输出验证解码 / NMS / 选目标
 * 运行: bash apps/green_led_ai/test/run_host_test.sh
 */
#include <stdio.h>
#include <string.h>

#include "postproc.h"

#define N_BOX 8        /* 测试用的小规模，排布与真实一致 */
#define NC    3
#define N_CH  (4 + NC)

/* c_major=1 排布：out[c * N_BOX + i] */
static float out_cmajor[N_CH * N_BOX];

static void set_box(int i, float cx, float cy, float w, float h,
                    float s0, float s1, float s2)
{
    out_cmajor[0 * N_BOX + i] = cx;
    out_cmajor[1 * N_BOX + i] = cy;
    out_cmajor[2 * N_BOX + i] = w;
    out_cmajor[3 * N_BOX + i] = h;
    out_cmajor[4 * N_BOX + i] = s0;
    out_cmajor[5 * N_BOX + i] = s1;
    out_cmajor[6 * N_BOX + i] = s2;
}

int main(void)
{
    int fail = 0;
    memset(out_cmajor, 0, sizeof(out_cmajor));

    /*
     * ratio = 0.5（模型输入 320，源码 640x360 的横向缩放）：
     * 模型空间中心 (80,60)、宽高 (20,20) -> 源码空间中心 (160,120)、40x40
     */
    set_box(0, 80.0f, 60.0f, 20.0f, 20.0f, 0.05f, 0.90f, 0.02f);  /* 绿灯类(cls1) */
    set_box(1, 81.0f, 60.5f, 20.0f, 20.0f, 0.05f, 0.80f, 0.02f);  /* 同类别高度重叠 -> 应被抑制 */
    set_box(2, 140.0f, 30.0f, 16.0f, 16.0f, 0.70f, 0.10f, 0.05f); /* 另一类(cls0)，远离 -> 保留 */
    set_box(3, 200.0f, 100.0f, 18.0f, 18.0f, 0.01f, 0.20f, 0.01f);/* 低分 -> 被阈值滤掉 */

    det_box_t boxes[N_BOX];
    int n = yolov8_decode(out_cmajor, N_BOX, N_CH, 1, NC, 0.30f, 0.5f, boxes, N_BOX);
    printf("TEST1 decode: n=%d (expect 3) %s\n", n, n == 3 ? "PASS" : "FAIL");
    if (n != 3)
        fail++;

    if (n >= 3) {
        float cx = 0.5f * (boxes[0].x1 + boxes[0].x2);
        float cy = 0.5f * (boxes[0].y1 + boxes[0].y2);
        int ok = (boxes[0].cls == 1) &&
                 (cx > 159.0f && cx < 161.0f) &&
                 (cy > 119.0f && cy < 121.0f) &&
                 (boxes[0].x2 - boxes[0].x1 > 39.0f && boxes[0].x2 - boxes[0].x1 < 41.0f);
        printf("TEST2 letterbox 反算: best cls=%d center=(%.1f,%.1f) box=(%.1f,%.1f,%.1f,%.1f) %s\n",
               boxes[0].cls, cx, cy, boxes[0].x1, boxes[0].y1, boxes[0].x2, boxes[0].y2,
               ok ? "PASS" : "FAIL");
        if (!ok)
            fail++;
    }

    int kept = nms_boxes(boxes, n, 0.45f);
    printf("TEST3 nms: kept=%d (expect 2, 同类重叠框被抑制) %s\n", kept,
           kept == 2 ? "PASS" : "FAIL");
    if (kept != 2)
        fail++;

    int bi = pick_best(boxes, kept, 1);
    int ok4 = (bi >= 0 && boxes[bi].cls == 1 && boxes[bi].score > 0.89f);
    printf("TEST4 pick_best(cls=1): idx=%d cls=%d score=%.2f %s\n", bi,
           bi >= 0 ? boxes[bi].cls : -1, bi >= 0 ? boxes[bi].score : 0.0f,
           ok4 ? "PASS" : "FAIL");
    if (!ok4)
        fail++;

    /* ratio=1 时坐标应原样返回 */
    n = yolov8_decode(out_cmajor, N_BOX, N_CH, 1, NC, 0.30f, 1.0f, boxes, N_BOX);
    float cx0 = 0.5f * (boxes[0].x1 + boxes[0].x2);
    int ok5 = (n == 3 && cx0 > 79.0f && cx0 < 81.0f);
    printf("TEST5 ratio=1: n=%d center_x=%.1f (expect ~80) %s\n", n, cx0, ok5 ? "PASS" : "FAIL");
    if (!ok5)
        fail++;

    /* N x C 排布（c_major=0）应与 C x N 等价 */
    static float out_nmajor[N_BOX * N_CH];
    memset(out_nmajor, 0, sizeof(out_nmajor));
    for (int i = 0; i < N_BOX; ++i) {
        for (int c = 0; c < N_CH; ++c)
            out_nmajor[i * N_CH + c] = out_cmajor[c * N_BOX + i];
    }
    det_box_t b1[N_BOX], b2[N_BOX];
    int n1 = yolov8_decode(out_cmajor, N_BOX, N_CH, 1, NC, 0.30f, 0.5f, b1, N_BOX);
    int n2 = yolov8_decode(out_nmajor, N_BOX, N_CH, 0, NC, 0.30f, 0.5f, b2, N_BOX);
    int ok6 = (n1 == n2 && n1 == 3);
    for (int i = 0; i < n1 && i < n2 && ok6; ++i) {
        if (b2[i].cls != b1[i].cls ||
            b2[i].x1 != b1[i].x1 || b2[i].y1 != b1[i].y1 ||
            b2[i].x2 != b1[i].x2 || b2[i].y2 != b1[i].y2)
            ok6 = 0;
    }
    printf("TEST6 N x C 排布与 C x N 等价: n=%d/%d %s\n", n1, n2, ok6 ? "PASS" : "FAIL");
    if (!ok6)
        fail++;

    printf("postproc host test: %s (%d failures)\n", fail ? "FAIL" : "ALL PASS", fail);
    return fail ? 1 : 0;
}
