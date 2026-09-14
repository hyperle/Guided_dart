/*
 * detect_color —— 无模型检测器：NV12 -> RGB565 -> CIELAB 查表阈值 -> 连通域
 *
 * 与旧工程 / Python 原型完全同一套颜色口径：
 *   - NV12 按 BT.601 视频范围做整数转 RGB（与旧工程相同系数）
 *   - 先量化到 RGB565（CanMV find_blobs 就是在 RGB565 上查 lab_table）
 *   - 阈值 L>=l_min(<=100) && A<=a_max && B>=b_min，默认 12 / -20 / 8
 *   - 连通域（8 邻域）后按 最小像素数 / 填充率 / 长宽比 筛选，取最大 blob 中心
 *
 * 与旧实现的区别（这就是「消除 CPU 瓶颈」的部分）：
 *   旧实现逐采样点做 float sRGB->CIELAB（powf/cbrtf 查表 + 多次浮点乘加）。
 *   这里在 init 时把「RGB565 -> 是否命中阈值」预先算成 64KB 查表（g_lut565），
 *   运行期每个采样点只剩：NV12->RGB565 整数运算 + 1 次查表。
 *   NV12->RGB565 用 RVV 向量化（128bit 向量单元，16 像素/迭代），
 *   向量宽度/步长不满足条件时自动退回标量路径。
 *
 * 计时：out->t_us 给出本帧纯检测耗时（不含映射/释放），用于核对 <5ms 目标。
 */
#include "detect.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"

#if defined(__riscv_vector)
#include <riscv_vector.h>
#define DET_HAVE_RVV 1
#endif

/* blob 形状筛选：与 Python 原型的 FILL_LO / ASPECT_LO~HI 一致 */
#define FILL_LO_PCT   55   /* est_px*100 >= 55*w*h */
#define ASPECT_LO_NUM 7    /* 10*w >= 7*h  */
#define ASPECT_HI_NUM 14   /* 10*w <= 14*h */

#define CBRT_LUT_N 1024

/* ============================ 颜色表 ============================ */

static float   g_srgb_lin[256];
static float   g_cbrt_lut[CBRT_LUT_N + 1];
static uint8_t g_lut565[65536];     /* RGB565 -> 是否命中 Lab 阈值 */

static int quant565_r(int v) { return (v & 0xF8) | (v >> 5); }
static int quant565_g(int v) { return (v & 0xFC) | (v >> 6); }
static int quant565_b(int v) { return (v & 0xF8) | (v >> 5); }

static inline float fast_cbrt(float t)
{
    if (t <= 0.0f)
        return 0.0f;
    if (t >= 1.0f)
        return 1.0f;
    float x = t * (float)CBRT_LUT_N;
    int   i = (int)x;
    float f = x - (float)i;
    return g_cbrt_lut[i] * (1.0f - f) + g_cbrt_lut[i + 1] * f;
}

static inline float lab_f(float t)
{
    if (t <= 0.008856f)
        return 7.787f * t + 16.0f / 116.0f;
    return fast_cbrt(t);
}

/* sRGB(D65) -> CIELAB，四舍五入取整（与 CanMV lab_table 同口径） */
static void rgb_to_lab(int r, int g, int b, int *L, int *A, int *B)
{
    r = quant565_r(r);
    g = quant565_g(g);
    b = quant565_b(b);
    float R = g_srgb_lin[r], G = g_srgb_lin[g], Bb = g_srgb_lin[b];
    float X = 0.4124564f * R + 0.3575761f * G + 0.1804375f * Bb;
    float Y = 0.2126729f * R + 0.7151522f * G + 0.0721750f * Bb;
    float Z = 0.0193339f * R + 0.1191920f * G + 0.9503041f * Bb;
    float fx = lab_f(X / 0.95047f);
    float fy = lab_f(Y / 1.00000f);
    float fz = lab_f(Z / 1.08883f);
    *L = (int)lroundf(116.0f * fy - 16.0f);
    *A = (int)lroundf(500.0f * (fx - fy));
    *B = (int)lroundf(200.0f * (fy - fz));
}

/* 由 RGB565 展开出 8bit RGB（与旧工程 quant565_* 的展开方式一致） */
static inline void rgb565_unpack(int v, int *r, int *g, int *b)
{
    int r5 = (v >> 11) & 0x1F;
    int g6 = (v >> 5) & 0x3F;
    int b5 = v & 0x1F;
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}

static void color_lut_build(void)
{
    for (int i = 0; i < 256; ++i) {
        float c = (float)i / 255.0f;
        g_srgb_lin[i] = (c <= 0.04045f)
                            ? (c / 12.92f)
                            : powf((c + 0.055f) / 1.055f, 2.4f);
    }
    for (int i = 0; i <= CBRT_LUT_N; ++i)
        g_cbrt_lut[i] = cbrtf((float)i / (float)CBRT_LUT_N);

    for (int v = 0; v < 65536; ++v) {
        int r, g, b, L, A, B;
        rgb565_unpack(v, &r, &g, &b);
        rgb_to_lab(r, g, b, &L, &A, &B);
        int pass = (L >= g_cfg.lab_l_min && L <= 100 &&
                    A <= g_cfg.lab_a_max && B >= g_cfg.lab_b_min);
        g_lut565[v] = pass ? 1 : 0;
    }
}

/* ============================ 采样点掩码 ============================ */

typedef struct {
    int      probe_best;
    int      probe_x, probe_y;
    int      probe_r, probe_g, probe_b;
    int      probe_L, probe_A, probe_B;
    int      y_min, y_max;
    uint64_t y_sum;
    uint32_t green;
} scan_acc_t;

static uint8_t *g_mask;
static uint32_t *g_stack;
static uint32_t  g_cells;

static int scratch_ensure(uint32_t cells)
{
    if (cells <= g_cells && g_mask && g_stack)
        return 0;
    free(g_mask);
    free(g_stack);
    g_mask = (uint8_t *)malloc(cells);
    g_stack = (uint32_t *)malloc((size_t)cells * sizeof(uint32_t));
    if (!g_mask || !g_stack) {
        free(g_mask);
        free(g_stack);
        g_mask = NULL;
        g_stack = NULL;
        g_cells = 0;
        return -1;
    }
    g_cells = cells;
    return 0;
}

/* 单个采样点：NV12 -> RGB -> RGB565 -> 查表 -> 掩码 + 探针 + Y 统计 */
static inline void scan_pixel(uint8_t yv, uint8_t uu, uint8_t vv, uint32_t ix,
                              uint32_t iy, uint8_t *slot, scan_acc_t *acc)
{
    int c = (int)yv - 16;
    int d = (int)uu - 128;
    int e = (int)vv - 128;
    int rr = (298 * c + 409 * e + 128) >> 8;
    int gg = (298 * c - 100 * d - 208 * e + 128) >> 8;
    int bb = (298 * c + 516 * d + 128) >> 8;
    if (rr < 0) rr = 0; else if (rr > 255) rr = 255;
    if (gg < 0) gg = 0; else if (gg > 255) gg = 255;
    if (bb < 0) bb = 0; else if (bb > 255) bb = 255;

    if ((int)yv < acc->y_min) acc->y_min = (int)yv;
    if ((int)yv > acc->y_max) acc->y_max = (int)yv;
    acc->y_sum += yv;

    int idx = ((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3);
    uint8_t pass = g_lut565[idx];
    *slot = pass;
    if (pass)
        acc->green++;

    /* 探针用 RGB565 反解后的值（与 RVV 路径逐位一致，且就是查表实际看到的值） */
    int ur, ug, ub;
    rgb565_unpack(idx, &ur, &ug, &ub);
    int gr = ug - (ur > ub ? ur : ub);
    if (gr > acc->probe_best) {
        acc->probe_best = gr;
        acc->probe_x = (int)ix;
        acc->probe_y = (int)iy;
        acc->probe_r = ur;
        acc->probe_g = ug;
        acc->probe_b = ub;
    }
}

/* ============================ RVV 行扫描 ============================ */

#if defined(DET_HAVE_RVV)

/*
 * 处理一行里连续的 16 个采样点：
 *   - 采样步长 sx 为偶数且窗口左边界 wx 为偶数时，UV 采样也是恒定步长 sx
 *     （ix 恒为偶数 -> UV 字节偏移 = ix），因此可以用 vlse8 跨步加载。
 *   - Y/U/V 用 vlse8 取 16 个点，零扩展到 32bit 后做 BT.601 整数矩阵，
 *     夹到 0..255 后打包成 RGB565 索引，最后 16 次标量查表。
 * 返回本次消化的采样点数（0 表示条件不满足，调用方走标量）。
 *
 * 注意：本仓库工具链是 GCC 12（RVV intrinsic 用无前缀命名，如 vsetvl_e32m4）。
 */
static uint32_t rvv_scan_row16(const uint8_t *yp, const uint8_t *uvp, uint32_t wx,
                               uint32_t sx, uint32_t gw, uint8_t *mrow,
                               scan_acc_t *acc, uint32_t gy)
{
    uint32_t gx = 0;
    uint32_t idxbuf[16] = { 0 };   /* vl 保证 16，仍显式初始化以杜绝未初始化读 */

    while (gx + 16 <= gw) {
        size_t vl = vsetvl_e32m4(16);
        if (vl < 16)
            break;

        const uint8_t *ybase = yp + wx + (size_t)gx * sx;
        const uint8_t *ubase = uvp + wx + (size_t)gx * sx;

        vuint8m1_t vy8 = vlse8_v_u8m1(ybase, (ptrdiff_t)sx, vl);
        vuint8m1_t vu8 = vlse8_v_u8m1(ubase, (ptrdiff_t)sx, vl);
        vuint8m1_t vv8 = vlse8_v_u8m1(ubase + 1, (ptrdiff_t)sx, vl);

        vuint32m4_t vy32 = vzext_vf4_u32m4(vy8, vl);
        vuint32m4_t vu32 = vzext_vf4_u32m4(vu8, vl);
        vuint32m4_t vv32 = vzext_vf4_u32m4(vv8, vl);

        vint32m4_t c = vsub_vx_i32m4(vreinterpret_v_u32m4_i32m4(vy32), 16, vl);
        vint32m4_t d = vsub_vx_i32m4(vreinterpret_v_u32m4_i32m4(vu32), 128, vl);
        vint32m4_t e = vsub_vx_i32m4(vreinterpret_v_u32m4_i32m4(vv32), 128, vl);

        vint32m4_t t298 = vmul_vx_i32m4(c, 298, vl);
        vint32m4_t rr = vsra_vx_i32m4(
            vadd_vx_i32m4(vadd_vv_i32m4(t298, vmul_vx_i32m4(e, 409, vl), vl), 128, vl),
            8, vl);
        vint32m4_t gg = vsra_vx_i32m4(
            vadd_vx_i32m4(
                vsub_vv_i32m4(vsub_vv_i32m4(t298, vmul_vx_i32m4(d, 100, vl), vl),
                              vmul_vx_i32m4(e, 208, vl), vl),
                128, vl),
            8, vl);
        vint32m4_t bb = vsra_vx_i32m4(
            vadd_vx_i32m4(vadd_vv_i32m4(t298, vmul_vx_i32m4(d, 516, vl), vl), 128, vl),
            8, vl);

        rr = vmin_vx_i32m4(vmax_vx_i32m4(rr, 0, vl), 255, vl);
        gg = vmin_vx_i32m4(vmax_vx_i32m4(gg, 0, vl), 255, vl);
        bb = vmin_vx_i32m4(vmax_vx_i32m4(bb, 0, vl), 255, vl);

        /* idx = (r>>3)<<11 | (g>>2)<<5 | (b>>3)
         * 注意：本工具链（GCC 12，RVV v0.11 命名）的移位 intrinsic 只对
         * 无符号类型提供，所以先 reinterpret 成 u32 再移位。 */
        vuint32m4_t urr = vreinterpret_v_i32m4_u32m4(rr);
        vuint32m4_t ugg = vreinterpret_v_i32m4_u32m4(gg);
        vuint32m4_t ubb = vreinterpret_v_i32m4_u32m4(bb);
        vuint32m4_t ir = vsll_vx_u32m4(vsrl_vx_u32m4(urr, 3, vl), 11, vl);
        vuint32m4_t ig = vsll_vx_u32m4(vsrl_vx_u32m4(ugg, 2, vl), 5, vl);
        vuint32m4_t ib = vsrl_vx_u32m4(ubb, 3, vl);
        vuint32m4_t uidx = vor_vv_u32m4(vor_vv_u32m4(ir, ig, vl), ib, vl);

        vse32_v_u32m4(idxbuf, uidx, vl);

        for (int k = 0; k < 16; ++k) {
            uint32_t ix = wx + (gx + (uint32_t)k) * sx;
            uint8_t  yv = ybase[(size_t)k * sx];

            if ((int)yv < acc->y_min) acc->y_min = (int)yv;
            if ((int)yv > acc->y_max) acc->y_max = (int)yv;
            acc->y_sum += yv;

            uint8_t pass = g_lut565[idxbuf[k] & 0xFFFFu];
            mrow[gx + (uint32_t)k] = pass;
            if (pass)
                acc->green++;

            /* 探针需要真实 RGB：与标量路径一致地用 RGB565 反解 */
            int r, g, b;
            rgb565_unpack((int)(idxbuf[k] & 0xFFFFu), &r, &g, &b);
            int gr = g - (r > b ? r : b);
            if (gr > acc->probe_best) {
                acc->probe_best = gr;
                acc->probe_x = (int)ix;
                acc->probe_y = (int)gy;
                acc->probe_r = r;
                acc->probe_g = g;
                acc->probe_b = b;
            }
        }
        gx += 16;
    }
    return gx;
}

#endif /* DET_HAVE_RVV */

/* ============================ 主检测 ============================ */

/*
 * 扫描一个窗口，填充 mask（gw x gh，行主序）并累计统计。
 * allow_rvv=0 时强制走标量路径（自检用来和 RVV 对比）。
 */
static void scan_window(const frame_view_t *f, uint32_t wx, uint32_t wy,
                        uint32_t wh, uint32_t sx, uint32_t sy, uint8_t *mask,
                        uint32_t gw, scan_acc_t *acc, int allow_rvv)
{
    uint32_t gh = (wh + sy - 1) / sy;

    for (uint32_t gy = 0; gy < gh; ++gy) {
        uint32_t iy = wy + gy * sy;
        const uint8_t *yp = f->y + (size_t)iy * f->stride;
        const uint8_t *uvp = f->uv + (size_t)(iy >> 1) * f->stride;
        uint8_t *mrow = mask + (size_t)gy * gw;
        uint32_t gx = 0;

#if defined(DET_HAVE_RVV)
        /* RVV 路径前提：窗口左边界与步长都是偶数（UV 采样才是恒定步长） */
        if (allow_rvv && (wx & 1u) == 0 && (sx & 1u) == 0 && gw >= 16)
            gx = rvv_scan_row16(yp, uvp, wx, sx, gw, mrow, acc, iy);
#else
        (void)allow_rvv;
#endif
        for (; gx < gw; ++gx) {
            uint32_t ix = wx + gx * sx;
            uint32_t uvo = (ix >> 1) * 2u;
            scan_pixel(yp[ix], uvp[uvo], uvp[uvo + 1], ix, iy, &mrow[gx], acc);
        }
    }
}

static void detect_color_run(const frame_view_t *f, int full_scan, detect_out_t *out)
{
    uint64_t t0 = mono_us();

    memset(out, 0, sizeof(*out));
    out->cx = -1;
    out->cy = -1;
    out->cls = -1;

    uint32_t wx, wy, ww, wh;
    if (full_scan || g_cfg.roi_w == 0 || g_cfg.roi_h == 0) {
        wx = 0; wy = 0; ww = f->width; wh = f->height;
    } else {
        wx = g_cfg.roi_x; wy = g_cfg.roi_y;
        ww = g_cfg.roi_w; wh = g_cfg.roi_h;
        if (wx >= f->width) wx = 0;
        if (wy >= f->height) wy = 0;
        if (ww > f->width - wx) ww = f->width - wx;
        if (wh > f->height - wy) wh = f->height - wy;
    }
    out->roi_x = wx; out->roi_y = wy; out->roi_w = ww; out->roi_h = wh;
    out->full_scan = full_scan;

    uint32_t sx = g_cfg.step_x ? g_cfg.step_x : 1;
    uint32_t sy = g_cfg.step_y ? g_cfg.step_y : 1;
    uint32_t gw = (ww + sx - 1) / sx;
    uint32_t gh = (wh + sy - 1) / sy;

    if (gw == 0 || gh == 0 || scratch_ensure(gw * gh) != 0) {
        out->t_us = (uint32_t)(mono_us() - t0);
        return;
    }

    scan_acc_t acc;
    memset(&acc, 0, sizeof(acc));
    acc.probe_best = -1;
    acc.y_min = 255;
    acc.y_max = 0;

    scan_window(f, wx, wy, wh, sx, sy, g_mask, gw, &acc, 1);

    out->green_px = acc.green;
    out->y_min = acc.y_min;
    out->y_max = acc.y_max;
    out->y_mean = (gw && gh) ? (uint32_t)(acc.y_sum / (uint64_t)(gw * gh)) : 0;
    if (acc.probe_best >= 0) {
        out->probe_ok = 1;
        out->probe_x = acc.probe_x;
        out->probe_y = acc.probe_y;
        out->probe_r = acc.probe_r;
        out->probe_g = acc.probe_g;
        out->probe_b = acc.probe_b;
        rgb_to_lab(acc.probe_r, acc.probe_g, acc.probe_b,
                   &out->probe_L, &out->probe_A, &out->probe_B);
    }

    if (acc.green == 0) {
        out->t_us = (uint32_t)(mono_us() - t0);
        return;
    }

    /* --- 连通域（BFS，mask 访问后清零，省一个 seen 数组） --- */
    uint32_t best_px = 0;
    for (uint32_t gy = 0; gy < gh; ++gy) {
        for (uint32_t gx = 0; gx < gw; ++gx) {
            uint32_t idx = gy * gw + gx;
            if (!g_mask[idx])
                continue;
            uint32_t sp = 0;
            g_stack[sp++] = idx;
            g_mask[idx] = 0;
            uint32_t n = 0;
            uint32_t minx = 0xffffffffu, maxx = 0, miny = 0xffffffffu, maxy = 0;
            while (sp) {
                uint32_t cur = g_stack[--sp];
                uint32_t cy = cur / gw, cx = cur % gw;
                uint32_t ix = wx + cx * sx;
                uint32_t iy = wy + cy * sy;
                n++;
                if (ix < minx) minx = ix;
                if (ix > maxx) maxx = ix;
                if (iy < miny) miny = iy;
                if (iy > maxy) maxy = iy;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (!dx && !dy)
                            continue;
                        int nx = (int)cx + dx, ny = (int)cy + dy;
                        if (nx < 0 || ny < 0 || nx >= (int)gw || ny >= (int)gh)
                            continue;
                        uint32_t nidx = (uint32_t)ny * gw + (uint32_t)nx;
                        if (g_mask[nidx]) {
                            g_mask[nidx] = 0;
                            g_stack[sp++] = nidx;
                        }
                    }
                }
            }
            uint32_t est_px = n * sx * sy;
            uint32_t bw = (maxx - minx) + sx;
            uint32_t bh = (maxy - miny) + sy;
            if (bw < 2 || bh < 2)
                continue;
            if (est_px < g_cfg.pix_min)
                continue;
            if ((uint64_t)est_px * 100u < (uint64_t)FILL_LO_PCT * bw * bh)
                continue;
            if (10u * bw < (uint32_t)ASPECT_LO_NUM * bh)
                continue;
            if (10u * bw > (uint32_t)ASPECT_HI_NUM * bh)
                continue;
            out->blobs++;
            if (est_px > best_px) {
                best_px = est_px;
                out->px = est_px;
                out->cx = (int32_t)(minx + bw / 2u);
                out->cy = (int32_t)(miny + bh / 2u);
            }
        }
    }

    out->t_us = (uint32_t)(mono_us() - t0);
}

/* ============================ 自检（RVV 路径 vs 标量路径） ============================ */

/*
 * 板上没有模拟器可跑 RVV，所以把「RVV 结果必须与标量逐位一致」做成开机自检：
 * 造一张含灰底/绿色方块/明暗渐变的 NV12 小图，两条路径各扫一遍，
 * 比较掩码、命中像素数、Y 统计、探针位置与颜色。结果写进日志第一条。
 */
#if defined(DET_HAVE_RVV)
#define SELFTEST_W 64
#define SELFTEST_H 48

static int det_selftest(void)
{
    static uint8_t y[SELFTEST_W * SELFTEST_H];
    static uint8_t uv[SELFTEST_W * SELFTEST_H / 2];
    static uint8_t mask_a[SELFTEST_W / 2 * SELFTEST_H / 2 + 64];
    static uint8_t mask_b[sizeof(mask_a)];

    for (int i = 0; i < SELFTEST_W * SELFTEST_H; ++i)
        y[i] = (uint8_t)(40 + (i % 97));                 /* 明暗渐变 */
    for (int i = 0; i < SELFTEST_W * SELFTEST_H / 2; i += 2) {
        uv[i] = 128;                                     /* 灰底 U */
        uv[i + 1] = 128;                                 /* 灰底 V */
    }
    /* 中间放 16x16 的绿色块：BT.601 视频范围下 RGB(0,200,0) -> YUV(约 105,74,106) */
    for (int yy = 16; yy < 32; ++yy) {
        for (int xx = 24; xx < 40; ++xx) {
            y[yy * SELFTEST_W + xx] = 105;
            int uvo = (yy / 2) * SELFTEST_W + (xx / 2) * 2;
            uv[uvo] = 74;
            uv[uvo + 1] = 106;
        }
    }

    frame_view_t f;
    f.y = y;
    f.uv = uv;
    f.stride = SELFTEST_W;
    f.width = SELFTEST_W;
    f.height = SELFTEST_H;
    f.seq = 0;

    uint32_t sx = 2, sy = 2;
    uint32_t gw = SELFTEST_W / sx;
    uint32_t gh = SELFTEST_H / sy;

    scan_acc_t a, b;
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    a.probe_best = b.probe_best = -1;
    a.y_min = b.y_min = 255;

    scan_window(&f, 0, 0, SELFTEST_H, sx, sy, mask_a, gw, &a, 1);
    scan_window(&f, 0, 0, SELFTEST_H, sx, sy, mask_b, gw, &b, 0);

    int mismatch = 0;
    for (uint32_t i = 0; i < gw * gh; ++i)
        if (mask_a[i] != mask_b[i])
            mismatch++;
    int same = (mismatch == 0 && a.green == b.green && a.y_min == b.y_min &&
                a.y_max == b.y_max && a.y_sum == b.y_sum &&
                a.probe_x == b.probe_x && a.probe_y == b.probe_y &&
                a.probe_r == b.probe_r && a.probe_g == b.probe_g && a.probe_b == b.probe_b);

    app_log(APP_NAME_STR ": selftest rvv-vs-scalar %s (mask_mismatch=%d green=%u/%u "
            "probe=(%d,%d) rgb=(%d,%d,%d) y=[%d,%d])\n",
            same ? "PASS" : "FAIL", mismatch, a.green, b.green,
            a.probe_x, a.probe_y, a.probe_r, a.probe_g, a.probe_b, a.y_min, a.y_max);
    return same ? 0 : -1;
}
#endif /* DET_HAVE_RVV */

/* ============================ 算力微基准 ============================ */

/*
 * 开机自测一次「全画面 step2 的纯扫描耗时」：标量 vs RVV。
 * 用的是普通内存（cached）里的合成帧，因此量到的是**纯算力**；
 * 板端 status 里的 det= 是真实帧（含内存访问）的耗时，两者对照就能区分
 * 「算力不够」还是「内存访问慢」——上一轮 det=31ms 就是后者（uncached 逐样本读）。
 */
#if defined(DET_HAVE_RVV)
static void det_bench(void)
{
    const uint32_t w = 640, h = 480, sx = 2, sy = 2;
    uint32_t gw = w / sx, gh = h / sy;
    const int reps = 20;

    uint8_t *y = (uint8_t *)malloc((size_t)w * h);
    uint8_t *uv = (uint8_t *)malloc((size_t)w * h / 2);
    uint8_t *mask = (uint8_t *)malloc(gw * gh);
    if (!y || !uv || !mask) {
        free(y); free(uv); free(mask);
        return;
    }
    for (uint32_t i = 0; i < w * h; ++i)
        y[i] = (uint8_t)(40 + (i % 97));
    for (uint32_t i = 0; i + 1 < w * h / 2; i += 2) {
        uv[i] = 128;
        uv[i + 1] = 128;
    }
    for (uint32_t yy = 200; yy < 300; ++yy)
        for (uint32_t xx = 280; xx < 380; ++xx) {
            y[(size_t)yy * w + xx] = 105;
            size_t uvo = (size_t)(yy / 2) * w + (xx / 2) * 2;
            uv[uvo] = 74;
            uv[uvo + 1] = 106;
        }

    frame_view_t f;
    memset(&f, 0, sizeof(f));
    f.y = y;
    f.uv = uv;
    f.stride = w;
    f.width = w;
    f.height = h;

    scan_acc_t a;
    uint64_t t0, t1, scalar_us, rvv_us;

    t0 = mono_us();
    for (int r = 0; r < reps; ++r) {
        memset(&a, 0, sizeof(a));
        a.probe_best = -1;
        a.y_min = 255;
        scan_window(&f, 0, 0, h, sx, sy, mask, gw, &a, 0);
    }
    t1 = mono_us();
    scalar_us = (t1 - t0) / reps;

    t0 = mono_us();
    for (int r = 0; r < reps; ++r) {
        memset(&a, 0, sizeof(a));
        a.probe_best = -1;
        a.y_min = 255;
        scan_window(&f, 0, 0, h, sx, sy, mask, gw, &a, 1);
    }
    t1 = mono_us();
    rvv_us = (t1 - t0) / reps;

    app_log(APP_NAME_STR ": bench 全画面%ux%u step%u 纯扫描(cached 合成帧): "
            "scalar=%lluus rvv=%lluus (rvv/scalar=%llu%%)\n",
            w, h, sx, (unsigned long long)scalar_us, (unsigned long long)rvv_us,
            (unsigned long long)(scalar_us ? rvv_us * 100 / scalar_us : 0));

    free(y);
    free(uv);
    free(mask);
}
#endif

/* ============================ 初始化 ============================ */

static int detect_color_init(void)
{
    color_lut_build();
    uint32_t gw = (g_cfg.vis_w + g_cfg.step_x - 1) / g_cfg.step_x;
    uint32_t gh = (g_cfg.vis_h + g_cfg.step_y - 1) / g_cfg.step_y;
    if (scratch_ensure(gw * gh) != 0) {
        app_log(APP_NAME_STR ": detector scratch alloc failed (%u cells)\n", gw * gh);
        return -1;
    }
    app_log(APP_NAME_STR ": detector=color lab(L>=%d,A<=%d,B>=%d) "
            "roi=(%u,%u,%u,%u) step=(%u,%u) pix_min=%u %s\n",
            g_cfg.lab_l_min, g_cfg.lab_a_max, g_cfg.lab_b_min,
            g_cfg.roi_x, g_cfg.roi_y, g_cfg.roi_w, g_cfg.roi_h,
            g_cfg.step_x, g_cfg.step_y, g_cfg.pix_min,
#if defined(DET_HAVE_RVV)
            "[RVV on]"
#else
            "[scalar]"
#endif
    );
#if defined(DET_HAVE_RVV)
    det_selftest();
    det_bench();
#endif
    return 0;
}

static void detect_color_deinit(void)
{
    free(g_mask);
    free(g_stack);
    g_mask = NULL;
    g_stack = NULL;
    g_cells = 0;
}

static const detector_t g_detector_color = {
    .name = "color",
    .init = detect_color_init,
    .run = detect_color_run,
    .submit = NULL,        /* 同步后端 */
    .collect = NULL,
    .stats_line = NULL,
    .deinit = detect_color_deinit,
};

const detector_t *detector_color(void)
{
    return &g_detector_color;
}

const detector_t *detector_get(const char *name)
{
    if (name && strcmp(name, "kpu") == 0) {
#ifdef GREEN_LED_WITH_KPU
        return detector_kpu();
#else
        app_log(APP_NAME_STR ": 本二进制未编译 KPU 后端（用 green_led_ai_kpu 目标，"
                "见 apps/green_led_ai/README.md），回退 color\n");
#endif
    }
    return &g_detector_color;
}
