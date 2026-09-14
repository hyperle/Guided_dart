/*
 * green_led_rtos —— VICAP 唯一持有者 + 绿灯识别（CIELAB）+ 默认常录
 *
 * 管线（对齐板上已被验证过的组合）：
 *   sensor(gc2093) 1920x1080@30
 *     -> VICAP OFFLINE 采集 -> CHN0 缩放到 640x480 NV12
 *          |- 识别线程: NV12 -> RGB -> CIELAB 阈值 -> blob 筛选 -> 绿灯重心
 *          |- 录像模块(recorder.c): H.264 落盘 + 逐帧 CSV(帧号/结果/时间戳)
 *
 * 本程序是 VICAP 的唯一持有者，不依赖任何 USB 回传；运行情况完全通过下面
 * 三份板载产物判断（本文件所有关键判断都写日志，便于离线复核）：
 *   /sdcard/app/logs/green_led_rtos.log        识别与管线状态（每 status-period 一行）
 *   /sdcard/app/logs/green_led_recorder.log    录像会话统计（每段开始/结束各一行）
 *   /sdcard/app/recording/rec_XXXX.h264/.csv   画面 + 逐帧识别结果
 *
 * 为什么这样配（都是踩过的坑，不要随手改回去）：
 *
 * 1) 探测请求必须是 1920x1080@30。
 *    kd_mpi_sensor_adapt_get() 是三级匹配：精确(宽+高+fps) -> 同宽高 -> 找更大
 *    分辨率，且候选表按 fps 降序。gc2093 在 CSI2 只有 1920x1080@30/60 与
 *    1280x960@90 / 1280x720@90。请求 640x480@120 时三级都匹配不到 640x480，
 *    会**静默**落到 1280x960@90：实际分辨率与帧率和代码里的常量全都对不上，
 *    管线跑在没人验证过的组合上。请求 1920x1080@30 才精确命中。
 *
 * 2) 采集用 VICAP_WORK_OFFLINE_MODE，缩放交给 CHN0。
 *    官方 sample_uvc_dev_vicap（1080p->小图预览/编码的用例）与 CanMV 运行时
 *    都是这么做的。官方 sample_vicap_sensor 那种 ONLINE + CHN0 不缩放的写法
 *    只适合全尺寸直出。
 *
 * 3) AE 关（固定曝光，防 LED 过曝/AGC 反扑），AWB 开（颜色阈值就是在自动白平衡
 *    下标的），DNR3 开（厂商默认）。曝光/增益在 start_stream 之后用 sensor ioctl
 *    写死，与 Python 原型 scripts/green_led_test_k230.py 的 apply_exposure() 一致。
 *
 * 4) 识别用 CIELAB 阈值，数值直接沿用 Python 原型的 TH_GREEN：
 *        L in [12,100], A <= -20, B >= 8
 *    （CanMV 的 find_blobs 就是用 sRGB->CIELAB(D65) 的查表，所以这里的标准
 *      Lab 公式与那套阈值同一个色彩空间。）再用与原型相同的 blob 筛选：
 *    最小像素数 / 填充率 / 长宽比，最后取最大 blob 的包围盒中心。
 *
 * 5) 采集线程"最新帧优先、绝不阻塞"：信箱被占就把刚 dump 的帧立刻 release 并
 *    计数，绝不等待识别线程。否则识别慢一拍时会一直攥着 VICAP buffer，很快
 *    把 6 个 buffer 耗光，dump 超时/丢帧刷屏。录像在投递前先喂，所以识别丢帧
 *    也不影响录像的连续性。
 *
 * 6) 日志里能看到"实际命中哪个 sensor 模式、phys1-phys0 是否等于 stride*height、
 *    识别帧率/命中像素数/最绿像素的 LAB"。调阈值时直接看日志里的
 *    probe=rgb(...) lab(...)，不用再猜。
 */
#define main vendor_sample_main
#include "/mnt/mydata/kRTOSSDK/src/rtsmart/examples/mpp/sample_vicap_sensor/sample_vicap_sensor.c"
#undef main

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "mpi_sensor_api.h"
#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpp_mem.h"
#include "recorder.h"

/* ============================ 常量 ============================ */

#define DEV       VICAP_DEV_ID_0
#define LOG_PATH  "/sdcard/app/logs/green_led_rtos.log"

/* 采集侧：必须是适配表里精确命中的模式（见文件头说明 1） */
#define ISP_WIDTH   1920
#define ISP_HEIGHT  1080
#define ISP_FPS     30

/* 识别/录像侧输出尺寸（CHN0 缩放） */
#define OUT_WIDTH   640
#define OUT_HEIGHT  480

/* blob 形状筛选：与 Python 原型的 FILL_LO / ASPECT_LO~HI 一致 */
#define FILL_LO_PCT   55   /* px*100 >= 55*w*h */
#define ASPECT_LO_NUM 7    /* 10*w >= 7*h  */
#define ASPECT_HI_NUM 14   /* 10*w <= 14*h */

/* cbrt 查表粒度（LAB 的 f(t)） */
#define CBRT_LUT_N 1024

/* 前若干帧逐帧打印 trace 行（fsync 落盘），用于定位"板子在哪一步被搞挂"。
 * 每次 panic 重启后，日志里最后一条 trace 就是凶手那一步。 */
#define TRACE_FRAMES 5

/* ============================ 配置 ============================ */

typedef struct {
    int      csi;
    int      probe_fps;
    uint32_t out_w, out_h;
    uint32_t roi_x, roi_y, roi_w, roi_h;  /* roi_w 或 roi_h 为 0 => 全画面 */
    uint32_t step_x, step_y;
    int      lab_l_min, lab_a_max, lab_b_min;
    uint32_t pix_min;                      /* blob 最小图像像素数 */
    int      miss_full_scan;               /* 连续丢失 N 帧后整幅重扫；0=关闭 */
    int      exposure_us;                  /* 固定曝光(us)；0=不设 */
    int      fix_gain;                     /* 1=增益压到最低 */
    int      ae_enable;                    /* 默认 0：关 AE（固定曝光） */
    int      record;                       /* 1=开机常录 */
    uint32_t bitrate_kbps;
    const char *rec_dir;
    int      status_period_s;              /* 状态日志周期(秒) */
    int      dump_every_s;                 /* 每隔几秒导出整帧 PPM；0=关闭 */
    const char *dump_dir;                  /* 导出目录 */
} cfg_t;

static cfg_t g_cfg = {
    .csi            = 2,
    .probe_fps      = ISP_FPS,
    .out_w          = OUT_WIDTH,
    .out_h          = OUT_HEIGHT,
    .roi_x          = 160,
    .roi_y          = 120,
    .roi_w          = 320,
    .roi_h          = 240,
    .step_x         = 2,
    .step_y         = 2,
    .lab_l_min      = 12,
    .lab_a_max      = -20,
    .lab_b_min      = 8,
    .pix_min        = 50,
    .miss_full_scan = 3,
    .exposure_us    = 200,
    .fix_gain       = 1,
    .ae_enable      = 0,
    .record         = 1,
    .bitrate_kbps   = 12000,
    .rec_dir        = REC_DEFAULT_DIR,
    .status_period_s = 2,
    .dump_every_s   = 0,
    .dump_dir       = "/sdcard/app/recording",
};

/* ============================ 运行状态 ============================ */

static volatile int app_running = 1;

static pthread_mutex_t frame_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  frame_ready = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t result_lock = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    k_video_frame_info frame;
    uint64_t seq;
    uint64_t mono_us;
} shared_item_t;

static shared_item_t shared_frame;
static int      have_shared_frame = 0;   /* 信箱内有未消费帧 */
static uint64_t frame_seq_counter = 0;
static uint64_t frames_dumped = 0;
static uint64_t frames_dropped = 0;      /* 信箱被占而丢弃（识别跟不上） */
static uint64_t dump_fail = 0;

typedef struct {
    int32_t  center_x;
    int32_t  center_y;
    uint32_t roi_x, roi_y, roi_w, roi_h;
    uint32_t fps;
    uint64_t seq;
} detection_result_t;

static detection_result_t result = {
    .center_x = -1, .center_y = -1,
    .roi_x = 160, .roi_y = 120, .roi_w = 320, .roi_h = 240,
    .fps = 0, .seq = 0
};

/* ============================ 日志 ============================ */

static void log_line(const char *fmt, ...)
{
    static int dir_ok;
    if (!dir_ok) {
        mkdir("/sdcard/app/logs", 0777);
        dir_ok = 1;
    }
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if (n > (int)sizeof(buf) - 1)
        n = (int)sizeof(buf) - 1;

    /* 同时进 stdout（launcher 会重定向到 launcher-child.log，msh 手动跑也直接可见） */
    fwrite(buf, 1, (size_t)n, stdout);
    fflush(stdout);

    int fd = open(LOG_PATH, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0)
        return;
    ssize_t w = write(fd, buf, (size_t)n);
    fsync(fd);
    close(fd);
    (void)w;
}

/* ============================ 时间 ============================ */

static uint64_t mono_us_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/* ============================ 颜色 ============================ */

/* sRGB 反 gamma 查表 + f(t)=cbrt(t) 查表：避免每像素 powf/cbrtf */
static float g_srgb_lin[256];
static float g_cbrt_lut[CBRT_LUT_N + 1];

static void color_tables_init(void)
{
    for (int i = 0; i < 256; ++i) {
        float c = (float)i / 255.0f;
        g_srgb_lin[i] = (c <= 0.04045f)
                            ? (c / 12.92f)
                            : powf((c + 0.055f) / 1.055f, 2.4f);
    }
    for (int i = 0; i <= CBRT_LUT_N; ++i)
        g_cbrt_lut[i] = cbrtf((float)i / (float)CBRT_LUT_N);
}

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

/* CIELAB 的 f(t)：标准分段函数（暗部走线性段） */
static inline float lab_f(float t)
{
    if (t <= 0.008856f)
        return 7.787f * t + 16.0f / 116.0f;
    return fast_cbrt(t);
}

/* sRGB -> RGB565 量化：CanMV 的 find_blobs 就是在 RGB565 上查 lab_table，
 * 这里先做同样的量化，阈值口径才和 Python 原型完全一致。 */
static inline int quant565_r(int v) { return (v & 0xF8) | (v >> 5); }
static inline int quant565_g(int v) { return (v & 0xFC) | (v >> 6); }
static inline int quant565_b(int v) { return (v & 0xF8) | (v >> 5); }

/* sRGB(D65) -> CIELAB，四舍五入到整数（与 CanMV 的 lab_table 同口径） */
static inline void rgb_to_lab(int r, int g, int b, int *L, int *A, int *B)
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

/* NV12(BT.601, 视频范围) -> RGB888 */
static inline void nv12_to_rgb(uint8_t yy, uint8_t uu, uint8_t vv,
                               int *r, int *g, int *b)
{
    int c = (int)yy - 16;
    int d = (int)uu - 128;
    int e = (int)vv - 128;
    int rr = (298 * c + 409 * e + 128) >> 8;
    int gg = (298 * c - 100 * d - 208 * e + 128) >> 8;
    int bb = (298 * c + 516 * d + 128) >> 8;
    *r = rr < 0 ? 0 : (rr > 255 ? 255 : rr);
    *g = gg < 0 ? 0 : (gg > 255 ? 255 : gg);
    *b = bb < 0 ? 0 : (bb > 255 ? 255 : bb);
}

/* ============================ 平面映射 ============================ */

/*
 * 注意：mmap 的长度必须覆盖**实际会读到的全部字节**。
 * NV12 在 K230 上是连续布局（phys1 == phys0 + stride*height），UV 指针就等于
 * y + stride*height；如果只按 Y 平面大小映射，读 UV 时就会越过映射边界，
 * RT-Smart 上这不是"进程收到 SIGSEGV"，而是直接把内核打挂重启（实测 29 次
 * 重启循环就是这么来的）。所以连续布局时一次映射整帧；只有分段布局才分开映射。
 */
typedef struct {
    uint8_t *y;
    uint8_t *uv;
    uint8_t *uv_map;      /* 非 NULL 表示 UV 是单独映射出来的 */
    uint32_t ysize;       /* Y 平面大小 */
    uint32_t usize;       /* UV 平面大小 */
    uint32_t y_map_len;   /* phys_addr[0] 那次映射的长度 */
    uint32_t uv_map_len;  /* phys_addr[1] 那次映射的长度 */
} plane_map_t;

static int plane_map(const k_video_frame_info *f, uint32_t stride, uint32_t height,
                     plane_map_t *m)
{
    memset(m, 0, sizeof(*m));
    uint32_t ysize = stride * height;
    uint32_t usize = ysize / 2;
    m->ysize = ysize;
    m->usize = usize;

    uint64_t y_phys = f->v_frame.phys_addr[0];
    uint64_t uv_phys = f->v_frame.phys_addr[1];
    int contiguous = (!uv_phys || uv_phys == y_phys + ysize);

    uint32_t y_len = contiguous ? (ysize + usize) : ysize;
    uint8_t *y = (uint8_t *)mpp_mem_map(y_phys, y_len);
    if (!y)
        return -1;
    m->y = y;
    m->y_map_len = y_len;

    if (contiguous) {
        m->uv = y + ysize;   /* 与 Y 同一次映射之内，不会越界 */
        return 0;
    }
    uint8_t *uv = (uint8_t *)mpp_mem_map(uv_phys, usize);
    if (uv) {
        m->uv = uv;
        m->uv_map = uv;
        m->uv_map_len = usize;
        return 0;
    }
    m->uv = y + ysize;       /* 退化为连续假设（此时 UV 可能读不到，但不会崩） */
    return 0;
}

static void plane_unmap(plane_map_t *m)
{
    /* 必须按 map 时的长度 unmap，长度不匹配同样会打挂内核 */
    if (m->uv_map)
        mpp_mem_unmap(m->uv_map, m->uv_map_len);
    if (m->y)
        mpp_mem_unmap(m->y, m->y_map_len);
    memset(m, 0, sizeof(*m));
}

/* ============================ 识别 ============================ */

typedef struct {
    uint32_t cells;
    uint8_t *mask;
    uint32_t *stack;
} scratch_t;

static scratch_t g_scratch;

static int scratch_init(uint32_t cells)
{
    g_scratch.mask = (uint8_t *)malloc(cells);
    g_scratch.stack = (uint32_t *)malloc((size_t)cells * sizeof(uint32_t));
    if (!g_scratch.mask || !g_scratch.stack)
        return -1;
    g_scratch.cells = cells;
    return 0;
}

typedef struct {
    int32_t  cx, cy;        /* 最大 blob 的包围盒中心；-1 = 未命中 */
    uint32_t px;            /* 该 blob 的估算图像像素数 */
    uint32_t blobs;         /* 通过形状筛选的 blob 数 */
    uint32_t green_px;      /* 落在阈值内的采样点数 */
    uint32_t roi_x, roi_y, roi_w, roi_h;   /* 本次扫描窗口 */
    int      full_scan;
    /* 探针：窗口内"最绿"像素（用于对照日志调阈值） */
    int      probe_ok;
    int      probe_x, probe_y;
    int      probe_r, probe_g, probe_b;
    int      probe_L, probe_A, probe_B;
    /* 扫描窗口内的 Y（亮度）统计：判断"画面是不是全黑/曝光够不够" */
    int      y_min, y_max;
    uint32_t y_mean;
} detect_out_t;

/*
 * 扫描窗口 + 采样网格上的连通域（8 邻域），复刻 Python 原型 find_blobs 的筛选：
 *   像素数 >= pix_min、填充率 >= 55%、长宽比 0.7~1.4，取最大者，中心 = 包围盒中心。
 * mask 就是"未被访问"表：BFS 访问后清零，省一个 seen 数组。
 */
static void detect_run(const uint8_t *y, const uint8_t *uv, uint32_t stride,
                       uint32_t width, uint32_t height, int full_scan,
                       detect_out_t *out)
{
    memset(out, 0, sizeof(*out));
    out->cx = -1;
    out->cy = -1;

    uint32_t wx, wy, ww, wh;
    if (full_scan || g_cfg.roi_w == 0 || g_cfg.roi_h == 0) {
        wx = 0; wy = 0; ww = width; wh = height;
    } else {
        wx = g_cfg.roi_x; wy = g_cfg.roi_y;
        ww = g_cfg.roi_w; wh = g_cfg.roi_h;
        if (wx >= width) wx = 0;
        if (wy >= height) wy = 0;
        if (ww > width - wx) ww = width - wx;
        if (wh > height - wy) wh = height - wy;
    }
    out->roi_x = wx; out->roi_y = wy; out->roi_w = ww; out->roi_h = wh;
    out->full_scan = full_scan;

    uint32_t sx = g_cfg.step_x ? g_cfg.step_x : 1;
    uint32_t sy = g_cfg.step_y ? g_cfg.step_y : 1;
    uint32_t gw = (ww + sx - 1) / sx;
    uint32_t gh = (wh + sy - 1) / sy;
    if ((uint64_t)gw * gh > g_scratch.cells)
        return;   /* 不应发生：容量按全画面 + 步长算过 */

    /* --- 阈值掩码 + 探针 + 亮度统计 --- */
    int probe_best = -1;
    int ymin = 255, ymax = 0;
    uint64_t ysum = 0;
    for (uint32_t gy = 0; gy < gh; ++gy) {
        uint32_t iy = wy + gy * sy;
        const uint8_t *yp = y + (size_t)iy * stride;
        const uint8_t *uvp = uv + (size_t)(iy >> 1) * stride;
        for (uint32_t gx = 0; gx < gw; ++gx) {
            uint32_t ix = wx + gx * sx;
            uint32_t uvo = (ix >> 1) * 2u;
            int r, g, b, L, A, B;
            uint8_t yv = yp[ix];
            if (yv < ymin) ymin = yv;
            if (yv > ymax) ymax = yv;
            ysum += yv;
            nv12_to_rgb(yv, uvp[uvo], uvp[uvo + 1], &r, &g, &b);
            rgb_to_lab(r, g, b, &L, &A, &B);
            int pass = (L >= g_cfg.lab_l_min && L <= 100 &&
                        A <= g_cfg.lab_a_max && B >= g_cfg.lab_b_min);
            g_scratch.mask[gy * gw + gx] = pass ? 1 : 0;
            if (pass)
                out->green_px++;
            int gr = g - (r > b ? r : b);
            if (gr > probe_best) {
                probe_best = gr;
                out->probe_ok = 1;
                out->probe_x = (int)ix;
                out->probe_y = (int)iy;
                out->probe_r = r; out->probe_g = g; out->probe_b = b;
                out->probe_L = L; out->probe_A = A; out->probe_B = B;
            }
        }
    }
    out->y_min = ymin;
    out->y_max = ymax;
    out->y_mean = (gw && gh) ? (uint32_t)(ysum / (uint64_t)(gw * gh)) : 0;
    if (!out->green_px)
        return;

    /* --- 连通域（BFS，mask 访问后清零） --- */
    uint32_t best_px = 0;
    for (uint32_t gy = 0; gy < gh; ++gy) {
        for (uint32_t gx = 0; gx < gw; ++gx) {
            uint32_t idx = gy * gw + gx;
            if (!g_scratch.mask[idx])
                continue;
            uint32_t sp = 0;
            g_scratch.stack[sp++] = idx;
            g_scratch.mask[idx] = 0;
            uint32_t n = 0;
            uint32_t minx = 0xffffffffu, maxx = 0, miny = 0xffffffffu, maxy = 0;
            while (sp) {
                uint32_t cur = g_scratch.stack[--sp];
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
                        if (g_scratch.mask[nidx]) {
                            g_scratch.mask[nidx] = 0;
                            g_scratch.stack[sp++] = nidx;
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
}

/*
 * 把当前帧写成 PPM(P6) 落盘：不依赖录像/VENC，就能在主机上"看到画面"，
 * 用来判断曝光够不够、灯在不在视野里。文件路径会写进日志。
 */
static void dump_ppm(const char *dir, uint64_t seq,
                     const uint8_t *y, const uint8_t *uv, uint32_t stride,
                     uint32_t w, uint32_t h)
{
    char path[192];
    snprintf(path, sizeof(path), "%s/frame_%05llu.ppm", dir,
             (unsigned long long)(seq % 100000ull));

    if (w > OUT_WIDTH) w = OUT_WIDTH;          /* 只用静态缓冲，避免大块 malloc */
    size_t need = 15u + (size_t)w * h * 3u;
    uint8_t *buf = (uint8_t *)malloc(need);
    if (!buf) {
        log_line("green_led_rtos: dump %s failed (oom)\n", path);
        return;
    }
    int hl = snprintf((char *)buf, 32, "P6\n%u %u\n255\n", w, h);
    uint8_t *p = buf + hl;
    for (uint32_t yy = 0; yy < h; ++yy) {
        const uint8_t *yp = y + (size_t)yy * stride;
        const uint8_t *uvp = uv + (size_t)(yy >> 1) * stride;
        for (uint32_t xx = 0; xx < w; ++xx) {
            uint32_t uvo = (xx >> 1) * 2u;
            int r, g, b;
            nv12_to_rgb(yp[xx], uvp[uvo], uvp[uvo + 1], &r, &g, &b);
            *p++ = (uint8_t)r;
            *p++ = (uint8_t)g;
            *p++ = (uint8_t)b;
        }
    }
    /* 一次写完：SD/FAT 上按行写会非常慢（实测把识别压到 11fps） */
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        free(buf);
        log_line("green_led_rtos: dump %s failed\n", path);
        return;
    }
    size_t wr = fwrite(buf, 1, need, fp);
    fclose(fp);
    free(buf);
    log_line("green_led_rtos: dumped %s (%u bytes)\n", path, (unsigned)wr);
}

static void update_result(int32_t cx, int32_t cy, uint32_t fps, uint64_t seq,
                          const detect_out_t *d)
{
    detection_result_t next;
    next.center_x = cx;
    next.center_y = cy;
    next.roi_x = d->roi_x;
    next.roi_y = d->roi_y;
    next.roi_w = d->roi_w;
    next.roi_h = d->roi_h;
    next.fps = fps;
    next.seq = seq;
    pthread_mutex_lock(&result_lock);
    result = next;
    pthread_mutex_unlock(&result_lock);
}

/* ============================ 采集 ============================ */

static void *capture_thread(void *arg)
{
    (void)arg;
    int layout_logged = 0;
    while (app_running) {
        k_video_frame_info frame;
        k_s32 ret = kd_mpi_vicap_dump_frame(DEV, VICAP_CHN_ID_0, VICAP_DUMP_YUV,
                                            &frame, 200);
        if (ret != K_SUCCESS) {
            dump_fail++;
            if (dump_fail % 50u == 1u)
                log_line("green_led_rtos: vicap dump failed ret=%d (fail=%" PRIu64
                         ")\n", (int)ret, dump_fail);
            usleep(2000);
            continue;
        }
        frames_dumped++;
        uint32_t stride = frame.v_frame.stride[0] ? frame.v_frame.stride[0]
                                                  : frame.v_frame.width;
        uint64_t seq = ++frame_seq_counter;

        if (!layout_logged) {
            layout_logged = 1;
            log_line("green_led_rtos: frame %ux%u stride=%u fmt=%d "
                     "phys1-phys0=%" PRId64 " stride*h=%u\n",
                     frame.v_frame.width, frame.v_frame.height, stride,
                     (int)frame.v_frame.pixel_format,
                     (int64_t)(frame.v_frame.phys_addr[1] - frame.v_frame.phys_addr[0]),
                     stride * frame.v_frame.height);
        }

        /* 1) 先喂录像：保证录像是连续的（识别丢帧不影响录像） */
        int fed = -1;
        if (rec_is_active()) {
            if (seq <= TRACE_FRAMES)
                log_line("trace cap seq=%" PRIu64 " feed-begin\n", seq);
            fed = rec_feed_frame(&frame, seq, mono_us_now());
            if (seq <= TRACE_FRAMES)
                log_line("trace cap seq=%" PRIu64 " feed-done ret=%d\n", seq, fed);
        }

        /* 2) 最新帧优先投递，信箱满就丢本帧（绝不阻塞，避免拖死 VICAP） */
        int published = 0;
        pthread_mutex_lock(&frame_lock);
        if (!have_shared_frame) {
            shared_frame.frame = frame;
            shared_frame.seq = seq;
            shared_frame.mono_us = mono_us_now();
            have_shared_frame = 1;
            published = 1;
            pthread_cond_signal(&frame_ready);
        } else {
            frames_dropped++;
        }
        pthread_mutex_unlock(&frame_lock);

        if (seq <= TRACE_FRAMES)
            log_line("trace cap seq=%" PRIu64 " publish=%d\n", seq, published);

        if (!published) {
            kd_mpi_vicap_dump_release(DEV, VICAP_CHN_ID_0, &frame);
            if (seq <= TRACE_FRAMES)
                log_line("trace cap seq=%" PRIu64 " self-released\n", seq);
        }
    }
    return NULL;
}

/* ============================ 识别线程 ============================ */

static void *detect_thread(void *arg)
{
    (void)arg;
    struct timespec last;
    clock_gettime(CLOCK_MONOTONIC, &last);
    uint32_t frames = 0;
    uint32_t fps = 0;
    uint32_t misses = 0;
    int      found_prev = 0;
    uint64_t found_total = 0, lost_total = 0;

    while (app_running) {
        pthread_mutex_lock(&frame_lock);
        while (app_running && !have_shared_frame)
            pthread_cond_wait(&frame_ready, &frame_lock);
        if (!app_running) {
            pthread_mutex_unlock(&frame_lock);
            break;
        }
        k_video_frame_info frame = shared_frame.frame;
        uint64_t seq = shared_frame.seq;
        have_shared_frame = 0;
        pthread_mutex_unlock(&frame_lock);

        uint32_t stride = frame.v_frame.stride[0] ? frame.v_frame.stride[0]
                                                  : frame.v_frame.width;
        uint32_t width = frame.v_frame.width;
        uint32_t height = frame.v_frame.height;

        int full_scan = (g_cfg.miss_full_scan > 0 &&
                         misses >= (uint32_t)g_cfg.miss_full_scan);
        detect_out_t d;
        memset(&d, 0, sizeof(d));
        plane_map_t pm;
        if (seq <= TRACE_FRAMES)
            log_line("trace det seq=%" PRIu64 " take (scan=%s)\n",
                     seq, full_scan ? "full" : "roi");
        mpp_mem_begin();   /* 映射->扫描->解映射整段串行，见 mpp_mem.h */
        if (plane_map(&frame, stride, height, &pm) == 0) {
            if (seq <= TRACE_FRAMES)
                log_line("trace det seq=%" PRIu64 " mapped y=%p uv=%p len=%u/%u\n",
                         seq, (void *)pm.y, (void *)pm.uv,
                         pm.y_map_len, pm.uv_map_len);
            detect_run(pm.y, pm.uv, stride, width, height, full_scan, &d);
            if (seq <= TRACE_FRAMES)
                log_line("trace det seq=%" PRIu64 " detect-done cx=%d px=%u\n",
                         seq, d.cx, d.px);
            if (g_cfg.dump_every_s > 0) {
                struct timespec dn;
                clock_gettime(CLOCK_MONOTONIC, &dn);
                static time_t last_dump;
                if (dn.tv_sec - last_dump >= g_cfg.dump_every_s) {
                    last_dump = dn.tv_sec;
                    dump_ppm(g_cfg.dump_dir, seq, pm.y, pm.uv, stride,
                             width, height);
                }
            }
            plane_unmap(&pm);
            if (seq <= TRACE_FRAMES)
                log_line("trace det seq=%" PRIu64 " unmapped\n", seq);
        } else {
            log_line("green_led_rtos: frame mmap failed\n");
        }
        mpp_mem_end();

        if (d.cx >= 0) {
            misses = 0;
            found_total++;
        } else {
            misses++;
            lost_total++;
        }
        frames++;

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double dt = (double)(now.tv_sec - last.tv_sec) +
                    (double)(now.tv_nsec - last.tv_nsec) / 1e9;

        update_result(d.cx, d.cy, fps, seq, &d);
        rec_note_result(seq, d.cx, d.cy, fps);

        int found = (d.cx >= 0);
        if (found != found_prev) {
            if (found)
                log_line("green_led_rtos: detect FOUND center=(%d,%d) px=%u "
                         "blobs=%u scan=%s (lost %u frames before)\n",
                         d.cx, d.cy, d.px, d.blobs,
                         full_scan ? "full" : "roi", misses ? misses - 1 : 0);
            else
                log_line("green_led_rtos: detect LOST (green_px=%u probe_lab=(%d,%d,%d)"
                         " probe_rgb=(%d,%d,%d))\n",
                         d.green_px, d.probe_L, d.probe_A, d.probe_B,
                         d.probe_r, d.probe_g, d.probe_b);
            found_prev = found;
        }

        if (dt >= (double)g_cfg.status_period_s) {
            fps = (uint32_t)((double)frames / dt + 0.5);
            frames = 0;
            last = now;
            recorder_stats_t rs;
            rec_get_stats(&rs);
            uint64_t maps = 0, waits = 0;
            mpp_mem_stats(&maps, &waits);
            log_line("green_led_rtos: status det_fps=%u center=(%d,%d) px=%u blobs=%u "
                     "green_px=%u y=[%d,%d,%u] scan=%s miss=%u found=%" PRIu64 "/%" PRIu64
                     " | dumped=%" PRIu64 " drop=%" PRIu64 " dump_fail=%" PRIu64
                     " | rec active=%d sess=%u in=%u sent=%u drop=%u written=%u "
                     "bytes=%" PRIu64 " | maps=%" PRIu64 " contended=%" PRIu64 "\n",
                     fps, d.cx, d.cy, d.px, d.blobs, d.green_px,
                     d.y_min, d.y_max, d.y_mean,
                     full_scan ? "full" : "roi", misses, found_total, lost_total,
                     frames_dumped, frames_dropped, dump_fail,
                     rec_is_active(), rs.sessions, rs.frames_in, rs.frames_sent,
                     rs.frames_dropped, rs.frames_written, rs.bytes, maps, waits);
            log_line("green_led_rtos: probe rgb=(%d,%d,%d) lab=(%d,%d,%d) at (%d,%d) "
                     "thr L>=%d A<=%d B>=%d step=(%u,%u) roi=(%u,%u,%u,%u)\n",
                     d.probe_r, d.probe_g, d.probe_b, d.probe_L, d.probe_A, d.probe_B,
                     d.probe_x, d.probe_y,
                     g_cfg.lab_l_min, g_cfg.lab_a_max, g_cfg.lab_b_min,
                     g_cfg.step_x, g_cfg.step_y,
                     g_cfg.roi_x, g_cfg.roi_y, g_cfg.roi_w, g_cfg.roi_h);
        }

        kd_mpi_vicap_dump_release(DEV, VICAP_CHN_ID_0, &frame);
        if (seq <= TRACE_FRAMES)
            log_line("trace det seq=%" PRIu64 " released\n", seq);
    }
    return NULL;
}

/* ============================ 信号 ============================ */

static void green_signal(int sig)
{
    (void)sig;
    app_running = 0;
    pthread_cond_broadcast(&frame_ready);
}

static void rec_start_signal(int sig)
{
    (void)sig;
    rec_signal_start();
}

static void rec_stop_signal(int sig)
{
    (void)sig;
    rec_signal_stop();
}

/* ============================ VICAP ============================ */

static k_vicap_sensor_attr g_sensor_attr;

static k_s32 green_vicap_init(uint32_t acq_w, uint32_t acq_h)
{
    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;
    memset(&dev_attr, 0, sizeof(dev_attr));
    memset(&chn_attr, 0, sizeof(chn_attr));

    dev_attr.acq_win.width = acq_w;
    dev_attr.acq_win.height = acq_h;
    dev_attr.mode = VICAP_WORK_OFFLINE_MODE;   /* 见文件头说明 2 */
    dev_attr.buffer_num = 6;
    dev_attr.buffer_size = VB_ALIGN_UP((uint64_t)acq_w * acq_h * 2, 4096);
    dev_attr.buffer_pool_id = VB_INVALID_POOLID;
    dev_attr.pipe_ctrl.data = 0xFFFFFFFF;
    dev_attr.pipe_ctrl.bits.ae_enable = g_cfg.ae_enable ? K_TRUE : K_FALSE;
    dev_attr.pipe_ctrl.bits.awb_enable = K_TRUE;    /* 阈值在 AWB 下标的 */
    dev_attr.pipe_ctrl.bits.ahdr_enable = K_FALSE;
    dev_attr.pipe_ctrl.bits.dnr3_enable = K_TRUE;
    memcpy(&dev_attr.sensor_info, &g_sensor_info, sizeof(k_vicap_sensor_info));

    k_s32 ret = kd_mpi_vicap_set_dev_attr(DEV, dev_attr);
    if (ret != K_SUCCESS) {
        log_line("green_led_rtos: set_dev_attr failed ret=%d\n", (int)ret);
        return ret;
    }

    chn_attr.out_win.width = g_cfg.out_w;
    chn_attr.out_win.height = g_cfg.out_h;
    chn_attr.crop_win = dev_attr.acq_win;
    chn_attr.scale_win = chn_attr.out_win;
    chn_attr.crop_enable = K_FALSE;
    chn_attr.scale_enable = (g_cfg.out_w != acq_w || g_cfg.out_h != acq_h) ? K_TRUE
                                                                          : K_FALSE;
    chn_attr.chn_enable = K_TRUE;
    chn_attr.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    chn_attr.buffer_num = 6;
    chn_attr.buffer_size = VB_ALIGN_UP((uint64_t)g_cfg.out_w * g_cfg.out_h * 3 / 2, 4096);
    chn_attr.alignment = 12;
    chn_attr.buffer_pool_id = VB_INVALID_POOLID;

    ret = kd_mpi_vicap_set_chn_attr(DEV, VICAP_CHN_ID_0, chn_attr);
    if (ret != K_SUCCESS) {
        log_line("green_led_rtos: set_chn_attr failed ret=%d\n", (int)ret);
        return ret;
    }
    ret = kd_mpi_vicap_init(DEV);
    if (ret != K_SUCCESS)
        log_line("green_led_rtos: vicap_init failed ret=%d\n", (int)ret);
    return ret;
}

/* 固定曝光 / 最低增益：必须在 start_stream 之后设置（与官方示例一致） */
static void apply_sensor_exposure(void)
{
    if (g_cfg.ae_enable) {
        log_line("green_led_rtos: sensor AE enabled; skip fixed exposure/gain\n");
        return;
    }
    if (g_sensor_attr.sensor_fd < 0) {
        log_line("green_led_rtos: sensor fd unavailable; skip exposure/gain\n");
        return;
    }
    if (g_cfg.exposure_us > 0) {
        k_sensor_exposure_time_range range;
        memset(&range, 0, sizeof(range));
        if (kd_mpi_sensor_get_exposure_time_range(g_sensor_attr.sensor_fd, &range) == 0) {
            float lo = range.min_intg_time_us / 1000000.0f;
            float hi = range.max_intg_time_us / 1000000.0f;
            float want = (float)g_cfg.exposure_us / 1000000.0f;
            if (want < lo) want = lo;
            if (want > hi) want = hi;
            k_sensor_intg_time t;
            memset(&t, 0, sizeof(t));
            t.intg_time[0] = want;
            int rc = (int)kd_mpi_sensor_intg_time_set(g_sensor_attr.sensor_fd, t);
            log_line("green_led_rtos: exposure fixed %.0f us (range %.0f-%.0f us) rc=%d\n",
                     want * 1000000.0f, range.min_intg_time_us, range.max_intg_time_us, rc);
        } else {
            log_line("green_led_rtos: get exposure range failed; keep default\n");
        }
    }
    if (g_cfg.fix_gain) {
        k_sensor_gain_info gr;
        memset(&gr, 0, sizeof(gr));
        if (kd_mpi_sensor_get_gain_range(g_sensor_attr.sensor_fd, &gr) == 0) {
            k_sensor_gain gain;
            memset(&gain, 0, sizeof(gain));
            gain.gain[0] = gr.min;
            int rc = (int)kd_mpi_sensor_again_set(g_sensor_attr.sensor_fd, gain);
            log_line("green_led_rtos: gain fixed %.3f (range %.3f-%.3f) rc=%d\n",
                     gr.min, gr.min, gr.max, rc);
        } else {
            log_line("green_led_rtos: get gain range failed; keep default\n");
        }
    }
}

/* ============================ 命令行 ============================ */

static int parse_pair(const char *s, uint32_t *a, uint32_t *b)
{
    char *end = NULL;
    unsigned long v1 = strtoul(s, &end, 0);
    if (!end || (*end != 'x' && *end != 'X' && *end != ',' && *end != ':'))
        return -1;
    unsigned long v2 = strtoul(end + 1, NULL, 0);
    *a = (uint32_t)v1;
    *b = (uint32_t)v2;
    return 0;
}

static int parse_triple_i(const char *s, int *a, int *b, int *c)
{
    char *end = NULL;
    long v1 = strtol(s, &end, 0);
    if (!end || (*end != ',' && *end != ':' && *end != 'x'))
        return -1;
    char *end2 = NULL;
    long v2 = strtol(end + 1, &end2, 0);
    if (!end2 || (*end2 != ',' && *end2 != ':' && *end2 != 'x'))
        return -1;
    long v3 = strtol(end2 + 1, NULL, 0);
    *a = (int)v1;
    *b = (int)v2;
    *c = (int)v3;
    return 0;
}

static int parse_quad(const char *s, uint32_t *a, uint32_t *b, uint32_t *c, uint32_t *d)
{
    char *end = NULL;
    unsigned long v1 = strtoul(s, &end, 0);
    if (!end || (*end != ',' && *end != ':' && *end != 'x'))
        return -1;
    char *e2 = NULL;
    unsigned long v2 = strtoul(end + 1, &e2, 0);
    if (!e2 || (*e2 != ',' && *e2 != ':' && *e2 != 'x'))
        return -1;
    char *e3 = NULL;
    unsigned long v3 = strtoul(e2 + 1, &e3, 0);
    if (!e3 || (*e3 != ',' && *e3 != ':' && *e3 != 'x'))
        return -1;
    unsigned long v4 = strtoul(e3 + 1, NULL, 0);
    *a = (uint32_t)v1; *b = (uint32_t)v2; *c = (uint32_t)v3; *d = (uint32_t)v4;
    return 0;
}

static void usage(const char *prog)
{
    log_line("usage: %s [options]\n", prog);
    log_line("  --csi <0-2>           CSI 号 (默认 %d)\n", g_cfg.csi);
    log_line("  --fps <n>             探测请求帧率 (默认 %d)\n", g_cfg.probe_fps);
    log_line("  --out <WxH>           输出/录像尺寸 (默认 %ux%u)\n", g_cfg.out_w, g_cfg.out_h);
    log_line("  -w <px> -h <px>       同上，分别指定宽/高（兼容旧启动项写法）\n");
    log_line("  --roi <x,y,w,h>       识别 ROI，w/h=0 表示全画面 (默认 %u,%u,%u,%u)\n",
             g_cfg.roi_x, g_cfg.roi_y, g_cfg.roi_w, g_cfg.roi_h);
    log_line("  --step <x,y>          采样步长 (默认 %u,%u)\n", g_cfg.step_x, g_cfg.step_y);
    log_line("  --lab <Lmin,Amax,Bmin>  CIELAB 阈值 (默认 %d,%d,%d)\n",
             g_cfg.lab_l_min, g_cfg.lab_a_max, g_cfg.lab_b_min);
    log_line("  --pix-min <n>         blob 最小像素数 (默认 %u)\n", g_cfg.pix_min);
    log_line("  --miss <n>            连续丢失 n 帧后整幅重扫，0=关 (默认 %d)\n",
             g_cfg.miss_full_scan);
    log_line("  --expo-us <us>        固定曝光微秒，0=不设 (默认 %d)\n", g_cfg.exposure_us);
    log_line("  --keep-gain           不把增益压到最低\n");
    log_line("  --ae                  打开 AE（默认关，配 --expo-us 用固定曝光）\n");
    log_line("  --no-record           不开录像\n");
    log_line("  --bitrate <kbps>      H.264 码率 (默认 %u)\n", g_cfg.bitrate_kbps);
    log_line("  --rec-dir <path>      录像目录 (默认 %s)\n", g_cfg.rec_dir);
    log_line("  --status <s>          状态日志周期秒 (默认 %d)\n", g_cfg.status_period_s);
    log_line("  --dump-every <s>      每 s 秒导出一张整帧 PPM(P6)，0=关 (默认 %d)\n",
             g_cfg.dump_every_s);
    log_line("  --dump-dir <path>     PPM 输出目录 (默认 %s)\n", g_cfg.dump_dir);
    log_line("  --help                本帮助\n");
}

static int parse_args(int argc, char **argv)
{
    static const struct option opts[] = {
        {"csi",        required_argument, 0, 'c'},
        {"fps",        required_argument, 0, 'f'},
        {"out",        required_argument, 0, 'o'},
        {"width",      required_argument, 0, 'w'},
        {"height",     required_argument, 0, 'H'},
        {"roi",        required_argument, 0, 'r'},
        {"step",       required_argument, 0, 's'},
        {"lab",        required_argument, 0, 'L'},
        {"pix-min",    required_argument, 0, 'p'},
        {"miss",       required_argument, 0, 'm'},
        {"expo-us",    required_argument, 0, 'e'},
        {"keep-gain",  no_argument,       0, 'G'},
        {"ae",         no_argument,       0, 'A'},
        {"no-record",  no_argument,       0, 'n'},
        {"bitrate",    required_argument, 0, 'b'},
        {"rec-dir",    required_argument, 0, 'D'},
        {"status",     required_argument, 0, 'S'},
        {"dump-every", required_argument, 0, 'P'},
        {"dump-dir",   required_argument, 0, 'Q'},
        {"help",       no_argument,       0, 1},
        {0, 0, 0, 0}
    };
    int c;
    while ((c = getopt_long(argc, argv, "w:h:n", opts, NULL)) != -1) {
        switch (c) {
        case 'c': g_cfg.csi = atoi(optarg); break;
        case 'f': g_cfg.probe_fps = atoi(optarg); break;
        case 'o':
            if (parse_pair(optarg, &g_cfg.out_w, &g_cfg.out_h) != 0)
                return -1;
            break;
        case 'w': g_cfg.out_w = (uint32_t)atoi(optarg); break;
        case 'h': g_cfg.out_h = (uint32_t)atoi(optarg); break;
        case 'H':
            g_cfg.out_h = (uint32_t)atoi(optarg);
            break;
        case 'r':
            if (parse_quad(optarg, &g_cfg.roi_x, &g_cfg.roi_y,
                           &g_cfg.roi_w, &g_cfg.roi_h) != 0)
                return -1;
            break;
        case 's':
            if (parse_pair(optarg, &g_cfg.step_x, &g_cfg.step_y) != 0)
                return -1;
            break;
        case 'L':
            if (parse_triple_i(optarg, &g_cfg.lab_l_min, &g_cfg.lab_a_max,
                               &g_cfg.lab_b_min) != 0)
                return -1;
            break;
        case 'p': g_cfg.pix_min = (uint32_t)atoi(optarg); break;
        case 'm': g_cfg.miss_full_scan = atoi(optarg); break;
        case 'e': g_cfg.exposure_us = atoi(optarg); break;
        case 'G': g_cfg.fix_gain = 0; break;
        case 'A': g_cfg.ae_enable = 1; break;
        case 'n': g_cfg.record = 0; break;
        case 'b': g_cfg.bitrate_kbps = (uint32_t)atoi(optarg); break;
        case 'D': g_cfg.rec_dir = optarg; break;
        case 'S': g_cfg.status_period_s = atoi(optarg); break;
        case 'P': g_cfg.dump_every_s = atoi(optarg); break;
        case 'Q': g_cfg.dump_dir = optarg; break;
        case 1: return 2;   /* --help */
        default: return -1;
        }
    }
    if (g_cfg.csi < 0 || g_cfg.csi > 2)
        return -1;
    if (g_cfg.probe_fps <= 0)
        g_cfg.probe_fps = ISP_FPS;
    if (g_cfg.out_w < 160 || g_cfg.out_h < 120)
        return -1;
    if (g_cfg.step_x == 0 || g_cfg.step_y == 0)
        return -1;
    if (g_cfg.status_period_s < 1)
        g_cfg.status_period_s = 1;
    return 0;
}

/* ============================ main ============================ */

int main(int argc, char **argv)
{
    int rc = parse_args(argc, argv);
    if (rc != 0) {
        usage(argv[0]);
        return rc == 2 ? 0 : 1;   /* 2 = --help；其它 = 参数错误 */
    }

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    signal(SIGINT, green_signal);
    signal(SIGTERM, green_signal);
    signal(SIGUSR1, rec_start_signal);
    signal(SIGUSR2, rec_stop_signal);
    mkdir("/sdcard/app/logs", 0777);
    color_tables_init();

    uint32_t gw = (g_cfg.out_w + g_cfg.step_x - 1) / g_cfg.step_x;
    uint32_t gh = (g_cfg.out_h + g_cfg.step_y - 1) / g_cfg.step_y;
    if (scratch_init(gw * gh) != 0) {
        log_line("green_led_rtos: scratch alloc failed (%u cells)\n", gw * gh);
        return -1;
    }

    log_line("green_led_rtos: start pid=%d\n", (int)getpid());
    {
        /* 记录启动命令行：板端日志能直接看出是谁、用什么参数拉起来的
         * （launcher 清单里换名字/改参数时特别有用） */
        char cmdline[256];
        int off = 0;
        for (int i = 0; i < argc && off < (int)sizeof(cmdline) - 2; ++i)
            off += snprintf(cmdline + off, sizeof(cmdline) - (size_t)off,
                            "%s%s", i ? " " : "", argv[i]);
        log_line("green_led_rtos: argv=[%s]\n", cmdline);
    }
    log_line("green_led_rtos: probe request csi=%d %dx%d@%d "
             "(适配表精确命中要求，见 main.c 头注释)\n",
             g_cfg.csi, ISP_WIDTH, ISP_HEIGHT, g_cfg.probe_fps);

    k_u32 sw = 0, sh = 0, sfps = 0;
    k_s32 ret = get_sensor_resolution((k_vicap_dev)g_cfg.csi, &sw, &sh, &sfps,
                                      ISP_WIDTH, ISP_HEIGHT, (k_u32)g_cfg.probe_fps);
    if (ret != 0) {
        log_line("green_led_rtos: sensor probe failed ret=%d csi=%d\n", (int)ret, g_cfg.csi);
        return -1;
    }
    log_line("green_led_rtos: sensor actual %ux%u@%u type=%d%s\n", sw, sh, sfps,
             (int)g_sensor_info.sensor_type,
             (sw == ISP_WIDTH && sh == ISP_HEIGHT)
                 ? ""
                 : " WARN: 与请求不一致，管线按实际模式配置");

    ret = sample_vb_init();
    if (ret != K_SUCCESS) {
        log_line("green_led_rtos: VB init failed ret=%d\n", (int)ret);
        return -1;
    }

    ret = green_vicap_init(sw, sh);
    if (ret != K_SUCCESS) {
        kd_mpi_vb_exit();
        return -1;
    }

    /* sensor fd：固定曝光/增益用（官方示例同样在 vicap_init 之后取） */
    memset(&g_sensor_attr, 0, sizeof(g_sensor_attr));
    g_sensor_attr.dev_num = DEV;
    if (kd_mpi_vicap_get_sensor_fd(&g_sensor_attr) != 0)
        g_sensor_attr.sensor_fd = -1;

    log_line("green_led_rtos: vicap dev=%d mode=offline acq=%ux%u out=%ux%u "
             "ae=%d awb=1 dnr3=1 buffers=6+6\n",
             (int)DEV, sw, sh, g_cfg.out_w, g_cfg.out_h, g_cfg.ae_enable);
    log_line("green_led_rtos: detect roi=(%u,%u,%u,%u) step=(%u,%u) "
             "LAB(L>=%d,A<=%d,B>=%d) pix_min=%u miss_full=%d\n",
             g_cfg.roi_x, g_cfg.roi_y, g_cfg.roi_w, g_cfg.roi_h,
             g_cfg.step_x, g_cfg.step_y,
             g_cfg.lab_l_min, g_cfg.lab_a_max, g_cfg.lab_b_min,
             g_cfg.pix_min, g_cfg.miss_full_scan);

    ret = kd_mpi_vicap_start_stream(DEV);
    if (ret != K_SUCCESS) {
        log_line("green_led_rtos: start stream failed ret=%d\n", (int)ret);
        kd_mpi_vicap_deinit(DEV);
        kd_mpi_vb_exit();
        return -1;
    }
    apply_sensor_exposure();

    if (g_cfg.record) {
        recorder_config_t rcfg;
        memset(&rcfg, 0, sizeof(rcfg));
        rcfg.record_dir = g_cfg.rec_dir;
        rcfg.cap_bytes = REC_DEFAULT_CAP_BYTES;
        rcfg.width = g_cfg.out_w;
        rcfg.height = g_cfg.out_h;
        rcfg.nominal_fps = (uint32_t)g_cfg.probe_fps;   /* 真实帧率，H.264 播放速度靠它 */
        rcfg.bitrate_kbps = g_cfg.bitrate_kbps;
        if (rec_init(&rcfg) == 0)
            log_line("green_led_rtos: recorder on dir=%s fps=%d bitrate=%u kbps\n",
                     g_cfg.rec_dir, g_cfg.probe_fps, g_cfg.bitrate_kbps);
        else
            log_line("green_led_rtos: recorder init failed (detection only)\n");
    } else {
        log_line("green_led_rtos: recorder disabled (--no-record)\n");
    }

    {
        int pfd = open(REC_PID_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (pfd >= 0) {
            char pbuf[32];
            int n = snprintf(pbuf, sizeof(pbuf), "%d\n", (int)getpid());
            if (n > 0)
                write(pfd, pbuf, (size_t)n);
            close(pfd);
        }
    }

    pthread_t capture, detect;
    int capture_started = 0, detect_started = 0;
    if (pthread_create(&capture, NULL, capture_thread, NULL) == 0)
        capture_started = 1;
    else
        log_line("green_led_rtos: capture thread create failed\n");
    if (capture_started && pthread_create(&detect, NULL, detect_thread, NULL) == 0)
        detect_started = 1;
    else if (capture_started)
        log_line("green_led_rtos: detect thread create failed\n");

    if (!capture_started || !detect_started) {
        log_line("green_led_rtos: thread creation failed\n");
        app_running = 0;
        pthread_cond_broadcast(&frame_ready);
    } else {
        if (g_cfg.dump_every_s > 0)
        log_line("green_led_rtos: frame dump on: every %ds -> %s\n",
                 g_cfg.dump_every_s, g_cfg.dump_dir);
    log_line("green_led_rtos: running (detector owns VICAP)\n");
    }

    if (capture_started)
        pthread_join(capture, NULL);
    if (detect_started)
        pthread_join(detect, NULL);

    rec_deinit();   /* 停会话、回收编码器与 VB（须在 kd_mpi_vb_exit 之前） */
    kd_mpi_vicap_stop_stream(DEV);
    kd_mpi_vicap_deinit(DEV);
    kd_mpi_vb_exit();
    log_line("green_led_rtos: exit (dumped=%" PRIu64 " drop=%" PRIu64
             " dump_fail=%" PRIu64 ")\n",
             frames_dumped, frames_dropped, dump_fail);
    return 0;
}
