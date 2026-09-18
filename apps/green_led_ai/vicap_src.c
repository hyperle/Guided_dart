/*
 * vicap_src —— 实现
 */
#include "vicap_src.h"

#include <stdio.h>
#include <string.h>

#include "app.h"
#include "mpi_sensor_api.h"
#include "mpi_sys_api.h"
#include "mpi_vb_api.h"

#define DEV VICAP_DEV_ID_0

static int      g_opened;
static uint32_t g_acq_w, g_acq_h, g_acq_fps;
static uint32_t g_chn_w[2], g_chn_h[2];

static k_vicap_sensor_info g_sensor_info;
static k_vicap_sensor_attr g_sensor_attr;

uint32_t vicap_acq_width(void)  { return g_acq_w; }
uint32_t vicap_acq_height(void) { return g_acq_h; }
uint32_t vicap_acq_fps(void)    { return g_acq_fps; }
int      vicap_is_open(void)    { return g_opened; }

uint32_t vicap_chn_width(int chn)
{
    return (chn >= 0 && chn < 2) ? g_chn_w[chn] : 0;
}

uint32_t vicap_chn_height(int chn)
{
    return (chn >= 0 && chn < 2) ? g_chn_h[chn] : 0;
}

/* ============================ VB ============================ */

static int vb_init(void)
{
    k_vb_config config;
    memset(&config, 0, sizeof(config));
    config.max_pool_cnt = 64;

    k_s32 ret = kd_mpi_vb_set_config(&config);
    if (ret != K_SUCCESS) {
        app_log(APP_NAME_STR ": vb_set_config failed ret=%d\n", (int)ret);
        return -1;
    }

    k_vb_supplement_config sup;
    memset(&sup, 0, sizeof(sup));
    sup.supplement_config |= VB_SUPPLEMENT_JPEG_MASK;
    ret = kd_mpi_vb_set_supplement_config(&sup);
    if (ret != K_SUCCESS) {
        app_log(APP_NAME_STR ": vb_set_supplement_config failed ret=%d\n", (int)ret);
        return -1;
    }

    ret = kd_mpi_vb_init();
    if (ret != K_SUCCESS) {
        app_log(APP_NAME_STR ": vb_init failed ret=%d\n", (int)ret);
        return -1;
    }
    return 0;
}

/* ============================ sensor 探测 ============================ */

/*
 * kd_mpi_sensor_adapt_get() 是三级匹配：精确(宽+高+fps) -> 同宽高 -> 找更大
 * 分辨率，候选按 fps 降序。所以请求必须是适配表里精确存在的模式
 * （gc2093 在 CSI2 精确命中 1920x1080@30），否则会静默落到别的模式，
 * 采集侧实际分辨率/帧率和代码常量全都对不上。
 */
static int sensor_probe(void)
{
    k_vicap_sensor_info info;
    k_vicap_probe_config probe;

    memset(&info, 0, sizeof(info));
    memset(&probe, 0, sizeof(probe));
    probe.csi_num = (k_u32)g_cfg.csi;
    probe.width   = g_cfg.acq_w;
    probe.height  = g_cfg.acq_h;
    probe.fps     = (k_u32)g_cfg.probe_fps;

    if (kd_mpi_sensor_adapt_get(&probe, &info) != 0) {
        app_log(APP_NAME_STR ": sensor probe failed csi=%d %ux%u@%d\n",
                g_cfg.csi, g_cfg.acq_w, g_cfg.acq_h, g_cfg.probe_fps);
        return -1;
    }
    if (kd_mpi_vicap_get_sensor_info(info.sensor_type, &info) != K_SUCCESS) {
        app_log(APP_NAME_STR ": get_sensor_info failed type=%d\n", (int)info.sensor_type);
        return -1;
    }

    memcpy(&g_sensor_info, &info, sizeof(info));
    g_acq_w = info.width;
    g_acq_h = info.height;
    g_acq_fps = info.fps;

    int exact = (g_acq_w == g_cfg.acq_w && g_acq_h == g_cfg.acq_h &&
                 g_acq_fps == (uint32_t)g_cfg.probe_fps);
    app_log(APP_NAME_STR ": sensor actual %ux%u@%u type=%d"
            "（请求 %ux%u@%d）%s\n",
            g_acq_w, g_acq_h, g_acq_fps, (int)info.sensor_type,
            g_cfg.acq_w, g_cfg.acq_h, g_cfg.probe_fps,
            exact ? ""
                  : " WARN: 请求没精确命中 -> 已退回『同分辨率最高帧率』。"
                    "gc2093/CSI2 适配表只有 1920x1080@30、1920x1080@60、1280x960@90、"
                    "1280x720@90 四档，管线按**实际**模式配置");
    return 0;
}

/* ============================ VICAP ============================ */

static int chn_config(int chn, uint32_t out_w, uint32_t out_h, uint32_t pix_format,
                      uint32_t buffer_num, uint32_t bytes_per_px_num, uint32_t bytes_per_px_den)
{
    k_vicap_chn_attr attr;

    /* 硬件要求输出宽高 8 对齐 */
    if ((out_w & 7u) != 0)
        out_w = (out_w + 7u) & ~7u;
    if ((out_h & 1u) != 0)
        out_h = (out_h + 1u) & ~1u;

    memset(&attr, 0, sizeof(attr));
    attr.out_win.width  = out_w;
    attr.out_win.height = out_h;
    attr.crop_win.width = g_acq_w;
    attr.crop_win.height = g_acq_h;
    attr.scale_win      = attr.out_win;
    attr.crop_enable    = K_FALSE;
    /* ONLINE：尺寸由 ISP 输出窗口给出，通道缩放器关掉（官方示例写法）；
     * OFFLINE：通道缩放器负责缩放 */
    attr.scale_enable   = (!g_cfg.vicap_online && (out_w != g_acq_w || out_h != g_acq_h))
                              ? K_TRUE : K_FALSE;
    attr.chn_enable     = K_TRUE;
    attr.pix_format     = (k_pixel_format)pix_format;
    attr.buffer_num     = buffer_num;
    attr.buffer_size    = VB_ALIGN_UP((uint64_t)out_w * out_h * bytes_per_px_num /
                                          bytes_per_px_den, 4096);
    attr.alignment      = 12;
    attr.buffer_pool_id = VB_INVALID_POOLID;                 /* 用 VB 公共池 */

    k_s32 ret = kd_mpi_vicap_set_chn_attr(DEV, (k_vicap_chn)chn, attr);
    if (ret != K_SUCCESS) {
        app_log(APP_NAME_STR ": set_chn_attr CHN%d (%ux%u fmt=%u) failed ret=%d\n",
                chn, out_w, out_h, pix_format, (int)ret);
        return -1;
    }
    if (chn < 2) {
        g_chn_w[chn] = out_w;
        g_chn_h[chn] = out_h;
    }
    return 0;
}

static int vicap_hw_setup(void)
{
    k_vicap_dev_attr dev_attr;

    memset(&dev_attr, 0, sizeof(dev_attr));
    dev_attr.acq_win.width  = g_acq_w;
    dev_attr.acq_win.height = g_acq_h;
    /*
     * ONLINE（官方 yolov8_run_camera 的组合）：ISP 直接按各通道 out_win 出图，
     *   不需要 dev 帧缓冲，通道侧 scale_enable=K_FALSE。
     * OFFLINE（旧工程 green_led_rtos 实测跑过 4 万帧）：ISP 先落 DRAM，
     *   各通道用自己的缩放器（scale_enable=K_TRUE），需要 dev 帧缓冲。
     */
    dev_attr.mode = g_cfg.vicap_online ? VICAP_WORK_ONLINE_MODE
                                       : VICAP_WORK_OFFLINE_MODE;
    if (!g_cfg.vicap_online) {
        dev_attr.buffer_num     = 6;
        dev_attr.buffer_size    = VB_ALIGN_UP((uint64_t)g_acq_w * g_acq_h * 2, 4096);
        dev_attr.buffer_pool_id = VB_INVALID_POOLID;
    }
    dev_attr.pipe_ctrl.data = 0xFFFFFFFF;
    dev_attr.pipe_ctrl.bits.ae_enable   = g_cfg.ae_enable ? K_TRUE : K_FALSE;
    dev_attr.pipe_ctrl.bits.awb_enable  = K_TRUE;   /* 颜色阈值是在 AWB 下标定的 */
    dev_attr.pipe_ctrl.bits.ahdr_enable = K_FALSE;
    dev_attr.pipe_ctrl.bits.dnr3_enable = K_TRUE;
    memcpy(&dev_attr.sensor_info, &g_sensor_info, sizeof(g_sensor_info));

    k_s32 ret = kd_mpi_vicap_set_dev_attr(DEV, dev_attr);
    if (ret != K_SUCCESS) {
        app_log(APP_NAME_STR ": set_dev_attr failed ret=%d\n", (int)ret);
        return -1;
    }

    /*
     * 只配 CHN0（唯一通道）。CHN0 是整条流水唯一的帧源：
     *   --vision on  -> 识别检测它
     *   --record on  -> 录像是把它**拷一份**给自己的块再喂 VENC（见 record.c）
     * 双通道（CHN1 给录像）在板端已被证伪：bind 双通道跑 4~147 帧后 CHN0 永久
     * NOTREADY；所以这里连 CHN1 都不再配置。--vision off 时 CHN0 照配（否则没有
     * 帧源，录像也就无从谈起），只是识别侧不跑检测。
     */
    if (g_cfg.vis_rgb_planar) {
        if (chn_config(VICAP_CHN_VISION, g_cfg.vis_w, g_cfg.vis_h,
                       PIXEL_FORMAT_RGB_888_PLANAR, 6, 3, 1) != 0)
            return -1;
    } else {
        if (chn_config(VICAP_CHN_VISION, g_cfg.vis_w, g_cfg.vis_h,
                       PIXEL_FORMAT_YUV_SEMIPLANAR_420, 6, 3, 2) != 0)
            return -1;
    }

    ret = kd_mpi_vicap_init(DEV);
    if (ret != K_SUCCESS) {
        app_log(APP_NAME_STR ": vicap_init failed ret=%d\n", (int)ret);
        return -1;
    }
    return 0;
}

/* 固定曝光 / 最低增益：必须在 start_stream 之后设置（与官方示例一致） */
static void apply_exposure(void)
{
    memset(&g_sensor_attr, 0, sizeof(g_sensor_attr));
    g_sensor_attr.dev_num = DEV;
    if (kd_mpi_vicap_get_sensor_fd(&g_sensor_attr) != 0 || g_sensor_attr.sensor_fd < 0) {
        app_log(APP_NAME_STR ": sensor fd unavailable; skip exposure/gain\n");
        return;
    }

    if (g_cfg.ae_enable) {
        app_log(APP_NAME_STR ": sensor AE enabled; skip fixed exposure/gain\n");
        return;
    }

    if (g_cfg.exposure_us > 0) {
        k_sensor_exposure_time_range range;
        memset(&range, 0, sizeof(range));
        if (kd_mpi_sensor_get_exposure_time_range(g_sensor_attr.sensor_fd, &range) == 0) {
            /*
             * 单位必须统一：k_sensor_intg_time 用的是**秒**，而 range 给的是**微秒**。
             * （2025-09 板端日志抓到过一版把两者混比的写法：请求 200us 被夹到
             *   min_intg_time_us=27.375 当秒写进传感器，曝光彻底跑偏。）
             */
            float min_s = range.min_intg_time_us / 1000000.0f;
            float max_s = range.max_intg_time_us / 1000000.0f;
            float want = (float)g_cfg.exposure_us / 1000000.0f;
            float clamped = want;
            if (clamped < min_s) clamped = min_s;
            if (clamped > max_s) clamped = max_s;
            k_sensor_intg_time t;
            memset(&t, 0, sizeof(t));
            t.intg_time[0] = clamped;
            int rc = (int)kd_mpi_sensor_intg_time_set(g_sensor_attr.sensor_fd, t);
            app_log(APP_NAME_STR ": exposure fixed %.0f us (请求 %d us, range %.0f-%.0f us) rc=%d%s\n",
                    clamped * 1000000.0f, g_cfg.exposure_us,
                    range.min_intg_time_us, range.max_intg_time_us, rc,
                    (clamped != want) ? " [被范围夹住]" : "");
        } else {
            app_log(APP_NAME_STR ": get exposure range failed; keep default\n");
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
            app_log(APP_NAME_STR ": gain fixed %.3f (range %.3f-%.3f) rc=%d\n",
                    gr.min, gr.min, gr.max, rc);
        } else {
            app_log(APP_NAME_STR ": get gain range failed; keep default\n");
        }
    }
}

int vicap_setup(void)
{
    if (vb_init() != 0)
        return -1;
    if (sensor_probe() != 0)
        goto fail_vb;
    if (vicap_hw_setup() != 0)
        goto fail_vicap;

    g_opened = 1;
    return 0;

fail_vicap:
    kd_mpi_vicap_deinit(DEV);
fail_vb:
    kd_mpi_vb_exit();
    return -1;
}

int vicap_start(void)
{
    if (!g_opened)
        return -1;

    k_s32 ret = kd_mpi_vicap_start_stream(DEV);
    if (ret != K_SUCCESS) {
        app_log(APP_NAME_STR ": start_stream failed ret=%d\n", (int)ret);
        return -1;
    }

    apply_exposure();

    app_log(APP_NAME_STR ": vicap dev=%d mode=%s acq=%ux%u@%u "
            "CHN0=%ux%u 单通道（detect=%s record=%s 都吃这一条）ae=%d awb=1 dnr3=1\n",
            (int)DEV, g_cfg.vicap_online ? "online" : "offline", g_acq_w, g_acq_h, g_acq_fps,
            g_chn_w[0], g_chn_h[0],
            g_cfg.vision_on ? "on" : "off", g_cfg.record_on ? "on" : "off",
            g_cfg.ae_enable);

    return 0;
}

void vicap_stop(void)
{
    mpp_enter();
    if (g_opened) {
        kd_mpi_vicap_stop_stream(DEV);
        kd_mpi_vicap_deinit(DEV);
        g_opened = 0;
    }
    kd_mpi_vb_exit();
    mpp_leave();
}

/* ============================ dump / release ============================ */

int vicap_dump(int chn, k_video_frame_info *frame, int timeout_ms)
{
    /*
     * VICAP 段**不占用 MPP 全局锁**：dump 是"阻塞等下一帧"的调用，占着全局锁
     * 就等于让采集线程独吞所有 MPP 调用（板上实测会把整链压到 13fps）。
     * 本工程约定 VICAP 只由采集线程调用，vicap_enter 只做并发自检（不阻塞）。
     */
    vicap_enter();
    k_s32 ret = kd_mpi_vicap_dump_frame(DEV, (k_vicap_chn)chn, VICAP_DUMP_YUV,
                                        frame, (k_s32)timeout_ms);
    vicap_leave();
    return (ret == K_SUCCESS) ? 0 : (int)ret;
}

/*
 * 返回值：0=拿到帧，1=暂时没帧（BUF_EMPTY，常态），2=流水未就绪（NOTREADY，
 * 说明 ISP/VICAP 被反压卡住了，不是"没帧"这么简单），-1=其它错误。
 *
 * 这两个错误码必须分开：板端实测 "视觉停流" 时报的是 NOTREADY(errid=16)，
 * 含义是"模块未就绪/流水停了"；而 BUF_EMPTY(14) 才是"此刻没有帧"。
 * 之前混成一个分支，导致日志一直显示"在等帧"，把真正的原因盖住了。
 */
int vicap_dump_soft(int chn, k_video_frame_info *frame, int timeout_ms, int *ret)
{
    vicap_enter();
    k_s32 r = kd_mpi_vicap_dump_frame(DEV, (k_vicap_chn)chn, VICAP_DUMP_YUV,
                                      frame, (k_s32)timeout_ms);
    vicap_leave();
    if (ret)
        *ret = (int)r;
    if (r == K_SUCCESS)
        return 0;
    if ((int)r == (int)K_ERR_VICAP_BUF_EMPTY)
        return 1;
    if ((int)r == (int)K_ERR_VICAP_NOTREADY)
        return 2;
    return -1;
}

static uint64_t g_release_fail;

int vicap_release(int chn, const k_video_frame_info *frame)
{
    vicap_enter();
    k_s32 ret = kd_mpi_vicap_dump_release(DEV, (k_vicap_chn)chn, frame);
    vicap_leave();
    if (ret != K_SUCCESS) {
        /* 归还失败 = VB 块回不到环形缓冲，攒够 6 个整条通道就死了：
         * 这个计数必须盯着（板端 2025-09 的"停流"就是这类症状） */
        g_release_fail++;
        if (g_release_fail % 20u == 1u)
            app_log(APP_NAME_STR ": vicap release CHN%d 失败 ret=0x%08x (累计 %llu)\n",
                    chn, (unsigned)ret, (unsigned long long)g_release_fail);
        return (int)ret;
    }
    return 0;
}

uint64_t vicap_release_fail_count(void)
{
    return g_release_fail;
}
