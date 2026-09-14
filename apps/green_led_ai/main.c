/*
 * green_led_ai —— 绿灯识别 + H.264 录像（K230/K230D RT-Smart，单进程双子系统）
 *
 *   sensor(gc2093) 1920x1080@30
 *     -> VICAP OFFLINE
 *          |- CHN0 (视觉)  -> 视觉线程: NV12 -> RGB565 -> CIELAB 查表阈值(RVV)
 *          |                             -> 连通域 -> 绿灯重心
 *          |- CHN1 (录像)  -> 录像线程: 25fps 抽样 -> VENC(直接吃 VICAP 物理地址)
 *                                        -> 落盘 /sdcard/app/recording/rec_XXXX.h264
 *
 * 启动与日志沿用旧工程模式（这是明确要求）：
 *   - 由 /sdcard/app/launcher 读 startup_final.list 后台拉起（行尾 &）；
 *   - 日志每行 open->write->fsync->close 直写
 *     /sdcard/app/logs/green_led_ai.log，同时进 stdout（launcher 收到的
 *     /sdcard/app/logs/launcher-child.log）；
 *   - pid 写 /sdcard/app/green_led_ai.pid；
 *   - 运行期用 /sdcard/app/green_led_ai.ctl（Unix socket）或 SIGUSR1/SIGUSR2
 *     分别启停视觉与录像。
 *
 * 与旧工程 green_led_rtos 的关系：采集/编码/落盘/日志/控制这一套沿用，识别
 * 从「逐帧 mmap + float CIELAB + 与录像共用同一帧」改为「通道隔离 + 映射缓存
 * + RVV 查表阈值」，修掉旧工程的帧双重所有权崩溃（详见 README.md）。
 */
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "k_pm_comm.h"
#include "mpi_pm_api.h"

#include "app.h"
#include "control.h"
#include "record.h"
#include "vicap_src.h"
#include "vision.h"

/* ============================ 默认配置 ============================ */

static cfg_t g_default = {
    .csi = 2,
    .probe_fps = 30,
    .acq_w = 1920,
    .acq_h = 1080,
    .ae_enable = 0,
    .exposure_us = 200,
    .fix_gain = 1,

    .vision_on = 1,
    .record_on = 1,
    .vicap_online = -1,        /* -1=auto */

    .vis_w = 640,
    .vis_h = 480,
    .rec_w = 640,
    .rec_h = 480,

    .roi_x = 160, .roi_y = 120, .roi_w = 320, .roi_h = 240,
    .step_x = 2, .step_y = 2,
    .lab_l_min = 12, .lab_a_max = -20, .lab_b_min = 8,
    .pix_min = 50,
    .miss_full_scan = 3,

    .rec_fps = 25,
    .bitrate_kbps = 12000,
    .rec_dir = REC_DEFAULT_DIR,
    .cap_bytes = REC_DEFAULT_CAP_BYTES,

    .detector = "color",
    .kmodel = NULL,
    .vis_rgb_planar = 0,
    .kpu_conf = 0.35f,
    .kpu_iou = 0.45f,
    .kpu_class = -1,
    .kpu_nc = 0,
    .kpu_nms = "own",
    .kpu_sync = 0,

    .pm_perf = 0,
    .rec_mode = 0,               /* 默认 shared：单通道，板端验证过的形态 */
    .dump_timeout_ms = 150,      /* 连续 dump + 长超时：旧工程 40079 帧稳定 */
    .status_period_s = 2,
    .trace_frames = 5,
};

/* ============================ 运行标志 ============================ */

static volatile int g_running = 1;
static volatile int g_sig_rec_on, g_sig_rec_off;

static void on_term(int sig)
{
    (void)sig;
    g_running = 0;
}

static void on_usr1(int sig)
{
    (void)sig;
    g_sig_rec_on = 1;
}

static void on_usr2(int sig)
{
    (void)sig;
    g_sig_rec_off = 1;
}

/* ============================ 参数 ============================ */

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

static int parse_quad(const char *s, uint32_t *a, uint32_t *b, uint32_t *c, uint32_t *d)
{
    char *e1 = NULL, *e2 = NULL, *e3 = NULL;
    unsigned long v1 = strtoul(s, &e1, 0);
    if (!e1 || (*e1 != ',' && *e1 != ':' && *e1 != 'x'))
        return -1;
    unsigned long v2 = strtoul(e1 + 1, &e2, 0);
    if (!e2 || (*e2 != ',' && *e2 != ':' && *e2 != 'x'))
        return -1;
    unsigned long v3 = strtoul(e2 + 1, &e3, 0);
    if (!e3 || (*e3 != ',' && *e3 != ':' && *e3 != 'x'))
        return -1;
    unsigned long v4 = strtoul(e3 + 1, NULL, 0);
    *a = (uint32_t)v1; *b = (uint32_t)v2; *c = (uint32_t)v3; *d = (uint32_t)v4;
    return 0;
}

static int parse_triple_i(const char *s, int *a, int *b, int *c)
{
    char *e1 = NULL, *e2 = NULL;
    long v1 = strtol(s, &e1, 0);
    if (!e1 || (*e1 != ',' && *e1 != ':' && *e1 != 'x'))
        return -1;
    long v2 = strtol(e1 + 1, &e2, 0);
    if (!e2 || (*e2 != ',' && *e2 != ':' && *e2 != 'x'))
        return -1;
    long v3 = strtol(e2 + 1, NULL, 0);
    *a = (int)v1; *b = (int)v2; *c = (int)v3;
    return 0;
}

static int parse_onoff(const char *s, int *out)
{
    if (!strcmp(s, "on") || !strcmp(s, "1") || !strcmp(s, "yes")) {
        *out = 1;
        return 0;
    }
    if (!strcmp(s, "off") || !strcmp(s, "0") || !strcmp(s, "no")) {
        *out = 0;
        return 0;
    }
    return -1;
}

static void usage(void)
{
    app_log("usage: " APP_NAME_STR " [options]\n");
    app_log("  --csi <0-2>            CSI 号 (默认 %d，庐山派在 CSI2)\n", g_default.csi);
    app_log("  --fps <n>              探测请求帧率 (默认 %d，须与 1920x1080 成对)\n",
            g_default.probe_fps);
    app_log("  --vision <on|off>      视觉子系统 (默认 on；off 则不配置 CHN0)\n");
    app_log("  --record <on|off>      录像子系统 (默认 on；off 则不配置 CHN1)\n");
    app_log("  --vis-out <WxH>        视觉通道尺寸 (默认 %ux%u)\n",
            g_default.vis_w, g_default.vis_h);
    app_log("  --rec-out <WxH>        录像通道尺寸 (默认 %ux%u)\n",
            g_default.rec_w, g_default.rec_h);
    app_log("  --roi <x,y,w,h>        识别 ROI，w/h=0 表示全画面 (默认 %u,%u,%u,%u)\n",
            g_default.roi_x, g_default.roi_y, g_default.roi_w, g_default.roi_h);
    app_log("  --step <x,y>           采样步长 (默认 %u,%u)\n",
            g_default.step_x, g_default.step_y);
    app_log("  --lab <Lmin,Amax,Bmin> CIELAB 阈值 (默认 %d,%d,%d)\n",
            g_default.lab_l_min, g_default.lab_a_max, g_default.lab_b_min);
    app_log("  --pix-min <n>          blob 最小像素数 (默认 %u)\n", g_default.pix_min);
    app_log("  --miss <n>             连续丢失 n 帧后整幅重扫，0=关 (默认 %d)\n",
            g_default.miss_full_scan);
    app_log("  --expo-us <us>         固定曝光微秒，0=不设 (默认 %d)\n", g_default.exposure_us);
    app_log("  --keep-gain            不把增益压到最低\n");
    app_log("  --ae                   打开自动曝光 (默认关)\n");
    app_log("  --rec-fps <n>          录像抽样帧率 (默认 %d)\n", g_default.rec_fps);
    app_log("  --bitrate <kbps>       H.264 码率 (默认 %u)\n", g_default.bitrate_kbps);
    app_log("  --rec-dir <path>       录像目录 (默认 %s)\n", g_default.rec_dir);
    app_log("  --cap <bytes>          目录总容量上限 (默认 8GB)\n");
    app_log("  --detector <name>      color|kpu (默认 color)\n");
    app_log("  --kmodel <path>        kpu 后端的 kmodel 路径（YOLOv8 风格单输出）\n");
    app_log("  --kpu-conf <f>         kpu 置信度阈值 (默认 %.2f)\n", g_default.kpu_conf);
    app_log("  --kpu-iou <f>          kpu NMS IoU 阈值 (默认 %.2f)\n", g_default.kpu_iou);
    app_log("  --kpu-class <n>        kpu 只认该类（-1=任意，默认 -1）\n");
    app_log("  --kpu-nc <n>           kpu 类别数（0=按输出形状推断）\n");
    app_log("  --kpu-nms <own>        kpu NMS 实现（只支持 own；SDK 的 librvv.a\n");
    app_log("                         实际没实现 nms()，见 README 说明）\n");
    app_log("  --kpu-sync <0|1>       kpu 输入 tensor 是否 sync_write_back（默认 0）\n");
    app_log("  --rec-mode <shared|bind>  录像取帧方式 (默认 shared:\n");
    app_log("                        单通道、识别交棒给编码器；bind=双通道硬件直连，实验特性)\n");
    app_log("  --dump-timeout <ms>    dump 等待上限 (默认 %d ms)\n", g_default.dump_timeout_ms);
    app_log("  --vicap-mode <auto|online|offline>  采集工作模式 (默认 auto:\n");
    app_log("                 shared 单通道 -> offline；bind 双通道 -> online)\n");
    app_log("  --pm-perf              把 CPU/KPU 的 PM governor 设为 performance\n");
    app_log("                         (若板端 DVFS 把大核降频，加这个；失败只记日志)\n");
    app_log("  --status <s>           状态日志周期秒 (默认 %d)\n", g_default.status_period_s);
    app_log("  --trace <n>            前 n 帧逐帧 trace，0=关 (默认 %d)\n", g_default.trace_frames);
    app_log("  --help                 本帮助\n");
}

static int parse_args(int argc, char **argv)
{
    static const struct option opts[] = {
        { "csi",      required_argument, 0, 'c' },
        { "fps",      required_argument, 0, 'f' },
        { "vision",   required_argument, 0, 'V' },
        { "record",   required_argument, 0, 'R' },
        { "vis-out",  required_argument, 0, 'v' },
        { "rec-out",  required_argument, 0, 'r' },
        { "roi",      required_argument, 0, 'o' },
        { "step",     required_argument, 0, 's' },
        { "lab",      required_argument, 0, 'L' },
        { "pix-min",  required_argument, 0, 'p' },
        { "miss",     required_argument, 0, 'm' },
        { "expo-us",  required_argument, 0, 'e' },
        { "keep-gain",no_argument,       0, 'G' },
        { "ae",       no_argument,       0, 'A' },
        { "rec-fps",  required_argument, 0, 'F' },
        { "bitrate",  required_argument, 0, 'b' },
        { "rec-dir",  required_argument, 0, 'D' },
        { "cap",      required_argument, 0, 'C' },
        { "detector", required_argument, 0, 'd' },
        { "pm-perf",  no_argument,       0, 'P' },
        { "vicap-mode", required_argument, 0, 'M' },
        { "dump-timeout", required_argument, 0, 'u' },
        { "kmodel",   required_argument, 0, 'k' },
        { "kpu-conf", required_argument, 0, 'q' },
        { "kpu-iou",  required_argument, 0, 'Q' },
        { "kpu-class",required_argument, 0, 'y' },
        { "kpu-nc",   required_argument, 0, 'Y' },
        { "kpu-nms",  required_argument, 0, 'z' },
        { "kpu-sync", required_argument, 0, 'Z' },
        { "status",   required_argument, 0, 'S' },
        { "trace",    required_argument, 0, 'T' },
        { "help",     no_argument,       0, 'h' },
        { 0, 0, 0, 0 }
    };

    int c;
    while ((c = getopt_long(argc, argv, "", opts, NULL)) != -1) {
        switch (c) {
        case 'c': g_cfg.csi = atoi(optarg); break;
        case 'f': g_cfg.probe_fps = atoi(optarg); break;
        case 'V':
            if (parse_onoff(optarg, &g_cfg.vision_on) != 0)
                return -1;
            break;
        case 'R':
            if (parse_onoff(optarg, &g_cfg.record_on) != 0)
                return -1;
            break;
        case 'v':
            if (parse_pair(optarg, &g_cfg.vis_w, &g_cfg.vis_h) != 0)
                return -1;
            break;
        case 'r':
            if (parse_pair(optarg, &g_cfg.rec_w, &g_cfg.rec_h) != 0)
                return -1;
            break;
        case 'o':
            if (parse_quad(optarg, &g_cfg.roi_x, &g_cfg.roi_y, &g_cfg.roi_w, &g_cfg.roi_h) != 0)
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
        case 'F': g_cfg.rec_fps = atoi(optarg); break;
        case 'b': g_cfg.bitrate_kbps = (uint32_t)atoi(optarg); break;
        case 'D': g_cfg.rec_dir = optarg; break;
        case 'C': g_cfg.cap_bytes = strtoull(optarg, NULL, 0); break;
        case 'd': g_cfg.detector = optarg; break;
        case 'P': g_cfg.pm_perf = 1; break;
        case 'j': g_cfg.rec_mode = (strcmp(optarg, "bind") == 0) ? 1 : 0; break;
        case 'u': g_cfg.dump_timeout_ms = atoi(optarg); break;
        case 'M':
            if (strcmp(optarg, "online") == 0)
                g_cfg.vicap_online = 1;
            else if (strcmp(optarg, "offline") == 0)
                g_cfg.vicap_online = 0;
            else
                g_cfg.vicap_online = -1;
            break;
        case 'k': g_cfg.kmodel = optarg; break;
        case 'q': g_cfg.kpu_conf = strtof(optarg, NULL); break;
        case 'Q': g_cfg.kpu_iou = strtof(optarg, NULL); break;
        case 'y': g_cfg.kpu_class = atoi(optarg); break;
        case 'Y': g_cfg.kpu_nc = atoi(optarg); break;
        case 'z': g_cfg.kpu_nms = optarg; break;
        case 'Z': g_cfg.kpu_sync = atoi(optarg); break;
        case 'S': g_cfg.status_period_s = atoi(optarg); break;
        case 'T': g_cfg.trace_frames = atoi(optarg); break;
        case 'h': return 2;
        default: return -1;
        }
    }

    if (g_cfg.csi < 0 || g_cfg.csi > 2)
        return -1;
    if (g_cfg.probe_fps <= 0)
        g_cfg.probe_fps = g_default.probe_fps;
    if (g_cfg.vis_w < 160 || g_cfg.vis_h < 120 || g_cfg.rec_w < 160 || g_cfg.rec_h < 120)
        return -1;
    if (g_cfg.step_x == 0 || g_cfg.step_y == 0)
        return -1;
    if (g_cfg.rec_fps <= 0)
        g_cfg.rec_fps = g_default.rec_fps;
    if (g_cfg.status_period_s < 1)
        g_cfg.status_period_s = 1;
    /*
     * 录像取帧方式：
     *   shared（默认）：只有 CHN0 一条通道，识别处理完把帧交给编码器，
     *                   取流线程在拿到对应码流后归还 —— 与旧工程同形态（实测 40079 帧稳定）
     *   bind（实验）： 额外配 CHN1 并硬件绑定到 VENC。本板实测该形态 200~400 帧后
     *                   两条通道一起衰减到停，故不作为默认。
     */
    if (g_cfg.rec_mode == 0 && g_cfg.record_on &&
        (g_cfg.rec_w != g_cfg.vis_w || g_cfg.rec_h != g_cfg.vis_h))
        app_log(APP_NAME_STR ": shared 模式与识别共用 CHN0，--rec-out 被忽略"
                "（录像尺寸 = 识别尺寸 %ux%u）\n", g_cfg.vis_w, g_cfg.vis_h);

    if (g_cfg.rec_mode == 0 && !g_cfg.vision_on && g_cfg.record_on) {
        app_log(APP_NAME_STR ": vision off + shared 无法供帧，录像自动改用 bind 模式\n");
        g_cfg.rec_mode = 1;
    }

    /*
     * 采集工作模式（auto）：shared 是单通道 -> OFFLINE（旧工程验证过）；
     * bind 是双通道 -> ONLINE（官方 yolov8_run_camera 的组合）。
     */
    if (g_cfg.vicap_online < 0)
        g_cfg.vicap_online = (g_cfg.rec_mode == 1) ? 1 : 0;

    /* 后端决定视觉通道的像素格式：color 需要 NV12，kpu 走 ISP 出的 RGB 平面 */
    g_cfg.vis_rgb_planar = (strcmp(g_cfg.detector, "kpu") == 0) ? 1 : 0;
    if (g_cfg.vis_rgb_planar && !g_cfg.vision_on) {
        app_log(APP_NAME_STR ": --detector kpu 需要视觉通道，但 --vision off\n");
        return -1;
    }
    if (g_cfg.vis_rgb_planar && !g_cfg.kmodel) {
        app_log(APP_NAME_STR ": --detector kpu 必须给 --kmodel\n");
        return -1;
    }
    if (!g_cfg.vision_on && !g_cfg.record_on)
        app_log(APP_NAME_STR ": WARN 视觉与录像都关闭，只会初始化采集然后空转\n");
    return 0;
}

/* ============================ PM（可选：把 CPU/KPU 锁到最高频档） ============================ */

static void pm_lock_performance(void)
{
    int32_t cur = -1;
    (void)kd_mpi_pm_get_profile(PM_DOMAIN_CPU, &cur);
    int rc = kd_mpi_pm_set_governor(PM_DOMAIN_CPU, PM_GOVERNOR_PERFORMANCE);
    app_log(APP_NAME_STR ": pm CPU governor->performance rc=%d (before profile=%d)\n", rc, cur);

    cur = -1;
    (void)kd_mpi_pm_get_profile(PM_DOMAIN_KPU, &cur);
    rc = kd_mpi_pm_set_governor(PM_DOMAIN_KPU, PM_GOVERNOR_PERFORMANCE);
    app_log(APP_NAME_STR ": pm KPU governor->performance rc=%d (before profile=%d)\n", rc, cur);
}

/* ============================ pid ============================ */

static void write_pid(void)
{
    int fd = open(APP_PID_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0)
        return;
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "%d\n", (int)getpid());
    if (n > 0)
        (void)write(fd, buf, (size_t)n);
    close(fd);
}

/* ============================ main ============================ */

int main(int argc, char **argv)
{
    g_cfg = g_default;

    int rc = parse_args(argc, argv);
    if (rc != 0) {
        usage();
        return rc == 2 ? 0 : 1;
    }

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    signal(SIGINT, on_term);
    signal(SIGTERM, on_term);
    signal(SIGUSR1, on_usr1);
    signal(SIGUSR2, on_usr2);

    {
        char cmdline[256];
        int off = 0;
        for (int i = 0; i < argc && off < (int)sizeof(cmdline) - 2; ++i)
            off += snprintf(cmdline + off, sizeof(cmdline) - (size_t)off,
                            "%s%s", i ? " " : "", argv[i]);
        app_log(APP_NAME_STR ": start pid=%d argv=[%s]\n", (int)getpid(), cmdline);
    }

    app_log(APP_NAME_STR ": probe request csi=%d %ux%u@%d (适配表精确命中要求)\n",
            g_cfg.csi, g_cfg.acq_w, g_cfg.acq_h, g_cfg.probe_fps);
    app_log(APP_NAME_STR ": subsystems vision=%s record=%s rec_fps=%d detector=%s "
            "vicap_mode=%s rec_mode=%s dump_timeout=%dms\n",
            g_cfg.vision_on ? "on" : "off", g_cfg.record_on ? "on" : "off", g_cfg.rec_fps,
            g_cfg.detector, g_cfg.vicap_online ? "online" : "offline",
            g_cfg.rec_mode ? "bind" : "shared", g_cfg.dump_timeout_ms);

    if (g_cfg.pm_perf)
        pm_lock_performance();

    /*
     * 采集/录像的编排顺序照抄官方 sample_venc：
     *   vicap_setup -> record_bind(VICAP CHN1 -> VENC) -> vicap_start -> record_run
     * 绑定必须在 start_stream 之前完成。
     */
    if (vicap_setup() != 0) {
        app_log(APP_NAME_STR ": vicap setup failed, exit\n");
        return 1;
    }

    int rec_bound = 0;
    if (g_cfg.record_on)
        rec_bound = (record_bind() == 0);

    if (vicap_start() != 0) {
        app_log(APP_NAME_STR ": vicap start failed, exit\n");
        vicap_stop();
        return 1;
    }

    if (g_cfg.vision_on && vision_start() != 0)
        app_log(APP_NAME_STR ": vision start failed (录像不受影响)\n");

    if (g_cfg.record_on) {
        if (!rec_bound) {
            app_log(APP_NAME_STR ": 录像已禁用（绑定失败），视觉继续工作\n");
        } else if (record_run() != 0) {
            app_log(APP_NAME_STR ": record start failed (视觉不受影响)\n");
        }
    } else {
        app_log(APP_NAME_STR ": recorder disabled (--record off：未配置 CHN1/VENC)\n");
    }

    control_start();
    write_pid();
    app_log(APP_NAME_STR ": running (vision=%s record=%s)\n",
            vision_enabled() ? "on" : "off", record_enabled() ? "on" : "off");

    uint64_t last_status_us = mono_us();
    while (g_running) {
        usleep(200000);

        if (g_sig_rec_on) {
            g_sig_rec_on = 0;
            record_set_enabled(1);
        }
        if (g_sig_rec_off) {
            g_sig_rec_off = 0;
            record_set_enabled(0);
        }

        uint64_t now = mono_us();
        if (now - last_status_us >= (uint64_t)g_cfg.status_period_s * 1000000ull) {
            last_status_us = now;
            char line[1400];
            control_status_line(line, sizeof(line));
            app_log("%s", line);
            /* 探针行（照旧工程口径）：rgb/lab + 亮度 + 阈值，用来判断画面与调阈值 */
            control_probe_line(line, sizeof(line));
            if (line[0])
                app_log("%s", line);
        }
    }

    app_log(APP_NAME_STR ": shutting down...\n");
    control_stop();
    vision_stop();
    record_deinit();    /* 解绑定 + 停编码器 + 收会话（须在 vicap_stop 之前） */
    mpp_shutdown();     /* 所有工作线程已 join，这里才允许 munmap */
    vicap_stop();

    {
        mpp_stats_t ms;
        mpp_get_stats(&ms);
        app_log(APP_NAME_STR ": exit (mpp calls=%llu contended=%llu maxhold=%lluus maps=%u)\n",
                (unsigned long long)ms.calls, (unsigned long long)ms.contended,
                (unsigned long long)ms.max_hold_us, ms.maps);
    }
    return 0;
}
