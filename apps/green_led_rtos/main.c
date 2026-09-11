/*
 * green_led_rtos: VICAP 唯一持有者 + 绿灯识别（默认常录录像）。
 *
 * 采集线程从 VICAP CHN0 抓取 640x480 YUV 帧放入单槽信箱，识别线程消费
 * 最新帧做 YUV->RGB 像素级"绿灯重心"检测，结果写入 detection_result_t 并
 * 同步给录像模块（rec_note_result）。
 *
 * 录像（recorder.c，解耦模块）：
 *   - 默认常录：启动后自动开录，每段 256MB 自动滚动续录（REC_ROLL_BYTES），
 *     recording/ 总容量上限默认 8GB（REC_DEFAULT_CAP_BYTES），超限删最旧；
 *   - 采集线程每帧调用 rec_feed_frame()（仅录像开启时拷贝入队，满则丢，
 *     不等待编码器/SD，不影响识别帧率）；
 *   - 外部暂停/恢复：rec_stop/rec_start API、SIGUSR2(暂停)/SIGUSR1(恢复)、
 *     控制 socket /sdcard/app/green_led_rtos.ctl（start/stop/status/cap/dir）。
 *   - 输出 /sdcard/app/recording/rec_XXXX.h264 + 同名前缀 .csv（逐帧识别结果）。
 *
 * 历史：曾把处理后画面经 VENC/JPEG + Unix socket 交给 usb_cam_stream 走
 * USB CDC ACM 回传主机实时预览，该方案存在恶性 bug 已放弃并删除。
 *
 * 单消费者信箱说明：识别线程取走帧(清空信箱)后采集线程即可抓取下一帧，
 * 二者可能同时处理相邻两帧；VICAP dump buffer 在 release 前不会被驱动
 * 复用，因此不会读到被覆盖的帧内容。
 */
#define main vendor_sample_main
#include "/mnt/mydata/kRTOSSDK/src/rtsmart/examples/mpp/sample_vicap_sensor/sample_vicap_sensor.c"
#undef main

#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "recorder.h"

#define DEV VICAP_DEV_ID_0
#define CSI 2
#define DETECT_FPS 120
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define PIXEL_STEP_X 3
#define PIXEL_STEP_Y 2
#define GREEN_DELTA 20
#define GREEN_MIN 40

/* Full-frame ROI preserves the existing detector behaviour. Change these
 * defaults when a narrower search region is desired. */
#define ROI_X 0
#define ROI_Y 0
#define ROI_W 0
#define ROI_H 0

static volatile int app_running = 1;
static pthread_mutex_t frame_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t frame_ready = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t result_lock = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    k_video_frame_info frame;
    uint64_t seq;     /* 帧序号：识别/录像共用，用于结果与画面的逐帧对齐 */
    uint64_t mono_us; /* 采集时刻（CLOCK_MONOTONIC, us） */
} shared_item_t;

static shared_item_t shared_frame;
static int have_shared_frame = 0; /* 信箱内有未消费帧 */
static uint64_t frame_seq_counter = 0;

typedef struct {
    int32_t center_x;
    int32_t center_y;
    uint32_t roi_x, roi_y, roi_w, roi_h;
    uint32_t fps;
    uint64_t seq;
} detection_result_t;

/* 识别输出。录像模块通过 rec_note_result() 取走逐帧结果（去重维护在模块内），
 * 本结构保留作为进程内结果出口。 */
static detection_result_t result = {
    .center_x = -1, .center_y = -1,
    .roi_x = ROI_X, .roi_y = ROI_Y, .roi_w = 0, .roi_h = 0,
    .fps = 0, .seq = 0
};

static void green_signal(int sig)
{
    (void)sig;
    app_running = 0;
    pthread_cond_broadcast(&frame_ready);
}

static void rec_start_signal(int sig)
{
    (void)sig;
    rec_signal_start(); /* 仅置位标志，由录像内部控制线程执行 */
}

static void rec_stop_signal(int sig)
{
    (void)sig;
    rec_signal_stop();
}

static void log_line(const char *msg)
{
    static const char *paths[] = {
        "/sdcard/app/logs/green_led_rtos.log",
    };
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i) {
        int fd = open(paths[i], O_WRONLY | O_CREAT | O_APPEND, 0666);
        if (fd < 0)
            continue;
        size_t len = strlen(msg);
        ssize_t written = write(fd, msg, len);
        fsync(fd);
        close(fd);
        if (written == (ssize_t)len)
            return;
    }
    fprintf(stderr, "%s", msg);
}

static k_s32 green_vicap_init(void)
{
    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;
    memset(&dev_attr, 0, sizeof(dev_attr));
    memset(&chn_attr, 0, sizeof(chn_attr));
    if (!g_sensor_info.sensor_name) return -1;
    dev_attr.acq_win.width = g_sensor_info.width;
    dev_attr.acq_win.height = g_sensor_info.height;
    dev_attr.mode = VICAP_WORK_ONLINE_MODE;
    dev_attr.buffer_num = 6;
    dev_attr.buffer_size = VB_ALIGN_UP(g_sensor_info.width * g_sensor_info.height * 2, 4096);
    dev_attr.buffer_pool_id = VB_INVALID_POOLID;
    dev_attr.pipe_ctrl.data = 0xFFFFFFFF;
    dev_attr.pipe_ctrl.bits.ae_enable = K_FALSE;
    dev_attr.pipe_ctrl.bits.awb_enable = K_FALSE;
    dev_attr.pipe_ctrl.bits.ahdr_enable = K_FALSE;
    dev_attr.pipe_ctrl.bits.dnr3_enable = K_FALSE;
    memcpy(&dev_attr.sensor_info, &g_sensor_info, sizeof(g_sensor_info));
    k_s32 ret = kd_mpi_vicap_set_dev_attr(VICAP_DEV_ID_0, dev_attr);
    if (ret != K_SUCCESS) return ret;
    chn_attr.out_win.width = FRAME_WIDTH;
    chn_attr.out_win.height = FRAME_HEIGHT;
    chn_attr.crop_win = dev_attr.acq_win;
    chn_attr.scale_win = chn_attr.out_win;
    chn_attr.crop_enable = K_FALSE;
    chn_attr.scale_enable = (FRAME_WIDTH != dev_attr.acq_win.width ||
                             FRAME_HEIGHT != dev_attr.acq_win.height);
    chn_attr.chn_enable = K_TRUE;
    chn_attr.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    chn_attr.buffer_num = 6;
    chn_attr.buffer_size = VB_ALIGN_UP(FRAME_WIDTH * FRAME_HEIGHT * 3 / 2, 4096);
    chn_attr.alignment = 12;
    chn_attr.buffer_pool_id = VB_INVALID_POOLID;
    ret = kd_mpi_vicap_set_chn_attr(VICAP_DEV_ID_0, VICAP_CHN_ID_0, chn_attr);
    if (ret != K_SUCCESS) return ret;
    return kd_mpi_vicap_init(VICAP_DEV_ID_0);
}

static void update_result(int32_t cx, int32_t cy, uint32_t fps, uint64_t seq)
{
    detection_result_t next;
    next.center_x = cx;
    next.center_y = cy;
    next.roi_x = ROI_X;
    next.roi_y = ROI_Y;
    next.roi_w = ROI_W ? ROI_W : FRAME_WIDTH;
    next.roi_h = ROI_H ? ROI_H : FRAME_HEIGHT;
    next.fps = fps;
    next.seq = seq;
    pthread_mutex_lock(&result_lock);
    result = next;
    pthread_mutex_unlock(&result_lock);
}

static void *capture_thread(void *arg)
{
    (void)arg;
    while (app_running) {
        k_video_frame_info frame;
        k_s32 ret = kd_mpi_vicap_dump_frame(DEV, VICAP_CHN_ID_0, VICAP_DUMP_YUV, &frame, 100);
        if (ret != K_SUCCESS)
            continue;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t mono_us = (uint64_t)ts.tv_sec * 1000000ull +
                           (uint64_t)(ts.tv_nsec / 1000);
        pthread_mutex_lock(&frame_lock);
        while (app_running && have_shared_frame)
            pthread_cond_wait(&frame_ready, &frame_lock); /* 上一帧尚未被识别线程取走 */
        if (!app_running) {
            pthread_mutex_unlock(&frame_lock);
            kd_mpi_vicap_dump_release(DEV, VICAP_CHN_ID_0, &frame);
            break;
        }
        shared_frame.frame = frame;
        shared_frame.seq = ++frame_seq_counter;
        shared_frame.mono_us = mono_us;
        have_shared_frame = 1;
        pthread_cond_broadcast(&frame_ready);
        pthread_mutex_unlock(&frame_lock);

        /* 录像解耦入口：仅当录像开启时拷帧入队（满则丢），本调用有界，
         * 绝不等编码器/SD，因此不影响识别。本帧 buffer 由本线程持有，
         * 调用期间有效（识别线程只读同一 buffer）。 */
        if (rec_is_active()) {
            uint32_t stride = frame.v_frame.stride[0] ? frame.v_frame.stride[0]
                                                      : frame.v_frame.width;
            rec_feed_frame(frame.v_frame.phys_addr[0], stride,
                           frame.v_frame.width, frame.v_frame.height,
                           shared_frame.seq, mono_us);
        }
    }
    return NULL;
}

static void *detect_thread(void *arg)
{
    (void)arg;
    struct timespec last = {0, 0};
    uint32_t count_frames = 0;
    while (app_running) {
        k_video_frame_info frame;
        uint64_t seq;
        pthread_mutex_lock(&frame_lock);
        while (app_running && !have_shared_frame)
            pthread_cond_wait(&frame_ready, &frame_lock);
        if (!app_running) {
            pthread_mutex_unlock(&frame_lock);
            break;
        }
        frame = shared_frame.frame;
        seq = shared_frame.seq;
        have_shared_frame = 0;
        pthread_cond_broadcast(&frame_ready); /* 采集线程可投递下一帧 */
        pthread_mutex_unlock(&frame_lock);

        uint32_t stride = frame.v_frame.stride[0] ? frame.v_frame.stride[0]
                                                  : frame.v_frame.width;
        uint32_t bytes = stride * frame.v_frame.height * 3u / 2u;
        uint8_t *rgb = (uint8_t *)kd_mpi_sys_mmap(frame.v_frame.phys_addr[0], bytes);
        int32_t cx = -1, cy = -1;
        uint64_t sx = 0, sy = 0, pixels = 0;
        uint32_t rx = ROI_X, ry = ROI_Y;
        uint32_t rw = ROI_W ? ROI_W : frame.v_frame.width;
        uint32_t rh = ROI_H ? ROI_H : frame.v_frame.height;
        if (rx >= frame.v_frame.width) rx = 0;
        if (ry >= frame.v_frame.height) ry = 0;
        if (rw > frame.v_frame.width - rx) rw = frame.v_frame.width - rx;
        if (rh > frame.v_frame.height - ry) rh = frame.v_frame.height - ry;
        if (rgb) {
            for (uint32_t y = ry; y < ry + rh; y += PIXEL_STEP_Y) {
                for (uint32_t x = rx; x < rx + rw; x += PIXEL_STEP_X) {
                    uint32_t i = y * stride + x;
                    uint8_t yy = rgb[i];
                    uint32_t uv = stride * frame.v_frame.height + (y / 2u) * stride + (x / 2u) * 2u;
                    int u = (int)rgb[uv] - 128, v = (int)rgb[uv + 1] - 128;
                    int r = (int)yy + (int)(1.402 * v);
                    int g = (int)yy - (int)(0.344 * u) - (int)(0.714 * v);
                    int b = (int)yy + (int)(1.772 * u);
                    if (g > GREEN_MIN && g > r + GREEN_DELTA &&
                        g > b + GREEN_DELTA) {
                        sx += x;
                        sy += y;
                        pixels++;
                    }
                }
            }
            kd_mpi_sys_munmap(rgb, bytes);
            if (pixels) {
                cx = (int32_t)(sx / pixels);
                cy = (int32_t)(sy / pixels);
            }
        } else {
            static unsigned map_failures;
            if ((++map_failures % 100u) == 1u)
                log_line("green_led_rtos: RGB frame mmap failed\n");
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        count_frames++;
        static uint32_t fps_value = 0;
        if (last.tv_sec == 0) {
            last = now;
        } else {
            double dt = (double)(now.tv_sec - last.tv_sec) +
                        (double)(now.tv_nsec - last.tv_nsec) / 1e9;
            if (dt >= 1.0) {
                fps_value = (uint32_t)(count_frames / dt);
                count_frames = 0;
                last = now;
                /* 临时状态行：每秒一条，便于板端日志确认识别正常；
                 * 后续任务接入录像/结果输出后可移除。 */
                char line[96];
                snprintf(line, sizeof(line),
                         "green_led_rtos: center=(%d,%d) fps=%u\n",
                         cx, cy, fps_value);
                log_line(line);
            }
        }
        update_result(cx, cy, fps_value, seq);
        rec_note_result(seq, cx, cy, fps_value);
        kd_mpi_vicap_dump_release(DEV, VICAP_CHN_ID_0, &frame);
    }
    return NULL;
}

int main(void)
{
    pthread_t capture, detect;
    int capture_started = 0, detect_started = 0;
    k_u32 sensor_width = FRAME_WIDTH, sensor_height = FRAME_HEIGHT;

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    signal(SIGINT, green_signal);
    signal(SIGTERM, green_signal);
    signal(SIGUSR1, rec_start_signal); /* 外部开启录像 */
    signal(SIGUSR2, rec_stop_signal);  /* 外部停止录像 */
    mkdir("/sdcard/app/logs", 0777);
    log_line("green_led_rtos: start\n");

    k_s32 ret = get_sensor_resolution(CSI, &sensor_width, &sensor_height, NULL,
                                      FRAME_WIDTH, FRAME_HEIGHT, DETECT_FPS);
    if (ret != 0) {
        char msg[128];
        snprintf(msg, sizeof(msg), "green_led_rtos: sensor probe failed ret=%d csi=%d\n", ret, CSI);
        log_line(msg);
        return -1;
    }
    ret = sample_vb_init();
    if (ret != K_SUCCESS) {
        char msg[128];
        snprintf(msg, sizeof(msg), "green_led_rtos: VB init failed ret=%d\n", ret);
        log_line(msg);
        return -1;
    }
    /* CHN0 提供 640x480 YUV 帧给识别线程。 */
    ret = green_vicap_init();
    if (ret != K_SUCCESS) {
        char msg[128];
        snprintf(msg, sizeof(msg), "green_led_rtos: VICAP init failed ret=%d sensor=%ux%u\n", ret, sensor_width, sensor_height);
        log_line(msg);
        kd_mpi_vb_exit();
        return -1;
    }

    ret = kd_mpi_vicap_start_stream(DEV);
    if (ret != K_SUCCESS) {
        char msg[128];
        snprintf(msg, sizeof(msg), "green_led_rtos: start stream failed ret=%d\n", ret);
        log_line(msg);
        kd_mpi_vicap_deinit(DEV);
        kd_mpi_vb_exit();
        return -1;
    }

    printf("green_led_rtos: detector owns VICAP; frame=%ux%u\n",
           sensor_width, sensor_height);
    log_line("green_led_rtos: VICAP started; detection active\n");

    /* 录像模块（解耦）：目录/容量/码率可在此调整；失败不影响识别 */
    {
        recorder_config_t rcfg;
        memset(&rcfg, 0, sizeof(rcfg));
        rcfg.record_dir = REC_DEFAULT_DIR;
        rcfg.cap_bytes = REC_DEFAULT_CAP_BYTES;
        rcfg.width = FRAME_WIDTH;
        rcfg.height = FRAME_HEIGHT;
        rcfg.nominal_fps = DETECT_FPS;
        rcfg.bitrate_kbps = 12000;
        if (rec_init(&rcfg) == 0) {
            char msg[160];
            snprintf(msg, sizeof(msg),
                     "green_led_rtos: recorder ready (auto-record on) "
                     "dir=%s cap=%llu\n",
                     REC_DEFAULT_DIR,
                     (unsigned long long)REC_DEFAULT_CAP_BYTES);
            log_line(msg);
        } else {
            log_line("green_led_rtos: recorder init failed (detection only)\n");
        }
    }
    /* pid 文件：外部进程可用 kill -USR1/-USR2 <pid> 控制录像 */
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
    }
    if (capture_started) pthread_join(capture, NULL);
    if (detect_started) pthread_join(detect, NULL);
    rec_deinit(); /* 停会话、回收编码器与 VB（须在 kd_mpi_vb_exit 前） */
    kd_mpi_vicap_stop_stream(DEV);
    kd_mpi_vicap_deinit(DEV);
    kd_mpi_vb_exit();
    log_line("green_led_rtos: exit\n");
    return 0;
}
