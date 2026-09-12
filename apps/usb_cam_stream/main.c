/*
 * usb_cam_stream —— 通过 Type-C USB(CDC ACM) 把相机画面发送给主机
 *
 * 管线（移植自 SDK 官方示例 sample_uvc_dev_vicap，把"发 UVC"换成"写串口"）:
 *   VICAP 采集 NV12 -> kd_mpi_venc_send_frame(JPEG) -> 取码流
 *   -> 打包成 [magic+len+w+h | JPEG] 写入 /dev/ttyGS0（RT-Smart 的 USB 虚拟串口）
 *
 * 主机端配合 scripts/usb_cam_viewer.py 收帧显示。
 *
 * 用法:
 *   /sdcard/app/usb_cam_stream [-c <csi 0-2>] [-w <宽>] [-h <高>]
 *                              [-q <JPEG质量>] [-f <目标帧率>] [-d <串口设备>]
 *   默认: csi=2, 640x480, q=75, 15fps, /dev/ttyGS0
 *
 * 说明:
 *   - USB 串口设备名以 RT-Smart 固件实际注册为准（ttyGS0 / ttyGS1），
 *     打开失败会打印错误并可用 -d 指定其它设备。
 *   - 阻塞写：主机端没在读时 USB 会背压，恢复读取后继续，属正常现象。
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "mpi_sensor_api.h"
#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_venc_api.h"
#include "mpi_vicap_api.h"

/* 运行状态日志：open/write/fsync 直写日志文件（RT-Smart 上 stdio 输出走控制台，
 * 不可靠），不经过重定向，也不经过 FILE* 缓冲。 */
static void log_raw(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    int fd = open("/sdcard/app/logs/usb_cam_stream.log",
                  O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0)
        return;
    ssize_t w = write(fd, buf, (size_t)n);
    fsync(fd);
    close(fd);
    (void)w;
}
#define LOG(...) log_raw(__VA_ARGS__)

/* ---------------- 帧协议 ---------------- */
#define FRAME_MAGIC0 'L'
#define FRAME_MAGIC1 'C'
#define FRAME_MAGIC2 'J'
#define FRAME_MAGIC3 'P'
#define FRAME_HEADER_LEN 16

/* ---------------- 参数 ---------------- */
static int g_csi = 2;
static int g_width = 640;
static int g_height = 480;
static int g_qfactor = 75;
static int g_fps = 15;
static const char *g_serial_dev = "/dev/ttyGS0";

/* Lushanpi sensor runs at 1920x1080; VICAP scales to requested output. */
#define ISP_WIDTH 1920
#define ISP_HEIGHT 1080

#define VENC_CH_ID_0 (0)
static k_vicap_dev vicap_dev = VICAP_DEV_ID_0;
/* Lushanpi maps the camera CSI connector to VICAP device 0. */

static volatile bool g_app_run = true;
static k_s32 venc_pool_id = VB_INVALID_POOLID;
static int serial_fd = -1;

static void handle_signal(int sig)
{
    (void)sig;
    g_app_run = false;
}

/* ---------------- 小工具 ---------------- */
static void put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}

/* 把整块数据写满 fd（串口可能短写，循环补齐） */
static int write_all(int fd, const uint8_t *buf, size_t len)
{
    size_t done = 0;
    while (done < len) {
        ssize_t n = write(fd, buf + done, len - done);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "usb_cam_stream: write failed: %s\n", strerror(errno));
            return -1;
        }
        done += (size_t)n;
    }
    return 0;
}

/* 发送一帧: [LCJP][len][w][h] + JPEG */
static int send_frame(int fd, const uint8_t *jpeg, uint32_t jpeg_len,
                      uint32_t width, uint32_t height)
{
    uint8_t hdr[FRAME_HEADER_LEN];
    hdr[0] = FRAME_MAGIC0;
    hdr[1] = FRAME_MAGIC1;
    hdr[2] = FRAME_MAGIC2;
    hdr[3] = FRAME_MAGIC3;
    put_u32le(hdr + 4, jpeg_len);
    put_u32le(hdr + 8, width);
    put_u32le(hdr + 12, height);
    if (write_all(fd, hdr, sizeof(hdr)) != 0)
        return -1;
    if (write_all(fd, jpeg, jpeg_len) != 0)
        return -1;
    return 0;
}

/* ---------------- VB ---------------- */
static k_s32 sample_vb_init(void)
{
    k_vb_config config = {0};
    config.max_pool_cnt = 64;
    k_s32 ret = kd_mpi_vb_set_config(&config);
    if (ret) {
        LOG("ERROR: kd_mpi_vb_set_config failed, ret=%d\n", ret);
        return ret;
    }
    k_vb_supplement_config supplement_config = {0};
    supplement_config.supplement_config |= VB_SUPPLEMENT_JPEG_MASK;
    ret = kd_mpi_vb_set_supplement_config(&supplement_config);
    if (ret) {
        LOG("ERROR: kd_mpi_vb_set_supplement_config failed, ret=%d\n", ret);
        return ret;
    }
    ret = kd_mpi_vb_init();
    if (ret)
        LOG("ERROR: kd_mpi_vb_init failed, ret=%d\n", ret);
    return ret;
}

/* ---------------- VICAP ---------------- */
static k_s32 sample_vicap_init(void)
{
    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;
    k_vicap_sensor_info sensor_info;
    k_vicap_probe_config probe_cfg;
    k_vicap_sensor_type sensor_type;

    memset(&sensor_info, 0, sizeof(sensor_info));
    probe_cfg.csi_num = (k_vicap_dev)g_csi;
    probe_cfg.width = ISP_WIDTH;
    probe_cfg.height = ISP_HEIGHT;
    probe_cfg.fps = 30;

    if (kd_mpi_sensor_adapt_get(&probe_cfg, &sensor_info) != 0) {
        LOG("ERROR: can't probe sensor on csi %d (%dx%d@30)\n",
               probe_cfg.csi_num, g_width, g_height);
        return -1;
    }
    sensor_type = sensor_info.sensor_type;

    k_s32 ret = kd_mpi_vicap_get_sensor_info(sensor_type, &sensor_info);
    if (ret) {
        LOG("ERROR: kd_mpi_vicap_get_sensor_info failed, ret=%d\n", ret);
        return ret;
    }

    memset(&dev_attr, 0, sizeof(dev_attr));
    dev_attr.acq_win.width = ISP_WIDTH;
    dev_attr.acq_win.height = ISP_HEIGHT;
    dev_attr.mode = VICAP_WORK_OFFLINE_MODE;
    dev_attr.buffer_num = 6;
    dev_attr.buffer_size = VB_ALIGN_UP(ISP_WIDTH * ISP_HEIGHT * 2, 1024);
    dev_attr.buffer_pool_id = VB_INVALID_POOLID;
    memcpy(&dev_attr.sensor_info, &sensor_info, sizeof(k_vicap_sensor_info));

    ret = kd_mpi_vicap_set_dev_attr(vicap_dev, dev_attr);
    if (ret) {
        LOG("ERROR: kd_mpi_vicap_set_dev_attr failed, ret=%d\n", ret);
        return ret;
    }

    memset(&chn_attr, 0, sizeof(chn_attr));
    chn_attr.out_win.width = g_width;
    chn_attr.out_win.height = g_height;
    chn_attr.crop_win = dev_attr.acq_win;
    chn_attr.scale_win = chn_attr.out_win;
    chn_attr.crop_enable = K_FALSE;
    chn_attr.scale_enable = K_TRUE;
    chn_attr.chn_enable = K_TRUE;
    chn_attr.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    chn_attr.buffer_num = 6;
    chn_attr.buffer_size = VB_ALIGN_UP(g_width * g_height * 3 / 2, 4096);
    chn_attr.buffer_pool_id = VB_INVALID_POOLID;
    chn_attr.alignment = 12;

    ret = kd_mpi_vicap_set_chn_attr(vicap_dev, VICAP_CHN_ID_0, chn_attr);
    if (ret) {
        LOG("ERROR: kd_mpi_vicap_set_chn_attr failed, ret=%d\n", ret);
        return ret;
    }

    ret = kd_mpi_vicap_init(vicap_dev);
    if (ret)
        LOG("ERROR: kd_mpi_vicap_init failed, ret=%d\n", ret);
    return ret;
}

/* ---------------- VENC(JPEG) ---------------- */
static k_s32 sample_venc_init(void)
{
    k_s32 poolid;

    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));

    venc_pool_id = VB_INVALID_POOLID;
    poolid = kd_mpi_vb_create_pool_ex(VB_ALIGN_UP(g_width * g_height, 4096), 4,
                                      VB_REMAP_MODE_NOCACHE);
    if (VB_INVALID_POOLID == poolid) {
        LOG("ERROR: create venc pool failed\n");
        return -1;
    }
    if (K_SUCCESS != kd_mpi_venc_attach_vb_pool(VENC_CH_ID_0, poolid)) {
        LOG("ERROR: attach venc pool failed\n");
        kd_mpi_vb_destory_pool(poolid);
        return -2;
    }
    venc_pool_id = poolid;

    attr.venc_attr.type = K_PT_JPEG;
    attr.venc_attr.pic_width = g_width;
    attr.venc_attr.pic_height = g_height;
    attr.rc_attr.rc_mode = K_VENC_RC_MODE_MJPEG_FIXQP;
    attr.rc_attr.mjpeg_fixqp.src_frame_rate = 30;
    attr.rc_attr.mjpeg_fixqp.dst_frame_rate = 30;
    attr.rc_attr.mjpeg_fixqp.q_factor = (k_u32)g_qfactor;

    k_s32 ret = kd_mpi_venc_create_chn(VENC_CH_ID_0, &attr);
    if (ret) {
        LOG("ERROR: kd_mpi_venc_create_chn failed, ret=%d\n", ret);
        return ret;
    }
    ret = kd_mpi_venc_start_chn(VENC_CH_ID_0);
    if (ret) {
        LOG("ERROR: kd_mpi_venc_start_chn failed, ret=%d\n", ret);
        kd_mpi_venc_destroy_chn(VENC_CH_ID_0);
    }
    return ret;
}

/* ---------------- 主循环 ---------------- */
static void sample_video_loop(void)
{
    k_video_frame_info vicap_frame;
    k_venc_chn_status venc_status;
    k_venc_stream venc_stream;
    size_t jpeg_max = (size_t)g_width * g_height * 2;
    uint8_t *jpeg_buf = malloc(jpeg_max);
    unsigned long long frame_count = 0;
    unsigned long long sent_count = 0;
    unsigned long long dump_fail_count = 0;
    unsigned int fps_interval_us = 1000000u / (unsigned int)(g_fps > 0 ? g_fps : 15);
    struct timespec tlast = {0, 0};

    if (!jpeg_buf) {
        LOG("ERROR: malloc jpeg buffer failed\n");
        return;
    }

    LOG("usb_cam_stream: pipeline started (%dx%d, q=%d, ~%dfps -> %s)\n",
           g_width, g_height, g_qfactor, g_fps, g_serial_dev);
    clock_gettime(CLOCK_MONOTONIC, &tlast);

    while (g_app_run) {
        /* 0. 每 2 秒报告一次状态（防止静默卡死） */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if ((long)(now.tv_sec - tlast.tv_sec) >= 2) {
            LOG("usb_cam_stream: tick: dumped=%llu sent=%llu dump_fail=%llu\n",
                frame_count, sent_count, dump_fail_count);
            tlast = now;
        }

        /* 1. 采集一帧 NV12 */
        memset(&vicap_frame, 0, sizeof(vicap_frame));
        k_s32 ret = kd_mpi_vicap_dump_frame(vicap_dev, VICAP_CHN_ID_0,
                                            VICAP_DUMP_YUV, &vicap_frame, 1000);
        if (ret != K_SUCCESS) {
            dump_fail_count++;
            if (dump_fail_count % 20 == 1)
                LOG("usb_cam_stream: vicap dump fail ret=%d (fail#%llu)\n",
                    (int)ret, dump_fail_count);
            usleep(10000);
            continue;
        }
        frame_count++;

        /* 2. 送 VENC 编码为 JPEG */
        ret = kd_mpi_venc_send_frame(VENC_CH_ID_0, &vicap_frame, 1000);
        kd_mpi_vicap_dump_release(vicap_dev, VICAP_CHN_ID_0, &vicap_frame);
        if (ret != K_SUCCESS) {
            usleep(10000);
            continue;
        }

        /* 3. 取编码码流 */
        memset(&venc_status, 0, sizeof(venc_status));
        if (kd_mpi_venc_query_status(VENC_CH_ID_0, &venc_status) != K_SUCCESS) {
            usleep(10000);
            continue;
        }
        memset(&venc_stream, 0, sizeof(venc_stream));
        venc_stream.pack_cnt = (venc_status.cur_packs > 0) ? venc_status.cur_packs : 1;
        venc_stream.pack = malloc(sizeof(k_venc_pack) * venc_stream.pack_cnt);
        if (!venc_stream.pack) {
            usleep(10000);
            continue;
        }
        ret = kd_mpi_venc_get_stream(VENC_CH_ID_0, &venc_stream, 1000);
        if (ret != K_SUCCESS) {
            free(venc_stream.pack);
            usleep(10000);
            continue;
        }

        /* 4. 把各 pack 拷贝成连续 JPEG */
        size_t total = 0;
        for (int i = 0; i < venc_stream.pack_cnt; i++) {
            if (total + venc_stream.pack[i].len > jpeg_max) {
                LOG("WARN: jpeg too large, truncated\n");
                break;
            }
            k_u8 *src = (k_u8 *)kd_mpi_sys_mmap(venc_stream.pack[i].phys_addr,
                                                venc_stream.pack[i].len);
            if (src) {
                memcpy(jpeg_buf + total, src, venc_stream.pack[i].len);
                total += venc_stream.pack[i].len;
                kd_mpi_sys_munmap(src, venc_stream.pack[i].len);
            }
        }
        kd_mpi_venc_release_stream(VENC_CH_ID_0, &venc_stream);
        free(venc_stream.pack);

        /* 5. 发给主机(USB 虚拟串口) */
        if (total > 0) {
            if (send_frame(serial_fd, jpeg_buf, (uint32_t)total,
                           (uint32_t)g_width, (uint32_t)g_height) != 0) {
                /* 串口异常(主机断开等): 稍等后重试，不退出 */
                usleep(100000);
                continue;
            }
            sent_count++;
        }

        frame_count++;
        if (sent_count % 25 == 0)
            LOG("usb_cam_stream: sent %llu frames\n", sent_count);

        /* 6. 节流到目标帧率 */
        usleep(fps_interval_us);
    }

    free(jpeg_buf);
    LOG("usb_cam_stream: loop finished\n");
}

/* ---------------- 参数解析 ---------------- */
static void print_usage(const char *name)
{
    LOG("Usage: %s [options]\n", name);
    LOG("  -c <0-2>    CSI 设备号 (default 2)\n");
    LOG("  -w <px>     图像宽 (default 640)\n");
    LOG("  -h <px>     图像高 (default 480)\n");
    LOG("  -q <0-100>  JPEG 质量 (default 75)\n");
    LOG("  -f <fps>    目标帧率 (default 15)\n");
    LOG("  -d <dev>    USB 串口设备 (default /dev/ttyGS0)\n");
}

int main(int argc, char **argv)
{
    /* 立即生效且所见即所得：stdout/stderr 均无缓冲（日志落盘不受 stdio 缓冲影响） */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    mkdir("/sdcard/app/logs", 0777);

    LOG("usb_cam_stream: main enter argc=%d (default csi=%d %dx%d q=%d fps=%d dev=%s)\n",
        argc, g_csi, g_width, g_height, g_qfactor, g_fps, g_serial_dev);

    int opt;
    while ((opt = getopt(argc, argv, "c:w:h:q:f:d:")) != -1) {
        switch (opt) {
        case 'c':
            g_csi = atoi(optarg);
            break;
        case 'w':
            g_width = atoi(optarg);
            break;
        case 'h':
            g_height = atoi(optarg);
            break;
        case 'q':
            g_qfactor = atoi(optarg);
            break;
        case 'f':
            g_fps = atoi(optarg);
            break;
        case 'd':
            g_serial_dev = optarg;
            break;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }
    if (g_width < 160 || g_width > 1920 || g_height < 120 || g_height > 1080 ||
        g_qfactor < 1 || g_qfactor > 100 || g_fps < 1 || g_fps > 30) {
        print_usage(argv[0]);
        return 1;
    }

    vicap_dev = VICAP_DEV_ID_0; /* CSI selects the sensor; the VICAP device remains 0 on Lushanpi. */
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    /* 打开 USB 虚拟串口（数据面，不是调试串口）。
     * 固件注册名可能是 ttyGS0/ttyGS1，逐个尝试并把结果写日志。 */
    {
        const char *candidates[] = { g_serial_dev, "/dev/ttyGS0", "/dev/ttyGS1" };
        for (int i = 0; i < (int)(sizeof(candidates) / sizeof(candidates[0])); i++) {
            serial_fd = open(candidates[i], O_WRONLY);
            if (serial_fd >= 0) {
                LOG("usb_cam_stream: opened %s\n", candidates[i]);
                break;
            }
            LOG("usb_cam_stream: open %s failed: %s\n",
                candidates[i], strerror(errno));
        }
    }
    if (serial_fd < 0) {
        LOG("usb_cam_stream: no USB serial device available; "
            "固件需含 CDC ACM 类(通常默认开启)\n");
        return 2;
    }

    if (sample_vb_init() != K_SUCCESS)
        goto cleanup_serial;
    if (sample_vicap_init() != K_SUCCESS)
        goto cleanup_vb;
    if (sample_venc_init() != K_SUCCESS)
        goto cleanup_vicap;

    if (kd_mpi_vicap_start_stream(vicap_dev) != K_SUCCESS) {
        LOG("ERROR: kd_mpi_vicap_start_stream failed\n");
        goto cleanup_venc;
    }

    sample_video_loop();

    kd_mpi_vicap_stop_stream(vicap_dev);
cleanup_venc:
    LOG("usb_cam_stream: venc cleanup\n");
    kd_mpi_venc_stop_chn(VENC_CH_ID_0);
    kd_mpi_venc_destroy_chn(VENC_CH_ID_0);
    kd_mpi_venc_detach_vb_pool(VENC_CH_ID_0);
    if (venc_pool_id != VB_INVALID_POOLID)
        kd_mpi_vb_destory_pool(venc_pool_id);
cleanup_vicap:
    LOG("usb_cam_stream: vicap cleanup\n");
    kd_mpi_vicap_deinit(vicap_dev);
cleanup_vb:
    LOG("usb_cam_stream: vb cleanup\n");
    kd_mpi_vb_exit();
cleanup_serial:
    if (serial_fd >= 0)
        close(serial_fd);
    LOG("usb_cam_stream: exit\n");
    return 0;
}
