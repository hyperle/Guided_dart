/*
 * control —— 实现
 */
#include "control.h"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include "app.h"
#include "record.h"
#include "vision.h"

static volatile int g_run;
static int          g_listen_fd = -1;
static pthread_t    g_tid;
static int          g_started;

static void report_detector(char *buf, size_t n);

/* ============================ 状态行 ============================ */

void control_status_line(char *buf, size_t n)
{
    vision_stats_t vs;
    record_stats_t rs;
    mpp_stats_t    ms;
    result_t       r;

    vision_get_stats(&vs);
    record_get_stats(&rs);
    mpp_get_stats(&ms);
    result_get(&r);

    /* 健康检查行：mpp 的三个数（contended/maxhold/maps）是「三条铁律」的自证 */
    snprintf(buf, n,
             APP_NAME_STR ": status vis=%s fps=%u dumped=%llu noframe=%llu fail=%llu "
             "drop=%llu center=(%d,%d) miss=%llu det=%u/%uus lat=%u/%uus(all %uus) | "
             "rec=%s auto=%d sess=%u fps=%u streams=%llu written=%llu disc=%llu "
             "bytes=%llu | mpp calls=%llu contended=%llu maxhold=%lluus maps=%u\n",
             vision_enabled() ? "on" : "off", vs.fps,
             (unsigned long long)vs.dumped, (unsigned long long)vs.noframe,
             (unsigned long long)vs.dump_fail, (unsigned long long)vs.dropped,
             r.center_x, r.center_y, (unsigned long long)vs.misses,
             vs.t_detect_avg_us, vs.t_detect_max_us,
             vs.t_lat_avg_us, vs.t_lat_max_us, vs.t_lat_max_all_us,
             rs.active ? "on" : "off", rs.enabled, rs.session_index, rs.real_fps,
             (unsigned long long)rs.streams, (unsigned long long)rs.written,
             (unsigned long long)rs.discarded, (unsigned long long)rs.bytes,
             (unsigned long long)ms.calls, (unsigned long long)ms.contended,
             (unsigned long long)ms.max_hold_us, ms.maps);

    /* 异步后端（kpu）的耗时分解：ai2d / kpu / 后处理 / 端到端延迟 / 流水帧率 */
    size_t used = strlen(buf);
    if (used + 2 < n) {
        char extra[640];
        extra[0] = '\0';
        vision_backend_stats(extra, sizeof(extra));
        if (extra[0])
            snprintf(buf + used, n - used, "%s", extra);
    }
}

void control_probe_line(char *buf, size_t n)
{
    report_detector(buf, n);
}

static void report_detector(char *buf, size_t n)
{
    vision_stats_t vs;
    detect_out_t   d;
    vision_get_stats(&vs);
    int have = vision_get_last(&d);
    if (!have)
        memset(&d, 0, sizeof(d));

    snprintf(buf, n,
             APP_NAME_STR ": probe rgb=(%d,%d,%d) lab=(%d,%d,%d) at (%d,%d) "
             "y=[%d,%d,%u] green_px=%u blobs=%u px=%u scan=%s | "
             "thr L>=%d A<=%d B>=%d step=(%u,%u) roi=(%u,%u,%u,%u) pix_min=%u | "
             "det=%u/%uus lat=%u/%uus(all %uus)\n",
             d.probe_r, d.probe_g, d.probe_b, d.probe_L, d.probe_A, d.probe_B,
             d.probe_x, d.probe_y, d.y_min, d.y_max, d.y_mean,
             d.green_px, d.blobs, d.px, d.full_scan ? "full" : "roi",
             g_cfg.lab_l_min, g_cfg.lab_a_max, g_cfg.lab_b_min,
             g_cfg.step_x, g_cfg.step_y,
             g_cfg.roi_x, g_cfg.roi_y, g_cfg.roi_w, g_cfg.roi_h,
             g_cfg.pix_min,
             vs.t_detect_avg_us, vs.t_detect_max_us,
             vs.t_lat_avg_us, vs.t_lat_max_us, vs.t_lat_max_all_us);
}

/* ============================ 命令处理 ============================ */

static void handle_command(const char *line, char *reply, size_t n)
{
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "%s", line);

    char *save = NULL;
    char *tok = strtok_r(cmd, " \t\r\n", &save);
    if (!tok) {
        snprintf(reply, n, "ERR empty\n");
        return;
    }

    if (strcmp(tok, "help") == 0) {
        snprintf(reply, n,
                 "vision on|off|status | record on|off|status|cap <bytes>|dir <path> | "
                 "status | help\n");
        return;
    }

    if (strcmp(tok, "status") == 0) {
        size_t used = 0;
        control_status_line(reply, n);
        used = strlen(reply);
        if (used + 1 < n)
            report_detector(reply + used, n - used);
        return;
    }

    if (strcmp(tok, "vision") == 0) {
        char *a = strtok_r(NULL, " \t\r\n", &save);
        if (!a) {
            snprintf(reply, n, "ERR vision <on|off|status>\n");
        } else if (!vision_is_started()) {
            snprintf(reply, n, "ERR vision 未启动（启动参数为 --vision off）；"
                               "要启用请改成 --vision on 重启进程\n");
        } else if (strcmp(a, "on") == 0) {
            vision_set_enabled(1);
            snprintf(reply, n, "OK vision on\n");
        } else if (strcmp(a, "off") == 0) {
            vision_set_enabled(0);
            snprintf(reply, n, "OK vision off\n");
        } else if (strcmp(a, "status") == 0) {
            vision_stats_t vs;
            vision_get_stats(&vs);
            snprintf(reply, n,
                     "vision=%s fps=%u det_avg=%uus det_max=%uus lat_avg=%uus lat_max=%uus "
                     "frames=%llu drop=%llu dump_fail=%llu map_calls=%llu\n",
                     vision_enabled() ? "on" : "off", vs.fps, vs.t_detect_avg_us,
                     vs.t_detect_max_us, vs.t_lat_avg_us, vs.t_lat_max_us,
                     (unsigned long long)vs.frames, (unsigned long long)vs.dropped,
                     (unsigned long long)vs.dump_fail, (unsigned long long)vs.map_calls);
        } else {
            snprintf(reply, n, "ERR vision <on|off|status>\n");
        }
        return;
    }

    if (strcmp(tok, "record") == 0) {
        char *a = strtok_r(NULL, " \t\r\n", &save);
        if (!a) {
            snprintf(reply, n, "ERR record <on|off|status|cap|dir>\n");
        } else if (!record_is_started()) {
            snprintf(reply, n, "ERR record 未启动（启动参数为 --record off）；"
                               "要启用请改成 --record on 重启进程\n");
        } else if (strcmp(a, "on") == 0 || strcmp(a, "start") == 0) {
            record_set_enabled(1);
            snprintf(reply, n, "OK record on\n");
        } else if (strcmp(a, "off") == 0 || strcmp(a, "stop") == 0) {
            record_set_enabled(0);
            snprintf(reply, n, "OK record off (当前会话收尾)\n");
        } else if (strcmp(a, "status") == 0) {
            record_stats_t rs;
            record_get_stats(&rs);
            snprintf(reply, n,
                     "record=%s auto=%d bound=%d sess=%u fps=%u streams=%llu written=%llu "
                     "disc=%llu bytes=%llu dir=%s cap=%llu\n",
                     rs.active ? "on" : "off", rs.enabled, rs.bound, rs.session_index,
                     rs.real_fps, (unsigned long long)rs.streams,
                     (unsigned long long)rs.written, (unsigned long long)rs.discarded,
                     (unsigned long long)rs.bytes, rs.dir ? rs.dir : "-",
                     (unsigned long long)rs.cap_bytes);
        } else if (strcmp(a, "cap") == 0) {
            char *v = strtok_r(NULL, " \t\r\n", &save);
            if (!v) {
                snprintf(reply, n, "ERR record cap <bytes>\n");
            } else {
                record_set_cap(strtoull(v, NULL, 0));
                snprintf(reply, n, "OK record cap=%llu\n", (unsigned long long)strtoull(v, NULL, 0));
            }
        } else if (strcmp(a, "dir") == 0) {
            char *v = strtok_r(NULL, " \t\r\n", &save);
            if (!v) {
                snprintf(reply, n, "ERR record dir <path>\n");
            } else if (record_set_dir(v) == 0) {
                snprintf(reply, n, "OK record dir=%s (下次会话生效)\n", v);
            } else {
                snprintf(reply, n, "ERR record dir busy\n");
            }
        } else {
            snprintf(reply, n, "ERR record <on|off|status|cap|dir>\n");
        }
        return;
    }

    /* 兼容旧工程的写法：start/stop 直接作用于录像常录开关 */
    if ((strcmp(tok, "start") == 0 || strcmp(tok, "stop") == 0) && !record_is_started()) {
        snprintf(reply, n, "ERR record 未启动（启动参数为 --record off）\n");
        return;
    }
    if (strcmp(tok, "start") == 0) {
        record_set_enabled(1);
        snprintf(reply, n, "OK record on\n");
        return;
    }
    if (strcmp(tok, "stop") == 0) {
        record_set_enabled(0);
        snprintf(reply, n, "OK record off (当前会话收尾)\n");
        return;
    }
    if (strcmp(tok, "cap") == 0) {
        char *v = strtok_r(NULL, " \t\r\n", &save);
        if (v) {
            record_set_cap(strtoull(v, NULL, 0));
            snprintf(reply, n, "OK cap set\n");
        } else {
            snprintf(reply, n, "ERR cap <bytes>\n");
        }
        return;
    }

    snprintf(reply, n, "ERR unknown command '%s' (try help)\n", tok);
}

static void serve_client(int fd)
{
    struct timeval tv = { 0, 500000 };   /* 500ms 读超时，防客户端卡死控制线程 */
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char req[512];
    ssize_t n = recv(fd, req, sizeof(req) - 1, 0);
    if (n <= 0) {
        close(fd);
        return;
    }
    req[n] = '\0';

    char reply[1200];
    reply[0] = '\0';
    handle_command(req, reply, sizeof(reply));
    if (reply[0])
        (void)send(fd, reply, strlen(reply), MSG_NOSIGNAL);
    close(fd);
}

/* ============================ 线程 ============================ */

static void *ctl_thread(void *arg)
{
    (void)arg;

    while (g_run) {
        struct pollfd pfd;
        pfd.fd = g_listen_fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        int pr = poll(&pfd, 1, 200);
        if (pr <= 0)
            continue;

        int fd = accept(g_listen_fd, NULL, NULL);
        if (fd < 0) {
            if (errno != EAGAIN && errno != EINTR && g_run)
                app_log(APP_NAME_STR ": control accept failed: %s\n", strerror(errno));
            continue;
        }
        serve_client(fd);
    }
    return NULL;
}

int control_start(void)
{
    struct sockaddr_un addr;

    if (strlen(APP_CTL_SOCKET) >= sizeof(addr.sun_path)) {
        app_log(APP_NAME_STR ": control socket path too long\n");
        return -1;
    }

    unlink(APP_CTL_SOCKET);
    g_listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (g_listen_fd < 0) {
        app_log(APP_NAME_STR ": control socket failed: %s\n", strerror(errno));
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", APP_CTL_SOCKET);

    if (bind(g_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        app_log(APP_NAME_STR ": control bind failed: %s\n", strerror(errno));
        close(g_listen_fd);
        g_listen_fd = -1;
        return -1;
    }
    if (listen(g_listen_fd, 4) != 0) {
        app_log(APP_NAME_STR ": control listen failed: %s\n", strerror(errno));
        close(g_listen_fd);
        g_listen_fd = -1;
        return -1;
    }

    g_run = 1;
    if (pthread_create(&g_tid, NULL, ctl_thread, NULL) != 0) {
        app_log(APP_NAME_STR ": control thread create failed\n");
        g_run = 0;
        close(g_listen_fd);
        g_listen_fd = -1;
        return -1;
    }
    g_started = 1;
    app_log(APP_NAME_STR ": control socket %s ready\n", APP_CTL_SOCKET);
    return 0;
}

void control_stop(void)
{
    if (!g_started)
        return;
    g_run = 0;
    pthread_join(g_tid, NULL);
    g_started = 0;
    if (g_listen_fd >= 0) {
        close(g_listen_fd);
        g_listen_fd = -1;
    }
    unlink(APP_CTL_SOCKET);
}
