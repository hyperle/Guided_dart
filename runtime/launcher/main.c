/*
 * launcher —— K230 RT-Smart 开机自启动器（v2）
 *
 * 职责边界：本程序只做"读清单 → fork/execv 拉起 → 监督回收"。它不认识相机、
 * 识别、录像，也不包含任何 K230 SDK 头文件；因此 host 与交叉构建都必须通过，
 * 也便于在电脑上用假清单验证解析与前后台语义。
 *
 * 自启链路（与 v1 相同，靠固件把**一个用户程序**注册为开机命令）：
 *   make menuconfig -> RT-Smart Configuration ->
 *       "Rtsmart auto execute command string" = /sdcard/app/launcher
 *   之后 launcher 读清单 /sdcard/app/startup_final.list（可用 argv 覆盖），
 *   把每一项拉起为独立 lwp 用户进程。
 *
 * 【不要】把自启命令写成 @preload / fastboot：快起跑的是烧在镜像固定位置的
 * rtapp 备份，根本不读文件系统，表现为"删了 /sdcard 下的 ELF 开机照跑、替换
 * 也不生效"。secure boot 下强制要求命令以 @preload 开头，此时本方案不可用。
 *
 * 清单语法（每行一条，与 v1 兼容）：
 *     绝对路径 [参数...] [&]
 *     行尾 "&"   -> 后台启动，不等待；由 launcher 统一监督（kill 探测）
 *     行尾 "&!"  -> 后台启动，退出后等待 5 秒并重启同一命令
 *     不带 "&"   -> 前台启动，等它结束后再执行下一行（初始化类程序）
 *     '#' 开头或空行 -> 注释
 *     单行 <= 511 字符，token <= 31 个（路径 + 最多 31 个参数）
 *
 * 相对 v1 的四处硬化（v1 的现场证据见 runtime/launcher/startup.list 与
 * docs/LAUNCHER_DESIGN.md）：
 *   1) 不把 waitpid 当唯一判据：前台项用 WNOHANG + kill(pid,0) 轮询判断存活，
 *      waitpid 返回 ECHILD/异常时退化为"已消失(状态未知)"并继续，不会像 v1 那样
 *      把"还在运行"记成 "exited, code=0"。
 *   2) 每个子进程的 stdout/stderr 写各自的 <name>.console.log（v1 全部塞进同一个
 *      launcher-child.log，多应用时无法归因），并写 <name>.pid。
 *   3) 单实例锁 logs/launcher.pid：已在运行则拒绝二次启动（双 launcher 会制造
 *      两份 VICAP 消费者，v1 实测必死）。--force 可强制覆盖。
 *   4) --dry-run：只解析并 stat 每项、打印将要执行的命令行，上电前先验证清单。
 *
 * 用法：
 *   launcher                       # 读默认清单 /sdcard/app/startup_final.list
 *   launcher /tmp/my.list          # 调试用自定义清单
 *   launcher -t                    # 只校验清单，不启动任何进程
 *   launcher -n                    # 启动完后台项后立即退出，不进入监督
 *
 * 构建：bash .script/build-rtos launcher   ->  build-cross/launcher/launcher
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "system/write_pause.h"

#define DEFAULT_LIST    "/sdcard/app/startup_final.list"
#define DEFAULT_LOG_DIR "/data/app/logs"

#define MAX_LINE 512  /* 单行最大长度（路径+参数） */
#define MAX_ARGS 32   /* 单行最大 token 数（含路径），末尾留给 NULL */
#define MAX_BG   32   /* 最多同时监督的后台进程数 */
#define MAX_PATH 512  /* 拼接出来的日志/pid 路径上限 */

struct options {
    const char* list_path;
    const char* log_dir;
    int no_supervise;
    int dry_run;
    int force;
    int fg_timeout_s; /* 0 = 前台项不限时 */
    int poll_ms;      /* 监督/等待轮询间隔 */
    int heartbeat_s;  /* 存活心跳日志间隔，0 = 关闭 */
};

struct bg_item {
    pid_t pid;
    char name[MAX_PATH];
    char command[MAX_LINE];
    char exit_marker[MAX_PATH];
    int restart;
};

static struct options g_opt = {DEFAULT_LIST, DEFAULT_LOG_DIR, 0, 0, 0, 0, 1000, 60};
static char g_log_path[MAX_PATH] = DEFAULT_LOG_DIR "/launcher.log";
static char g_pid_path[MAX_PATH] = DEFAULT_LOG_DIR "/launcher.pid";
static struct bg_item g_bg[MAX_BG];
static int g_bg_count = 0;

struct console_item {
    int used;
    int fd;
    char path[MAX_PATH];
};
static struct console_item g_console[MAX_BG * 2];
static void ensure_dir(const char* dir);

/* Children retain a pipe, never a log file. Keep consuming while paused so
 * console traffic cannot fill a pipe and stall application work. */
static int pump_consoles(void) {
    int active = 0;
    int paused = dart_writes_paused(dart_write_pause_path());
    for (unsigned i = 0; i < sizeof(g_console) / sizeof(g_console[0]); ++i) {
        struct console_item* item = &g_console[i];
        if (!item->used)
            continue;
        for (int budget = 0; budget < 64; ++budget) {
            char data[4096];
            ssize_t count = read(item->fd, data, sizeof(data));
            if (count > 0) {
                if (!paused) {
                    int fd = open(item->path, O_WRONLY | O_CREAT | O_APPEND, 0666);
                    if (fd < 0 && errno == ENOENT) {
                        ensure_dir(g_opt.log_dir);
                        fd = open(item->path, O_WRONLY | O_CREAT | O_APPEND, 0666);
                    }
                    if (fd >= 0) {
                        ssize_t offset = 0;
                        while (offset < count) {
                            ssize_t written = write(fd, data + offset, (size_t)(count - offset));
                            if (written < 0 && errno == EINTR)
                                continue;
                            if (written <= 0)
                                break;
                            offset += written;
                        }
                        close(fd);
                    }
                }
                continue;
            }
            if (count == 0 || (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
                close(item->fd);
                item->used = 0;
            }
            break;
        }
        active += item->used;
    }
    return active;
}

/* ------------------------------------------------------------------------- */
/* 日志：每行 打开->写->关闭。进程不长期持有日志 fd，板端 MTP/文件管理器
 * 才能随时读到；RT-Smart 上 printf 走控制台而不是文件。 */
/* ------------------------------------------------------------------------- */
static unsigned long long mono_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0;
    return (unsigned long long)ts.tv_sec * 1000ULL + (unsigned long long)(ts.tv_nsec / 1000000L);
}

static void sleep_ms(int ms) {
    struct timespec ts;
    while (ms > 0) {
        int step = ms > 20 ? 20 : ms;
        (void)pump_consoles();
        ts.tv_sec = 0;
        ts.tv_nsec = (long)step * 1000000L;
        while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {}
        ms -= step;
    }
}

static void llog(const char* fmt, ...) {
    if (dart_writes_paused(dart_write_pause_path()))
        return;
    char body[MAX_LINE];
    char line[MAX_LINE + 64];
    va_list ap;
    int n;
    int fd;
    int total;

    va_start(ap, fmt);
    n = vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;

    total = snprintf(line, sizeof(line), "ts_ms=%llu %s", mono_ms(), body);
    if (total < 0)
        return;
    if ((size_t)total >= sizeof(line))
        total = (int)sizeof(line) - 1;

    fd = open(g_log_path, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0 && errno == ENOENT) {
        ensure_dir(g_opt.log_dir);
        fd = open(g_log_path, O_WRONLY | O_CREAT | O_APPEND, 0666);
    }
    if (fd < 0) {
        /* 日志目录不可用（例如 /sdcard 未挂载）时退到 stderr，不影响启动流程。 */
        (void)!write(STDERR_FILENO, line, (size_t)total);
        return;
    }
    (void)!write(fd, line, (size_t)total);
    close(fd);
}

/* ------------------------------------------------------------------------- */
/* 路径与进程工具 */
/* ------------------------------------------------------------------------- */
static const char* path_base(const char* path) {
    const char* slash = strrchr(path, '/');
    if (slash == NULL)
        return path;
    return (slash[1] != '\0') ? slash + 1 : path;
}

/* 拼接 <dir>/<base><suffix>；成功返回 0，太长返回 -1。 */
static int item_path(char* out, size_t cap, const char* dir, const char* base, const char* suffix) {
    int n = snprintf(out, cap, "%s/%s%s", dir, base, suffix);
    if (n < 0 || (size_t)n >= cap) {
        if (cap > 0)
            out[cap - 1] = '\0';
        return -1;
    }
    return 0;
}

static int pid_alive(pid_t pid) {
    if (pid <= 0)
        return 0;
    if (kill(pid, 0) == 0)
        return 1;
    return errno != ESRCH; /* EPERM 等仍视为存活 */
}

static void pidfile_write(const char* path, pid_t pid) {
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "%d\n", (int)pid);
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0)
        return;
    (void)!write(fd, buf, (size_t)n);
    close(fd);
}

static pid_t pidfile_read(const char* path) {
    char buf[32];
    int fd = open(path, O_RDONLY);
    ssize_t n;
    if (fd < 0)
        return 0;
    n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return 0;
    buf[n] = '\0';
    return (pid_t)strtol(buf, NULL, 10);
}

/* 前台项等待：返回 0=已退出（*status_out 有效）、1=超时仍存活、2=已消失但取不到状态 */
static int wait_foreground(pid_t pid, const char* exit_marker, int* status_out) {
    unsigned long long start = mono_ms();
    unsigned long long last_hb = start;

    for (;;) {
        int status = 0;
        if (pidfile_read(exit_marker) == pid) {
            llog("level=INFO module=launcher op=foreground pid=%d result=done_marker\n", (int)pid);
            return 2;
        }
        pid_t r = waitpid(pid, &status, WNOHANG);

        if (r == pid) {
            *status_out = status;
            return 0;
        }
        if (r < 0 && errno != EINTR && errno != ECHILD) {
            /* waitpid 行为异常不是致命错误：继续用 kill 探测。 */
            llog(
                "level=WARN module=launcher op=waitpid pid=%d err=%s\n", (int)pid, strerror(errno));
        }

        /* 不把 waitpid 当唯一判据：ECHILD 或持续返回 0 时用 kill(pid,0) 判定。 */
        if (!pid_alive(pid)) {
            llog(
                "level=WARN module=launcher op=foreground pid=%d result=gone "
                "status=unknown note=waitpid_unavailable\n",
                (int)pid);
            return 2;
        }

        if (g_opt.fg_timeout_s > 0 && (long)((mono_ms() - start) / 1000ULL) >= g_opt.fg_timeout_s) {
            llog(
                "level=WARN module=launcher op=foreground pid=%d result=timeout "
                "timeout_s=%d note=still_running\n",
                (int)pid, g_opt.fg_timeout_s);
            return 1;
        }
        if (g_opt.heartbeat_s > 0 && (long)((mono_ms() - last_hb) / 1000ULL) >= g_opt.heartbeat_s) {
            last_hb = mono_ms();
            llog(
                "level=INFO module=launcher op=foreground pid=%d result=running "
                "elapsed_ms=%llu\n",
                (int)pid, mono_ms() - start);
        }
        sleep_ms(g_opt.poll_ms);
    }
}

static void
    log_status_line(const char* name, pid_t pid, const char* tag, const char* result, int status) {
    if (strcmp(result, "exit") == 0) {
        if (WIFEXITED(status)) {
            llog(
                "level=INFO module=launcher op=%s app=%s pid=%d result=exited "
                "code=%d\n",
                tag, name, (int)pid, WEXITSTATUS(status));
        } else if (WIFSIGNALED(status)) {
            llog(
                "level=WARN module=launcher op=%s app=%s pid=%d result=signaled "
                "signal=%d\n",
                tag, name, (int)pid, WTERMSIG(status));
        } else {
            llog(
                "level=INFO module=launcher op=%s app=%s pid=%d result=finished\n", tag, name,
                (int)pid);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 清单项启动 */
/* ------------------------------------------------------------------------- */
static void record_bg(
    pid_t pid, const char* name, const char* command, const char* exit_marker, int restart) {
    int index = 0;
    while (index < g_bg_count && g_bg[index].pid > 0)
        index++;
    if (index == g_bg_count && g_bg_count >= MAX_BG) {
        llog(
            "level=WARN module=launcher op=supervise pid=%d app=%s "
            "result=unsupervised note=too_many_background_apps max=%d\n",
            (int)pid, name, MAX_BG);
        return;
    }
    if (index == g_bg_count)
        g_bg_count++;
    g_bg[index].pid = pid;
    snprintf(g_bg[index].name, sizeof(g_bg[index].name), "%s", name);
    snprintf(g_bg[index].command, sizeof(g_bg[index].command), "%s", command);
    snprintf(g_bg[index].exit_marker, sizeof(g_bg[index].exit_marker), "%s", exit_marker);
    g_bg[index].restart = restart;
}

/* 解析并启动一行；返回 0 = 正常处理（前台项已结束/后台项已拉起），-1 = 本项失败。 */
static int launch_one(char* line, int lineno) {
    char* tok[MAX_ARGS + 1];
    int n = 0;
    int background = 0;
    int restart = 0;
    char command[MAX_LINE];
    char* p = line;
    char* path;
    const char* name;
    char pathbuf[MAX_PATH];
    char exit_marker[MAX_PATH] = "";
    struct stat st;
    pid_t pid;
    int console_pipe[2] = {-1, -1};
    int console_index = -1;

    snprintf(command, sizeof(command), "%s", line);
    /* 1) 按空白切 token */
    while (*p != '\0') {
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\0')
            break;
        if (n >= MAX_ARGS) {
            llog(
                "level=ERROR module=launcher op=parse line=%d code=too_many_tokens "
                "max=%d\n",
                lineno, MAX_ARGS);
            return -1;
        }
        tok[n++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t')
            p++;
        if (*p != '\0')
            *p++ = '\0';
    }
    if (n == 0)
        return 0;

    /* 2) 行尾 "&" = 后台 */
    if (strcmp(tok[n - 1], "&") == 0 || strcmp(tok[n - 1], "&!") == 0) {
        background = 1;
        restart = strcmp(tok[n - 1], "&!") == 0;
        n--;
    }
    tok[n] = NULL;
    if (n == 0)
        return 0;

    path = tok[0];
    name = path_base(path);

    /* 3) 文件存在性预检：在父进程报错比子进程 exec 失败更清晰 */
    if (stat(path, &st) != 0) {
        llog(
            "level=ERROR module=launcher op=precheck line=%d code=stat_failed "
            "err=%s path=%s\n",
            lineno, strerror(errno), path);
        return -1;
    }

    /* gphoto2/MTP 上传不保留可执行位；上板后顺手修正。 */
    if (chmod(path, 0755) != 0) {
        llog(
            "level=WARN module=launcher op=precheck line=%d code=chmod_failed "
            "err=%s path=%s\n",
            lineno, strerror(errno), path);
    }

    if (g_opt.dry_run) {
        int i;
        llog(
            "level=INFO module=launcher op=dry_run line=%d mode=%s path=%s\n", lineno,
            background ? "bg" : "fg", path);
        printf("%s [%s]", path, background ? "bg" : "fg");
        for (i = 1; i < n; i++)
            printf(" %s", tok[i]);
        printf("\n");
        return 0;
    }

    if (restart || !background) {
        if (item_path(exit_marker, sizeof(exit_marker), g_opt.log_dir, name, ".done") != 0) {
            llog("level=ERROR module=launcher op=precheck line=%d code=path_too_long\n", lineno);
            return -1;
        }
        if (unlink(exit_marker) != 0 && errno != ENOENT) {
            llog(
                "level=ERROR module=launcher op=precheck line=%d code=marker_unlink "
                "err=%s path=%s\n",
                lineno, strerror(errno), exit_marker);
            return -1;
        }
    }

    (void)pump_consoles();
    for (unsigned i = 0; i < sizeof(g_console) / sizeof(g_console[0]); ++i) {
        if (!g_console[i].used) {
            console_index = (int)i;
            break;
        }
    }
    if (console_index >= 0 && pipe(console_pipe) == 0) {
        if (fcntl(console_pipe[0], F_SETFL, O_NONBLOCK) != 0
            || fcntl(console_pipe[0], F_SETFD, FD_CLOEXEC) != 0) {
            close(console_pipe[0]);
            close(console_pipe[1]);
            console_pipe[0] = console_pipe[1] = -1;
        }
    }

    /* 4) fork + execv：RT-Smart 中每个 ELF 就是独立进程（lwp） */
    pid = fork();
    if (pid < 0) {
        if (console_pipe[0] >= 0) {
            close(console_pipe[0]);
            close(console_pipe[1]);
        }
        llog(
            "level=ERROR module=launcher op=fork line=%d code=fork_failed err=%s "
            "path=%s\n",
            lineno, strerror(errno), path);
        return -1;
    }
    if (pid == 0) {
        if ((restart || !background) && setenv("DART_EXIT_MARKER", exit_marker, 1) != 0)
            _exit(127);
        for (unsigned i = 0; i < sizeof(g_console) / sizeof(g_console[0]); ++i) {
            if (g_console[i].used)
                close(g_console[i].fd);
        }
        if (console_pipe[0] >= 0)
            close(console_pipe[0]);
        int output = console_pipe[1] >= 0 ? console_pipe[1] : open("/dev/null", O_WRONLY);
        if (output >= 0) {
            (void)dup2(output, STDOUT_FILENO);
            (void)dup2(output, STDERR_FILENO);
            if (output > STDERR_FILENO)
                close(output);
        }
        execv(path, tok); /* 成功后不返回 */
        llog(
            "level=ERROR module=launcher op=exec line=%d code=exec_failed err=%s "
            "path=%s\n",
            lineno, strerror(errno), path);
        _exit(127);
    }

    if (console_pipe[1] >= 0) {
        close(console_pipe[1]);
        struct console_item* item = &g_console[console_index];
        if (item_path(item->path, sizeof(item->path), g_opt.log_dir, name, ".console.log") == 0) {
            item->fd = console_pipe[0];
            item->used = 1;
        } else {
            close(console_pipe[0]);
        }
    }

    /* 5) 父进程 */
    if (background) {
        record_bg(pid, name, command, exit_marker, restart);
        if (item_path(pathbuf, sizeof(pathbuf), g_opt.log_dir, name, ".pid") == 0)
            pidfile_write(pathbuf, pid);
        llog(
            "level=INFO module=launcher op=start line=%d mode=bg app=%s pid=%d\n", lineno, name,
            (int)pid);
    } else {
        int status = 0;
        int r;
        llog(
            "level=INFO module=launcher op=start line=%d mode=fg app=%s pid=%d\n", lineno, name,
            (int)pid);
        r = wait_foreground(pid, exit_marker, &status);
        if (r == 0)
            log_status_line(name, pid, "foreground", "exit", status);
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* 监督阶段 */
/* ------------------------------------------------------------------------- */
/* 后台项结束：清掉它的 pid 文件并置为已回收（pid = -1）。 */
static void retire_bg(int index) {
    char pathbuf[MAX_PATH];

    if (item_path(pathbuf, sizeof(pathbuf), g_opt.log_dir, g_bg[index].name, ".pid") == 0) {
        unlink(pathbuf);
    }
    g_bg[index].pid = -1;
}

static int restart_bg(int index) {
    char command[MAX_LINE];
    snprintf(command, sizeof(command), "%s", g_bg[index].command);
    retire_bg(index);
    sleep_ms(5000);
    return launch_one(command, 0) == 0;
}

static void supervise(void) {
    int remaining = g_bg_count;
    unsigned long long last_hb = mono_ms();
    int i;

    if (remaining <= 0)
        return;

    llog("level=INFO module=launcher op=supervise result=start count=%d\n", remaining);

    while (remaining > 0) {
        for (i = 0; i < g_bg_count; i++) {
            int status = 0;
            pid_t r;

            if (g_bg[i].pid <= 0)
                continue;

            if (g_bg[i].restart && pidfile_read(g_bg[i].exit_marker) == g_bg[i].pid) {
                llog(
                    "level=INFO module=launcher op=supervise app=%s pid=%d "
                    "result=done_marker\n",
                    g_bg[i].name, (int)g_bg[i].pid);
                if (!restart_bg(i))
                    remaining--;
                continue;
            }

            r = waitpid(g_bg[i].pid, &status, WNOHANG);
            if (r == g_bg[i].pid) {
                log_status_line(g_bg[i].name, g_bg[i].pid, "supervise", "exit", status);
                if (g_bg[i].restart) {
                    if (!restart_bg(i))
                        remaining--;
                } else {
                    retire_bg(i);
                    remaining--;
                }
                continue;
            }
            if (r < 0 && errno == ECHILD) {
                /* ECHILD alone does not prove an RT-Smart lwp has exited. */
                if (pid_alive(g_bg[i].pid))
                    continue;
                llog(
                    "level=WARN module=launcher op=supervise app=%s pid=%d "
                    "result=gone status=unknown note=no_child_probe_esrch\n",
                    g_bg[i].name, (int)g_bg[i].pid);
                if (g_bg[i].restart) {
                    if (!restart_bg(i))
                        remaining--;
                } else {
                    retire_bg(i);
                    remaining--;
                }
                continue;
            }
            /* waitpid 返回 0 或异常：用 kill(pid,0) 兜底判定存活。 */
            if (!pid_alive(g_bg[i].pid)) {
                llog(
                    "level=WARN module=launcher op=supervise app=%s pid=%d "
                    "result=gone status=unknown note=probe_esrch\n",
                    g_bg[i].name, (int)g_bg[i].pid);
                if (g_bg[i].restart) {
                    if (!restart_bg(i))
                        remaining--;
                } else {
                    retire_bg(i);
                    remaining--;
                }
            }
        }
        if (remaining <= 0)
            break;
        if (g_opt.heartbeat_s > 0
            && (mono_ms() - last_hb) >= (unsigned long long)g_opt.heartbeat_s * 1000ULL) {
            last_hb = mono_ms();
            for (i = 0; i < g_bg_count; i++) {
                if (g_bg[i].pid <= 0)
                    continue;
                llog(
                    "level=INFO module=launcher op=supervise app=%s pid=%d "
                    "result=running\n",
                    g_bg[i].name, (int)g_bg[i].pid);
            }
        }
        sleep_ms(g_opt.poll_ms);
    }
}

static void usage(const char* argv0) {
    printf("用法: %s [选项] [清单路径]\n", argv0);
    printf("  清单默认 %s\n\n", DEFAULT_LIST);
    printf("  -n, --no-supervise    启动完后台项后立即退出，不进入监督\n");
    printf("  -t, --dry-run         只解析并检查清单，不启动任何进程\n");
    printf("  -L, --log-dir DIR     日志与 pid 目录（默认 %s）\n", DEFAULT_LOG_DIR);
    printf("      --fg-timeout SEC  前台项最长等待秒数，0 = 不限（默认 0）\n");
    printf("      --poll-ms MS      等待/监督轮询间隔（默认 1000）\n");
    printf("      --heartbeat SEC   存活心跳日志间隔，0 = 关闭（默认 60）\n");
    printf("      --force           忽略单实例锁，强制启动\n");
    printf("  -h, --help            显示本帮助\n");
}

static int parse_int(const char* text, int* out) {
    char* end = NULL;
    long value;
    errno = 0;
    value = strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || value < 0 || value > 1000000L)
        return -1;
    *out = (int)value;
    return 0;
}

/* 日志目录不存在则创建；父目录缺失时补建一层（例如首次上板 /sdcard/app
 * 还不存在）。失败只影响日志落地，不阻断启动流程。 */
static void ensure_dir(const char* dir) {
    char parent[MAX_PATH];
    const char* slash;

    if (dir == NULL || dir[0] == '\0')
        return;
    if (mkdir(dir, 0777) == 0 || errno == EEXIST)
        return;
    if (errno != ENOENT)
        return;
    slash = strrchr(dir, '/');
    if (slash == NULL || slash == dir)
        return;
    if ((size_t)(slash - dir) >= sizeof(parent))
        return;
    memcpy(parent, dir, (size_t)(slash - dir));
    parent[slash - dir] = '\0';
    (void)mkdir(parent, 0777);
    (void)mkdir(dir, 0777);
}

int main(int argc, char* argv[]) {
    FILE* fp;
    char* list_data;
    char next_path[MAX_PATH];
    long list_size;
    int lineno = 0;
    int i;
    int list_given = 0;
    pid_t lock_pid;

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    for (i = 1; i < argc; i++) {
        const char* a = argv[i];
        int handled = 0;
        int* target = NULL;

        if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else if (strcmp(a, "-n") == 0 || strcmp(a, "--no-supervise") == 0) {
            g_opt.no_supervise = 1;
            handled = 1;
        } else if (strcmp(a, "-t") == 0 || strcmp(a, "--dry-run") == 0) {
            g_opt.dry_run = 1;
            handled = 1;
        } else if (strcmp(a, "--force") == 0) {
            g_opt.force = 1;
            handled = 1;
        } else if (strcmp(a, "-L") == 0 || strcmp(a, "--log-dir") == 0) {
            if (++i >= argc) {
                fprintf(stderr, "%s: --log-dir 缺少参数\n", argv[0]);
                return 2;
            }
            g_opt.log_dir = argv[i];
            handled = 1;
        } else if (strcmp(a, "--fg-timeout") == 0) {
            target = &g_opt.fg_timeout_s;
        } else if (strcmp(a, "--poll-ms") == 0) {
            target = &g_opt.poll_ms;
        } else if (strcmp(a, "--heartbeat") == 0) {
            target = &g_opt.heartbeat_s;
        }

        if (handled)
            continue;
        if (target != NULL) {
            if (++i >= argc || parse_int(argv[i], target) != 0) {
                fprintf(stderr, "%s: %s 需要非负整数\n", argv[0], a);
                return 2;
            }
        } else if (a[0] == '-' && a[1] != '\0') {
            fprintf(stderr, "%s: 未知选项 %s（-h 查看用法）\n", argv[0], a);
            return 2;
        } else if (!list_given) {
            g_opt.list_path = a;
            list_given = 1;
        } else {
            fprintf(stderr, "%s: 多余的参数 %s\n", argv[0], a);
            return 2;
        }
    }

    if (g_opt.poll_ms <= 0)
        g_opt.poll_ms = 1000;

    /* 日志目录不存在则创建；失败只影响日志落地，不阻断启动流程。 */
    ensure_dir(g_opt.log_dir);

    snprintf(g_log_path, sizeof(g_log_path), "%s/launcher.log", g_opt.log_dir);
    snprintf(g_pid_path, sizeof(g_pid_path), "%s/launcher.pid", g_opt.log_dir);

    /* 单实例锁：双 launcher 会拉起两份独占 VICAP 的 payload。
     * RT-Smart 的第一个用户进程常拿到 PID 1。复位后旧 pid 文件中的
     * PID 可能仍是 1，而当前 launcher 也会是 1；同一 PID 不可能同时
     * 属于两个进程，因此这种情况是跨复位残留锁，不能误报 already_running。 */
    if (!g_opt.dry_run && !g_opt.force) {
        lock_pid = pidfile_read(g_pid_path);
        if (lock_pid > 0 && lock_pid != getpid() && pid_alive(lock_pid)) {
            llog(
                "level=ERROR module=launcher op=start code=already_running "
                "pid=%d pidfile=%s action=exit\n",
                (int)lock_pid, g_pid_path);
            fprintf(stderr, "launcher: 已在运行 (pid=%d)，--force 可强制启动\n", (int)lock_pid);
            return 1;
        }
    }

    llog(
        "level=INFO module=launcher op=start list=%s log_dir=%s dry_run=%d "
        "no_supervise=%d\n",
        g_opt.list_path, g_opt.log_dir, g_opt.dry_run, g_opt.no_supervise);

    /* A running foreground item may keep the old FAT file read-locked.
     * Deployments can stage <list>.next; the next boot promotes it before
     * taking a snapshot. Dry runs never activate a pending update. */
    if (!g_opt.dry_run) {
        int n = snprintf(next_path, sizeof(next_path), "%s.next", g_opt.list_path);
        if (n >= 0 && (size_t)n < sizeof(next_path) && access(next_path, F_OK) == 0) {
            char previous[MAX_PATH];
            int p = snprintf(previous, sizeof(previous), "%s.previous", g_opt.list_path);
            if (p < 0 || (size_t)p >= sizeof(previous))
                return 1;
            /* FatFS does not replace an existing rename target. Preserve the
             * old list before promoting the fully uploaded replacement. */
            if (unlink(previous) != 0 && errno != ENOENT)
                return 1;
            int had_previous = access(g_opt.list_path, F_OK) == 0;
            if (had_previous && rename(g_opt.list_path, previous) != 0) {
                llog(
                    "level=ERROR module=launcher op=list_update err=%s path=%s\n", strerror(errno),
                    g_opt.list_path);
                return 1;
            }
            if (rename(next_path, g_opt.list_path) != 0) {
                int saved_errno = errno;
                if (had_previous)
                    (void)rename(previous, g_opt.list_path);
                llog(
                    "level=ERROR module=launcher op=list_update err=%s path=%s\n",
                    strerror(saved_errno), next_path);
                return 1;
            }
            llog("level=INFO module=launcher op=list_update result=applied\n");
        }
    }

    fp = fopen(g_opt.list_path, "r");
    if (fp == NULL) {
        llog(
            "level=ERROR module=launcher op=list_open code=open_failed err=%s "
            "path=%s\n",
            strerror(errno), g_opt.list_path);
        fprintf(stderr, "launcher: 无法打开清单 %s: %s\n", g_opt.list_path, strerror(errno));
        return 1;
    }

    /* Close the startup file before launching any item, including foreground
     * jobs. Neither the parent nor its children may retain a FAT read lock. */
    if (fseek(fp, 0, SEEK_END) != 0 || (list_size = ftell(fp)) < 0 || list_size > 1024 * 1024
        || fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        llog("level=ERROR module=launcher op=list_read code=invalid_size\n");
        return 1;
    }
    list_data = malloc((size_t)list_size + 1);
    if (list_data == NULL) {
        fclose(fp);
        return 1;
    }
    size_t bytes = fread(list_data, 1, (size_t)list_size, fp);
    int read_failed = ferror(fp) || bytes != (size_t)list_size;
    if (fclose(fp) != 0)
        read_failed = 1;
    if (read_failed) {
        free(list_data);
        llog("level=ERROR module=launcher op=list_read code=read_failed\n");
        return 1;
    }
    list_data[bytes] = '\0';

    if (!g_opt.dry_run)
        pidfile_write(g_pid_path, getpid());

    char* cursor = list_data;
    while (*cursor != '\0') {
        char line[MAX_LINE];
        char* end = strchr(cursor, '\n');
        size_t length = end != NULL ? (size_t)(end - cursor) : strlen(cursor);
        lineno++;
        if (length >= sizeof(line)) {
            llog("level=ERROR module=launcher op=item line=%d code=line_too_long\n", lineno);
            cursor += length + (end != NULL);
            continue;
        }
        memcpy(line, cursor, length);
        line[length] = '\0';
        cursor += length + (end != NULL);
        char* s = line;
        char* nl;

        nl = strpbrk(line, "\r\n");
        if (nl != NULL)
            *nl = '\0';
        while (*s == ' ' || *s == '\t')
            s++;
        if (*s == '\0' || *s == '#')
            continue;

        if (launch_one(s, lineno) < 0)
            llog("level=WARN module=launcher op=item line=%d result=skipped\n", lineno);
    }
    free(list_data);

    if (!g_opt.no_supervise && !g_opt.dry_run)
        supervise();

    llog("level=INFO module=launcher op=exit result=done background=%d\n", g_bg_count);
    unlink(g_pid_path);
    /* -n and foreground timeouts leave applications running. Hand remaining
     * pipes to a detached collector so their consoles remain usable. */
    if (pump_consoles() > 0) {
        pid_t collector = fork();
        if (collector == 0) {
            while (pump_consoles() > 0)
                sleep_ms(20);
            _exit(0);
        }
        if (collector < 0) {
            while (pump_consoles() > 0)
                sleep_ms(20);
        }
    }
    return 0;
}
