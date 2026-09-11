/*
 * launcher —— K230 RT-Smart 开机自启动器
 *
 * 功能：
 *   读 /sdcard/app/startup_final.list（可用 argv[1] 覆盖路径），逐行解析，
 *   把每一项启动为一个独立用户进程：
 *     - 行尾带 "&"      -> 后台启动（不等待，适合摄像头/AI/网络等常驻服务）
 *     - 不带 "&"        -> 前台启动并等它退出再启动下一条（适合初始化类程序）
 *   所有后台子进程启动完后，本启动器进入"监督"状态：waitpid 回收退出的
 *   子进程并打印日志，直到全部结束（可自行扩展为退出后自动重启）。
 *
 * 开机注册（make menuconfig -> RT-Smart Configuration）：
 *     Rtsmart auto execute command string = /sdcard/app/launcher
 *   切勿使用 @preload/fastboot 快起，否则替换 SD 卡文件不生效。
 *
 * 说明：
 *   编译用 musl 工具链（fork/execv/waitpid 均为 POSIX 标准接口），
 *   运行依赖 RT-Smart lwp 的 sys_fork/sys_execve/sys_waitpid 系统调用
 *   （K230 SDK 内核已实现并注册，见 components/lwp/lwp_syscall.c）。
 *
 * 用法：
 *   /sdcard/app/launcher                 # 读默认清单 /sdcard/app/startup_final.list
 *   /sdcard/app/launcher /tmp/my.list    # 读自定义清单（调试用）
 *
 * 构建：bash scripts/build.sh launcher
 * 产物：build/launcher/launcher
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define DEFAULT_LIST "/sdcard/app/startup_final.list"
#define LOG_DIR      "/sdcard/app/logs"

/* 直写日志：每次 打开->写->fsync->关闭，进程不持有日志 fd，
 * MTP/文件管理器才能随时读到（RT-Smart 上 printf 走控制台不走 fd）。 */
static void llog(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    int fd = open(LOG_DIR "/launcher.log", O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0) {
        fprintf(stderr, "launcher: log open failed: %s\n", strerror(errno));
        return;
    }
    ssize_t w = write(fd, buf, (size_t)n);
    fsync(fd);
    close(fd);
    (void)w;
}

#define MAX_LINE  512  /* 单行最大长度（路径+参数） */
#define MAX_ARGS  16   /* 单行最大 token 数（含路径） */
#define MAX_BG    32   /* 最多同时监督的后台进程数 */

static pid_t bg_pids[MAX_BG];
static int   bg_count = 0;

static void record_bg(pid_t pid)
{
    if (bg_count < MAX_BG)
        bg_pids[bg_count++] = pid;
    else
        llog("launcher: too many background apps, pid %d unsupervised\n",
                (int)pid);
}

/*
 * 解析并启动一行命令。
 * 语法: 绝对路径 [参数...] [&]
 *   返回 0 表示正常（前台项已结束/后台项已拉起），-1 表示本项失败。
 */
static int launch_one(char *line, int lineno)
{
    char *tok[MAX_ARGS];
    int   n = 0;
    int   background = 0;
    char *p = line;

    /* 1) 按空白切 token */
    while (*p != '\0') {
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\0')
            break;
        if (n >= MAX_ARGS) {
            llog("launcher: line %d: too many tokens (>%d)\n",
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
    if (strcmp(tok[n - 1], "&") == 0) {
        background = 1;
        n--;
    }
    tok[n] = NULL;
    if (n == 0)
        return 0;

    /* 3) 文件存在性预检（失败在父进程报错，比子进程 exec 报错更清晰） */
    struct stat st;
    if (stat(tok[0], &st) != 0) {
        llog("launcher: line %d: cannot access %s: %s\n",
                lineno, tok[0], strerror(errno));
        return -1;
    }

    /* gphoto2 uploads do not preserve executable mode; fix it on-board. */
    if (chmod(tok[0], 0755) != 0)
        llog("launcher: chmod %s failed: %s\n", tok[0], strerror(errno));

    /* 4) fork + execv：RT-Smart 中每个 ELF 就是独立进程（lwp） */
    pid_t pid = fork();
    if (pid < 0) {
        llog("launcher: line %d: fork failed: %s\n",
                lineno, strerror(errno));
        return -1;
    }
    if (pid == 0) { /* 子进程 */
        int child_log = open(LOG_DIR "/launcher-child.log", O_WRONLY | O_CREAT | O_APPEND, 0666);
        if (child_log >= 0) {
            dup2(child_log, STDOUT_FILENO);
            dup2(child_log, STDERR_FILENO);
            close(child_log);
        }
        execv(tok[0], tok); /* 成功后不返回（程序自身负责自己的日志） */
        llog("launcher: line %d: exec %s failed: %s\n",
                lineno, tok[0], strerror(errno));
        _exit(127);
    }

    /* 5) 父进程 */
    if (background) {
        record_bg(pid);
        llog("launcher: [bg ] line %d: started %s (pid=%d)\n",
               lineno, tok[0], (int)pid);
    } else {
        int status = 0;
        while (waitpid(pid, &status, 0) < 0 && errno == EINTR)
            ;
        if (WIFEXITED(status))
            llog("launcher: [run] line %d: %s exited, code=%d\n",
                   lineno, tok[0], WEXITSTATUS(status));
        else if (WIFSIGNALED(status))
            llog("launcher: [run] line %d: %s killed by signal %d\n",
                   lineno, tok[0], WTERMSIG(status));
        else
            llog("launcher: [run] line %d: %s finished\n", lineno, tok[0]);
    }
    return 0;
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    const char *list_path = (argc > 1) ? argv[1] : DEFAULT_LIST;
    FILE *fp;
    char  line[MAX_LINE];

    /* 日志目录不存在则创建（/sdcard 未挂载时忽略，进程内输出会自行兜底） */
    mkdir(LOG_DIR, 0777);

    /* 所有 llog 消息直写 logs/launcher.log（每次写后关闭，MTP 可随时读取） */
    int   lineno = 0;

    llog("launcher: boot launcher start, list = %s\n", list_path);

    fp = fopen(list_path, "r");
    if (fp == NULL) {
        llog("launcher: cannot open %s: %s\n",
                list_path, strerror(errno));
        return 1;
    }

    while (fgets(line, sizeof(line), fp) != NULL) {
        char *s;
        char *nl;

        lineno++;
        /* 去掉行尾换行/回车 */
        nl = strpbrk(line, "\r\n");
        if (nl != NULL)
            *nl = '\0';
        /* 跳过前导空白与注释/空行 */
        s = line;
        while (*s == ' ' || *s == '\t')
            s++;
        if (*s == '\0' || *s == '#')
            continue;

        if (launch_one(s, lineno) < 0) {
            /* 单项失败不中断后续项，仅记录 */
            llog("launcher: line %d skipped\n", lineno);
        }
    }
    fclose(fp);

    /* 监督阶段：RT-Smart 的 waitpid 行为不可靠(可能返回 0)，这里轮询存活状态：
     * 每 5 秒在日志里打一次心跳；子进程退出(waitpid 能取到)或 kill 检测不到
     * (ESRCH) 时结束该条。launcher 因此常驻直到所有后台子进程结束。 */
    if (bg_count > 0) {
        llog("launcher: supervising %d background app(s)...\n", bg_count);
        for (int i = 0; i < bg_count; i++) {
            pid_t p = bg_pids[i];
            for (;;) {
                int status = 0;
                pid_t r = waitpid(p, &status, WNOHANG);
                if (r == p) { /* 已回收，正常退出 */
                    if (WIFEXITED(status))
                        llog("launcher: [sup] pid %d exited, code=%d\n",
                               (int)p, WEXITSTATUS(status));
                    else if (WIFSIGNALED(status))
                        llog("launcher: [sup] pid %d killed by signal %d\n",
                               (int)p, WTERMSIG(status));
                    else
                        llog("launcher: [sup] pid %d finished\n", (int)p);
                    break;
                }
                if (r < 0 && errno != EINTR) {
                    llog("launcher: [sup] waitpid(%d) err %s\n",
                           (int)p, strerror(errno));
                    /* waitpid 不可用(ENOSYS/ECHILD)时用 kill 探测 */
                    if (errno == ECHILD) {
                        llog("launcher: [sup] pid %d gone (no child)\n", (int)p);
                        break;
                    }
                }
                /* 探测进程是否还在 */
                if (kill(p, 0) != 0 && errno == ESRCH) {
                    llog("launcher: [sup] pid %d no longer alive (exit status "
                           "unknown)\n", (int)p);
                    break;
                }
                sleep(10);
                llog("launcher: [sup] pid %d still alive\n", (int)p);
            }
        }
    }

    llog("launcher: all apps done, launcher exit\n");
    return 0;
}
