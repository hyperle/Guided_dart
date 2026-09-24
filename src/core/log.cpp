#include "core/log.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace dart {
namespace {

// 日志落点按顺序试。第一个是正常路径（launcher 与 MTP 都从 /sdcard/app/logs 读）。
// 后面两个是**兜底**：上一版只有第一个，open 失败时静默降级成"只写 stdout"，
// 结果是"程序没跑"和"跑了但卡写不进去"在板端长得一模一样 —— 排障时最贵的坑。
constexpr const char *kCandidates[] = {
    "/sdcard/app/logs/self_guiding_dart.log",
    "/sdcard/app/self_guiding_dart.log",
    "/sdcard/self_guiding_dart.log", // /sdcard/app 不在时也要留下痕迹
    "/tmp/self_guiding_dart.log",
};

int         g_fd = -1;
bool        g_tried = false;
const char *g_path = "";

void open_log_file() {
    if (g_tried)
        return;
    g_tried = true;

    for (const char *path : kCandidates) {
        char dir[128];
        std::snprintf(dir, sizeof(dir), "%s", path);
        char *slash = std::strrchr(dir, '/');
        if (slash != nullptr && slash != dir) {
            *slash = '\0';
            mkdir(dir, 0777); // 已存在返回 EEXIST，无需处理；失败也无妨，open 会再报
        }

        const int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0666);
        if (fd >= 0) {
            g_fd = fd;
            g_path = path;
            return;
        }
    }
}

} // namespace

void log_line(const char *fmt, ...) {
    // 512 字节：识别/跟踪层的状态行字段多、且中文一个字 3 字节。板端 round12 实测
    // `识别/跟踪参数` 行 290 字节 > 256 被截断，**尾部换行也一起丢了**，于是下一行
    // 被粘在同一行里 —— "一行一条证据"的前提没了，复盘时对不上账。
    char buf[512];

    va_list ap;
    va_start(ap, fmt);
    int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if (n > static_cast<int>(sizeof(buf)) - 1) {
        // 就算还是太长，也必须保住换行：截断只许丢内容，不许把两行粘起来。
        n = static_cast<int>(sizeof(buf)) - 1;
        buf[n - 1] = '\n';
    }

    open_log_file();

    if (g_fd >= 0) {
        (void)!write(g_fd, buf, static_cast<size_t>(n));
        fsync(g_fd); // 断电容错靠这一行 —— 没它日志会停在页缓存里
    }
    (void)!write(STDOUT_FILENO, buf, static_cast<size_t>(n));
}

// fork 出来的看门狗子进程用：丢掉继承来的 fd，自己按 O_APPEND 重新打开，
// 免得两个进程共享同一个文件偏移互相踩。
void log_reopen() {
    if (g_fd >= 0)
        close(g_fd);
    g_fd = -1;
    g_tried = false;
    open_log_file();
    if (g_fd >= 0)
        log_line("日志: 看门狗进程接管 %s\n", g_path);
}

const char *log_path() { return g_path; }

// 供信号处理器使用：只做 write()，不碰 vsnprintf/fsync（那些在信号上下文里不安全）。
// 板端没有 shell，"进程静默消失"是最难查的一种失败，哪怕只有一行也要留下。
void log_fault(const char *msg, unsigned len) {
    if (g_fd >= 0)
        (void)!write(g_fd, msg, len);
    (void)!write(STDOUT_FILENO, msg, len);
}

} // namespace dart
