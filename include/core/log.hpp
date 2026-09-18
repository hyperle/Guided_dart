#pragma once

namespace dart {

// 板端日志。两条硬性约束决定它长这样：
//   1) 板子没有 shell，日志文件是唯一观测窗口，必须每行都真正落盘：
//      板子的 FAT 写入走页缓存，不 fsync 的话断电/拔卡就丢（对照 board_probe/plog：
//      它每行 fsync，日志从来没缺过行）。
//   2) 同时写 stdout：launcher 会把子进程 stdout 转存到 launcher-child.log，
//      两条通道互为备份，任一条断了都还能看到现场。
//
// 用法就是 printf：log_line("fps %.1f", v);
void log_line(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// 信号处理器专用：只 write()，异步信号安全（不要在普通代码里用它）
void log_fault(const char *msg, unsigned len);

// fork 之后在子进程里调用：换一份自己的日志 fd（O_APPEND），避免父子共享文件偏移
void log_reopen();

// 实际生效的日志路径（兜底路径生效时能一眼看出来）；尚未打开则返回 ""
const char *log_path();

} // namespace dart
