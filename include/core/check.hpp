#pragma once

#include <cstdlib>

#include "core/log.hpp"

namespace dart {

// 硬件初始化/不可恢复错误：直接退出。这里有意义地"不做保护"——
// 采集链路建不起来时，机器人继续跑只会输出垃圾结果，不如立刻停下并说清楚。
// FATAL 走 log_line：必须落进日志文件（板子没有 shell，stderr 没人看得到）。
[[noreturn]] inline void die(const char *what, long ret = 0) {
    if (ret)
        log_line("FATAL: %s (ret=0x%lx)\n", what, ret);
    else
        log_line("FATAL: %s\n", what);
    std::exit(1);
}

inline void check(bool ok, const char *what, long ret = 0) {
    if (!ok)
        die(what, ret);
}

} // namespace dart
