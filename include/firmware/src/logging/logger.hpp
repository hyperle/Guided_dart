#pragma once

#include <cstddef>
#include <fcntl.h>
#include <string_view>

#include "system/write_control.hpp"

namespace dart::logging {
/* File logging only. A failed log write must not become a console error loop. */
class Logger {
public:
    explicit Logger(platform::WriteControl* writes = nullptr) noexcept
        : writes_(writes) {}
    ~Logger() { end_batch(); }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    bool open(std::string_view path) noexcept;

    void begin_batch() noexcept { batching_ = true; }

    void end_batch() noexcept;

    void line(const char* fmt, ...) noexcept;

private:
    platform::WriteControl* writes_ = nullptr;
    char path_[512]{};
    int fd_ = -1;
    bool batching_ = false;
};

} // namespace dart::logging
