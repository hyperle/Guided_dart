#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string_view>

#include "logging/log_sink.hpp"
#include "system/write_control.hpp"

namespace dart::logging {
/* File logging only. A failed log write must not become a console error loop. */
class Logger {
public:
    explicit Logger(platform::WriteControl* writes = nullptr) noexcept
        : writes_(writes) {}
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    bool open(std::string_view path) noexcept;

    void begin_batch() noexcept { batching_.store(true, std::memory_order_relaxed); }

    void end_batch() noexcept;

    void line(const char* fmt, ...) noexcept;
    // Explicit durability barrier for shutdown/readback; never used per frame.
    bool flush(std::chrono::milliseconds deadline) noexcept;
    std::uint64_t dropped() const noexcept { return sink_.dropped(); }

private:
    platform::WriteControl* writes_ = nullptr;
    char path_[512]{};
    std::atomic<bool> batching_{false};
    LogSink sink_{};
};

} // namespace dart::logging
