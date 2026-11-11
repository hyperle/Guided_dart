#pragma once

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <string_view>

#include "logging/log_sink.hpp"
#include "system/write_control.hpp"

namespace dart::logging {
enum class Tag : std::uint8_t { kApp, kCamera, kVision, kRecord, kServo, kImu, kLog, kWifi };

const char* tag_text(Tag tag) noexcept;

// Producers format bounded rows; the sink owns file and console I/O.
class Logger {
public:
    explicit Logger(platform::WriteControl* writes = nullptr) noexcept
        : writes_(writes) {}
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    bool open(std::string_view path) noexcept;
    void set_console(bool on) noexcept { sink_.set_console(on); }

    void begin_batch() noexcept { batching_.store(true, std::memory_order_relaxed); }

    void end_batch() noexcept;

    void line(Tag tag, const char* fmt, ...) noexcept;
    // Existing diagnostics without an explicit owner use the application tag.
    void line(const char* fmt, ...) noexcept;
    // Wait for queued output and close the file; this is not a durability barrier.
    bool flush(std::chrono::milliseconds deadline) noexcept;
    std::uint64_t dropped() const noexcept { return sink_.dropped(); }
    LogSink::Stats stats() const noexcept { return sink_.stats(); }

private:
    void vline(Tag tag, const char* fmt, va_list args) noexcept;
    platform::WriteControl* writes_ = nullptr;
    std::atomic<bool> batching_{false};
    LogSink sink_{};
};

} // namespace dart::logging
