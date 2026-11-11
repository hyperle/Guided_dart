#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <string_view>

#include "system/write_control.hpp"

namespace dart::logging {

// Bounded multi-producer queue. Its worker owns file and console output;
// producers drop on contention or a full queue and never wait for I/O.
class LogSink final {
public:
    static constexpr std::size_t kLogQueueRows = 256;
    static constexpr std::size_t kLogRowBytes = 640;
    static constexpr std::uint32_t kConsoleRatePerSecond = 30;

    struct Stats {
        std::uint64_t dropped = 0, console_dropped = 0, io_errors = 0;
        std::uint32_t queued = 0, high_water = 0;
    };

    LogSink() noexcept = default;
    ~LogSink();
    LogSink(const LogSink&) = delete;
    LogSink& operator=(const LogSink&) = delete;

    // Lifecycle calls are serialized by the owner. An empty path disables
    // file output; set_console() independently selects the stderr mirror.
    bool start(std::string_view path, platform::WriteControl* writes) noexcept;
    void set_console(bool on) noexcept;
    void enqueue(const char* text, std::size_t size) noexcept;
    void request_close() noexcept;
    // Includes the in-flight row and closing the descriptor, without syncing.
    bool drain(std::chrono::milliseconds deadline) noexcept;
    void stop() noexcept;
    std::uint64_t dropped() const noexcept;
    Stats stats() const noexcept;

private:
    static constexpr std::int64_t kIdleWaitMs = 100;
    static constexpr std::int64_t kDropReportMs = 1000;
    struct Row {
        char text[kLogRowBytes];
        std::uint32_t size;
    };
    static void* entry(void* context) noexcept;
    void run() noexcept;
    bool enabled() const noexcept;
    void write_row(const Row& row) noexcept;
    void write_output(const char* text, std::size_t size) noexcept;
    void close_file() noexcept;
    void report_dropped(bool force = false) noexcept;
    void emit_console(const char* text, std::size_t size) noexcept;
    void report_console_dropped(bool force = false) noexcept;

    std::unique_ptr<Row[]> rows_{};
    char path_[512]{};
    platform::WriteControl* writes_ = nullptr;
    int fd_ = -1;
    pthread_t thread_{};
    bool started_ = false;
    std::mutex mutex_;
    std::condition_variable ready_, idle_cv_;
    std::size_t head_ = 0, tail_ = 0, count_ = 0;
    bool working_ = false, file_closed_ = true;
    std::atomic<bool> running_{false}, console_{false};
    std::atomic<std::uint32_t> close_requested_{0};
    std::atomic<std::uint64_t> dropped_{0}, console_dropped_{0}, io_errors_{0};
    std::atomic<std::uint32_t> queued_{0}, high_water_{0};
    // Only the writer touches reporting and token bucket state.
    std::uint64_t dropped_reported_ = 0, console_reported_ = 0;
    std::int64_t drop_report_ms_ = 0, console_report_ms_ = 0, console_refill_ms_ = 0;
    std::uint64_t console_credit_ = 0; // Thousandths of a line.
    bool console_active_ = false;
};

} // namespace dart::logging
