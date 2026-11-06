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

// Adapted from uart-rt-log b320f32f6: producers copy bounded rows; the worker
// owns open/write/fsync/close so slow storage cannot stall image acquisition.
class LogSink final {
public:
    static constexpr std::size_t kLogQueueRows = 256;
    static constexpr std::size_t kLogRowBytes = 640;

private:
    static constexpr std::int64_t kLogIdleWaitMs = 100;
    static constexpr std::int64_t kLogDropReportMs = 1000;

public:
    LogSink() noexcept = default;
    ~LogSink();
    LogSink(const LogSink&) = delete;
    LogSink& operator=(const LogSink&) = delete;

    // Allocate a fixed queue and start the sole owner of file I/O.
    bool start(std::string_view path, platform::WriteControl* writes) noexcept;
    bool started() const noexcept;

    /* Copies one formatted line (without the trailing '\0' accounting) into the
     * queue. The mutex is only held for queue bookkeeping and a bounded memcpy;
     * the writer thread never holds it across open/write/fsync, so waiting for
     * it can never wait for storage. A full queue drops the line and is counted,
     * without increasing memory use or waiting for storage. */
    void enqueue(const char* text, std::size_t size) noexcept;

    /* Asks the writer to flush and close the file after the queued lines. The
     * request is a separate flag, not a queue slot, so a full queue can never
     * swallow it and leave the file open. */
    void request_flush() noexcept;

    /* Blocks until every queued line is written and the following flush
     * finished (the file is fsynced and closed again). Used by shutdown and by
     * tests that need the file to be observable. */
    bool drain(std::chrono::milliseconds deadline) noexcept;

    void stop() noexcept;

    std::uint64_t dropped() const noexcept;

private:
    struct Row {
        char text[kLogRowBytes];
        std::uint32_t size;
    };

    static void* entry(void* context) noexcept;

    void run() noexcept;

    void write_row(const Row& row) noexcept;

    /* fsync+close is the durability point of a batch; it runs on this thread. */
    void flush_once() noexcept;

    /* Emits one summary line per second when the bounded queue had to drop
     * lines, so a diagnostic burst stays observable without blocking anyone. */
    void report_dropped() noexcept;

    std::unique_ptr<Row[]> rows_{};
    char path_[512]{};
    platform::WriteControl* writes_ = nullptr;
    int fd_ = -1;
    pthread_t thread_{};
    bool started_ = false;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::condition_variable idle_cv_;
    std::size_t head_ = 0, tail_ = 0, count_ = 0;
    bool running_ = false, flush_requested_ = false, flushing_ = false;
    std::atomic<std::uint64_t> dropped_{0};
    // Only the writer thread updates the drop summary state.
    std::uint64_t dropped_reported_ = 0;
    std::int64_t drop_report_ms_ = 0;
};

} // namespace dart::logging
