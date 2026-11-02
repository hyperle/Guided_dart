#ifndef DART_LOGGING_LOG_SINK_CPP
#define DART_LOGGING_LOG_SINK_CPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <new>
#include <pthread.h>
#include <string_view>
#include <unistd.h>

#include "storage/directories.cpp"
#include "system/monotonic_clock.cpp"
#include "system/write_control.cpp"

namespace dart::logging {

/* Console mirror rate limit, in lines per second (0 disables the limit).
 * Every periodic line is emitted once per --status-ms (default 1 s, ~7 lines),
 * so normal operation never touches the limit. A per-frame failure path can
 * emit ~95 lines/s, which exceeds a 115200 console (11.5 KB/s) and would block
 * the foreground process; 30 lines/s keeps the console near 6.6 KB/s. */
inline constexpr std::uint32_t kConsoleRatePerSecond = 30;
inline constexpr std::uint32_t kConsoleBurst = kConsoleRatePerSecond;
/* At most one dropped-line summary per second, independent of line batching. */
inline constexpr std::int64_t kConsoleDropReportMs = 1000;

/* Bounded line queue between the producers and the file writer. One slot is a
 * complete formatted line, so the sink never reformats on the writer thread.
 * 256 rows (161 KiB) cover ~50 s of the periodic batch (5 rows/s) and ~2.7 s of
 * the documented 95 line/s failure burst, matching the 256-row ring the CSV
 * writer already uses; a longer flood drops rows instead of growing memory. */
inline constexpr std::size_t kLogQueueRows = 256;
inline constexpr std::size_t kLogRowBytes = 640;
/* Idle wake-up. Producers notify on every enqueue and flush request, so this
 * only bounds how long the thread sleeps between notifications. */
inline constexpr std::int64_t kLogIdleWaitMs = 100;
/* Dropped-row summaries use the same "one line per second" budget as the
 * console summary; they are best-effort diagnostics and never block. */
inline constexpr std::int64_t kLogDropReportMs = 1000;

/* Single-writer file sink for the structured log.
 *
 * The capture loop calls line()/end_batch() from the hot path: formatting is
 * cheap, but open/write/fsync/close is not. On this board an fsync on the FAT
 * partition waits for the storage device to drain the recorder's image stream,
 * which measured 100-400 ms per status batch and stalled acquisition for that
 * long. Those calls now run on this thread; producers only copy a line into the
 * bounded queue.
 *
 * Durability is unchanged: every batch still ends with fsync+close and the file
 * stays closed between batches, so MTP can read it and a reset loses at most the
 * current second. Only the thread that waits for storage changed.
 */
class LogSink final {
public:
    LogSink() noexcept = default;
    ~LogSink() { stop(); }
    LogSink(const LogSink&) = delete;
    LogSink& operator=(const LogSink&) = delete;

    /* Starts the writer thread once. An empty path keeps the file closed but
     * still moves console mirroring off the producer. Returns false when no
     * thread could be started; the caller then keeps writing synchronously. */
    bool start(std::string_view path, platform::WriteControl* writes) noexcept {
        if (started_)
            return true;
        if (path.size() >= sizeof(path_))
            return false;
        std::memcpy(path_, path.data(), path.size());
        path_[path.size()] = '\0';
        writes_ = writes;
        if (!rows_)
            rows_.reset(new (std::nothrow) Row[kLogQueueRows]);
        if (!rows_)
            return false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            head_ = tail_ = count_ = 0;
            flush_requested_ = flushing_ = false;
            dropped_.store(0, std::memory_order_relaxed);
            dropped_reported_ = 0;
            running_ = true;
        }
        if (::pthread_create(&thread_, nullptr, &LogSink::entry, this) != 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
            return false;
        }
        started_ = true;
        return true;
    }
    bool started() const noexcept { return started_; }

    void set_console(bool on) noexcept {
        {
            std::lock_guard<std::mutex> lock(console_mutex_);
            if (on && !console_) {
                /* Start with a full bucket so the startup burst (camera_create,
                 * detector_prepare, csv_open …) is never dropped, and start the
                 * drop-report window now so the first summary is not emitted on
                 * the first dropped line. */
                const std::int64_t now = platform::mono_ms();
                console_tokens_ = kConsoleBurst;
                console_credit_ = 0;
                console_refill_ms_ = now;
                console_drop_report_ms_ = now;
            }
            console_ = on;
        }
        if (on)
            (void)start(std::string_view{path_}, writes_);
    }

    /* Copies one formatted line (without the trailing '\0' accounting) into the
     * queue. The mutex is only held for queue bookkeeping and a bounded memcpy;
     * the writer thread never holds it across open/write/fsync, so waiting for
     * it can never wait for storage. A full queue drops the line and is counted,
     * exactly like the console rate limiter. */
    void enqueue(const char* text, std::size_t size) noexcept {
        if (size >= kLogRowBytes)
            size = kLogRowBytes - 1;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || count_ >= kLogQueueRows) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        Row& row = rows_[tail_];
        std::memcpy(row.text, text, size);
        row.text[size] = '\0';
        row.size = static_cast<std::uint32_t>(size);
        tail_ = (tail_ + 1U) % kLogQueueRows;
        ++count_;
        ready_.notify_one();
    }

    /* Asks the writer to flush and close the file after the queued lines. The
     * request is an atomic flag, not a queue slot, so a full queue can never
     * swallow it and leave the file open. */
    void request_flush() noexcept {
        if (!started_) {
            flush_once();
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            flush_requested_ = true;
        }
        ready_.notify_one();
    }

    /* Blocks until every queued line is written and the following flush
     * finished (the file is fsynced and closed again). Used by shutdown and by
     * tests that need the file to be observable. */
    bool drain(std::chrono::milliseconds deadline) noexcept {
        if (!started_) {
            flush_once();
            return true;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        flush_requested_ = true;
        ready_.notify_one();
        return idle_cv_.wait_for(lock, deadline, [this] {
            return count_ == 0 && !flush_requested_ && !flushing_;
        });
    }

    void stop() noexcept {
        if (!started_)
            return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
        }
        ready_.notify_all();
        (void)::pthread_join(thread_, nullptr);
        started_ = false;
    }

    /* Emits the remaining console drop summary once at shutdown. */
    void finish_console() noexcept { flush_console_dropped(true); }
    std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

    /* Synchronous fallback used only when the writer thread could not start
     * (pthread_create failure): same file and console behaviour as before, on
     * the caller thread. */
    void write_now(const char* text, std::size_t size) noexcept {
        Row row{};
        if (size >= kLogRowBytes)
            size = kLogRowBytes - 1;
        std::memcpy(row.text, text, size);
        row.text[size] = '\0';
        row.size = static_cast<std::uint32_t>(size);
        write_row(row);
    }

private:
    struct Row {
        char text[kLogRowBytes];
        std::uint32_t size;
    };

    static void* entry(void* context) noexcept {
        static_cast<LogSink*>(context)->run();
        return nullptr;
    }

    void run() noexcept {
        for (;;) {
            Row row{};
            bool have_row = false, have_flush = false, stopping = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait_for(lock, std::chrono::milliseconds{kLogIdleWaitMs}, [this] {
                    return count_ != 0 || flush_requested_ || !running_;
                });
                if (count_ != 0) {
                    row = rows_[head_];
                    head_ = (head_ + 1U) % kLogQueueRows;
                    --count_;
                    have_row = true;
                } else if (flush_requested_) {
                    flush_requested_ = false;
                    flushing_ = true;
                    have_flush = true;
                } else if (!running_) {
                    stopping = true;
                }
            }
            if (have_row)
                write_row(row);
            if (have_flush) {
                flush_once();
                std::lock_guard<std::mutex> lock(mutex_);
                flushing_ = false;
            }
            if (stopping)
                break;
            std::lock_guard<std::mutex> lock(mutex_);
            if (count_ == 0 && !flush_requested_ && !flushing_)
                idle_cv_.notify_all();
        }
        flush_once();
    }

    void write_row(const Row& row) noexcept {
        if (writes_ != nullptr && !writes_->logging_enabled()) {
            /* A host pause silences both sinks and commits the tail written so
             * far, then leaves the file closed so MTP can read it. Repeated rows
             * are no-ops because flush_once() already closed the descriptor. */
            flush_once();
            return;
        }
        if (fd_ < 0 && path_[0] != '\0') {
            storage::ensure_parent(path_);
            fd_ = ::open(path_, O_WRONLY | O_CREAT | O_APPEND, 0666);
        }
        if (fd_ >= 0 && row.size > 0)
            (void)!::write(fd_, row.text, row.size);
        emit_console(row.text, row.size);
    }

    /* fsync+close is the durability point of a batch; it runs on this thread. */
    void flush_once() noexcept {
        report_dropped();
        if (fd_ >= 0) {
            (void)::fsync(fd_);
            (void)::close(fd_);
            fd_ = -1;
        }
        flush_console_dropped();
    }

    /* Emits one summary line per second when the bounded queue had to drop
     * lines, so a diagnostic burst stays observable without blocking anyone. */
    void report_dropped() noexcept {
        if (fd_ < 0)
            return;
        const auto total = dropped_.load(std::memory_order_relaxed);
        if (total == dropped_reported_)
            return;
        const std::int64_t now = platform::mono_ms();
        if (now - drop_report_ms_ < kLogDropReportMs)
            return;
        char line[96];
        const int written = std::snprintf(
            line, sizeof(line),
            "[log] ts_ms=%lld level=WARN op=log_dropped count=%llu\n",
            static_cast<long long>(now), static_cast<unsigned long long>(total - dropped_reported_));
        if (written <= 0)
            return;
        (void)!::write(fd_, line, static_cast<std::size_t>(written));
        dropped_reported_ = total;
        drop_report_ms_ = now;
    }

    void emit_console(const char* text, std::size_t length) noexcept {
        if (length == 0)
            return;
        {
            std::lock_guard<std::mutex> lock(console_mutex_);
            if (!console_)
                return;
            if (kConsoleRatePerSecond > 0) {
                /* Sub-millisecond calls must not lose their fraction: keep a
                 * credit counter in thousandths of a line instead of resetting
                 * the clock. */
                const std::int64_t now = platform::mono_ms();
                const std::int64_t elapsed = now - console_refill_ms_;
                if (elapsed > 0) {
                    console_credit_ += static_cast<std::uint64_t>(elapsed) * kConsoleRatePerSecond;
                    console_refill_ms_ = now;
                    const std::uint64_t whole = console_credit_ / 1000ULL;
                    if (whole > 0) {
                        console_credit_ -= whole * 1000ULL;
                        const std::uint64_t refilled = console_tokens_ + whole;
                        console_tokens_ = refilled >= kConsoleBurst
                                              ? kConsoleBurst
                                              : static_cast<std::uint32_t>(refilled);
                    }
                }
                if (console_tokens_ == 0) {
                    ++console_dropped_;
                    return;
                }
                --console_tokens_;
            }
        }
        (void)!::write(STDERR_FILENO, text, length);
    }

    /* Reports drops at most once per kConsoleDropReportMs. The summary bypasses
     * the token bucket but never exceeds one line per interval, so a flood can
     * not turn into a second flood of summaries. `force` reports the remaining
     * count when the process exits, so the tail of a flood is never lost. */
    void flush_console_dropped(bool force = false) noexcept {
        char buffer[128];
        int written = 0;
        {
            std::lock_guard<std::mutex> lock(console_mutex_);
            if (!console_ || console_dropped_ == 0)
                return;
            if (writes_ != nullptr && !writes_->logging_enabled())
                return;
            const std::int64_t now = platform::mono_ms();
            if (!force && now - console_drop_report_ms_ < kConsoleDropReportMs)
                return;
            written = std::snprintf(
                buffer, sizeof(buffer),
                "[log] ts_ms=%lld level=WARN op=console_dropped count=%llu\n",
                static_cast<long long>(now), static_cast<unsigned long long>(console_dropped_));
            console_dropped_ = 0;
            console_drop_report_ms_ = now;
        }
        if (written > 0)
            (void)!::write(STDERR_FILENO, buffer, static_cast<std::size_t>(written));
    }

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
    /* Writer-thread drop summary state, guarded by the queue mutex in practice
     * (only the writer reads it); kept outside the mutex to keep the hot enqueue
     * path free of bookkeeping. */
    std::uint64_t dropped_reported_ = 0;
    std::int64_t drop_report_ms_ = 0;

    std::mutex console_mutex_;
    bool console_ = false;
    std::uint32_t console_tokens_ = 0;
    std::uint64_t console_credit_ = 0; /* thousandths of a line, for sub-ms calls */
    std::uint64_t console_dropped_ = 0;
    std::int64_t console_refill_ms_ = 0;
    std::int64_t console_drop_report_ms_ = 0;
};

} // namespace dart::logging

#endif // DART_LOGGING_LOG_SINK_CPP
