#ifndef DART_LOGGING_LOGGER_CPP
#define DART_LOGGING_LOGGER_CPP

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string_view>
#include <unistd.h>

#include "logging/log_sink.cpp"

namespace dart::logging {

/* Feature ownership tag, printed at the start of every line so a reader can
 * attribute it to one subsystem instead of parsing the body. */
enum class Tag : std::uint8_t { kApp, kCamera, kVision, kRecord, kServo, kImu, kLog };

inline const char* tag_text(Tag tag) noexcept {
    switch (tag) {
    case Tag::kApp: return "app";
    case Tag::kCamera: return "camera";
    case Tag::kVision: return "vision";
    case Tag::kRecord: return "record";
    case Tag::kServo: return "servo";
    case Tag::kImu: return "imu";
    case Tag::kLog: return "log";
    }
    return "app";
}

/* Structured file log with an optional console mirror.
 *
 * line() only formats the line and hands it to the bounded sink queue; the sink
 * thread owns open/write/fsync/close and the console mirror. On the board an
 * fsync on the FAT partition waits for the storage device to drain the image
 * stream (100-400 ms measured per status batch), so doing it here used to stall
 * acquisition, detection and the servo for that long once per second. The
 * durability contract is unchanged: each batch still ends with fsync+close and
 * the file stays closed between batches, so MTP can read it and a reset loses at
 * most the current second.
 *
 * A failed log write must not become a console error loop: console output is
 * best-effort, never retried, and rate limited. If the sink thread cannot be
 * created the logger falls back to synchronous writes on the caller thread. */
class Logger {
public:
    explicit Logger(platform::WriteControl* writes = nullptr) noexcept : writes_(writes) {}
    ~Logger() {
        (void)sink_.drain(std::chrono::milliseconds{kShutdownDrainMs});
        sink_.stop();
        sink_.finish_console();
    }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    bool open(std::string_view path) noexcept {
        if (path.empty() || path.size() >= sizeof(path_))
            return false;
        std::memcpy(path_, path.data(), path.size());
        path_[path.size()] = '\0';
        if (writes_ != nullptr && !writes_->logging_enabled()) {
            /* Paused at startup: the file stays untouched, but the sink must
             * still start so writes resume asynchronously once the host
             * unpauses instead of silently staying on the synchronous path. */
            (void)sink_.start(std::string_view{path_, path.size()}, writes_);
            return true;
        }
        storage::ensure_parent(path_);
        const int fd = ::open(path_, O_WRONLY | O_CREAT | O_APPEND, 0666);
        if (fd < 0)
            return false;
        ::close(fd);
        /* The writer thread failing to start keeps the old synchronous path. */
        (void)sink_.start(std::string_view{path_, path.size()}, writes_);
        return true;
    }

    /* Mirror every written line to stderr. Off by default: only a foreground
     * run on the serial console benefits, and launcher redirects a child's
     * stderr into <app>.console.log, where the mirror would only double I/O.
     * Mirroring runs on the sink thread as well, so a console-only run no
     * longer blocks the capture loop on the serial port. */
    void set_console(bool on) noexcept { sink_.set_console(on); }

    void begin_batch() noexcept { batching_.store(true, std::memory_order_relaxed); }

    /* Ends a batch: the sink writes the queued lines and then fsyncs and closes
     * the file on its own thread. The call itself never touches storage. */
    void end_batch() noexcept {
        batching_.store(false, std::memory_order_relaxed);
        sink_.request_flush();
    }

    void line(Tag tag, const char* fmt, ...) noexcept {
        if (writes_ != nullptr && !writes_->logging_enabled()) {
            end_batch();
            return;
        }
        char body[512];
        char out[640];
        va_list ap;
        va_start(ap, fmt);
        const int body_n = std::vsnprintf(body, sizeof(body), fmt, ap);
        va_end(ap);
        if (body_n < 0)
            return;
        int out_n = std::snprintf(
            out, sizeof(out), "[%s] ts_ms=%lld %s", tag_text(tag),
            static_cast<long long>(platform::mono_ms()), body);
        if (out_n < 0)
            return;
        if (static_cast<std::size_t>(out_n) >= sizeof(out))
            out_n = static_cast<int>(sizeof(out)) - 1;
        /* Only a copy into the bounded queue happens here; storage I/O and the
         * console mirror belong to the sink thread. */
        if (sink_.started())
            sink_.enqueue(out, static_cast<std::size_t>(out_n));
        else
            sink_.write_now(out, static_cast<std::size_t>(out_n));
        if (!batching_.load(std::memory_order_relaxed))
            end_batch();
    }

    /* Blocks until queued lines are written and the file is fsynced+closed
     * again. Shutdown and tests use it to observe the file; the capture loop
     * never calls it. */
    bool flush(std::chrono::milliseconds deadline) noexcept { return sink_.drain(deadline); }

    /* Lines discarded because the bounded queue was full or its mutex was held.
     * Diagnostics only: dropping never blocks the producer. */
    std::uint64_t dropped() const noexcept { return sink_.dropped(); }

private:
    static constexpr std::int64_t kShutdownDrainMs = 2000;
    platform::WriteControl* writes_ = nullptr;
    char path_[512]{};
    std::atomic<bool> batching_{false};
    LogSink sink_{};
};

} // namespace dart::logging

#endif // DART_LOGGING_LOGGER_CPP
