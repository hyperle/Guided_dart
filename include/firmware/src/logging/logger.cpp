#ifndef DART_LOGGING_LOGGER_CPP
#define DART_LOGGING_LOGGER_CPP

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string_view>
#include <unistd.h>

#include "storage/directories.cpp"
#include "system/monotonic_clock.cpp"
#include "system/write_control.cpp"

namespace dart::logging {

/* Console mirror rate limit, in lines per second (0 disables the limit).
 * Every periodic line is emitted once per --status-ms (default 1 s, ~6 lines),
 * so normal operation never touches the limit. A per-frame failure path can
 * emit ~95 lines/s, which exceeds a 115200 console (11.5 KB/s) and would block
 * the foreground process; 30 lines/s keeps the console near 6.6 KB/s. */
inline constexpr std::uint32_t kConsoleRatePerSecond = 30;
inline constexpr std::uint32_t kConsoleBurst = kConsoleRatePerSecond;
/* At most one dropped-line summary per second, independent of line batching. */
inline constexpr std::int64_t kConsoleDropReportMs = 1000;

/* File logging with an optional console mirror. A failed log write must not
 * become a console error loop: console output is best-effort, never retried,
 * and rate limited. */
class Logger {
public:
    explicit Logger(platform::WriteControl* writes = nullptr) noexcept : writes_(writes) {}
    ~Logger() {
        end_batch();
        flush_console_dropped(true);
    }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    bool open(std::string_view path) noexcept {
        if (path.empty() || path.size() >= sizeof(path_))
            return false;
        std::memcpy(path_, path.data(), path.size());
        path_[path.size()] = '\0';
        if (writes_ != nullptr && !writes_->logging_enabled())
            return true;
        storage::ensure_parent(path_);
        const int fd = ::open(path_, O_WRONLY | O_CREAT | O_APPEND, 0666);
        if (fd < 0)
            return false;
        ::close(fd);
        return true;
    }

    /* Mirror every written line to stderr. Off by default: only a foreground
     * run on the serial console benefits, and launcher redirects a child's
     * stderr into <app>.console.log, where the mirror would only double I/O. */
    void set_console(bool on) noexcept {
        if (on && !console_) {
            /* Start with a full bucket so the startup burst (camera_create,
             * detector_prepare, csv_open …) is never dropped, and start the
             * drop-report window now so the first summary is not emitted on the
             * first dropped line. */
            const std::int64_t now = platform::mono_ms();
            console_tokens_ = kConsoleBurst;
            console_credit_ = 0;
            console_refill_ms_ = now;
            console_drop_report_ms_ = now;
        }
        console_ = on;
    }

    void begin_batch() noexcept { batching_ = true; }

    void end_batch() noexcept {
        batching_ = false;
        if (fd_ >= 0) {
            ::fsync(fd_);
            ::close(fd_);
            fd_ = -1;
        }
        flush_console_dropped();
    }

    void line(const char* fmt, ...) noexcept {
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
            out, sizeof(out), "ts_ms=%lld %s", static_cast<long long>(platform::mono_ms()), body);
        if (out_n < 0)
            return;
        if (static_cast<std::size_t>(out_n) >= sizeof(out))
            out_n = static_cast<int>(sizeof(out)) - 1;
        if (fd_ < 0 && path_[0] != '\0') {
            storage::ensure_parent(path_);
            fd_ = ::open(path_, O_WRONLY | O_CREAT | O_APPEND, 0666);
        }
        if (fd_ >= 0)
            (void)!::write(fd_, out, static_cast<std::size_t>(out_n));
        /* Mirror after the file write and before end_batch(), so a dropped-line
         * summary always follows the line that triggered the batch. */
        emit_console(out, static_cast<std::size_t>(out_n));
        if (!batching_)
            end_batch();
    }

private:
    void emit_console(const char* out, std::size_t length) noexcept {
        if (!console_)
            return;
        if (kConsoleRatePerSecond > 0) {
            /* Sub-millisecond calls must not lose their fraction: keep a credit
             * counter in thousandths of a line instead of resetting the clock. */
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
        (void)!::write(STDERR_FILENO, out, length);
    }

    /* Report drops at most once per kConsoleDropReportMs. The summary bypasses
     * the token bucket but never exceeds one line per interval, so a flood can
     * not turn into a second flood of summaries. `force` reports the remaining
     * count when the process exits, so the tail of a flood is never lost. */
    void flush_console_dropped(bool force = false) noexcept {
        if (!console_ || console_dropped_ == 0)
            return;
        if (writes_ != nullptr && !writes_->logging_enabled())
            return;
        const std::int64_t now = platform::mono_ms();
        if (!force && now - console_drop_report_ms_ < kConsoleDropReportMs)
            return;
        char buffer[128];
        const int written = std::snprintf(
            buffer, sizeof(buffer),
            "ts_ms=%lld level=WARN module=log op=console_dropped count=%llu\n",
            static_cast<long long>(now),
            static_cast<unsigned long long>(console_dropped_));
        if (written > 0)
            (void)!::write(STDERR_FILENO, buffer, static_cast<std::size_t>(written));
        console_dropped_ = 0;
        console_drop_report_ms_ = now;
    }

    platform::WriteControl* writes_ = nullptr;
    char path_[512]{};
    int fd_ = -1;
    bool batching_ = false;
    bool console_ = false;
    std::uint32_t console_tokens_ = 0;
    std::uint64_t console_credit_ = 0;   /* thousandths of a line, for sub-ms calls */
    std::uint64_t console_dropped_ = 0;
    std::int64_t console_refill_ms_ = 0;
    std::int64_t console_drop_report_ms_ = 0;
};

} // namespace dart::logging

#endif // DART_LOGGING_LOGGER_CPP
