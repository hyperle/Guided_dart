#include "logging/log_sink.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <new>
#include <unistd.h>

#include "storage/directories.hpp"
#include "system/monotonic_clock.hpp"

namespace dart::logging {

LogSink::~LogSink() { stop(); }

bool LogSink::start(std::string_view path, platform::WriteControl* writes) noexcept {
    if (started_) return true;
    if (path.size() >= sizeof(path_)) return false;
    if (!path.empty()) std::memcpy(path_, path.data(), path.size());
    path_[path.size()] = '\0';
    writes_ = writes;
    if (!rows_) rows_.reset(new (std::nothrow) Row[kLogQueueRows]);
    if (!rows_) return false;
    head_ = tail_ = count_ = 0;
    working_ = false;
    file_closed_ = true;
    close_requested_ = false;
    dropped_ = console_dropped_ = io_errors_ = 0;
    queued_ = high_water_ = 0;
    dropped_reported_ = console_reported_ = 0;
    drop_report_ms_ = console_report_ms_ = platform::mono_ms();
    console_active_ = false;
    running_ = true;
    if (::pthread_create(&thread_, nullptr, &LogSink::entry, this) != 0) {
        running_ = false;
        return false;
    }
    started_ = true;
    return true;
}

void LogSink::set_console(bool on) noexcept { console_.store(on, std::memory_order_relaxed); }

void LogSink::enqueue(const char* text, std::size_t size) noexcept {
    std::unique_lock lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !running_.load() || count_ == kLogQueueRows) {
        ++dropped_;
        return;
    }
    size = std::min(size, kLogRowBytes - 1);
    auto& row = rows_[tail_];
    std::memcpy(row.text, text, size);
    row.size = static_cast<std::uint32_t>(size);
    tail_ = (tail_ + 1U) % kLogQueueRows;
    ++count_;
    queued_ = static_cast<std::uint32_t>(count_);
    high_water_ = std::max(high_water_.load(), static_cast<std::uint32_t>(count_));
    lock.unlock();
    ready_.notify_one();
}

void LogSink::request_close() noexcept {
    close_requested_.store(true, std::memory_order_release);
    ready_.notify_one();
}

bool LogSink::drain(std::chrono::milliseconds deadline) noexcept {
    if (!started_) return true;
    std::unique_lock lock(mutex_);
    request_close();
    return idle_cv_.wait_for(lock, deadline, [this] {
        return count_ == 0 && !working_ && file_closed_ && !close_requested_.load(std::memory_order_acquire);
    });
}

void LogSink::stop() noexcept {
    if (!started_) return;
    {
        std::lock_guard lock(mutex_);
        running_ = false;
    }
    ready_.notify_one();
    (void)::pthread_join(thread_, nullptr);
    started_ = false;
}

std::uint64_t LogSink::dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

LogSink::Stats LogSink::stats() const noexcept {
    return {dropped(), console_dropped_.load(), io_errors_.load(), queued_.load(), high_water_.load()};
}

void* LogSink::entry(void* context) noexcept {
    static_cast<LogSink*>(context)->run();
    return nullptr;
}

void LogSink::run() noexcept {
    for (;;) {
        Row row{};
        bool have_row = false, close = false;
        {
            std::unique_lock lock(mutex_);
            ready_.wait_for(lock, std::chrono::milliseconds{kIdleWaitMs}, [this] {
                return count_ != 0 || close_requested_.load() || !running_.load();
            });
            if (count_ != 0) {
                row = rows_[head_];
                head_ = (head_ + 1U) % kLogQueueRows;
                --count_;
                queued_ = static_cast<std::uint32_t>(count_);
                have_row = true;
            } else if (close_requested_.exchange(false, std::memory_order_acq_rel)) {
                close = true;
            } else if (!running_.load()) {
                break;
            }
            working_ = true;
        }
        if (have_row) write_row(row);
        report_dropped();
        report_console_dropped();
        if (close || !have_row || !enabled()) close_file();
        {
            std::lock_guard lock(mutex_);
            working_ = false;
            file_closed_ = fd_ < 0;
            if (count_ == 0 && file_closed_ && !close_requested_.load()) idle_cv_.notify_all();
        }
    }
    report_dropped(true);
    report_console_dropped(true);
    close_file();
}

bool LogSink::enabled() const noexcept { return writes_ == nullptr || writes_->logging_enabled(); }

void LogSink::write_row(const Row& row) noexcept {
    if (enabled()) write_output(row.text, row.size);
}

void LogSink::write_output(const char* text, std::size_t size) noexcept {
    if (fd_ < 0 && path_[0] != '\0') {
        storage::ensure_parent(path_);
        fd_ = ::open(path_, O_WRONLY | O_CREAT | O_APPEND, 0666);
        if (fd_ < 0) ++io_errors_;
    }
    if (fd_ >= 0) {
        std::size_t offset = 0;
        while (offset < size) {
            const auto written = ::write(fd_, text + offset, size - offset);
            if (written < 0 && errno == EINTR) continue;
            if (written <= 0) {
                ++io_errors_;
                close_file();
                break;
            }
            offset += static_cast<std::size_t>(written);
        }
    }
    // Console output remains usable when the file could not be opened.
    emit_console(text, size);
}

void LogSink::close_file() noexcept {
    if (fd_ < 0) return;
    if (::close(fd_) != 0) ++io_errors_;
    fd_ = -1;
}

void LogSink::report_dropped(bool force) noexcept {
    if (!enabled()) return;
    const auto total = dropped_.load(std::memory_order_relaxed);
    const auto now = platform::mono_ms();
    if (total == dropped_reported_ || (!force && now - drop_report_ms_ < kDropReportMs)) return;
    char line[128];
    const int size = std::snprintf(line, sizeof(line),
        "[log] ts_ms=%lld level=WARN op=log_dropped count=%llu\n",
        static_cast<long long>(now), static_cast<unsigned long long>(total - dropped_reported_));
    if (size > 0) write_output(line, static_cast<std::size_t>(size));
    dropped_reported_ = total;
    drop_report_ms_ = now;
}

void LogSink::emit_console(const char* text, std::size_t size) noexcept {
    if (!console_.load(std::memory_order_relaxed)) {
        console_active_ = false;
        return;
    }
    const auto now = platform::mono_ms();
    if (!console_active_) {
        console_active_ = true;
        console_credit_ = kConsoleRatePerSecond * 1000ULL;
        console_refill_ms_ = console_report_ms_ = now;
    }
    if (now > console_refill_ms_) {
        console_credit_ = std::min<std::uint64_t>(kConsoleRatePerSecond * 1000ULL,
            console_credit_ + static_cast<std::uint64_t>(now - console_refill_ms_) * kConsoleRatePerSecond);
        console_refill_ms_ = now;
    }
    if (console_credit_ < 1000) {
        ++console_dropped_;
        return;
    }
    console_credit_ -= 1000;
    if (::write(STDERR_FILENO, text, size) != static_cast<ssize_t>(size)) ++io_errors_;
}

void LogSink::report_console_dropped(bool force) noexcept {
    if (!enabled() || !console_.load(std::memory_order_relaxed)) return;
    const auto total = console_dropped_.load(std::memory_order_relaxed);
    const auto now = platform::mono_ms();
    if (total == console_reported_ || (!force && now - console_report_ms_ < kDropReportMs)) return;
    char line[128];
    const int size = std::snprintf(line, sizeof(line),
        "[log] ts_ms=%lld level=WARN op=console_dropped count=%llu\n",
        static_cast<long long>(now), static_cast<unsigned long long>(total - console_reported_));
    if (size > 0 && ::write(STDERR_FILENO, line, static_cast<std::size_t>(size)) != size) ++io_errors_;
    console_reported_ = total;
    console_report_ms_ = now;
}

} // namespace dart::logging
