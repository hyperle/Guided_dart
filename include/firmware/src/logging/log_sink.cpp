#include "logging/log_sink.hpp"

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

bool LogSink::started() const noexcept { return started_; }

void LogSink::enqueue(const char* text, std::size_t size) noexcept {
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

void LogSink::request_flush() noexcept {
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

bool LogSink::drain(std::chrono::milliseconds deadline) noexcept {
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

void LogSink::stop() noexcept {
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

std::uint64_t LogSink::dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

void* LogSink::entry(void* context) noexcept {
    static_cast<LogSink*>(context)->run();
    return nullptr;
}

void LogSink::run() noexcept {
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

void LogSink::write_row(const Row& row) noexcept {
    if (writes_ != nullptr && !writes_->logging_enabled()) {
        /* A host pause silences the sink and commits the tail written so
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
}

void LogSink::flush_once() noexcept {
    report_dropped();
    if (fd_ >= 0) {
        (void)::fsync(fd_);
        (void)::close(fd_);
        fd_ = -1;
    }
}

void LogSink::report_dropped() noexcept {
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

} // namespace dart::logging
