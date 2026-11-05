#include "logging/csv_writer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <pthread.h>
#include <unistd.h>

#include "storage/directories.hpp"
#include "system/write_control.hpp"

namespace dart::logging {

bool CsvWriter::open(
    const char* path, const char* header, platform::WriteControl* writes) noexcept {
    close();
    if (path == nullptr || header == nullptr || std::strlen(path) >= sizeof(path_)
        || std::strlen(header) >= sizeof(header_))
        return false;
    std::strcpy(path_, path);
    std::strcpy(header_, header);
    writes_ = writes;
    if (!buffer_)
        buffer_.reset(new (std::nothrow) char[kBufferBytes]);
    if (writes_ == nullptr)
        return open_file("w");
    if (!rows_)
        rows_.reset(new (std::nothrow) Row[kRowCount]);
    if (!rows_)
        return false;
    head_ = tail_ = count_ = 0;
    running_.store(true);
    if (::pthread_create(&thread_, nullptr, &CsvWriter::entry, this) != 0) {
        running_.store(false);
        return false;
    }
    started_ = true;
    return true;
}

void CsvWriter::row(const char* format, ...) noexcept {
    if (writes_ != nullptr && (!running_.load() || !writes_->logging_enabled()))
        return;
    va_list args;
    va_start(args, format);
    if (writes_ == nullptr) {
        if (file_ != nullptr)
            (void)std::vfprintf(file_, format, args);
    } else {
        Row row{};
        const int size = std::vsnprintf(row.text, sizeof(row.text), format, args);
        std::unique_lock lock(mutex_, std::try_to_lock);
        if (size < 0 || static_cast<std::size_t>(size) >= sizeof(row.text) || !lock.owns_lock()
            || count_ == kRowCount) {
            ++dropped_;
        } else {
            rows_[tail_] = row;
            tail_ = (tail_ + 1) % kRowCount;
            ++count_;
            ready_.notify_one();
        }
    }
    va_end(args);
}

void CsvWriter::flush() noexcept {
    if (writes_ == nullptr)
        flush_file();
}

void CsvWriter::close() noexcept {
    {
        std::lock_guard lock(mutex_);
        running_.store(false);
    }
    ready_.notify_all();
    if (started_) {
        (void)::pthread_join(thread_, nullptr);
        started_ = false;
    }
    close_file();
}

bool CsvWriter::open_file(const char* mode) noexcept {
    storage::ensure_parent(path_);
    file_ = std::fopen(path_, mode);
    if (file_ == nullptr)
        return false;
    if (buffer_)
        (void)std::setvbuf(file_, buffer_.get(), _IOFBF, kBufferBytes);
    if (std::fseek(file_, 0, SEEK_END) != 0) {
        close_file();
        return false;
    }
    if (std::ftell(file_) == 0)
        (void)std::fputs(header_, file_);
    flush_file();
    return true;
}

void CsvWriter::flush_file() noexcept {
    if (file_ != nullptr) {
        (void)std::fflush(file_);
        (void)::fsync(::fileno(file_));
    }
}

void CsvWriter::close_file() noexcept {
    if (file_ != nullptr) {
        flush_file();
        (void)std::fclose(file_);
        file_ = nullptr;
    }
}

void* CsvWriter::entry(void* context) noexcept {
    static_cast<CsvWriter*>(context)->run();
    return nullptr;
}

void CsvWriter::run() noexcept {
    bool attempted = false;
    auto next_flush = std::chrono::steady_clock::now() + std::chrono::seconds{1};
    for (;;) {
        Row row{};
        bool have_row = false, empty = false;
        {
            std::unique_lock lock(mutex_);
            ready_.wait_for(lock, std::chrono::milliseconds{20}, [this] {
                return count_ != 0 || !running_.load();
            });
            if (count_ != 0) {
                row = rows_[head_];
                head_ = (head_ + 1) % kRowCount;
                --count_;
                have_row = true;
            }
            empty = count_ == 0;
        }
        const bool wanted = writes_->logging_enabled();
        if (wanted && running_.load() && !attempted) {
            attempted = true;
            (void)open_file("a");
        }
        if (have_row) {
            if (file_ != nullptr)
                (void)std::fputs(row.text, file_);
            else
                ++dropped_;
        }
        if (empty && (!wanted || !running_.load())) {
            close_file();
            attempted = false;
            if (!running_.load())
                break;
        }
        if (std::chrono::steady_clock::now() >= next_flush) {
            flush_file();
            next_flush = std::chrono::steady_clock::now() + std::chrono::seconds{1};
        }
    }
}

} // namespace dart::logging
