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
    if (!buffer_)
        return false;
    if (!rows_)
        rows_.reset(new (std::nothrow) Row[kRowCount]);
    if (!rows_)
        return false;
    head_ = tail_ = count_ = 0;
    dropped_ = io_errors_ = 0;
    running_.store(true);
    if (::pthread_create(&thread_, nullptr, &CsvWriter::entry, this) != 0) {
        running_.store(false);
        return false;
    }
    started_ = true;
    return true;
}

void CsvWriter::row(const char* format, ...) noexcept {
    if (!running_.load() || (writes_ != nullptr && !writes_->logging_enabled()))
        return;
    va_list args;
    va_start(args, format);
    Row row{};
    const int size = std::vsnprintf(row.text, sizeof(row.text), format, args);
    va_end(args);
    std::unique_lock lock(mutex_, std::try_to_lock);
    if (size < 0 || static_cast<std::size_t>(size) >= sizeof(row.text) || !lock.owns_lock()
        || !running_.load() || count_ == kRowCount) {
        ++dropped_;
        return;
    }
    rows_[tail_] = row;
    tail_ = (tail_ + 1) % kRowCount;
    ++count_;
    lock.unlock();
    ready_.notify_one();
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
    if (file_ == nullptr) {
        ++io_errors_;
        return false;
    }
    if (buffer_)
        (void)std::setvbuf(file_, buffer_.get(), _IOFBF, kBufferBytes);
    if (std::fseek(file_, 0, SEEK_END) != 0) {
        ++io_errors_;
        close_file();
        return false;
    }
    if (std::ftell(file_) == 0 && std::fputs(header_, file_) < 0) {
        ++io_errors_;
        close_file();
        return false;
    }
    return true;
}

void CsvWriter::close_file() noexcept {
    if (file_ != nullptr) {
        if (std::fclose(file_) != 0) ++io_errors_;
        file_ = nullptr;
    }
}

void* CsvWriter::entry(void* context) noexcept {
    static_cast<CsvWriter*>(context)->run();
    return nullptr;
}

void CsvWriter::run() noexcept {
    bool attempted = false;
    bool first_open = true;
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
        const bool wanted = writes_ == nullptr || writes_->logging_enabled();
        if (wanted && (running_.load() || have_row) && !attempted) {
            attempted = true;
            (void)open_file(first_open && writes_ == nullptr ? "w" : "a");
            first_open = false;
        }
        if (have_row) {
            if (wanted && file_ != nullptr) {
                if (std::fputs(row.text, file_) < 0) {
                    ++io_errors_;
                    ++dropped_;
                    close_file();
                }
            } else {
                ++dropped_;
            }
        }
        if (!wanted || (empty && !running_.load())) {
            close_file();
            attempted = false;
            if (empty && !running_.load())
                break;
        }
    }
}

} // namespace dart::logging
