#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <pthread.h>

#include "storage/directories.hpp"
#include "system/write_control.hpp"

namespace dart::logging {
// Own buffering and file durability; the caller owns column and row semantics.
class CsvWriter final {
public:
    CsvWriter() = default;
    ~CsvWriter() { close(); }
    CsvWriter(const CsvWriter&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;

    bool open(
        const char* path, const char* header, platform::WriteControl* writes = nullptr) noexcept;
    void row(const char* format, ...) noexcept;
    void flush() noexcept;
    void close() noexcept;
    std::uint64_t dropped() const noexcept { return dropped_.load(); }

private:
    bool open_file(const char* mode) noexcept;
    void flush_file() noexcept;
    void close_file() noexcept;
    static void* entry(void* context) noexcept;
    void run() noexcept;
    struct Row {
        char text[1024]{};
    };
    static constexpr std::size_t kRowCount = 256;
    platform::WriteControl* writes_ = nullptr;
    char path_[512]{}, header_[512]{};
    std::unique_ptr<Row[]> rows_{};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> dropped_{0};
    std::mutex mutex_;
    std::condition_variable ready_;
    pthread_t thread_{};
    bool started_ = false;
    std::size_t head_ = 0, tail_ = 0, count_ = 0;
    static constexpr std::size_t kBufferBytes = 64U * 1024U;
    std::unique_ptr<char[]> buffer_{};
    std::FILE* file_ = nullptr;
};

} // namespace dart::logging
