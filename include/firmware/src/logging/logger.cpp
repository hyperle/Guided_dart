#include "logging/logger.hpp"

#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string_view>
#include <unistd.h>

#include "storage/directories.hpp"
#include "system/monotonic_clock.hpp"
#include "system/write_control.hpp"

namespace dart::logging {

Logger::~Logger() { sink_.stop(); }

bool Logger::open(std::string_view path) noexcept {
    if (path.empty() || path.size() >= sizeof(path_))
        return false;
    std::memcpy(path_, path.data(), path.size());
    path_[path.size()] = '\0';
    if (writes_ != nullptr && !writes_->logging_enabled())
        return sink_.start(path, writes_);
    storage::ensure_parent(path_);
    const int fd = ::open(path_, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0)
        return false;
    ::close(fd);
    return sink_.start(path, writes_);
}

void Logger::end_batch() noexcept {
    batching_.store(false, std::memory_order_relaxed);
    sink_.request_flush();
}

bool Logger::flush(std::chrono::milliseconds deadline) noexcept { return sink_.drain(deadline); }

void Logger::line(const char* fmt, ...) noexcept {
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
    sink_.enqueue(out, static_cast<std::size_t>(out_n));
    if (!batching_.load(std::memory_order_relaxed))
        end_batch();
}

} // namespace dart::logging
