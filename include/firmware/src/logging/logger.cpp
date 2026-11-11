#include "logging/logger.hpp"

#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <string_view>

#include "system/monotonic_clock.hpp"
#include "system/write_control.hpp"

namespace dart::logging {

const char* tag_text(Tag tag) noexcept {
    switch (tag) {
    case Tag::kApp: return "app";
    case Tag::kCamera: return "camera";
    case Tag::kVision: return "vision";
    case Tag::kRecord: return "record";
    case Tag::kServo: return "servo";
    case Tag::kImu: return "imu";
    case Tag::kLog: return "log";
    case Tag::kWifi: return "wifi";
    }
    return "app";
}

Logger::~Logger() { sink_.stop(); }

bool Logger::open(std::string_view path) noexcept {
    // An empty path permits console-only output. File errors are reported by
    // sink statistics; startup never opens a file on the calling thread.
    return sink_.start(path, writes_);
}

void Logger::end_batch() noexcept {
    batching_.store(false, std::memory_order_relaxed);
    sink_.request_close();
}

bool Logger::flush(std::chrono::milliseconds deadline) noexcept { return sink_.drain(deadline); }

void Logger::line(Tag tag, const char* fmt, ...) noexcept {
    va_list args;
    va_start(args, fmt);
    vline(tag, fmt, args);
    va_end(args);
}

void Logger::line(const char* fmt, ...) noexcept {
    va_list args;
    va_start(args, fmt);
    vline(Tag::kApp, fmt, args);
    va_end(args);
}

void Logger::vline(Tag tag, const char* fmt, va_list args) noexcept {
    if (writes_ != nullptr && !writes_->logging_enabled()) {
        end_batch();
        return;
    }
    char body[512];
    char out[640];
    const int body_n = std::vsnprintf(body, sizeof(body), fmt, args);
    if (body_n < 0)
        return;
    int out_n = std::snprintf(
        out, sizeof(out), "[%s] ts_ms=%lld %s", tag_text(tag),
        static_cast<long long>(platform::mono_ms()), body);
    if (out_n < 0)
        return;
    if (static_cast<std::size_t>(out_n) >= sizeof(out))
        out_n = static_cast<int>(sizeof(out)) - 1;
    if (out_n > 0 && out[out_n - 1] != '\n') {
        if (static_cast<std::size_t>(out_n) == sizeof(out) - 1) --out_n;
        out[out_n++] = '\n';
    }
    sink_.enqueue(out, static_cast<std::size_t>(out_n));
    if (!batching_.load(std::memory_order_relaxed))
        end_batch();
}

} // namespace dart::logging
