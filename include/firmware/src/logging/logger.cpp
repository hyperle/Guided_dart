module;
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string_view>
#include <unistd.h>

export module dart.logging.logger;

import dart.storage.directories;
import dart.system.monotonic_clock;
import dart.system.write_control;

export namespace dart::logging {

/* File logging only. A failed log write must not become a console error loop. */
class Logger {
public:
    explicit Logger(platform::WriteControl* writes = nullptr) noexcept : writes_(writes) {}
    ~Logger() { end_batch(); }
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

    void begin_batch() noexcept { batching_ = true; }

    void end_batch() noexcept {
        batching_ = false;
        if (fd_ >= 0) {
            ::fsync(fd_);
            ::close(fd_);
            fd_ = -1;
        }
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
        if (fd_ >= 0) {
            (void)!::write(fd_, out, static_cast<std::size_t>(out_n));
            if (!batching_)
                end_batch();
        }
    }

private:
    platform::WriteControl* writes_ = nullptr;
    char path_[512]{};
    int fd_ = -1;
    bool batching_ = false;
};

} // namespace dart::logging
