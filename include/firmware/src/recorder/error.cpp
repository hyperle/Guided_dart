module;
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>

export module dart.recorder.error;



export namespace dart::recorder {

enum class Error : std::uint8_t {
    kNone,
    kAlreadyStarted,
    kInvalidConfig,
    kNoMemory,
    kPathTooLong,
    kDirectory,
    kEventFile,
    kImageFile,
    kStorage,
    kThread,
};

class RecorderError final {
public:
    static constexpr std::size_t kMaxPath = 512;
    Error last_error() const noexcept { return error_.load(std::memory_order_acquire); }
    int last_system_error() const noexcept { return system_error_; }
    const char* last_error_path() const noexcept { return path_; }
    void reset() noexcept {
        set_error(Error::kNone);
        system_error_ = 0;
        path_[0] = '\0';
    }
    void set_error(Error error) noexcept { error_.store(error, std::memory_order_release); }
    void set_file_error(Error error, const char* path) noexcept {
        system_error_ = errno;
        std::snprintf(path_, sizeof(path_), "%s", path);
        set_error(error);
    }

private:
    std::atomic<Error> error_{Error::kNone};
    int system_error_ = 0;
    char path_[kMaxPath]{};
};

} // namespace dart::recorder
