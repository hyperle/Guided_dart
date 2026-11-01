#pragma once

#include <cstddef>
#include <cstdint>

#include "recorder/config.hpp"
#include "recorder/error.hpp"

namespace dart::recorder {
class Session final {
public:
    static constexpr std::size_t kMaxPath = RecorderError::kMaxPath;
    Session(const Config& config, RecorderError& error) noexcept
        : config_(config)
        , error_(error) {}
    bool open() noexcept;
    const char* directory() const noexcept { return session_dir_; }
    const char* event_path() const noexcept { return event_path_; }
    const char* image_directory() const noexcept { return image_dir_; }
    const char* image_path() const noexcept { return image_path_; }
    std::uint64_t file_limit_bytes() const noexcept { return file_limit_bytes_; }

private:
    bool scan_sessions(const char*, std::uint32_t&, std::uint32_t&, std::uint32_t&) noexcept;
    bool remove_session(const char*, std::uint32_t) noexcept;
    bool remove_images(const char* path, bool image, unsigned level, bool remove) noexcept;
    void set_error(Error error) noexcept { error_.set_error(error); }
    void set_file_error(Error error, const char* path) noexcept;
    const Config& config_;
    RecorderError& error_;
    char session_dir_[kMaxPath]{};
    char image_dir_[kMaxPath]{};
    char image_path_[kMaxPath]{};
    char event_path_[kMaxPath]{};
    std::uint64_t file_limit_bytes_ = 0;

private:
    static bool mkdir_if_needed(const char* path) noexcept;

    static std::uint32_t session_number(const char* name) noexcept;

    static bool recorder_file_name(const char* name, bool image) noexcept;
    static bool part_name(const char* name) noexcept;
    static bool short_directory(const char* name, char prefix) noexcept;
};

} // namespace dart::recorder
