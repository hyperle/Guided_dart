#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <new>
#include <span>
#include <string_view>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include "recorder/config.hpp"
#include "recorder/encoding/binary_format.hpp"
#include "recorder/error.hpp"
#include "storage/directories.hpp"

namespace dart::recorder {

// One volume budget for both session admission and periodic image checks.
// Count allocated blocks across the filesystem, including older recordings.
class StorageSpace final {
public:
    static bool query(
        const char* path, std::uint64_t max_storage_bytes, std::uint64_t& available,
        std::uint64_t& block_bytes) noexcept;
};

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

// Per-image filesystem state belongs to the image worker, independently of
// session creation and retention. Encoding never accesses the filesystem.
class ImageFiles final {
public:
    ImageFiles(const Config& config, const Session& session) noexcept
        : config_(config)
        , session_(session) {}
    void reset() noexcept;
    bool png_path(
        std::uint64_t index, char* path, std::size_t path_size, char* name,
        std::size_t name_size) noexcept;
    bool space_available(std::size_t bytes) noexcept;

private:
    static bool mkdir_if_needed(const char* path) noexcept;
    const Config& config_;
    const Session& session_;
    std::uint64_t image_group_ = 0;
    std::uint64_t space_budget_ = 0;
    std::uint32_t space_checks_remaining_ = 0;
};

class EventWriter final {
public:
    explicit EventWriter(const Config& config) noexcept
        : config_(config) {}
    ~EventWriter() { close(); }
    bool prepare() noexcept;
    bool open(const Session& session, RecorderError& error) noexcept;
    bool close() noexcept;
    bool write(const FrameEvent& event) noexcept;
    static std::string_view decision_text(EventDecision decision) noexcept;

private:
    static constexpr std::size_t kEventBufferBytes = 64U * 1024U;
    const Config& config_;
    std::unique_ptr<char[]> event_buffer_{};
    std::FILE* event_file_ = nullptr;
};

inline bool StorageSpace::query(
    const char* path, std::uint64_t max_storage_bytes, std::uint64_t& available,
    std::uint64_t& block_bytes) noexcept {
    struct statvfs storage{};
    if (::statvfs(path, &storage) != 0)
        return false;
    block_bytes = storage.f_frsize != 0 ? storage.f_frsize : storage.f_bsize;
    if (block_bytes == 0
        || storage.f_bavail > std::numeric_limits<std::uint64_t>::max() / block_bytes) {
        errno = EOVERFLOW;
        return false;
    }
    available = static_cast<std::uint64_t>(storage.f_bavail) * block_bytes;
    if (max_storage_bytes != 0) {
        if (storage.f_blocks == 0 || storage.f_bfree > storage.f_blocks
            || storage.f_blocks > std::numeric_limits<std::uint64_t>::max() / block_bytes) {
            errno = EOVERFLOW;
            return false;
        }
        const auto used =
            static_cast<std::uint64_t>(storage.f_blocks - storage.f_bfree) * block_bytes;
        const auto remaining = used < max_storage_bytes ? max_storage_bytes - used : 0;
        available = std::min(available, remaining);
    }
    // Leave room for metadata, buffered writes and continuing application logs
    // within both the physical volume and the configured total ceiling.
    if (available <= Config::kStorageReserveBytes) {
        errno = ENOSPC;
        return false;
    }
    available -= Config::kStorageReserveBytes;
    return true;
}

inline void Session::set_file_error(Error error, const char* path) noexcept {
    error_.set_file_error(error, path);
}

inline bool Session::mkdir_if_needed(const char* path) noexcept {
    if (::mkdir(path, 0777) == 0)
        return true;
    if (errno != EEXIST)
        return false;
    struct stat st{};
    return ::stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

inline std::uint32_t Session::session_number(const char* name) noexcept {
    if (std::strlen(name) != 8 || std::memcmp(name, "rec_", 4) != 0)
        return 0;
    std::uint32_t number = 0;
    for (int i = 4; i < 8; ++i) {
        if (name[i] < '0' || name[i] > '9')
            return 0;
        number = number * 10U + static_cast<std::uint32_t>(name[i] - '0');
    }
    return number;
}

inline bool Session::recorder_file_name(const char* name, bool image) noexcept {
    if (!image)
        return std::strcmp(name, "frames.csv") == 0;
    if (std::strcmp(name, "recording.pgm") == 0 || std::strcmp(name, "recording.pbm") == 0
        || std::strcmp(name, "recording.dbv") == 0 || std::strcmp(name, "recording.pngs") == 0
        || std::strcmp(name, "recording.h264") == 0)
        return true;
    if (std::strlen(name) == 12 && std::strcmp(name + 8, ".png") == 0) {
        for (unsigned i = 0; i < 8; ++i)
            if (name[i] < '0' || name[i] > '9')
                return false;
        return true;
    }
    if (std::strlen(name) != 18 || std::memcmp(name, "frame_", 6) != 0)
        return false;
    for (int i = 6; i < 14; ++i)
        if (name[i] < '0' || name[i] > '9')
            return false;
    return std::strcmp(name + 14, ".pgm") == 0 || std::strcmp(name + 14, ".pbm") == 0
        || std::strcmp(name + 14, ".png") == 0 || std::strcmp(name + 14, ".tmp") == 0;
}

inline bool Session::part_name(const char* name) noexcept {
    if (std::strlen(name) != 11 || std::memcmp(name, "part_", 5) != 0)
        return false;
    for (unsigned i = 5; i < 11; ++i)
        if (name[i] < '0' || name[i] > '9')
            return false;
    return true;
}

inline bool Session::short_directory(const char* name, char prefix) noexcept {
    if (std::strlen(name) != 8 || name[0] != prefix)
        return false;
    for (unsigned i = 1; i < 8; ++i)
        if (name[i] < '0' || name[i] > '9')
            return false;
    return true;
}

inline bool Session::remove_images(const char* path, bool image, unsigned level, bool remove) noexcept {
    struct stat st{};
    if (::lstat(path, &st) != 0) {
        if (errno == ENOENT)
            return true;
        set_file_error(Error::kDirectory, path);
        return false;
    }
    if (!S_ISDIR(st.st_mode)) {
        errno = EPERM;
        set_file_error(Error::kDirectory, path);
        return false;
    }
    DIR* dir = ::opendir(path);
    if (!dir) {
        set_file_error(Error::kDirectory, path);
        return false;
    }
    bool ok = true;
    for (;;) {
        errno = 0;
        const auto* entry = ::readdir(dir);
        if (!entry) {
            ok = errno == 0;
            break;
        }
        if (!std::strcmp(entry->d_name, ".") || !std::strcmp(entry->d_name, ".."))
            continue;
        char file[kMaxPath]{};
        const int n = std::snprintf(file, sizeof(file), "%s/%s", path, entry->d_name);
        if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(file)) {
            errno = ENAMETOOLONG;
            ok = false;
            break;
        }
        if (image && level == 0 && part_name(entry->d_name)) {
            if (!remove_images(file, true, 2, remove)) {
                ok = false;
                break;
            }
            continue;
        }
        if (image
            && ((level == 0 && short_directory(entry->d_name, 'b'))
                || (level == 1 && short_directory(entry->d_name, 'p')))) {
            if (!remove_images(file, true, level + 1U, remove)) {
                ok = false;
                break;
            }
            continue;
        }
        if (!recorder_file_name(entry->d_name, image) || ::lstat(file, &st) != 0
            || !S_ISREG(st.st_mode)) {
            errno = EPERM;
            ok = false;
            break;
        }
        if (remove && ::unlink(file) != 0) {
            ok = false;
            break;
        }
    }
    const int saved = errno;
    ::closedir(dir);
    errno = saved;
    if (!ok || (remove && ::rmdir(path) != 0)) {
        set_file_error(Error::kDirectory, path);
        return false;
    }
    return true;
}

inline bool Session::scan_sessions(
    const char* root, std::uint32_t& count, std::uint32_t& oldest, std::uint32_t& newest) noexcept {
    DIR* dir = ::opendir(root);
    if (dir == nullptr) {
        set_file_error(Error::kDirectory, root);
        return false;
    }
    count = oldest = newest = 0;
    for (;;) {
        errno = 0;
        const dirent* entry = ::readdir(dir);
        if (entry == nullptr) {
            if (errno != 0) {
                set_file_error(Error::kDirectory, root);
                ::closedir(dir);
                return false;
            }
            break;
        }
        const std::uint32_t number = session_number(entry->d_name);
        if (number == 0)
            continue;
        char path[kMaxPath]{};
        const int n = std::snprintf(path, sizeof(path), "%s/%s", root, entry->d_name);
        if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(path)) {
            set_error(Error::kPathTooLong);
            ::closedir(dir);
            return false;
        }
        struct stat st{};
        if (::lstat(path, &st) != 0) {
            set_file_error(Error::kDirectory, path);
            ::closedir(dir);
            return false;
        }
        if (!S_ISDIR(st.st_mode))
            continue;
        ++count;
        if (oldest == 0 || number < oldest)
            oldest = number;
        if (number > newest)
            newest = number;
    }
    ::closedir(dir);
    return true;
}

inline bool Session::remove_session(const char* root, std::uint32_t index) noexcept {
    char path[kMaxPath]{};
    const int n = std::snprintf(path, sizeof(path), "%s/rec_%04u", root, index);
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(path)) {
        set_error(Error::kPathTooLong);
        return false;
    }
    DIR* session = ::opendir(path);
    if (session == nullptr) {
        set_file_error(Error::kDirectory, path);
        return false;
    }
    for (;;) {
        errno = 0;
        const dirent* entry = ::readdir(session);
        if (entry == nullptr) {
            if (errno != 0) {
                set_file_error(Error::kDirectory, path);
                ::closedir(session);
                return false;
            }
            break;
        }
        if (std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0)
            continue;
        if (std::strcmp(entry->d_name, "img") != 0 && std::strcmp(entry->d_name, "logs") != 0) {
            errno = EPERM;
            set_file_error(Error::kDirectory, path);
            ::closedir(session);
            return false;
        }
    }
    ::closedir(session);
    // Validate both subdirectories before deleting anything. Unknown files are never removed.
    for (int pass = 0; pass < 2; ++pass) {
        for (int kind = 0; kind < 2; ++kind) {
            const bool image = kind == 0;
            char subdir[kMaxPath]{};
            const int sub_n =
                std::snprintf(subdir, sizeof(subdir), "%s/%s", path, image ? "img" : "logs");
            if (sub_n <= 0 || static_cast<std::size_t>(sub_n) >= sizeof(subdir)) {
                set_error(Error::kPathTooLong);
                return false;
            }
            if (!remove_images(subdir, image, 0U, pass == 1))
                return false;
        }
    }
    if (::rmdir(path) != 0) {
        set_file_error(Error::kDirectory, path);
        return false;
    }
    return true;
}

inline bool Session::open() noexcept {
    const auto root = config_.output_directory;
    char root_path[kMaxPath]{};
    std::memcpy(root_path, root.data(), root.size());
    root_path[root.size()] = '\0';
    storage::ensure_dir(root_path);
    if (!mkdir_if_needed(root_path)) {
        set_file_error(Error::kDirectory, root_path);
        return false;
    }
    std::uint32_t count = 0, oldest = 0, newest = 0;
    if (!scan_sessions(root_path, count, oldest, newest))
        return false;
    while (config_.keep_sessions > 0 && count >= config_.keep_sessions) {
        if (!remove_session(root_path, oldest) || !scan_sessions(root_path, count, oldest, newest))
            return false;
    }
    std::uint64_t available = 0, block_bytes = 0;
    if (!StorageSpace::query(root_path, config_.max_storage_bytes, available, block_bytes)) {
        set_file_error(Error::kStorage, root_path);
        return false;
    }
    file_limit_bytes_ = std::min(config_.max_file_bytes, available);
    bool created = false;
    for (std::uint32_t offset = 0; offset < 9999; ++offset) {
        const std::uint32_t i = (newest + offset) % 9999U + 1U;
        const int n = std::snprintf(
            session_dir_, sizeof(session_dir_), "%.*s/rec_%04u", static_cast<int>(root.size()),
            root.data(), i);
        if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(session_dir_)) {
            set_error(Error::kPathTooLong);
            return false;
        }
        if (::mkdir(session_dir_, 0777) != 0) {
            if (errno == EEXIST)
                continue;
            set_file_error(Error::kDirectory, session_dir_);
            return false;
        }
        created = true;
        break;
    }
    if (!created) {
        errno = ENOSPC;
        set_file_error(Error::kDirectory, root_path);
        return false;
    }

    char log_dir[kMaxPath]{};
    const int image_n = std::snprintf(image_dir_, sizeof(image_dir_), "%s/img", session_dir_);
    const int log_n = std::snprintf(log_dir, sizeof(log_dir), "%s/logs", session_dir_);
    const int event_n = std::snprintf(event_path_, sizeof(event_path_), "%s/frames.csv", log_dir);
    const int stream_n = std::snprintf(
        image_path_, sizeof(image_path_), "%s/%s", image_dir_,
        config_.image_format == ImageFormat::kH264 ? "recording.h264"
        : config_.image_format == ImageFormat::kPng   ? "."
        : config_.image_format == ImageFormat::kPngStream ? "recording.pngs"
        : config_.image_format == ImageFormat::kDbv ? "recording.dbv"
        : config_.image_format == ImageFormat::kPbm ? "recording.pbm"
                                                    : "recording.pgm");
    if (image_n <= 0 || log_n <= 0 || event_n <= 0 || stream_n <= 0
        || static_cast<std::size_t>(image_n) >= sizeof(image_dir_)
        || static_cast<std::size_t>(log_n) >= sizeof(log_dir)
        || static_cast<std::size_t>(event_n) >= sizeof(event_path_)
        || static_cast<std::size_t>(stream_n) >= sizeof(image_path_)) {
        set_error(Error::kPathTooLong);
        return false;
    }
    if (!mkdir_if_needed(log_dir)) {
        set_file_error(Error::kDirectory, log_dir);
        return false;
    }
    if (!mkdir_if_needed(image_dir_)) {
        set_file_error(Error::kDirectory, image_dir_);
        return false;
    }
    return true;
}

inline void ImageFiles::reset() noexcept {
    image_group_ = space_budget_ = 0;
    space_checks_remaining_ = 0;
}

inline bool ImageFiles::mkdir_if_needed(const char* path) noexcept {
    if (::mkdir(path, 0777) == 0)
        return true;
    if (errno != EEXIST)
        return false;
    struct stat st{};
    return ::stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

inline bool ImageFiles::png_path(
    std::uint64_t index, char* path, std::size_t path_size, char* name,
    std::size_t name_size) noexcept {
    // Bound FAT directory size and lookup cost even for highly compressible frames.
    if (index == 0 || index > 99'999'999) {
        errno = EFBIG;
        return false;
    }
    char part[Session::kMaxPath]{}, bucket_path[Session::kMaxPath]{};
    const auto group_index = (index - 1U) / Config::kImagesPerGroup;
    const auto bucket =
        static_cast<unsigned long long>(group_index / Config::kGroupsPerBucket + 1U);
    const auto group = static_cast<unsigned long long>(group_index % Config::kGroupsPerBucket + 1U);
    const int b = std::snprintf(
        bucket_path, sizeof(bucket_path), "%s/b%07llu", session_.image_directory(), bucket);
    const int p = std::snprintf(part, sizeof(part), "%s/p%07llu", bucket_path, group);
    // Eight digits plus extension is a native FAT 8.3 name. A shared long
    // prefix would force repeated short-name alias searches for every file.
    const int n = std::snprintf(
        name, name_size, "img/b%07llu/p%07llu/%08llu.png", bucket, group,
        static_cast<unsigned long long>(index));
    const int f = std::snprintf(path, path_size, "%s/%s", session_.directory(), name);
    if (b <= 0 || static_cast<std::size_t>(b) >= sizeof(bucket_path) || p <= 0
        || static_cast<std::size_t>(p) >= sizeof(part) || n <= 0
        || static_cast<std::size_t>(n) >= name_size || f <= 0
        || static_cast<std::size_t>(f) >= path_size) {
        errno = ENAMETOOLONG;
        return false;
    }
    if (image_group_ == group_index + 1U)
        return true;
    if (!mkdir_if_needed(bucket_path) || !mkdir_if_needed(part))
        return false;
    image_group_ = group_index + 1U;
    return true;
}

inline bool ImageFiles::space_available(std::size_t bytes) noexcept {
    constexpr std::uint64_t guard = 1024U * 1024U;
    const bool continuous = config_.image_format != ImageFormat::kPng;
    const auto charge = continuous ? static_cast<std::uint64_t>(bytes)
                                   : std::max<std::uint64_t>(bytes + guard, guard * 2U);
    if (space_checks_remaining_ > 0 && space_budget_ >= charge) {
        --space_checks_remaining_;
        space_budget_ -= charge;
        return true;
    }
    std::uint64_t available = 0, block = 0;
    if (!StorageSpace::query(
            session_.image_directory(), config_.max_storage_bytes, available, block))
        return false;
    // Include allocation rounding and room for directory growth. Logical PNG
    // sizes alone substantially underestimate FAT usage for small images.
    // Continuous DBV records share clusters. Cover pending stdio bytes once
    // per query instead of charging every small record as an independent file.
    const auto allocation =
        continuous ? ((bytes + 256U * 1024U + block - 1U) / block) * block
                   : std::max<std::uint64_t>(charge, ((bytes + block - 1U) / block + 2U) * block);
    if (available < allocation) {
        errno = ENOSPC;
        return false;
    }
    space_budget_ = available - allocation;
    space_checks_remaining_ = config_.storage_check_frames - 1U;
    return true;
}

inline bool EventWriter::prepare() noexcept {
    event_buffer_.reset(new (std::nothrow) char[kEventBufferBytes]);
    return event_buffer_ != nullptr;
}

inline bool EventWriter::open(const Session& session, RecorderError& error) noexcept {
    event_file_ = std::fopen(session.event_path(), "w");
    if (event_file_ == nullptr) {
        error.set_file_error(Error::kEventFile, session.event_path());
        return false;
    }
    if (event_buffer_)
        (void)std::setvbuf(event_file_, event_buffer_.get(), _IOFBF, kEventBufferBytes);
    const int header_length = std::fprintf(
        event_file_,
        "# recorder format=2 width=%u height=%u fps=%u frame_timeout_ms=%lld queue_slots=%u max_file_bytes=%llu "
        "file_limit_bytes=%llu session=%s\n"
        "input_index,source_sequence,source_pts,source_pts_timebase_validated,received_us,"
        "decision,image_index,image_file,file_offset,file_bytes\n",
        config_.width, config_.height, config_.fps,
        static_cast<long long>(config_.frame_timeout.count()),
        config_.queue_slots,
        static_cast<unsigned long long>(config_.max_file_bytes),
        static_cast<unsigned long long>(session.file_limit_bytes()), session.directory());
    if (header_length < 0) {
        error.set_file_error(Error::kEventFile, session.event_path());
        return false;
    }
    return true;
}

inline bool EventWriter::close() noexcept {
    if (event_file_ != nullptr) {
        const int result = std::fclose(event_file_);
        event_file_ = nullptr;
        return result == 0;
    }
    return true;
}

inline bool EventWriter::write(const FrameEvent& event) noexcept {
    bool written = false;
    if (event_file_ != nullptr) {
        const auto decision = decision_text(event.decision);
        written = std::fprintf(
            event_file_, "%llu,%llu,%llu,%u,%lld,%.*s,%llu,%s,%llu,%llu\n",
            static_cast<unsigned long long>(event.input_index),
            static_cast<unsigned long long>(event.source_sequence),
            static_cast<unsigned long long>(event.source_pts),
            event.source_pts_timebase_validated ? 1U : 0U,
            static_cast<long long>(event.received_us), static_cast<int>(decision.size()),
            decision.data(), static_cast<unsigned long long>(event.image_index),
            event.image_name.data(), static_cast<unsigned long long>(event.file_offset),
            static_cast<unsigned long long>(event.file_bytes)) >= 0;
    }
    const int system_error = errno;
    if (config_.event_sink != nullptr)
        config_.event_sink(event, config_.event_sink_context);
    if (!written) errno = system_error;
    return written;
}

inline std::string_view EventWriter::decision_text(EventDecision decision) noexcept {
    switch (decision) {
    case EventDecision::kAccepted: return "accepted";
    case EventDecision::kDownsampled: return "downsampled";
    case EventDecision::kBusy: return "busy";
    case EventDecision::kStopped: return "stopped";
    case EventDecision::kInvalidFrame: return "invalid_frame";
    case EventDecision::kTimedOut: return "timed_out";
    case EventDecision::kBufferFull: return "buffer_full";
    case EventDecision::kWriteError: return "write_error";
    }
    return "unknown";
}

class ImageWriter final {
public:
    static constexpr std::size_t kPngStreamHeaderBytes = 16;
    ImageWriter(const Config& config, const Session& session) noexcept
        : config_(config)
        , session_(session) {}
    ~ImageWriter() { (void)close(); }
    bool prepare(std::size_t frame_bytes) noexcept;
    bool open(RecorderError& error) noexcept;
    bool close() noexcept;
    bool write_encoded(const EncodedImage& image, std::uint64_t index = 1) noexcept;
    std::size_t payload_bytes() const noexcept;
    std::size_t record_bytes() const noexcept;
    std::uint64_t file_offset() const noexcept { return last_offset_; }
    std::uint64_t bytes_written() const noexcept { return bytes_written_; }
    const char* image_name() const noexcept { return image_name_; }
    std::uint64_t pack_us() const noexcept { return pack_us_; }
    std::uint64_t io_us() const noexcept { return io_us_; }
    std::uint64_t space_us() const noexcept { return space_us_; }
    std::uint64_t open_us() const noexcept { return open_us_; }
    std::uint64_t write_us() const noexcept { return write_us_; }
    std::uint64_t close_us() const noexcept { return close_us_; }

private:
    bool write_png(std::span<const std::byte> pixels, std::uint64_t index) noexcept;
    bool write_png_stream(std::span<const std::byte> pixels) noexcept;
    static bool write_all(int fd, const void* data, std::size_t size) noexcept;
    bool write_binary(std::span<const std::byte> pixels) noexcept;
    static constexpr std::size_t kFileBufferBytes = 256U * 1024U;
    const Config& config_;
    const Session& session_;
    ImageFiles files_{config_, session_};
    std::size_t frame_bytes_ = 0;
    std::unique_ptr<char[]> file_buffer_{};
    std::size_t fixed_record_bytes_ = 0;
    std::size_t last_record_bytes_ = 0;
    std::uint64_t last_offset_ = 0;
    char image_name_[48]{};
    std::FILE* file_ = nullptr;
    int png_fd_ = -1;
    std::uint64_t bytes_written_ = 0;
    std::uint64_t pack_us_ = 0;
    std::uint64_t io_us_ = 0;
    std::uint64_t space_us_ = 0, open_us_ = 0, write_us_ = 0, close_us_ = 0;
};

inline bool ImageWriter::prepare(std::size_t frame_bytes) noexcept {
    frame_bytes_ = frame_bytes;
    char header[64]{};
    const bool pbm = config_.image_format == ImageFormat::kPbm;
    const int length = pbm
        ? std::snprintf(header, sizeof(header), "P4\n%u %u\n", config_.width, config_.height)
        : std::snprintf(header, sizeof(header), "P5\n%u %u\n255\n", config_.width, config_.height);
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(header)) return false;
    fixed_record_bytes_ = static_cast<std::size_t>(length) + payload_bytes();
    if (config_.image_format != ImageFormat::kPng && config_.image_format != ImageFormat::kPngStream) {
        file_buffer_.reset(new (std::nothrow) char[kFileBufferBytes]);
        return file_buffer_ != nullptr;
    }
    return true;
}

inline bool ImageWriter::open(RecorderError& error) noexcept {
    files_.reset();
    bytes_written_ = last_record_bytes_ = 0;
    last_offset_ = 0;
    image_name_[0] = '\0';
    if (config_.image_format == ImageFormat::kPngStream) {
        if (session_.file_limit_bytes() < kPngStreamHeaderBytes) {
            errno = ENOSPC;
            error.set_file_error(Error::kStorage, session_.directory());
            return false;
        }
        png_fd_ = ::open(session_.image_path(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (png_fd_ < 0) {
            error.set_file_error(Error::kImageFile, session_.image_path());
            return false;
        }
        std::uint8_t header[kPngStreamHeaderBytes]{'D', 'P', 'N', 'G', 1, 0, 16, 0};
        for (unsigned i = 0; i < 4; ++i) {
            header[8 + i] = static_cast<std::uint8_t>(config_.width >> (8U * i));
            header[12 + i] = static_cast<std::uint8_t>(config_.height >> (8U * i));
        }
        if (!write_all(png_fd_, header, sizeof(header))) {
            const int saved = errno;
            (void)::close(png_fd_);
            png_fd_ = -1;
            errno = saved;
            error.set_file_error(Error::kImageFile, session_.image_path());
            return false;
        }
        bytes_written_ = kPngStreamHeaderBytes;
        (void)std::snprintf(image_name_, sizeof(image_name_), "img/recording.pngs");
        return true;
    }
    if (config_.image_format == ImageFormat::kPng)
        return true;
    if (record_bytes() > session_.file_limit_bytes()) {
        errno = ENOSPC;
        error.set_file_error(Error::kStorage, session_.directory());
        return false;
    }
    file_ = std::fopen(session_.image_path(), "wb");
    if (file_ == nullptr) {
        error.set_file_error(Error::kImageFile, session_.image_path());
        return false;
    }
    (void)std::setvbuf(file_, file_buffer_.get(), _IOFBF, kFileBufferBytes);
    bytes_written_ = 0;
    return true;
}

inline bool ImageWriter::close() noexcept {
    if (png_fd_ >= 0) {
        const bool closed = ::close(png_fd_) == 0;
        png_fd_ = -1;
        return closed;
    }
    if (file_ == nullptr)
        return true;
    const int result = std::fclose(file_);
    file_ = nullptr;
    return result == 0;
}

inline std::size_t ImageWriter::payload_bytes() const noexcept {
    return config_.image_format == ImageFormat::kPbm
        ? ((static_cast<std::size_t>(config_.width) + 7U) / 8U) * config_.height : frame_bytes_;
}

inline std::size_t ImageWriter::record_bytes() const noexcept {
    return config_.variable_record_size() ? last_record_bytes_ : fixed_record_bytes_;
}

inline bool ImageWriter::write_all(int fd, const void* data, std::size_t size) noexcept {
    const auto* bytes = static_cast<const std::byte*>(data);
    std::size_t offset = 0;
    while (offset < size) {
        const auto written = ::write(fd, bytes + offset, size - offset);
        if (written < 0 && errno == EINTR)
            continue;
        if (written <= 0) {
            if (written == 0)
                errno = EIO;
            return false;
        }
        offset += static_cast<std::size_t>(written);
    }
    return true;
}

inline bool ImageWriter::write_png_stream(std::span<const std::byte> encoded) noexcept {
    if (png_fd_ < 0)
        return false;
    const auto encoded_at = std::chrono::steady_clock::now();
    if (encoded.empty()) {
        errno = EIO;
        return false;
    }
    if (encoded.size() > session_.file_limit_bytes() - bytes_written_) {
        errno = EFBIG;
        return false;
    }
    if (!files_.space_available(encoded.size()))
        return false;
    const auto space_at = std::chrono::steady_clock::now();
    space_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(space_at - encoded_at).count());
    if (!write_all(png_fd_, encoded.data(), encoded.size()))
        return false;
    const auto write_end = std::chrono::steady_clock::now();
    write_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(write_end - space_at).count());
    last_offset_ = bytes_written_;
    last_record_bytes_ = encoded.size();
    bytes_written_ += encoded.size();
    io_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                         std::chrono::steady_clock::now() - encoded_at).count());
    open_us_ = close_us_ = 0;
    return true;
}

inline bool ImageWriter::write_png(std::span<const std::byte> encoded, std::uint64_t index) noexcept {
    const auto encoded_at = std::chrono::steady_clock::now();
    const auto io_begin = encoded_at;
    if (encoded.empty()) {
        errno = EIO;
        return false;
    }
    if (encoded.size() > session_.file_limit_bytes() - bytes_written_) {
        errno = EFBIG;
        return false;
    }
    if (!files_.space_available(encoded.size()))
        return false;
    const auto space_end = std::chrono::steady_clock::now();
    space_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(space_end - io_begin).count());
    char path[Session::kMaxPath]{};
    if (!files_.png_path(index, path, sizeof(path), image_name_, sizeof(image_name_)))
        return false;
    const int file = ::open(path, O_WRONLY | O_CREAT | O_EXCL, 0666);
    if (file < 0)
        return false;
    const auto open_end = std::chrono::steady_clock::now();
    open_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(open_end - space_end).count());
    // The complete PNG is already in memory. Send it in one write rather than
    // splitting it through libc's small stdio buffer. MTP cannot read a file
    // while this firmware holds it open for writing.
    std::size_t offset = 0;
    while (offset < encoded.size()) {
        const auto written = ::write(file, encoded.data() + offset, encoded.size() - offset);
        if (written < 0 && errno == EINTR)
            continue;
        if (written <= 0) {
            if (written == 0)
                errno = EIO;
            break;
        }
        offset += static_cast<std::size_t>(written);
    }
    bool ok = offset == encoded.size();
    const auto write_end = std::chrono::steady_clock::now();
    write_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(write_end - open_end).count());
    const int saved = errno;
    const int closed = ::close(file);
    if (!ok || closed != 0) {
        const int failure = ok ? errno : saved;
        (void)::unlink(path);
        errno = failure;
        return false;
    }
    close_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                               std::chrono::steady_clock::now() - write_end)
                                               .count());
    last_record_bytes_ = encoded.size();
    bytes_written_ += encoded.size();
    io_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                            std::chrono::steady_clock::now() - io_begin)
                                            .count());
    return true;
}

inline bool ImageWriter::write_binary(std::span<const std::byte> encoded) noexcept {
    if (!file_)
        return false;
    const auto encoded_at = std::chrono::steady_clock::now();
    if (encoded.empty()) {
        errno = EIO;
        return false;
    }
    const auto header = bytes_written_ == 0 ? BinaryFormat::kHeaderBytes : 0;
    const auto size = header + encoded.size();
    if (size > session_.file_limit_bytes() - bytes_written_) {
        errno = EFBIG;
        return false;
    }
    if (!files_.space_available(size))
        return false;
    const auto space_at = std::chrono::steady_clock::now();
    space_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(space_at - encoded_at).count());
    std::byte geometry[BinaryFormat::kHeaderBytes];
    BinaryFormat::header(geometry, config_.width, config_.height);
    if ((header && std::fwrite(geometry, 1, header, file_) != header)
        || std::fwrite(encoded.data(), 1, encoded.size(), file_) != encoded.size())
        return false;
    last_offset_ = bytes_written_;
    last_record_bytes_ = size;
    bytes_written_ += size;
    (void)std::snprintf(image_name_, sizeof(image_name_), "img/recording.dbv");
    io_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                            std::chrono::steady_clock::now() - encoded_at)
                                            .count());
    write_us_ = io_us_ - space_us_;
    open_us_ = close_us_ = 0;
    return true;
}

inline bool ImageWriter::write_encoded(const EncodedImage& image, std::uint64_t index) noexcept {
    io_us_ = space_us_ = open_us_ = write_us_ = close_us_ = 0;
    if (!image.valid()) { errno = EIO; return false; }
    if (config_.image_format == ImageFormat::kDbv) return write_binary(image.pixels);
    if (config_.image_format == ImageFormat::kPngStream) return write_png_stream(image.pixels);
    if (config_.image_format == ImageFormat::kPng) return write_png(image.pixels, index);
    if (file_ == nullptr) { errno = EBADF; return false; }
    if (image.size() > session_.file_limit_bytes() - bytes_written_) { errno = EFBIG; return false; }
    const auto begin = std::chrono::steady_clock::now();
    if (config_.max_storage_bytes != 0 && !files_.space_available(image.size())) return false;
    const auto checked = std::chrono::steady_clock::now();
    space_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(checked - begin).count());
    if (std::fwrite(image.header.data(), 1, image.header.size(), file_) != image.header.size()) return false;
    if (image.borrowed_input) {
        for (std::uint32_t y = 0; y < image.height; ++y) {
            const auto* row = image.pixels.data() + static_cast<std::size_t>(y) * image.stride;
            if (std::fwrite(row, 1, image.width, file_) != image.width) return false;
        }
    } else if (std::fwrite(image.pixels.data(), 1, image.pixels.size(), file_) != image.pixels.size()) return false;
    const auto written = std::chrono::steady_clock::now();
    last_offset_ = bytes_written_;
    last_record_bytes_ = image.size();
    bytes_written_ += image.size();
    (void)std::snprintf(image_name_, sizeof(image_name_), "img/recording.%s",
        config_.image_format == ImageFormat::kH264 ? "h264"
            : config_.image_format == ImageFormat::kPbm ? "pbm" : "pgm");
    write_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(written - checked).count());
    io_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count());
    return true;
}

} // namespace dart::recorder
