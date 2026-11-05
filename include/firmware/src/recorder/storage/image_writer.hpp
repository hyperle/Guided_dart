#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <memory>
#include <span>

#include "camera/frame_view.hpp"
#include "recorder/config.hpp"
#include "recorder/encoding/binary_encoder.hpp"
#include "recorder/encoding/binary_format.hpp"
#include "recorder/encoding/png_encoder.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/image_files.hpp"
#include "recorder/storage/session.hpp"

namespace dart::recorder {
class ImageWriter final {
public:
    ImageWriter(const Config& config, const Session& session) noexcept
        : config_(config)
        , session_(session) {}
    ~ImageWriter() { (void)close(); }
    bool prepare(std::size_t frame_bytes) noexcept;
    bool open(RecorderError& error) noexcept;
    bool close() noexcept;
    bool valid_view(const camera::FrameView& view) const noexcept;
    bool copy_image(std::span<std::byte> pixels, const camera::FrameView& view) noexcept;
    bool write_image(std::span<const std::byte> pixels, std::uint64_t index = 1) noexcept;
    std::size_t payload_bytes() const noexcept;
    std::size_t record_bytes() const noexcept;
    std::uint64_t file_offset() const noexcept { return last_offset_; }
    const char* image_name() const noexcept { return image_name_; }
    std::uint64_t pack_us() const noexcept { return pack_us_; }
    std::uint64_t io_us() const noexcept { return io_us_; }
    std::uint64_t space_us() const noexcept { return space_us_; }
    std::uint64_t open_us() const noexcept { return open_us_; }
    std::uint64_t write_us() const noexcept { return write_us_; }
    std::uint64_t close_us() const noexcept { return close_us_; }

private:
    bool write_png(std::span<const std::byte> pixels, std::uint64_t index) noexcept;
    bool write_binary(std::span<const std::byte> pixels) noexcept;
    bool sync() noexcept;
    static constexpr std::size_t kFileBufferBytes = 256U * 1024U;
    const Config& config_;
    const Session& session_;
    ImageFiles files_{config_, session_};
    std::size_t frame_bytes_ = 0;
    std::size_t pbm_row_bytes_ = 0;
    char header_[64]{};
    std::size_t header_bytes_ = 0;
    std::unique_ptr<std::byte[]> pbm_frame_{};
    std::unique_ptr<char[]> file_buffer_{};
    PngEncoder png_{};
    BinaryEncoder binary_{};
    std::size_t last_record_bytes_ = 0;
    std::uint64_t last_offset_ = 0;
    char image_name_[48]{};
    std::FILE* file_ = nullptr;
    std::uint64_t bytes_written_ = 0;
    std::uint64_t pack_us_ = 0;
    std::uint64_t io_us_ = 0;
    std::uint64_t space_us_ = 0, open_us_ = 0, write_us_ = 0, close_us_ = 0;
    std::chrono::steady_clock::time_point last_flush_{};
};

} // namespace dart::recorder
