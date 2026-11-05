#include "recorder/storage/image_writer.hpp"

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <new>
#include <span>
#include <unistd.h>

#include "camera/frame_view.hpp"
#include "recorder/config.hpp"
#include "recorder/encoding/binary_encoder.hpp"
#include "recorder/encoding/binary_format.hpp"
#include "recorder/encoding/png_encoder.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/image_files.hpp"
#include "recorder/storage/session.hpp"

namespace dart::recorder {

bool ImageWriter::prepare(std::size_t frame_bytes) noexcept {
    frame_bytes_ = frame_bytes;
    if (config_.image_format == ImageFormat::kPng)
        return png_.prepare(config_.width, config_.height, config_.pbm_threshold);
    if (config_.image_format == ImageFormat::kDbv) {
        file_buffer_.reset(new (std::nothrow) char[kFileBufferBytes]);
        return file_buffer_
            && binary_.prepare(config_.width, config_.height, config_.pbm_threshold);
    }
    const bool pbm = config_.image_format == ImageFormat::kPbm;
    const int n =
        pbm ? std::snprintf(header_, sizeof(header_), "P4\n%u %u\n", config_.width, config_.height)
            : std::snprintf(
                  header_, sizeof(header_), "P5\n%u %u\n255\n", config_.width, config_.height);
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(header_))
        return false;
    header_bytes_ = static_cast<std::size_t>(n);
    file_buffer_.reset(new (std::nothrow) char[kFileBufferBytes]);
    if (!file_buffer_)
        return false;
    if (config_.image_format == ImageFormat::kPbm) {
        pbm_row_bytes_ = (static_cast<std::size_t>(config_.width) + 7U) / 8U;
        pbm_frame_.reset(new (std::nothrow) std::byte[payload_bytes()]);
        return pbm_frame_ != nullptr;
    }
    return true;
}

bool ImageWriter::open(RecorderError& error) noexcept {
    files_.reset();
    bytes_written_ = last_record_bytes_ = 0;
    last_offset_ = 0;
    binary_.reset();
    image_name_[0] = '\0';
    if (config_.image_format == ImageFormat::kPng) {
        last_flush_ = std::chrono::steady_clock::now();
        return true;
    }
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
    last_flush_ = std::chrono::steady_clock::now();
    return true;
}

bool ImageWriter::close() noexcept {
    if (file_ == nullptr)
        return true;
    const bool synced = sync();
    const int sync_error = errno;
    const int result = std::fclose(file_);
    file_ = nullptr;
    if (!synced)
        errno = sync_error;
    return synced && result == 0;
}

std::size_t ImageWriter::payload_bytes() const noexcept {
    return config_.image_format == ImageFormat::kPbm
             ? pbm_row_bytes_ * static_cast<std::size_t>(config_.height)
             : frame_bytes_;
}

std::size_t ImageWriter::record_bytes() const noexcept {
    return config_.variable_record_size() ? last_record_bytes_ : header_bytes_ + payload_bytes();
}

bool ImageWriter::sync() noexcept {
    // fflush only drains libc. FAT updates the directory size and cluster
    // chain on fsync; without it a reset can leave a zero-byte recording.
    return std::fflush(file_) == 0 && ::fsync(::fileno(file_)) == 0;
}

bool ImageWriter::valid_view(const camera::FrameView& view) const noexcept {
    if (view.width != config_.width || view.height != config_.height || view.stride < view.width
        || view.luma.data() == nullptr)
        return false;
    const std::uint64_t required =
        static_cast<std::uint64_t>(view.height - 1U) * view.stride + view.width;
    return required <= view.luma.size();
}

bool ImageWriter::copy_image(std::span<std::byte> pixels, const camera::FrameView& view) noexcept {
    if (!valid_view(view) || pixels.size() < frame_bytes_)
        return false;
    auto* destination = pixels.data();
    for (std::uint32_t y = 0; y < view.height; ++y) {
        const auto* source = view.luma.data() + static_cast<std::size_t>(y) * view.stride;
        std::memcpy(destination + static_cast<std::size_t>(y) * view.width, source, view.width);
    }
    return true;
}

bool ImageWriter::write_png(std::span<const std::byte> pixels, std::uint64_t index) noexcept {
    const auto begin = std::chrono::steady_clock::now();
    const auto encoded = png_.encode(pixels);
    const auto io_begin = std::chrono::steady_clock::now();
    pack_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(io_begin - begin).count());
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
    const bool flush = bytes_written_ == 0 || io_begin - last_flush_ >= config_.flush_period;
    if (ok && flush)
        ok = ::fsync(file) == 0;
    const int saved = errno;
    const int closed = ::close(file);
    if (!ok || closed != 0) {
        const int failure = ok ? errno : saved;
        (void)::unlink(path);
        errno = failure;
        return false;
    }
    if (flush)
        last_flush_ = io_begin;
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

bool ImageWriter::write_binary(std::span<const std::byte> pixels) noexcept {
    if (!file_)
        return false;
    const auto begin = std::chrono::steady_clock::now();
    const auto encoded = binary_.encode(pixels);
    const auto encoded_at = std::chrono::steady_clock::now();
    pack_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(encoded_at - begin).count());
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
    if (bytes_written_ == 0 || space_at - last_flush_ >= config_.flush_period) {
        if (!sync())
            return false;
        last_flush_ = space_at;
    }
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

bool ImageWriter::write_image(std::span<const std::byte> pixels, std::uint64_t index) noexcept {
    if (config_.image_format == ImageFormat::kDbv)
        return write_binary(pixels);
    if (config_.image_format == ImageFormat::kPng)
        return write_png(pixels, index);
    if (file_ == nullptr || pixels.size() < frame_bytes_)
        return false;
    // Check the complete record before either fwrite, including buffered bytes.
    if (record_bytes() > session_.file_limit_bytes() - bytes_written_) {
        errno = EFBIG;
        return false;
    }
    if (config_.max_storage_bytes != 0 && !files_.space_available(record_bytes()))
        return false;
    const std::byte* payload = pixels.data();
    const auto pack_begin = std::chrono::steady_clock::now();
    if (config_.image_format == ImageFormat::kPbm) {
        const auto* gray = reinterpret_cast<const std::uint8_t*>(pixels.data());
        auto* packed = reinterpret_cast<std::uint8_t*>(pbm_frame_.get());
        for (std::uint32_t y = 0; y < config_.height; ++y) {
            const auto* row = gray + static_cast<std::size_t>(y) * config_.width;
            auto* out = packed + static_cast<std::size_t>(y) * pbm_row_bytes_;
            const auto threshold = config_.pbm_threshold;
            std::uint32_t x = 0;
            // Build each output byte in registers. Avoid a branch and repeated
            // read/modify/write of the packed buffer for every input pixel.
            for (; x + 8U <= config_.width; x += 8U) {
                *out++ = static_cast<std::uint8_t>(
                    ((row[x] < threshold) << 7U) | ((row[x + 1] < threshold) << 6U)
                    | ((row[x + 2] < threshold) << 5U) | ((row[x + 3] < threshold) << 4U)
                    | ((row[x + 4] < threshold) << 3U) | ((row[x + 5] < threshold) << 2U)
                    | ((row[x + 6] < threshold) << 1U) | (row[x + 7] < threshold));
            }
            if (x < config_.width) {
                std::uint8_t tail = 0;
                for (unsigned bit = 0; x < config_.width; ++x, ++bit)
                    tail |= static_cast<std::uint8_t>((row[x] < threshold) << (7U - bit));
                *out = tail;
            }
        }
        payload = pbm_frame_.get();
    }
    const auto io_begin = std::chrono::steady_clock::now();
    pack_us_ = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(io_begin - pack_begin).count());
    if (std::fwrite(header_, 1, header_bytes_, file_) != header_bytes_
        || std::fwrite(payload, 1, payload_bytes(), file_) != payload_bytes())
        return false;
    bytes_written_ += record_bytes();
    const auto now = std::chrono::steady_clock::now();
    if (bytes_written_ == record_bytes() || now - last_flush_ >= config_.flush_period) {
        if (!sync())
            return false;
        last_flush_ = now;
    }
    io_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                            std::chrono::steady_clock::now() - io_begin)
                                            .count());
    return true;
}

} // namespace dart::recorder
