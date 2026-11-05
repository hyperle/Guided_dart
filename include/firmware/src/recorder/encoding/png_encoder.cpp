#include "recorder/encoding/png_encoder.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <zlib.h>

namespace dart::recorder {

PngEncoder::~PngEncoder() {
    if (initialized_)
        (void)::deflateEnd(&stream_);
}

bool PngEncoder::prepare(
    std::uint32_t width, std::uint32_t height, std::uint8_t threshold) noexcept {
    if (initialized_)
        (void)::deflateEnd(&stream_);
    stream_ = {};
    initialized_ = false;
    width_ = width;
    height_ = height;
    threshold_ = threshold;
    row_bytes_ = (static_cast<std::size_t>(width) + 7U) / 8U + 1U;
    raw_bytes_ = row_bytes_ * height;
    if (::deflateInit2(&stream_, Z_BEST_SPEED, Z_DEFLATED, 15, 8, Z_DEFAULT_STRATEGY) != Z_OK)
        return false;
    initialized_ = true;
    capacity_ = static_cast<std::size_t>(::deflateBound(&stream_, raw_bytes_)) + 57U;
    raw_.reset(new (std::nothrow) std::uint8_t[raw_bytes_]);
    png_.reset(new (std::nothrow) std::uint8_t[capacity_]);
    return raw_ && png_;
}

void PngEncoder::big_endian(std::uint8_t* out, std::uint32_t value) noexcept {
    for (unsigned i = 0; i < 4; ++i)
        out[i] = static_cast<std::uint8_t>(value >> (24U - i * 8U));
}

void PngEncoder::chunk(std::uint8_t* out, const char* type, std::size_t size) noexcept {
    big_endian(out, static_cast<std::uint32_t>(size));
    std::memcpy(out + 4, type, 4);
    big_endian(
        out + 8 + size,
        static_cast<std::uint32_t>(::crc32(0, out + 4, static_cast<uInt>(size + 4))));
}

std::span<const std::byte> PngEncoder::encode(std::span<const std::byte> pixels) noexcept {
    if (!raw_ || !png_ || pixels.size() < static_cast<std::size_t>(width_) * height_)
        return {};
    const auto* gray = reinterpret_cast<const std::uint8_t*>(pixels.data());
    for (std::uint32_t y = 0; y < height_; ++y) {
        const auto* row = gray + static_cast<std::size_t>(y) * width_;
        auto* out = raw_.get() + static_cast<std::size_t>(y) * row_bytes_;
        *out++ = 0;                     // PNG filter None. In grayscale PNG, bit 1 means white.
        std::uint32_t x = 0;
        for (; x + 8U <= width_; x += 8U) {
            *out++ = static_cast<std::uint8_t>(
                ((row[x] >= threshold_) << 7U) | ((row[x + 1] >= threshold_) << 6U)
                | ((row[x + 2] >= threshold_) << 5U) | ((row[x + 3] >= threshold_) << 4U)
                | ((row[x + 4] >= threshold_) << 3U) | ((row[x + 5] >= threshold_) << 2U)
                | ((row[x + 6] >= threshold_) << 1U) | (row[x + 7] >= threshold_));
        }
        if (x < width_) {
            std::uint8_t tail = 0;
            for (unsigned bit = 0; x < width_; ++x, ++bit)
                tail |= static_cast<std::uint8_t>((row[x] >= threshold_) << (7U - bit));
            *out = tail;
        }
    }
    if (::deflateReset(&stream_) != Z_OK)
        return {};
    stream_.next_in = raw_.get();
    stream_.avail_in = static_cast<uInt>(raw_bytes_);
    stream_.next_out = png_.get() + 41; // signature + IHDR + IDAT length/type
    stream_.avail_out = static_cast<uInt>(capacity_ - 57);
    if (::deflate(&stream_, Z_FINISH) != Z_STREAM_END)
        return {};
    const auto compressed = static_cast<std::size_t>(stream_.total_out);
    std::memcpy(png_.get(), "\x89PNG\r\n\x1a\n", 8);
    big_endian(png_.get() + 16, width_);
    big_endian(png_.get() + 20, height_);
    png_[24] = 1;                       // bit depth
    std::memset(png_.get() + 25, 0, 4); // grayscale, DEFLATE, filters, no interlace
    chunk(png_.get() + 8, "IHDR", 13);
    chunk(png_.get() + 33, "IDAT", compressed);
    chunk(png_.get() + 45 + compressed, "IEND", 0);
    return {reinterpret_cast<const std::byte*>(png_.get()), compressed + 57U};
}

} // namespace dart::recorder
