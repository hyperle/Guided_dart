#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <zlib.h>

#include "recorder/config.hpp"
#include "recorder/encoding/binary_format.hpp"

namespace dart::recorder {

// Lossless 8-bit grayscale, Up row filtering and RLE DEFLATE. All working memory
// and the compressor state are allocated once, outside the capture loop.
class PngEncoder final {
public:
    ~PngEncoder();
    bool prepare(std::uint32_t width, std::uint32_t height) noexcept;
    std::size_t max_output_bytes() const noexcept { return capacity_; }
    std::span<const std::byte> encode(std::span<const std::byte> pixels, std::uint32_t stride = 0) noexcept;

private:
    static void big_endian(std::uint8_t* out, std::uint32_t value) noexcept;
    static void chunk(std::uint8_t* out, const char* type, std::size_t size) noexcept;
    z_stream stream_{};
    bool initialized_ = false;
    std::uint32_t width_ = 0, height_ = 0;
    std::size_t row_bytes_ = 0, raw_bytes_ = 0, capacity_ = 0;
    std::unique_ptr<std::uint8_t[]> raw_, png_;
};

class BinaryEncoder final {
public:
    ~BinaryEncoder();
    bool prepare(std::uint32_t width, std::uint32_t height, std::uint8_t threshold) noexcept;
    std::size_t max_output_bytes() const noexcept { return packed_ + 5U; }
    void reset() noexcept { has_previous_ = false; }
    std::span<const std::byte> encode(std::span<const std::byte> pixels, std::uint32_t stride = 0) noexcept;

private:
    std::size_t compress(const std::byte* input) noexcept;
    z_stream stream_{};
    bool initialized_ = false, has_previous_ = false;
    std::uint32_t width_ = 0, height_ = 0;
    std::uint8_t threshold_ = 128;
    std::size_t packed_ = 0, capacity_ = 0;
    std::unique_ptr<std::byte[]> bits_, previous_, delta_, compressed_, record_;
};

inline PngEncoder::~PngEncoder() {
    if (initialized_)
        (void)::deflateEnd(&stream_);
}

inline bool PngEncoder::prepare(std::uint32_t width, std::uint32_t height) noexcept {
    if (initialized_)
        (void)::deflateEnd(&stream_);
    stream_ = {};
    initialized_ = false;
    width_ = width;
    height_ = height;
    if (width == 0 || height == 0 || width > 16384 || height > 16384)
        return false;
    row_bytes_ = static_cast<std::size_t>(width) + 1U;
    raw_bytes_ = row_bytes_ * height;
    if (::deflateInit2(&stream_, Z_BEST_SPEED, Z_DEFLATED, 15, 8, Z_RLE) != Z_OK)
        return false;
    initialized_ = true;
    capacity_ = static_cast<std::size_t>(::deflateBound(&stream_, raw_bytes_)) + 57U;
    raw_.reset(new (std::nothrow) std::uint8_t[raw_bytes_]);
    png_.reset(new (std::nothrow) std::uint8_t[capacity_]);
    return raw_ && png_;
}

inline void PngEncoder::big_endian(std::uint8_t* out, std::uint32_t value) noexcept {
    for (unsigned i = 0; i < 4; ++i)
        out[i] = static_cast<std::uint8_t>(value >> (24U - i * 8U));
}

inline void PngEncoder::chunk(std::uint8_t* out, const char* type, std::size_t size) noexcept {
    big_endian(out, static_cast<std::uint32_t>(size));
    std::memcpy(out + 4, type, 4);
    big_endian(
        out + 8 + size,
        static_cast<std::uint32_t>(::crc32(0, out + 4, static_cast<uInt>(size + 4))));
}

inline std::span<const std::byte> PngEncoder::encode(std::span<const std::byte> pixels, std::uint32_t stride) noexcept {
    if (stride == 0) stride = width_;
    if (stride < width_ || height_ == 0
        || pixels.size() < static_cast<std::uint64_t>(height_ - 1U) * stride + width_)
        return {};
    if (!raw_ || !png_ || pixels.data() == nullptr)
        return {};
    const auto* gray = reinterpret_cast<const std::uint8_t*>(pixels.data());
    for (std::uint32_t y = 0; y < height_; ++y) {
        auto* out = raw_.get() + static_cast<std::size_t>(y) * row_bytes_;
        *out++ = 2; // PNG Up: first row references zero; subtraction wraps modulo 256.
        const auto* row = gray + static_cast<std::size_t>(y) * stride;
        if (y == 0) {
            std::memcpy(out, row, width_);
        } else {
            const auto* previous = row - stride;
            for (std::uint32_t x = 0; x < width_; ++x)
                out[x] = static_cast<std::uint8_t>(row[x] - previous[x]);
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
    png_[24] = 8;                       // bit depth
    std::memset(png_.get() + 25, 0, 4); // grayscale, DEFLATE, filters, no interlace
    chunk(png_.get() + 8, "IHDR", 13);
    chunk(png_.get() + 33, "IDAT", compressed);
    chunk(png_.get() + 45 + compressed, "IEND", 0);
    return {reinterpret_cast<const std::byte*>(png_.get()), compressed + 57U};
}

inline BinaryEncoder::~BinaryEncoder() {
    if (initialized_)
        (void)::deflateEnd(&stream_);
}

inline bool BinaryEncoder::prepare(
    std::uint32_t width, std::uint32_t height, std::uint8_t threshold) noexcept {
    if (initialized_)
        (void)::deflateEnd(&stream_);
    initialized_ = false;
    stream_ = {};
    width_ = width;
    height_ = height;
    threshold_ = threshold;
    packed_ = BinaryFormat::packed_bytes(width, height);
    if (!packed_ || ::deflateInit(&stream_, Z_BEST_COMPRESSION) != Z_OK)
        return false;
    initialized_ = true;
    capacity_ = static_cast<std::size_t>(::deflateBound(&stream_, packed_));
    bits_.reset(new (std::nothrow) std::byte[packed_]);
    previous_.reset(new (std::nothrow) std::byte[packed_]);
    delta_.reset(new (std::nothrow) std::byte[packed_]);
    compressed_.reset(new (std::nothrow) std::byte[capacity_]);
    record_.reset(new (std::nothrow) std::byte[packed_ + 5]);
    reset();
    return bits_ && previous_ && delta_ && compressed_ && record_;
}

inline std::size_t BinaryEncoder::compress(const std::byte* input) noexcept {
    if (::deflateReset(&stream_) != Z_OK)
        return 0;
    stream_.next_in = reinterpret_cast<Bytef*>(const_cast<std::byte*>(input));
    stream_.avail_in = static_cast<uInt>(packed_);
    stream_.next_out = reinterpret_cast<Bytef*>(compressed_.get());
    stream_.avail_out = static_cast<uInt>(capacity_);
    if (::deflate(&stream_, Z_FINISH) != Z_STREAM_END)
        return 0;
    return stream_.total_out;
}

inline std::span<const std::byte> BinaryEncoder::encode(std::span<const std::byte> pixels, std::uint32_t stride) noexcept {
    if (stride == 0) stride = width_;
    if (stride < width_ || height_ == 0
        || pixels.size() < static_cast<std::uint64_t>(height_ - 1U) * stride + width_)
        return {};
    if (!initialized_ || !record_ || pixels.data() == nullptr)
        return {};
    const auto row_bytes = (static_cast<std::size_t>(width_) + 7U) / 8U;
    for (std::uint32_t y = 0; y < height_; ++y) {
        const auto row = static_cast<std::size_t>(y) * stride;
        for (std::uint32_t x = 0; x < width_; x += 8U) {
            unsigned byte = 0;
            const auto count = std::min(8U, width_ - x);
            for (unsigned bit = 0; bit < count; ++bit)
                byte |= (std::to_integer<unsigned>(pixels[row + x + bit]) >= threshold_)
                     << (7U - bit);
            bits_[static_cast<std::size_t>(y) * row_bytes + x / 8U] = std::byte(byte);
        }
    }
    // Start with raw bits, so incompressible noise never expands significantly.
    auto kind = BinaryFormat::kRaw;
    auto size = packed_;
    std::memcpy(record_.get() + 5, bits_.get(), size);
    const auto full = compress(bits_.get());
    if (!full)
        return {};
    if (full < size) {
        kind = BinaryFormat::kFull;
        size = full;
        std::memcpy(record_.get() + 5, compressed_.get(), size);
    }
    if (has_previous_) {
        for (std::size_t i = 0; i < packed_; ++i)
            delta_[i] = bits_[i] ^ previous_[i];
        const auto delta = compress(delta_.get());
        if (!delta)
            return {};
        if (delta < size) {
            kind = BinaryFormat::kDelta;
            size = delta;
            std::memcpy(record_.get() + 5, compressed_.get(), size);
        }
        // Very sparse changes are smaller as byte gaps + XOR masks than even
        // a minimal zlib stream. Reuse the scratch buffer, no extra allocation.
        std::size_t sparse = 0, next = 0;
        for (std::size_t i = 0; i < packed_ && sparse < size; ++i) {
            if (delta_[i] == std::byte{0})
                continue;
            std::byte gap[5];
            const auto count = BinaryFormat::prefix(gap, static_cast<std::uint32_t>(i - next));
            if (sparse + count + 1 >= size) {
                sparse = size;
                break;
            }
            std::memcpy(compressed_.get() + sparse, gap, count);
            sparse += count;
            compressed_[sparse++] = delta_[i];
            next = i + 1;
        }
        if (sparse != 0 && sparse < size) {
            kind = BinaryFormat::kSparse;
            size = sparse;
            std::memcpy(record_.get() + 5, compressed_.get(), size);
        }
    }
    const auto prefix =
        BinaryFormat::prefix(record_.get(), static_cast<std::uint32_t>((size << 2U) | kind));
    std::memmove(record_.get() + prefix, record_.get() + 5, size);
    std::memcpy(previous_.get(), bits_.get(), packed_);
    has_previous_ = true;
    return {record_.get(), prefix + size};
}

} // namespace dart::recorder
