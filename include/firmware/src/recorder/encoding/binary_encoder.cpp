module;
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <zlib.h>

export module dart.recorder.encoding.binary_encoder;

import dart.recorder.encoding.binary_format;

export namespace dart::recorder {

class BinaryEncoder final {
public:
    ~BinaryEncoder() {
        if (initialized_)
            (void)::deflateEnd(&stream_);
    }
    bool prepare(std::uint32_t width, std::uint32_t height, std::uint8_t threshold) noexcept {
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
    void reset() noexcept { has_previous_ = false; }
    std::span<const std::byte> encode(std::span<const std::byte> pixels) noexcept;

private:
    std::size_t compress(const std::byte* input) noexcept {
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
    z_stream stream_{};
    bool initialized_ = false, has_previous_ = false;
    std::uint32_t width_ = 0, height_ = 0;
    std::uint8_t threshold_ = 128;
    std::size_t packed_ = 0, capacity_ = 0;
    std::unique_ptr<std::byte[]> bits_, previous_, delta_, compressed_, record_;
};

std::span<const std::byte>
    BinaryEncoder::encode(std::span<const std::byte> pixels) noexcept {
    if (!initialized_ || !record_ || pixels.size() < static_cast<std::size_t>(width_) * height_)
        return {};
    const auto row_bytes = (static_cast<std::size_t>(width_) + 7U) / 8U;
    for (std::uint32_t y = 0; y < height_; ++y) {
        const auto row = static_cast<std::size_t>(y) * width_;
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
    const auto prefix = BinaryFormat::prefix(
        record_.get(), static_cast<std::uint32_t>((size << 2U) | kind));
    std::memmove(record_.get() + prefix, record_.get() + 5, size);
    std::memcpy(previous_.get(), bits_.get(), packed_);
    has_previous_ = true;
    return {record_.get(), prefix + size};
}

} // namespace dart::recorder
