#include "recorder/encoding/binary_decoder.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <zlib.h>

#include "recorder/encoding/binary_format.hpp"

namespace dart::recorder {

bool BinaryDecoder::prepare(std::uint32_t width, std::uint32_t height) noexcept {
    size_ = BinaryFormat::packed_bytes(width, height);
    previous_ = false;
    if (!size_)
        return false;
    bits_.reset(new (std::nothrow) std::byte[size_]);
    scratch_.reset(new (std::nothrow) std::byte[size_]);
    return bits_ && scratch_;
}

bool BinaryDecoder::decode(std::uint32_t code, std::span<const std::byte> payload) noexcept {
    if (!bits_ || !BinaryFormat::valid(code, size_, previous_) || payload.size() != code >> 2U)
        return false;
    const auto kind = code & 3U;
    if (kind == BinaryFormat::kRepeat) {
        if (payload.empty())
            return true;
        std::memcpy(scratch_.get(), bits_.get(), size_);
        std::size_t offset = 0, position = 0;
        while (offset < payload.size()) {
            std::uint32_t gap = 0;
            const auto count = BinaryFormat::parse(payload.subspan(offset), gap);
            if (count <= 0 || gap >= size_ - position)
                return false;
            offset += static_cast<std::size_t>(count);
            position += gap;
            if (offset >= payload.size() || payload[offset] == std::byte{0})
                return false;
            scratch_[position++] ^= payload[offset++];
        }
        std::memcpy(bits_.get(), scratch_.get(), size_);
        return true;
    }
    if (kind == BinaryFormat::kRaw) {
        std::memcpy(scratch_.get(), payload.data(), size_);
    } else {
        z_stream stream{};
        if (::inflateInit(&stream) != Z_OK)
            return false;
        stream.next_in = reinterpret_cast<Bytef*>(const_cast<std::byte*>(payload.data()));
        stream.avail_in = static_cast<uInt>(payload.size());
        stream.next_out = reinterpret_cast<Bytef*>(scratch_.get());
        stream.avail_out = static_cast<uInt>(size_);
        const auto result = ::inflate(&stream, Z_FINISH);
        const bool valid = result == Z_STREAM_END && stream.total_out == size_
                        && stream.total_in == payload.size();
        (void)::inflateEnd(&stream);
        if (!valid)
            return false;
    }
    for (std::size_t i = 0; i < size_; ++i)
        bits_[i] = kind == BinaryFormat::kDelta ? bits_[i] ^ scratch_[i] : scratch_[i];
    previous_ = true;
    return true;
}

} // namespace dart::recorder
