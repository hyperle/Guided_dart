#include "recorder/encoding/binary_format.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

namespace dart::recorder {

std::size_t BinaryFormat::packed_bytes(std::uint32_t width, std::uint32_t height) noexcept {
    if (width == 0 || height == 0 || width > kMaxDimension || height > kMaxDimension
        || static_cast<std::uint64_t>(width) * height > kMaxPixels)
        return 0;
    return ((static_cast<std::size_t>(width) + 7U) / 8U) * height;
}

void BinaryFormat::put32(std::byte* out, std::uint32_t value) noexcept {
    for (unsigned i = 0; i < 4; ++i)
        out[i] = std::byte((value >> (8U * i)) & 255U);
}

std::uint32_t BinaryFormat::get32(const std::byte* in) noexcept {
    std::uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i)
        value |= std::to_integer<std::uint32_t>(in[i]) << (8U * i);
    return value;
}

void BinaryFormat::header(std::byte* out, std::uint32_t width, std::uint32_t height) noexcept {
    std::memcpy(out, "DBV1", 4);
    put32(out + 4, width);
    put32(out + 8, height);
}

bool BinaryFormat::geometry(
    std::span<const std::byte> data, std::uint32_t& width, std::uint32_t& height) noexcept {
    if (data.size() != kHeaderBytes || std::memcmp(data.data(), "DBV1", 4) != 0)
        return false;
    width = get32(data.data() + 4);
    height = get32(data.data() + 8);
    return packed_bytes(width, height) != 0;
}

std::size_t BinaryFormat::prefix(std::byte* out, std::uint32_t value) noexcept {
    std::size_t size = 0;
    do {
        const auto low = value & 127U;
        value >>= 7U;
        out[size++] = std::byte(low | (value ? 128U : 0U));
    } while (value);
    return size;
}

// Zero means incomplete, -1 invalid, otherwise the consumed prefix bytes.
int BinaryFormat::parse(std::span<const std::byte> data, std::uint32_t& value) noexcept {
    value = 0;
    for (std::size_t i = 0; i < data.size() && i < 5; ++i) {
        const auto byte = std::to_integer<unsigned>(data[i]);
        if (i == 4 && byte > 15U)
            return -1;
        value |= (byte & 127U) << (7U * i);
        if (!(byte & 128U))
            return i != 0 && byte == 0 ? -1 : static_cast<int>(i + 1);
    }
    return data.size() >= 5 ? -1 : 0;
}

bool BinaryFormat::valid(std::uint32_t code, std::size_t packed, bool previous) noexcept {
    const auto kind = code & 3U;
    const auto bytes = code >> 2U;
    if (kind == kRepeat)
        return previous && (bytes == 0 || (bytes >= 2 && bytes <= packed));
    if (kind == kRaw)
        return bytes == packed;
    return bytes > 0 && bytes <= packed && (kind != kDelta || previous);
}

} // namespace dart::recorder
