#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace dart::recorder {
// DBV1: one 12-byte file header, then unsigned LEB128 (payload_bytes << 2 | kind)
// and payload. Geometry is shared, identities stay in frames.csv. Compressed
// records use the zlib checksum; no duplicate checksum or PNG wrapper is stored.
struct BinaryFormat final {
    // Kind zero carries sparse XOR. Empty repeats remain readable for legacy
    // DBV1 files; current encoders always write a nonempty payload.
    enum Kind : unsigned { kRepeat = 0, kSparse = 0, kRaw = 1, kFull = 2, kDelta = 3 };
    static constexpr std::size_t kHeaderBytes = 12;
    static constexpr std::uint32_t kMaxDimension = 16384;
    static constexpr std::size_t kMaxPixels = 64U * 1024U * 1024U;

    static std::size_t packed_bytes(std::uint32_t width, std::uint32_t height) noexcept;
    static void put32(std::byte* out, std::uint32_t value) noexcept;
    static std::uint32_t get32(const std::byte* in) noexcept;
    static void header(std::byte* out, std::uint32_t width, std::uint32_t height) noexcept;
    static bool geometry(
        std::span<const std::byte> data, std::uint32_t& width, std::uint32_t& height) noexcept;
    static std::size_t prefix(std::byte* out, std::uint32_t value) noexcept;
    // Zero means incomplete, -1 invalid, otherwise the consumed prefix bytes.
    static int parse(std::span<const std::byte> data, std::uint32_t& value) noexcept;
    static bool valid(std::uint32_t code, std::size_t packed, bool previous) noexcept;
};

} // namespace dart::recorder
