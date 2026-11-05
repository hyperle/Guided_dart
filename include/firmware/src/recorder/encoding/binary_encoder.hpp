#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <zlib.h>

namespace dart::recorder {
class BinaryEncoder final {
public:
    ~BinaryEncoder();
    bool prepare(std::uint32_t width, std::uint32_t height, std::uint8_t threshold) noexcept;
    void reset() noexcept { has_previous_ = false; }
    std::span<const std::byte> encode(std::span<const std::byte> pixels) noexcept;

private:
    std::size_t compress(const std::byte* input) noexcept;
    z_stream stream_{};
    bool initialized_ = false, has_previous_ = false;
    std::uint32_t width_ = 0, height_ = 0;
    std::uint8_t threshold_ = 128;
    std::size_t packed_ = 0, capacity_ = 0;
    std::unique_ptr<std::byte[]> bits_, previous_, delta_, compressed_, record_;
};

} // namespace dart::recorder
