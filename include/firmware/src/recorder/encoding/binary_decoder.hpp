#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <zlib.h>

namespace dart::recorder {
class BinaryDecoder final {
public:
    bool prepare(std::uint32_t width, std::uint32_t height) noexcept;
    bool decode(std::uint32_t code, std::span<const std::byte> payload) noexcept;
    std::span<const std::byte> bits() const noexcept { return {bits_.get(), size_}; }

private:
    std::size_t size_ = 0;
    bool previous_ = false;
    std::unique_ptr<std::byte[]> bits_, scratch_;
};

} // namespace dart::recorder
