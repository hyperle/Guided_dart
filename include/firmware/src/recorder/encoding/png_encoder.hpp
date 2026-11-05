#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <zlib.h>

namespace dart::recorder {
// One grayscale bit per pixel, filter None, fast DEFLATE. All working memory
// and the compressor state are allocated once, outside the capture loop.
class PngEncoder final {
public:
    ~PngEncoder();
    bool prepare(std::uint32_t width, std::uint32_t height, std::uint8_t threshold) noexcept;
    std::span<const std::byte> encode(std::span<const std::byte> pixels) noexcept;

private:
    static void big_endian(std::uint8_t* out, std::uint32_t value) noexcept;
    static void chunk(std::uint8_t* out, const char* type, std::size_t size) noexcept;
    z_stream stream_{};
    bool initialized_ = false;
    std::uint32_t width_ = 0, height_ = 0;
    std::uint8_t threshold_ = 128;
    std::size_t row_bytes_ = 0, raw_bytes_ = 0, capacity_ = 0;
    std::unique_ptr<std::uint8_t[]> raw_, png_;
};

} // namespace dart::recorder
