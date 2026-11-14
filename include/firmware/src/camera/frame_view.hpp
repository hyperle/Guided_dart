#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace dart::camera {
enum class FrameFlag : std::uint32_t {
    kSourcePtsUnvalidated = 1U,
};

/* Pixel layouts a backend may deliver. Consumers check the format they accept
 * instead of guessing from the buffer size.
 *  kY8:     one 8-bit grayscale byte per pixel.
 *  kRgb565: one 16-bit little-endian word per pixel, R in bits 15..11,
 *           G in 10..5, B in 4..0 (the CanMV/OpenMV in-memory layout).
 */
enum class PixelFormat : std::uint8_t {
    kY8 = 0,
    kRgb565 = 1,
};

constexpr std::uint32_t bytes_per_pixel(PixelFormat format) noexcept {
    return format == PixelFormat::kRgb565 ? 2U : 1U;
}

class FrameView {
public:
    /* Read-only pixels in `format`. Rows may contain padding: stride is in
     * bytes and covers at least width * bytes_per_pixel(format). The span is
     * valid only while its FrameLease is held; consumers must not retain a
     * bare view.
     */
    std::span<const std::byte> pixels{};
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t stride = 0;
    PixelFormat format = PixelFormat::kY8;
    std::uint64_t source_pts = 0; // Unmodified backend timestamp; not an exposure time.
    /* Frame identity: the lease token, stamped when a slot is reserved. Later
     * stages need it to name evidence, to correlate a detection result with the
     * frame that produced it, and to spot gaps. A gap means that acquisition
     * attempt produced no frame (E_NO_FRAME); a drop for lack of capacity
     * consumes nothing, because the driver is never touched.
     */
    std::uint64_t sequence = 0;
    bool source_pts_timebase_validated = false;
    /* Receipt time on the host's monotonic clock. The source PTS flag does not
     * establish a conversion to this clock or synchronization with the IMU.
     */
    std::chrono::microseconds received{};

    std::uint32_t row_bytes() const noexcept { return width * bytes_per_pixel(format); }

    // Geometry and buffer agree: every row of every pixel lies inside `pixels`.
    bool valid() const noexcept {
        if (width == 0 || height == 0 || pixels.data() == nullptr)
            return false;
        const std::uint64_t row = static_cast<std::uint64_t>(width) * bytes_per_pixel(format);
        return stride >= row
            && static_cast<std::uint64_t>(height - 1U) * stride + row <= pixels.size();
    }

    // Callers check valid() once per frame; row() itself does no bounds check.
    const std::byte* row(std::uint32_t y) const noexcept {
        return pixels.data() + static_cast<std::size_t>(y) * stride;
    }
};

} // namespace dart::camera
