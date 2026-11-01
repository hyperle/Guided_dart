#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace dart::camera {
enum class FrameFlag : std::uint32_t {
    kSourcePtsUnvalidated = 1U,
};

class FrameView {
public:
    std::span<const std::byte> luma{};
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t stride = 0;
    std::uint64_t source_pts = 0;
    /* Frame identity: the lease token, stamped when a slot is reserved. Later
     * stages need it to name evidence, to correlate a detection result with the
     * frame that produced it, and to spot gaps. A gap means that acquisition
     * attempt produced no frame (E_NO_FRAME); a drop for lack of capacity
     * consumes nothing, because the driver is never touched.
     */
    std::uint64_t sequence = 0;
    bool source_pts_timebase_validated = false;
    std::chrono::microseconds received{};
};

} // namespace dart::camera
