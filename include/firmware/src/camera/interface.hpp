#pragma once

#include <chrono>
#include <cstdint>
#include <expected>

#include "camera/frame_lease.hpp"

namespace dart::camera {
/* Actual output geometry and format, available after factory creation, before
 * start(). It stays fixed until the camera is destroyed; a new mode requires a
 * new camera. Every frame carries the same format and dimensions.
 */
class FrameInfo {
public:
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    PixelFormat format = PixelFormat::kY8;
    /* Physical width / height of one output pixel. A backend that scales the
     * sensor view anisotropically reports it so shape tests can undo it; e.g.
     * SC132GS 1080x1280 scaled to 480x640 gives (1080/480)/(1280/640) = 1.125.
     */
    float pixel_aspect = 1.0F;
};

enum class AcquireStatus : std::uint8_t {
    /* The lease holds a frame, valid until it is released. */
    kFrame,
    /* No frame arrived inside the timeout: a normal negative result, not an
     * error, and it allocates nothing.
     */
    kNoFrame,
    /* Every lease slot is held, so no frame was taken from the driver and the
     * frame is recycled there: the bounded-capacity drop signal.
     */
    kLeaseExhausted,
};

class AcquireResult {
public:
    AcquireStatus status = AcquireStatus::kNoFrame;
    FrameLease lease{};
};

enum class State : std::uint32_t {
    kEmpty = 0,
    kInitialized = 1,
    kStarted = 2,
    kFaulted = 3,
};

/* Backends own sensor configuration, pixel conversion and device resources.
 * All calls and lease releases belong to the acquisition thread. A camera must
 * outlive its leases; stop() returns BUSY while frames remain outstanding.
 * Acquisition/release use preallocated storage, with no per-frame allocation.
 */
class ICamera {
public:
    virtual ~ICamera() = default;

    virtual std::expected<void, CameraError> start() noexcept = 0;
    /* Three outcomes, only one of which is an error: a frame, no frame, or a full
     * lease pool. A caller with its own bounded pipeline drops on
     * kLeaseExhausted and counts it.
     */
    virtual std::expected<AcquireResult, CameraError>
        acquire(std::chrono::milliseconds timeout) noexcept = 0;
    virtual std::expected<void, CameraError> stop() noexcept = 0;
    virtual State state() const noexcept = 0;
    virtual const FrameInfo& frame_info() const noexcept = 0;

    /* Lease slots fixed at creation, and how many are in use right now. A
     * producer can check these before acquiring to apply its own drop policy.
     */
    virtual std::uint32_t capacity() const noexcept = 0;
    virtual std::uint32_t outstanding() const noexcept = 0;
};

} // namespace dart::camera
