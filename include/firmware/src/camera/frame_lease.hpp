#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>

#include "camera/error.hpp"
#include "camera/frame_view.hpp"

namespace dart::camera {
class FrameLease;

class FrameLeaseAccess {
public:
    enum class SlotState : std::uint8_t {
        kFree,
        kHeld,
        /* The camera was destroyed while this lease was still held. The lease can no
         * longer be released; doing so reports a stale lease instead of calling into
         * a dead object.
         */
        kDetached,
    };

    class Slot;
    using ReleaseFunction = std::expected<void, CameraError> (*)(Slot*) noexcept;

    class Slot {
    public:
        FrameView view{};
        void* owner = nullptr;
        std::uint64_t token = 0;
        const std::byte* data = nullptr;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t stride = 0;
        std::uint32_t flags = 0;
        std::uint64_t source_pts = 0;
        std::uint64_t received_us = 0;
        ReleaseFunction release = nullptr;
        /* Set when a lease is destroyed without a successful release, so the owning
         * camera can retry it and report it instead of losing it.
         */
        std::optional<CameraError> pending_error{};
        SlotState state = SlotState::kFree;
    };

    static FrameLease make(Slot* slot) noexcept;
};

class FrameLease {
public:
    FrameLease() noexcept = default;
    FrameLease(FrameLease&& other) noexcept;
    FrameLease& operator=(FrameLease&& other) noexcept;
    FrameLease(const FrameLease&) = delete;
    FrameLease& operator=(const FrameLease&) = delete;
    ~FrameLease();

    const FrameView& view() const noexcept;
    std::expected<void, CameraError> release() noexcept;
    bool valid() const noexcept;

private:
    explicit FrameLease(FrameLeaseAccess::Slot* slot) noexcept
        : slot_(slot) {}

    FrameLeaseAccess::Slot* slot_ = nullptr;
    friend class FrameLeaseAccess;

private:
    static std::expected<void, CameraError> stale_lease_release(std::uint64_t token) noexcept;
};

} // namespace dart::camera
