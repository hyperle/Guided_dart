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
        /* A backend detached this slot while its storage still exists. This
         * state does not extend the camera/slot lifetime: leases must always
         * be released before their owning camera is destroyed.
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
