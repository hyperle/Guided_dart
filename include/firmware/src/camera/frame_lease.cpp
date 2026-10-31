module;
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <utility>

#include "camera/error_codes.h"

export module dart.camera.frame_lease;

export import dart.camera.error;
export import dart.camera.frame_view;

export namespace dart::camera {

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
    static std::expected<void, CameraError> stale_lease_release(std::uint64_t token) noexcept {
        CallContext context{};
        context.operation = Operation::kRelease;
        context.lease_token = token;
        context.lease_generation = token;
        return std::unexpected(CameraError{DART_CAMERA_E_STALE_LEASE, context});
    }
};

FrameLease::FrameLease(FrameLease&& other) noexcept
    : slot_(other.slot_) {
    other.slot_ = nullptr;
}

FrameLease& FrameLease::operator=(FrameLease&& other) noexcept {
    if (this == &other)
        return *this;
    if (slot_ != nullptr && slot_->state == FrameLeaseAccess::SlotState::kHeld) {
        auto result = release();
        if (!result && slot_ != nullptr)
            slot_->pending_error = std::move(result.error());
    }
    slot_ = other.slot_;
    other.slot_ = nullptr;
    return *this;
}

FrameLease::~FrameLease() {
    if (slot_ == nullptr || slot_->state != FrameLeaseAccess::SlotState::kHeld)
        return;
    auto result = release();
    if (!result && slot_ != nullptr && slot_->state == FrameLeaseAccess::SlotState::kHeld) {
        /* A destructor cannot report a failure, so the owning camera keeps it and
         * surfaces it on its next protected call instead of losing it. */
        slot_->pending_error = std::move(result.error());
    }
}

const FrameView& FrameLease::view() const noexcept {
    static const FrameView empty{};
    if (slot_ == nullptr || slot_->state != FrameLeaseAccess::SlotState::kHeld)
        return empty;
    return slot_->view;
}

bool FrameLease::valid() const noexcept {
    return slot_ != nullptr && slot_->state == FrameLeaseAccess::SlotState::kHeld;
}

std::expected<void, CameraError> FrameLease::release() noexcept {
    if (slot_ == nullptr)
        return {};
    FrameLeaseAccess::Slot* slot = slot_;
    if (slot->state == FrameLeaseAccess::SlotState::kFree) {
        slot_ = nullptr;
        return {};
    }
    if (slot->state == FrameLeaseAccess::SlotState::kDetached || slot->release == nullptr) {
        slot_ = nullptr;
        return stale_lease_release(slot->token);
    }
    auto result = slot->release(slot);
    if (result)
        slot_ = nullptr;
    return result;
}

FrameLease FrameLeaseAccess::make(Slot* slot) noexcept {
    return FrameLease(slot);
}

} // namespace dart::camera
