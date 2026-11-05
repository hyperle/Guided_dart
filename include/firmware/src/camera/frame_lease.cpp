#include "camera/frame_lease.hpp"

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <utility>

#include "camera/error_codes.h"

#include "camera/error.hpp"
#include "camera/frame_view.hpp"

namespace dart::camera {

std::expected<void, CameraError> FrameLease::stale_lease_release(std::uint64_t token) noexcept {
    CallContext context{};
    context.operation = Operation::kRelease;
    context.lease_token = token;
    context.lease_generation = token;
    return std::unexpected(CameraError{DART_CAMERA_E_STALE_LEASE, context});
}

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

FrameLease FrameLeaseAccess::make(Slot* slot) noexcept { return FrameLease(slot); }

} // namespace dart::camera
