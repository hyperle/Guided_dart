#include <expected>

#include <cassert>
#include <utility>
#include "camera/error_codes.h"

import dart.camera.frame_lease;

import dart.tests.odr_consumer;

namespace dart::tests {

class CameraContractTest final {
public:
    static void run() {
        assert(OdrConsumer::check());
        camera::CallContext context{};
        context.operation = camera::Operation::kAcquire;
        const camera::CameraError error{DART_CAMERA_E_INVALID_STATE, context};
        assert(error.kind() == camera::ErrorKind::kState);
        char text[128]{};
        camera::ErrorTextSink sink{text, sizeof(text)};
        error.append_context(sink);
        assert(sink.size() > 0);

        camera::FrameLeaseAccess::Slot slot{};
        slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
        slot.release = &CameraContractTest::release;
        slot.token = 42;
        auto lease = camera::FrameLeaseAccess::make(&slot);
        auto moved = std::move(lease);
        assert(!lease.valid() && moved.valid());
        assert(moved.release());
        assert(releases_ == 1 && !moved.valid());
        assert(moved.release() && releases_ == 1);
        slot.state = camera::FrameLeaseAccess::SlotState::kDetached;
        auto detached = camera::FrameLeaseAccess::make(&slot);
        auto stale = detached.release();
        assert(!stale && stale.error().kind() == camera::ErrorKind::kStaleLease);
        assert(releases_ == 1);
    }

private:
    static std::expected<void, camera::CameraError>
        release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        ++releases_;
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }
    inline static int releases_ = 0;
};

} // namespace dart::tests

int main() { dart::tests::CameraContractTest::run(); }
