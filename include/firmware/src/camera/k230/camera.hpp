#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <limits>
#include <memory>
#include <new>
#include <span>
#include <utility>

#include "camera/c_api/camera.h"
#include "camera/error_codes.h"

// Host tests use the neutral C defaults. A board build must consume the
// firmware routing configuration rather than silently probing CSI0.
#if defined(__riscv) && !defined(CONFIG_MPP_SENSOR_DEFAULT_CSI)
# error "K230 camera adapter requires the SDK generated/autoconf.h CSI configuration"
#endif

#include "camera/frame_lease.hpp"
#include "camera/interface.hpp"

namespace dart::camera::k230 {
template <std::uint32_t kCapacity>
class Camera final : public ICamera {
public:
    static_assert(kCapacity >= 1U, "a camera needs at least one frame lease slot");
    static_assert(
        kCapacity <= DART_CAMERA_MAX_OUTSTANDING_FRAMES,
        "lease capacity exceeds what the C ABI can represent");

    static std::expected<std::unique_ptr<Camera>, CameraError>
        create(const Controls& controls = {}) noexcept;
    ~Camera() override;

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    std::expected<void, CameraError> start() noexcept override;
    std::expected<AcquireResult, CameraError>
        acquire(std::chrono::milliseconds timeout) noexcept override;
    std::expected<void, CameraError> stop() noexcept override;
    State state() const noexcept override { return state_; }
    const FrameInfo& frame_info() const noexcept override { return frame_info_; }
    std::expected<void, CameraError>
        set_fixed_exposure_lines(std::uint32_t lines) noexcept override;
    std::uint32_t capacity() const noexcept override { return kCapacity; }
    std::uint32_t outstanding() const noexcept override;

private:
    explicit Camera(const Controls& controls) noexcept
        : controls_(controls) {}

    /* Trampoline stored in a lease slot, so a lease can release itself without
     * knowing which camera type owns the slot.
     */
    static std::expected<void, CameraError> slot_release(FrameLeaseAccess::Slot* slot) noexcept;

    std::expected<void, CameraError> release_slot(FrameLeaseAccess::Slot* slot) noexcept;
    std::expected<void, CameraError> retry_pending_releases() noexcept;
    FrameLeaseAccess::Slot* free_slot() noexcept;
    void fill_slot(FrameLeaseAccess::Slot* slot, const dart_camera_frame_t& frame) noexcept;
    void detach_leases() noexcept;

    std::expected<void, CameraError> refresh_caps() noexcept;
    std::expected<void, CameraError> refresh_state(Operation operation) const noexcept;
    std::expected<void, CameraError> destroy_handles() noexcept;

    Controls controls_{};
    FrameInfo frame_info_{};
    mutable State state_ = State::kEmpty;
    ::dart_media* media_ = nullptr;
    ::dart_camera* camera_ = nullptr;
    std::array<FrameLeaseAccess::Slot, kCapacity> slots_{};

private:
    static CallContext context_for(Operation operation) noexcept {
        CallContext context{};
        context.operation = operation;
        return context;
    }

    template <typename T>
    static std::expected<T, CameraError>
        failure(std::int32_t code, const CallContext& context) noexcept {
        return std::unexpected(CameraError{code, context});
    }

    /* The lease capacity is injected here and nowhere else: the C data plane sizes
     * its slots from the same number, so both sides agree by construction.
     */
    static dart_camera_config_t to_c_config(const Controls& controls) noexcept {
        dart_camera_config_t result{};
        dart_camera_config_init(&result);
        /* Zero geometry selects the driver's supported mode. Output follows it.
         * The firmware board configuration owns CSI routing, not the consumer. */
#ifdef CONFIG_MPP_SENSOR_DEFAULT_CSI
        result.csi_port = CONFIG_MPP_SENSOR_DEFAULT_CSI;
#endif
        result.capture_buffers = 4;
        result.channel_buffers = kCapacity > 6U ? kCapacity : 6U;
        result.max_outstanding_frames = kCapacity;
        result.exposure_mode = static_cast<std::uint32_t>(controls.exposure_mode);
        result.raw10_layout = static_cast<std::uint32_t>(controls.raw10_layout);
        /* start() applies exposure through the same path as later adjustments. */
        result.initial_exposure_lines = 0;
        return result;
    }

    static std::expected<void, CameraError> validate_config(const Controls& controls) noexcept {
        if (controls.exposure_mode == ExposureMode::kAuto
            && controls.initial_exposure_lines != 0U) {
            CallContext context = context_for(Operation::kConfigValidate);
            context.requested = controls.initial_exposure_lines;
            return failure<void>(DART_CAMERA_E_CONFIG_INVALID, context);
        }
        const dart_camera_config_t c_config = to_c_config(controls);
        const int rc = dart_camera_config_validate(&c_config);
        if (rc != DART_CAMERA_OK) {
            CallContext context = context_for(Operation::kConfigValidate);
            context.native_hint = rc;
            return failure<void>(rc, context);
        }
        return {};
    }
};

static_assert(static_cast<std::uint32_t>(State::kEmpty) == DART_CAMERA_STATE_EMPTY);
static_assert(static_cast<std::uint32_t>(State::kInitialized) == DART_CAMERA_STATE_INITIALIZED);
static_assert(static_cast<std::uint32_t>(State::kStarted) == DART_CAMERA_STATE_STARTED);
static_assert(static_cast<std::uint32_t>(State::kFaulted) == DART_CAMERA_STATE_FAULTED);
static_assert(static_cast<std::uint32_t>(ExposureMode::kFixed) == DART_CAMERA_EXPOSURE_FIXED);
static_assert(static_cast<std::uint32_t>(ExposureMode::kAuto) == DART_CAMERA_EXPOSURE_AUTO);
static_assert(static_cast<std::uint32_t>(Raw10Layout::kLe16Lsb) == DART_CAMERA_RAW10_LE16_LSB);
static_assert(static_cast<std::uint32_t>(Raw10Layout::kLe16Msb) == DART_CAMERA_RAW10_LE16_MSB);
static_assert(static_cast<std::uint32_t>(Raw10Layout::kMipi) == DART_CAMERA_RAW10_MIPI);
static_assert(static_cast<std::uint32_t>(Raw10Layout::kPackedLsb) == DART_CAMERA_RAW10_PACKED_LSB);
static_assert(
    static_cast<std::uint32_t>(FrameFlag::kSourcePtsUnvalidated)
    == DART_CAMERA_FRAME_SOURCE_PTS_UNVALIDATED);

template <std::uint32_t kCapacity>
std::expected<std::unique_ptr<Camera<kCapacity>>, CameraError>
    Camera<kCapacity>::create(const Controls& controls) noexcept {
    auto valid = validate_config(controls);
    if (!valid)
        return std::unexpected<CameraError>(std::move(valid.error()));

    auto object = std::unique_ptr<Camera>(new (std::nothrow) Camera(controls));
    if (object == nullptr) {
        return failure<std::unique_ptr<Camera>>(
            DART_CAMERA_E_NO_MEMORY, context_for(Operation::kCreate));
    }

    dart_media_t* media = nullptr;
    int rc = dart_media_create(&media);
    object->media_ = media;
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kMediaCreate);
        context.native_hint = rc;
        (void)object->destroy_handles();
        return failure<std::unique_ptr<Camera>>(rc, context);
    }

    dart_camera_t* camera = nullptr;
    rc = dart_camera_create(&camera);
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kCreate);
        context.native_hint = rc;
        (void)object->destroy_handles();
        return failure<std::unique_ptr<Camera>>(rc, context);
    }
    object->camera_ = camera;

    const dart_camera_config_t c_config = to_c_config(controls);
    rc = dart_camera_init(object->camera_, object->media_, &c_config);
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kInitialize);
        context.native_hint = rc;
        (void)object->refresh_state(Operation::kInitialize);
        (void)object->destroy_handles();
        return failure<std::unique_ptr<Camera>>(rc, context);
    }
    object->state_ = State::kInitialized;
    auto caps = object->refresh_caps();
    if (!caps) {
        (void)object->destroy_handles();
        return std::unexpected<CameraError>(std::move(caps.error()));
    }
    return object;
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError> Camera<kCapacity>::refresh_caps() noexcept {
    if (camera_ == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context_for(Operation::kGetCaps));
    }
    dart_camera_caps_t value{};
    const int rc = dart_camera_get_caps(camera_, &value);
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kGetCaps);
        context.native_hint = rc;
        return failure<void>(rc, context);
    }
    if (value.abi_version != DART_CAMERA_ABI_VERSION || value.struct_size != sizeof(value)
        || value.exposure_mode > DART_CAMERA_EXPOSURE_AUTO) {
        auto context = context_for(Operation::kGetCaps);
        context.observed = value.abi_version;
        return failure<void>(DART_CAMERA_E_CONFIG_VERSION, context);
    }
    if (value.output_width == 0U || value.output_height == 0U) {
        return failure<void>(
            DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH, context_for(Operation::kGetCaps));
    }
    frame_info_.width = value.output_width;
    frame_info_.height = value.output_height;
    return {};
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError>
    Camera<kCapacity>::refresh_state(Operation operation) const noexcept {
    if (camera_ == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context_for(operation));
    }
    std::uint32_t value = 0;
    const int rc = dart_camera_get_state(camera_, &value);
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(operation);
        context.native_hint = rc;
        return failure<void>(rc, context);
    }
    if (value > DART_CAMERA_STATE_FAULTED) {
        auto context = context_for(operation);
        context.observed = value;
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context);
    }
    state_ = static_cast<State>(value);
    return {};
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError> Camera<kCapacity>::start() noexcept {
    auto pending = retry_pending_releases();
    if (!pending)
        return pending;
    if (camera_ == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context_for(Operation::kStart));
    }
    int rc = dart_camera_start(camera_);
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kStart);
        context.native_hint = rc;
        (void)refresh_state(Operation::kStart);
        context.rollback_failed = state_ == State::kFaulted;
        return failure<void>(rc, context);
    }
    state_ = State::kStarted;
    if (controls_.exposure_mode == ExposureMode::kFixed && controls_.initial_exposure_lines != 0U) {
        auto applied = set_fixed_exposure_lines(controls_.initial_exposure_lines);
        if (!applied) {
            CameraError original = std::move(applied.error());
            const int stop_rc = dart_camera_stop(camera_);
            if (stop_rc != DART_CAMERA_OK || !refresh_state(Operation::kStart))
                state_ = State::kFaulted;
            if (stop_rc == DART_CAMERA_OK) {
                return std::unexpected<CameraError>(std::move(original));
            }
            auto context = context_for(Operation::kStart);
            context.native_hint = original.code();
            context.rollback_failed = true;
            return failure<void>(original.code(), context);
        }
    }
    return {};
}

template <std::uint32_t kCapacity>
std::uint32_t Camera<kCapacity>::outstanding() const noexcept {
    std::uint32_t count = 0;
    for (const auto& slot : slots_) {
        if (slot.state != FrameLeaseAccess::SlotState::kFree)
            ++count;
    }
    return count;
}

template <std::uint32_t kCapacity>
FrameLeaseAccess::Slot* Camera<kCapacity>::free_slot() noexcept {
    for (auto& slot : slots_) {
        if (slot.state == FrameLeaseAccess::SlotState::kFree)
            return &slot;
    }
    return nullptr;
}

template <std::uint32_t kCapacity>
void Camera<kCapacity>::fill_slot(
    FrameLeaseAccess::Slot* slot, const dart_camera_frame_t& frame) noexcept {
    const std::uint64_t bytes =
        static_cast<std::uint64_t>(frame.stride) * static_cast<std::uint64_t>(frame.height);
    slot->token = frame.token;
    slot->data = reinterpret_cast<const std::byte*>(frame.data);
    slot->width = frame.width;
    slot->height = frame.height;
    slot->stride = frame.stride;
    slot->flags = frame.flags;
    slot->source_pts = frame.source_pts;
    slot->received_us = frame.received_us;
    slot->view.luma = std::span<const std::byte>(slot->data, static_cast<std::size_t>(bytes));
    slot->view.width = frame.width;
    slot->view.height = frame.height;
    slot->view.stride = frame.stride;
    slot->view.source_pts = frame.source_pts;
    slot->view.sequence = frame.token;
    slot->view.source_pts_timebase_validated =
        (frame.flags & DART_CAMERA_FRAME_SOURCE_PTS_UNVALIDATED) == 0U;
    slot->view.received = std::chrono::microseconds(frame.received_us);
    slot->owner = this;
    slot->release = &Camera::slot_release;
    slot->pending_error.reset();
    slot->state = FrameLeaseAccess::SlotState::kHeld;
}

template <std::uint32_t kCapacity>
std::expected<AcquireResult, CameraError>
    Camera<kCapacity>::acquire(std::chrono::milliseconds timeout) noexcept {
    /* A release that failed earlier is retried before anything else: a stuck
     * lease keeps its slot, and reporting it immediately is more useful than
     * letting the pipeline wonder why capacity shrank.
     */
    auto retried = retry_pending_releases();
    if (!retried)
        return std::unexpected<CameraError>(std::move(retried.error()));

    const auto count = timeout.count();
    if (count < 0 || count > 1000) {
        auto context = context_for(Operation::kAcquire);
        context.requested = count < 0 ? 0U : static_cast<std::uint32_t>(count);
        return failure<AcquireResult>(DART_CAMERA_E_OUT_OF_RANGE, context);
    }
    if (camera_ == nullptr) {
        return failure<AcquireResult>(
            DART_CAMERA_E_INVALID_STATE, context_for(Operation::kAcquire));
    }

    /* Bounded capacity: when the pipeline holds every slot, the driver is not
     * touched at all, so the frame stays in its pool and is recycled there. That
     * is the drop signal, not an error.
     */
    FrameLeaseAccess::Slot* slot = free_slot();
    if (slot == nullptr) {
        return AcquireResult{AcquireStatus::kLeaseExhausted, FrameLease{}};
    }

    dart_camera_frame_t frame{};
    const int rc = dart_camera_acquire_frame(camera_, static_cast<std::uint32_t>(count), &frame);
    if (rc == DART_CAMERA_E_NO_FRAME) {
        return AcquireResult{AcquireStatus::kNoFrame, FrameLease{}};
    }
    if (rc == DART_CAMERA_E_LEASE_EXHAUSTED) {
        /* The C pool is sized from the same capacity, so this only means the two
         * sides disagree; it is still a drop, not a failure. */
        return AcquireResult{AcquireStatus::kLeaseExhausted, FrameLease{}};
    }
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kAcquire);
        context.native_hint = rc;
        context.observed = outstanding();
        return failure<AcquireResult>(rc, context);
    }

    const std::uint64_t bytes =
        static_cast<std::uint64_t>(frame.stride) * static_cast<std::uint64_t>(frame.height);
    if (bytes > std::numeric_limits<std::size_t>::max()) {
        (void)dart_camera_release_frame(camera_, &frame);
        return failure<AcquireResult>(DART_CAMERA_E_OUT_OF_RANGE, context_for(Operation::kAcquire));
    }
    fill_slot(slot, frame);
    return AcquireResult{AcquireStatus::kFrame, FrameLeaseAccess::make(slot)};
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError>
    Camera<kCapacity>::slot_release(FrameLeaseAccess::Slot* slot) noexcept {
    if (slot == nullptr || slot->owner == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_ARGUMENT, context_for(Operation::kRelease));
    }
    auto* self = static_cast<Camera*>(slot->owner);
    return self->release_slot(slot);
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError>
    Camera<kCapacity>::release_slot(FrameLeaseAccess::Slot* slot) noexcept {
    if (slot == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_ARGUMENT, context_for(Operation::kRelease));
    }
    if (camera_ == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context_for(Operation::kRelease));
    }
    dart_camera_frame_t frame{};
    frame.abi_version = DART_CAMERA_ABI_VERSION;
    frame.struct_size = sizeof(frame);
    frame.data = reinterpret_cast<const std::uint8_t*>(slot->data);
    frame.width = slot->width;
    frame.height = slot->height;
    frame.stride = slot->stride;
    frame.flags = slot->flags;
    frame.source_pts = slot->source_pts;
    frame.received_us = slot->received_us;
    frame.token = slot->token;
    frame.owner = camera_;
    const int rc = dart_camera_release_frame(camera_, &frame);
    if (rc == DART_CAMERA_OK) {
        /* Hand the slot back empty: a stale lease can no longer point at any
         * frame data, and the next acquire may reuse it with a fresh token. */
        *slot = FrameLeaseAccess::Slot{};
        return {};
    }
    auto context = context_for(Operation::kRelease);
    context.native_hint = rc;
    context.requested = static_cast<std::uint32_t>(slot->token);
    context.lease_token = slot->token;
    context.lease_generation = slot->token;
    context.lease_owner = reinterpret_cast<std::uintptr_t>(camera_);
    return failure<void>(rc, context);
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError> Camera<kCapacity>::retry_pending_releases() noexcept {
    for (auto& slot : slots_) {
        if (slot.state != FrameLeaseAccess::SlotState::kHeld)
            continue;
        if (!slot.pending_error)
            continue;
        auto retried = release_slot(&slot);
        if (!retried)
            return retried;
    }
    return {};
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError>
    Camera<kCapacity>::set_fixed_exposure_lines(std::uint32_t lines) noexcept {
    auto pending = retry_pending_releases();
    if (!pending)
        return pending;
    if (camera_ == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context_for(Operation::kSetExposure));
    }
    if (state_ != State::kStarted) {
        auto context = context_for(Operation::kSetExposure);
        context.state = static_cast<std::uint32_t>(state_);
        context.required_state = static_cast<std::uint32_t>(State::kStarted);
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context);
    }
    if (controls_.exposure_mode == ExposureMode::kAuto) {
        return failure<void>(DART_CAMERA_E_NOT_SUPPORTED, context_for(Operation::kSetExposure));
    }
    std::uint32_t min_lines = 0;
    std::uint32_t max_lines = 0;
    int rc = dart_camera_get_exposure_range(camera_, &min_lines, &max_lines);
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kExposureRange);
        context.native_hint = rc;
        context.has_exposure = true;
        context.requested_lines = lines;
        context.min_lines = min_lines;
        context.max_lines = max_lines;
        return failure<void>(rc, context);
    }
    auto context = context_for(Operation::kSetExposure);
    context.has_exposure = true;
    context.requested_lines = lines;
    context.min_lines = min_lines;
    context.max_lines = max_lines;
    if (min_lines == 0U || max_lines < min_lines) {
        return std::unexpected(
            CameraError{ErrorKind::kExposure, DART_CAMERA_E_OUT_OF_RANGE, context});
    }
    const auto applied = lines < min_lines ? min_lines : (lines > max_lines ? max_lines : lines);
    context.applied_lines = applied;
    rc = dart_camera_set_exposure_lines(camera_, applied);
    if (rc != DART_CAMERA_OK) {
        context.native_hint = rc;
        (void)refresh_state(Operation::kSetExposure);
        return failure<void>(rc, context);
    }
    return {};
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError> Camera<kCapacity>::stop() noexcept {
    auto pending = retry_pending_releases();
    if (!pending)
        return pending;
    if (camera_ == nullptr) {
        return failure<void>(DART_CAMERA_E_INVALID_STATE, context_for(Operation::kStop));
    }
    /* The C layer refuses to stop while any lease is outstanding, so a caller
     * that still holds frames gets BUSY here rather than a silently dead stream. */
    const int rc = dart_camera_stop(camera_);
    if (rc != DART_CAMERA_OK) {
        auto context = context_for(Operation::kStop);
        context.native_hint = rc;
        context.observed = outstanding();
        (void)refresh_state(Operation::kStop);
        return failure<void>(rc, context);
    }
    return refresh_state(Operation::kStop);
}

template <std::uint32_t kCapacity>
std::expected<void, CameraError> Camera<kCapacity>::destroy_handles() noexcept {
    auto pending = retry_pending_releases();
    if (!pending)
        return pending;
    if (camera_ != nullptr) {
        auto observed = refresh_state(Operation::kGetState);
        if (!observed)
            return observed;

        if (state_ == State::kStarted) {
            const int rc = dart_camera_stop(camera_);
            if (rc != DART_CAMERA_OK) {
                auto context = context_for(Operation::kStop);
                context.native_hint = rc;
                state_ = State::kFaulted;
                return failure<void>(rc, context);
            }
            state_ = State::kInitialized;
        }
        if (state_ != State::kEmpty) {
            const int rc = dart_camera_deinit(camera_);
            if (rc != DART_CAMERA_OK) {
                auto context = context_for(Operation::kDeinitialize);
                context.native_hint = rc;
                (void)refresh_state(Operation::kDeinitialize);
                return failure<void>(rc, context);
            }
            state_ = State::kEmpty;
        }
        const int rc = dart_camera_destroy(camera_);
        if (rc != DART_CAMERA_OK) {
            auto context = context_for(Operation::kDestroy);
            context.native_hint = rc;
            return failure<void>(rc, context);
        }
        camera_ = nullptr;
    }
    if (media_ != nullptr) {
        const int rc = dart_media_destroy(media_);
        if (rc != DART_CAMERA_OK) {
            auto context = context_for(Operation::kMediaDestroy);
            context.native_hint = rc;
            return failure<void>(rc, context);
        }
        media_ = nullptr;
    }
    return {};
}

template <std::uint32_t kCapacity>
void Camera<kCapacity>::detach_leases() noexcept {
    for (auto& slot : slots_) {
        if (slot.state == FrameLeaseAccess::SlotState::kFree)
            continue;
        /* The C layer will not release a frame while the caller still holds the
         * lease, so this path cannot free the dump. Detaching keeps a later
         * release from calling into a destroyed object; the caller gets a stale
         * lease instead, and the SDK handles stay for an explicit retry.
         */
        slot.state = FrameLeaseAccess::SlotState::kDetached;
        slot.owner = nullptr;
        slot.release = nullptr;
    }
}

template <std::uint32_t kCapacity>
Camera<kCapacity>::~Camera() {
    detach_leases();
    (void)destroy_handles();
}

/* Every capacity the C ABI allows is instantiated here, so the upper layer can
 * pick one at its call site. Raising DART_CAMERA_MAX_OUTSTANDING_FRAMES needs the
 * matching line below.
 */

} // namespace dart::camera::k230
