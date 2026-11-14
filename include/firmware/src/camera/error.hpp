#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "camera/error_codes.h"

namespace dart::camera {
class ErrorTextSink {
public:
    ErrorTextSink(char* buffer, std::size_t capacity) noexcept;

    void append(std::string_view text) noexcept;
    void append_field(std::string_view key, std::string_view value) noexcept;
    void append_field(std::string_view key, std::uint64_t value) noexcept;
    void append_field(std::string_view key, std::int64_t value) noexcept;
    void append_field(std::string_view key, std::uint32_t value) noexcept;
    void append_field(std::string_view key, std::int32_t value) noexcept;
    void append_field(std::string_view key, bool value) noexcept;
    void append_hex_field(std::string_view key, std::uint64_t value) noexcept;
    void append_float_field(std::string_view key, float value) noexcept;

    std::size_t size() const noexcept { return size_; }
    bool truncated() const noexcept { return truncated_; }

private:
    void begin_field(std::string_view key) noexcept;
    void append_unsigned(std::uint64_t value) noexcept;
    void append_signed(std::int64_t value) noexcept;

    char* buffer_;
    std::size_t capacity_;
    std::size_t size_;
    bool truncated_;
    bool has_field_;
};

enum class Operation {
    kUnknown,
    kConfigValidate,
    kMediaCreate,
    kMediaDestroy,
    kCreate,
    kInitialize,
    kStart,
    kAcquire,
    kRelease,
    kStop,
    kDeinitialize,
    kDestroy,
    kGetState,
    kGetCaps,
    kExposureRange,
    kSetExposure,
};

class CallContext {
public:
    struct FrameGeometry {
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t stride = 0;
        std::uint32_t format = 0;
        std::uint32_t pool_id = 0;
        std::uint64_t physical_address = 0;
        std::uint64_t length = 0;
        std::uint64_t block_bytes = 0;
    };

    struct SensorMode {
        std::uint32_t csi_port = 0;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t fps = 0;
        std::uint32_t hdr_mode = 0;
    };

    Operation operation = Operation::kUnknown;
    // Owned, bounded backend diagnostics. The common layer never decodes a
    // vendor bit layout or borrows a backend-owned message buffer.
    std::array<char, 16> backend{};
    std::array<char, 96> backend_detail{};
    std::int32_t native_hint = 0;
    std::uint32_t requested = 0;
    std::uint32_t observed = 0;
    std::uint32_t state = 0;
    std::uint32_t required_state = 0;
    bool rollback_failed = false;
    bool has_sensor_mode = false;
    SensorMode requested_sensor{};
    SensorMode observed_sensor{};
    bool has_frame_geometry = false;
    FrameGeometry frame_geometry{};
    std::uint64_t physical_address = 0;
    std::uintptr_t mapped_address = 0;
    std::uint64_t length = 0;
    std::uint64_t block_bytes = 0;
    bool has_exposure = false;
    std::uint32_t requested_lines = 0;
    std::uint32_t min_lines = 0;
    std::uint32_t max_lines = 0;
    std::uint32_t applied_lines = 0;
    std::int32_t previous_release_code = 0;
    std::uint64_t lease_token = 0;
    std::uint64_t lease_generation = 0;
    std::uintptr_t lease_owner = 0;
};

enum class ErrorKind {
    kArgument,
    kState,
    kBusy,
    kUnsupported,
    kOutOfRange,
    kNoMemory,
    kStaleLease,
    kHardware,
    kCacheSync,
    kRelease,
    kPlatform,
    kExposure,
};

class CameraError final {
public:
    CameraError(std::int32_t code, const CallContext& context) noexcept
        : CameraError(classify(code), code, context) {}

    // A native code can have a more specific meaning at its source: an invalid
    // sensor exposure range uses OUT_OF_RANGE but belongs to the exposure domain.
    CameraError(ErrorKind kind, std::int32_t code, const CallContext& context) noexcept
        : kind_(kind)
        , code_(code)
        , context_(context) {}

    ErrorKind kind() const noexcept { return kind_; }
    std::string_view category() const noexcept { return description().category; }
    std::string_view message() const noexcept { return description().message; }
    std::int32_t code() const noexcept { return code_; }
    Operation operation() const noexcept { return context_.operation; }
    const CallContext& context() const noexcept { return context_; }
    void append_context(ErrorTextSink& sink) const noexcept;

private:
    static std::string_view operation_name(Operation operation) noexcept;

    struct Description {
        std::string_view category;
        std::string_view message;
    };

    static ErrorKind classify(std::int32_t code) noexcept;
    Description description() const noexcept;

    ErrorKind kind_;
    std::int32_t code_;
    CallContext context_;
};

static_assert(std::is_trivially_copyable_v<CameraError>);
static_assert(std::is_standard_layout_v<CameraError>);
static_assert(
    DART_CAMERA_E_SITE_CODE_MAX < DART_CAMERA_CODE_ERRNO_MAX,
    "site code range must remain below errno range");
static_assert(
    [] {
        constexpr std::array<std::int32_t, 26> codes = {
            DART_CAMERA_E_INVALID_ARGUMENT,
            DART_CAMERA_E_INVALID_STATE,
            DART_CAMERA_E_BUSY,
            DART_CAMERA_E_NOT_SUPPORTED,
            DART_CAMERA_E_OUT_OF_RANGE,
            DART_CAMERA_E_STALE_LEASE,
            DART_CAMERA_E_NO_MEMORY,
            DART_CAMERA_E_NO_FRAME,
            DART_CAMERA_E_CONFIG_VERSION,
            DART_CAMERA_E_CONFIG_INVALID,
            DART_CAMERA_E_SENSOR_PROBE_FAILED,
            DART_CAMERA_E_SENSOR_MODE_MISMATCH,
            DART_CAMERA_E_POOL_CREATE_FAILED,
            DART_CAMERA_E_BLOCK_EXHAUSTED,
            DART_CAMERA_E_MAP_FAILED,
            DART_CAMERA_E_BLOCK_RELEASE_FAILED,
            DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH,
            DART_CAMERA_E_FRAME_POOL_MISMATCH,
            DART_CAMERA_E_FRAME_NOT_MAPPED,
            DART_CAMERA_E_CACHE_INVALIDATE_FAILED,
            DART_CAMERA_E_CLOCK_FAILED,
            DART_CAMERA_E_EXPOSURE_RANGE_FAILED,
            DART_CAMERA_E_EXPOSURE_WRITE_FAILED,
            DART_CAMERA_E_LEASE_ABI_MISMATCH,
            DART_CAMERA_E_LEASE_EXHAUSTED,
            DART_CAMERA_E_SITE_CODE_MAX,
        };
        for (std::size_t i = 0; i < codes.size(); ++i) {
            for (std::size_t j = i + 1; j < codes.size(); ++j) {
                if (codes[i] == codes[j])
                    return false;
            }
        }
        return true;
    }(),
    "camera site codes must be unique");

} // namespace dart::camera
