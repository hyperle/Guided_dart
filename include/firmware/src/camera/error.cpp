module;
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>

#include "camera/error_codes.h"

export module dart.camera.error;



export namespace dart::camera {

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

ErrorTextSink::ErrorTextSink(char* buffer, std::size_t capacity) noexcept
    : buffer_(buffer)
    , capacity_(capacity)
    , size_(0)
    , truncated_(false)
    , has_field_(false) {
    if (buffer_ != nullptr && capacity_ != 0)
        buffer_[0] = '\0';
}

void ErrorTextSink::append(std::string_view text) noexcept {
    if (text.empty())
        return;
    if (buffer_ == nullptr || capacity_ == 0 || size_ >= capacity_) {
        truncated_ = true;
        return;
    }
    const std::size_t available = capacity_ - 1U - size_;
    const std::size_t count = text.size() < available ? text.size() : available;
    if (count != 0) {
        std::memcpy(buffer_ + size_, text.data(), count);
        size_ += count;
        buffer_[size_] = '\0';
    }
    if (count != text.size())
        truncated_ = true;
}

void ErrorTextSink::begin_field(std::string_view key) noexcept {
    if (has_field_)
        append(" ");
    has_field_ = true;
    append(key);
    append("=");
}

void ErrorTextSink::append_field(std::string_view key, std::string_view value) noexcept {
    begin_field(key);
    append(value);
}

void ErrorTextSink::append_unsigned(std::uint64_t value) noexcept {
    char text[24] = {};
    const auto result = std::to_chars(text, text + sizeof(text), value);
    if (result.ec == std::errc{})
        append(std::string_view(text, static_cast<std::size_t>(result.ptr - text)));
    else
        append("?");
}

void ErrorTextSink::append_signed(std::int64_t value) noexcept {
    char text[24] = {};
    const auto result = std::to_chars(text, text + sizeof(text), value);
    if (result.ec == std::errc{})
        append(std::string_view(text, static_cast<std::size_t>(result.ptr - text)));
    else
        append("?");
}

void ErrorTextSink::append_field(std::string_view key, std::uint64_t value) noexcept {
    begin_field(key);
    append_unsigned(value);
}

void ErrorTextSink::append_field(std::string_view key, std::int64_t value) noexcept {
    begin_field(key);
    append_signed(value);
}

void ErrorTextSink::append_field(std::string_view key, std::uint32_t value) noexcept {
    append_field(key, static_cast<std::uint64_t>(value));
}

void ErrorTextSink::append_field(std::string_view key, std::int32_t value) noexcept {
    append_field(key, static_cast<std::int64_t>(value));
}

void ErrorTextSink::append_field(std::string_view key, bool value) noexcept {
    begin_field(key);
    append(value ? "true" : "false");
}

void ErrorTextSink::append_hex_field(std::string_view key, std::uint64_t value) noexcept {
    begin_field(key);
    char text[18] = {};
    const auto result = std::to_chars(text, text + sizeof(text), value, 16);
    if (result.ec == std::errc{})
        append(std::string_view(text, static_cast<std::size_t>(result.ptr - text)));
    else
        append("?");
}

void ErrorTextSink::append_float_field(std::string_view key, float value) noexcept {
    begin_field(key);
    char text[32] = {};
    const auto result =
        std::to_chars(text, text + sizeof(text), value, std::chars_format::fixed, 3);
    if (result.ec == std::errc{})
        append(std::string_view(text, static_cast<std::size_t>(result.ptr - text)));
    else
        append("?");
}

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
    static std::string_view operation_name(Operation operation) noexcept {
        switch (operation) {
        case Operation::kConfigValidate: return "config_validate";
        case Operation::kMediaCreate: return "media_create";
        case Operation::kMediaDestroy: return "media_destroy";
        case Operation::kCreate: return "create";
        case Operation::kInitialize: return "init";
        case Operation::kStart: return "start";
        case Operation::kAcquire: return "acquire";
        case Operation::kRelease: return "release";
        case Operation::kStop: return "stop";
        case Operation::kDeinitialize: return "deinit";
        case Operation::kDestroy: return "destroy";
        case Operation::kGetState: return "get_state";
        case Operation::kGetCaps: return "get_caps";
        case Operation::kExposureRange: return "exposure_range";
        case Operation::kSetExposure: return "set_exposure";
        case Operation::kUnknown: break;
        }
        return "unknown";
    }

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

ErrorKind CameraError::classify(std::int32_t code) noexcept {
    if (code < 0 && code >= -DART_CAMERA_CODE_ERRNO_MAX)
        return ErrorKind::kPlatform;
    if (code <= DART_CAMERA_CODE_VENDOR_MAX)
        return ErrorKind::kHardware;

    switch (code) {
    case DART_CAMERA_E_INVALID_ARGUMENT:
    case DART_CAMERA_E_CONFIG_VERSION:
    case DART_CAMERA_E_CONFIG_INVALID:
    case DART_CAMERA_E_LEASE_ABI_MISMATCH: return ErrorKind::kArgument;
    case DART_CAMERA_E_INVALID_STATE: return ErrorKind::kState;
    case DART_CAMERA_E_BUSY:
    case DART_CAMERA_E_LEASE_EXHAUSTED: return ErrorKind::kBusy;
    case DART_CAMERA_E_NOT_SUPPORTED: return ErrorKind::kUnsupported;
    case DART_CAMERA_E_OUT_OF_RANGE: return ErrorKind::kOutOfRange;
    case DART_CAMERA_E_NO_MEMORY: return ErrorKind::kNoMemory;
    case DART_CAMERA_E_STALE_LEASE: return ErrorKind::kStaleLease;
    case DART_CAMERA_E_CACHE_INVALIDATE_FAILED: return ErrorKind::kCacheSync;
    case DART_CAMERA_E_BLOCK_RELEASE_FAILED: return ErrorKind::kRelease;
    case DART_CAMERA_E_CLOCK_FAILED: return ErrorKind::kPlatform;
    case DART_CAMERA_E_EXPOSURE_RANGE_FAILED:
    case DART_CAMERA_E_EXPOSURE_WRITE_FAILED: return ErrorKind::kExposure;
    default: return ErrorKind::kHardware;
    }
}

CameraError::Description CameraError::description() const noexcept {
    switch (kind_) {
    case ErrorKind::kArgument:
        return {"camera.argument", "invalid camera argument or configuration"};
    case ErrorKind::kState:
        return {"camera.state", "camera operation is invalid in the current state"};
    case ErrorKind::kBusy: return {"camera.busy", "camera resource is still owned"};
    case ErrorKind::kUnsupported:
        return {"camera.unsupported", "camera operation is not supported"};
    case ErrorKind::kOutOfRange:
        return {"camera.range", "camera value is outside the supported range"};
    case ErrorKind::kNoMemory: return {"camera.memory", "camera allocation failed"};
    case ErrorKind::kStaleLease:
        return {"camera.lease", "frame lease is stale or belongs to another camera"};
    case ErrorKind::kCacheSync: return {"camera.cache", "frame cache invalidation failed"};
    case ErrorKind::kRelease: return {"camera.release", "camera resource release failed"};
    case ErrorKind::kPlatform: return {"camera.platform", "platform operation failed"};
    case ErrorKind::kExposure: return {"camera.exposure", "exposure operation failed"};
    case ErrorKind::kHardware:
        switch (code_) {
        case DART_CAMERA_E_SENSOR_MODE_MISMATCH:
            return {"camera.sensor_mode", "sensor did not provide the exact requested mode"};
        case DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH:
        case DART_CAMERA_E_FRAME_POOL_MISMATCH:
        case DART_CAMERA_E_FRAME_NOT_MAPPED:
            return {"camera.frame_geometry", "frame geometry or pool ownership did not match"};
        default: return {"camera.hardware", "camera hardware operation failed"};
        }
    }
    return {"camera.hardware", "camera hardware operation failed"};
}

void CameraError::append_context(ErrorTextSink& sink) const noexcept {
    sink.append_field("code", static_cast<std::int64_t>(code_));
    sink.append_field("op", operation_name(context_.operation));
    if (context_.native_hint != 0) {
        sink.append_field("native_hint", static_cast<std::int64_t>(context_.native_hint));
    }
    if (context_.requested != 0)
        sink.append_field("requested", context_.requested);
    if (context_.observed != 0)
        sink.append_field("observed", context_.observed);
    if (context_.state != 0 || context_.required_state != 0) {
        sink.append_field("state", context_.state);
        sink.append_field("required_state", context_.required_state);
    }
    if (context_.lease_token != 0)
        sink.append_field("lease_token", context_.lease_token);
    if (context_.lease_generation != 0)
        sink.append_field("lease_generation", context_.lease_generation);
    if (context_.lease_owner != 0)
        sink.append_hex_field("lease_owner", context_.lease_owner);
    if (context_.rollback_failed)
        sink.append_field("rollback_failed", true);
    if (context_.previous_release_code != 0)
        sink.append_field(
            "previous_release_code", static_cast<std::int64_t>(context_.previous_release_code));

    switch (kind_) {
    case ErrorKind::kPlatform:
        if (code_ < 0)
            sink.append_field("errno", -static_cast<std::int64_t>(code_));
        break;
    case ErrorKind::kExposure:
        if (context_.has_exposure) {
            sink.append_field("requested_lines", context_.requested_lines);
            sink.append_field("min_lines", context_.min_lines);
            sink.append_field("max_lines", context_.max_lines);
            sink.append_field("applied_lines", context_.applied_lines);
        }
        break;
    case ErrorKind::kCacheSync:
        sink.append_hex_field("phys", context_.physical_address);
        sink.append_hex_field("mapped", static_cast<std::uint64_t>(context_.mapped_address));
        sink.append_field("length", context_.length);
        break;
    case ErrorKind::kHardware:
        switch (code_) {
        case DART_CAMERA_E_SENSOR_MODE_MISMATCH:
            if (context_.has_sensor_mode) {
                const auto& requested = context_.requested_sensor;
                const auto& observed = context_.observed_sensor;
                sink.append_field("req_csi", requested.csi_port);
                sink.append_field("req_w", requested.width);
                sink.append_field("req_h", requested.height);
                sink.append_field("req_fps", requested.fps);
                sink.append_field("obs_csi", observed.csi_port);
                sink.append_field("obs_w", observed.width);
                sink.append_field("obs_h", observed.height);
                sink.append_field("obs_fps", observed.fps);
            }
            break;
        case DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH:
        case DART_CAMERA_E_FRAME_POOL_MISMATCH:
        case DART_CAMERA_E_FRAME_NOT_MAPPED:
            if (context_.has_frame_geometry) {
                const auto& frame = context_.frame_geometry;
                sink.append_field("width", frame.width);
                sink.append_field("height", frame.height);
                sink.append_field("stride", frame.stride);
                sink.append_field("format", frame.format);
                sink.append_field("pool_id", frame.pool_id);
                sink.append_hex_field("phys", frame.physical_address);
                sink.append_field("length", frame.length);
                sink.append_field("block_bytes", frame.block_bytes);
            }
            break;
        default:
            if (code_ <= DART_CAMERA_CODE_VENDOR_MAX) {
                const auto bits = static_cast<std::uint32_t>(code_);
                sink.append_field("module", (bits >> 16U) & 0xffU);
                sink.append_field("level", (bits >> 13U) & 0x7U);
                sink.append_field("errid", bits & 0x1fffU);
            }
            break;
        }
        break;
    default: break;
    }
}

static_assert(std::is_trivially_copyable_v<CameraError>);
static_assert(std::is_standard_layout_v<CameraError>);
static_assert(
    DART_CAMERA_E_SITE_CODE_MAX < DART_CAMERA_CODE_ERRNO_MAX,
    "site code range must remain below errno range");
static_assert(
    DART_CAMERA_CODE_VENDOR_MAX < -DART_CAMERA_CODE_ERRNO_MAX,
    "vendor code range must remain below errno range");
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
