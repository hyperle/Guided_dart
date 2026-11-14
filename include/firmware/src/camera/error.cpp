#include "camera/error.hpp"

#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>

#include "camera/error_codes.h"

namespace dart::camera {

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

std::string_view CameraError::operation_name(Operation operation) noexcept {
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

ErrorKind CameraError::classify(std::int32_t code) noexcept {
    if (code < 0 && code >= -DART_CAMERA_CODE_ERRNO_MAX)
        return ErrorKind::kPlatform;

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
    if (context_.backend[0] != '\0')
        sink.append_field("backend", std::string_view{context_.backend.data()});
    if (context_.backend_detail[0] != '\0')
        sink.append_field("backend_detail", std::string_view{context_.backend_detail.data()});
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
        default: break;
        }
        break;
    default: break;
    }
}

} // namespace dart::camera
