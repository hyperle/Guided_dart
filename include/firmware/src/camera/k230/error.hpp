#pragma once

#include <cstdint>

#include "camera/error.hpp"
#include "camera/k230/error_codes.h"

namespace dart::camera::k230 {

class Error {
public:
    static CameraError from_code(std::int32_t code, CallContext context) noexcept {
        context.backend = {'k', '2', '3', '0'};
        if (code <= DART_K230_CAMERA_CODE_VENDOR_MAX) {
            const auto bits = static_cast<std::uint32_t>(code);
            ErrorTextSink sink{context.backend_detail.data(), context.backend_detail.size()};
            sink.append_field("module", (bits >> 16U) & 0xffU);
            sink.append_field("level", (bits >> 13U) & 0x7U);
            sink.append_field("errid", bits & 0x1fffU);
            return CameraError{ErrorKind::kHardware, code, context};
        }
        return CameraError{code, context};
    }
};

static_assert(DART_K230_CAMERA_CODE_VENDOR_MAX < -DART_CAMERA_CODE_ERRNO_MAX);

} // namespace dart::camera::k230
