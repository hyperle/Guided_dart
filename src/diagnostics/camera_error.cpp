module;
#include <string_view>

export module dart.app.diagnostics.camera_error;

import dart.camera.error;
import dart.logging.logger;

namespace dart::app {

std::string_view camera_operation_text(camera::Operation operation) noexcept {
    switch (operation) {
    case camera::Operation::kUnknown: return "unknown";
    case camera::Operation::kConfigValidate: return "config_validate";
    case camera::Operation::kMediaCreate: return "media_create";
    case camera::Operation::kMediaDestroy: return "media_destroy";
    case camera::Operation::kCreate: return "create";
    case camera::Operation::kInitialize: return "initialize";
    case camera::Operation::kStart: return "start";
    case camera::Operation::kAcquire: return "acquire";
    case camera::Operation::kRelease: return "release";
    case camera::Operation::kStop: return "stop";
    case camera::Operation::kDeinitialize: return "deinitialize";
    case camera::Operation::kDestroy: return "destroy";
    case camera::Operation::kGetState: return "get_state";
    case camera::Operation::kGetCaps: return "get_caps";
    case camera::Operation::kExposureRange: return "exposure_range";
    case camera::Operation::kSetExposure: return "set_exposure";
    }
    return "unknown";
}

/* Recovery belongs to the caller. This function only formats one observation. */
export void log_camera_error(
    logging::Logger& log, const char* op, const char* action,
    const camera::CameraError& error) noexcept {
    const auto& context = error.context();
    const auto operation = camera_operation_text(context.operation);
    log.line(
        "level=ERROR module=camera op=%s code=%.*s native=%d detail=\"%.*s\" "
        "ctx_op=%.*s requested=%u observed=%u state=%u req_state=%u action=%s\n",
        op, static_cast<int>(error.category().size()), error.category().data(), error.code(),
        static_cast<int>(error.message().size()), error.message().data(),
        static_cast<int>(operation.size()), operation.data(), context.requested, context.observed,
        context.state, context.required_state, action);
}

} // namespace dart::app
