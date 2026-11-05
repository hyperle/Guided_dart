#pragma once

#include <expected>

#include "camera/frame_view.hpp"
#include "src/detector/result.hpp"

namespace dart::detection {
enum class DetectionError : int {
    kInvalidConfig = 1,
    kNoMemory,
    kNotPrepared,
    kInvalidFrame,
    kInvalidWindow,
    kWorkspaceExhausted,
    kOutOfOrderFrame,
};

class IDetector {
public:
    virtual ~IDetector() = default;
    virtual std::expected<DetectResult, DetectionError>
        Detect(const camera::FrameView& frame) noexcept = 0;
    virtual void Reset() noexcept = 0;
};

} // namespace dart::detection
