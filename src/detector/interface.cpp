module;
#include <expected>

export module dart.detection.interface;

export import dart.camera.frame_view;
export import dart.detection.result;

export namespace dart::detection {

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
