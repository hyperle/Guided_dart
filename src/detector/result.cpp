module;
#include <chrono>
#include <cstdint>

export module dart.detection.result;

export import dart.detection.geometry;

export namespace dart::detection {

class Target {
public:
    bool valid = false;
    Point center{};
    Rect bounds{};
    std::uint32_t area = 0;
    float radius = 0.0F;
    float quality = 0.0F;
    bool clipped = false;
    float contrast = 0.0F;
    float snr = 0.0F;
    float stability = 0.0F;
    float brightness_ratio = 0.0F;
    float gap_contrast = 0.0F;
    bool stability_checked = false;
};

class FrameMetadata {
public:
    std::uint64_t frame_id = 0;
    std::uint64_t source_pts = 0;
    std::chrono::microseconds received{};
    bool source_pts_timebase_validated = false;
};

enum class DetectionPhase : std::uint8_t { kDetect, kTrack };

class ScanMetadata {
public:
    Rect search_window{};
    bool full_frame = false;
    bool rescanned = false;
    std::uint32_t candidates = 0;
    std::uint32_t rejected_contrast = 0, rejected_snr = 0, rejected_stability = 0;
    DetectionPhase phase = DetectionPhase::kDetect;
    bool full_validation = false;
    std::uint32_t stability_checks = 0;
};

enum class TargetKind : std::uint8_t { kNone, kLight, kArmor };

class DetectResult {
public:
    TargetKind kind = TargetKind::kNone;
    Target target{}; // Armor center when available, otherwise light center.
    Target light{};
    Target armor{};
    FrameMetadata source{};
    ScanMetadata scan{};
    std::uint32_t missed_frames = 0;
};

} // namespace dart::detection
