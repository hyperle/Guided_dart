#pragma once

#include <cstdint>

#include "core/frame.hpp"
#include "detection/domain/geometry.hpp"

namespace dart::detection {

enum class TrackState : uint8_t {
    Startup  = 0,
    Tracking = 1,
    Lost     = 2,
};

inline const char *to_string(TrackState s) {
    switch (s) {
    case TrackState::Startup:
        return "启动";
    case TrackState::Tracking:
        return "跟踪";
    case TrackState::Lost:
        return "丢失";
    }
    return "?";
}

struct TrackOutput {
    TrackState state = TrackState::Startup;
    bool       found = false;
    float      cx = 0.0f;
    float      cy = 0.0f;
    float      radius = 0.0f;
    float      circularity = -1.0f;
    RoiState   roi = RoiState::NoTargetInRoi;
    RoiWindow  window{};
    bool       kf_rejected = false;
};

struct TrackerCounters {
    uint64_t confirm_gained = 0;
    uint64_t to_lost = 0;
    uint64_t to_tracking = 0;
    uint64_t to_startup = 0;
    uint64_t kf_reject = 0;
    uint64_t full_scans = 0;
    uint64_t roi_scans = 0;
    uint64_t confirm_frames = 0;
    uint64_t confirm_reject = 0;
    uint64_t confirm_border_skip = 0;
    uint64_t out_of_roi = 0;
    uint64_t reacquire_soft = 0;
    uint64_t full_confirm = 0;
    uint64_t reacquire_hard = 0;
};

} // namespace dart::detection
