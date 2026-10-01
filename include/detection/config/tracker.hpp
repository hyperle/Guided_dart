#pragma once

#include <cstdint>

namespace dart::detection {

struct TrackerConfig {
    uint32_t lost_after = 3;
    uint32_t rescan_after = 6;
    uint32_t hard_reset_after = 36;
};

} // namespace dart::detection
