#pragma once

#include <cstdint>

#include "imu/sample.hpp"
#include "system/write_control.hpp"

namespace dart::app {
// Optional recording policy. Only the IMU sampling thread touches this state;
// it publishes a memory request and never opens files or waits for storage.
class RecordingTrigger final {
public:
    RecordingTrigger(
        platform::WriteControl& writes, float acceleration_m_s2, std::uint32_t hold_ms,
        std::uint32_t duration_ms = 0) noexcept
        : writes_(writes)
        , threshold_(acceleration_m_s2)
        , hold_ns_(static_cast<std::uint64_t>(hold_ms) * 1'000'000)
        , duration_ns_(static_cast<std::uint64_t>(duration_ms) * 1'000'000) {}

    static void observe(const imu::Sample& sample, void* context) noexcept;
    void update(const imu::Sample& sample) noexcept;

private:
    platform::WriteControl& writes_;
    float threshold_;
    std::uint64_t hold_ns_, duration_ns_;
    std::uint64_t previous_ns_ = 0, above_ns_ = 0, triggered_ns_ = 0;
};

} // namespace dart::app
