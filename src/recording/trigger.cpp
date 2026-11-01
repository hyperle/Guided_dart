#include "src/recording/trigger.hpp"

#include <cmath>
#include <cstdint>

#include "imu/sample.hpp"
#include "system/write_control.hpp"

namespace dart::app {

void RecordingTrigger::observe(const imu::Sample& sample, void* context) noexcept {
    static_cast<RecordingTrigger*>(context)->update(sample);
}

void RecordingTrigger::update(const imu::Sample& sample) noexcept {
    const auto now = sample.timestamp_ns;
    if (triggered_ns_ != 0) {
        if (duration_ns_ != 0 && now >= triggered_ns_ && now - triggered_ns_ >= duration_ns_)
            writes_.request_recording(false);
        return;
    }
    const auto& a = sample.acceleration_m_s2;
    const double norm = std::sqrt(double(a.x) * a.x + double(a.y) * a.y + double(a.z) * a.z);
    const bool continuous = previous_ns_ != 0 && now > previous_ns_
                         && now - previous_ns_ <= 5'000'000; // At most five 1 kHz sample periods.
    previous_ns_ = now;
    if (!continuous)
        above_ns_ = 0;
    if (threshold_ <= 0 || hold_ns_ == 0 || now == 0 || !std::isfinite(norm)
        || imu::Sample::has_flag(sample.flags, imu::SampleFlag::kAccelerometerSaturated)
        || norm < threshold_) {
        above_ns_ = 0;
        return;
    }
    if (above_ns_ == 0)
        above_ns_ = now;
    if (now - above_ns_ >= hold_ns_) {
        triggered_ns_ = now;
        writes_.request_recording(true, now);
    }
}

} // namespace dart::app
