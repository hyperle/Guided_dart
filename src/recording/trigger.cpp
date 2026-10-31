module;
#include <cmath>
#include <cstdint>

export module dart.app.recording.trigger;

import dart.imu.sample;
import dart.system.write_control;

export namespace dart::app {

// Optional recording policy. Only the IMU sampling thread touches this state;
// it publishes a memory request and never opens files or waits for storage.
class RecordingTrigger final {
public:
    RecordingTrigger(
        platform::WriteControl& writes, float acceleration_m_s2, std::uint32_t hold_ms,
        std::uint32_t duration_ms = 0) noexcept
        : writes_(writes), threshold_(acceleration_m_s2),
          hold_ns_(static_cast<std::uint64_t>(hold_ms) * 1'000'000),
          duration_ns_(static_cast<std::uint64_t>(duration_ms) * 1'000'000) {}

    static void observe(const imu::Sample& sample, void* context) noexcept {
        static_cast<RecordingTrigger*>(context)->update(sample);
    }
    void update(const imu::Sample& sample) noexcept {
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

private:
    platform::WriteControl& writes_;
    float threshold_;
    std::uint64_t hold_ns_, duration_ns_;
    std::uint64_t previous_ns_ = 0, above_ns_ = 0, triggered_ns_ = 0;
};

} // namespace dart::app
