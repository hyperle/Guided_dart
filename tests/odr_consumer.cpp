#include "tests/odr_consumer.hpp"

#include "camera/frame_lease.hpp"
#include "imu/estimator.hpp"
#include "pwm/session.hpp"
#include "src/recorder/record.hpp"
#include "src/control/servo.hpp"
#include "src/detector/detect.hpp"

namespace dart::tests {

// Independent callers share these compiled definitions across ordinary translation units.
// The main tests call this method so the archive must participate in the link.
bool OdrConsumer::check() noexcept {
    imu::Estimator estimator;
    detection::Detect detector;
    recorder::RecorderStatus recording{};
    control::ServoConfig config;
    return !estimator.current().has_reference_frame && detector.Prepare({64, 64, camera::PixelFormat::kRgb565, 1.0F})
        && !recording.started() && config.valid()
        && pwm::valid_pulses(pwm::Pulses{config.channels[0].initial_pulse_us, config.channels[1].initial_pulse_us,
                                           config.channels[2].initial_pulse_us, config.channels[3].initial_pulse_us});
}

} // namespace dart::tests
