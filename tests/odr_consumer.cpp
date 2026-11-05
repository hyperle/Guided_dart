#include "tests/odr_consumer.hpp"

#include "camera/frame_lease.hpp"
#include "imu/estimator.hpp"
#include "pwm/session.hpp"
#include "recorder/recorder.hpp"
#include "src/control/servo_mapping.hpp"
#include "src/detector/roi.hpp"

namespace dart::tests {

// Independent callers share these compiled definitions across ordinary translation units.
// The main tests call this method so the archive must participate in the link.
bool OdrConsumer::check() noexcept {
    imu::Estimator estimator;
    detection::RoiDetector detector;
    recorder::Recorder recording{recorder::Config{}};
    control::ServoMapping mapping;
    return !estimator.current().has_reference_frame && detector.Prepare(64, 64)
        && !recording.started() && mapping.pulses().has_value()
        && pwm::valid_pulses(mapping.pulses()->pulse_us);
}

} // namespace dart::tests
