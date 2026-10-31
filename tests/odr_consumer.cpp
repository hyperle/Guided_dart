module;

export module dart.tests.odr_consumer;

import dart.camera.frame_lease;
import dart.control.servo_mapping;
import dart.detection.roi;
import dart.imu.estimator;
import dart.pwm.session;
import dart.recorder.recorder;

export namespace dart::tests {

class OdrConsumer final {
public:
    static bool check() noexcept;
};

// Independent callers share these compiled definitions through module imports.
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
