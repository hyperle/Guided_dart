#include "src/control/servo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "src/control/predictor.hpp"

namespace dart::control {

bool ServoConfig::valid() const noexcept {
    if (!std::isfinite(max_delta_us) || max_delta_us < 0.0F || target_timeout.count() <= 0
        || !std::isfinite(aim.x) || !std::isfinite(aim.y) || aim.x < 0.0F || aim.y < 0.0F)
        return false;
    for (const auto& channel : channels) {
        const double initial = channel.initial_pulse_us;
        const double limit = max_delta_us;
        if (!std::isfinite(initial) || !std::isfinite(channel.horizontal_us_per_pixel)
            || !std::isfinite(channel.vertical_us_per_pixel)
            || (initial - limit) * 1000.0 < 1.0
            || (initial + limit) * 1000.0 > pwm::kPeriodNs)
            return false;
    }
    return true;
}

class Servo final {
public:
    explicit Servo(ServoConfig::Channel config) noexcept : config_(config) {}
    double output(
        double vertical, double horizontal, double roll_correction, double target_limit) const noexcept {
        const double target_increment = vertical * config_.vertical_us_per_pixel
                                      + horizontal * config_.horizontal_us_per_pixel;
        // The configured limit applies only to image-target correction. Roll
        // compensation is a separate control term and intentionally remains
        // outside this clamp.
        const double limited_target = std::clamp(target_increment, -target_limit, target_limit);
        return config_.initial_pulse_us + limited_target + roll_correction;
    }
private:
    ServoConfig::Channel config_;
};

class DutyMapping final : public executor::Component {
public:
    DutyMapping(const Spec& spec, const ServoConfig& config, BlackBoard& blackboard,
                pwm::IPwm& pwm, ServoStatus& status)
        : Component(spec), config_(config), blackboard_(blackboard), pwm_(pwm), status_(status),
          servos_{Servo{config.channels[0]}, Servo{config.channels[1]},
                  Servo{config.channels[2]}, Servo{config.channels[3]}} {}
    ~DutyMapping() override { (void)shutingdowm(); }

private:
    std::expected<void, executor::Error> initializing() noexcept override {
        status_.writes = status_.stale_targets = 0;
        status_.native_error = 0;
        status_.operation = "none";
        for (std::size_t i = 0; i < output_.size(); ++i)
            output_[i] = config_.channels[i].initial_pulse_us;
        for (std::size_t i = 0; i < output_.size(); ++i)
            status_.pulse_us[i] = output_[i];
        if (const auto started = pwm_.start(output_); !started) {
            record_error(started.error());
            (void)pwm_.stop();
            return std::unexpected(executor::Error::output_failed);
        }
        started_ = true;
        status_.active = true;
        return {};
    }

    std::expected<void, executor::Error> capture_inputs() noexcept override {
        // Session::current() is a cached snapshot; acquisition remains in the IMU service.
        blackboard_.update();
        use_imu_ = blackboard_.imu_working();
        inputs_.imu_snapshot = blackboard_.read(blackboard_.imu_state);
        // Acquire the publication time before the value: an old coordinate
        // must never be paired with a newer publication time.
        const auto published_at = blackboard_.target.published_at();
        inputs_.target = blackboard_.read(blackboard_.target);
        if (std::chrono::steady_clock::now() - published_at
            >= config_.target_timeout) {
            inputs_.target = {-1.0F, -1.0F};
            ++status_.stale_targets;
        }
        return {};
    }

    std::expected<void, executor::Error> update() noexcept override {
        const auto& estimate = inputs_.imu_snapshot.value;
        const auto& target = inputs_.target;
        pose_.update(estimate.attitude_rpy_rad.x, estimate.attitude_rpy_rad.y,
                     estimate.attitude_rpy_rad.z, estimate.velocity_m_s.x,
                     estimate.velocity_m_s.y, estimate.velocity_m_s.z);
        controller_.update(pose_);
        const auto increment = use_imu_
            ? predictor_.update(target.x, target.y, pose_)
            : Preditor::no_prediction(target.x, target.y, config_.aim.x, config_.aim.y);
        for (std::size_t i = 0; i < output_.size(); ++i)
            output_[i] = static_cast<float>(servos_[i].output(
                increment.vertical_increment, increment.horizontal_increment,
                controller_.get_stability(), config_.max_delta_us));
        return {};
    }

    std::expected<void, executor::Error> publish_outputs() noexcept override {
        if (const auto written = pwm_.write(output_); !written) {
            record_error(written.error());
            return std::unexpected(executor::Error::output_failed);
        }
        for (std::size_t i = 0; i < output_.size(); ++i)
            status_.pulse_us[i] = output_[i];
        ++status_.writes;
        return {};
    }

    std::expected<void, executor::Error> shutingdowm() noexcept override {
        if (!started_)
            return {};
        started_ = false;
        status_.active = false;
        if (const auto stopped = pwm_.stop(); !stopped) {
            record_error(stopped.error());
            return std::unexpected(executor::Error::output_failed);
        }
        return {};
    }
    void record_error(const pwm::Error& error) noexcept {
        status_.operation = error.operation;
        status_.channel = error.channel;
        status_.native_error = error.code;
    }

    struct Inputs {
        Latest<imu::Estimator::Estimate>::Snapshot imu_snapshot{};
        detection::Point target{-1.0F, -1.0F};
    };
    ServoConfig config_;
    BlackBoard& blackboard_;
    pwm::IPwm& pwm_;
    ServoStatus& status_;
    std::array<Servo, pwm::kChannelCount> servos_;
    Preditor predictor_;
    Pose pose_;
    PoseControl controller_;
    Inputs inputs_{};
    pwm::Pulses output_{};
    bool use_imu_ = false, started_ = false;
};

std::expected<std::unique_ptr<executor::Component>, executor::Error> ServoFactory::create(
    const executor::Component::Spec& spec, const ServoConfig& config,
    BlackBoard& blackboard, pwm::IPwm& pwm, ServoStatus& status) noexcept {
    if (!spec.validate() || !config.valid() || !spec.parameter.empty())
        return std::unexpected(executor::Error::invalid_spec);
    return std::unique_ptr<executor::Component>{new DutyMapping{spec, config, blackboard, pwm, status}};
}

} // namespace dart::control
