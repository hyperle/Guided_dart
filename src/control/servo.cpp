#include <algorithm>
#include <array>
#include <cmath>

#include "pwm/interface.hpp"
#include "blackboard.hpp"
#include "predictor.hpp"
#include "src/component.hpp"

namespace dart::control {

class Servo final {
public:
    Servo(double initial_pose, std::array<double, 2> servo_direction) 
    : initial_pose_(initial_pose), output_pose_(initial_pose), direction(servo_direction) {}

    void update_pose_output(double vertical_increment, double horizontal_increment, double stability) {
        auto increment =   vertical_increment * direction[0]  + horizontal_increment * direction[1];
        increment = std::clamp(increment, -150.0, 150.0);
        output_pose_= initial_pose_ + increment + stability;
    }


    double output_pose() const {return output_pose_;}
private:
    double initial_pose_;
    double output_pose_;
    std::array<double, 2>direction =  {1.0, 1.0};
};

class DutyMapping final : public executor::Component{
public:
    explicit DutyMapping(const Spec& spec, BlackBoard& blackboard, pwm::IPwm& pwm)
        : Component(spec)
        , blackboard_(blackboard)
        , pwm_(pwm) {}

    void start_up() noexcept {
        use_imu_ = blackboard_.imu_working();
    }

    std::expected<void, executor::Error> update() noexcept override {
        const auto& estimate = inputs_.imu_snapshot.value;
        const auto& target = inputs_.target;

        pose_.update(
            estimate.attitude_rpy_rad.x,
            estimate.attitude_rpy_rad.y,
            estimate.attitude_rpy_rad.z,
            estimate.velocity_m_s.x,
            estimate.velocity_m_s.y,
            estimate.velocity_m_s.z);
        controller_.update(pose_);
        const double stability = controller_.get_stability();

        const auto increment = use_imu_
            ? predictor_.update(target.x, target.y, pose_)
            : Preditor::no_prediction(target.x, target.y);

        servo_1_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);
        servo_2_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);
        servo_3_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);
        servo_4_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);

        output_ = {
            static_cast<float>(servo_2_.output_pose()),
            static_cast<float>(servo_3_.output_pose()),
            static_cast<float>(servo_4_.output_pose()),
            static_cast<float>(servo_1_.output_pose())
        };

        return {};
    } 

private:
    struct Inputs {
        Latest<imu::Estimator::Estimate>::Snapshot imu_snapshot{};
        detection::Point target{-1.0F, -1.0F};
    };

    BlackBoard& blackboard_;
    pwm::IPwm& pwm_;
    Preditor predictor_;
    Pose pose_;
    PoseControl controller_;
    Inputs inputs_{};
    bool use_imu_ = false;
    Servo servo_1_{1475.0, { 1.0,  1.0}};
    Servo servo_2_{1445.0, { 1.0, -1.0}};
    Servo servo_3_{1490.0, {-1.0, -1.0}};
    Servo servo_4_{1150.0, {-1.0,  1.0}};

    pwm::Pulses output_{};

protected:
    std::expected<void, executor::Error> initializing() noexcept override {
        start_up();
        return {};
    }

    std::expected<void, executor::Error> capture_inputs() noexcept override {
        inputs_.imu_snapshot = blackboard_.read(blackboard_.imu_state);
        inputs_.target = blackboard_.read(blackboard_.target);
        return {};
    }

    std::expected<void, executor::Error> publish_outputs() noexcept override {
        const auto written = pwm_.write(output_);
        if (!written) {
            return std::unexpected(executor::Error::output_failed);
        }
        return {};
    }
};

} //namespace dart::controll
