#include <algorithm>
#include <array>
#include <cmath>

#include "pose_control.hpp"
#include "pwm/interface.hpp"
#include "predictor.hpp"

namespace dart::controll::servo {

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

class Duty_mapping final {
public:
    explicit Duty_mapping(dart::control::predictor::Preditor& predictor_)
        : predictor_(predictor_) {}

    std::expected<void, pwm::Error> update(
        dart::pwm::IPwm& pwm, const imu::Estimator::Estimate& estimate) {
        pose_.update(estimate);
        controller_.update(pose_);
        const double stability = controller_.get_stability();

        const auto increment = predictor_.predictor_update();

        servo_1_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);
        servo_2_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);
        servo_3_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);
        servo_4_.update_pose_output(increment.vertical_increment, increment.horizontal_increment, stability);

        dart::pwm::Pulses duty_mapping{
            static_cast<float>(servo_2_.output_pose()),
            static_cast<float>(servo_3_.output_pose()),
            static_cast<float>(servo_4_.output_pose()),
            static_cast<float>(servo_1_.output_pose())
        };

        auto output_ = pwm.write(duty_mapping);
        return output_;
    } 



private:
    dart::control::pose_control::Pose pose_;
    dart::control::pose_control::PoseControl controller_;
    dart::control::predictor::Preditor& predictor_;
    Servo servo_1_{1475.0, { 1.0,  1.0}};
    Servo servo_2_{1445.0, { 1.0, -1.0}};
    Servo servo_3_{1490.0, {-1.0, -1.0}};
    Servo servo_4_{1150.0, {-1.0,  1.0}};
};

} //namespace dart::controll::servo
