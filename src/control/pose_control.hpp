#pragma once

#include <cmath>

#include "imu/estimator.hpp"

namespace dart::control {
class Pose final {
public:
    void update(const imu::Estimator::Estimate& estimate) {
        attitude_ = estimate.attitude;
        rpy_rad_ = estimate.attitude_rpy_rad;
        angular_velocity_rad_s_ = estimate.angular_velocity_rad_s;
        velocity_m_s_ = estimate.velocity_m_s;

        const double vx = velocity_m_s_.x;
        const double vy = velocity_m_s_.y;
        const double vz = velocity_m_s_.z;
        // Velocity direction uses the same reference frame and ZYX convention as attitude.
        const double velocity_pitch = std::atan2(-vz, std::hypot(vx, vy));
        const double velocity_yaw = std::atan2(vy, vx);
        angle_of_attack_ = rpy_rad_.y - velocity_pitch;
        angle_of_yaw_ = rpy_rad_.z - velocity_yaw;
        speed_ = std::hypot(vx, vy, vz);
    }

    double roll() const {return rpy_rad_.x;}
    double angle_of_attack() const {return angle_of_attack_;}
    double angle_of_yaw() const {return angle_of_yaw_;}
    double speed() const {return speed_;}

private:
    double angle_of_attack_ = 0.0;
    double angle_of_yaw_ = 0.0;
    double speed_ = 0.0;
    imu::Quaternionf attitude_{};
    imu::Vector3f rpy_rad_{};
    imu::Vector3f angular_velocity_rad_s_{};
    imu::Vector3f velocity_m_s_{}; 
};

class PoseControl final {
public:
    static constexpr auto  roll = [](const Pose& pose) {return pose.roll();};

    void update(const Pose& pose) {
        stability_ = roll(pose) * 57.27;
    }

    double get_stability() const {return stability_;}
private:
    double stability_ = 0.0;
};

} // namespace dart::control
