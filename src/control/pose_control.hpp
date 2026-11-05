#pragma once

#include <cmath>

namespace dart::control {
class Pose final {
public:
    void update(
        double roll_rad, double pitch_rad, double yaw_rad,
        double vx_m_s, double vy_m_s, double vz_m_s) {
        roll_rad_ = roll_rad;

        const double vx = vx_m_s;
        const double vy = vy_m_s;
        const double vz = vz_m_s;
        // Velocity direction uses the same reference frame and ZYX convention as attitude.
        const double velocity_pitch = std::atan2(-vz, std::hypot(vx, vy));
        const double velocity_yaw = std::atan2(vy, vx);
        angle_of_attack_ = pitch_rad - velocity_pitch;
        angle_of_yaw_ = yaw_rad - velocity_yaw;
        speed_ = std::hypot(vx, vy, vz);
    }

    double roll() const {return roll_rad_;}
    double angle_of_attack() const {return angle_of_attack_;}
    double angle_of_yaw() const {return angle_of_yaw_;}
    double speed() const {return speed_;}

private:
    double angle_of_attack_ = 0.0;
    double angle_of_yaw_ = 0.0;
    double speed_ = 0.0;
    double roll_rad_ = 0.0;
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
