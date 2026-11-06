#pragma once

#include "pose_control.hpp"

namespace dart::control {
class Preditor final {
public:
    struct Increment {
        double vertical_increment;
        double horizontal_increment;
    };

    Increment update(double target_x, double target_y, const Pose& pose) {
        if (target_x == -1.0F || target_y == -1.0F) {
            return {0.0,0.0};
        }

        previous_increment_ = increment_;
        velocity_ = pose.speed();
        return increment_;
    }

    static Increment no_prediction(
        double target_x, double target_y, double aim_x = 320.0, double aim_y = 240.0) {
        if (target_x == -1.0F || target_y == -1.0F) {
            return {0.0,0.0};
        }

        auto v_i = target_y - aim_y;
        auto h_i = target_x - aim_x;
        return Increment{v_i, h_i};
    }

private:
    Increment previous_increment_{0.0, 0.0};
    Increment increment_{0.0, 0.0};
    double velocity_;
};
} // namespace dart::control

