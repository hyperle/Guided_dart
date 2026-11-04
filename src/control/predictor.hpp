#pragma once

#include "src/detector/result.hpp"
#include "pose_control.hpp"
#include "blackboard.hpp"

namespace dart::control {
class Preditor final {
public:
    explicit Preditor(BlackBoard& blackboard)
        : blackboard_(blackboard) {}


    struct Increment {
        double vertical_increment;
        double horizontal_increment;
    };

    void start_up() noexcept {
        use_imu_ = blackboard_.imu_working();
    }

    Increment update (const detection::DetectResult& result) {
        const auto target = result.target.center;

        if (target.x == -1.0F || target.y == -1.0F) {
            return {0.0,0.0};
        }

        if (!use_imu_) {
            return no_prediction(result);
        }

        const auto snapshot = blackboard_.read(blackboard_.imu_state);
        pose.update(snapshot.value);
        
        previous_increment_ = increment_;
        velocity_ = pose.speed();
        return increment_;
    }

    static Increment no_prediction(const detection::DetectResult& result) {
        const auto target = result.target.center;
        auto v_i = target.y - 240;
        auto h_i = target.x - 320;
        return Increment{v_i, h_i};
    }

private:
    BlackBoard& blackboard_;
    Increment previous_increment_{0.0, 0.0};
    Increment increment_{0.0, 0.0};
    Pose pose;
    
    bool use_imu_ = false;
    double velocity_;
};
} // namespace dart::control

