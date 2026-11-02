#pragma once

#include "runtime/imu/session.hpp"
#include "src/detector/result.hpp"
#include "pose_control.hpp"

namespace dart::control::predictor {
class Preditor final {
public:
    explicit Preditor(const imu::Session& session, const detection::DetectResult& result)
        : imu_session_(session)
        , result_(result) {}


    struct Increment {
        double vertical_increment;
        double horizontal_increment;
    };

    Increment predictor_update () {
        if (!imu_session_.imu_working()) {
            return no_prediction(result_);
        }
        

    }

    static Increment no_prediction(const detection::DetectResult& result) {
        const auto target = result.target.center;
        auto v_i = target.y - 240;
        auto h_i = target.x - 320;
        return Increment{v_i, h_i};
    }

private:
    const imu::Session& imu_session_;
    const detection::DetectResult& result_;
};
} // namespace dart::control::predictor

