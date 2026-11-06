#include <cassert>
#include <cmath>
#include <numbers>

#include "src/control/pose_control.hpp"
#include "imu/estimator.hpp"

namespace dart::tests {

class PoseControlTest final {
public:
    static void run() {
        control::Pose pose;
        imu::Estimator::Estimate estimate{};

        // Roll is independent of velocity; readiness flags do not gate this calculation.
        estimate.attitude_rpy_rad = {0.4F, 0.0F, 0.0F};
        estimate.velocity_m_s = {13.0F, 0.0F, 0.0F};
        pose.update(estimate.attitude_rpy_rad.x, estimate.attitude_rpy_rad.y,
                    estimate.attitude_rpy_rad.z, estimate.velocity_m_s.x,
                    estimate.velocity_m_s.y, estimate.velocity_m_s.z);
        assert(near(pose.roll(), 0.4));
        assert(near(pose.angle_of_attack(), 0.0));
        assert(near(pose.angle_of_yaw(), 0.0));
        assert(near(pose.speed(), 13.0));

        // The 3-4-12 velocity has magnitude 13 and horizontal magnitude 5.
        estimate.attitude_rpy_rad = {-0.25F, 0.8F, 1.2F};
        estimate.velocity_m_s = {3.0F, 4.0F, -12.0F};
        pose.update(estimate.attitude_rpy_rad.x, estimate.attitude_rpy_rad.y,
                    estimate.attitude_rpy_rad.z, estimate.velocity_m_s.x,
                    estimate.velocity_m_s.y, estimate.velocity_m_s.z);
        assert(near(pose.roll(), -0.25));
        assert(near(pose.angle_of_attack(), 0.8 - std::asin(12.0 / 13.0)));
        assert(near(pose.angle_of_yaw(), 1.2 - std::acos(3.0 / 5.0)));
        assert(near(pose.speed(), 13.0));

        // A velocity along the heading gives zero pitch/yaw difference even with roll.
        constexpr float pi = std::numbers::pi_v<float>;
        estimate.attitude_rpy_rad = {0.7F, pi / 6.0F, pi / 3.0F};
        estimate.velocity_m_s = {std::sqrt(3.0F), 3.0F, -2.0F};
        pose.update(estimate.attitude_rpy_rad.x, estimate.attitude_rpy_rad.y,
                    estimate.attitude_rpy_rad.z, estimate.velocity_m_s.x,
                    estimate.velocity_m_s.y, estimate.velocity_m_s.z);
        assert(near(pose.roll(), 0.7));
        assert(near(pose.angle_of_attack(), 0.0));
        assert(near(pose.angle_of_yaw(), 0.0));
        assert(near(pose.speed(), 4.0));

        // Reverse x motion must preserve the yaw quadrant.
        estimate.attitude_rpy_rad = {};
        estimate.velocity_m_s = {-3.0F, 3.0F, 0.0F};
        pose.update(estimate.attitude_rpy_rad.x, estimate.attitude_rpy_rad.y,
                    estimate.attitude_rpy_rad.z, estimate.velocity_m_s.x,
                    estimate.velocity_m_s.y, estimate.velocity_m_s.z);
        assert(near(pose.angle_of_yaw(), -3.0 * std::numbers::pi / 4.0));
        estimate.velocity_m_s.y = -3.0F;
        pose.update(estimate.attitude_rpy_rad.x, estimate.attitude_rpy_rad.y,
                    estimate.attitude_rpy_rad.z, estimate.velocity_m_s.x,
                    estimate.velocity_m_s.y, estimate.velocity_m_s.z);
        assert(near(pose.angle_of_yaw(), 3.0 * std::numbers::pi / 4.0));
    }

private:
    static bool near(double actual, double expected) {
        return std::abs(actual - expected) < 1.0e-6;
    }
};

} // namespace dart::tests

int main() { dart::tests::PoseControlTest::run(); }
