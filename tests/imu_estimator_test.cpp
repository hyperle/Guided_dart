#include <cstdint>

#include <cassert>
#include <cmath>
#include <limits>

#include "imu/estimator.hpp"

namespace dart::tests {

class ImuEstimatorTest final {
public:
    static void run() {
        imu::Estimator estimator;
        imu::Sample sample{};
        sample.acceleration_m_s2.z = 9.80665F;
        sample.timestamp_ns = 1'000'000'000;
        auto unseeded = estimator.update(sample);
        assert(unseeded.sample_accepted && !unseeded.has_reference_frame && !unseeded.integrated);
        assert(estimator.seed(sample, {}).accepted);
        sample.timestamp_ns += 10'000'000;
        auto stationary = estimator.update(sample);
        assert(stationary.integrated && stationary.has_reference_frame);
        assert(std::abs(stationary.velocity_m_s.z) < 1e-6F);
        auto duplicate = estimator.update(sample);
        assert(!duplicate.sample_accepted);
        assert(
            duplicate.quality_events
            & static_cast<std::uint32_t>(imu::Estimator::QualityEvent::kTimestampNonMonotonic));
        sample.timestamp_ns += 10'000'000;
        sample.acceleration_m_s2.x = std::numeric_limits<float>::quiet_NaN();
        assert(!estimator.update(sample).sample_accepted);
        sample.acceleration_m_s2.x = 0;
        sample.timestamp_ns += 500'000'000;
        auto gap = estimator.update(sample);
        assert(gap.sample_accepted && !gap.integrated);
        assert(gap.quality_events & static_cast<std::uint32_t>(imu::Estimator::QualityEvent::kTimestampGap));
        estimator.reset();
        assert(!estimator.current().has_reference_frame);
    }
};

} // namespace dart::tests

int main() { dart::tests::ImuEstimatorTest::run(); }
