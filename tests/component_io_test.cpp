#include "runtime/component.hpp"

#include <cassert>
#include <type_traits>

#include "src/control/blackboard.hpp"

namespace dart::tests {

class ComponentIoTest final {
    using Target = detection::Point;
    using TargetChannel = control::AtomicValue<Target>;
    using Estimate = imu::Estimator::Estimate;
    using ImuChannel = control::Latest<Estimate>;

    struct Inputs {
        executor::Input<Target> target;
        executor::Input<ImuChannel::Snapshot> imu;
    };

    struct Outputs {
        executor::Output<Target> target;
        executor::Output<Estimate> imu;
    };

    struct FallibleChannel {
        std::expected<void, executor::Error> write(const Target&);
    };

    static_assert(std::is_constructible_v<executor::Input<Target>, const TargetChannel&>);
    static_assert(!std::is_constructible_v<executor::Input<Target>, TargetChannel&&>);
    static_assert(!std::is_constructible_v<executor::Input<Target>, ImuChannel&>);
    static_assert(!std::is_default_constructible_v<executor::Input<Target>>);
    static_assert(!std::is_constructible_v<executor::Output<Target>, const TargetChannel&>);
    static_assert(!std::is_constructible_v<executor::Output<Target>, FallibleChannel&>);
    static_assert(!std::is_default_constructible_v<executor::Output<Target>>);

public:
    static void run() {
        TargetChannel target{Target{-1.0F, -1.0F}};
        ImuChannel imu;
        const Inputs inputs{executor::Input<Target>{target},
            executor::Input<ImuChannel::Snapshot>{imu}};
        const Outputs outputs{executor::Output<Target>{target}, executor::Output<Estimate>{imu}};

        // Different publication rates share the same channels without a refresh pass.
        for (std::uint64_t tick = 0; tick < 12; ++tick) {
            Estimate estimate{};
            estimate.sample.timestamp_ns = (tick + 1) * 1'000'000;
            outputs.imu.publish(estimate);
            if (tick % 4 == 0) {
                const auto coordinate = static_cast<float>(tick);
                outputs.target.publish(Target{coordinate, -coordinate});
            }

            const auto observed_target = inputs.target.read();
            const auto observed_imu = inputs.imu.read();
            const auto expected_coordinate = static_cast<float>(tick / 4 * 4);
            assert(observed_target.x == expected_coordinate);
            assert(observed_target.y == -expected_coordinate);
            assert(observed_imu.value.sample.timestamp_ns == estimate.sample.timestamp_ns);
            assert(observed_imu.sequence == tick + 1);
            const auto repeated_imu = inputs.imu.read();
            assert(repeated_imu.sequence == observed_imu.sequence);
            assert(repeated_imu.published_at == observed_imu.published_at);
        }

        // A copied port keeps its channel binding after the original port is destroyed.
        const auto copied_input = copy_input(target);
        const auto copied_output = copy_output(target);
        copied_output.publish(Target{100.0F, 200.0F});
        assert(copied_input.read().x == 100.0F);
        assert(inputs.target.read().y == 200.0F);

        // The existing executor channel writes by value; blackboard channels use const&.
        executor::AtomicValue<std::uint64_t> counter;
        executor::Output<std::uint64_t>{counter}.publish(42);
        assert(executor::Input<std::uint64_t>{counter}.read() == 42);
    }

private:
    static executor::Input<Target> copy_input(TargetChannel& channel) {
        executor::Input<Target> original{channel};
        return executor::Input<Target>{original};
    }

    static executor::Output<Target> copy_output(TargetChannel& channel) {
        executor::Output<Target> original{channel};
        return executor::Output<Target>{original};
    }
};

} // namespace dart::tests

int main() {
    dart::tests::ComponentIoTest::run();
}
