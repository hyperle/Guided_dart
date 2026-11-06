#include "src/control/servo.hpp"
#include "tests/odr_consumer.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <thread>

namespace dart::tests {

class ServoComponentTest final {
    class Pwm final : public pwm::IPwm {
    public:
        std::expected<void, pwm::Error> start(const pwm::Pulses& pulses) noexcept override {
            ++starts; output = pulses;
            if (fail_start) return std::unexpected(pwm::Error{"start", -1, 10});
            return {};
        }
        std::expected<void, pwm::Error> write(const pwm::Pulses& pulses) noexcept override {
            ++writes; output = pulses;
            if (fail_write) return std::unexpected(pwm::Error{"write", 2, 20});
            return {};
        }
        std::expected<void, pwm::Error> stop() noexcept override { ++stops; return {}; }
        pwm::Pulses output{};
        unsigned starts = 0, writes = 0, stops = 0;
        bool fail_start = false, fail_write = false;
    };
public:
    static void run() {
        assert(OdrConsumer::check());
        control::BlackBoard board;
        control::ServoStatus status;
        control::ServoConfig config{};
        config.target_timeout = std::chrono::milliseconds{5};
        const executor::Component::Spec spec{"servo", "servo", 333.0, {}};
        Pwm pwm;
        auto created = control::ServoFactory::create(spec, config, board, pwm, status);
        assert(created && (*created)->initialze());
        const pwm::Pulses initial{1445, 1490, 1150, 1475};
        assert(pwm.output == initial && status.active);
        board.target.write({330, 260});
        assert((*created)->step());
        assert((pwm.output == pwm::Pulses{1455, 1460, 1140, 1505}));
        board.target.write({-1, -1});
        assert((*created)->step() && pwm.output == initial);
        board.target.write({10000, 10000});
        assert((*created)->step());
        assert((pwm.output == pwm::Pulses{1525, 1340, 1070, 1625}));
        board.target.write({330, 260});
        std::this_thread::sleep_for(std::chrono::milliseconds{7});
        assert((*created)->step() && pwm.output == initial && status.stale_targets > 0);
        imu::Estimator::Estimate estimate{};
        estimate.attitude_rpy_rad.x = 10.0F;
        board.imu_state.write(estimate);
        assert((*created)->step());
        for (std::size_t i = 0; i < initial.size(); ++i)
            assert(std::abs(pwm.output[i] - initial[i] - 572.7F) < 0.001F);
        board.target.write({10000, 10000});
        assert((*created)->step());
        assert(std::abs(pwm.output[1] - initial[1] - (-150.0F + 572.7F)) < 0.001F);
        pwm.fail_write = true;
        assert(!(*created)->step());
        assert(status.native_error == 20 && status.channel == 2);
        assert((*created)->shutdown());
        created->reset();
        assert(pwm.starts == 1 && pwm.stops == 1 && !status.active);
        Pwm failed;
        failed.fail_start = true;
        created = control::ServoFactory::create(spec, config, board, failed, status);
        assert(created && !(*created)->initialze());
        created->reset();
        assert(failed.starts == 1 && failed.stops == 1 && failed.writes == 0);
        config.max_delta_us = 1500;
        assert(!control::ServoFactory::create(spec, config, board, pwm, status));
        assert(!control::ServoFactory::create(
            executor::Component::Spec{"servo", "servo", 333.0, {{"unsupported", 1}}},
            control::ServoConfig{}, board, pwm, status));
    }
};

} // namespace dart::tests

int main() { dart::tests::ServoComponentTest::run(); }
