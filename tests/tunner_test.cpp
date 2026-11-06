#include "runtime/tunner.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <memory>
#include <thread>

namespace dart::tests {

class TunnerTest final {
    struct State {
        std::atomic<unsigned> initialized{0}, captured{0}, updated{0}, published{0}, stopped{0};
        std::thread::id initialization_thread{}, execution_thread{}, shutdown_thread{};
        bool fail_initialize = false, fail_update = false, fail_shutdown = false;
    };
    class Worker final : public executor::Component {
    public:
        explicit Worker(State& state) : Component(Spec{"test", "test", 1000, {}}), state_(state) {}
    private:
        std::expected<void, executor::Error> initializing() noexcept override {
            ++state_.initialized;
            state_.initialization_thread = std::this_thread::get_id();
            if (state_.fail_initialize) return std::unexpected(executor::Error::invalid_spec);
            return {};
        }
        std::expected<void, executor::Error> capture_inputs() noexcept override {
            ++state_.captured;
            state_.execution_thread = std::this_thread::get_id();
            return {};
        }
        std::expected<void, executor::Error> update() noexcept override {
            ++state_.updated;
            if (state_.fail_update) return std::unexpected(executor::Error::input_failed);
            return {};
        }
        std::expected<void, executor::Error> publish_outputs() noexcept override {
            ++state_.published;
            return {};
        }
        std::expected<void, executor::Error> shutingdowm() noexcept override {
            ++state_.stopped;
            state_.shutdown_thread = std::this_thread::get_id();
            if (state_.fail_shutdown) return std::unexpected(executor::Error::output_failed);
            return {};
        }
        State& state_;
    };
    template <typename Predicate> static void until(Predicate predicate) {
        for (unsigned i = 0; i < 1000; ++i) {
            if (predicate()) return;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(false);
    }
public:
    static void run() {
        const auto owner = std::this_thread::get_id();
        executor::Tunner tunner;
        assert(tunner.finished() && !tunner.error());
        State normal{};
        assert(tunner.start(std::make_unique<Worker>(normal)));
        assert(!tunner.start(std::make_unique<Worker>(normal)));
        until([&] { return normal.published.load() >= 3; });
        tunner.request_stop();
        tunner.join();
        assert(tunner.finished() && !tunner.error());
        assert(normal.initialized == 1 && normal.stopped == 1);
        assert(normal.captured == normal.updated && normal.updated == normal.published);
        assert(normal.initialization_thread == owner && normal.execution_thread != owner);
        assert(normal.execution_thread == normal.shutdown_thread);
        tunner.stop();
        assert(normal.stopped == 1);
        State run_failure{};
        run_failure.fail_update = true;
        run_failure.fail_shutdown = true;
        assert(tunner.start(std::make_unique<Worker>(run_failure)));
        until([&] { return tunner.finished(); });
        assert(tunner.error() == executor::Error::input_failed);
        tunner.join();
        assert(run_failure.captured == 1 && run_failure.published == 0 && run_failure.stopped == 1);
        State shutdown_failure{};
        shutdown_failure.fail_shutdown = true;
        assert(tunner.start(std::make_unique<Worker>(shutdown_failure)));
        tunner.stop();
        assert(tunner.error() == executor::Error::output_failed);
        State invalid{};
        invalid.fail_initialize = true;
        assert(!tunner.start(std::make_unique<Worker>(invalid)));
        assert(tunner.finished() && tunner.error() == executor::Error::invalid_spec);
        assert(invalid.captured == 0 && invalid.stopped == 0);
        State last{};
        {
            executor::Tunner automatic;
            assert(automatic.start(std::make_unique<Worker>(last)));
            until([&] { return last.published.load() > 0; });
        }
        assert(last.stopped == 1);
    }
};

} // namespace dart::tests

int main() { dart::tests::TunnerTest::run(); }
