#pragma once

#include <atomic>
#include <expected>
#include <memory>
#include <optional>
#include <pthread.h>
#include <stop_token>
#include <utility>

#include "runtime/component.hpp"

namespace dart::executor {

// Owns one component and its execution thread. The caller keeps injected devices
// and channels alive until join() returns. Lifecycle calls belong to the caller;
// run() and shutdown() are serialized on the worker after successful startup.
class Tunner final {
public:
    Tunner() = default;
    ~Tunner() { stop(); }
    Tunner(const Tunner&) = delete;
    Tunner& operator=(const Tunner&) = delete;

    std::expected<void, Error> start(std::unique_ptr<Component> component) noexcept {
        if (joinable_ || !component)
            return std::unexpected(Error::invalid_state);
        run_error_ = shutdown_error_ = kNoError;
        finished_.store(false, std::memory_order_relaxed);
        if (const auto initialized = component->initialze(); !initialized) {
            run_error_ = static_cast<int>(initialized.error());
            finished_.store(true, std::memory_order_release);
            return initialized;
        }
        component_ = std::move(component);
        stop_.emplace();
        if (::pthread_create(&thread_, nullptr, &Tunner::entry, this) != 0) {
            if (const auto closed = component_->shutdown(); !closed)
                shutdown_error_ = static_cast<int>(closed.error());
            component_.reset();
            stop_.reset();
            run_error_ = static_cast<int>(Error::creation_fail);
            finished_.store(true, std::memory_order_release);
            return std::unexpected(Error::creation_fail);
        }
        joinable_ = true;
        return {};
    }

    void request_stop() noexcept {
        if (joinable_)
            stop_->request_stop();
    }
    void join() noexcept {
        if (!joinable_)
            return;
        (void)::pthread_join(thread_, nullptr);
        joinable_ = false;
        component_.reset();
        stop_.reset();
    }
    void stop() noexcept {
        request_stop();
        join();
    }
    bool finished() const noexcept { return finished_.load(std::memory_order_acquire); }
    std::optional<Error> error() const noexcept {
        const int run = run_error_.load(std::memory_order_relaxed);
        const int shutdown = shutdown_error_.load(std::memory_order_relaxed);
        if (run != kNoError)
            return static_cast<Error>(run);
        if (shutdown != kNoError)
            return static_cast<Error>(shutdown);
        return std::nullopt;
    }

private:
    static void* entry(void* context) noexcept {
        auto& self = *static_cast<Tunner*>(context);
        if (const auto result = self.component_->run(self.stop_->get_token()); !result)
            self.run_error_ = static_cast<int>(result.error());
        if (const auto result = self.component_->shutdown(); !result)
            self.shutdown_error_ = static_cast<int>(result.error());
        self.finished_.store(true, std::memory_order_release);
        return nullptr;
    }

    static constexpr int kNoError = -1;
    std::unique_ptr<Component> component_{};
    std::optional<std::stop_source> stop_{};
    pthread_t thread_{};
    std::atomic<int> run_error_{kNoError}, shutdown_error_{kNoError};
    std::atomic<bool> finished_{true};
    bool joinable_ = false;
};

} // namespace dart::executor
