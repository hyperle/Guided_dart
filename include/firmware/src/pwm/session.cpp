#include "pwm/session.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <pthread.h>

#include "pwm/interface.hpp"
#include "system/monotonic_clock.hpp"

namespace dart::pwm {

Session::~Session() {
    stop();
    ::pthread_mutex_destroy(&mutex_);
}

std::expected<void, Error>
    Session::start(IPwm& device, const Pulses& initial, std::uint32_t timeout_ms) noexcept {
    if (joinable_ || timeout_ms == 0 || !valid_pulses(initial))
        return std::unexpected(Error{"session_config", -1, -1});
    auto started = device.start(initial);
    if (!started)
        return started;
    device_ = &device;
    initial_ = desired_ = initial;
    timeout_ = std::chrono::milliseconds{timeout_ms};
    has_target_ = false;
    stats_ = {true, true, 0, 0, initial, {}};
    stop_.store(false);
    const int rc = ::pthread_create(&thread_, nullptr, entry, this);
    if (rc != 0) {
        (void)device_->stop();
        stats_.running = false;
        stats_.error = {"thread_start", -1, rc};
        return std::unexpected(stats_.error);
    }
    joinable_ = true;
    return {};
}

bool Session::publish(const Pulses& pulses, bool has_target) noexcept {
    if (!valid_pulses(pulses))
        return false;
    ::pthread_mutex_lock(&mutex_);
    desired_ = pulses;
    has_target_ = has_target;
    observed_ = std::chrono::steady_clock::now();
    ::pthread_mutex_unlock(&mutex_);
    return true;
}

Session::Stats Session::stats() noexcept {
    ::pthread_mutex_lock(&mutex_);
    const auto result = stats_;
    ::pthread_mutex_unlock(&mutex_);
    return result;
}

void Session::stop() noexcept {
    if (!joinable_)
        return;
    stop_.store(true);
    ::pthread_join(thread_, nullptr);
    joinable_ = false;
}

void* Session::entry(void* context) noexcept {
    static_cast<Session*>(context)->run();
    return nullptr;
}

void Session::run() noexcept {
    Pulses applied = initial_;
    for (;;) {
        if (stop_.load())
            break;
        ::pthread_mutex_lock(&mutex_);
        const bool expired =
            has_target_ && std::chrono::steady_clock::now() - observed_ >= timeout_;
        if (expired) {
            has_target_ = false;
            ++stats_.timeouts;
        }
        const bool neutral = !has_target_;
        const Pulses command = neutral ? initial_ : desired_;
        ::pthread_mutex_unlock(&mutex_);
        if (command != applied) {
            const auto written = device_->write(command);
            ::pthread_mutex_lock(&mutex_);
            if (!written)
                stats_.error = written.error();
            else {
                applied = command;
                stats_.pulse_us = command;
                ++stats_.writes;
            }
            ::pthread_mutex_unlock(&mutex_);
            if (!written)
                break;
        }
        ::pthread_mutex_lock(&mutex_);
        stats_.neutral = neutral;
        ::pthread_mutex_unlock(&mutex_);
        platform::sleep_ms(10);
    }
    // Disable before returning, including after a partial four-channel write.
    const auto stopped = device_->stop();
    ::pthread_mutex_lock(&mutex_);
    stats_.running = false;
    if (!stopped && stats_.error.code == 0)
        stats_.error = stopped.error();
    ::pthread_mutex_unlock(&mutex_);
}

} // namespace dart::pwm
