#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <pthread.h>

#include "pwm/interface.hpp"
#include "system/monotonic_clock.hpp"

namespace dart::pwm {
// The capture thread only publishes values. This task owns all subsequent HAL
// writes and expires observations even while capture or file logging is blocked.
class Session final {
public:
    struct Stats {
        bool running = false;
        bool neutral = true;
        std::uint64_t writes = 0;
        std::uint64_t timeouts = 0;
        Pulses pulse_us{};
        Error error{};
    };

    Session() = default;
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    std::expected<void, Error>
        start(IPwm& device, const Pulses& initial, std::uint32_t timeout_ms) noexcept;

    bool publish(const Pulses& pulses, bool has_target) noexcept;

    Stats stats() noexcept;

    void stop() noexcept;

private:
    static void* entry(void* context) noexcept;

    void run() noexcept;

    IPwm* device_ = nullptr;
    Pulses initial_{}, desired_{};
    bool has_target_ = false;
    bool joinable_ = false;
    std::chrono::steady_clock::time_point observed_{};
    std::chrono::milliseconds timeout_{200};
    Stats stats_{};
    pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
    pthread_t thread_{};
    std::atomic<bool> stop_{false};
};

} // namespace dart::pwm
