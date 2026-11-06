#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <functional>
#include <utility>
#include "imu/estimator.hpp"
#include "runtime/imu/session.hpp"
#include "src/detector/target.hpp"


namespace dart::control {

template<typename T>
class AtomicValue final {
    static_assert(std::atomic<T>::is_always_lock_free);

public:
    explicit AtomicValue(T initial = {}) noexcept
        : value_(initial) {}

    void write(const T& value) noexcept {
        value_.store(value, std::memory_order_relaxed);
        published_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_release);
    }

    T read() const noexcept {
        return value_.load(std::memory_order_relaxed);
    }

    std::chrono::steady_clock::time_point published_at() const noexcept {
        return std::chrono::steady_clock::time_point{
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::nanoseconds{published_ns_.load(std::memory_order_acquire)})};
    }

private:
    std::atomic<T> value_;
    std::atomic<std::int64_t> published_ns_{0};
};

template<typename T>
class Latest {
public:
    struct Snapshot {
        T value{};
        std::uint64_t sequence = 0;
        std::chrono::steady_clock::time_point published_at{};
    };

    void write(const T& value) {
        std::lock_guard lock(mutex_);
        snapshot_.value = value;
        ++snapshot_.sequence;
        snapshot_.published_at = std::chrono::steady_clock::now();
    }

    Snapshot read() const {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }
private:
    mutable std::mutex mutex_;
    Snapshot snapshot_{};
};

class BlackBoard final {
public:
    explicit BlackBoard(const imu::Session* session = nullptr)
        : imu_session_(session) {}

    explicit BlackBoard(const imu::Session& session)
        : BlackBoard(&session) {}

    template<typename Slot>
    auto read(const Slot& slot) const {
        return slot.read();
    }

    template<typename Slot, typename T>
    void write(Slot& slot, const T& value) {
        slot.write(value);
    }

    template<typename Slot, typename Fetch>
    void update(Slot& slot, Fetch&& fetch) {
        write(slot, std::invoke(std::forward<Fetch>(fetch)));
    }

    void update() {
        if (imu_session_ != nullptr) write(imu_state, imu_session_->current());
    }

    bool imu_working() {
        return imu_session_ != nullptr && imu_session_->imu_working();
    }

    Latest<imu::Estimator::Estimate> imu_state;
    AtomicValue<detection::Point> target{detection::Point{-1.0F, -1.0F}};
private:
    const imu::Session* imu_session_;
};
} //namespace dart::control
