#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <functional>
#include <utility>
#include "imu/estimator.hpp"
#include "runtime/imu/session.hpp"
#include "src/detector/geometry.hpp"


namespace dart::control {

template<typename T>
class AtomicValue final {
    static_assert(std::atomic<T>::is_always_lock_free);

public:
    explicit AtomicValue(T initial = {}) noexcept
        : value_(initial) {}

    void write(const T& value) noexcept {
        value_.store(value, std::memory_order_relaxed);
    }

    T read() const noexcept {
        return value_.load(std::memory_order_relaxed);
    }

private:
    std::atomic<T> value_;
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
    explicit BlackBoard(const imu::Session& session, const AtomicValue<detection::Point>& vision_source)
        : imu_session_(session)
        , vision_source_(vision_source) {}

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
        write(imu_state, imu_session_.current());
        write(target, read(vision_source_));
    }

    bool imu_working() {
        return imu_session_.imu_working();
    }

    Latest<imu::Estimator::Estimate> imu_state;
    AtomicValue<detection::Point> target{detection::Point{-1.0F, -1.0F}};
private:
    const imu::Session& imu_session_;
    const AtomicValue<detection::Point>& vision_source_;
};
} //namespace dart::control
