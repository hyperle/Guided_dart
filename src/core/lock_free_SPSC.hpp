#pragma once

#include <atomic>
#include <cstddef>
#include <array>

namespace dart::core {

template <typename T, size_t Capacity>
class LockFreeSPSCQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity should be a pwoer of 2");

public:
    LockFreeSPSCQueue() : head_(0), tail_(0) {}

    bool push(const T& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t current_head = head_.load(std::memory_order_acquire);

        if ((current_tail - current_head) >= Capacity) {
            return false;
        }

        buffer_[current_tail & MASK] = item;
    }

    bool pop(T& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t current_head = head_.load(std::memory_order_acquire);

        if (current_head == current_tail) {
            return false;
        }

        item = buffer_[current_head & MASK];
        head_.store(current_head + 1, std::memory_order_release);
        return true;
    }

    size_t size() const {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t tail = tail_.load(std::memory_order_relaxed);
        return (tail >= head) ? (tail - head) : 0;
    }

private:
    static constexpr size_t MASK = Capacity - 1;
    std::array<T, Capacity> buffer_;

    alignas(sizeof(size_t)) std::atomic<size_t> head_;
    alignas(sizeof(size_t)) std::atomic<size_t> tail_;

};
} // namespace dart::core