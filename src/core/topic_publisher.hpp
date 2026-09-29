#pragma once

#include "lock_free_SPSC.hpp"
#include <cstdint>

namespace dart::core {
template <typename MsgType, std::size_t QueueCapacity, uint8_t MaxSubscribers = 4>
class LockFreeTopic {
public:
    using QueueType = LockFreeSPSCQueue<MsgType, QueueCapacity>;

    bool subscribe(QueueType& sub_queue) {
        if (sub_count_ < MaxSubscribers) {
            subscribers_[sub_count_++] = &sub_queue;
            return true;
        }
        return false;
    }

    void publish_from_isr(const MsgType& msg) {
        for (uint8_t i = 0; i < sub_count_; ++i) {
            subscribers_[i]->push(msg);
        }
    }

private:
    QueueType* subscribers_[MaxSubscribers]{nullptr};
    uint8_t sub_count_ = 0;
};
} //namespace dart::core