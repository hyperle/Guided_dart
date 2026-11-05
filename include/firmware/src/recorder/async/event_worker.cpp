#include "recorder/async/event_worker.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <pthread.h>

#include "recorder/config.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/event_writer.hpp"
#include "recorder/storage/session.hpp"

namespace dart::recorder {

bool EventWorker::prepare() noexcept {
    slots_.reset(new (std::nothrow) FrameEvent[config_.event_slots]);
    return slots_ != nullptr && writer_.prepare();
}

bool EventWorker::open(const Session& session, RecorderError& error) noexcept {
    return writer_.open(session, error);
}

bool EventWorker::start() noexcept {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_ = count_ = 0;
        dropped_.store(0);
        running_ = true;
    }
    if (::pthread_create(&thread_, nullptr, &EventWorker::entry, this) != 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        return false;
    }
    started_ = true;
    return true;
}

void EventWorker::stop() noexcept {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
    ready_.notify_all();
    if (started_) {
        (void)::pthread_join(thread_, nullptr);
        started_ = false;
    }
    writer_.close();
}

bool EventWorker::enqueue(const FrameEvent& event) noexcept {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !running_ || count_ >= config_.event_slots) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    slots_[tail_] = event;
    tail_ = (tail_ + 1U) % config_.event_slots;
    ++count_;
    lock.unlock();
    ready_.notify_one();
    return true;
}

void* EventWorker::entry(void* context) noexcept {
    static_cast<EventWorker*>(context)->run();
    return nullptr;
}

void EventWorker::run() noexcept {
    for (;;) {
        FrameEvent event{};
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [this] { return count_ != 0 || !running_; });
            if (count_ == 0 && !running_)
                break;
            event = slots_[head_];
            head_ = (head_ + 1U) % config_.event_slots;
            --count_;
        }
        writer_.write(event);
    }
}

} // namespace dart::recorder
