module;
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <pthread.h>

export module dart.recorder.async.event_worker;

import dart.recorder.config;
import dart.recorder.error;
import dart.recorder.storage.event_writer;
import dart.recorder.storage.session;

export namespace dart::recorder {

// Bounded metadata queue and its consumer lifetime. Producers never wait for
// disk or for the queue lock; stop drains accepted rows before closing the CSV.
class EventWorker final {
public:
    explicit EventWorker(const Config& config) noexcept
        : config_(config)
        , writer_(config) {}
    ~EventWorker() { stop(); }
    bool prepare() noexcept {
        slots_.reset(new (std::nothrow) FrameEvent[config_.event_slots]);
        return slots_ != nullptr && writer_.prepare();
    }
    bool open(const Session& session, RecorderError& error) noexcept {
        return writer_.open(session, error);
    }
    bool start() noexcept {
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
    void stop() noexcept {
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
    bool started() const noexcept { return started_; }
    bool enqueue(const FrameEvent& event) noexcept {
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
    std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

private:
    static void* entry(void* context) noexcept {
        static_cast<EventWorker*>(context)->run();
        return nullptr;
    }
    void run() noexcept {
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
    const Config& config_;
    EventWriter writer_;
    std::unique_ptr<FrameEvent[]> slots_;
    std::mutex mutex_;
    std::condition_variable ready_;
    pthread_t thread_{};
    bool started_ = false;
    bool running_ = false;
    std::uint32_t head_ = 0, tail_ = 0, count_ = 0;
    std::atomic<std::uint64_t> dropped_{0};
};

} // namespace dart::recorder
