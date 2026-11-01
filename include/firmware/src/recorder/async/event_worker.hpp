#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <pthread.h>

#include "recorder/config.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/event_writer.hpp"
#include "recorder/storage/session.hpp"

namespace dart::recorder {
// Bounded metadata queue and its consumer lifetime. Producers never wait for
// disk or for the queue lock; stop drains accepted rows before closing the CSV.
class EventWorker final {
public:
    explicit EventWorker(const Config& config) noexcept
        : config_(config)
        , writer_(config) {}
    ~EventWorker() { stop(); }
    bool prepare() noexcept;
    bool open(const Session& session, RecorderError& error) noexcept;
    bool start() noexcept;
    void stop() noexcept;
    bool started() const noexcept { return started_; }
    bool enqueue(const FrameEvent& event) noexcept;
    std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

private:
    static void* entry(void* context) noexcept;
    void run() noexcept;
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
