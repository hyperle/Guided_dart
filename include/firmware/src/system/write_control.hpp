#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <pthread.h>

#include "write_pause.h"

namespace dart::platform {
// File polling stays on this service thread. Runtime callers only publish
// memory requests; a host pause always takes precedence over both requests.
class WriteControl final {
public:
    WriteControl() = default;
    ~WriteControl() { stop(); }
    WriteControl(const WriteControl&) = delete;
    WriteControl& operator=(const WriteControl&) = delete;

    bool start(const char* path = dart_write_pause_path()) noexcept;
    void stop() noexcept;
    void request_recording(bool enabled, std::uint64_t timestamp_ns = 0) noexcept;
    void request_logging(bool enabled) noexcept { logging_.store(enabled); }
    bool recording_enabled() const noexcept { return !paused_.load() && recording_.load(); }
    bool logging_enabled() const noexcept { return !paused_.load() && logging_.load(); }
    bool paused() const noexcept { return paused_.load(); }
    std::uint64_t recording_request_ns() const noexcept { return recording_request_ns_.load(); }

private:
    static void* entry(void* context) noexcept;
    char path_[512]{};
    std::atomic<bool> running_{false}, paused_{true}, recording_{true}, logging_{true};
    std::atomic<std::uint64_t> recording_request_ns_{0};
    std::mutex mutex_;
    std::condition_variable wake_;
    pthread_t thread_{};
    bool started_ = false;
};

} // namespace dart::platform
