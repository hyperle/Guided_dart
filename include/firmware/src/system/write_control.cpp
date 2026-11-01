#include "system/write_control.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <pthread.h>

#include "write_pause.h"

namespace dart::platform {

bool WriteControl::start(const char* path) noexcept {
    if (started_ || path == nullptr || path[0] == '\0' || std::strlen(path) >= sizeof(path_))
        return false;
    std::strcpy(path_, path);
    paused_.store(dart_writes_paused(path_));
    running_.store(true);
    if (::pthread_create(&thread_, nullptr, &WriteControl::entry, this) != 0) {
        running_.store(false);
        paused_.store(true);
        return false;
    }
    started_ = true;
    return true;
}

void WriteControl::stop() noexcept {
    {
        std::lock_guard lock(mutex_);
        running_.store(false);
    }
    wake_.notify_all();
    if (started_) {
        (void)::pthread_join(thread_, nullptr);
        started_ = false;
    }
    paused_.store(true);
}

void WriteControl::request_recording(bool enabled, std::uint64_t timestamp_ns) noexcept {
    if (enabled)
        recording_request_ns_.store(timestamp_ns);
    recording_.store(enabled);
}

void* WriteControl::entry(void* context) noexcept {
    auto& self = *static_cast<WriteControl*>(context);
    while (self.running_.load()) {
        self.paused_.store(dart_writes_paused(self.path_));
        std::unique_lock lock(self.mutex_);
        self.wake_.wait_for(
            lock, std::chrono::milliseconds{50}, [&] { return !self.running_.load(); });
    }
    return nullptr;
}

} // namespace dart::platform
