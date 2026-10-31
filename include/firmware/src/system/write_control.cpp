module;
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <pthread.h>

#include "write_pause.h"

export module dart.system.write_control;



export namespace dart::platform {

// File polling stays on this service thread. Runtime callers only publish
// memory requests; a host pause always takes precedence over both requests.
class WriteControl final {
public:
    WriteControl() = default;
    ~WriteControl() { stop(); }
    WriteControl(const WriteControl&) = delete;
    WriteControl& operator=(const WriteControl&) = delete;

    bool start(const char* path = dart_write_pause_path()) noexcept {
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
    void stop() noexcept {
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
    void request_recording(bool enabled, std::uint64_t timestamp_ns = 0) noexcept {
        if (enabled)
            recording_request_ns_.store(timestamp_ns);
        recording_.store(enabled);
    }
    void request_logging(bool enabled) noexcept { logging_.store(enabled); }
    bool recording_enabled() const noexcept { return !paused_.load() && recording_.load(); }
    bool logging_enabled() const noexcept { return !paused_.load() && logging_.load(); }
    bool paused() const noexcept { return paused_.load(); }
    std::uint64_t recording_request_ns() const noexcept { return recording_request_ns_.load(); }

private:
    static void* entry(void* context) noexcept {
        auto& self = *static_cast<WriteControl*>(context);
        while (self.running_.load()) {
            self.paused_.store(dart_writes_paused(self.path_));
            std::unique_lock lock(self.mutex_);
            self.wake_.wait_for(lock, std::chrono::milliseconds{50}, [&] {
                return !self.running_.load();
            });
        }
        return nullptr;
    }
    char path_[512]{};
    std::atomic<bool> running_{false}, paused_{true}, recording_{true}, logging_{true};
    std::atomic<std::uint64_t> recording_request_ns_{0};
    std::mutex mutex_;
    std::condition_variable wake_;
    pthread_t thread_{};
    bool started_ = false;
};

} // namespace dart::platform
