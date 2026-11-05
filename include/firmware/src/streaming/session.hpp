#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <pthread.h>

#include "streaming/jpeg.h"

#include "camera/frame_view.hpp"
#include "streaming/interface.hpp"

namespace dart::streaming {
enum class State : std::uint8_t { kStopped, kStarting, kRunning, kStopping, kPaused, kFailed };

const char* state_text(State state) noexcept;

class Session final {
public:
    struct Stats {
        std::uint64_t accepted = 0, downsampled = 0, busy = 0, invalid = 0;
        std::uint64_t raw_replaced = 0, jpeg_replaced = 0, encoded = 0, jpeg_errors = 0;
        std::uint64_t uploaded = 0, bytes = 0, network_errors = 0, stale_dropped = 0;
        bool connected = false;
        int last_native = 0;
        std::uint64_t buffer_bytes = 0;
    };

    Session() = default;
    ~Session() { stop(); }
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // The owner serializes start/start_controlled/stop. Controlled start gives
    // worker ownership to the supervisor; capture only borrows pixels through
    // try_submit. It never joins, waits for a device, or touches control files.
    bool start(
        const char* path, std::uint32_t width, std::uint32_t height,
        TransportFactory factory) noexcept;

    bool start_controlled(
        const char* path, std::uint32_t width, std::uint32_t height, TransportFactory factory,
        const char* pause_path = "/sdcard/app/control/stream.pause") noexcept;

    bool try_submit(const camera::FrameView& frame) noexcept;

    void stop() noexcept;

    Stats stats() const noexcept;
    State state() const noexcept { return state_.load(); }
    bool accepting() const noexcept { return accepting_.load(); }
    Error error() const noexcept;

private:
    enum Counter : unsigned {
        kAccepted,
        kDownsampled,
        kBusy,
        kInvalid,
        kRawReplaced,
        kJpegReplaced,
        kEncoded,
        kJpegErrors,
        kUploaded,
        kBytes,
        kNetworkErrors,
        kStaleDropped,
        kCount
    };
    struct Slot {
        std::unique_ptr<std::uint8_t[]> pixels;
        std::size_t size = 0;
        std::uint64_t sequence = 0, received_us = 0;
    };
    void set_error(Error error) noexcept;
    bool prepare_source(
        std::uint32_t width, std::uint32_t height, TransportFactory factory) noexcept;
    bool load_configuration(const char* path) noexcept;
    // Requires lifecycle_mutex_. These calls may wait for worker/device exit;
    // the capture path exclusively uses try_lock and never waits on this lock.
    bool start_workers() noexcept;
    void stop_workers() noexcept;
    bool paused() const noexcept;
    static void* supervisor_entry(void* context) noexcept;
    void supervise() noexcept;
    static void* compress_entry(void* context) noexcept;
    static void* send_entry(void* context) noexcept;
    void compress() noexcept;
    void send() noexcept;

    Config config_{};
    std::atomic<std::uint64_t> error_{0}, buffer_bytes_{0};
    std::atomic<State> state_{State::kStopped};
    std::atomic_bool accepting_{false}, supervisor_running_{false};
    TransportFactory factory_ = nullptr;
    char pause_path_[512]{}, config_path_[512]{};
    std::mutex lifecycle_mutex_, supervisor_mutex_;
    std::condition_variable supervisor_wake_;
    pthread_t supervisor_{};
    bool supervisor_started_ = false;
    std::uint32_t width_ = 0, height_ = 0;
    std::size_t jpeg_capacity_ = 0;
    dart_jpeg_encoder_t encoder_{};
    std::array<Slot, 2> raw_, jpeg_;
    std::array<bool, 2> raw_busy_{};
    int raw_pending_ = -1, jpeg_pending_ = -1, sending_ = -1;
    std::unique_ptr<ITransport> transport_;
    std::chrono::steady_clock::time_point next_frame_{};
    std::array<std::atomic<std::uint64_t>, kCount> counts_{};
    std::atomic_bool stopping_{true}, connected_{false};
    std::atomic_int last_native_{0};
    std::mutex mutex_;
    std::condition_variable wake_;
    pthread_t compressor_{}, sender_{};
    bool compressor_started_ = false, sender_started_ = false;
};

} // namespace dart::streaming
