#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <string_view>
#include <utility>

#include "camera/frame_lease.hpp"
#include "recorder/async/event_worker.hpp"
#include "recorder/config.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/image_writer.hpp"
#include "recorder/storage/session.hpp"
#include "system/write_control.hpp"

namespace dart::recorder {
enum class Admission : std::uint8_t {
    kAccepted,
    kDownsampled,
    kBusy,
    kStopped,
    kInvalidFrame,
};

class Recorder final {
public:
    struct Stats {
        std::uint64_t input = 0;
        std::uint64_t accepted = 0;
        std::uint64_t copied = 0;
        std::uint64_t written = 0;
        std::uint64_t downsampled = 0;
        std::uint64_t busy = 0;
        std::uint64_t timed_out = 0;
        std::uint64_t stopped = 0;
        std::uint64_t invalid = 0;
        std::uint64_t copy_fail = 0;
        std::uint64_t write_fail = 0;
        std::uint64_t event_dropped = 0;
        std::uint64_t lease_release_fail = 0;
        std::uint64_t bytes = 0;
        std::uint64_t copy_total_us = 0;
        std::uint64_t pack_total_us = 0;
        std::uint64_t io_total_us = 0;
        std::uint64_t io_max_us = 0;
        std::uint64_t space_total_us = 0, open_total_us = 0, write_total_us = 0, close_total_us = 0;
        std::uint32_t queued = 0;
        std::uint32_t release_pending = 0;
        std::uint32_t queue_high_water = 0;
    };

    explicit Recorder(Config config) noexcept;
    ~Recorder();
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    bool start() noexcept;
    // File sessions run on the supervisor; acquisition only tries admission.
    // A false->true write request starts a fresh session after releases drain.
    bool start_controlled(platform::WriteControl& writes) noexcept;
    void stop() noexcept;

    /* On kAccepted pixels are copied into a bounded slot and ownership moves
     * to the release queue. Otherwise the caller retains the lease. This never
     * waits for a queue or file I/O; drain_releases runs in acquisition. */
    Admission try_submit(camera::FrameLease&& lease) noexcept;
    Admission submit(camera::FrameLease&& lease) noexcept { return try_submit(std::move(lease)); }

    /* Camera release is serialized here with acquisition. Poll every capture
     * iteration and once after stop(), before destroying the camera. */
    std::uint32_t drain_releases() noexcept;
    std::uint32_t pending_releases() const noexcept;

    bool started() const noexcept;
    bool limit_reached() const noexcept { return limit_reached_.load(std::memory_order_acquire); }
    // Paths and the session budget are stable after stop(), or with lifecycle
    // calls serialized by the owner. Use stats()/started() during controlled runs.
    const char* directory() const noexcept { return session_.directory(); }
    const char* recording_path() const noexcept { return session_.image_path(); }
    std::uint64_t file_limit_bytes() const noexcept { return session_.file_limit_bytes(); }
    Error last_error() const noexcept { return error_.last_error(); }
    int last_system_error() const noexcept { return error_.last_system_error(); }
    const char* last_error_path() const noexcept { return error_.last_error_path(); }
    static std::string_view error_text(Error error) noexcept;
    static std::string_view decision_text(EventDecision decision) noexcept;
    Stats stats() const noexcept;

private:
    struct ImageSlot {
        std::unique_ptr<std::byte[]> pixels{};
        FrameEvent event{};
        std::chrono::steady_clock::time_point queued_at{};
    };

    static constexpr std::uint64_t kMaxFrameDimension = 16384;

    static bool
        checked_frame_bytes(std::uint32_t width, std::uint32_t height, std::size_t& bytes) noexcept;

    static constexpr std::size_t kMaxPath = 512;
    static constexpr std::uint32_t kMaxQueueSlots = 64;
    static constexpr std::uint32_t kMaxEventSlots = 4096;

    bool validate_config() noexcept;
    bool allocate_buffers() noexcept;

    void image_loop() noexcept;
    /* pthread entry points.  This module builds under -fno-exceptions, where a
     * throwing std::thread constructor has no recoverable error path, so thread
     * creation reports failure through pthread_create's return code instead. */
    static void* image_thread_entry(void* context) noexcept;
    void rollback_start() noexcept;
    void stop_session() noexcept;
    static void* control_entry(void* context) noexcept;

    void set_error(Error error) noexcept { error_.set_error(error); }

    Config config_{};
    RecorderError error_{};
    Session session_{config_, error_};
    ImageWriter images_{config_, session_};
    EventWorker events_{config_};
    std::size_t frame_bytes_ = 0;

    std::unique_ptr<ImageSlot[]> image_slots_{};
    std::unique_ptr<camera::FrameLease[]> release_slots_{};

    std::uint32_t image_capacity_ = 0;
    std::uint32_t image_head_ = 0;
    std::uint32_t image_tail_ = 0;
    std::uint32_t image_count_ = 0;
    std::uint32_t release_head_ = 0;
    std::uint32_t release_tail_ = 0;
    std::uint32_t release_count_ = 0;
    mutable std::mutex image_mutex_;
    std::condition_variable image_cv_;
    pthread_t image_thread_{};
    bool image_thread_started_ = false;
    bool accepting_ = false;
    // Restart is allowed only after the old workers and camera releases drain.
    std::atomic<bool> started_once_{false};
    std::atomic<bool> limit_reached_{false};
    bool buffers_ready_ = false;
    std::mutex lifecycle_mutex_;
    platform::WriteControl* writes_ = nullptr;
    pthread_t control_thread_{};
    bool control_started_ = false;
    std::atomic<bool> controlling_{false};
    std::mutex control_mutex_;
    std::condition_variable control_wake_;

    std::atomic<std::uint64_t> input_{0};
    std::atomic<std::uint64_t> accepted_{0};
    std::atomic<std::uint64_t> copied_{0};
    std::atomic<std::uint64_t> written_{0};
    std::atomic<std::uint64_t> downsampled_{0};
    std::atomic<std::uint64_t> busy_{0};
    std::atomic<std::uint64_t> timed_out_{0};
    std::atomic<std::uint64_t> stopped_{0};
    std::atomic<std::uint64_t> invalid_{0};
    std::atomic<std::uint64_t> copy_fail_{0};
    std::atomic<std::uint64_t> write_fail_{0};
    std::atomic<std::uint64_t> lease_release_fail_{0};
    std::atomic<std::uint64_t> bytes_{0};
    std::atomic<std::uint64_t> copy_total_us_{0};
    std::atomic<std::uint64_t> pack_total_us_{0};
    std::atomic<std::uint64_t> io_total_us_{0};
    std::atomic<std::uint64_t> io_max_us_{0};
    std::atomic<std::uint64_t> space_total_us_{0}, open_total_us_{0}, write_total_us_{0},
        close_total_us_{0};
    std::atomic<std::uint64_t> next_sample_tick_{0};
    std::atomic<std::uint32_t> queue_high_water_{0};
};

} // namespace dart::recorder
