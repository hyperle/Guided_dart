#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <limits>
#include <memory>
#include <mutex>
#include <string_view>

#include "camera/frame_view.hpp"
#include "recorder/config.hpp"
#include "recorder/error.hpp"
#include "runtime/component.hpp"
#include "system/write_control.hpp"

namespace dart::recorder {

enum class Admission : std::uint8_t { kAccepted, kDownsampled, kBusy, kStopped, kInvalidFrame };

class Record final {
public:
    class Sampler final {
    public:
        explicit Sampler(std::uint32_t fps = 60) noexcept : fps_(fps) {}
        void reset() noexcept { next_tick_ = 0; }
        bool due(std::chrono::microseconds received) noexcept {
            const auto us = static_cast<std::uint64_t>(received.count());
            const auto tick = (us / 1'000'000U) * fps_ + (us % 1'000'000U) * fps_ / 1'000'000U;
            if (tick < next_tick_) return false;
            next_tick_ = tick + 1U;
            return true;
        }
    private:
        std::uint32_t fps_;
        std::uint64_t next_tick_ = 0;
    };

    explicit Record(const Config& config) noexcept : config_(config) {}
    static Error validate(const Config& config) noexcept {
        if (config.output_directory.empty()) return Error::kInvalidConfig;
        if (config.output_directory.size() >= RecorderError::kMaxPath) return Error::kPathTooLong;
        if (config.width == 0 || config.height == 0 || config.width > 16384 || config.height > 16384
            || static_cast<std::uint64_t>(config.width) * config.height > std::numeric_limits<std::size_t>::max()
            || config.fps == 0 || config.fps > 1'000'000 || config.event_slots == 0 || config.event_slots > 4096
            || config.queue_slots == 0 || config.queue_slots > 64
            || config.frame_timeout.count() < 0
            || config.frame_timeout.count() > 60'000 || config.keep_sessions > 9999
            || config.max_file_bytes == 0 || config.max_file_bytes > Config::kMaxFileBytes
            || config.storage_check_frames == 0 || config.storage_check_frames > 32
            || static_cast<unsigned>(config.image_format) > static_cast<unsigned>(ImageFormat::kH264))
            return Error::kInvalidConfig;
        if (config.image_format == ImageFormat::kH264
            && (config.video_encoder_factory == nullptr || config.fps > 120
                || config.width % 2 != 0 || config.height % 2 != 0))
            return Error::kInvalidConfig;
        return Error::kNone;
    }
    bool valid_view(const camera::FrameView& view) const noexcept {
        return view.format == camera::PixelFormat::kY8 && view.valid()
            && view.width == config_.width && view.height == config_.height
            && view.received.count() >= 0;
    }
    bool expired(std::chrono::steady_clock::time_point queued_at) const noexcept {
        const auto timeout = config_.frame_timeout.count() == 0
            ? std::chrono::microseconds{(1'000'000U + config_.fps - 1U) / config_.fps}
            : std::chrono::duration_cast<std::chrono::microseconds>(config_.frame_timeout);
        return std::chrono::steady_clock::now() - queued_at >= timeout;
    }
private:
    const Config& config_;
};

// Shared counters are atomic; paths and detailed errors use a separate snapshot.
// The detector never accesses the details mutex or any file operation.
class RecorderStatus final {
public:
    struct Stats {
        // Copy counters stay zero: acquisition-to-recorder handoff moves a lease.
        std::uint64_t input = 0, accepted = 0, copied = 0, written = 0;
        std::uint64_t encode_attempts = 0, encoded = 0, enqueued = 0, write_attempts = 0;
        std::uint64_t downsampled = 0, busy = 0, timed_out = 0, stopped = 0, invalid = 0;
        std::uint64_t copy_fail = 0, write_fail = 0, event_dropped = 0, lease_release_fail = 0;
        std::uint64_t buffer_dropped = 0;
        std::uint64_t bytes = 0, copy_total_us = 0, pack_total_us = 0, io_total_us = 0, io_max_us = 0;
        std::uint64_t space_total_us = 0, open_total_us = 0, write_total_us = 0, close_total_us = 0;
        std::uint64_t pack_max_us = 0, buffer_copy_total_us = 0, buffer_copy_max_us = 0, drain_us = 0;
        std::uint32_t queued = 0, release_pending = 0, queue_high_water = 0;
        std::uint32_t buffered = 0, buffer_high_water = 0;
    };
    struct Details {
        int system_error = 0;
        std::array<char, RecorderError::kMaxPath> error_path{}, directory{}, recording_path{};
        std::uint64_t file_limit_bytes = 0;
    };
    struct SessionEvent {
        enum class Kind : std::uint8_t { kStarted, kStartFailed, kStopped };
        enum class Reason : std::uint8_t { kRequested, kPaused, kShutdown, kLimit, kError };
        Kind kind = Kind::kStarted;
        Reason reason = Reason::kRequested;
        std::uint64_t generation = 0;
        Details details{};
        Stats stats{};
        Error error = Error::kNone;
    };
    // Lifecycle events retain the final counters before the next session
    // resets them. Only Recorder publishes; application diagnostics consumes.
    bool pop_session_event(SessionEvent& event) noexcept {
        std::unique_lock lock(session_mutex_, std::try_to_lock);
        if (!lock.owns_lock() || session_count_ == 0) return false;
        event = session_events_[session_head_];
        session_head_ = (session_head_ + 1U) % session_events_.size();
        --session_count_;
        return true;
    }
    std::uint64_t session_events_dropped() const noexcept { return session_events_dropped_.load(); }
    Stats stats() const noexcept {
        Stats s{};
#define DART_RECORDER_LOAD(name) s.name = name##_.load(std::memory_order_relaxed)
        DART_RECORDER_LOAD(input); DART_RECORDER_LOAD(accepted); DART_RECORDER_LOAD(written);
        DART_RECORDER_LOAD(encode_attempts); DART_RECORDER_LOAD(encoded);
        DART_RECORDER_LOAD(enqueued); DART_RECORDER_LOAD(write_attempts);
        DART_RECORDER_LOAD(downsampled); DART_RECORDER_LOAD(busy); DART_RECORDER_LOAD(timed_out);
        DART_RECORDER_LOAD(stopped); DART_RECORDER_LOAD(invalid); DART_RECORDER_LOAD(write_fail);
        DART_RECORDER_LOAD(event_dropped); DART_RECORDER_LOAD(lease_release_fail); DART_RECORDER_LOAD(bytes);
        DART_RECORDER_LOAD(pack_total_us); DART_RECORDER_LOAD(io_total_us); DART_RECORDER_LOAD(io_max_us);
        DART_RECORDER_LOAD(space_total_us); DART_RECORDER_LOAD(open_total_us); DART_RECORDER_LOAD(write_total_us);
        DART_RECORDER_LOAD(close_total_us); DART_RECORDER_LOAD(queued); DART_RECORDER_LOAD(release_pending);
        DART_RECORDER_LOAD(pack_max_us); DART_RECORDER_LOAD(buffer_copy_total_us);
        DART_RECORDER_LOAD(buffer_copy_max_us); DART_RECORDER_LOAD(drain_us);
        DART_RECORDER_LOAD(queue_high_water);
        DART_RECORDER_LOAD(buffer_dropped); DART_RECORDER_LOAD(buffered); DART_RECORDER_LOAD(buffer_high_water);
#undef DART_RECORDER_LOAD
        return s;
    }
    Details details() const noexcept { std::lock_guard lock(details_mutex_); return details_; }
    bool started() const noexcept { return active_.load(std::memory_order_acquire); }
    std::uint64_t generation() const noexcept { return generation_.load(std::memory_order_acquire); }
    bool limit_reached() const noexcept { return limit_.load(std::memory_order_acquire); }
    Error last_error() const noexcept { return error_.load(std::memory_order_acquire); }
    void set_error(Error error) noexcept { error_.store(error, std::memory_order_release); }
    static std::string_view error_text(Error error) noexcept {
        switch (error) {
        case Error::kNone: return "none";
        case Error::kAlreadyStarted: return "already_started";
        case Error::kInvalidConfig: return "invalid_config";
        case Error::kNoMemory: return "no_memory";
        case Error::kPathTooLong: return "path_too_long";
        case Error::kDirectory: return "directory";
        case Error::kEventFile: return "event_file";
        case Error::kImageFile: return "image_file";
        case Error::kStorage: return "storage";
        case Error::kThread: return "thread";
        }
        return "unknown";
    }
private:
    friend class FrameChannel;
    friend class Recorder;
    friend class BufferedStorage;
    void publish_session(SessionEvent::Kind kind, SessionEvent::Reason reason) noexcept {
        const SessionEvent event{kind, reason, generation(), details(), stats(), last_error()};
        std::lock_guard lock(session_mutex_);
        if (session_count_ == session_events_.size()) { ++session_events_dropped_; return; }
        session_events_[session_tail_] = event;
        session_tail_ = (session_tail_ + 1U) % session_events_.size();
        ++session_count_;
    }
    void reset_session() noexcept {
        input_ = accepted_ = written_ = downsampled_ = busy_ = timed_out_ = stopped_ = invalid_ = 0;
        encode_attempts_ = encoded_ = enqueued_ = write_attempts_ = 0;
        write_fail_ = event_dropped_ = lease_release_fail_ = bytes_ = pack_total_us_ = io_total_us_ = io_max_us_ = 0;
        space_total_us_ = open_total_us_ = write_total_us_ = close_total_us_ = 0;
        pack_max_us_ = buffer_copy_total_us_ = buffer_copy_max_us_ = drain_us_ = 0;
        queued_ = release_pending_ = queue_high_water_ = 0;
        buffer_dropped_ = buffered_ = buffer_high_water_ = 0;
        limit_ = false;
        error_ = Error::kNone;
    }
    mutable std::mutex details_mutex_;
    Details details_{};
    std::mutex session_mutex_;
    std::array<SessionEvent, 8> session_events_{};
    std::size_t session_head_ = 0, session_tail_ = 0, session_count_ = 0;
    std::atomic<std::uint64_t> session_events_dropped_{0};
    std::atomic<bool> active_{false}, limit_{false};
    std::atomic<Error> error_{Error::kNone};
    std::atomic<std::uint64_t> generation_{0};
    std::atomic<std::uint64_t> input_{0}, accepted_{0}, written_{0}, downsampled_{0}, busy_{0};
    std::atomic<std::uint64_t> encode_attempts_{0}, encoded_{0}, enqueued_{0}, write_attempts_{0};
    std::atomic<std::uint64_t> timed_out_{0}, stopped_{0}, invalid_{0}, write_fail_{0}, event_dropped_{0};
    std::atomic<std::uint64_t> lease_release_fail_{0}, bytes_{0}, pack_total_us_{0}, io_total_us_{0}, io_max_us_{0};
    std::atomic<std::uint64_t> space_total_us_{0}, open_total_us_{0}, write_total_us_{0}, close_total_us_{0};
    std::atomic<std::uint64_t> pack_max_us_{0}, buffer_copy_total_us_{0}, buffer_copy_max_us_{0}, drain_us_{0};
    std::atomic<std::uint32_t> queued_{0}, release_pending_{0}, queue_high_water_{0};
    std::atomic<std::uint64_t> buffer_dropped_{0};
    std::atomic<std::uint32_t> buffered_{0}, buffer_high_water_{0};
};

class FrameChannel;
class RecorderFactory final {
public:
    // Channel, status, and optional write control outlive the component and its runner.
    static std::expected<std::unique_ptr<executor::Component>, executor::Error> create(
        const executor::Component::Spec& spec, Config config, FrameChannel& frames,
        RecorderStatus& status, platform::WriteControl* writes = nullptr);
};

} // namespace dart::recorder
