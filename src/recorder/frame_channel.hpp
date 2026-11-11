#pragma once

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <utility>

#include "camera/frame_lease.hpp"
#include "src/recorder/record.hpp"

namespace dart::recorder {

// One capture-thread producer and one recorder-thread consumer. All producer
// locks are try-locks. The camera owner drains returns even when recording is off.
class FrameChannel final {
public:
    struct Frame {
        camera::FrameLease lease{};
        FrameEvent event{};
        std::chrono::steady_clock::time_point queued_at{};
    };

    bool prepare(const Config& config, RecorderStatus& status,
                 platform::WriteControl* writes = nullptr) noexcept {
        if ((prepared_ && !quiescent()) || Record::validate(config) != Error::kNone) return false;
        event_head_ = event_tail_ = event_count_ = 0;
        events_.reset(new (std::nothrow) FrameEvent[config.event_slots]);
        if (!events_) return false;
        config_ = config;
        sampler_ = Record::Sampler{config.fps};
        status_ = &status;
        writes_ = writes;
        prepared_ = true;
        return true;
    }

    Admission try_submit(camera::FrameLease&& lease) noexcept {
        if (!prepared_ || !status_->started()
            || (writes_ != nullptr && !writes_->recording_enabled())) return Admission::kStopped;
        const auto input = status_->input_.fetch_add(1, std::memory_order_relaxed) + 1U;
        FrameEvent event{};
        event.input_index = input;
        if (lease.valid()) {
            const auto& view = lease.view();
            event.source_sequence = view.sequence;
            event.source_pts = view.source_pts;
            event.received_us = view.received.count();
            event.source_pts_timebase_validated = view.source_pts_timebase_validated;
        }
        const auto finish = [&](Admission admission, EventDecision decision) noexcept {
            event.decision = decision;
            switch (admission) {
            case Admission::kDownsampled: ++status_->downsampled_; break;
            case Admission::kBusy:
                ++status_->busy_;
                if (decision == EventDecision::kBufferFull) ++status_->buffer_dropped_;
                break;
            case Admission::kStopped: ++status_->stopped_; break;
            case Admission::kInvalidFrame: ++status_->invalid_; break;
            case Admission::kAccepted: break;
            }
            enqueue_event(event);
            return admission;
        };
        if (!lease.valid() || !Record{config_}.valid_view(lease.view()))
            return finish(Admission::kInvalidFrame, EventDecision::kInvalidFrame);
        std::unique_lock lock(frame_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return finish(Admission::kBusy, EventDecision::kBusy);
        if (!accepting_) return finish(Admission::kStopped, EventDecision::kStopped);
        // Consume the sampling interval before checking readiness: no catch-up burst.
        if (!sampler_.due(lease.view().received))
            return finish(Admission::kDownsampled, EventDecision::kDownsampled);
        if (status_->buffered_.load(std::memory_order_acquire) >= config_.queue_slots)
            return finish(Admission::kBusy, EventDecision::kBufferFull);
        if (!idle_ || state_.load(std::memory_order_acquire) != State::kEmpty)
            return finish(Admission::kBusy, EventDecision::kBusy);
        frame_.event = event;
        frame_.queued_at = std::chrono::steady_clock::now();
        frame_.lease = std::move(lease);
        idle_ = false;
        status_->queued_ = 1;
        status_->queue_high_water_ = 1;
        ++status_->accepted_;
        state_.store(State::kPending, std::memory_order_release);
        return Admission::kAccepted;
    }

    // These methods run on the recorder thread; no file I/O occurs under a lock.
    Frame take() noexcept {
        std::lock_guard lock(frame_mutex_);
        if (state_.load(std::memory_order_acquire) != State::kPending) return {};
        Frame result{std::move(frame_.lease), frame_.event, frame_.queued_at};
        status_->queued_ = 0;
        state_.store(State::kHeld, std::memory_order_release);
        return result;
    }
    void return_frame(camera::FrameLease&& lease) noexcept {
        if (!lease.valid()) return;
        std::lock_guard lock(frame_mutex_);
        assert(state_.load(std::memory_order_acquire) == State::kHeld);
        frame_.lease = std::move(lease);
        status_->release_pending_ = 1;
        state_.store(State::kReturned, std::memory_order_release);
    }
    void complete() noexcept { std::lock_guard lock(frame_mutex_); idle_ = true; }
    void close() noexcept {
        std::lock_guard lock(frame_mutex_);
        accepting_ = false;
        status_->active_.store(false, std::memory_order_release);
    }
    void open() noexcept {
        std::lock_guard lock(frame_mutex_);
        assert(quiescent());
        sampler_.reset();
        accepting_ = idle_ = true;
        status_->active_.store(true, std::memory_order_release);
    }
    bool quiescent() const noexcept {
        return state_.load(std::memory_order_acquire) == State::kEmpty;
    }

    // Only the camera/capture thread calls this, including after recorder shutdown.
    // A failed release remains here for retry; the slot is not reused meanwhile.
    std::uint32_t drain_releases() noexcept {
        if (!prepared_) return 0;
        if (!releasing_.valid()) {
            std::unique_lock lock(frame_mutex_, std::try_to_lock);
            if (!lock.owns_lock() || state_.load(std::memory_order_acquire) != State::kReturned) return 0;
            releasing_ = std::move(frame_.lease);
            state_.store(State::kReleasing, std::memory_order_release);
        }
        if (const auto result = releasing_.release(); !result) {
            ++status_->lease_release_fail_;
            return 0;
        }
        status_->release_pending_ = 0;
        state_.store(State::kEmpty, std::memory_order_release);
        return 1;
    }
    std::uint32_t pending_releases() const noexcept {
        const auto state = state_.load(std::memory_order_acquire);
        return state == State::kReturned || state == State::kReleasing ? 1U : 0U;
    }
    bool pop_event(FrameEvent& event) noexcept {
        std::lock_guard lock(event_mutex_);
        if (event_count_ == 0) return false;
        event = events_[event_head_];
        event_head_ = (event_head_ + 1U) % config_.event_slots;
        --event_count_;
        return true;
    }
    void record_event(const FrameEvent& event) noexcept { enqueue_event(event); }

private:
    enum class State : std::uint8_t { kEmpty, kPending, kHeld, kReturned, kReleasing };
    void enqueue_event(const FrameEvent& event) noexcept {
        std::unique_lock lock(event_mutex_, std::try_to_lock);
        if (!lock.owns_lock() || event_count_ == config_.event_slots) {
            ++status_->event_dropped_;
            return;
        }
        events_[event_tail_] = event;
        event_tail_ = (event_tail_ + 1U) % config_.event_slots;
        ++event_count_;
    }
    Config config_{};
    Record::Sampler sampler_{};
    RecorderStatus* status_ = nullptr;
    platform::WriteControl* writes_ = nullptr;
    std::mutex frame_mutex_, event_mutex_;
    std::atomic<State> state_{State::kEmpty};
    Frame frame_{};
    camera::FrameLease releasing_{};
    std::unique_ptr<FrameEvent[]> events_{};
    std::uint32_t event_head_ = 0, event_tail_ = 0, event_count_ = 0;
    bool accepting_ = false, idle_ = false, prepared_ = false;
};

} // namespace dart::recorder
