#include "recorder/recorder.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <pthread.h>
#include <string_view>
#include <utility>

#include "camera/frame_lease.hpp"
#include "recorder/async/event_worker.hpp"
#include "recorder/config.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/event_writer.hpp"
#include "recorder/storage/image_writer.hpp"
#include "recorder/storage/session.hpp"
#include "system/write_control.hpp"

namespace dart::recorder {

std::string_view Recorder::decision_text(EventDecision decision) noexcept {
    return EventWriter::decision_text(decision);
}

bool Recorder::checked_frame_bytes(
    std::uint32_t width, std::uint32_t height, std::size_t& bytes) noexcept {
    if (width == 0 || height == 0 || width > kMaxFrameDimension || height > kMaxFrameDimension)
        return false;
    const auto w = static_cast<std::size_t>(width);
    const auto h = static_cast<std::size_t>(height);
    if (h > std::numeric_limits<std::size_t>::max() / w)
        return false;
    bytes = w * h;
    return true;
}

Recorder::Recorder(Config config) noexcept
    : config_(config) {}

Recorder::~Recorder() {
    stop();
    if (release_slots_) {
        for (std::uint32_t i = 0; i < image_capacity_; ++i)
            release_slots_[i] = camera::FrameLease{};
    }
}

bool Recorder::validate_config() noexcept {
    if (!checked_frame_bytes(config_.width, config_.height, frame_bytes_)) {
        set_error(Error::kInvalidConfig);
        return false;
    }
    if (config_.fps == 0 || config_.fps > 1000000 || config_.queue_slots == 0
        || config_.queue_slots > kMaxQueueSlots || config_.event_slots == 0
        || config_.event_slots > kMaxEventSlots || config_.output_directory.empty()
        || config_.output_directory.size() >= kMaxPath || config_.flush_period.count() < 0
        || config_.frame_timeout.count() < 0 || config_.frame_timeout.count() > 60'000
        || config_.keep_sessions > 9999 || config_.max_file_bytes == 0
        || config_.max_file_bytes > Config::kMaxFileBytes) {
        set_error(
            config_.output_directory.size() >= kMaxPath ? Error::kPathTooLong
                                                        : Error::kInvalidConfig);
        return false;
    }
    if (config_.storage_check_frames == 0 || config_.storage_check_frames > 32) {
        set_error(Error::kInvalidConfig);
        return false;
    }
    return true;
}

bool Recorder::allocate_buffers() noexcept {
    if (buffers_ready_)
        return true;
    image_capacity_ = config_.queue_slots;
    image_slots_.reset(new (std::nothrow) ImageSlot[image_capacity_]);
    release_slots_.reset(new (std::nothrow) camera::FrameLease[image_capacity_]);
    if (!image_slots_ || !release_slots_) {
        set_error(Error::kNoMemory);
        return false;
    }
    for (std::uint32_t i = 0; i < image_capacity_; ++i) {
        image_slots_[i].pixels.reset(new (std::nothrow) std::byte[frame_bytes_]);
        if (!image_slots_[i].pixels) {
            set_error(Error::kNoMemory);
            return false;
        }
    }
    if (!images_.prepare(frame_bytes_) || !events_.prepare()) {
        set_error(Error::kNoMemory);
        return false;
    }
    if (config_.max_file_bytes < images_.record_bytes()) {
        set_error(Error::kInvalidConfig);
        return false;
    }
    buffers_ready_ = true;
    return true;
}

bool Recorder::start() noexcept {
    std::lock_guard lifecycle(lifecycle_mutex_);
    if (image_thread_started_ || events_.started() || pending_releases() != 0) {
        set_error(Error::kAlreadyStarted);
        return false;
    }
    error_.reset();
    limit_reached_.store(false);
    started_once_.store(false);
    if (!validate_config() || !allocate_buffers() || !session_.open() || !images_.open(error_)
        || !events_.open(session_, error_)) {
        (void)images_.close();
        events_.stop();
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_head_ = image_tail_ = image_count_ = 0;
        release_head_ = release_tail_ = release_count_ = 0;
        next_sample_tick_.store(0, std::memory_order_relaxed);
        accepting_ = true;
    }
    // Statistics describe this file session, including its independent budget.
    input_ = accepted_ = copied_ = written_ = downsampled_ = busy_ = timed_out_ = stopped_ =
        invalid_ = 0;
    copy_fail_ = write_fail_ = lease_release_fail_ = bytes_ = copy_total_us_ = 0;
    pack_total_us_ = io_total_us_ = io_max_us_ = space_total_us_ = 0;
    open_total_us_ = write_total_us_ = close_total_us_ = 0;
    queue_high_water_ = 0;
    if (::pthread_create(&image_thread_, nullptr, &Recorder::image_thread_entry, this) != 0) {
        rollback_start();
        return false;
    }
    image_thread_started_ = true;
    if (!events_.start()) {
        rollback_start();
        return false;
    }
    started_once_.store(true, std::memory_order_release);
    return true;
}

void Recorder::rollback_start() noexcept {
    set_error(Error::kThread);
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        accepting_ = false;
    }
    image_cv_.notify_all();
    if (image_thread_started_) {
        (void)::pthread_join(image_thread_, nullptr);
        image_thread_started_ = false;
    }
    events_.stop();
    (void)images_.close();
}

void Recorder::stop() noexcept {
    {
        std::lock_guard lock(control_mutex_);
        controlling_.store(false);
    }
    control_wake_.notify_all();
    if (control_started_) {
        (void)::pthread_join(control_thread_, nullptr);
        control_started_ = false;
    }
    stop_session();
}

void Recorder::stop_session() noexcept {
    std::lock_guard lifecycle(lifecycle_mutex_);
    if (!started() && !image_thread_started_ && !events_.started())
        return;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        accepting_ = false;
    }
    image_cv_.notify_all();
    if (image_thread_started_) {
        (void)::pthread_join(image_thread_, nullptr);
        image_thread_started_ = false;
    }
    // Variable-size image metadata is produced by the image writer. Keep
    // its consumer alive until the image worker has drained its queue.
    events_.stop();
}

bool Recorder::start_controlled(platform::WriteControl& writes) noexcept {
    if (control_started_ || image_thread_started_ || !validate_config() || !allocate_buffers())
        return false;
    writes_ = &writes;
    controlling_.store(true);
    if (::pthread_create(&control_thread_, nullptr, &Recorder::control_entry, this) != 0) {
        controlling_.store(false);
        set_error(Error::kThread);
        return false;
    }
    control_started_ = true;
    return true;
}

void* Recorder::control_entry(void* context) noexcept {
    auto& self = *static_cast<Recorder*>(context);
    bool attempted = false;
    while (self.controlling_.load()) {
        const bool wanted = self.writes_->recording_enabled();
        if (!wanted) {
            self.stop_session();
            attempted = false;
        } else if (!attempted && self.pending_releases() == 0) {
            // A failed or full session stays stopped until a fresh request.
            attempted = true;
            (void)self.start();
        } else if (self.limit_reached() || self.last_error() != Error::kNone) {
            self.stop_session();
        }
        std::unique_lock lock(self.control_mutex_);
        self.control_wake_.wait_for(
            lock, std::chrono::milliseconds{20}, [&] { return !self.controlling_.load(); });
    }
    return nullptr;
}

bool Recorder::started() const noexcept {
    std::lock_guard<std::mutex> lock(image_mutex_);
    return accepting_;
}

Admission Recorder::try_submit(camera::FrameLease&& lease) noexcept {
    std::unique_lock lifecycle(lifecycle_mutex_, std::try_to_lock);
    if (!lifecycle.owns_lock())
        return Admission::kBusy;
    if (writes_ != nullptr && !writes_->recording_enabled())
        return Admission::kStopped;
    const std::uint64_t input_index = input_.fetch_add(1, std::memory_order_relaxed) + 1;
    const auto* view_ptr = lease.valid() ? &lease.view() : nullptr;
    FrameEvent event{};
    event.input_index = input_index;
    if (view_ptr != nullptr) {
        event.source_sequence = view_ptr->sequence;
        event.source_pts = view_ptr->source_pts;
        event.received_us = view_ptr->received.count();
        event.source_pts_timebase_validated = view_ptr->source_pts_timebase_validated;
    }

    const auto finish = [&](Admission admission, EventDecision decision) noexcept {
        event.decision = decision;
        if (admission == Admission::kAccepted)
            accepted_.fetch_add(1, std::memory_order_relaxed);
        else if (admission == Admission::kDownsampled)
            downsampled_.fetch_add(1, std::memory_order_relaxed);
        else if (admission == Admission::kBusy)
            busy_.fetch_add(1, std::memory_order_relaxed);
        else if (admission == Admission::kStopped)
            stopped_.fetch_add(1, std::memory_order_relaxed);
        else if (admission == Admission::kInvalidFrame)
            invalid_.fetch_add(1, std::memory_order_relaxed);
        (void)events_.enqueue(event);
        return admission;
    };

    if (view_ptr == nullptr)
        return finish(Admission::kInvalidFrame, EventDecision::kInvalidFrame);
    if (!started_once_.load(std::memory_order_acquire) || limit_reached())
        return finish(Admission::kStopped, EventDecision::kStopped);
    if (!images_.valid_view(*view_ptr) || event.received_us < 0)
        return finish(Admission::kInvalidFrame, EventDecision::kInvalidFrame);

    /* One image per 1/fps CLOCK_MONOTONIC interval, independent of input FPS.
     * Split seconds and microseconds to avoid overflow and integer-period drift.
     * Advance before the queue check: missed/busy intervals are never caught up
     * by a burst of writes when capture or storage resumes. */
    const auto received_us = static_cast<std::uint64_t>(event.received_us);
    const std::uint64_t tick =
        (received_us / 1000000U) * config_.fps + (received_us % 1000000U) * config_.fps / 1000000U;
    auto next_tick = next_sample_tick_.load(std::memory_order_relaxed);
    if (tick < next_tick
        || !next_sample_tick_.compare_exchange_strong(
            next_tick, tick + 1U, std::memory_order_relaxed))
        return finish(Admission::kDownsampled, EventDecision::kDownsampled);

    /* No condition-variable wait, and even the queue mutex is try-locked. */
    std::unique_lock<std::mutex> lock(image_mutex_, std::try_to_lock);
    /* Pixel slots and pending camera releases have separate bounded queues.
     * Copied pixels may wait for disk without retaining camera lease slots. */
    if (!lock.owns_lock())
        return finish(Admission::kBusy, EventDecision::kBusy);
    const bool full = image_count_ >= image_capacity_ || release_count_ >= image_capacity_;
    if (!accepting_ || full) {
        return finish(
            !accepting_ ? Admission::kStopped : Admission::kBusy,
            !accepting_ ? EventDecision::kStopped : EventDecision::kBusy);
    }
    ImageSlot& slot = image_slots_[image_tail_];
    const auto copy_begin = std::chrono::steady_clock::now();
    const bool copied = images_.copy_image({slot.pixels.get(), frame_bytes_}, *view_ptr);
    copy_total_us_.fetch_add(
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now() - copy_begin)
                                       .count()),
        std::memory_order_relaxed);
    if (!copied) {
        copy_fail_.fetch_add(1, std::memory_order_relaxed);
        lock.unlock();
        return finish(Admission::kInvalidFrame, EventDecision::kInvalidFrame);
    }
    copied_.fetch_add(1, std::memory_order_relaxed);
    slot.event = event;
    slot.event.decision = EventDecision::kAccepted;
    slot.queued_at = copy_begin;
    release_slots_[release_tail_] = std::move(lease);
    release_tail_ = (release_tail_ + 1U) % image_capacity_;
    ++release_count_;
    image_tail_ = (image_tail_ + 1U) % image_capacity_;
    ++image_count_;
    const auto high = std::max(queue_high_water_.load(std::memory_order_relaxed), image_count_);
    queue_high_water_.store(high, std::memory_order_relaxed);
    lock.unlock();
    accepted_.fetch_add(1, std::memory_order_relaxed);
    image_cv_.notify_one();
    return Admission::kAccepted;
}

std::uint32_t Recorder::drain_releases() noexcept {
    std::uint32_t drained = 0;
    for (;;) {
        camera::FrameLease lease{};
        {
            std::unique_lock<std::mutex> lock(image_mutex_, std::try_to_lock);
            if (!lock.owns_lock() || release_count_ == 0)
                break;
            camera::FrameLease& slot = release_slots_[release_head_];
            lease = std::move(slot);
            release_head_ = (release_head_ + 1U) % image_capacity_;
            --release_count_;
        }
        if (lease.valid()) {
            auto released = lease.release();
            if (!released)
                lease_release_fail_.fetch_add(1, std::memory_order_relaxed);
        }
        ++drained;
    }
    return drained;
}

std::uint32_t Recorder::pending_releases() const noexcept {
    std::lock_guard<std::mutex> lock(image_mutex_);
    return release_count_;
}

void* Recorder::image_thread_entry(void* context) noexcept {
    static_cast<Recorder*>(context)->image_loop();
    return nullptr;
}

void Recorder::image_loop() noexcept {
    const auto timeout =
        config_.frame_timeout.count() == 0
            ? std::chrono::microseconds{(1'000'000U + config_.fps - 1U) / config_.fps}
            : std::chrono::duration_cast<std::chrono::microseconds>(config_.frame_timeout);
    bool write_failed = false;
    for (;;) {
        ImageSlot* slot = nullptr;
        {
            std::unique_lock<std::mutex> lock(image_mutex_);
            image_cv_.wait(lock, [this] { return image_count_ != 0 || !accepting_; });
            if (image_count_ == 0 && !accepting_)
                break;
            slot = &image_slots_[image_head_];
        }
        FrameEvent event = slot->event;
        if (write_failed) {
            event.decision = EventDecision::kStopped;
        } else if (std::chrono::steady_clock::now() - slot->queued_at >= timeout) {
            event.decision = EventDecision::kTimedOut;
            timed_out_.fetch_add(1, std::memory_order_relaxed);
        } else {
            // Output indices and offsets describe saved images only. Expired
            // queue entries consume neither file budget nor playback indices.
            event.image_index = written_.load(std::memory_order_relaxed) + 1U;
            event.file_offset = bytes_.load(std::memory_order_relaxed);
            if (images_.write_image({slot->pixels.get(), frame_bytes_}, event.image_index)) {
                pack_total_us_.fetch_add(images_.pack_us(), std::memory_order_relaxed);
                io_total_us_.fetch_add(images_.io_us(), std::memory_order_relaxed);
                space_total_us_.fetch_add(images_.space_us(), std::memory_order_relaxed);
                open_total_us_.fetch_add(images_.open_us(), std::memory_order_relaxed);
                write_total_us_.fetch_add(images_.write_us(), std::memory_order_relaxed);
                close_total_us_.fetch_add(images_.close_us(), std::memory_order_relaxed);
                io_max_us_.store(
                    std::max(io_max_us_.load(std::memory_order_relaxed), images_.io_us()),
                    std::memory_order_relaxed);
                written_.fetch_add(1, std::memory_order_relaxed);
                bytes_.fetch_add(images_.record_bytes(), std::memory_order_relaxed);
                event.file_bytes = images_.record_bytes();
                if (config_.variable_record_size()) {
                    event.file_offset = images_.file_offset();
                    (void)std::snprintf(
                        event.image_name.data(), event.image_name.size(), "%s",
                        images_.image_name());
                } else {
                    (void)std::snprintf(
                        event.image_name.data(), event.image_name.size(), "img/recording.%s",
                        config_.image_format == ImageFormat::kPbm ? "pbm" : "pgm");
                }
                const auto remaining =
                    session_.file_limit_bytes() - bytes_.load(std::memory_order_relaxed);
                if (remaining == 0
                    || (!config_.variable_record_size() && remaining < images_.record_bytes())) {
                    limit_reached_.store(true, std::memory_order_release);
                    write_failed = true;
                }
            } else {
                if (errno == EFBIG) {
                    limit_reached_.store(true, std::memory_order_release);
                } else {
                    write_fail_.fetch_add(1, std::memory_order_relaxed);
                    error_.set_file_error(
                        errno == ENOSPC ? Error::kStorage : Error::kImageFile,
                        session_.image_path());
                }
                event.decision = EventDecision::kStopped;
                event.image_index = 0;
                event.file_offset = 0;
                write_failed = true;
            }
        }
        (void)events_.enqueue(event);
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            image_head_ = (image_head_ + 1U) % image_capacity_;
            --image_count_;
            if (write_failed)
                accepting_ = false;
        }
        image_cv_.notify_all();
    }
    if (!images_.close()) {
        write_fail_.fetch_add(1, std::memory_order_relaxed);
        error_.set_file_error(Error::kImageFile, session_.image_path());
    }
}

std::string_view Recorder::error_text(Error error) noexcept {
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

Recorder::Stats Recorder::stats() const noexcept {
    Stats result{};
    result.input = input_.load(std::memory_order_relaxed);
    result.accepted = accepted_.load(std::memory_order_relaxed);
    result.copied = copied_.load(std::memory_order_relaxed);
    result.written = written_.load(std::memory_order_relaxed);
    result.downsampled = downsampled_.load(std::memory_order_relaxed);
    result.busy = busy_.load(std::memory_order_relaxed);
    result.timed_out = timed_out_.load(std::memory_order_relaxed);
    result.stopped = stopped_.load(std::memory_order_relaxed);
    result.invalid = invalid_.load(std::memory_order_relaxed);
    result.copy_fail = copy_fail_.load(std::memory_order_relaxed);
    result.write_fail = write_fail_.load(std::memory_order_relaxed);
    result.event_dropped = events_.dropped();
    result.lease_release_fail = lease_release_fail_.load(std::memory_order_relaxed);
    result.bytes = bytes_.load(std::memory_order_relaxed);
    result.copy_total_us = copy_total_us_.load(std::memory_order_relaxed);
    result.pack_total_us = pack_total_us_.load(std::memory_order_relaxed);
    result.io_total_us = io_total_us_.load(std::memory_order_relaxed);
    result.io_max_us = io_max_us_.load(std::memory_order_relaxed);
    result.space_total_us = space_total_us_.load(std::memory_order_relaxed);
    result.open_total_us = open_total_us_.load(std::memory_order_relaxed);
    result.write_total_us = write_total_us_.load(std::memory_order_relaxed);
    result.close_total_us = close_total_us_.load(std::memory_order_relaxed);
    result.queue_high_water = queue_high_water_.load(std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        result.queued = image_count_;
        result.release_pending = release_count_;
    }
    return result;
}

} // namespace dart::recorder
