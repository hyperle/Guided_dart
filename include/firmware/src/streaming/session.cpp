#include "streaming/session.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <memory>
#include <mutex>
#include <new>
#include <pthread.h>
#include <sys/stat.h>
#include <utility>

#include "streaming/jpeg.h"

#include "camera/frame_view.hpp"
#include "streaming/interface.hpp"

namespace dart::streaming {

const char* state_text(State state) noexcept {
    switch (state) {
    case State::kStopped: return "stopped";
    case State::kStarting: return "starting";
    case State::kRunning: return "running";
    case State::kStopping: return "stopping";
    case State::kPaused: return "paused";
    case State::kFailed: return "failed";
    }
    return "unknown";
}

// The owner serializes start/start_controlled/stop. Controlled start gives
// worker ownership to the supervisor; capture only borrows pixels through
// try_submit. It never joins, waits for a device, or touches control files.
bool Session::start(
    const char* path, std::uint32_t width, std::uint32_t height,
    TransportFactory factory) noexcept {
    std::lock_guard lifecycle(lifecycle_mutex_);
    if (!prepare_source(width, height, factory) || !load_configuration(path))
        return false;
    return start_workers();
}

bool Session::start_controlled(
    const char* path, std::uint32_t width, std::uint32_t height, TransportFactory factory,
    const char* pause_path) noexcept {
    std::lock_guard lifecycle(lifecycle_mutex_);
    if (pause_path == nullptr || pause_path[0] == '\0'
        || std::strlen(pause_path) >= sizeof(pause_path_) || path == nullptr || path[0] == '\0'
        || std::strlen(path) >= sizeof(config_path_)) {
        set_error({Code::kConfig, EINVAL});
        return false;
    }
    if (!prepare_source(width, height, factory))
        return false;
    std::strcpy(pause_path_, pause_path);
    std::strcpy(config_path_, path);
    state_.store(State::kStarting);
    supervisor_running_.store(true);
    const int result = pthread_create(&supervisor_, nullptr, &supervisor_entry, this);
    if (result != 0) {
        supervisor_running_.store(false);
        state_.store(State::kFailed);
        set_error({Code::kThread, result});
        return false;
    }
    supervisor_started_ = true;
    return true;
}

bool Session::try_submit(const camera::FrameView& frame) noexcept {
    if (!accepting_.load())
        return false;
    // The supervisor owns worker creation/destruction. A frame never waits
    // behind initialization or teardown, and buffers cannot be freed during
    // this copy. Queue contention is also handled by try_lock below.
    std::unique_lock lifecycle(lifecycle_mutex_, std::try_to_lock);
    if (!lifecycle.owns_lock()) {
        ++counts_[kBusy];
        return false;
    }
    if (!accepting_.load())
        return false;
    if (frame.width != width_ || frame.height != height_ || frame.stride < frame.width
        || frame.luma.size() < static_cast<std::uint64_t>(height_ - 1) * frame.stride + width_
        || frame.received.count() < 0) {
        ++counts_[kInvalid];
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < next_frame_) {
        ++counts_[kDownsampled];
        return false;
    }
    std::unique_lock lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        ++counts_[kBusy];
        return false;
    }
    const int index = raw_pending_ >= 0 ? raw_pending_ : !raw_busy_[0] ? 0 : 1;
    if (raw_pending_ >= 0)
        ++counts_[kRawReplaced];
    auto& slot = raw_[index];
    for (std::uint32_t row = 0; row < height_; ++row)
        std::memcpy(
            slot.pixels.get() + static_cast<std::size_t>(row) * width_,
            frame.luma.data() + static_cast<std::size_t>(row) * frame.stride, width_);
    slot.sequence = frame.sequence;
    slot.received_us = static_cast<std::uint64_t>(frame.received.count());
    raw_pending_ = index;
    next_frame_ = now + std::chrono::microseconds{1000000 / config_.fps};
    ++counts_[kAccepted];
    lock.unlock();
    wake_.notify_all();
    return true;
}

void Session::stop() noexcept {
    accepting_.store(false);
    {
        std::lock_guard lock(supervisor_mutex_);
        supervisor_running_.store(false);
    }
    supervisor_wake_.notify_all();
    if (supervisor_started_) {
        (void)pthread_join(supervisor_, nullptr);
        supervisor_started_ = false;
    }
    std::lock_guard lifecycle(lifecycle_mutex_);
    stop_workers();
    state_.store(State::kStopped);
}

Session::Stats Session::stats() const noexcept {
    return {
        counts_[kAccepted].load(),
        counts_[kDownsampled].load(),
        counts_[kBusy].load(),
        counts_[kInvalid].load(),
        counts_[kRawReplaced].load(),
        counts_[kJpegReplaced].load(),
        counts_[kEncoded].load(),
        counts_[kJpegErrors].load(),
        counts_[kUploaded].load(),
        counts_[kBytes].load(),
        counts_[kNetworkErrors].load(),
        counts_[kStaleDropped].load(),
        connected_.load(),
        last_native_.load(),
        buffer_bytes_.load()};
}

Error Session::error() const noexcept {
    const auto value = error_.load();
    return {static_cast<Code>(value >> 32), static_cast<std::int32_t>(value)};
}

void Session::set_error(Error error) noexcept {
    error_.store(
        (static_cast<std::uint64_t>(error.code) << 32) | static_cast<std::uint32_t>(error.native));
}

bool Session::prepare_source(
    std::uint32_t width, std::uint32_t height, TransportFactory factory) noexcept {
    if (supervisor_started_ || compressor_started_ || sender_started_) {
        set_error({Code::kThread, EBUSY});
        return false;
    }
    if (factory == nullptr) {
        set_error({Code::kBackend, ENOSYS});
        return false;
    }
    if (width == 0 || height == 0 || width > 4096 || height > 4096) {
        set_error({Code::kConfig, EINVAL});
        return false;
    }
    factory_ = factory;
    width_ = width;
    height_ = height;
    for (auto& count : counts_)
        count.store(0);
    last_native_.store(0);
    set_error({});
    return true;
}

bool Session::load_configuration(const char* path) noexcept {
    auto parsed = load_config(path);
    if (!parsed) {
        set_error(parsed.error());
        return false;
    }
    if (parsed->width > width_ || parsed->height > height_) {
        set_error({Code::kConfig, EINVAL});
        return false;
    }
    config_ = *parsed;
    jpeg_capacity_ = static_cast<std::size_t>(config_.width) * config_.height * 4 + 4096;
    set_error({});
    return true;
}

// Requires lifecycle_mutex_. These calls may wait for worker/device exit;
// the capture path exclusively uses try_lock and never waits on this lock.
bool Session::start_workers() noexcept {
    state_.store(State::kStarting);
    transport_ = factory_();
    if (!transport_) {
        set_error({Code::kBackend, ENOSYS});
        state_.store(State::kFailed);
        return false;
    }
    const auto pixels = static_cast<std::size_t>(width_) * height_;
    for (auto& raw : raw_)
        raw.pixels.reset(new (std::nothrow) std::uint8_t[pixels]);
    for (auto& jpeg : jpeg_)
        jpeg.pixels.reset(new (std::nothrow) std::uint8_t[jpeg_capacity_]);
    if (!raw_[0].pixels || !raw_[1].pixels || !jpeg_[0].pixels || !jpeg_[1].pixels) {
        set_error({Code::kMemory, ENOMEM});
        stop_workers();
        state_.store(State::kFailed);
        return false;
    }
    buffer_bytes_.store(2 * pixels + 2 * jpeg_capacity_);
    (void)dart_jpeg_init(&encoder_, config_.quality);
    next_frame_ = {};
    raw_pending_ = jpeg_pending_ = sending_ = -1;
    raw_busy_.fill(false);
    stopping_.store(false);
    const int compressor_result = pthread_create(&compressor_, nullptr, &compress_entry, this);
    if (compressor_result != 0) {
        set_error({Code::kThread, compressor_result});
        stop_workers();
        state_.store(State::kFailed);
        return false;
    }
    compressor_started_ = true;
    const int sender_result = pthread_create(&sender_, nullptr, &send_entry, this);
    if (sender_result != 0) {
        set_error({Code::kThread, sender_result});
        stop_workers();
        state_.store(State::kFailed);
        return false;
    }
    sender_started_ = true;
    set_error({});
    state_.store(State::kRunning);
    accepting_.store(true);
    return true;
}

void Session::stop_workers() noexcept {
    accepting_.store(false);
    {
        std::lock_guard lock(mutex_);
        stopping_.store(true);
    }
    wake_.notify_all();
    if (compressor_started_) {
        (void)pthread_join(compressor_, nullptr);
        compressor_started_ = false;
    }
    if (sender_started_) {
        (void)pthread_join(sender_, nullptr);
        sender_started_ = false;
    }
    connected_.store(false);
    transport_.reset();
    for (auto& slot : raw_)
        slot.pixels.reset();
    for (auto& slot : jpeg_)
        slot.pixels.reset();
    buffer_bytes_.store(0);
}

bool Session::paused() const noexcept {
    struct stat status;
    return ::lstat(pause_path_, &status) == 0 || errno != ENOENT;
}

void* Session::supervisor_entry(void* context) noexcept {
    static_cast<Session*>(context)->supervise();
    return nullptr;
}

void Session::supervise() noexcept {
    while (supervisor_running_.load()) {
        if (paused()) {
            // Stop admission before any potentially blocking driver cleanup.
            accepting_.store(false);
            std::lock_guard lifecycle(lifecycle_mutex_);
            if (state_.load() != State::kPaused) {
                state_.store(State::kStopping);
                stop_workers();
                state_.store(State::kPaused);
            }
        } else if (state_.load() == State::kStarting || state_.load() == State::kPaused) {
            std::lock_guard lifecycle(lifecycle_mutex_);
            if (supervisor_running_.load()) {
                if (load_configuration(config_path_))
                    (void)start_workers();
                else
                    state_.store(State::kFailed);
            }
        }
        // A startup failure is latched. An explicit off/on retries it;
        // invalid hardware/configuration cannot cause a tight restart loop.
        std::unique_lock lock(supervisor_mutex_);
        supervisor_wake_.wait_for(
            lock, std::chrono::milliseconds{50}, [&] { return !supervisor_running_.load(); });
    }
    accepting_.store(false);
    std::lock_guard lifecycle(lifecycle_mutex_);
    state_.store(State::kStopping);
    stop_workers();
    state_.store(State::kStopped);
}

void* Session::compress_entry(void* context) noexcept {
    static_cast<Session*>(context)->compress();
    return nullptr;
}

void* Session::send_entry(void* context) noexcept {
    static_cast<Session*>(context)->send();
    return nullptr;
}

void Session::compress() noexcept {
    while (!stopping_.load()) {
        std::unique_lock lock(mutex_);
        wake_.wait(lock, [&] { return stopping_.load() || raw_pending_ >= 0; });
        if (stopping_.load())
            break;
        const int raw_index = std::exchange(raw_pending_, -1);
        raw_busy_[raw_index] = true;
        const int jpeg_index = jpeg_pending_ >= 0 ? jpeg_pending_ : sending_ == 0 ? 1 : 0;
        if (jpeg_pending_ >= 0)
            ++counts_[kJpegReplaced];
        jpeg_pending_ = -1;
        lock.unlock();
        auto& input = raw_[raw_index];
        auto& output = jpeg_[jpeg_index];
        output.sequence = input.sequence;
        output.received_us = input.received_us;
        const auto result = dart_jpeg_encode(
            &encoder_, input.pixels.get(), static_cast<std::size_t>(width_) * height_, width_,
            height_, width_, config_.width, config_.height, output.pixels.get(), jpeg_capacity_,
            &output.size);
        lock.lock();
        raw_busy_[raw_index] = false;
        if (result == DART_STATUS_OK) {
            jpeg_pending_ = jpeg_index;
            ++counts_[kEncoded];
        } else {
            ++counts_[kJpegErrors];
        }
        lock.unlock();
        wake_.notify_all();
    }
}

void Session::send() noexcept {
    unsigned backoff_ms = 250;
    while (!stopping_.load()) {
        std::unique_lock lock(mutex_);
        wake_.wait(lock, [&] { return stopping_.load() || jpeg_pending_ >= 0; });
        if (stopping_.load())
            break;
        sending_ = std::exchange(jpeg_pending_, -1);
        const int index = sending_;
        lock.unlock();
        std::expected<void, Error> result{};
        if (!connected_.load())
            result = transport_->open(config_, stopping_);
        if (result && !stopping_.load()) {
            connected_.store(true);
            // An association/reconnect may take seconds. Send only if the
            // copied frame is still within the configured I/O age budget.
            const auto age = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count()
                           - static_cast<std::int64_t>(jpeg_[index].received_us);
            if (age >= 0 && age <= static_cast<std::int64_t>(config_.io_timeout_ms) * 1000) {
                const auto& slot = jpeg_[index];
                result = transport_->upload(
                    {slot.pixels.get(), slot.size}, slot.sequence, slot.received_us, stopping_);
                if (result) {
                    ++counts_[kUploaded];
                    counts_[kBytes] += slot.size;
                }
            } else {
                ++counts_[kStaleDropped];
            }
        }
        if (!result) {
            if (!stopping_.load()) {
                ++counts_[kNetworkErrors];
                last_native_.store(result.error().native);
            }
            transport_->close();
            connected_.store(false);
        }
        lock.lock();
        sending_ = -1;
        if (!result) {
            wake_.wait_for(
                lock, std::chrono::milliseconds{backoff_ms}, [&] { return stopping_.load(); });
            backoff_ms = backoff_ms < 8000 ? backoff_ms * 2 : 8000;
        } else {
            backoff_ms = 250;
        }
    }
    transport_->close();
}

} // namespace dart::streaming
