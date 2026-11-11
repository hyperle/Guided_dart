#include "src/recorder/buffered_storage.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <new>
#include <span>

namespace dart::recorder {

BufferedStorage::BufferedStorage(const Config& config, FrameChannel& frames, RecorderStatus& status,
                                 platform::WriteControl* writes) noexcept
    : config_(config), frames_(frames), status_(status), writes_(writes), storage_(config) {}

BufferedStorage::~BufferedStorage() { close(); }

bool BufferedStorage::prepare(std::size_t input_bytes, std::size_t output_bytes) noexcept {
    if (started_ || output_bytes == 0 || config_.queue_slots == 0 || config_.queue_slots > 64)
        return false;
    slots_.reset(new (std::nothrow) Slot[config_.queue_slots]);
    if (!slots_) return false;
    slot_bytes_ = output_bytes;
    for (std::uint32_t i = 0; i < config_.queue_slots; ++i) {
        slots_[i].bytes.reset(new (std::nothrow) std::byte[slot_bytes_]);
        if (!slots_[i].bytes) return false;
    }
    return storage_.prepare(input_bytes);
}

bool BufferedStorage::open() noexcept {
    if (started_ || !slots_) return false;
    produced_ = consumed_ = 0;
    storage_.error().reset();
    if (!storage_.open()) {
        publish_details();
        close_files();
        return false;
    }
    status_.bytes_ = storage_.images().bytes_written();
    publish_details();
    running_.store(true, std::memory_order_release);
    if (::pthread_create(&thread_, nullptr, &BufferedStorage::entry, this) != 0) {
        running_ = false;
        status_.set_error(Error::kThread);
        close_files();
        return false;
    }
    started_ = true;
    return true;
}

void BufferedStorage::close() noexcept {
    running_.store(false, std::memory_order_release);
    ready_.notify_one();
    if (started_) {
        (void)::pthread_join(thread_, nullptr);
        started_ = false;
    }
}

bool BufferedStorage::available() const noexcept {
    return running_.load(std::memory_order_acquire)
        && status_.buffered_.load(std::memory_order_acquire) < config_.queue_slots;
}

bool BufferedStorage::submit(const EncodedImage& image, const FrameEvent& event) noexcept {
    if (!available()) { errno = EAGAIN; return false; }
    if (!image.valid() || image.size() > slot_bytes_) { errno = EOVERFLOW; return false; }
    const auto begin = std::chrono::steady_clock::now();
    const auto tail = produced_.load(std::memory_order_relaxed);
    auto& slot = slots_[tail % config_.queue_slots];
    auto* data = slot.bytes.get();
    if (!image.header.empty()) std::memcpy(data, image.header.data(), image.header.size());
    auto* pixels = data + image.header.size();
    if (image.borrowed_input) {
        for (std::uint32_t row = 0; row < image.height; ++row)
            std::memcpy(pixels + static_cast<std::size_t>(row) * image.width,
                        image.pixels.data() + static_cast<std::size_t>(row) * image.stride, image.width);
    } else {
        std::memcpy(pixels, image.pixels.data(), image.pixels.size());
    }
    slot.image = {std::span<const std::byte>{data, image.header.size()},
                  std::span<const std::byte>{pixels, image.payload_bytes()}};
    slot.event = event;
    const auto elapsed = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - begin).count());
    status_.buffer_copy_total_us_.fetch_add(elapsed, std::memory_order_relaxed);
    status_.buffer_copy_max_us_ = std::max(status_.buffer_copy_max_us_.load(), elapsed);
    ++status_.enqueued_;
    const auto buffered = status_.buffered_.fetch_add(1, std::memory_order_acq_rel) + 1U;
    status_.buffer_high_water_ = std::max(status_.buffer_high_water_.load(), buffered);
    produced_.store(tail + 1U, std::memory_order_release);
    ready_.notify_one();
    return true;
}

void BufferedStorage::fail(int system_error, Error error, const char* path) noexcept {
    if (system_error == 0) system_error = EIO;
    ++status_.write_fail_;
    {
        std::lock_guard lock(status_.details_mutex_);
        auto& details = status_.details_;
        details.system_error = system_error;
        (void)std::snprintf(details.error_path.data(), details.error_path.size(), "%s",
                            path == nullptr ? storage_.session().image_path() : path);
        status_.set_error(system_error == ENOSPC ? Error::kStorage : error);
    }
    frames_.close();
}

void* BufferedStorage::entry(void* context) noexcept {
    static_cast<BufferedStorage*>(context)->run();
    return nullptr;
}

void BufferedStorage::run() noexcept {
    for (;;) {
        const auto head = consumed_.load(std::memory_order_relaxed);
        const bool have_image = head != produced_.load(std::memory_order_acquire);
        if (have_image) {
            write(slots_[head % config_.queue_slots]);
            // Release capacity only after all reads of this slot have ended.
            // The published positions independently delimit ready payloads.
            status_.buffered_.fetch_sub(1, std::memory_order_release);
            consumed_.store(head + 1U, std::memory_order_release);
        }
        drain_events();
        if (!running_.load(std::memory_order_acquire)
            && consumed_.load(std::memory_order_relaxed) == produced_.load(std::memory_order_acquire))
            break;
        if (!have_image) {
            std::unique_lock lock(wake_mutex_);
            ready_.wait_for(lock, std::chrono::milliseconds{4}, [this] {
                return !running_.load(std::memory_order_acquire)
                    || consumed_.load(std::memory_order_relaxed) != produced_.load(std::memory_order_acquire);
            });
        }
    }
    drain_events();
    close_files();
}

void BufferedStorage::write(Slot& slot) noexcept {
    auto& event = slot.event;
    event.decision = EventDecision::kStopped;
    if (!status_.limit_reached() && status_.last_error() == Error::kNone
        && (writes_ == nullptr || writes_->recording_enabled())) {
        const auto index = status_.written_.load(std::memory_order_relaxed) + 1U;
        auto& images = storage_.images();
        ++status_.write_attempts_;
        const auto begin = std::chrono::steady_clock::now();
        const bool written = images.write_encoded(slot.image, index);
        const int system_error = errno;
        const auto elapsed = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - begin).count());
        status_.io_total_us_.fetch_add(elapsed, std::memory_order_relaxed);
        status_.space_total_us_.fetch_add(images.space_us(), std::memory_order_relaxed);
        status_.open_total_us_.fetch_add(images.open_us(), std::memory_order_relaxed);
        status_.write_total_us_.fetch_add(images.write_us(), std::memory_order_relaxed);
        status_.close_total_us_.fetch_add(images.close_us(), std::memory_order_relaxed);
        status_.io_max_us_ = std::max(status_.io_max_us_.load(), elapsed);
        if (written) {
            ++status_.written_;
            status_.bytes_ = images.bytes_written();
            event.decision = EventDecision::kAccepted;
            event.image_index = index;
            event.file_offset = images.file_offset();
            event.file_bytes = images.record_bytes();
            (void)std::snprintf(event.image_name.data(), event.image_name.size(), "%s", images.image_name());
            const auto remaining = storage_.session().file_limit_bytes() - images.bytes_written();
            if (remaining == 0 || (!config_.variable_record_size() && remaining < images.record_bytes()))
                status_.limit_ = true;
        } else if (system_error == EFBIG) {
            status_.limit_ = true;
        } else {
            event.decision = EventDecision::kWriteError;
            fail(system_error);
        }
        if (status_.limit_reached()) frames_.close();
    }
    if (event.decision == EventDecision::kStopped) ++status_.stopped_;
    write_event(event);
}

void BufferedStorage::write_event(const FrameEvent& event) noexcept {
    if (storage_.events().write(event)) return;
    ++status_.event_dropped_;
    if (status_.last_error() == Error::kNone)
        fail(errno, Error::kEventFile, storage_.session().event_path());
}

void BufferedStorage::drain_events() noexcept {
    FrameEvent event{};
    for (std::uint32_t i = 0; i < config_.event_slots && frames_.pop_event(event); ++i)
        write_event(event);
}

void BufferedStorage::close_files() noexcept {
    if (!storage_.images().close()) fail(errno);
    if (!storage_.events().close() && status_.last_error() == Error::kNone)
        fail(errno, Error::kEventFile, storage_.session().event_path());
}

void BufferedStorage::publish_details() noexcept {
    std::lock_guard lock(status_.details_mutex_);
    auto& details = status_.details_;
    details.system_error = storage_.error().last_system_error();
    (void)std::snprintf(details.error_path.data(), details.error_path.size(), "%s", storage_.error().last_error_path());
    (void)std::snprintf(details.directory.data(), details.directory.size(), "%s", storage_.session().directory());
    (void)std::snprintf(details.recording_path.data(), details.recording_path.size(), "%s", storage_.session().image_path());
    details.file_limit_bytes = storage_.session().file_limit_bytes();
    status_.set_error(storage_.error().last_error());
}

} // namespace dart::recorder
