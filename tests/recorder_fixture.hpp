#pragma once

#include "runtime/tunner.hpp"

#include <cassert>
#include <chrono>
#include <cstdint>
#include <thread>
#include <utility>

#include "src/recorder/frame_channel.hpp"
#include "src/recorder/encode.hpp"
#include "src/recorder/storage.hpp"

namespace dart::tests {

// Exercise file adapters together with the production encoder without a worker.
class ImageWriterFixture final {
public:
    ImageWriterFixture(const recorder::Config& config, const recorder::Session& session) noexcept
        : config_(config), encoder_(config), writer_(config, session) {}
    bool prepare(std::size_t bytes) noexcept { return encoder_.prepare() && writer_.prepare(bytes); }
    bool open(recorder::RecorderError& error) noexcept { encoder_.reset(); return writer_.open(error); }
    bool close() noexcept { return writer_.close(); }
    bool write_image(std::span<const std::byte> pixels, std::uint64_t index = 1) noexcept {
        camera::FrameView view{pixels, config_.width, config_.height, config_.width};
        return writer_.write_encoded(encoder_.encode(view), index);
    }
    std::size_t payload_bytes() const noexcept { return writer_.payload_bytes(); }
    std::size_t record_bytes() const noexcept { return writer_.record_bytes(); }
    std::uint64_t file_offset() const noexcept { return writer_.file_offset(); }
    std::uint64_t bytes_written() const noexcept { return writer_.bytes_written(); }
    const char* image_name() const noexcept { return writer_.image_name(); }
private:
    const recorder::Config& config_;
    recorder::Encoder encoder_;
    recorder::ImageWriter writer_;
};

// Test driver for the real component and its public channels.
class RecorderFixture final {
public:
    explicit RecorderFixture(recorder::Config config) noexcept : config_(config) {}
    ~RecorderFixture() { stop(); (void)drain_releases(); }
    bool start() noexcept { return launch(nullptr, true); }
    bool start_controlled(platform::WriteControl& writes) noexcept { return launch(&writes, false); }
    void stop() noexcept { runner_.stop(); }
    recorder::Admission try_submit(camera::FrameLease&& lease) noexcept { return frames_.try_submit(std::move(lease)); }
    std::uint32_t drain_releases() noexcept { return frames_.drain_releases(); }
    std::uint32_t pending_releases() const noexcept { return frames_.pending_releases(); }
    void wait_released() noexcept {
        for (unsigned attempt = 0; attempt < 5000; ++attempt) {
            (void)drain_releases();
            if (frames_.quiescent()) return;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(false && "recorder retained a frame");
    }
    bool started() const noexcept { return status_.started(); }
    bool limit_reached() const noexcept { return status_.limit_reached(); }
    recorder::Error last_error() const noexcept { return status_.last_error(); }
    int last_system_error() const noexcept { return status_.details().system_error; }
    const char* last_error_path() const noexcept { details_ = status_.details(); return details_.error_path.data(); }
    const char* directory() const noexcept { details_ = status_.details(); return details_.directory.data(); }
    const char* recording_path() const noexcept { details_ = status_.details(); return details_.recording_path.data(); }
    std::uint64_t file_limit_bytes() const noexcept { return status_.details().file_limit_bytes; }
    recorder::RecorderStatus::Stats stats() const noexcept { return status_.stats(); }
private:
    bool launch(platform::WriteControl* writes, bool wait) noexcept {
        if (!frames_.quiescent()) { status_.set_error(recorder::Error::kAlreadyStarted); return false; }
        const executor::Component::Spec spec{"recorder", "recorder", 1000.0, {}};
        auto created = recorder::RecorderFactory::create(spec, config_, frames_, status_, writes);
        if (!created || !runner_.start(std::move(*created))) return false;
        if (!wait) return true;
        for (int i = 0; i < 2000; ++i) {
            if (started()) return true;
            if (last_error() != recorder::Error::kNone) { stop(); return false; }
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        stop();
        return false;
    }
    recorder::Config config_;
    recorder::FrameChannel frames_{};
    recorder::RecorderStatus status_{};
    executor::Tunner runner_{};
    mutable recorder::RecorderStatus::Details details_{};
};

} // namespace dart::tests
