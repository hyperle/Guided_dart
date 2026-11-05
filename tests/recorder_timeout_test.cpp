#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <latch>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "recorder/recorder.hpp"

namespace dart::tests {

static std::atomic<bool> hold_sync{false}, sync_entered{false};
extern "C" int __real_fsync(int);
extern "C" int __wrap_fsync(int fd) {
    if (hold_sync.load()) {
        sync_entered.store(true);
        while (hold_sync.load())
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    return __real_fsync(fd);
}

class RecorderTimeoutTest final {
    class Metadata final {
    public:
        std::vector<recorder::FrameEvent> events;
        static void sink(const recorder::FrameEvent& event, void* context) noexcept {
            auto& self = *static_cast<Metadata*>(context);
            if (!self.entered_.exchange(true))
                self.resume_.wait();
            self.events.push_back(event);
        }
        void hold(recorder::Recorder& rec) {
            for (unsigned wait = 0; wait < 2000 && !entered_.load(); ++wait) {
                camera::FrameLease empty;
                assert(rec.try_submit(std::move(empty)) == recorder::Admission::kInvalidFrame);
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
            assert(entered_.load());
        }
        void resume() { resume_.count_down(); }

    private:
        std::atomic<bool> entered_{false};
        std::latch resume_{1};
    };

    static std::expected<void, camera::CameraError>
        release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }

    static void submit(recorder::Recorder& rec, camera::FrameLeaseAccess::Slot& slot) {
        slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
        auto lease = camera::FrameLeaseAccess::make(&slot);
        bool accepted = false;
        for (unsigned attempt = 0; attempt < 2000; ++attempt) {
            slot.view.received += std::chrono::seconds{1};
            const auto result = rec.try_submit(std::move(lease));
            if (result == recorder::Admission::kAccepted) {
                accepted = true;
                break;
            }
            assert(result == recorder::Admission::kBusy && lease.valid());
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(accepted && !lease.valid());
        for (unsigned wait = 0; wait < 2000 && rec.pending_releases(); ++wait) {
            (void)rec.drain_releases();
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(rec.pending_releases() == 0);
        assert(slot.state == camera::FrameLeaseAccess::SlotState::kFree);
    }

    static void drain_images(recorder::Recorder& rec) {
        for (unsigned wait = 0; wait < 2000 && rec.stats().queued; ++wait)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(rec.stats().queued == 0);
    }

    static void slow_disk(recorder::ImageFormat format, unsigned fps, unsigned timeout_ms) {
        char path[] = "/tmp/dart-recorder-timeout-XXXXXX";
        assert(::mkdtemp(path));
        Metadata metadata;
        recorder::Config config;
        config.width = 8;
        config.height = 1;
        config.fps = fps;
        config.frame_timeout = std::chrono::milliseconds{timeout_ms};
        config.queue_slots = 4;
        config.image_format = format;
        config.output_directory = path;
        config.flush_period = std::chrono::milliseconds{60'000};
        config.event_sink = Metadata::sink;
        config.event_sink_context = &metadata;
        const bool fixed = !config.variable_record_size();
        const bool pbm = format == recorder::ImageFormat::kPbm;
        const std::string header = pbm ? "P4\n8 1\n" : "P5\n8 1\n255\n";
        const auto frame_bytes = header.size() + (pbm ? 1U : 8U);
        if (fixed)
            config.max_file_bytes = 2 * frame_bytes;
        recorder::Recorder rec{config};
        assert(rec.start());
        metadata.hold(rec);
        std::array<std::byte, 8> pixels{};
        camera::FrameLeaseAccess::Slot slot{};
        slot.view.luma = pixels;
        slot.view.width = slot.view.stride = 8;
        slot.view.height = 1;
        slot.release = release;
        hold_sync.store(true);
        sync_entered.store(false);
        for (unsigned frame = 1; frame <= 3; ++frame) {
            pixels.fill(frame == 1 ? std::byte{0} : std::byte{255});
            slot.view.sequence = frame;
            submit(rec, slot);
            if (frame == 1) {
                for (unsigned wait = 0; wait < 2000 && !sync_entered.load(); ++wait)
                    std::this_thread::sleep_for(std::chrono::milliseconds{1});
                assert(sync_entered.load());
            }
        }
        // A blocked disk cannot retain camera memory or stop further admission.
        assert(rec.stats().queued == 3 && rec.stats().written == 0);
        const auto timeout_us = timeout_ms ? timeout_ms * 1000U : (1'000'000U + fps - 1U) / fps;
        std::this_thread::sleep_for(std::chrono::microseconds{timeout_us + 20'000U});
        hold_sync.store(false);
        drain_images(rec);
        assert(rec.stats().written == 1 && rec.stats().timed_out == 2);
        assert(rec.started() && !rec.limit_reached());
        // A fresh image must follow the saved image without an index/offset gap.
        slot.view.sequence = 4;
        pixels.fill(std::byte{255});
        submit(rec, slot);
        drain_images(rec);
        metadata.resume();
        rec.stop();
        const auto stats = rec.stats();
        assert(stats.accepted == 4 && stats.copied == 4 && stats.written == 2);
        assert(stats.timed_out == 2 && stats.write_fail == 0 && stats.event_dropped == 0);
        assert(rec.last_error() == recorder::Error::kNone && rec.pending_releases() == 0);
        assert(rec.limit_reached() == fixed);
        unsigned saved = 0, expired = 0;
        std::uint64_t offset = 0;
        for (const auto& event : metadata.events) {
            if (event.decision == recorder::EventDecision::kTimedOut) {
                ++expired;
                assert(event.source_sequence == 2 || event.source_sequence == 3);
                assert(event.image_index == 0 && event.image_name[0] == 0);
                assert(event.file_offset == 0 && event.file_bytes == 0);
            } else if (event.decision == recorder::EventDecision::kAccepted) {
                ++saved;
                assert(event.image_index == saved);
                assert(event.source_sequence == (saved == 1 ? 1U : 4U));
                assert(event.file_offset == (format == recorder::ImageFormat::kPng ? 0 : offset));
                assert(event.file_bytes > 0 && event.image_name[0] != 0);
                offset += event.file_bytes;
            }
        }
        assert(saved == 2 && expired == 2 && offset == stats.bytes);
        std::ifstream csv{std::filesystem::path{rec.directory()} / "logs/frames.csv"};
        const std::string events{std::istreambuf_iterator<char>{csv}, {}};
        assert(events.find(",timed_out,0,,0,0") != std::string::npos);
        if (fixed) {
            std::ifstream image{rec.recording_path(), std::ios::binary};
            const std::string bytes{std::istreambuf_iterator<char>{image}, {}};
            const auto first = pbm ? std::string(1, '\xff') : std::string(8, '\0');
            const auto last = pbm ? std::string(1, '\0') : std::string(8, '\xff');
            assert(bytes == header + first + header + last);
            assert(stats.bytes == config.max_file_bytes);
        }
        assert(rec.start());
        assert(rec.stats().timed_out == 0);
        rec.stop();
        std::filesystem::remove_all(path);
    }

public:
    static void run() {
        for (const auto timeout : {-1, 60'001}) {
            recorder::Config invalid;
            invalid.width = invalid.height = 1;
            invalid.frame_timeout = std::chrono::milliseconds{timeout};
            recorder::Recorder rec{invalid};
            assert(!rec.start() && rec.last_error() == recorder::Error::kInvalidConfig);
        }
        for (const auto format : {recorder::ImageFormat::kPgm, recorder::ImageFormat::kPbm,
                                  recorder::ImageFormat::kPng, recorder::ImageFormat::kDbv}) {
            slow_disk(format, 60, 0);
            slow_disk(format, 10, 0);
            slow_disk(format, 60, 100);
        }
    }
};

} // namespace dart::tests

int main() { dart::tests::RecorderTimeoutTest::run(); }
