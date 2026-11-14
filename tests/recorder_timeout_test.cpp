#include "tests/recorder_fixture.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

namespace dart::tests {

class RecorderTimeoutTest final {
    static std::expected<void, camera::CameraError> release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }
    static void sink(const recorder::FrameEvent& event, void* context) noexcept {
        static_cast<std::vector<recorder::FrameEvent>*>(context)->push_back(event);
    }
    static void expiration(recorder::ImageFormat format, unsigned fps, unsigned timeout_ms) {
        char path[] = "/tmp/dart-recorder-timeout-XXXXXX";
        assert(::mkdtemp(path));
        recorder::Config config;
        config.width = 8;
        config.height = 1;
        config.fps = fps;
        config.frame_timeout = std::chrono::milliseconds{timeout_ms};
        config.image_format = format;
        config.output_directory = path;
        const bool fixed = !config.variable_record_size();
        const bool pbm = format == recorder::ImageFormat::kPbm;
        const std::string header = pbm ? "P4\n8 1\n" : "P5\n8 1\n255\n";
        const auto bytes = header.size() + (pbm ? 1U : 8U);
        if (fixed) config.max_file_bytes = 2 * bytes;
        std::vector<recorder::FrameEvent> events;
        config.event_sink = sink;
        config.event_sink_context = &events;
        recorder::FrameChannel frames;
        recorder::RecorderStatus status;
        const executor::Component::Spec spec{"recorder", "recorder", 1000.0, {}};
        auto created = recorder::RecorderFactory::create(spec, config, frames, status);
        assert(created);
        auto component = std::move(*created);
        assert(component->initialze() && component->step());
        std::array<std::byte, 8> pixels{};
        camera::FrameLeaseAccess::Slot slot;
        slot.release = release;
        slot.view.pixels = pixels;
        slot.view.width = slot.view.stride = 8;
        slot.view.height = 1;
        for (unsigned i = 0; i < 3; ++i) {
            pixels.fill(i == 2 ? std::byte{255} : std::byte{0});
            slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
            slot.view.sequence = i + 1;
            slot.view.received += std::chrono::seconds{1};
            auto lease = camera::FrameLeaseAccess::make(&slot);
            assert(frames.try_submit(std::move(lease)) == recorder::Admission::kAccepted);
            assert(!lease.valid() && slot.state == camera::FrameLeaseAccess::SlotState::kHeld);
            if (i == 1) {
                const auto delay = timeout_ms == 0 ? (1'000'000U + fps - 1U) / fps : timeout_ms * 1000U;
                std::this_thread::sleep_for(std::chrono::microseconds{delay + 5000U});
            }
            assert(component->step());
            assert(frames.drain_releases() == 1);
            assert(slot.state == camera::FrameLeaseAccess::SlotState::kFree);
        }
        assert(component->shutdown());
        const auto stats = status.stats();
        assert(stats.accepted == 3 && stats.written == 2 && stats.timed_out == 1);
        assert(stats.copied == 0 && stats.write_fail == 0 && frames.quiescent());
        unsigned saved = 0, expired = 0;
        for (const auto& event : events) {
            if (event.decision == recorder::EventDecision::kTimedOut) {
                ++expired;
                assert(event.source_sequence == 2 && event.image_index == 0 && event.file_bytes == 0);
            } else if (event.decision == recorder::EventDecision::kAccepted) {
                assert(event.image_index == ++saved);
                assert(event.source_sequence == (saved == 1 ? 1U : 3U));
            }
        }
        assert(saved == 2 && expired == 1);
        if (fixed) {
            assert(status.limit_reached() && stats.bytes == config.max_file_bytes);
            std::ifstream image{status.details().recording_path.data(), std::ios::binary};
            const std::string actual{std::istreambuf_iterator<char>{image}, {}};
            const auto first = pbm ? std::string(1, '\xff') : std::string(8, '\0');
            const auto last = pbm ? std::string(1, '\0') : std::string(8, '\xff');
            assert(actual == header + first + header + last);
        }
        std::filesystem::remove_all(path);
    }
public:
    static void run() {
        for (const auto timeout : {-1, 60'001}) {
            recorder::Config invalid;
            invalid.width = invalid.height = 1;
            invalid.frame_timeout = std::chrono::milliseconds{timeout};
            RecorderFixture rec{invalid};
            assert(!rec.start() && rec.last_error() == recorder::Error::kInvalidConfig);
        }
        for (const auto format : {recorder::ImageFormat::kPgm, recorder::ImageFormat::kPbm,
                                  recorder::ImageFormat::kPng, recorder::ImageFormat::kDbv,
                                  recorder::ImageFormat::kPngStream}) {
            expiration(format, 60, 0);
            expiration(format, 30, 0);
            expiration(format, 10, 0);
            expiration(format, 60, 100);
        }
    }
};

} // namespace dart::tests

int main() { dart::tests::RecorderTimeoutTest::run(); }
