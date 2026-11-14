#include <cstdint>
#include <cstddef>
#include <expected>
#include <chrono>
#include <utility>
# include <algorithm>
# include <array>
# include <atomic>
# include <cassert>
# include <cerrno>
# include <cstdio>
# include <cstdlib>
# include <filesystem>
# include <fstream>
# include <latch>
# include <string>
# include <thread>
# include <unistd.h>

#include "camera/frame_lease.hpp"
#include "tests/recorder_fixture.hpp"

namespace dart::tests {

class RecorderSessionTest final {
public:
    static void run() {
        char path[] = "/tmp/dart-recorder-session-XXXXXX";
        assert(::mkdtemp(path) != nullptr);
        const std::filesystem::path root(path);
        dart::recorder::Config config{};
        config.width = 32;
        config.height = 2;
        config.output_directory = path;
        config.keep_sessions = 2;

        for (int i = 1; i <= 3; ++i) {
            dart::tests::RecorderFixture recorder(config);
            assert(recorder.start());
            recorder.stop();
            const auto session = root / ("rec_000" + std::to_string(i));
            assert(std::filesystem::is_directory(session / "img"));
            assert(std::filesystem::exists(session / "logs/frames.csv"));
        }
        assert(!std::filesystem::exists(root / "rec_0001"));
        assert(std::filesystem::exists(root / "rec_0002"));
        assert(std::filesystem::exists(root / "rec_0003"));

        std::ofstream(root / "rec_0002/img/user.txt") << "keep";
        dart::tests::RecorderFixture guarded(config);
        assert(!guarded.start());
        assert(guarded.last_system_error() == EPERM);
        assert(std::filesystem::exists(root / "rec_0002/img/user.txt"));
        assert(std::filesystem::exists(root / "rec_0003/logs/frames.csv"));
        assert(!std::filesystem::exists(root / "rec_0004"));

        std::filesystem::remove_all(root);
        images(dart::recorder::ImageFormat::kPgm);
        images(dart::recorder::ImageFormat::kPbm);
        for (const auto format :
             {dart::recorder::ImageFormat::kPgm, dart::recorder::ImageFormat::kPbm}) {
            size_limit(format, false);
            size_limit(format, true);
        }
        sampling();
        std::puts("recorder sessions, streams, size limit and 60 FPS sampling passed");
    }

private:
    static std::expected<void, dart::camera::CameraError>
        release(dart::camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = dart::camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }

    static void images(dart::recorder::ImageFormat format) {
        char path[] = "/tmp/dart-recorder-images-XXXXXX";
        assert(::mkdtemp(path) != nullptr);
        std::array<std::byte, 64> pixels{};
        pixels.fill(std::byte{255});
        dart::camera::FrameLeaseAccess::Slot slot{};
        slot.view.pixels = pixels;
        slot.view.width = slot.view.stride = 32;
        slot.view.height = 2;
        slot.view.sequence = 17;
        slot.view.received = std::chrono::microseconds{1'000'000};
        slot.state = dart::camera::FrameLeaseAccess::SlotState::kHeld;
        slot.release = &RecorderSessionTest::release;
        dart::recorder::Config config{};
        config.width = 32;
        config.height = 2;
        config.image_format = format;
        config.output_directory = path;
        dart::tests::RecorderFixture recorder{config};
        assert(recorder.start());
        auto lease = dart::camera::FrameLeaseAccess::make(&slot);
        bool accepted = false;
        for (int attempt = 0; attempt < 100; ++attempt) {
            const auto admission = recorder.try_submit(std::move(lease));
            if (admission == dart::recorder::Admission::kAccepted) {
                accepted = true;
                break;
            }
            // Admission may race the worker's queue lock. Keep ownership and
            // retry in a later sampling interval, as a capture loop would.
            assert(admission == dart::recorder::Admission::kBusy && lease.valid());
            slot.view.received += std::chrono::milliseconds{50};
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(accepted);
        assert(!lease.valid());
        recorder.stop();
        assert(recorder.pending_releases() == 1);
        assert(!recorder.start()); // Restart cannot discard an outstanding camera lease.
        assert(recorder.last_error() == dart::recorder::Error::kAlreadyStarted);
        assert(recorder.drain_releases() == 1);
        assert(slot.state == dart::camera::FrameLeaseAccess::SlotState::kFree);
        assert(recorder.stats().written == 1);
        const bool pbm = format == dart::recorder::ImageFormat::kPbm;
        const auto root = std::filesystem::path{path} / "rec_0001";
        const auto image = root / (pbm ? "img/recording.pbm" : "img/recording.pgm");
        std::ifstream input(image, std::ios::binary);
        const std::string actual{std::istreambuf_iterator<char>{input}, {}};
        const std::string header = pbm ? "P4\n32 2\n" : "P5\n32 2\n255\n";
        assert(actual.substr(0, header.size()) == header);
        assert(actual.size() == header.size() + (pbm ? 8 : 64));
        for (std::size_t i = header.size(); i < actual.size(); ++i)
            assert(static_cast<unsigned char>(actual[i]) == (pbm ? 0 : 255));
        std::ifstream csv(root / "logs/frames.csv");
        const std::string events{std::istreambuf_iterator<char>{csv}, {}};
        assert(
            std::count(events.begin(), events.end(), '\n') - 2 + recorder.stats().event_dropped
            == recorder.stats().input);
        assert(events.find(",17,") != std::string::npos);
        assert(events.find(",accepted,") != std::string::npos);
        assert(recorder.start());
        slot.state = dart::camera::FrameLeaseAccess::SlotState::kHeld;
        slot.view.sequence = 18;
        slot.view.received = std::chrono::microseconds{1};
        submit(recorder, slot);
        recorder.stop();
        assert(recorder.drain_releases() == 1);
        assert(recorder.stats().written == 1);
        const auto second = std::filesystem::path{path} / "rec_0002"
            / (pbm ? "img/recording.pbm" : "img/recording.pgm");
        assert(std::filesystem::file_size(second) == actual.size());
        assert(std::filesystem::file_size(image) == actual.size());
        std::filesystem::remove_all(path);
    }

    static void
        submit(dart::tests::RecorderFixture& recorder, dart::camera::FrameLeaseAccess::Slot& slot) {
        auto lease = dart::camera::FrameLeaseAccess::make(&slot);
        for (int attempt = 0; attempt < 100; ++attempt) {
            const auto admission = recorder.try_submit(std::move(lease));
            if (admission == dart::recorder::Admission::kAccepted) {
                assert(!lease.valid());
                return;
            }
            assert(admission == dart::recorder::Admission::kBusy && lease.valid());
            slot.view.received += std::chrono::milliseconds{50};
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(false && "recorder never admitted a frame");
    }

    static void size_limit(dart::recorder::ImageFormat format, bool slack) {
        char path[] = "/tmp/dart-recorder-limit-XXXXXX";
        assert(::mkdtemp(path) != nullptr);
        const bool pbm = format == dart::recorder::ImageFormat::kPbm;
        const std::string header = pbm ? "P4\n9 2\n" : "P5\n9 2\n255\n";
        const std::size_t frame_bytes = header.size() + (pbm ? 4 : 18);
        dart::recorder::Config config{};
        config.width = 9;
        config.height = 2;
        config.image_format = format;
        config.output_directory = path;
        config.max_file_bytes = 2 * frame_bytes + (slack ? frame_bytes - 1 : 0);
        std::array<std::byte, 18> pixels{};
        pixels.fill(std::byte{255});
        pixels[0] = pixels[8] = std::byte{0};
        std::array<dart::camera::FrameLeaseAccess::Slot, 3> slots{};
        dart::tests::RecorderFixture recorder{config};
        assert(recorder.start());
        for (std::size_t i = 0; i < slots.size(); ++i) {
            auto& slot = slots[i];
            slot.view.pixels = pixels;
            slot.view.width = slot.view.stride = 9;
            slot.view.height = 2;
            slot.view.sequence = i + 1;
            slot.view.received =
                std::chrono::microseconds{static_cast<std::int64_t>(i + 1) * 100000};
            slot.state = dart::camera::FrameLeaseAccess::SlotState::kHeld;
            slot.release = &RecorderSessionTest::release;
            if (i < 2) {
                submit(recorder, slot);
                recorder.wait_released();
            } else {
                for (unsigned wait = 0; wait < 1000 && recorder.started(); ++wait)
                    std::this_thread::sleep_for(std::chrono::milliseconds{1});
                assert(recorder.limit_reached() && !recorder.started());
                auto lease = dart::camera::FrameLeaseAccess::make(&slot);
                assert(
                    recorder.try_submit(std::move(lease)) == dart::recorder::Admission::kStopped);
                assert(lease.valid() && lease.release());
            }
        }
        recorder.stop();
        assert(recorder.drain_releases() == 0);
        assert(recorder.pending_releases() == 0);
        for (const auto& slot : slots)
            assert(slot.state == dart::camera::FrameLeaseAccess::SlotState::kFree);
        assert(recorder.stats().written == 2 && recorder.stats().write_fail == 0);
        assert(recorder.last_error() == dart::recorder::Error::kNone);
        assert(recorder.stats().bytes == 2 * frame_bytes);
        const auto root = std::filesystem::path{path} / "rec_0001";
        const auto image = root / (pbm ? "img/recording.pbm" : "img/recording.pgm");
        assert(std::filesystem::file_size(image) == 2 * frame_bytes);
        assert(std::filesystem::file_size(image) <= config.max_file_bytes);
        assert(
            std::distance(
                std::filesystem::directory_iterator(root / "img"),
                std::filesystem::directory_iterator{})
            == 1);
        std::ifstream input(image, std::ios::binary);
        const std::string actual{std::istreambuf_iterator<char>{input}, {}};
        const std::string payload =
            pbm ? std::string{"\x80\x80\0\0", 4}
                : std::string{reinterpret_cast<const char*>(pixels.data()), pixels.size()};
        const std::string frame = header + payload;
        assert(actual == frame + frame);
        std::ifstream csv(root / "logs/frames.csv");
        const std::string events{std::istreambuf_iterator<char>{csv}, {}};
        assert(
            std::count(events.begin(), events.end(), '\n') - 2 + recorder.stats().event_dropped
            == recorder.stats().input);
        assert(events.find("file_offset,file_bytes") != std::string::npos);
        assert(
            events.find("," + std::to_string(frame_bytes) + "," + std::to_string(frame_bytes))
            != std::string::npos);

        // Simulate another boot. The first stream must survive unchanged.
        dart::tests::RecorderFixture next{config};
        assert(next.start());
        next.stop();
        assert(std::filesystem::exists(std::filesystem::path{path} / "rec_0002/img"));
        assert(std::filesystem::file_size(image) == 2 * frame_bytes);
        for (const auto cap :
             {std::uint64_t{0}, frame_bytes - 1, dart::recorder::Config::kMaxFileBytes + 1}) {
            config.max_file_bytes = cap;
            dart::tests::RecorderFixture invalid{config};
            assert(!invalid.start());
            assert(invalid.last_error() == dart::recorder::Error::kInvalidConfig);
        }
        std::filesystem::remove_all(path);
    }

    static void sampling() {
        char path[] = "/tmp/dart-recorder-fps-XXXXXX";
        assert(::mkdtemp(path) != nullptr);
        dart::recorder::Config config{};
        assert(config.fps == 60 && config.max_file_bytes == 500'000'000);
        config.width = 1;
        config.height = 1;
        config.output_directory = path;
        dart::tests::RecorderFixture recorder{config};
        assert(recorder.start());
        std::array<std::byte, 1> pixels{};
        dart::camera::FrameLeaseAccess::Slot slot{};
        slot.view.pixels = pixels;
        slot.view.width = slot.view.stride = slot.view.height = 1;
        slot.release = &RecorderSessionTest::release;
        for (std::uint64_t i = 0; i < 120; ++i) {
            slot.state = dart::camera::FrameLeaseAccess::SlotState::kHeld;
            slot.view.sequence = i;
            // Offset by 1 ms so integer timestamps stay inside their intended bin.
            slot.view.received = std::chrono::microseconds{1'001'000 + i * 1'000'000 / 120};
            auto lease = dart::camera::FrameLeaseAccess::make(&slot);
            const auto admission = recorder.try_submit(std::move(lease));
            if (admission == dart::recorder::Admission::kAccepted) {
                recorder.wait_released();
                assert(slot.state == dart::camera::FrameLeaseAccess::SlotState::kFree);
            } else {
                assert(lease.valid() && lease.release());
            }
        }
        recorder.stop();
        const auto stats = recorder.stats();
        assert(stats.input == 120 && stats.downsampled == 60);
        assert(stats.accepted + stats.busy == 60 && stats.accepted > 0);
        assert(stats.written == stats.accepted && stats.write_fail == 0);
        std::filesystem::remove_all(path);
    }
};

} // namespace dart::tests

int main() { dart::tests::RecorderSessionTest::run(); }
