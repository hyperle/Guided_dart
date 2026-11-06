#include <cstdint>
#include <cstddef>
#include <expected>
#include <utility>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "logging/csv_writer.hpp"
#include "logging/logger.hpp"
#include "tests/recorder_fixture.hpp"
#include "runtime/imu/session.hpp"
#include "runtime/recording_trigger.hpp"

#include "system/write_control.hpp"

namespace dart::tests {

class WriteControlTest final {
    class Source final : public imu::IImu {
    public:
        std::expected<imu::Sample, imu::ErrorCode> read_sample() noexcept override {
            imu::Sample sample{};
            sample.acceleration_m_s2.z = 9.80665F;
            sample.timestamp_ns = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            return sample;
        }
    };
    template <typename Predicate> static void until(Predicate predicate) {
        for (int i = 0; i < 3000; ++i) {
            if (predicate())
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(false && "background writer did not reach the requested state");
    }
    static bool open_files_below(const std::filesystem::path& root) {
        for (const auto& fd : std::filesystem::directory_iterator{"/proc/self/fd"}) {
            std::error_code error;
            const auto path = std::filesystem::read_symlink(fd.path(), error);
            if (!error && path.string().starts_with(root.string() + '/'))
                return true;
        }
        return false;
    }
    static std::expected<void, camera::CameraError> release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }
    static void submit(RecorderFixture& rec, std::uint64_t sequence) {
        std::array<std::byte, 64> pixels{};
        pixels.fill(std::byte{255});
        camera::FrameLeaseAccess::Slot slot{};
        slot.view.luma = pixels;
        slot.view.width = slot.view.stride = 32;
        slot.view.height = 2;
        slot.view.sequence = sequence;
        slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
        slot.release = release;
        auto lease = camera::FrameLeaseAccess::make(&slot);
        until([&] {
            slot.view.received += std::chrono::milliseconds{50};
            const auto result = rec.try_submit(std::move(lease));
            return result == recorder::Admission::kAccepted;
        });
        until([&] {
            (void)rec.drain_releases();
            return rec.pending_releases() == 0 && rec.stats().written == 1;
        });
        assert(slot.state == camera::FrameLeaseAccess::SlotState::kFree);
    }
    static void trigger(platform::WriteControl& writes) {
        writes.request_recording(false);
        app::RecordingTrigger trigger{writes, 100, 3, 10};
        imu::Sample sample{};
        sample.acceleration_m_s2.x = 150;
        sample.timestamp_ns = 1'000'000;
        trigger.update(sample);
        sample.timestamp_ns = 20'000'000; // A missing interval cannot satisfy the hold.
        trigger.update(sample);
        assert(!writes.recording_enabled());
        for (unsigned i = 1; i <= 3; ++i) {
            sample.timestamp_ns = 20'000'000 + i * 1'000'000;
            trigger.update(sample);
        }
        assert(writes.recording_enabled() && writes.recording_request_ns() == 23'000'000);
        sample.timestamp_ns = 33'000'000;
        trigger.update(sample);
        assert(!writes.recording_enabled());
    }
    static void cycle(const std::filesystem::path& root, recorder::ImageFormat format) {
        const auto pause = root / "write.pause";
        std::ofstream{pause} << "pause";
        platform::WriteControl writes;
        assert(writes.start(pause.c_str()) && writes.paused());
        const auto records = root / "records";
        const auto logs = root / "logs";
        const auto record_path = records.string();
        recorder::Config config{};
        config.width = 32;
        config.height = 2;
        config.output_directory = record_path;
        config.image_format = format;
        RecorderFixture rec{config};
        assert(rec.start_controlled(writes));
        logging::Logger log{&writes};
        assert(log.open((logs / "app.log").string()));
        logging::CsvWriter csv;
        assert(csv.open((logs / "detect.csv").c_str(), "frame\n", &writes));
        Source source;
        imu::Session imu{source};
        assert(imu.start((logs / "imu.log").c_str(), 1000, true,
                         imu::Session::kDefaultLogLimitBytes, &writes));
        until([&] { return imu.stats().samples >= 10; });
        log.line("paused\n");
        csv.row("paused\n");
        assert(!std::filesystem::exists(records) && !std::filesystem::exists(logs));

        std::filesystem::remove(pause);
        until([&] { return !writes.paused() && rec.started() && imu.stats().log_bytes > 0; });
        submit(rec, 1);
        log.line("first\n");
        csv.row("first\n");
        const auto first_image = records / "rec_0001";
        until([&] { return std::filesystem::exists(logs / "detect.csv"); });
        const auto before = imu.stats().samples;
        std::ofstream{pause} << "pause";
        until([&] { return writes.paused() && !rec.started() && !open_files_below(root); });
        const auto original_size = std::filesystem::file_size(logs / "imu_000001.log");
        until([&] { return imu.stats().samples >= before + 10; });
        assert(imu.current().has_reference_frame);
        writes.request_recording(true);
        log.line("paused again\n");
        csv.row("paused again\n");
        assert(!open_files_below(root));
        assert(std::filesystem::file_size(logs / "imu_000001.log") == original_size);

        std::filesystem::remove(pause);
        until([&] { return !writes.paused() && rec.started()
            && std::filesystem::exists(logs / "imu_000002.log"); });
        submit(rec, 2);
        assert(std::filesystem::exists(first_image));
        assert(std::filesystem::exists(records / "rec_0002"));
        writes.request_recording(false);
        until([&] { return !rec.started(); });
        writes.request_logging(false);
        until([&] { return !open_files_below(root); });
        std::filesystem::remove_all(logs);
        std::filesystem::remove_all(records);
        writes.request_recording(true);
        until([&] { return rec.started(); });
        submit(rec, 3);
        assert(!std::filesystem::exists(logs));
        writes.request_logging(true);
        log.line("resumed\n");
        csv.row("resumed\n");
        until([&] { return std::filesystem::exists(logs / "imu_000001.log"); });
        rec.stop();
        imu.stop();
        csv.close();
        assert(log.flush(std::chrono::milliseconds{5000}));
        trigger(writes);
        assert(!open_files_below(root));
        std::filesystem::remove_all(root);
    }
public:
    static void run() {
        for (const auto format : {recorder::ImageFormat::kDbv, recorder::ImageFormat::kPng, recorder::ImageFormat::kPgm,
                                  recorder::ImageFormat::kPbm, recorder::ImageFormat::kPngStream}) {
            char path[] = "/tmp/dart-write-control-XXXXXX";
            assert(::mkdtemp(path));
            cycle(path, format);
        }
    }
};

} // namespace dart::tests

int main() { dart::tests::WriteControlTest::run(); }
