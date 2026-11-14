#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <expected>
#include <filesystem>
#include <memory>
#include <span>
#include <thread>
#include <utility>

#include "camera/frame_lease.hpp"
#include "tests/recorder_fixture.hpp"
#include "streaming/session.hpp"

namespace dart::tests {

class StreamingControlTest final {
    inline static std::atomic_uint factories_{0}, opens_{0}, uploads_{0}, live_{0};
    inline static std::atomic_bool stall_{false}, hold_cleanup_{false}, cancelled_{false},
        fail_{false};

    class Transport final : public streaming::ITransport {
    public:
        Transport() { ++live_; }
        ~Transport() override { --live_; }
        std::expected<void, streaming::Error>
            open(const streaming::Config&, const std::atomic_bool& stop) noexcept override {
            ++opens_;
            while (stall_.load() && !stop.load())
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            if (stop.load()) {
                cancelled_.store(true);
                // A synchronous firmware ioctl may not return immediately on
                // cancellation. Its cleanup must never stall the producer.
                while (hold_cleanup_.load())
                    std::this_thread::sleep_for(std::chrono::milliseconds{1});
                return std::unexpected(streaming::Error{streaming::Code::kConnect, 125});
            }
            return {};
        }
        std::expected<void, streaming::Error> upload(
            std::span<const std::uint8_t> jpeg, std::uint64_t, std::uint64_t,
            const std::atomic_bool&) noexcept override {
            assert(jpeg.size() > 100 && jpeg[0] == 0xff && jpeg[1] == 0xd8);
            ++uploads_;
            return {};
        }
        void close() noexcept override {}
    };

    static std::unique_ptr<streaming::ITransport> create() noexcept {
        ++factories_;
        if (fail_.exchange(false))
            return nullptr;
        return std::make_unique<Transport>();
    }
    static std::expected<void, camera::CameraError>
        release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }

public:
    static void run() {
        char directory[] = "/tmp/dart-stream-control-XXXXXX";
        assert(mkdtemp(directory));
        const std::filesystem::path root{directory};
        const auto pause = root / "stream.pause";
        const auto config_path = root / "wifi.conf";
        FILE* config_file = std::fopen(config_path.c_str(), "w");
        assert(config_file);
        assert(
            std::fprintf(
                config_file, "host=127.0.0.1\ntoken=test-token\nfps=120\nwidth=16\nheight=16\n")
            > 0);
        assert(std::fclose(config_file) == 0);

        const auto recording_path = (root / "records").string();
        recorder::Config config;
        config.width = config.height = 32;
        config.fps = 100;

        config.output_directory = recording_path;
        RecorderFixture rec{config};
        assert(rec.start());
        std::array<std::byte, 40 * 32> pixels;
        pixels.fill(std::byte{127});
        camera::FrameLeaseAccess::Slot slot;
        slot.state = camera::FrameLeaseAccess::SlotState::kFree;
        slot.release = release;
        slot.view.pixels = pixels;
        slot.view.width = slot.view.height = 32;
        slot.view.stride = 40;
        streaming::Session stream;
        unsigned producer_steps = 0;
        auto pump = [&] {
            ++producer_steps;
            (void)rec.drain_releases();
            if (slot.state == camera::FrameLeaseAccess::SlotState::kFree) {
                slot.view.received = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch());
                ++slot.view.sequence;
                slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
                auto lease = camera::FrameLeaseAccess::make(&slot);
                (void)stream.try_submit(slot.view);
                (void)rec.try_submit(std::move(lease));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        };
        auto until = [&](auto predicate) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
            while (!predicate()) {
                assert(std::chrono::steady_clock::now() < deadline);
                pump();
            }
        };
        auto paused = [&] {
            return stream.state() == streaming::State::kPaused && stream.stats().buffer_bytes == 0
                && !stream.accepting();
        };

        // Startup pause does not instantiate a transport or encoder buffers.
        assert(std::filesystem::create_directory(pause));
        assert(stream.start_controlled(config_path.c_str(), 32, 32, create, pause.c_str()));
        until(paused);
        until([&] { return rec.stats().written >= 3; });
        assert(factories_ == 0 && opens_ == 0 && stream.stats().encoded == 0);
        assert(std::filesystem::remove(pause));
        until([&] { return uploads_ >= 3; });
        assert(stream.state() == streaming::State::kRunning && stream.accepting());

        // Off stops both workers, releases owned buffers and leaves recording
        // alive. It also permits subsequent on to create a fresh transport.
        assert(std::filesystem::create_directory(pause));
        until(paused);
        assert(live_ == 0);
        const auto encoded_before = stream.stats().encoded;
        auto written_before = rec.stats().written;
        until([&] { return rec.stats().written >= written_before + 3; });
        assert(stream.stats().encoded == encoded_before);

        stall_ = hold_cleanup_ = true;
        cancelled_ = false;
        const auto opens_before = opens_.load();
        assert(std::filesystem::remove(pause));
        until([&] { return opens_ > opens_before; });
        assert(std::filesystem::create_directory(pause));
        until([&] { return cancelled_.load(); });
        assert(stream.state() == streaming::State::kStopping && !stream.accepting());
        assert(stream.stats().buffer_bytes != 0 && live_ == 1);
        const auto admitted_before = stream.stats().accepted;
        const auto steps_before = producer_steps;
        written_before = rec.stats().written;
        until([&] { return rec.stats().written >= written_before + 5; });
        assert(producer_steps > steps_before + 5);
        assert(stream.stats().accepted == admitted_before);
        // Only after the old worker exits may memory be reclaimed.
        hold_cleanup_ = false;
        until(paused);
        assert(live_ == 0);
        stall_ = false;
        assert(std::filesystem::remove(pause));
        const auto uploaded_before = uploads_.load();
        until([&] { return uploads_ > uploaded_before; });

        // Failed worker startup is isolated and latched, rather than allocating
        // and restarting continuously alongside the existing recorder.
        assert(std::filesystem::create_directory(pause));
        until(paused);
        fail_ = true;
        assert(std::filesystem::remove(pause));
        until([&] { return stream.state() == streaming::State::kFailed; });
        assert(stream.stats().buffer_bytes == 0 && !stream.accepting() && live_ == 0);
        const auto factory_calls = factories_.load();
        written_before = rec.stats().written;
        until([&] { return rec.stats().written >= written_before + 5; });
        assert(factories_ == factory_calls);
        assert(std::filesystem::create_directory(pause));
        until(paused);
        assert(std::filesystem::remove(pause));
        until([&] { return stream.state() == streaming::State::kRunning; });
        assert(stream.error().code == streaming::Code::kNone);
        stream.stop();
        // Configuration file I/O is also on the supervisor. Missing files fail
        // only this component, and fixing the file plus off/on recovers it.
        assert(stream.start_controlled(
            (root / "missing.conf").c_str(), 32, 32, create, pause.c_str()));
        until([&] { return stream.state() == streaming::State::kFailed; });
        assert(stream.error().code == streaming::Code::kConfig && live_ == 0);
        written_before = rec.stats().written;
        until([&] { return rec.stats().written >= written_before + 3; });
        assert(std::filesystem::create_directory(pause));
        until(paused);
        std::filesystem::copy_file(config_path, root / "missing.conf");
        assert(std::filesystem::remove(pause));
        until([&] { return stream.state() == streaming::State::kRunning; });
        stream.stop();
        assert(stream.state() == streaming::State::kStopped && live_ == 0);
        assert(stream.stats().buffer_bytes == 0);
        written_before = rec.stats().written;
        until([&] { return rec.stats().written >= written_before + 3; });
        assert(rec.last_error() == recorder::Error::kNone);
        rec.stop();
        until([&] { return rec.pending_releases() == 0; });
        assert(slot.state == camera::FrameLeaseAccess::SlotState::kFree);
        std::filesystem::remove_all(root);
    }
};

} // namespace dart::tests

int main() { dart::tests::StreamingControlTest::run(); }
