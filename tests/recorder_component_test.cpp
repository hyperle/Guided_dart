#include "runtime/tunner.hpp"
#include "src/control/blackboard.hpp"
#include "src/detector/detect.hpp"
#include "src/recorder/frame_channel.hpp"
#include "src/recorder/storage.hpp"
#include "src/recorder/encode.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>

namespace dart::tests {

static std::atomic<bool> stall_sync{false}, sync_entered{false}, fail_sync{false};
extern "C" int __real_fsync(int);
extern "C" int __wrap_fsync(int fd) {
    if (stall_sync.load()) {
        sync_entered = true;
        while (stall_sync.load()) std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    if (fail_sync.load()) { errno = EIO; return -1; }
    return __real_fsync(fd);
}

class RecorderComponentTest final {
    class Camera final : public camera::ICamera {
    public:
        Camera() {
            for (std::size_t i = 0; i < slots.size(); ++i) {
                slots[i].owner = this;
                slots[i].release = release;
                slots[i].view.luma = pixels[i];
                slots[i].view.width = slots[i].view.stride = info.width;
                slots[i].view.height = info.height;
            }
        }
        std::expected<void, camera::CameraError> start() noexcept override {
            owner_thread = std::this_thread::get_id();
            state_ = camera::State::kStarted;
            return {};
        }
        std::expected<camera::AcquireResult, camera::CameraError> acquire(std::chrono::milliseconds) noexcept override {
            assert(std::this_thread::get_id() == owner_thread && state_ == camera::State::kStarted);
            ++acquires;
            for (std::size_t i = 0; i < slots.size(); ++i) {
                auto& slot = slots[i];
                if (slot.state != camera::FrameLeaseAccess::SlotState::kFree) continue;
                pixels[i].fill(std::byte{20});
                slot.view.sequence = ++sequence;
                slot.view.received = std::chrono::microseconds{sequence * 20'000U};
                slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
                return camera::AcquireResult{camera::AcquireStatus::kFrame, camera::FrameLeaseAccess::make(&slot)};
            }
            return camera::AcquireResult{camera::AcquireStatus::kLeaseExhausted, {}};
        }
        std::expected<void, camera::CameraError> stop() noexcept override {
            assert(outstanding() == 0);
            state_ = camera::State::kInitialized;
            return {};
        }
        camera::State state() const noexcept override { return state_; }
        const camera::FrameInfo& frame_info() const noexcept override { return info; }
        std::expected<void, camera::CameraError> set_fixed_exposure_lines(std::uint32_t) noexcept override { return {}; }
        std::uint32_t capacity() const noexcept override { return slots.size(); }
        std::uint32_t outstanding() const noexcept override {
            unsigned count = 0;
            for (const auto& slot : slots) count += slot.state == camera::FrameLeaseAccess::SlotState::kHeld;
            return count;
        }
        static std::expected<void, camera::CameraError> release(camera::FrameLeaseAccess::Slot* slot) noexcept {
            auto& self = *static_cast<Camera*>(slot->owner);
            assert(std::this_thread::get_id() == self.owner_thread);
            if (self.fail_release) {
                self.fail_release = false;
                return std::unexpected(camera::CameraError{DART_CAMERA_E_INVALID_STATE, {}});
            }
            ++self.releases;
            slot->state = camera::FrameLeaseAccess::SlotState::kFree;
            return {};
        }
        std::array<camera::FrameLeaseAccess::Slot, 4> slots{};
        std::array<std::array<std::byte, 32 * 24>, 4> pixels{};
        camera::FrameInfo info{32, 24};
        std::thread::id owner_thread{};
        std::uint64_t sequence = 0;
        unsigned acquires = 0, releases = 0;
        bool fail_release = false;
    private:
        camera::State state_ = camera::State::kInitialized;
    };
    template<typename Predicate> static void until(Predicate predicate) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
        while (!predicate()) {
            assert(std::chrono::steady_clock::now() < deadline);
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
    }
    static executor::Component::Spec spec() { return {"recorder", "recorder", 1000.0, {}}; }
    static void channel_ownership() {
        Camera camera;
        assert(camera.start());
        recorder::Config config;
        config.width = camera.info.width;
        config.height = camera.info.height;
        recorder::FrameChannel frames;
        recorder::RecorderStatus status;
        assert(frames.prepare(config, status));
        frames.open();
        auto first = camera.acquire({});
        const auto* address = first->lease.view().luma.data();
        assert(frames.try_submit(std::move(first->lease)) == recorder::Admission::kAccepted);
        assert(!first->lease.valid() && camera.outstanding() == 1);
        auto owned = frames.take();
        assert(owned.lease.view().luma.data() == address);
        auto second = camera.acquire({});
        assert(frames.try_submit(std::move(second->lease)) == recorder::Admission::kBusy);
        assert(second->lease.valid() && second->lease.release());
        frames.return_frame(std::move(owned.lease));
        frames.complete();
        camera.fail_release = true;
        assert(frames.drain_releases() == 0 && !frames.quiescent());
        assert(camera.outstanding() == 1 && status.stats().lease_release_fail == 1);
        assert(frames.drain_releases() == 1 && frames.quiescent());
        assert(camera.outstanding() == 0 && status.stats().accepted == 1);
        frames.close();
        assert(camera.stop());
    }
    static void stride_and_no_copy() {
        char path[] = "/tmp/dart-recorder-stride-XXXXXX";
        assert(::mkdtemp(path));
        recorder::Config config;
        config.width = 2;
        config.height = 2;
        config.output_directory = path;
        recorder::FrameChannel frames;
        recorder::RecorderStatus status;
        auto created = recorder::RecorderFactory::create(spec(), config, frames, status);
        assert(created);
        auto component = std::move(*created);
        assert(!component->step());
        assert(component->initialze() && component->step());
        // Padding must not enter any format; the final row needs no trailing padding.
        std::array<std::byte, 6> pixels{std::byte{1}, std::byte{2}, std::byte{99}, std::byte{99}, std::byte{3}, std::byte{4}};
        camera::FrameLeaseAccess::Slot slot{};
        slot.view = {pixels, 2, 2, 4};
        slot.view.received = std::chrono::seconds{1};
        slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
        slot.release = [](camera::FrameLeaseAccess::Slot* held) noexcept -> std::expected<void, camera::CameraError> {
            held->state = camera::FrameLeaseAccess::SlotState::kFree;
            return {};
        };
        auto lease = camera::FrameLeaseAccess::make(&slot);
        assert(frames.try_submit(std::move(lease)) == recorder::Admission::kAccepted);
        pixels[0] = std::byte{7}; // Sequential observation proves submission did not snapshot pixels.
        assert(component->step() && frames.drain_releases() == 1 && component->shutdown());
        std::ifstream input(status.details().recording_path.data(), std::ios::binary);
        const std::string actual{std::istreambuf_iterator<char>{input}, {}};
        assert(actual == std::string("P5\n2 2\n255\n") + std::string("\7\2\3\4", 4));
        assert(status.stats().copied == 0 && status.stats().written == 1);
        for (const auto format : {recorder::ImageFormat::kPbm, recorder::ImageFormat::kPng, recorder::ImageFormat::kDbv}) {
            config.image_format = format;
            recorder::Encoder encoder{config};
            assert(encoder.prepare());
            const auto padded = encoder.encode(slot.view);
            assert(padded.valid() && !padded.borrowed_input);
            const std::string expected(reinterpret_cast<const char*>(padded.pixels.data()), padded.pixels.size());
            std::array<std::byte, 4> tight{pixels[0], pixels[1], pixels[4], pixels[5]};
            recorder::Encoder comparison{config};
            assert(comparison.prepare());
            camera::FrameView view{tight, 2, 2, 2};
            const auto result = comparison.encode(view);
            assert(expected == std::string(reinterpret_cast<const char*>(result.pixels.data()), result.pixels.size()));
        }
        std::filesystem::remove_all(path);
    }
    static void stalled_storage(recorder::ImageFormat format, bool failure) {
        char path[] = "/tmp/dart-recorder-component-XXXXXX";
        assert(::mkdtemp(path));
        Camera camera;
        control::BlackBoard board;
        recorder::Config config;
        config.width = camera.info.width;
        config.height = camera.info.height;
        config.image_format = format;
        config.frame_timeout = std::chrono::seconds{5};
        config.output_directory = path;
        recorder::FrameChannel frames;
        recorder::RecorderStatus status;
        executor::Tunner runner;
        auto created = recorder::RecorderFactory::create(spec(), config, frames, status);
        assert(created && runner.start(std::move(*created)));
        until([&] { return status.started(); });
        detection::DetectorStatus detection_status;
        const executor::Component::Spec detector_spec{"detector", "detector", 240.0, {}};
        auto detector_result = detection::DetectorFactory::create(detector_spec, camera,
            executor::Output<detection::Point>{board.target}, detection_status,
            [&](camera::FrameLease& lease, const detection::DetectResult*) {
                (void)frames.drain_releases();
                (void)frames.try_submit(std::move(lease));
            });
        assert(detector_result);
        auto detector = std::move(*detector_result);
        assert(detector->initialze());
        sync_entered = false;
        stall_sync = true;
        assert(detector->step());
        until([&] { return sync_entered.load(); });
        const auto begin = std::chrono::steady_clock::now();
        for (unsigned i = 0; i < 100; ++i) {
            (void)frames.drain_releases();
            assert(detector->step());
            assert(detection_status.acquisition == camera::AcquireStatus::kFrame);
            assert(camera.outstanding() <= (format == recorder::ImageFormat::kPgm ? 1U : 0U));
        }
        assert(std::chrono::steady_clock::now() - begin < std::chrono::seconds{1});
        assert(camera.acquires == 101 && status.stats().busy == 100 && status.stats().accepted == 1);
        assert(status.stats().copied == 0 && status.stats().written == 0);
        fail_sync = failure;
        stall_sync = false;
        until([&] {
            (void)frames.drain_releases();
            return failure ? status.last_error() != recorder::Error::kNone : status.stats().written == 1;
        });
        // Stop never requires the capture owner to concurrently drain returns.
        runner.stop();
        (void)frames.drain_releases();
        assert(frames.quiescent() && camera.outstanding() == 0);
        assert(camera.releases == camera.acquires);
        assert(detector->shutdown());
        assert(failure ? status.stats().write_fail > 0 : status.stats().written == 1);
        fail_sync = false;
        std::filesystem::remove_all(path);
    }
public:
    static void run() {
        channel_ownership();
        stride_and_no_copy();
        stalled_storage(recorder::ImageFormat::kPgm, false);
        stalled_storage(recorder::ImageFormat::kPng, false);
        stalled_storage(recorder::ImageFormat::kPng, true);
    }
};

} // namespace dart::tests

int main() { dart::tests::RecorderComponentTest::run(); }
