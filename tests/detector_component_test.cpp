#include "src/detector/detect.hpp"
#include "tests/detector_fixture.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <limits>
#include <optional>
#include <stop_token>

#include "src/control/blackboard.hpp"

namespace dart::tests {

class DetectorComponentTest final {
    class Camera final : public camera::ICamera {
    public:
        Camera() {
            slot.owner = this;
            slot.release = release;
        }

        std::expected<void, camera::CameraError> start() noexcept override {
            ++starts;
            if (fail_start)
                return std::unexpected(error(camera::Operation::kStart));
            state_ = camera::State::kStarted;
            return {};
        }

        std::expected<camera::AcquireResult, camera::CameraError>
            acquire(std::chrono::milliseconds timeout) noexcept override {
            assert(state_ == camera::State::kStarted);
            assert(timeout == std::chrono::milliseconds{10});
            ++acquires;
            if (fail_acquire)
                return std::unexpected(error(camera::Operation::kAcquire));
            camera::AcquireResult result;
            result.status = acquisition;
            if (acquisition == camera::AcquireStatus::kFrame) {
                assert(slot.state == camera::FrameLeaseAccess::SlotState::kFree);
                slot.view = image.view(sequence + 1, static_cast<std::int64_t>(sequence + 1) * 10000);
                slot.view.format = info.format;
                slot.view.width = invalid_frame ? 0 : info.width;
                slot.view.sequence = ++sequence;
                slot.view.received = std::chrono::microseconds{sequence * 10000};
                slot.token = sequence;
                slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
                result.lease = camera::FrameLeaseAccess::make(&slot);
            }
            return result;
        }

        std::expected<void, camera::CameraError> stop() noexcept override {
            ++stops;
            assert(outstanding() == 0);
            state_ = camera::State::kInitialized;
            if (fail_stop)
                return std::unexpected(error(camera::Operation::kStop));
            return {};
        }
        camera::State state() const noexcept override { return state_; }
        const camera::FrameInfo& frame_info() const noexcept override { return info; }
        std::uint32_t capacity() const noexcept override { return 1; }
        std::uint32_t outstanding() const noexcept override {
            return slot.state == camera::FrameLeaseAccess::SlotState::kHeld;
        }

        // A green lamp with a blue plate above it.
        void scene(bool armor = true) {
            image.clear();
            image.rect(150, 160, 8, 8, kGreen);
            if (armor) {
                image.rect(144, 140, 2, 12, kBlue);
                image.rect(164, 140, 2, 12, kBlue);
            }
        }

        ColorImage image{320, 240};
        camera::FrameInfo info = image.info();
        camera::FrameLeaseAccess::Slot slot{};
        camera::AcquireStatus acquisition = camera::AcquireStatus::kFrame;
        std::uint64_t sequence = 0;
        unsigned starts = 0, stops = 0, acquires = 0, releases = 0;
        bool fail_start = false, fail_acquire = false, fail_release = false, fail_stop = false;
        bool invalid_frame = false;

    private:
        static camera::CameraError error(camera::Operation operation) {
            camera::CallContext context{};
            context.operation = operation;
            return {DART_CAMERA_E_INVALID_STATE, context};
        }
        static std::expected<void, camera::CameraError>
            release(camera::FrameLeaseAccess::Slot* slot) noexcept {
            auto& self = *static_cast<Camera*>(slot->owner);
            ++self.releases;
            if (self.fail_release)
                return std::unexpected(error(camera::Operation::kRelease));
            slot->state = camera::FrameLeaseAccess::SlotState::kFree;
            return {};
        }
        camera::State state_ = camera::State::kInitialized;
    };

    static executor::Component::Spec spec() {
        return {
            "detector",
            "detector",
            240,
            {{"acquire_ms", 10},
             {"max_acquire_errors", 2},
             {"light.search_step", 1},
             {"light.confirm_frames", 1},
             {"light.far_confirm_frames", 1},
             {"armor.confirm_frames", 1},
             {"light.deadband_px", 0}}};
    }

    static void expect_missing(const control::BlackBoard& board) {
        const auto point = board.target.read();
        assert(point.x == -1 && point.y == -1);
    }

public:
    static void publication_and_ownership() {
        Camera camera;
        camera.scene();
        control::BlackBoard board;
        detection::DetectorStatus status;
        unsigned callbacks = 0;
        std::optional<camera::FrameLease> transferred;
        bool transfer = false;
        auto created = detection::DetectorFactory::create(
            spec(), camera, executor::Output<detection::Point>{board.target}, status,
            [&](camera::FrameLease& lease, const detection::DetectResult* result) {
                ++callbacks;
                assert(lease.valid() && camera.outstanding() == 1);
                const auto published = board.target.read();
                if (result != nullptr) {
                    assert(published.x == result->target.center.x);
                    assert(published.y == result->target.center.y);
                } else
                    expect_missing(board);
                if (transfer)
                    transferred.emplace(std::move(lease));
            });
        assert(created);
        auto component = std::move(*created);
        assert(!component->step());
        assert(component->initialze());
        assert(camera.starts == 1);
        expect_missing(board);

        assert(component->step());
        const auto armor = board.target.read();
        assert(armor.x == 154.5F && armor.y == 145.5F);
        assert(callbacks == 1 && camera.releases == 1 && camera.outstanding() == 0);
        assert(status.released_frame && status.span_high_water > 0);

        camera.scene(false);
        assert(component->step());
        const auto light = board.target.read();
        assert(light.x == 153.5F && light.y == 163.5F);

        camera.image.clear();
        assert(component->step());
        expect_missing(board); // Retained tracking history is never published as a measurement.

        const auto before_negative = callbacks;
        for (const auto acquisition :
             {camera::AcquireStatus::kNoFrame, camera::AcquireStatus::kLeaseExhausted}) {
            camera.acquisition = acquisition;
            assert(component->step());
            assert(status.acquisition == acquisition);
            expect_missing(board);
        }
        assert(callbacks == before_negative);

        camera.acquisition = camera::AcquireStatus::kFrame;
        camera.invalid_frame = true;
        assert(component->step());
        assert(status.detection_error == detection::DetectionError::kInvalidFrame);
        expect_missing(board);
        assert(camera.outstanding() == 0);

        camera.invalid_frame = false;
        camera.scene();
        transfer = true;
        assert(component->step());
        assert(transferred && transferred->valid() && camera.outstanding() == 1);
        assert(!status.released_frame);
        assert(transferred->release());
        transferred.reset();
        const auto releases = camera.releases;
        assert(component->shutdown());
        expect_missing(board);
        assert(camera.stops == 1);
        component.reset();
        assert(camera.releases == releases && camera.stops == 1);
    }

    static void failures_and_parameters() {
        Camera camera;
        camera.scene();
        control::BlackBoard board;
        detection::DetectorStatus status;
        auto settings = spec();
        settings.parameter["light.min_area"] = 1.5;
        assert(!detection::DetectorFactory::create(
            settings, camera, executor::Output<detection::Point>{board.target}, status));
        settings = spec();
        settings.parameter["light.min_area"] = std::numeric_limits<double>::quiet_NaN();
        assert(!detection::DetectorFactory::create(
            settings, camera, executor::Output<detection::Point>{board.target}, status));
        settings = spec();
        settings.parameter["unknown"] = 1;
        assert(!detection::DetectorFactory::create(
            settings, camera, executor::Output<detection::Point>{board.target}, status));
        assert(camera.starts == 0);

        settings = spec();
        settings.parameter["workspace.max_spans"] = 0;
        auto invalid = detection::DetectorFactory::create(
            settings, camera, executor::Output<detection::Point>{board.target}, status);
        assert(invalid && !(*invalid)->initialze());
        assert(status.detection_error == detection::DetectionError::kInvalidConfig);
        assert(camera.starts == 0);
        invalid->reset();

        // A grayscale camera cannot feed the color detector; recording-only still works.
        Camera gray;
        gray.info.format = camera::PixelFormat::kY8;
        auto unsupported = detection::DetectorFactory::create(
            spec(), gray, executor::Output<detection::Point>{board.target}, status);
        assert(unsupported && !(*unsupported)->initialze());
        assert(status.detection_error == detection::DetectionError::kUnsupportedFormat);
        assert(gray.starts == 0);
        settings = spec();
        settings.parameter["enabled"] = 0;
        auto capture_only = detection::DetectorFactory::create(
            settings, gray, executor::Output<detection::Point>{board.target}, status);
        assert(capture_only && (*capture_only)->initialze() && (*capture_only)->step());
        assert(status.acquired_frame && !status.detection_error);
        assert((*capture_only)->shutdown());

        auto created = detection::DetectorFactory::create(
            spec(), camera, executor::Output<detection::Point>{board.target}, status);
        assert(created);
        auto component = std::move(*created);
        assert(component->initialze() && component->step());
        camera.fail_acquire = true;
        assert(component->step());
        expect_missing(board);
        assert(status.camera_error && status.consecutive_acquire_errors == 1);
        assert(!component->step());
        expect_missing(board);
        assert(component->shutdown());
        assert(camera.stops == 1);
    }

    static void release_failure_and_record_only() {
        Camera camera;
        camera.scene();
        control::BlackBoard board;
        detection::DetectorStatus status;
        auto created = detection::DetectorFactory::create(
            spec(), camera, executor::Output<detection::Point>{board.target}, status);
        assert(created && (*created)->initialze());
        camera.fail_release = true;
        assert(!(*created)->step());
        assert(status.release_error);
        expect_missing(board);
        camera.fail_release = false;
        assert((*created)->shutdown());
        assert(camera.outstanding() == 0 && camera.stops == 1);
        created->reset();

        auto settings = spec();
        settings.parameter["enabled"] = 0;
        unsigned frames = 0;
        auto record_only = detection::DetectorFactory::create(
            settings, camera, executor::Output<detection::Point>{board.target}, status,
            [&](camera::FrameLease& lease, const detection::DetectResult* result) {
                assert(lease.valid() && result == nullptr);
                ++frames;
            });
        assert(record_only && (*record_only)->initialze());
        assert((*record_only)->step());
        assert(frames == 1 && !status.vision_enabled);
        expect_missing(board);
        assert((*record_only)->shutdown());
    }

    static void run_and_start_stop_errors() {
        Camera camera;
        camera.scene();
        control::BlackBoard board;
        detection::DetectorStatus status;
        camera.fail_start = true;
        auto failed = detection::DetectorFactory::create(
            spec(), camera, executor::Output<detection::Point>{board.target}, status);
        assert(failed && !(*failed)->initialze());
        assert(status.camera_error->operation() == camera::Operation::kStart);
        expect_missing(board);
        failed->reset();
        assert(camera.stops == 0);

        camera.fail_start = false;
        std::stop_source stop;
        unsigned frames = 0;
        auto created = detection::DetectorFactory::create(
            spec(), camera, executor::Output<detection::Point>{board.target}, status,
            [&](camera::FrameLease&, const detection::DetectResult* result) {
                assert(result && result->armor.valid);
                if (++frames == 2)
                    stop.request_stop();
            });
        assert(created && (*created)->initialze());
        assert((*created)->run(stop.get_token()));
        assert(frames == 2 && camera.acquires == 2);
        assert((*created)->statistics().ticks == 2);
        assert(board.target.read().y == 145.5F);
        camera.fail_stop = true;
        assert(!(*created)->shutdown());
        expect_missing(board);
        assert(status.camera_error->operation() == camera::Operation::kStop);
        assert(camera.outstanding() == 0 && camera.stops == 1);
    }
};

} // namespace dart::tests

int main() {
    dart::tests::DetectorComponentTest::publication_and_ownership();
    dart::tests::DetectorComponentTest::failures_and_parameters();
    dart::tests::DetectorComponentTest::release_failure_and_record_only();
    dart::tests::DetectorComponentTest::run_and_start_stop_errors();
}
