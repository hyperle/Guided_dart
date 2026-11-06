#include <cstddef>
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <vector>

#include "tests/detector_fixture.hpp"

namespace dart::tests {

class DetectorTrackingTest {
    class Scene {
    public:
        std::vector<std::byte> pixels = std::vector<std::byte>(324 * 240);
        camera::FrameView frame{};
        Scene() {
            frame.luma = pixels;
            frame.width = 320; frame.height = 240; frame.stride = 324;
            clear();
        }
        void clear() { std::fill(pixels.begin(), pixels.end(), std::byte{20}); }
        void box(unsigned x, unsigned y, unsigned w, unsigned h, unsigned value = 180) {
            for (auto yy = y; yy < y + h; ++yy)
                for (auto xx = x; xx < x + w; ++xx)
                    pixels[yy * frame.stride + xx] = static_cast<std::byte>(value);
        }
        void plate(unsigned x = 144) { box(x, 140, 2, 12); box(x + 20, 140, 2, 12); }
        detection::DetectResult next(DetectionCalculator& detector, unsigned us = 10000) {
            ++frame.sequence;
            frame.received += std::chrono::microseconds{us};
            auto result = detector.Detect(frame);
            assert(result);
            return *result;
        }
    };

    static DetectionCalculator::Config config() {
        DetectionCalculator::Config c{};
        c.light.search_step = 1;
        c.light.confirm_frames = c.light.far_confirm_frames = 1;
        c.light.deadband_px = 0;
        c.armor.confirm_frames = 1;
        c.roi.min_width = c.roi.min_height = 24;
        c.roi.size_k = 1;
        c.roi.margin = 12;
        c.roi.full_area_fraction = 0.95F;
        c.track.association_k = 1;
        c.track.association_min_px = 3;
        c.track.velocity_alpha = 1;
        c.track.validation_interval = 3;
        return c;
    }

public:
    static void prediction_and_identity() {
        auto c = config();
        c.light.confirm_frames = 2;
        c.track.validation_interval = 100;
        DetectionCalculator detector{c};
        assert(detector.Prepare(320, 240));
        Scene s;
        s.box(40, 80, 8, 8);
        assert(!s.next(detector).target.valid);
        s.clear(); s.box(48, 80, 8, 8);
        assert(s.next(detector).light.valid);
        s.clear(); s.box(56, 80, 8, 8);
        assert(s.next(detector).light.valid); // Learn 800 px/s from true receive times.
        s.clear(); s.box(72, 80, 8, 8); s.box(52, 80, 16, 16, 220);
        const auto moving = s.next(detector, 20000); // One frame interval was skipped.
        assert(moving.scan.phase == detection::DetectionPhase::kTrack);
        assert(moving.light.valid && moving.light.bounds.x == 72);
        assert(!moving.scan.rescanned && !moving.scan.full_frame);
        assert(moving.scan.search_window.x > 45); // ROI moved toward the prediction.
        // An unrelated bright target must not inherit the lock, even on the full rescan.
        for (unsigned i = 0; i < c.roi.reset_after_misses; ++i) {
            s.clear(); s.box(200, 80, 16, 16, 220);
            const auto missing = s.next(detector);
            assert(!missing.target.valid && missing.scan.phase == detection::DetectionPhase::kTrack);
        }
        assert(!s.next(detector).target.valid); // First new detection still needs confirmation.
        const auto acquired = s.next(detector);
        assert(acquired.scan.phase == detection::DetectionPhase::kDetect);
        assert(acquired.light.valid && acquired.light.bounds.x == 200);
    }

    static void validation_schedule() {
        auto c = config();
        DetectionCalculator detector{c};
        assert(detector.Prepare(320, 240));
        Scene s;
        s.box(150, 160, 8, 8); s.plate();
        for (unsigned x = 134; x < 178; x += 6) s.box(x, 132, 2, 2);
        const auto acquired = s.next(detector);
        assert(acquired.armor.valid && acquired.armor.stability_checked);
        for (unsigned n = 1; n <= 6; ++n) {
            const auto result = s.next(detector);
            assert(result.armor.valid && result.scan.phase == detection::DetectionPhase::kTrack);
            const bool due = n % 3 == 0;
            assert(result.scan.full_validation == due);
            assert(result.scan.stability_checks == (due ? 3 : 0));
            assert(result.target.stability_checked == due);
        }
        // Quality loss triggers validation before the next scheduled frame.
        s.box(150, 160, 8, 8, 120);
        const auto dimmer = s.next(detector);
        assert(dimmer.light.valid && dimmer.scan.full_validation);
        assert(dimmer.scan.stability_checks == 3);
    }

    static void periodic_rejection_and_snr() {
        auto c = config();
        DetectionCalculator detector{c};
        assert(detector.Prepare(320, 240));
        Scene s;
        s.box(150, 160, 8, 8, 80);
        assert(s.next(detector).target.valid);
        s.box(150, 160, 8, 8, 54);
        s.box(150, 160, 2, 8, 100); s.box(156, 160, 2, 8, 100);
        for (unsigned i = 0; i < 2; ++i) {
            const auto fast = s.next(detector);
            assert(fast.light.valid && !fast.light.stability_checked);
        }
        const auto rejected = s.next(detector);
        assert(!rejected.target.valid && rejected.scan.full_validation);
        assert(rejected.scan.rejected_stability > 0);
        // SNR is checked on every tracking frame, independent of the cadence.
        detector.Reset(); s.clear(); s.box(150, 160, 8, 8, 80);
        assert(s.next(detector).light.valid);
        for (unsigned y = 0; y < 240; ++y)
            for (unsigned x = 0; x < 320; ++x)
                s.pixels[y * 324 + x] = static_cast<std::byte>((x + y) % 2 ? 0 : 40);
        s.box(150, 160, 8, 8, 80);
        const auto noisy = s.next(detector);
        assert(!noisy.target.valid && noisy.scan.rejected_snr > 0);
    }

    static void switching_armor_requires_confirmation() {
        auto c = config();
        c.armor.confirm_frames = 2;
        c.armor.window_width_k = 10;
        c.track.validation_interval = 100;
        DetectionCalculator detector{c};
        assert(detector.Prepare(320, 240));
        Scene s;
        s.box(150, 160, 8, 8); s.plate();
        assert(!s.next(detector).armor.valid);
        const auto original = s.next(detector);
        assert(original.armor.valid && original.armor.center.x < 160);
        s.clear(); s.box(150, 160, 8, 8); s.plate(174);
        for (unsigned i = 0; i < 3; ++i) {
            const auto switching = s.next(detector);
            assert(switching.light.valid && !switching.armor.valid);
            assert(switching.kind == detection::TargetKind::kLight);
        }
        const auto replacement = s.next(detector);
        assert(replacement.armor.valid && replacement.armor.center.x > 180);
        assert(replacement.armor.stability_checked);
    }

    static void acquisition_and_association_gates() {
        auto c = config(); c.light.confirm_frames = 2;
        DetectionCalculator detector{c};
        assert(detector.Prepare(320, 240));
        Scene s;
        s.box(30, 80, 12, 12);
        assert(!s.next(detector).target.valid);
        s.clear(); s.box(200, 80, 12, 12);
        assert(!s.next(detector).target.valid); // Near targets also reset the confirmation streak.
        assert(s.next(detector).target.valid);
        detector.Reset(); s.clear(); s.box(80, 80, 8, 8);
        assert(!s.next(detector).target.valid);
        assert(s.next(detector).target.valid);
        s.box(80, 80, 8, 8, 80);
        assert(!s.next(detector).target.valid); // Contrast changed beyond the association gate.
        c.track.min_size_ratio = 0.75F;
        DetectionCalculator size_detector{c};
        assert(size_detector.Prepare(320, 240));
        s.clear(); s.box(80, 80, 8, 8);
        assert(!s.next(size_detector).target.valid);
        assert(s.next(size_detector).target.valid);
        s.clear(); s.box(76, 76, 16, 16);
        assert(!s.next(size_detector).target.valid);
    }

    static void time_reset_and_config() {
        auto c = config();
        DetectionCalculator detector{c};
        assert(detector.Prepare(320, 240));
        Scene s; s.box(100, 100, 8, 8);
        assert(s.next(detector).scan.phase == detection::DetectionPhase::kDetect);
        assert(s.next(detector).scan.phase == detection::DetectionPhase::kTrack);
        assert(s.next(detector, 300000).scan.phase == detection::DetectionPhase::kDetect);
        const auto duplicate = detector.Detect(s.frame);
        assert(!duplicate && duplicate.error() == detection::DetectionError::kOutOfOrderFrame);
        c.track.validation_interval = 0;
        DetectionCalculator invalid{c}; assert(!invalid.Prepare(320, 240));
        c.track.validation_interval = 3; c.track.min_size_ratio = 0;
        DetectionCalculator invalid_ratio{c}; assert(!invalid_ratio.Prepare(320, 240));
        c.track.min_size_ratio = 0.5F; c.track.velocity_alpha = 1.1F;
        DetectionCalculator invalid_alpha{c}; assert(!invalid_alpha.Prepare(320, 240));
    }
};

} // namespace dart::tests

int main() {
    dart::tests::DetectorTrackingTest::prediction_and_identity();
    dart::tests::DetectorTrackingTest::validation_schedule();
    dart::tests::DetectorTrackingTest::periodic_rejection_and_snr();
    dart::tests::DetectorTrackingTest::switching_armor_requires_confirmation();
    dart::tests::DetectorTrackingTest::acquisition_and_association_gates();
    dart::tests::DetectorTrackingTest::time_reset_and_config();
}
