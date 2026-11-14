#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "tests/detector_fixture.hpp"

namespace dart::tests {

using detection::DetectReason;
using detection::DetectionPhase;
using detection::TargetKind;

class DetectorTrackingTest final {
    class Scene {
    public:
        ColorImage image{320, 240, 2};
        std::uint64_t sequence = 0;
        std::int64_t received_us = 0;

        detection::DetectResult next(DetectionCalculator& detector, std::int64_t elapsed_us = 10'000) {
            received_us += elapsed_us;
            auto result = detector.Detect(image.view(++sequence, received_us));
            assert(result);
            return *result;
        }
        void lamp(float x, float y, std::uint16_t color = kGreen) {
            image.clear();
            image.disk(x, y, 4, color);
        }
        // Lamp at (x, 160) and a blue plate centered 18 px above it.
        void armor(float x, bool bars = true) {
            image.clear();
            image.disk(x, 160, 4, kGreen);
            if (bars) {
                image.rect(static_cast<std::uint32_t>(x) - 11, 134, 3, 16, kBlue);
                image.rect(static_cast<std::uint32_t>(x) + 9, 134, 3, 16, kBlue);
            }
        }
    };

    static bool near(float value, float expected, float tolerance = 1e-3F) {
        return std::fabs(value - expected) <= tolerance;
    }

    static DetectionCalculator::Config config() {
        DetectionCalculator::Config c{};
        c.light.deadband_px = 0;
        c.track.velocity_alpha = 1;
        c.track.validation_interval = 100;
        return c;
    }

public:
    static void confirmation_prediction_and_identity() {
        DetectionCalculator detector{config()};
        Scene s;
        assert(detector.Prepare(s.image.info()));
        s.lamp(40, 80);
        auto result = s.next(detector);
        assert(!result.target.valid && result.reason == DetectReason::kLightConfirm && result.light_hits == 1);
        s.lamp(48, 80);
        result = s.next(detector);
        assert(result.kind == TargetKind::kLight && result.scan.phase == DetectionPhase::kDetect);
        s.lamp(56, 80);
        result = s.next(detector); // Learns 0.8 px/ms from receive times.
        assert(result.light.valid && result.scan.phase == DetectionPhase::kTrack && !result.scan.full_frame);

        // One frame interval skipped: 16 px exceeds the 13.5 px association
        // radius around the last position, but not around the prediction.
        s.lamp(72, 80);
        s.image.disk(250, 200, 7, kGreen); // A larger lamp elsewhere never takes the lock.
        result = s.next(detector, 20'000);
        assert(result.light.valid && near(result.target.center.x, 72) && near(result.target.center.y, 80));
        assert(result.scan.search_window.x > 40 && !result.scan.rescanned);

        // The tracked lamp vanishes: the same-frame full rescan stays bound to it.
        s.image.clear();
        s.image.disk(250, 200, 7, kGreen);
        for (std::uint32_t miss = 1; miss <= 3; ++miss) {
            result = s.next(detector);
            assert(!result.target.valid && result.missed_frames == miss);
            assert(result.scan.rescanned && result.scan.full_frame);
            assert(result.scan.counters.rejected_association > 0);
        }
        // After the reset the other lamp is a new acquisition and needs confirmation.
        result = s.next(detector);
        assert(result.scan.phase == DetectionPhase::kDetect && !result.target.valid);
        result = s.next(detector);
        assert(result.kind == TargetKind::kLight && near(result.target.center.x, 250));
    }

    static void color_is_part_of_identity() {
        auto c = config();
        c.light.colors = detection::mask_of(detection::Color::kGreen) | detection::mask_of(detection::Color::kBlue);
        DetectionCalculator detector{c};
        Scene s;
        assert(detector.Prepare(s.image.info()));
        s.lamp(100, 100);
        s.next(detector);
        assert(s.next(detector).target.valid);
        s.lamp(100, 100, kBlue);
        const auto result = s.next(detector);
        assert(!result.target.valid && result.scan.counters.rejected_association > 0);
        // During acquisition a color change also restarts confirmation.
        detector.Reset();
        s.lamp(100, 100);
        s.next(detector);
        s.lamp(100, 100, kBlue);
        assert(!s.next(detector).target.valid);
        assert(s.next(detector).target.color == detection::Color::kBlue);
    }

    static void deadband_applies_to_publication_only() {
        auto c = config();
        c.light.deadband_px = 2;
        c.light.confirm_frames = 1;
        DetectionCalculator detector{c};
        Scene s;
        assert(detector.Prepare(s.image.info()));
        s.lamp(100, 100);
        assert(near(s.next(detector).target.center.x, 100));
        s.lamp(101, 100);
        assert(near(s.next(detector).target.center.x, 100)); // Inside the deadband.
        s.lamp(102, 100);
        assert(near(s.next(detector).target.center.x, 100)); // Still 2 px from the published value.
        s.lamp(105, 100);
        assert(near(s.next(detector).target.center.x, 105));
    }

    static void gaps_and_frame_order() {
        DetectionCalculator detector{config()};
        Scene s;
        assert(detector.Prepare(s.image.info()));
        s.lamp(100, 100);
        s.next(detector);
        assert(s.next(detector).target.valid);
        auto result = s.next(detector, 300'000); // Longer than roi.max_gap.
        assert(result.scan.gap_reset && result.scan.phase == DetectionPhase::kDetect && !result.target.valid);
        assert(s.next(detector).target.valid);

        auto stale = s.image.view(s.sequence, s.received_us + 10'000); // Repeated sequence.
        assert(detector.Detect(stale).error() == detection::DetectionError::kOutOfOrderFrame);
        stale = s.image.view(s.sequence + 1, s.received_us - 1); // Receive time went back.
        assert(detector.Detect(stale).error() == detection::DetectionError::kOutOfOrderFrame);
        // The stream continues, but history was dropped.
        result = s.next(detector);
        assert(result.scan.phase == DetectionPhase::kDetect && !result.target.valid);
    }

    static void armor_confirmation_and_hold() {
        auto c = config();
        c.light.confirm_frames = 1;
        DetectionCalculator detector{c};
        Scene s;
        assert(detector.Prepare(s.image.info()));
        s.armor(160);
        auto result = s.next(detector);
        assert(result.kind == TargetKind::kLight && result.reason == DetectReason::kArmorConfirm);
        result = s.next(detector);
        assert(result.kind == TargetKind::kArmor && near(result.target.center.x, 160.0F));
        assert(near(result.target.center.y, 141.5F) && result.scan.phase == DetectionPhase::kTrack);
        s.armor(162);
        result = s.next(detector); // A tracked plate moves with its bars.
        assert(result.kind == TargetKind::kArmor && near(result.target.center.x, 162.0F));

        s.armor(162, false);
        result = s.next(detector);
        assert(result.kind == TargetKind::kLight && result.scan.armor_scanned); // One miss is held.
        result = s.next(detector);
        assert(result.kind == TargetKind::kLight); // Second miss releases the pair.
        s.armor(162);
        result = s.next(detector);
        assert(result.kind == TargetKind::kLight && result.reason == DetectReason::kArmorConfirm);
        assert(s.next(detector).kind == TargetKind::kArmor);
    }

    static void validation_cadence() {
        auto c = config();
        c.light.confirm_frames = 1;
        c.track.validation_interval = 3;
        DetectionCalculator detector{c};
        Scene s;
        assert(detector.Prepare(s.image.info()));
        s.lamp(100, 120);
        assert(s.next(detector).scan.armor_scanned); // Acquisition always validates.
        unsigned scanned = 0;
        for (int frame = 0; frame < 9; ++frame) {
            const auto result = s.next(detector);
            assert(result.kind == TargetKind::kLight);
            scanned += result.scan.armor_scanned;
            assert(result.reason == (result.scan.armor_scanned ? DetectReason::kNoArmorColor
                                                               : DetectReason::kLightLocked));
        }
        assert(scanned == 3);
    }
};

} // namespace dart::tests

int main() {
    using dart::tests::DetectorTrackingTest;
    DetectorTrackingTest::confirmation_prediction_and_identity();
    DetectorTrackingTest::color_is_part_of_identity();
    DetectorTrackingTest::deadband_applies_to_publication_only();
    DetectorTrackingTest::gaps_and_frame_order();
    DetectorTrackingTest::armor_confirmation_and_hold();
    DetectorTrackingTest::validation_cadence();
}
