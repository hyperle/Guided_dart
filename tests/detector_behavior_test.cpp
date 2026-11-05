#include <cstdint>
#include <limits>

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>

#include "src/detector/roi.hpp"

namespace dart::tests {

class DetectorBehaviorTest final {
public:
    static void grayscale() {
        // Both patches produce the same mask at 50. Only the high-contrast
        // patch should survive grayscale validation, at either scan step.
        std::array<std::byte, 68 * 64> pixels;
        pixels.fill(std::byte{49}); // Includes row padding to catch stride mistakes.
        camera::FrameView frame{};
        frame.luma = pixels;
        frame.width = frame.height = 64;
        frame.stride = 68;
        detection::BlobWorkspace workspace;
        assert(workspace.Prepare(64, 64));
        for (const auto step : {1U, 2U}) {
            for (const auto level : {50U, 61U, 180U}) {
                for (unsigned y = 28; y < 36; ++y)
                    for (unsigned x = 28; x < 36; ++x)
                        pixels[y * frame.stride + x] = static_cast<std::byte>(level);
                const auto blobs = workspace.Scan(frame, {0, 0, 64, 64}, 50, 2, step);
                assert(blobs && blobs->size() == (level < 61 ? 0 : 1));
                if (!blobs->empty()) {
                    assert((*blobs)[0].mean_luma == level);
                    assert((*blobs)[0].background_luma == 49);
                    assert((*blobs)[0].contrast == level - 49);
                }
            }
        }
        // Estimate the ring from the frame even when it lies beyond this ROI.
        const auto cropped = workspace.Scan(frame, {28, 28, 8, 8}, 50);
        assert(cropped && cropped->size() == 1 && (*cropped)[0].contrast == 131);
        // Bright background rejects the same shape and absolute intensity.
        pixels.fill(std::byte{179});
        for (unsigned y = 28; y < 36; ++y)
            for (unsigned x = 28; x < 36; ++x)
                pixels[y * frame.stride + x] = std::byte{180};
        const auto washed = workspace.Scan(frame, {0, 0, 64, 64}, 180);
        assert(washed && washed->empty());
        // Explicitly disabling all grayscale gates restores threshold-only extraction.
        detection::BlobWorkspace::Config config{};
        config.min_contrast = 0;
        config.min_snr = 0;
        config.min_stability = 0;
        assert(workspace.Prepare(64, 64, config));
        const auto legacy = workspace.Scan(frame, {0, 0, 64, 64}, 180);
        assert(legacy && legacy->size() == 1);
        config.min_contrast = -1;
        assert(!workspace.Prepare(64, 64, config));
        config.min_contrast = std::numeric_limits<float>::quiet_NaN();
        assert(!workspace.Prepare(64, 64, config));
        // An image-filling component has no surrounding evidence.
        pixels.fill(std::byte{180});
        assert(workspace.Prepare(64, 64));
        const auto full = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(full && full->empty());
        // A frame-edge light still has an observable ring on its other sides.
        pixels.fill(std::byte{20});
        for (unsigned y = 0; y < 8; ++y)
            for (unsigned x = 0; x < 8; ++x)
                pixels[y * frame.stride + x] = std::byte{100};
        const auto edge = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(edge && edge->size() == 1 && (*edge)[0].contrast == 80);
    }

    static void grayscale_features() {
        std::array<std::byte, 68 * 64> pixels{};
        camera::FrameView frame{};
        frame.luma = pixels;
        frame.width = frame.height = 64;
        frame.stride = 68;
        const auto box = [&](unsigned x, unsigned y, unsigned w, unsigned h, unsigned level) {
            for (auto row = y; row < y + h; ++row)
                for (auto col = x; col < x + w; ++col)
                    pixels[row * frame.stride + col] = static_cast<std::byte>(level);
        };
        detection::BlobWorkspace workspace;
        assert(workspace.Prepare(64, 64));
        // Equal foreground/background means, very different background variance.
        for (const auto step : {1U, 2U}) {
            pixels.fill(std::byte{20});
            box(28, 28, 8, 8, 80);
            auto quiet = workspace.Scan(frame, {0, 0, 64, 64}, 50, 2, step);
            assert(quiet && quiet->size() == 1);
            assert((*quiet)[0].contrast == 60 && (*quiet)[0].snr == 15);
            for (unsigned y = 0; y < 64; ++y)
                for (unsigned x = 0; x < 64; ++x)
                    pixels[y * 68 + x] = static_cast<std::byte>((x + y) % 2 ? 0 : 40);
            box(28, 28, 8, 8, 80);
            auto noisy = workspace.Scan(frame, {0, 0, 64, 64}, 50, 2, step);
            assert(noisy && noisy->empty());
        }
        detection::BlobWorkspace::Config config{};
        config.min_snr = 0; // Isolate stability from the independent SNR gate.
        assert(workspace.Prepare(64, 64, config));
        pixels.fill(std::byte{20});
        box(28, 28, 8, 8, 54); // Appears at T, disappears at T+8.
        auto weak = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(weak && weak->empty());
        // A smooth centered halo can shrink to 36% of its original area.
        box(27, 27, 10, 10, 54);
        box(29, 29, 6, 6, 80);
        auto halo = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(halo && halo->size() == 1 && (*halo)[0].stability > 0.59F);
        pixels.fill(std::byte{20});
        box(24, 28, 12, 4, 54);
        box(24, 28, 3, 4, 80);
        box(33, 28, 3, 4, 80); // Same mask, but splits into two equal pieces.
        auto split = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(split && split->empty());
        pixels.fill(std::byte{20});
        box(24, 28, 16, 8, 46);
        box(24, 28, 4, 8, 80);
        box(36, 28, 4, 8, 80); // Lower threshold joins two separate lights.
        auto merged = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(merged && merged->empty());
        pixels.fill(std::byte{20});
        box(20, 28, 20, 4, 54);
        box(34, 28, 6, 4, 80); // Retains enough area, but center moves too far.
        auto shifted = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(shifted && shifted->empty());
        pixels.fill(std::byte{20});
        box(28, 28, 8, 8, 80);
        const auto full = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(full && full->size() == 1);
        const auto measured = (*full)[0];
        const auto crop = workspace.Scan(frame, {27, 27, 10, 10}, 50);
        assert(crop && crop->size() == 1);
        assert((*crop)[0].stability == measured.stability);
        assert((*crop)[0].snr == measured.snr);
        // Far targets, image edges, saturated thresholds and off-origin lattices.
        pixels.fill(std::byte{0});
        box(0, 0, 3, 3, 255);
        for (const auto step : {1U, 2U}) {
            for (const auto threshold : {0U, 1U, 247U, 255U}) {
                const auto edge = workspace.Scan(frame, {0, 0, 64, 64}, threshold, 2, step);
                assert(edge && edge->size() == (threshold == 0 ? 0 : 1));
            }
        }
        pixels.fill(std::byte{20});
        box(29, 29, 6, 6, 180);
        const auto odd_roi = workspace.Scan(frame, {27, 27, 12, 12}, 50, 2, 2);
        assert(odd_roi && odd_roi->size() == 1 && (*odd_roi)[0].stability == 1);
        config.min_stability = 1.1F;
        assert(!workspace.Prepare(64, 64, config));
        config.min_stability = 0.5F;
        config.min_snr = -1;
        assert(!workspace.Prepare(64, 64, config));
        config.min_snr = 3;
        config.stability_delta = 256;
        assert(!workspace.Prepare(64, 64, config));
    }

    static void armor_grayscale() {
        std::array<std::byte, 64 * 64> pixels;
        pixels.fill(std::byte{20});
        camera::FrameView frame{};
        frame.luma = pixels;
        frame.width = frame.height = frame.stride = 64;
        for (unsigned y = 20; y < 32; ++y)
            for (const auto x : {20U, 21U, 40U, 41U}) pixels[y * 64 + x] = std::byte{180};
        detection::BlobWorkspace workspace;
        assert(workspace.Prepare(64, 64));
        auto blobs = workspace.Scan(frame, {0, 0, 64, 64}, 50);
        assert(blobs && blobs->size() == 2);
        std::array<detection::Blob, 2> bars{(*blobs)[0], (*blobs)[1]};
        detection::Target anchor{};
        anchor.valid = true;
        anchor.center = {30.5F, 50};
        anchor.bounds = {27, 46, 8, 8};
        anchor.radius = 4;
        detection::ArmorDetector detector;
        const auto target = detector.Detect(bars, anchor, nullptr, &frame);
        assert(target.valid && target.brightness_ratio == 1 && target.gap_contrast == 160);
        assert(target.quality > 0.99F);
        bars[1].mean_luma = 70;
        bars[1].contrast = 50;
        assert(!detector.Detect(bars, anchor, nullptr, &frame).valid);
        bars[1] = (*blobs)[1];
        // Keep the same measured pair geometry to isolate the gap classifier.
        for (unsigned y = 20; y < 32; ++y)
            for (unsigned x = 22; x < 40; ++x) pixels[y * 64 + x] = std::byte{175};
        assert(!detector.Detect(bars, anchor, nullptr, &frame).valid);
        detection::ArmorDetector::Config config{};
        config.min_gap_contrast = 0;
        detection::ArmorDetector relaxed{config};
        const auto low_quality = relaxed.Detect(bars, anchor, nullptr, &frame);
        assert(low_quality.valid && low_quality.quality < target.quality);
    }

    static void grayscale_ranking_and_tracking() {
        std::array<detection::Blob, 2> blobs{};
        for (auto& blob : blobs) {
            blob.bounds = {10, 10, 8, 8};
            blob.area = 64;
            blob.fill = blob.circularity = 1;
        }
        blobs[0].center = {14, 14};
        blobs[0].gray_quality = 0.2F;
        blobs[1].center = {40, 40};
        blobs[1].gray_quality = 0.9F;
        detection::LightDetector detector;
        assert(detector.Detect(blobs, 4096).center.x == 40);
        detection::LightTracker tracker{detection::LightDetector::Config{}};
        detection::Target light{};
        light.valid = true;
        light.area = 4;
        light.bounds = {10, 10, 2, 2};
        light.center = {10.5F, 10.5F};
        assert(!tracker.Confirm(light, 1, {}));
        assert(!tracker.Confirm(light, 2, {}));
        assert(tracker.Confirm(light, 3, {}));
        light.center = {40.5F, 40.5F};
        assert(!tracker.Confirm(light, 4, {})); // A new small spot must be confirmed again.
        assert(!tracker.Confirm(light, 4, {})); // Same-frame rescan never adds a hit.
        assert(!tracker.Confirm(light, 5, {}));
        assert(tracker.Confirm(light, 6, {}));
    }

    static void run() {
        std::array<std::byte, 64 * 64> pixels{};
        for (std::size_t y = 28; y < 36; ++y)
            for (std::size_t x = 28; x < 36; ++x)
                pixels[y * 64 + x] = std::byte{255};
        camera::FrameView frame{};
        frame.luma = pixels;
        frame.width = frame.height = frame.stride = 64;
        frame.sequence = 1;
        frame.received = std::chrono::microseconds{1000};

        detection::BlobWorkspace workspace;
        auto unprepared = workspace.Scan(frame, {0, 0, 64, 64}, 128);
        assert(!unprepared && unprepared.error() == detection::DetectionError::kNotPrepared);
        assert(workspace.Prepare(64, 64));
        auto blobs = workspace.Scan(frame, {0, 0, 64, 64}, 128);
        assert(blobs && blobs->size() == 1);
        assert((*blobs)[0].area == 64);
        assert(std::abs((*blobs)[0].center.x - 31.5F) < 0.001F);
        detection::LightDetector light;
        assert(light.Detect(*blobs, 4096).valid);
        assert(!workspace.Scan(frame, {63, 0, 2, 64}, 128));
        detection::BlobWorkspace::Config small{};
        small.max_runs = small.max_candidates = 1;
        assert(workspace.Prepare(64, 64, small));
        auto exhausted = workspace.Scan(frame, {0, 0, 64, 64}, 128);
        assert(!exhausted && exhausted.error() == detection::DetectionError::kWorkspaceExhausted);

        detection::RoiDetector::Config config{};
        config.light.search_step = 1;
        config.light.confirm_frames = config.light.far_confirm_frames = 1;
        detection::RoiDetector detector{config};
        assert(detector.Prepare(64, 64));
        auto measured = detector.Detect(frame);
        assert(measured && measured->light.valid);
        assert(measured->source.frame_id == 1);
        auto duplicate = detector.Detect(frame);
        assert(!duplicate && duplicate.error() == detection::DetectionError::kOutOfOrderFrame);
        pixels.fill(std::byte{0});
        frame.sequence = 2;
        frame.received += std::chrono::milliseconds{10};
        auto missing = detector.Detect(frame);
        assert(missing && !missing->target.valid && missing->kind == detection::TargetKind::kNone);
        detector.Reset();
        frame.sequence = 1;
        assert(detector.Detect(frame));

        std::array<detection::Blob, 2> bars{};
        for (std::size_t i = 0; i < bars.size(); ++i) {
            auto& bar = bars[i];
            bar.bounds = {static_cast<std::uint32_t>(20 + i * 20), 20, 2, 12};
            bar.center = {static_cast<float>(bar.bounds.x) + 0.5F, 25.5F};
            bar.area = 24;
            bar.fill = 1.0F;
            bar.length = 12.0F;
            bar.thickness = 2.0F;
            bar.angle = 1.57079632679F;
        }
        detection::Target anchor{};
        anchor.valid = true;
        anchor.center = {30.5F, 50.0F};
        anchor.bounds = {27, 46, 8, 8};
        anchor.radius = 4.0F;
        detection::ArmorDetector armor;
        const auto plate = armor.Detect(bars, anchor);
        assert(plate.valid);
        assert(std::abs(plate.center.x - 30.5F) < 0.001F);
        assert(std::abs(plate.center.y - 25.5F) < 0.001F);

        // A 30-degree rotated 5x40 bar has a roughly 24x37 bounding box.
        // Leave moment dimensions empty to ensure ArmorDetector recovers the
        // principal dimensions from bounds + area and still computes the center
        // from the bars' long-axis endpoints.
        std::array<detection::Blob, 2> rotated{};
        for (std::size_t i = 0; i < rotated.size(); ++i) {
            auto& bar = rotated[i];
            // The center spacing is perpendicular to the 60-degree long axis,
            // so the two bars have full projected overlap like a real plate.
            bar.bounds = {i == 0 ? 300u : 355u, i == 0 ? 100u : 68u, 24, 37};
            bar.center = {static_cast<float>(bar.bounds.x) + 11.5F,
                          static_cast<float>(bar.bounds.y) + 18.0F};
            bar.area = 200;
            // The moment angle is absent here; bounds + area must recover it.
        }
        auto rotated_config = detection::ArmorDetector::Config{};
        rotated_config.window_width_k = 16.0F;
        rotated_config.window_height_k = 16.0F;
        detection::ArmorDetector rotated_armor{rotated_config};
        detection::Target rotated_anchor{};
        rotated_anchor.valid = true;
        rotated_anchor.center = {339.0F, 190.0F};
        rotated_anchor.bounds = {335, 186, 8, 8};
        rotated_anchor.radius = 4.0F;
        const auto rotated_plate = rotated_armor.Detect(rotated, rotated_anchor);
        assert(rotated_plate.valid);
        assert(std::abs(rotated_plate.center.x - 339.0F) < 1.5F);
        assert(std::abs(rotated_plate.center.y - 102.0F) < 1.5F);
        anchor.valid = false;
        assert(!armor.Detect(bars, anchor).valid);
    }
};

} // namespace dart::tests

int main() {
    dart::tests::DetectorBehaviorTest::grayscale();
    dart::tests::DetectorBehaviorTest::grayscale_features();
    dart::tests::DetectorBehaviorTest::armor_grayscale();
    dart::tests::DetectorBehaviorTest::grayscale_ranking_and_tracking();
    dart::tests::DetectorBehaviorTest::run();
}
