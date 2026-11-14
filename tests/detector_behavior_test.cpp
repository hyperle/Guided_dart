#include <cassert>
#include <cmath>
#include <cstdlib>

#include "tests/detector_fixture.hpp"

namespace dart::tests {

using detection::Color;
using detection::ColorTable;
using detection::DetectionError;
using detection::Rect;
using detection::ScanCounters;
using Workspace = detection::Detect::Workspace;

class DetectorBehaviorTest final {
    static bool near(float value, float expected, float tolerance) {
        return std::fabs(value - expected) <= tolerance;
    }

    struct Scanner {
        explicit Scanner(const ColorImage& image, float pixel_aspect = 1.0F,
                         ColorTable::Config colors = {}, Workspace::Config config = {}) {
            assert(table.Prepare(colors));
            const auto info = image.info();
            assert(workspace.Prepare(info.width, info.height, pixel_aspect, config));
        }
        std::expected<std::span<const detection::Blob>, DetectionError> scan(
            const ColorImage& image, Rect window, detection::ColorMask colors, std::uint32_t step = 1,
            std::uint32_t min_area = 1) {
            counters = {};
            return workspace.Scan(image.view(), table, {window, colors, step, min_area}, counters);
        }
        ColorTable table;
        Workspace workspace;
        ScanCounters counters;
    };

public:
    static void color_conversion() {
        // OpenMV's floor convention; pure green lies outside the default green range.
        const auto green = ColorTable::ToLab(0x07E0);
        assert(green[0] == 87 && green[1] == -87 && green[2] == 83);
        const auto white = ColorTable::ToLab(0xFFFF);
        assert(white[0] == 100 && std::abs(white[1]) <= 1 && std::abs(white[2]) <= 1);
        const auto black = ColorTable::ToLab(0x0000);
        assert(black[0] == 0 && black[1] == 0 && black[2] == 0);

        ColorTable table;
        assert(table.Prepare({}));
        assert(table.classify(kGreen) == detection::mask_of(Color::kGreen));
        assert(table.classify(kRed) == detection::mask_of(Color::kRed));
        assert(table.classify(kBlue) == detection::mask_of(Color::kBlue));
        assert(table.classify(kDark) == 0 && table.classify(kWhite) == 0 && table.classify(0x07E0) == 0);
        const auto* lab = table.lab(kGreen);
        assert(lab[0] == 78 && lab[1] == -68 && lab[2] == 44);

        ColorTable::Config invalid{};
        invalid.ranges[0].l_min = 97; // Above l_max.
        assert(!ColorTable::ValidConfig(invalid));
        invalid = {};
        invalid.min_chroma = -1.0F;
        assert(!ColorTable::ValidConfig(invalid));
    }

    static void region_measurement() {
        ColorImage image{160, 120, 7}; // Padded rows.
        image.disk(40, 40, 5, kGreen);
        image.rect(100, 30, 2, 12, kBlue);
        Scanner scanner{image};
        for (const std::uint32_t step : {1U, 2U}) {
            // Seeds are sampled, but each region is filled pixel-exactly.
            const auto blobs = scanner.scan(image, Rect::full(160, 120), detection::kAllColors, step);
            assert(blobs && blobs->size() == 2);
            const auto& lamp = (*blobs)[0];
            assert(lamp.color == Color::kGreen && lamp.area == 81);
            assert(near(lamp.center.x, 40, 1e-4F) && near(lamp.center.y, 40, 1e-4F));
            // Disk fill 81/121 against pi/4 limits circularity to 0.85.
            assert(lamp.circularity > 0.84F && near(lamp.aspect, 1, 1e-6F) && !lamp.clipped);
            assert(near(lamp.lab.l, 78, 1e-4F) && near(lamp.lab.a, -68, 1e-4F) && near(lamp.lab.b, 44, 1e-4F));
            assert(lamp.color_quality > 0.5F && lamp.color_quality <= 1.0F);
            const auto& bar = (*blobs)[1];
            assert(bar.color == Color::kBlue && bar.area == 24);
            assert(near(bar.length, 12, 1e-4F) && near(bar.thickness, 2, 1e-4F));
            assert(near(std::fabs(bar.axis.y), 1, 1e-4F) && near(bar.oriented_fill, 1, 1e-4F));
            assert(near(bar.box_center.x, 100.5F, 1e-4F) && near(bar.box_center.y, 35.5F, 1e-4F));
            assert(scanner.counters.raw_blobs == 2);
        }
        // Color selection and area floor.
        auto green = scanner.scan(image, Rect::full(160, 120), detection::mask_of(Color::kGreen));
        assert(green && green->size() == 1);
        assert(scanner.scan(image, Rect::full(160, 120), detection::kAllColors, 1, 82)->size() == 0);
        // A window border cuts the lamp: it is measured inside the window and clipped.
        const auto cut = scanner.scan(image, {40, 0, 120, 120}, detection::mask_of(Color::kGreen));
        assert(cut && cut->size() == 1 && (*cut)[0].clipped && (*cut)[0].bounds.x == 40);
        // Windows outside the frame are rejected.
        assert(scanner.scan(image, {150, 0, 20, 10}, 1).error() == DetectionError::kInvalidWindow);
        assert(scanner.scan(image, {0, 0, 0, 10}, 1).error() == DetectionError::kInvalidWindow);
    }

    static void rotated_bar_and_connectivity() {
        ColorImage image{160, 120};
        image.bar(80, 60, 40, 5, 60, kBlue);
        Scanner scanner{image};
        const auto blobs = scanner.scan(image, Rect::full(160, 120), detection::mask_of(Color::kBlue));
        assert(blobs && blobs->size() == 1);
        const auto& bar = (*blobs)[0];
        // The minimum-area rectangle recovers the rotated dimensions.
        assert(near(bar.length, 40, 1.5F) && near(bar.thickness, 5, 1.5F));
        const float degrees = std::atan2(std::fabs(bar.axis.y), std::fabs(bar.axis.x)) * 180.0F / 3.14159265F;
        assert(near(degrees, 60, 3) && bar.oriented_fill > 0.8F);

        // Diagonal neighbours are one region (8-connectivity).
        ColorImage diagonal{32, 32};
        for (std::uint32_t i = 0; i < 6; ++i)
            diagonal.set(10 + i, 10 + i, kGreen);
        Scanner diagonal_scanner{diagonal};
        const auto line = diagonal_scanner.scan(diagonal, Rect::full(32, 32), 1);
        assert(line && line->size() == 1 && (*line)[0].area == 6);
        assert(near((*line)[0].length, 1 + 5 * std::sqrt(2.0F), 1e-3F) && near((*line)[0].thickness, 1, 1e-3F));
    }

    static void color_quality_and_capacity() {
        ColorImage image{64, 48};
        image.disk(20, 20, 4, kGreen);
        ColorTable::Config strict{};
        strict.min_chroma = 90.0F; // Above the lamp's chroma of 81.
        Scanner rejecting{image, 1.0F, strict};
        const auto rejected = rejecting.scan(image, Rect::full(64, 48), 1);
        assert(rejected && rejected->empty());
        assert(rejecting.counters.raw_blobs == 1 && rejecting.counters.rejected_color == 1);

        image.disk(40, 20, 4, kGreen);
        image.disk(20, 36, 4, kGreen);
        Scanner small{image, 1.0F, {}, {2, 16384}};
        assert(small.scan(image, Rect::full(64, 48), 1).error() == DetectionError::kWorkspaceExhausted);
        // A failed scan leaves no stale state behind.
        assert(small.scan(image, {0, 0, 64, 28}, 1)->size() == 2);
        // A fork queues two spans at once; one pending span is not enough.
        ColorImage fork{64, 48};
        fork.rect(10, 10, 20, 3, kGreen);
        fork.rect(10, 13, 3, 10, kGreen);
        fork.rect(27, 13, 3, 10, kGreen);
        Scanner shallow{fork, 1.0F, {}, {16, 1}};
        assert(shallow.scan(fork, Rect::full(64, 48), 1).error() == DetectionError::kWorkspaceExhausted);
        Scanner deep{fork, 1.0F, {}, {16, 2}};
        assert(deep.scan(fork, Rect::full(64, 48), 1)->size() == 1);
    }

    static void pixel_aspect() {
        // A round lamp after the 1080x1280 -> 480x640 ISP scale is narrower
        // than tall by 1.125. Correcting the scale restores its roundness.
        ColorImage image{96, 96};
        image.ellipse(48, 48, 12.0F / 1.125F, 12.0F, kGreen);
        Scanner square{image, 1.0F};
        Scanner corrected{image, 1.125F};
        const auto raw = square.scan(image, Rect::full(96, 96), 1);
        const auto fixed = corrected.scan(image, Rect::full(96, 96), 1);
        assert(raw && fixed && raw->size() == 1 && fixed->size() == 1);
        assert(near((*fixed)[0].aspect, 1.0F, 0.08F) && (*raw)[0].aspect < 0.9F);
        assert((*fixed)[0].circularity > (*raw)[0].circularity + 0.05F);
    }

    static detection::Detect::Config confirmed_once() {
        detection::Detect::Config config{};
        config.light.confirm_frames = config.light.far_confirm_frames = 1;
        config.armor.confirm_frames = 1;
        return config;
    }

    // Lamp at (80, 100); two vertical bars 20 px apart, centered 30 px above it.
    static void armor_scene(ColorImage& image, std::uint16_t left = kBlue, std::uint16_t right = kBlue,
                            std::uint32_t shift = 0) {
        image.clear();
        image.disk(80, 100, 4, kGreen);
        image.rect(69 + shift, 62, 3, 16, left);
        image.rect(89 + shift, 62, 3, 16, right);
    }

    static void armor_pairing() {
        ColorImage image{160, 120};
        armor_scene(image);
        DetectionCalculator calculator{confirmed_once()};
        assert(calculator.Prepare(image.info()));
        auto result = calculator.Detect(image.view(1, 10'000));
        assert(result && result->kind == detection::TargetKind::kArmor);
        assert(result->reason == detection::DetectReason::kArmorLocked);
        assert(near(result->target.center.x, 80, 1e-3F) && near(result->target.center.y, 69.5F, 1e-3F));
        assert(result->armor.color == Color::kBlue && result->armor.gap_color_fraction == 0.0F);
        assert(result->armor.bar_lab_distance == 0.0F && result->armor.has_bars);
        assert(result->light.valid && result->light.color == Color::kGreen);
        assert(result->scan.full_frame && result->scan.phase == detection::DetectionPhase::kDetect);

        const auto reject = [&](std::uint32_t ScanCounters::* counter, DetectionCalculator& detector) {
            detector.Reset();
            const auto rejected = detector.Detect(image.view(1, 10'000));
            assert(rejected && rejected->kind == detection::TargetKind::kLight);
            assert(rejected->reason == detection::DetectReason::kArmorRejected);
            assert(rejected->scan.counters.*counter > 0);
        };
        armor_scene(image, kBlue, kRed); // Both bars must share one profile.
        reject(&ScanCounters::rejected_pair_color, calculator);
        armor_scene(image);
        image.rect(75, 62, 11, 16, kBlue); // Plate interior of the bar color.
        reject(&ScanCounters::rejected_gap, calculator);
        auto narrow = confirmed_once();
        narrow.armor.plate_offx_pct = 50; // 4.5 px for this 9 px lamp.
        DetectionCalculator anchored{narrow};
        assert(anchored.Prepare(image.info()));
        armor_scene(image, kBlue, kBlue, 10);
        reject(&ScanCounters::rejected_anchor, anchored);
        armor_scene(image, kBlue, kBlue, 4);
        anchored.Reset();
        assert(anchored.Detect(image.view(1, 10'000))->kind == detection::TargetKind::kArmor);

        // Bars farther than the anchored search window are not scanned.
        calculator.Reset();
        image.clear();
        image.disk(80, 110, 3, kGreen);
        image.rect(20, 10, 3, 16, kBlue);
        image.rect(40, 10, 3, 16, kBlue);
        result = calculator.Detect(image.view(1, 10'000));
        assert(result && result->kind == detection::TargetKind::kLight);
        assert(result->reason == detection::DetectReason::kNoArmorColor);
    }

    static void light_rules_and_reasons() {
        ColorImage image{160, 120};
        DetectionCalculator calculator{confirmed_once()};
        assert(calculator.Prepare(image.info()));
        image.disk(80, 60, 6, kWhite); // Bright, but not a lamp color.
        auto result = calculator.Detect(image.view(1, 10'000));
        assert(result && !result->target.valid && result->reason == detection::DetectReason::kNoLightColor);
        assert(result->coordinate().x == -1 && result->coordinate().y == -1);

        calculator.Reset();
        image.clear();
        image.rect(40, 40, 40, 6, kGreen); // Green, but a bar, not a lamp.
        result = calculator.Detect(image.view(1, 10'000));
        assert(result && !result->target.valid && result->reason == detection::DetectReason::kLightRejected);
        assert(result->scan.counters.rejected_light_shape == 1);

        // Square lamps pass (disk-fill similarity 0.785); the larger lamp wins acquisition.
        calculator.Reset();
        image.clear();
        image.rect(20, 20, 8, 8, kGreen);
        image.disk(100, 60, 6, kGreen);
        result = calculator.Detect(image.view(1, 10'000));
        assert(result && result->kind == detection::TargetKind::kLight);
        assert(near(result->target.center.x, 100, 1e-3F) && near(result->target.center.y, 60, 1e-3F));
        assert(result->target.radius > 5.5F && result->target.quality > 0.4F);
    }

    static void frame_contract() {
        ColorImage image{64, 48};
        DetectionCalculator calculator;
        auto y8 = image.info();
        y8.format = camera::PixelFormat::kY8;
        assert(calculator.Prepare(y8).error() == DetectionError::kUnsupportedFormat);
        assert(calculator.Detect(image.view()).error() == DetectionError::kNotPrepared);
        auto bad_aspect = image.info(0.0F);
        assert(calculator.Prepare(bad_aspect).error() == DetectionError::kInvalidConfig);
        assert(calculator.Prepare(image.info()));
        auto frame = image.view();
        frame.format = camera::PixelFormat::kY8;
        assert(calculator.Detect(frame).error() == DetectionError::kUnsupportedFormat);
        frame = image.view();
        frame.stride = 64; // Shorter than one RGB565 row.
        assert(calculator.Detect(frame).error() == DetectionError::kInvalidFrame);
        ColorImage larger{65, 48};
        assert(calculator.Detect(larger.view()).error() == DetectionError::kInvalidFrame);

        detection::Detect::Config config{};
        config.light.colors = 0;
        DetectionCalculator invalid{config};
        assert(invalid.Prepare(image.info()).error() == DetectionError::kInvalidConfig);
    }

    static void parameters() {
        detection::Detect::Config config{};
        using detection::Detect;
        assert(Detect::set_parameter(config, "light.min_area", 4) && config.light.min_area == 4);
        assert(Detect::set_parameter(config, "color.min_chroma", 12.5) && config.color.min_chroma == 12.5F);
        assert(Detect::set_parameter(config, "color.blue.b_min", -80) && config.color.ranges[2].b_min == -80);
        assert(Detect::set_parameter(config, "color.green.l_max", 100) && config.color.ranges[0].l_max == 100);
        assert(Detect::set_parameter(config, "armor.colors", 4) && config.armor.colors == 4);
        assert(Detect::set_parameter(config, "track.min_color_ratio", 0.4) && config.track.min_color_ratio == 0.4F);
        assert(Detect::set_parameter(config, "roi.max_gap", 300) && config.roi.max_gap.count() == 300);
        assert(!Detect::set_parameter(config, "light.min_area", 2.5));
        assert(!Detect::set_parameter(config, "light.colors", 0));
        assert(!Detect::set_parameter(config, "color.green.l_min", 128));
        assert(!Detect::set_parameter(config, "color.purple.l_min", 1));
        assert(!Detect::set_parameter(config, "color.green", 1));
        assert(!Detect::set_parameter(config, "roi.max_gap", 0));
        assert(!Detect::set_parameter(config, "threshold", 50)); // Grayscale parameter removed.
        assert(!Detect::set_parameter(config, "light.far_jump_k", std::nan("")));
    }
};

} // namespace dart::tests

int main() {
    using dart::tests::DetectorBehaviorTest;
    DetectorBehaviorTest::color_conversion();
    DetectorBehaviorTest::region_measurement();
    DetectorBehaviorTest::rotated_bar_and_connectivity();
    DetectorBehaviorTest::color_quality_and_capacity();
    DetectorBehaviorTest::pixel_aspect();
    DetectorBehaviorTest::armor_pairing();
    DetectorBehaviorTest::light_rules_and_reasons();
    DetectorBehaviorTest::frame_contract();
    DetectorBehaviorTest::parameters();
}
