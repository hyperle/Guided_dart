#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>

#include "src/detector/geometry.hpp"
#include "src/detector/interface.hpp"
#include "src/detector/numeric.hpp"

namespace dart::detection {
class Blob {
public:
    Point center{};
    Rect bounds{};
    std::uint32_t area = 0;
    float fill = 0.0F;
    float circularity = 0.0F;
    float angle = 0.0F;
    float length = 0.0F;
    float thickness = 0.0F;
    float mean_luma = 0.0F;
    float background_luma = 0.0F;
    float contrast = 0.0F;
    float background_stddev = 0.0F;
    float snr = 0.0F;
    float stability = 1.0F;
    float gray_quality = 1.0F;
    bool stability_checked = false;
    bool clipped = false;
};

class BlobWorkspace {
public:
    struct Config {
        static constexpr std::uint32_t kMaxCandidates = 256;
        std::uint32_t max_runs = 16384;
        std::uint32_t max_candidates = kMaxCandidates;
        // Foreground mean minus the surrounding ring mean, in Y8 levels.
        float min_contrast = 12.0F;
        std::uint32_t background_margin = 3;
        float min_snr = 3.0F;
        std::uint32_t stability_delta = 8;
        float min_stability = 0.5F;
    };
    BlobWorkspace() noexcept;
    ~BlobWorkspace();
    BlobWorkspace(const BlobWorkspace&) = delete;
    BlobWorkspace& operator=(const BlobWorkspace&) = delete;

    std::expected<void, DetectionError>
        Prepare(std::uint32_t max_width, std::uint32_t max_height, Config config) noexcept;
    std::expected<void, DetectionError>
        Prepare(std::uint32_t max_width, std::uint32_t max_height) noexcept;
    std::expected<void, DetectionError> Validate(const camera::FrameView& frame) const noexcept;
    std::expected<std::span<const Blob>, DetectionError> Scan(
        const camera::FrameView& frame, Rect window, std::uint8_t threshold,
        std::uint32_t min_area = 2, std::uint32_t sample_step = 1,
        bool check_stability = true) noexcept;
    std::expected<bool, DetectionError> ValidateStability(
        const camera::FrameView& frame, Blob& blob, std::uint8_t threshold, Rect window,
        std::uint32_t step = 1) noexcept;

    std::uint32_t runs() const noexcept { return run_count_; }
    std::uint32_t run_high_water() const noexcept { return run_high_water_; }
    std::uint32_t rejected_contrast() const noexcept { return rejected_contrast_; }
    std::uint32_t rejected_snr() const noexcept { return rejected_snr_; }
    std::uint32_t rejected_stability() const noexcept { return rejected_stability_; }
    std::uint32_t stability_checks() const noexcept { return stability_checks_; }

private:
    struct Run {
        std::uint32_t x0 = 0;
        std::uint32_t x1 = 0;
        std::uint32_t y = 0;
        std::uint32_t parent = 0;
        std::uint32_t rank = 0;
    };

    struct Component {
        std::uint64_t area = 0;
        std::uint64_t sx = 0;
        std::uint64_t sy = 0;
        std::uint64_t sxx = 0;
        std::uint64_t syy = 0;
        std::uint64_t sxy = 0;
        std::uint64_t luma = 0;
        std::uint32_t x0 = 0;
        std::uint32_t x1 = 0;
        std::uint32_t y0 = 0;
        std::uint32_t y1 = 0;
    };

    std::uint32_t Root(std::uint32_t id) noexcept;
    void Unite(std::uint32_t a, std::uint32_t b) noexcept;
    std::expected<void, DetectionError> Extract(
        const camera::FrameView& frame, Rect window, std::uint8_t threshold,
        std::uint32_t step) noexcept;
    std::expected<float, DetectionError> Stability(
        const camera::FrameView& frame, const Blob& blob, std::uint8_t threshold, Rect scan_window,
        std::uint32_t step) noexcept;

    std::unique_ptr<Run[]> runs_;
    std::unique_ptr<Component[]> components_;
    std::unique_ptr<Blob[]> candidates_;
    std::uint32_t max_width_ = 0;
    std::uint32_t max_height_ = 0;
    std::uint32_t max_runs_ = 0;
    std::uint32_t max_candidates_ = 0;
    float min_contrast_ = 0.0F;
    std::uint32_t background_margin_ = 0;
    float min_snr_ = 0.0F;
    std::uint32_t stability_delta_ = 0;
    float min_stability_ = 0.0F;
    std::uint32_t run_count_ = 0;
    std::uint32_t run_high_water_ = 0;
    std::uint32_t rejected_contrast_ = 0, rejected_snr_ = 0, rejected_stability_ = 0;
    std::uint32_t stability_checks_ = 0;
};

} // namespace dart::detection
