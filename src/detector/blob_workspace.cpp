module;
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <limits>
#include <memory>
#include <new>
#include <span>
#include <utility>

export module dart.detection.blob_workspace;

import dart.detection.geometry;
export import dart.detection.interface;
import dart.detection.numeric;

export namespace dart::detection {

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

    std::expected<void, DetectionError> Prepare(
        std::uint32_t max_width, std::uint32_t max_height, Config config) noexcept;
    std::expected<void, DetectionError> Prepare(
        std::uint32_t max_width, std::uint32_t max_height) noexcept {
        return Prepare(max_width, max_height, Config{});
    }
    std::expected<void, DetectionError> Validate(const camera::FrameView& frame) const noexcept;
    std::expected<std::span<const Blob>, DetectionError> Scan(
        const camera::FrameView& frame, Rect window, std::uint8_t threshold,
        std::uint32_t min_area = 2, std::uint32_t sample_step = 1,
        bool check_stability = true) noexcept;
    std::expected<bool, DetectionError> ValidateStability(
        const camera::FrameView& frame, Blob& blob, std::uint8_t threshold,
        Rect window, std::uint32_t step = 1) noexcept;

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
        const camera::FrameView& frame, const Blob& blob, std::uint8_t threshold,
        Rect scan_window, std::uint32_t step) noexcept;

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

BlobWorkspace::BlobWorkspace() noexcept = default;

BlobWorkspace::~BlobWorkspace() = default;

std::expected<void, DetectionError> BlobWorkspace::Prepare(
    std::uint32_t max_width, std::uint32_t max_height, Config config) noexcept {
    const auto max_runs = config.max_runs;
    const auto max_candidates = config.max_candidates;
    if (max_width == 0 || max_height == 0 || max_width > 65535 || max_height > 65535
        || max_runs == 0 || max_runs > 65535 || max_candidates == 0
        || max_candidates > Config::kMaxCandidates || max_candidates > max_runs
        || !std::isfinite(config.min_contrast) || config.min_contrast < 0.0F
        || config.min_contrast > 255.0F || config.background_margin == 0
        || config.background_margin > 65535 || !std::isfinite(config.min_snr)
        || config.min_snr < 0.0F || config.stability_delta > 255
        || !std::isfinite(config.min_stability) || config.min_stability < 0.0F
        || config.min_stability > 1.0F)
        return std::unexpected(DetectionError::kInvalidConfig);
    auto runs = std::unique_ptr<Run[]>(new (std::nothrow) Run[max_runs]);
    auto components = std::unique_ptr<Component[]>(new (std::nothrow) Component[max_runs]);
    auto candidates = std::unique_ptr<Blob[]>(new (std::nothrow) Blob[max_candidates]);
    if (!runs || !components || !candidates)
        return std::unexpected(DetectionError::kNoMemory);
    runs_ = std::move(runs);
    components_ = std::move(components);
    candidates_ = std::move(candidates);
    max_width_ = max_width;
    max_height_ = max_height;
    max_runs_ = max_runs;
    max_candidates_ = max_candidates;
    min_contrast_ = config.min_contrast;
    background_margin_ = config.background_margin;
    min_snr_ = config.min_snr;
    stability_delta_ = config.stability_delta;
    min_stability_ = config.min_stability;
    run_count_ = run_high_water_ = 0;
    return {};
}

std::expected<void, DetectionError>
    BlobWorkspace::Validate(const camera::FrameView& frame) const noexcept {
    if (!runs_)
        return std::unexpected(DetectionError::kNotPrepared);
    if (frame.width == 0 || frame.height == 0 || frame.width > max_width_
        || frame.height > max_height_ || frame.stride < frame.width || frame.luma.data() == nullptr)
        return std::unexpected(DetectionError::kInvalidFrame);
    const std::uint64_t required =
        static_cast<std::uint64_t>(frame.height - 1) * frame.stride + frame.width;
    if (required > frame.luma.size())
        return std::unexpected(DetectionError::kInvalidFrame);
    return {};
}

std::expected<std::span<const Blob>, DetectionError> BlobWorkspace::Scan(
    const camera::FrameView& frame, Rect window, std::uint8_t threshold, std::uint32_t min_area,
    std::uint32_t sample_step, bool check_stability) noexcept {
    auto valid = Validate(frame);
    if (!valid)
        return std::unexpected(valid.error());
    if (window.width == 0 || window.height == 0 || window.x >= frame.width
        || window.y >= frame.height || window.width > frame.width - window.x
        || window.height > frame.height - window.y)
        return std::unexpected(DetectionError::kInvalidWindow);
    sample_step = std::clamp(sample_step, 1U, 8U);
    rejected_contrast_ = rejected_snr_ = rejected_stability_ = 0;
    stability_checks_ = 0;
    auto extracted = Extract(frame, window, threshold, sample_step);
    if (!extracted)
        return std::unexpected(extracted.error());
    std::uint32_t count = 0;
    for (std::uint32_t i = 0; i < run_count_; ++i) {
        const auto& c = components_[i];
        if (c.area == 0)
            continue;
        // Convert sampled points back to physical pixels for far-area and radius gates.
        const std::uint64_t physical_area = c.area * sample_step * sample_step;
        if (physical_area < min_area)
            continue;
        const double area = static_cast<double>(c.area), mx = static_cast<double>(c.sx) / area,
                     my = static_cast<double>(c.sy) / area;
        const double xx = std::max(0.0, static_cast<double>(c.sxx) / area - mx * mx);
        const double yy = std::max(0.0, static_cast<double>(c.syy) / area - my * my);
        const double xy = static_cast<double>(c.sxy) / area - mx * my;
        const double spread = std::hypot(xx - yy, 2.0 * xy);
        const double major = std::max(0.0, (xx + yy + spread) * 0.5);
        const double minor = std::max(0.0, (xx + yy - spread) * 0.5);
        Blob blob{};
        const auto bbox_x = c.x0 * sample_step;
        const auto bbox_y = c.y0 * sample_step;
        const auto physical_width = std::min(
            window.width - std::min(window.width, bbox_x), (c.x1 - c.x0 + 1) * sample_step);
        const auto physical_height = std::min(
            window.height - std::min(window.height, bbox_y), (c.y1 - c.y0 + 1) * sample_step);
        blob.bounds = {window.x + bbox_x, window.y + bbox_y, physical_width, physical_height};
        blob.mean_luma = static_cast<float>(static_cast<double>(c.luma) / area);
        // Read the ring from the complete frame, independently of the ROI.
        const auto x0 = blob.bounds.x - std::min(blob.bounds.x, background_margin_);
        const auto y0 = blob.bounds.y - std::min(blob.bounds.y, background_margin_);
        const auto bx1 = blob.bounds.x + blob.bounds.width;
        const auto by1 = blob.bounds.y + blob.bounds.height;
        const auto x1 = bx1 + std::min(frame.width - bx1, background_margin_);
        const auto y1 = by1 + std::min(frame.height - by1, background_margin_);
        std::uint64_t sum = 0, squares = 0, n = 0;
        std::uint64_t dark_sum = 0, dark_squares = 0, dark_n = 0;
        for (auto y = y0; y < y1; ++y) {
            const auto* row = frame.luma.data() + static_cast<std::size_t>(y) * frame.stride;
            const auto add = [&](std::uint32_t begin, std::uint32_t end) {
                for (auto x = begin; x < end; ++x) {
                    const auto v = std::to_integer<std::uint8_t>(row[x]);
                    sum += v;
                    squares += v * v;
                    ++n;
                    if (v < blob.mean_luma) {
                        dark_sum += v;
                        dark_squares += v * v;
                        ++dark_n;
                    }
                }
            };
            if (y < blob.bounds.y || y >= by1) add(x0, x1);
            else { add(x0, blob.bounds.x); add(bx1, x1); }
        }
        // Mask sparse neighbors at least as bright as this candidate. If they
        // occupy over 1/4 of the ring, keep all samples: a bright/noisy surround
        // must not be turned into an artificial dark background.
        if (dark_n >= 8 && dark_n * 4 >= n * 3) {
            sum = dark_sum;
            squares = dark_squares;
            n = dark_n;
        }
        const double mean = n > 0 ? static_cast<double>(sum) / n : blob.mean_luma;
        blob.background_luma = static_cast<float>(mean);
        blob.background_stddev = n > 0
            ? static_cast<float>(std::sqrt(std::max(0.0, static_cast<double>(squares) / n - mean * mean)))
            : 0.0F;
        blob.contrast = blob.mean_luma - blob.background_luma;
        blob.snr = blob.contrast / (blob.background_stddev + 4.0F);
        if (blob.contrast < min_contrast_) { ++rejected_contrast_; continue; }
        if (blob.snr < min_snr_) { ++rejected_snr_; continue; }
        if (count == max_candidates_)
            return std::unexpected(DetectionError::kWorkspaceExhausted);
        blob.center = {
            static_cast<float>(window.x + mx * sample_step),
            static_cast<float>(window.y + my * sample_step)};
        blob.area = static_cast<std::uint32_t>(
            std::min<std::uint64_t>(physical_area, std::numeric_limits<std::uint32_t>::max()));
        blob.fill = static_cast<float>(
            static_cast<double>(physical_area)
            / (static_cast<double>(blob.bounds.width) * blob.bounds.height));
        const double elongation = major > 1.0e-9 ? std::sqrt(minor / major) : 0.6;
        blob.circularity =
            static_cast<float>(elongation * std::min(1.0, blob.fill / (kPi * 0.25)));
        blob.angle = static_cast<float>(0.5 * std::atan2(2.0 * xy, xx - yy));
        blob.length = static_cast<float>(std::sqrt(12.0 * major) * sample_step);
        blob.thickness = static_cast<float>(std::sqrt(12.0 * minor) * sample_step);
        blob.clipped = c.x0 == 0 || c.y0 == 0
                    || c.x1 + 1 >= (window.width + sample_step - 1) / sample_step
                    || c.y1 + 1 >= (window.height + sample_step - 1) / sample_step;
        candidates_[count++] = blob;
    }
    // Extraction scratch can now be reused without invalidating candidate data.
    const auto base_runs = run_count_;
    std::uint32_t kept = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
        auto blob = candidates_[i];
        blob.stability = 0.0F;
        blob.gray_quality = std::clamp(blob.snr / std::max(6.0F, 2.0F * min_snr_), 0.0F, 1.0F);
        if (check_stability) {
            auto accepted = ValidateStability(frame, blob, threshold, window, sample_step);
            if (!accepted) return std::unexpected(accepted.error());
            if (!*accepted) continue;
        }
        candidates_[kept++] = blob;
    }
    run_count_ = base_runs;
    return std::span<const Blob>(candidates_.get(), kept);
}

std::expected<bool, DetectionError> BlobWorkspace::ValidateStability(
    const camera::FrameView& frame, Blob& blob, std::uint8_t threshold,
    Rect window, std::uint32_t step) noexcept {
    if (stability_delta_ == 0 || min_stability_ == 0.0F) return true;
    ++stability_checks_;
    auto stability = Stability(frame, blob, threshold, window, step);
    if (!stability) return std::unexpected(stability.error());
    blob.stability = *stability;
    blob.stability_checked = true;
    blob.gray_quality *= blob.stability;
    if (blob.stability < min_stability_) { ++rejected_stability_; return false; }
    return true;
}

std::expected<void, DetectionError> BlobWorkspace::Extract(
    const camera::FrameView& frame, Rect window, std::uint8_t threshold,
    std::uint32_t sample_step) noexcept {
    run_count_ = 0;
    std::uint32_t previous_begin = 0, previous_end = 0;
    std::uint32_t sampled_y = 0;
    for (std::uint32_t y = 0; y < window.height; y += sample_step, ++sampled_y) {
        const auto* row =
            frame.luma.data() + static_cast<std::size_t>(window.y + y) * frame.stride + window.x;
        const auto current_begin = run_count_;
        auto previous = previous_begin;
        std::uint32_t x = 0;
        while (x < window.width) {
            if (std::to_integer<std::uint8_t>(row[x]) < threshold) {
                x += sample_step;
                continue;
            }
            const auto start = x;
            x += sample_step;
            while (x < window.width && std::to_integer<std::uint8_t>(row[x]) >= threshold)
                x += sample_step;
            if (run_count_ == max_runs_)
                return std::unexpected(DetectionError::kWorkspaceExhausted);
            const auto id = run_count_++;
            runs_[id] = {start / sample_step, (x - sample_step) / sample_step, sampled_y, id, 0};
            run_high_water_ = std::max(run_high_water_, run_count_);
            const auto sampled_start = start / sample_step;
            const auto sampled_end = (x - sample_step) / sample_step;
            while (previous < previous_end && runs_[previous].x1 + 1 < sampled_start)
                ++previous;
            for (auto k = previous; k < previous_end && runs_[k].x0 <= sampled_end + 1; ++k)
                Unite(id, k);
        }
        previous_begin = current_begin;
        previous_end = run_count_;
    }
    std::fill_n(components_.get(), run_count_, Component{});
    for (std::uint32_t i = 0; i < run_count_; ++i) {
        const auto& run = runs_[i];
        auto& component = components_[Root(i)];
        const std::uint64_t length = run.x1 - run.x0 + 1;
        const std::uint64_t sx = (static_cast<std::uint64_t>(run.x0) + run.x1) * length / 2;
        if (component.area == 0) {
            component.x0 = run.x0;
            component.x1 = run.x1;
            component.y0 = component.y1 = run.y;
        } else {
            component.x0 = std::min(component.x0, run.x0);
            component.x1 = std::max(component.x1, run.x1);
            component.y1 = std::max(component.y1, run.y);
        }
        component.area += length;
        component.sx += sx;
        component.sy += length * run.y;
        component.sxx +=
            SumSquares(run.x1) - (run.x0 > 0 ? SumSquares(run.x0 - 1) : 0);
        component.syy += length * run.y * run.y;
        component.sxy += sx * run.y;
        const auto* row = frame.luma.data()
                        + static_cast<std::size_t>(window.y + run.y * sample_step) * frame.stride
                        + window.x;
        for (auto x = run.x0; x <= run.x1; ++x)
            component.luma += std::to_integer<std::uint8_t>(row[x * sample_step]);
    }
    return {};
}

std::expected<float, DetectionError> BlobWorkspace::Stability(
    const camera::FrameView& frame, const Blob& blob, std::uint8_t threshold,
    Rect scan_window, std::uint32_t step) noexcept {
    const auto& b = blob.bounds;
    const auto margin = std::max(4U, std::max(b.width, b.height) / 2);
    // Preserve the original sampling lattice, including an off-origin ROI.
    const auto phase_x = scan_window.x % step, phase_y = scan_window.y % step;
    const auto align = [step](std::uint32_t v, std::uint32_t phase) {
        return v > phase ? phase + (v - phase) / step * step : phase;
    };
    const auto x0 = align(b.x - std::min(b.x, margin), phase_x);
    const auto y0 = align(b.y - std::min(b.y, margin), phase_y);
    const auto x1 = b.x + b.width + std::min(frame.width - b.x - b.width, margin);
    const auto y1 = b.y + b.height + std::min(frame.height - b.y - b.height, margin);
    const Rect local{x0, y0, x1 - x0, y1 - y0};
    const float base_area = static_cast<float>(blob.area) / (step * step);
    const auto lower = static_cast<std::uint8_t>(std::min<unsigned>(threshold,
        std::max<int>(std::max(0, static_cast<int>(threshold) - static_cast<int>(stability_delta_)),
                      std::min(255, static_cast<int>(blob.background_luma + blob.background_stddev) + 1))));
    const auto upper = static_cast<std::uint8_t>(std::min(255U, threshold + stability_delta_));
    float stability = 1.0F;
    for (const auto level : {lower, upper}) {
        if (level == threshold) continue;
        auto extracted = Extract(frame, local, level, step);
        if (!extracted) return std::unexpected(extracted.error());
        // luma is scratch here: count overlap with the original threshold mask.
        for (std::uint32_t i = 0; i < run_count_; ++i) components_[i].luma = 0;
        for (std::uint32_t i = 0; i < run_count_; ++i) {
            const auto& r = runs_[i];
            const auto y = y0 + r.y * step;
            if (y < b.y || y >= b.y + b.height) continue;
            auto& overlap = components_[Root(i)].luma;
            const auto* row = frame.luma.data() + static_cast<std::size_t>(y) * frame.stride;
            for (auto x = r.x0; x <= r.x1; ++x) {
                const auto px = x0 + x * step;
                if (px >= b.x && px < b.x + b.width
                    && std::to_integer<std::uint8_t>(row[px]) >= threshold) ++overlap;
            }
        }
        const Component* match = nullptr;
        std::uint64_t total_overlap = 0;
        for (std::uint32_t i = 0; i < run_count_; ++i) {
            const auto& c = components_[i];
            total_overlap += c.luma;
            if (c.luma > 0 && (!match || c.luma > match->luma)) match = &c;
        }
        if (!match) return 0.0F;
        const auto& c = *match;
        // Expansion out of the local neighborhood is evidence of a merge.
        if (level < threshold
            && ((c.x0 == 0 && x0 >= step) || (c.y0 == 0 && y0 >= step)
                || (c.x1 + 1 >= (local.width + step - 1) / step && x1 < frame.width)
                || (c.y1 + 1 >= (local.height + step - 1) / step && y1 < frame.height)))
            return 0.0F;
        const float size_ratio = static_cast<float>(c.area) / base_area;
        const float area_score = std::sqrt(std::min(size_ratio, 1.0F / size_ratio));
        const float cx = x0 + static_cast<float>(c.sx) / c.area * step;
        const float cy = y0 + static_cast<float>(c.sy) / c.area * step;
        const float shift = std::hypot(cx - blob.center.x, cy - blob.center.y);
        const float center_score = 1.0F / (1.0F + shift / std::max(2.0F, 0.25F * std::max(b.width, b.height)));
        const float connectivity = level > threshold
            ? std::max(0.0F, 2.0F * static_cast<float>(c.luma) / total_overlap - 1.0F) : 1.0F;
        stability = std::min({stability, area_score, center_score, connectivity});
    }
    return stability;
}

std::uint32_t BlobWorkspace::Root(std::uint32_t id) noexcept {
    while (runs_[id].parent != id) {
        runs_[id].parent = runs_[runs_[id].parent].parent;
        id = runs_[id].parent;
    }
    return id;
}

void BlobWorkspace::Unite(std::uint32_t a, std::uint32_t b) noexcept {
    a = Root(a);
    b = Root(b);
    if (a == b)
        return;
    if (runs_[a].rank < runs_[b].rank)
        std::swap(a, b);
    runs_[b].parent = a;
    if (runs_[a].rank == runs_[b].rank)
        ++runs_[a].rank;
}

} // namespace dart::detection
