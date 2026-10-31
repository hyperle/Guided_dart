module;
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <utility>

export module dart.detection.armor;

export import dart.detection.blob_workspace;
import dart.detection.numeric;
import dart.detection.result;

export namespace dart::detection {

class ArmorDetector {
public:
    struct Config {
        std::uint32_t length_floor = 2;
        std::uint32_t area_floor = 2;
        float length_min_k = 0.2F;
        float length_max_k = 8.0F;
        float tiny_px = 8.0F;
        float aspect_min = 2.0F;
        float fill_min = 0.5F;
        float flat_deg = 12.0F;
        float parallel_deg = 15.0F;
        float thickness_ratio = 0.3F;
        float length_ratio = 0.4F;
        float overlap = 0.4F;
        float gap_max_k = 10.0F;
        float diagonal_min_sin = 0.30F;
        float border_penalty = 0.0F;
        float window_width_k = 3.0F;
        float window_height_k = 2.0F;
        float window_gap_k = 0.0F;
        float window_min_side = 24.0F;
        float near_k = 8.0F;
        float near_min_px = 32.0F;
        float span_min_k = 0.2F;
        float span_max_k = 12.0F;
        std::uint32_t plate_up_lo_pct = 0;
        std::uint32_t plate_up_hi_pct = 1200;
        std::uint32_t plate_offx_pct = 600;
        float association_k = 0.6F;
        float association_min_px = 4.0F;
        std::uint32_t track_hold = 2;
        std::uint32_t confirm_frames = 2;
        float min_brightness_ratio = 0.45F;
        float min_gap_contrast = 8.0F;
    };
    ArmorDetector() noexcept;
    explicit ArmorDetector(Config config) noexcept;
    bool ValidConfig() const noexcept;
    Target Detect(
        std::span<const Blob> blobs, const Target& light,
        const Target* previous = nullptr, const camera::FrameView* frame = nullptr,
        bool associated_only = false, std::array<Blob, 2>* selected = nullptr) const noexcept;
    Rect
        SearchWindow(const Target& light, std::uint32_t width, std::uint32_t height) const noexcept;

private:
    struct Bar {
        const Blob* blob = nullptr;
        Point center{};
        Point axis{};
        float length = 0.0F;
        float thickness = 0.0F;
    };

    // Recover the dimensions of a filled rectangular bar from its axis-aligned
    // bounds and area. A rotated bar's bounding-box aspect/fill can be poor even
    // when its principal dimensions are still a valid light bar.
    // The caller keeps the axis-aligned bbox interpretation if recovery fails.
    static bool RecoverRectShape(
        const Blob& blob, float& length, float& thickness, float& tilt) noexcept {
        if (blob.bounds.width == 0 || blob.bounds.height == 0 || blob.area == 0)
            return false;
        const double w = blob.bounds.width;
        const double h = blob.bounds.height;
        const double area = blob.area;
        const double sum = w + h;
        const double product = w * h;
        const double coefficient = 2.0 * product - 4.0 * area - sum * sum;
        const double discriminant = coefficient * coefficient - 8.0 * area * sum * sum;
        if (discriminant < 0.0)
            return false;
        const double root = std::sqrt(discriminant);
        for (const double z0 : {
                 (-coefficient + root) / (4.0 * area),
                 (-coefficient - root) / (4.0 * area)}) {
            if (z0 < 1.0 - 1.0e-6 || z0 > 2.0 + 1.0e-6)
                continue;
            const double z = std::clamp(z0, 1.0, 2.0);
            const double sum_edges = sum / std::sqrt(z);
            const double difference_squared = sum_edges * sum_edges - 4.0 * area;
            if (difference_squared < 0.0)
                continue;
            const double difference = std::sqrt(difference_squared);
            const double long_edge = 0.5 * (sum_edges + difference);
            const double short_edge = 0.5 * (sum_edges - difference);
            if (!(std::isfinite(long_edge) && std::isfinite(short_edge)
                  && long_edge >= short_edge && short_edge > 0.0))
                continue;
            length = static_cast<float>(long_edge);
            thickness = static_cast<float>(short_edge);
            tilt = static_cast<float>(0.5 * std::asin(std::clamp(z - 1.0, -1.0, 1.0)));
            return true;
        }
        return false;
    }

    static float Dot(Point a, Point b) noexcept { return a.x * b.x + a.y * b.y; }

    static float Cross(Point a, Point b) noexcept { return a.x * b.y - a.y * b.x; }

    static Point Delta(Point a, Point b) noexcept { return {a.x - b.x, a.y - b.y}; }

    static bool
        PlateAnchorOk(const Config& config, Point center, const Target& light) noexcept {
        const float scale = static_cast<float>(std::max(light.bounds.width, light.bounds.height));
        if (!(scale > 0.0F))
            return false;
        const float up = light.center.y - center.y;
        const float off_x = std::fabs(center.x - light.center.x);
        return up * 100.0F >= static_cast<float>(config.plate_up_lo_pct) * scale
            && up * 100.0F <= static_cast<float>(config.plate_up_hi_pct) * scale
            && off_x * 100.0F <= static_cast<float>(config.plate_offx_pct) * scale;
    }

    static bool MakeBar(
        const Config& config, const Blob& blob, float scale, Bar& bar) noexcept {
        const float minimum = std::max(
            static_cast<float>(config.length_floor), std::floor(config.length_min_k * scale));
        const float maximum = std::max(minimum, std::floor(config.length_max_k * scale));
        if (static_cast<float>(blob.area)
            < std::max(static_cast<float>(config.area_floor), std::floor(minimum * 0.5F)))
            return false;
        bar.blob = &blob;
        bar.center = {
            static_cast<float>(blob.bounds.x) + static_cast<float>((blob.bounds.width - 1)) * 0.5F,
            static_cast<float>(blob.bounds.y)
                + static_cast<float>((blob.bounds.height - 1)) * 0.5F};
        float angle = blob.angle;
        bar.length = blob.length;
        bar.thickness = blob.thickness;
        // Area and bounds cannot describe every foreground shape. Preserve
        // measured moment geometry and recover a rectangle only when it is absent.
        if (!(IsPositive(bar.length) && IsPositive(bar.thickness) && std::isfinite(angle))) {
            float tilt = 0.0F;
            if (RecoverRectShape(blob, bar.length, bar.thickness, tilt)) {
                const float long_axis_angle = blob.bounds.width > blob.bounds.height
                    ? tilt : static_cast<float>(kPi * 0.5) - tilt;
                angle = std::copysign(long_axis_angle, blob.angle);
            } else {
                // Keep the legacy bbox interpretation when area/bounds cannot be
                // explained by a filled rotated rectangle.
                bar.length = static_cast<float>(std::max(blob.bounds.width, blob.bounds.height));
                bar.thickness = static_cast<float>(std::min(blob.bounds.width, blob.bounds.height));
                angle = blob.bounds.height >= blob.bounds.width ? static_cast<float>(kPi * 0.5)
                                                                : 0.0F;
            }
        }
        // Remote blobs are too quantized for shape tests; LightTracker's temporal
        // confirmation is the gate. Keep only the area/pixel floor above.
        const bool far = static_cast<float>(std::max(blob.bounds.width, blob.bounds.height))
                       < static_cast<float>(config.tiny_px);
        if (!far && (bar.length < minimum || bar.length > maximum))
            return false;
        const float degrees = std::fabs(angle) * kDegrees;
        const float skew = std::min(degrees, std::fabs(90.0F - degrees));
        if (!far) {
            if (bar.length < config.aspect_min * bar.thickness)
                return false;
            // For a recovered rotated rectangle, the bbox fill is expected to
            // fall as the angle grows. Keep the fill gate only near flat bars;
            // unrecoverable blobs retain the legacy bbox fallback behavior.
            if (skew <= config.flat_deg && blob.fill < config.fill_min)
                return false;
        }
        bar.axis = {std::cos(angle), std::sin(angle)};
        return true;
    }

    static void BarEndpoints(
        const Config& config, const Bar& bar, Point& first, Point& last) noexcept {
        Point axis = bar.axis;
        float length = bar.length;
        // Fix endpoint order across the +/- pi/2 discontinuity of the principal axis.
        const bool vertical = std::fabs(axis.y) >= std::fabs(axis.x);
        if ((vertical && axis.y < 0.0F) || (!vertical && axis.x < 0.0F))
            axis = {-axis.x, -axis.y};
        const float skew = std::atan2(
                               std::min(std::fabs(axis.x), std::fabs(axis.y)),
                               std::max(std::fabs(axis.x), std::fabs(axis.y)))
                         * kDegrees;
        if (skew <= config.flat_deg) {
            axis = vertical ? Point{0.0F, 1.0F} : Point{1.0F, 0.0F};
            length = static_cast<float>(
                (vertical ? bar.blob->bounds.height : bar.blob->bounds.width) - 1);
        }
        first = {bar.center.x - axis.x * length * 0.5F, bar.center.y - axis.y * length * 0.5F};
        last = {bar.center.x + axis.x * length * 0.5F, bar.center.y + axis.y * length * 0.5F};
    }

    static bool
        ArmorCenter(const Config& config, const Bar& a, const Bar& b, Point& out) noexcept {
        Point a0, a1, b0, b1;
        BarEndpoints(config, a, a0, a1);
        BarEndpoints(config, b, b0, b1);
        // Noisy axes can straddle the vertical/horizontal convention at 45 degrees.
        // Align endpoint order before constructing the crossed diagonals.
        if (Dot(Delta(a1, a0), Delta(b1, b0)) < 0.0F)
            std::swap(b0, b1);
        const Point u = Delta(b1, a0), v = Delta(b0, a1), offset = Delta(a1, a0);
        const double determinant = Cross(u, v);
        const double lengths = std::sqrt(static_cast<double>(Dot(u, u)) * Dot(v, v));
        if (lengths <= 0.0 || std::fabs(determinant) < config.diagonal_min_sin * lengths)
            return false;
        const double t = Cross(offset, v) / determinant;
        const double s = Cross(offset, u) / determinant;
        if (t < 0.0 || t > 1.0 || s < 0.0 || s > 1.0)
            return false;
        out = {static_cast<float>(a0.x + t * u.x), static_cast<float>(a0.y + t * u.y)};
        return std::isfinite(out.x) && std::isfinite(out.y);
    }

    static Rect UniteBounds(Rect a, Rect b) noexcept {
        const auto x = std::min(a.x, b.x), y = std::min(a.y, b.y);
        return {
            x, y, std::max(a.x + a.width, b.x + b.width) - x,
            std::max(a.y + a.height, b.y + b.height) - y};
    }

    static float GapLuma(const camera::FrameView& frame, const Bar& a, const Bar& b) noexcept {
        const auto offset = Delta(b.center, a.center);
        const float separation = std::fabs(Cross(offset, a.axis));
        const float lo = std::min(0.45F, a.thickness * 0.5F / separation);
        const float hi = 1.0F - std::min(0.45F, b.thickness * 0.5F / separation);
        float sum = 0.0F;
        // Nine samples inside the gap, away from the bars and their end caps.
        for (int row = -1; row <= 1; ++row) {
            for (int col = 1; col <= 3; ++col) {
                const float t = lo + (hi - lo) * col * 0.25F;
                const float along = row * std::min(a.length, b.length) * 0.25F;
                const auto x = static_cast<std::uint32_t>(std::clamp(
                    std::round(a.center.x + offset.x * t + a.axis.x * along),
                    0.0F, static_cast<float>(frame.width - 1)));
                const auto y = static_cast<std::uint32_t>(std::clamp(
                    std::round(a.center.y + offset.y * t + a.axis.y * along),
                    0.0F, static_cast<float>(frame.height - 1)));
                sum += std::to_integer<std::uint8_t>(frame.luma[static_cast<std::size_t>(y) * frame.stride + x]);
            }
        }
        return sum / 9.0F;
    }

    Config config_;
};

ArmorDetector::ArmorDetector() noexcept : ArmorDetector(Config{}) {}

ArmorDetector::ArmorDetector(Config config) noexcept
    : config_(config) {}

bool ArmorDetector::ValidConfig() const noexcept {
    const std::array values{
        config_.length_min_k,   config_.length_max_k,    config_.tiny_px,
        config_.aspect_min,     config_.gap_max_k,       config_.diagonal_min_sin,
        config_.window_width_k, config_.window_height_k, config_.window_min_side,
        config_.near_k,         config_.near_min_px,     config_.span_min_k,
        config_.span_max_k,     config_.association_k,   config_.association_min_px};
    return std::all_of(values.begin(), values.end(), IsPositive)
        && config_.length_floor > 0 && config_.area_floor > 0
        && config_.length_min_k <= config_.length_max_k && config_.span_min_k <= config_.span_max_k
        && config_.diagonal_min_sin <= 1.0F && config_.plate_up_lo_pct <= config_.plate_up_hi_pct
        && config_.plate_offx_pct > 0 && config_.track_hold > 0 && config_.confirm_frames > 0
        && IsFraction(config_.thickness_ratio) && IsFraction(config_.length_ratio)
        && IsFraction(config_.overlap) && std::isfinite(config_.window_gap_k)
        && config_.window_gap_k >= 0.0F && std::isfinite(config_.border_penalty)
        && config_.border_penalty >= 0.0F && std::isfinite(config_.flat_deg)
        && config_.flat_deg >= 0.0F && config_.flat_deg <= 45.0F
        && IsPositive(config_.parallel_deg) && config_.parallel_deg < 90.0F
        && IsFraction(config_.min_brightness_ratio) && std::isfinite(config_.min_gap_contrast)
        && config_.min_gap_contrast >= 0.0F && config_.min_gap_contrast <= 255.0F;
}

// Both selectors read measured candidates. The current light anchors the armor
// search above it; an old target is used only for association.
Target ArmorDetector::Detect(
    std::span<const Blob> blobs, const Target& light, const Target* previous,
    const camera::FrameView* frame, bool associated_only, std::array<Blob, 2>* selected) const noexcept {
    if (!light.valid || !IsPositive(light.radius)
        || blobs.size() > BlobWorkspace::Config::kMaxCandidates)
        return {};
    if (associated_only && (!previous || !previous->valid)) return {};
    if (frame && (frame->width == 0 || frame->height == 0 || frame->stride < frame->width
        || frame->luma.data() == nullptr || static_cast<std::uint64_t>(frame->height - 1)
            * frame->stride + frame->width > frame->luma.size())) return {};
    // Candidate coordinates come from a validated workspace (at most 65535).
    const Rect window = SearchWindow(light, 65535, 65535);
    const float scale = 2.0F * light.radius;
    std::array<Bar, BlobWorkspace::Config::kMaxCandidates> bars{};
    std::size_t count = 0;
    for (const auto& blob : blobs) {
        if (blob.bounds.width == 0 || blob.bounds.height == 0
            || blob.center.x < static_cast<float>(window.x)
            || blob.center.x >= static_cast<float>(window.x + window.width)
            || blob.center.y < static_cast<float>(window.y)
            || blob.center.y >= static_cast<float>(window.y + window.height))
            continue;
        if (associated_only) {
            const float margin = std::max(config_.association_min_px,
                config_.association_k * std::max(previous->bounds.width, previous->bounds.height));
            if (blob.bounds.x + blob.bounds.width < previous->bounds.x - margin
                || blob.bounds.y + blob.bounds.height < previous->bounds.y - margin
                || blob.bounds.x > previous->bounds.x + previous->bounds.width + margin
                || blob.bounds.y > previous->bounds.y + previous->bounds.height + margin) continue;
        }
        if (MakeBar(config_, blob, scale, bars[count]))
            ++count;
    }
    Target best{}, associated{};
    std::array<Blob, 2> best_pair{}, associated_pair{};
    float best_score = std::numeric_limits<float>::infinity(), associated_score = best_score;
    const float parallel = std::cos(config_.parallel_deg / kDegrees);
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = i + 1; j < count; ++j) {
            const auto& a = bars[i];
            const auto& b = bars[j];
            if (std::fabs(Dot(a.axis, b.axis)) < parallel)
                continue;
            const float longest = std::max(a.length, b.length),
                        shortest = std::min(a.length, b.length);
            if (shortest < config_.length_ratio * longest
                || std::min(a.thickness, b.thickness)
                       < config_.thickness_ratio * std::max(a.thickness, b.thickness))
                continue;
            const Point offset = Delta(b.center, a.center);
            const float separation = std::fabs(Cross(offset, a.axis));
            const float along = std::fabs(Dot(offset, a.axis));
            if ((a.length + b.length) * 0.5F - along < config_.overlap * shortest
                || separation <= (a.thickness + b.thickness) * 0.5F
                || separation > config_.gap_max_k * longest
                || separation < config_.span_min_k * scale
                || separation > config_.span_max_k * scale)
                continue;
            Point center;
            if (!ArmorCenter(config_, a, b, center))
                continue;
            if (!PlateAnchorOk(config_, center, light))
                continue;
            float brightness_ratio = 1.0F, gap_contrast = 0.0F, gap_quality = 1.0F;
            if (frame) {
                const float bright = std::max(a.blob->contrast, b.blob->contrast);
                const float dim = std::min(a.blob->contrast, b.blob->contrast);
                brightness_ratio = bright > 0.0F ? std::clamp(dim / bright, 0.0F, 1.0F) : 0.0F;
                gap_contrast = std::min(a.blob->mean_luma, b.blob->mean_luma) - GapLuma(*frame, a, b);
                if (brightness_ratio < config_.min_brightness_ratio
                    || gap_contrast < config_.min_gap_contrast) continue;
                gap_quality = std::clamp(gap_contrast / std::max(1.0F, dim), 0.0F, 1.0F);
            }
            const float score =
                ((longest - shortest) + along) * 100.0F / longest
                + (1.0F - brightness_ratio) * 25.0F + (1.0F - gap_quality) * 25.0F
                + static_cast<float>(config_.border_penalty)
                      * (static_cast<float>(a.blob->clipped) + static_cast<float>(b.blob->clipped));
            const Rect bounds = UniteBounds(a.blob->bounds, b.blob->bounds);
            const std::uint32_t area = a.blob->area + b.blob->area;
            const bool clipped = a.blob->clipped || b.blob->clipped;
            const Target target{
                true,
                center,
                bounds,
                area,
                std::sqrt(static_cast<float>(area) / static_cast<float>(kPi)),
                std::min(a.blob->gray_quality, b.blob->gray_quality) * brightness_ratio
                    * gap_quality * (clipped ? 0.25F : 1.0F),
                clipped,
                std::min(a.blob->contrast, b.blob->contrast),
                std::min(a.blob->snr, b.blob->snr),
                std::min(a.blob->stability, b.blob->stability), brightness_ratio, gap_contrast,
                a.blob->stability_checked && b.blob->stability_checked};
            if (score < best_score || (score == best_score && area > best.area)) {
                best = target;
                best_score = score;
                if (selected) best_pair = {*a.blob, *b.blob};
            }
            if (previous && previous->valid) {
                const Point change = Delta(center, previous->center);
                const float tolerance =
                    std::max(config_.association_min_px, config_.association_k * longest);
                const float old_size =
                    static_cast<float>(std::max(previous->bounds.width, previous->bounds.height));
                const float size = static_cast<float>(std::max(bounds.width, bounds.height));
                if (Dot(change, change) <= tolerance * tolerance
                    && size >= old_size * 0.5F && size <= old_size * 2.0F
                    && score + 100.0F * Dot(change, change) / (tolerance * tolerance) < associated_score) {
                    associated = target;
                    associated_score = score + 100.0F * Dot(change, change) / (tolerance * tolerance);
                    if (selected) associated_pair = {*a.blob, *b.blob};
                }
            }
        }
    }
    if (associated.valid) { if (selected) *selected = associated_pair; return associated; }
    if (associated_only) return {};
    if (selected && best.valid) *selected = best_pair;
    return best;
}

Rect ArmorDetector::SearchWindow(
    const Target& light, std::uint32_t width, std::uint32_t height) const noexcept {
    if (!light.valid || !IsPositive(light.radius) || !std::isfinite(light.center.x)
        || !std::isfinite(light.center.y))
        return {};
    const double scale = 2.0 * light.radius;
    const double w = std::max<double>(config_.window_min_side, config_.window_width_k * scale);
    const double h = std::max<double>(config_.window_min_side, config_.window_height_k * scale);
    const double x = light.center.x - w * 0.5;
    const double y = light.center.y - light.radius - config_.window_gap_k * scale - h;
    const double x0 = std::clamp(std::floor(x), 0.0, static_cast<double>(width));
    const double y0 = std::clamp(std::floor(y), 0.0, static_cast<double>(height));
    const double x1 = std::clamp(std::ceil(x + w), x0, static_cast<double>(width));
    const double y1 = std::clamp(std::ceil(y + h), y0, static_cast<double>(height));
    return {
        static_cast<std::uint32_t>(x0), static_cast<std::uint32_t>(y0),
        static_cast<std::uint32_t>(x1 - x0), static_cast<std::uint32_t>(y1 - y0)};
}

} // namespace dart::detection
