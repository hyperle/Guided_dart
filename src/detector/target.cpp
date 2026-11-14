#include "src/detector/target.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace dart::detection {

Rect Rect::Clip(float x, float y, float w, float h, std::uint32_t width, std::uint32_t height) noexcept {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) || !std::isfinite(h))
        return {};
    const auto fw = static_cast<double>(width), fh = static_cast<double>(height);
    const double x0 = std::clamp(std::floor(static_cast<double>(x)), 0.0, fw);
    const double y0 = std::clamp(std::floor(static_cast<double>(y)), 0.0, fh);
    const double x1 = std::clamp(std::ceil(static_cast<double>(x) + w), x0, fw);
    const double y1 = std::clamp(std::ceil(static_cast<double>(y) + h), y0, fh);
    return {
        static_cast<std::uint32_t>(x0), static_cast<std::uint32_t>(y0),
        static_cast<std::uint32_t>(x1 - x0), static_cast<std::uint32_t>(y1 - y0)};
}

bool Association::ValidConfig(const Config& c) noexcept {
    return Numeric::IsPositive(c.association_k) && Numeric::IsPositive(c.association_min_px)
        && Numeric::IsPositive(c.min_size_ratio) && c.min_size_ratio <= 1.0F
        && Numeric::IsFraction(c.min_color_ratio) && Numeric::IsPositive(c.max_lab_distance);
}

float Association::Cost(
    const Target& value, const Target& predicted, const Target& reference,
    const Config& c) noexcept {
    if (!value.valid || !predicted.valid || !reference.valid || value.color != reference.color
        || reference.bounds.empty())
        return kInvalidCost;
    const float wr = static_cast<float>(value.bounds.width) / static_cast<float>(reference.bounds.width);
    const float hr = static_cast<float>(value.bounds.height) / static_cast<float>(reference.bounds.height);
    const float low = std::min(value.color_quality, reference.color_quality);
    const float high = std::max(value.color_quality, reference.color_quality);
    const float lab_distance = Lab::distance(value.lab, reference.lab);
    const float minimum = c.min_size_ratio;
    if (wr < minimum || wr > 1.0F / minimum || hr < minimum || hr > 1.0F / minimum
        || low < c.min_color_ratio * high || lab_distance > c.max_lab_distance)
        return kInvalidCost;
    const float radius = std::max(
        c.association_min_px,
        c.association_k * static_cast<float>(std::max(reference.bounds.width, reference.bounds.height)));
    const float dx = value.center.x - predicted.center.x, dy = value.center.y - predicted.center.y;
    const float position = (dx * dx + dy * dy) / (radius * radius);
    if (!(position <= 1.0F))
        return kInvalidCost;
    return position + 0.1F * (std::fabs(1.0F - wr) + std::fabs(1.0F - hr))
         + 0.1F * lab_distance / c.max_lab_distance;
}

bool Association::SameBars(const Target& value, const Target& reference, float tolerance) noexcept {
    if (!value.valid || !reference.valid)
        return false;
    if (!value.has_bars || !reference.has_bars)
        return true;
    const auto squared = [](Point p, Point q) {
        return (p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y);
    };
    const auto& [a, b] = value.bars;
    const auto& [old_a, old_b] = reference.bars;
    const float direct = std::max(squared(a, old_a), squared(b, old_b));
    const float reverse = std::max(squared(a, old_b), squared(b, old_a));
    return std::min(direct, reverse) <= tolerance * tolerance;
}

bool Light::ValidConfig() const noexcept {
    const auto& c = config_;
    return c.colors != 0 && (c.colors & ~kAllColors) == 0 && c.min_area > 0
        && c.far_area_threshold > 0 && c.far_size_px > 0 && c.search_step > 0
        && c.search_step <= 8 && c.confirm_frames > 0 && c.far_confirm_frames > 0
        && Numeric::IsPositive(c.far_jump_k) && c.far_jump_min_px > 0
        && Numeric::IsFraction(c.min_fill) && Numeric::IsFraction(c.min_circularity)
        && std::isfinite(c.circularity_weight) && c.circularity_weight >= 0.0F
        && Numeric::IsPositive(c.aspect_min) && Numeric::IsPositive(c.aspect_max)
        && c.aspect_min <= c.aspect_max && Numeric::IsPositive(c.max_area_fraction)
        && c.max_area_fraction <= 1.0F;
}

Light::Rejection Light::Check(const Blob& blob, std::uint64_t frame_area) const noexcept {
    if (blob.area < config_.min_area)
        return Rejection::kSmall;
    if (static_cast<double>(blob.area) > static_cast<double>(frame_area) * config_.max_area_fraction)
        return Rejection::kArea;
    // Small / far lamps must pass circularity as well; it is corrected for pixel aspect.
    if (blob.circularity < config_.min_circularity)
        return Rejection::kCircularity;
    const bool far = blob.area < config_.far_area_threshold
                  || std::max(blob.bounds.width, blob.bounds.height) < config_.far_size_px;
    if (!far) {
        if (blob.fill < config_.min_fill)
            return Rejection::kFill;
        if (blob.aspect < config_.aspect_min || blob.aspect > config_.aspect_max)
            return Rejection::kAspect;
    }
    return Rejection::kNone;
}

float Light::Score(const Blob& blob) const noexcept {
    const float roundness = std::max(0.05F, blob.circularity);
    return static_cast<float>(blob.area) * std::pow(roundness, config_.circularity_weight)
         * std::max(0.05F, blob.color_quality);
}

bool Light::Far(const Target& target) const noexcept {
    return target.area < config_.far_area_threshold
        || std::max(target.bounds.width, target.bounds.height) < config_.far_size_px;
}

Target Light::ToTarget(const Blob& blob) noexcept {
    Target target{};
    target.valid = true;
    target.center = blob.center;
    target.bounds = blob.bounds;
    target.area = blob.area;
    target.radius = std::sqrt(static_cast<float>(blob.area) / kPi);
    target.quality = blob.color_quality * blob.circularity * (blob.clipped ? 0.25F : 1.0F);
    target.clipped = blob.clipped;
    target.color = blob.color;
    target.lab = blob.lab;
    target.color_quality = blob.color_quality;
    target.circularity = blob.circularity;
    return target;
}

bool Armor::ValidConfig() const noexcept {
    const auto& c = config_;
    const std::array positive{
        c.length_min_k, c.length_max_k,   c.tiny_px,         c.aspect_min,
        c.gap_max_k,    c.diagonal_min_sin, c.window_width_k, c.window_height_k,
        c.span_min_k,   c.span_max_k,     c.association_k,   c.association_min_px,
        c.max_lab_distance, c.parallel_deg};
    return c.colors != 0 && (c.colors & ~kAllColors) == 0
        && std::all_of(positive.begin(), positive.end(), Numeric::IsPositive)
        && c.length_floor > 0 && c.area_floor > 0 && c.length_min_k <= c.length_max_k
        && c.span_min_k <= c.span_max_k && c.diagonal_min_sin <= 1.0F
        && c.plate_up_lo_pct <= c.plate_up_hi_pct && c.plate_offx_pct > 0 && c.track_hold > 0
        && c.confirm_frames > 0 && Numeric::IsFraction(c.fill_min)
        && Numeric::IsFraction(c.thickness_ratio) && Numeric::IsFraction(c.length_ratio)
        && Numeric::IsFraction(c.overlap) && Numeric::IsFraction(c.max_gap_color_fraction)
        && std::isfinite(c.window_gap_k) && c.window_gap_k >= 0.0F
        && std::isfinite(c.border_penalty) && c.border_penalty >= 0.0F
        && std::isfinite(c.flat_deg) && c.flat_deg >= 0.0F && c.flat_deg <= 45.0F
        && c.parallel_deg < 90.0F;
}

Rect Armor::SearchWindow(
    const Config& c, const Target& light, std::uint32_t width, std::uint32_t height) noexcept {
    if (!light.valid || !Numeric::IsPositive(light.radius))
        return {};
    const float diameter = 2.0F * light.radius;
    const float w = c.window_width_k * diameter, h = c.window_height_k * diameter;
    return Rect::Clip(
        light.center.x - w * 0.5F, light.center.y - light.radius - c.window_gap_k * diameter - h,
        w, h, width, height);
}

bool Armor::MakeBar(const Blob& blob, float scale, Bar& bar) const noexcept {
    // Bars are filled rectangles; lamp circularity is never a bar gate.
    const auto& c = config_;
    const float minimum = std::max(static_cast<float>(c.length_floor), std::floor(c.length_min_k * scale));
    const float maximum = std::max(minimum, std::floor(c.length_max_k * scale));
    if (static_cast<float>(blob.area) < std::max(static_cast<float>(c.area_floor), std::floor(minimum * 0.5F)))
        return false;
    // Tiny bars may relax the scale-dependent length, but must stay rectangular.
    if (static_cast<float>(std::max(blob.bounds.width, blob.bounds.height)) >= c.tiny_px
        && (blob.length < minimum || blob.length > maximum))
        return false;
    if (blob.length < c.aspect_min * blob.thickness || blob.oriented_fill < c.fill_min)
        return false;
    Point axis = blob.axis;
    const bool vertical = std::fabs(axis.y) >= std::fabs(axis.x);
    if ((vertical && axis.y < 0.0F) || (!vertical && axis.x < 0.0F))
        axis = {-axis.x, -axis.y};
    const float skew = std::atan2(
        std::min(std::fabs(axis.x), std::fabs(axis.y)), std::max(std::fabs(axis.x), std::fabs(axis.y)))
        * 180.0F / kPi;
    float span = std::max(0.0F, blob.length - 1.0F);
    if (skew <= c.flat_deg) {
        axis = vertical ? Point{0.0F, 1.0F} : Point{1.0F, 0.0F};
        span = static_cast<float>((vertical ? blob.bounds.height : blob.bounds.width) - 1U);
    }
    bar.blob = &blob;
    bar.center = blob.box_center;
    bar.axis = axis;
    bar.length = blob.length;
    bar.thickness = blob.thickness;
    bar.first = {bar.center.x - axis.x * span * 0.5F, bar.center.y - axis.y * span * 0.5F};
    bar.last = {bar.center.x + axis.x * span * 0.5F, bar.center.y + axis.y * span * 0.5F};
    return true;
}

bool Armor::PairCenter(const Bar& a, const Bar& b, Point& center) const noexcept {
    // The plate center is the intersection of the endpoint quadrilateral's diagonals.
    const auto cross = [](Point p, Point q) { return static_cast<double>(p.x) * q.y - static_cast<double>(p.y) * q.x; };
    Point b0 = b.first, b1 = b.last;
    if ((a.last.x - a.first.x) * (b1.x - b0.x) + (a.last.y - a.first.y) * (b1.y - b0.y) < 0.0F)
        std::swap(b0, b1);
    const Point u{b1.x - a.first.x, b1.y - a.first.y}, v{b0.x - a.last.x, b0.y - a.last.y};
    const Point offset{a.last.x - a.first.x, a.last.y - a.first.y};
    const double determinant = cross(u, v);
    const double lengths = std::sqrt(
        (static_cast<double>(u.x) * u.x + static_cast<double>(u.y) * u.y)
        * (static_cast<double>(v.x) * v.x + static_cast<double>(v.y) * v.y));
    if (lengths <= 0.0 || std::fabs(determinant) < config_.diagonal_min_sin * lengths)
        return false;
    const double t = cross(offset, v) / determinant, s = cross(offset, u) / determinant;
    if (t < 0.0 || t > 1.0 || s < 0.0 || s > 1.0)
        return false;
    center = {static_cast<float>(a.first.x + t * u.x), static_cast<float>(a.first.y + t * u.y)};
    return std::isfinite(center.x) && std::isfinite(center.y);
}

float Armor::GapColorFraction(
    const Bar& a, const Bar& b, float separation, const camera::FrameView& frame,
    const ColorTable& colors) noexcept {
    // Nine samples between the bars, away from both bars and their end caps.
    const float dx = b.center.x - a.center.x, dy = b.center.y - a.center.y;
    const float low = std::min(0.45F, a.thickness * 0.5F / separation);
    const float high = 1.0F - std::min(0.45F, b.thickness * 0.5F / separation);
    const ColorMask bar_color = mask_of(a.blob->color);
    const auto max_x = static_cast<float>(frame.width - 1U), max_y = static_cast<float>(frame.height - 1U);
    unsigned matched = 0;
    for (int row = -1; row <= 1; ++row) {
        const float along = static_cast<float>(row) * std::min(a.length, b.length) * 0.25F;
        for (int col = 1; col <= 3; ++col) {
            const float t = low + (high - low) * static_cast<float>(col) * 0.25F;
            const auto x = static_cast<std::uint32_t>(
                std::clamp(std::nearbyint(a.center.x + dx * t + a.axis.x * along), 0.0F, max_x));
            const auto y = static_cast<std::uint32_t>(
                std::clamp(std::nearbyint(a.center.y + dy * t + a.axis.y * along), 0.0F, max_y));
            if ((colors.classify(ColorTable::Rgb565(frame.row(y), x)) & bar_color) != 0)
                ++matched;
        }
    }
    return static_cast<float>(matched) / 9.0F;
}

Target Armor::Select(std::span<const Blob> blobs, const Search& search, ScanCounters& counters) const noexcept {
    const auto& c = config_;
    const Target& light = search.light;
    const Target& predicted = search.predicted;
    if (!light.valid || !Numeric::IsPositive(light.radius))
        return {};
    const float scale = 2.0F * light.radius;
    std::array<Bar, 256> bars{};
    std::size_t count = 0;
    for (const auto& blob : blobs) {
        if (count == bars.size())
            break;
        if (predicted.valid) {
            // A tracked plate keeps its bars: candidates must touch the predicted box.
            const float margin = AssociationTolerance(predicted);
            const float px0 = static_cast<float>(predicted.bounds.x) - margin;
            const float py0 = static_cast<float>(predicted.bounds.y) - margin;
            const float px1 = static_cast<float>(predicted.bounds.x + predicted.bounds.width) + margin;
            const float py1 = static_cast<float>(predicted.bounds.y + predicted.bounds.height) + margin;
            const float bx0 = static_cast<float>(blob.bounds.x), by0 = static_cast<float>(blob.bounds.y);
            const float bx1 = bx0 + static_cast<float>(blob.bounds.width);
            const float by1 = by0 + static_cast<float>(blob.bounds.height);
            if (std::min(px1, bx1) - std::max(px0, bx0) <= 0.0F
                || std::min(py1, by1) - std::max(py0, by0) <= 0.0F) {
                ++counters.rejected_association;
                continue;
            }
        }
        if (MakeBar(blob, scale, bars[count]))
            ++count;
        else
            ++counters.rejected_bar_shape;
    }

    const float parallel = std::cos(c.parallel_deg * kPi / 180.0F);
    const float anchor = static_cast<float>(std::max(light.bounds.width, light.bounds.height));
    Target best{};
    float best_score = kInvalidCost;
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = i + 1; j < count; ++j) {
            const Bar& a = bars[i];
            const Bar& b = bars[j];
            const Blob& ba = *a.blob;
            const Blob& bb = *b.blob;
            if (ba.color != bb.color) {
                ++counters.rejected_pair_color;
                continue;
            }
            const float dot = a.axis.x * b.axis.x + a.axis.y * b.axis.y;
            const float longest = std::max(a.length, b.length), shortest = std::min(a.length, b.length);
            const Point offset{b.center.x - a.center.x, b.center.y - a.center.y};
            const float separation = std::fabs(offset.x * a.axis.y - offset.y * a.axis.x);
            const float along = std::fabs(offset.x * a.axis.x + offset.y * a.axis.y);
            Point center{};
            if (std::fabs(dot) < parallel || shortest < c.length_ratio * longest
                || std::min(a.thickness, b.thickness) < c.thickness_ratio * std::max(a.thickness, b.thickness)
                || (a.length + b.length) * 0.5F - along < c.overlap * shortest
                || separation <= (a.thickness + b.thickness) * 0.5F || separation > c.gap_max_k * longest
                || separation < c.span_min_k * scale || separation > c.span_max_k * scale
                || !PairCenter(a, b, center)) {
                ++counters.rejected_pair_geometry;
                continue;
            }
            const float up = light.center.y - center.y;
            const float off_x = std::fabs(center.x - light.center.x);
            if (up * 100.0F < static_cast<float>(c.plate_up_lo_pct) * anchor
                || up * 100.0F > static_cast<float>(c.plate_up_hi_pct) * anchor
                || off_x * 100.0F > static_cast<float>(c.plate_offx_pct) * anchor) {
                ++counters.rejected_anchor;
                continue;
            }
            const float lab_distance = Lab::distance(ba.lab, bb.lab);
            if (lab_distance > c.max_lab_distance) {
                ++counters.rejected_pair_lab;
                continue;
            }
            const float gap = GapColorFraction(a, b, separation, search.frame, search.colors);
            if (gap > c.max_gap_color_fraction) {
                ++counters.rejected_gap;
                continue;
            }
            const float similarity = std::max(0.0F, 1.0F - lab_distance / c.max_lab_distance);
            Target target{};
            target.valid = true;
            target.center = center;
            target.bounds = ba.bounds.unite(bb.bounds);
            target.area = ba.area + bb.area;
            target.radius = std::sqrt(static_cast<float>(target.area) / kPi);
            target.clipped = ba.clipped || bb.clipped;
            target.color = ba.color;
            target.color_quality = std::min(ba.color_quality, bb.color_quality);
            target.quality = target.color_quality * (0.5F + 0.5F * similarity) * (1.0F - gap)
                           * (target.clipped ? 0.25F : 1.0F);
            target.lab = {(ba.lab.l + bb.lab.l) * 0.5F, (ba.lab.a + bb.lab.a) * 0.5F,
                          (ba.lab.b + bb.lab.b) * 0.5F};
            target.has_bars = true;
            target.bars = {a.center, b.center};
            target.bar_lab_distance = lab_distance;
            target.gap_color_fraction = gap;
            float score = ((longest - shortest) + along) * 100.0F / longest
                        + (1.0F - similarity) * 25.0F + gap * 25.0F
                        + c.border_penalty * static_cast<float>(int{ba.clipped} + int{bb.clipped});
            if (predicted.valid) {
                const float tolerance = std::max(c.association_min_px, c.association_k * longest);
                const float change = (center.x - predicted.center.x) * (center.x - predicted.center.x)
                                   + (center.y - predicted.center.y) * (center.y - predicted.center.y);
                if (change > tolerance * tolerance
                    || !Association::SameBars(target, predicted, tolerance)
                    || !std::isfinite(Association::Cost(target, predicted, search.reference, search.association))) {
                    ++counters.rejected_association;
                    continue;
                }
                score += 100.0F * change / (tolerance * tolerance);
            }
            if (score < best_score || (score == best_score && target.area > best.area)) {
                best = target;
                best_score = score;
            }
        }
    }
    return best;
}

} // namespace dart::detection
