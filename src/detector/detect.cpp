#include "src/detector/detect.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <new>

namespace dart::detection {

std::expected<void, DetectionError> Detect::Workspace::Prepare(
    std::uint32_t max_width, std::uint32_t max_height, float pixel_aspect, Config config) noexcept {
    if (max_width == 0 || max_height == 0 || max_width > 65535 || max_height > 65535
        || !Numeric::IsPositive(pixel_aspect) || config.max_candidates == 0
        || config.max_candidates > Config::kMaxCandidates || config.max_spans == 0
        || config.max_spans > (1U << 20U))
        return std::unexpected(DetectionError::kInvalidConfig);
    const std::uint32_t words = (max_width + 63U) / 64U;
    const std::size_t corners = 2U * static_cast<std::size_t>(max_height);
    std::unique_ptr<std::uint64_t[]> visited(new (std::nothrow) std::uint64_t[std::size_t{words} * max_height]());
    std::unique_ptr<Span[]> spans(new (std::nothrow) Span[config.max_spans]);
    std::unique_ptr<std::uint32_t[]> row_min(new (std::nothrow) std::uint32_t[max_height]);
    std::unique_ptr<std::uint32_t[]> row_max(new (std::nothrow) std::uint32_t[max_height]);
    std::unique_ptr<Corner[]> points(new (std::nothrow) Corner[corners]);
    std::unique_ptr<Corner[]> hull(new (std::nothrow) Corner[2U * corners + 1U]);
    std::unique_ptr<Blob[]> candidates(new (std::nothrow) Blob[config.max_candidates]);
    if (!visited || !spans || !row_min || !row_max || !points || !hull || !candidates)
        return std::unexpected(DetectionError::kNoMemory);
    std::fill_n(row_min.get(), max_height, std::numeric_limits<std::uint32_t>::max());
    std::fill_n(row_max.get(), max_height, 0U);
    visited_ = std::move(visited);
    spans_ = std::move(spans);
    row_min_ = std::move(row_min);
    row_max_ = std::move(row_max);
    points_ = std::move(points);
    hull_ = std::move(hull);
    candidates_ = std::move(candidates);
    max_width_ = max_width;
    max_height_ = max_height;
    words_ = words;
    max_spans_ = config.max_spans;
    max_candidates_ = config.max_candidates;
    span_high_water_ = 0;
    pixel_aspect_ = pixel_aspect;
    return {};
}

void Detect::Workspace::ClearVisited(Rect window) noexcept {
    const std::uint32_t first = window.x / 64U, last = (window.x + window.width - 1U) / 64U;
    for (std::uint32_t y = window.y; y < window.y + window.height; ++y)
        std::fill(&visited_[y * words_ + first], &visited_[y * words_ + last] + 1, std::uint64_t{0});
}

bool Detect::Workspace::Fill(
    const camera::FrameView& frame, const ColorTable& table, Rect window, ColorMask color,
    std::uint32_t seed_x, std::uint32_t seed_y, Region& region) noexcept {
    // Scanline fill, 8-connected, confined to the window. Every span is
    // marked when pushed, so each pixel enters the region exactly once.
    const std::uint32_t right = window.x + window.width, bottom = window.y + window.height;
    const auto open = [&](const std::byte* row, std::uint32_t x, std::uint32_t y) {
        return !visited(x, y) && (table.classify(ColorTable::Rgb565(row, x)) & color) != 0;
    };
    const auto sum_squares = [](std::uint64_t n) { return n * (n + 1U) * (2U * n + 1U) / 6U; };
    std::uint32_t top = 0;
    const auto add = [&](std::uint32_t y, std::uint32_t x0, std::uint32_t x1) {
        if (top == max_spans_)
            return false;
        spans_[top++] = {y, x0, x1};
        span_high_water_ = std::max(span_high_water_, top);
        const auto* row = frame.row(y);
        for (std::uint32_t x = x0; x <= x1; ++x) {
            visited_[y * words_ + x / 64U] |= std::uint64_t{1} << (x % 64U);
            const auto* lab = table.lab(ColorTable::Rgb565(row, x));
            region.l += lab[0];
            region.a += lab[1];
            region.b += lab[2];
        }
        const std::uint64_t n = x1 - x0 + 1U;
        const std::uint64_t sum_x = (std::uint64_t{x0} + x1) * n / 2U;
        region.area += n;
        region.sx += sum_x;
        region.sy += n * y;
        region.sxx += sum_squares(x1) - (x0 > 0 ? sum_squares(x0 - 1U) : 0U);
        region.syy += n * y * y;
        region.sxy += sum_x * y;
        region.x0 = std::min(region.x0, x0);
        region.x1 = std::max(region.x1, x1);
        region.y0 = std::min(region.y0, y);
        region.y1 = std::max(region.y1, y);
        row_min_[y] = std::min(row_min_[y], x0);
        row_max_[y] = std::max(row_max_[y], x1);
        return true;
    };
    const auto extend = [&](const std::byte* row, std::uint32_t x, std::uint32_t y,
                            std::uint32_t& x0, std::uint32_t& x1) {
        x0 = x1 = x;
        while (x0 > window.x && open(row, x0 - 1U, y))
            --x0;
        while (x1 + 1U < right && open(row, x1 + 1U, y))
            ++x1;
    };
    region = {};
    region.x0 = region.x1 = seed_x;
    region.y0 = region.y1 = seed_y;
    std::uint32_t x0 = 0, x1 = 0;
    extend(frame.row(seed_y), seed_x, seed_y, x0, x1);
    if (!add(seed_y, x0, x1))
        return false;
    while (top > 0) {
        const Span span = spans_[--top];
        for (const bool up : {true, false}) {
            if ((up && span.y == window.y) || (!up && span.y + 1U >= bottom))
                continue;
            const std::uint32_t y = up ? span.y - 1U : span.y + 1U;
            const auto* row = frame.row(y);
            const std::uint32_t last = std::min(span.x1 + 1U, right - 1U);
            for (std::uint32_t x = span.x0 > window.x ? span.x0 - 1U : span.x0; x <= last; ++x) {
                if (!open(row, x, y))
                    continue;
                extend(row, x, y, x0, x1);
                if (!add(y, x0, x1))
                    return false;
                x = x1;
            }
        }
    }
    return true;
}

Detect::Workspace::Rectangle Detect::Workspace::MinimumRectangle(const Region& region) noexcept {
    // Collect each row's extreme pixel centers, already ordered by (y, x),
    // and restore the row tables for the next region.
    std::size_t count = 0;
    for (std::uint32_t y = region.y0; y <= region.y1; ++y) {
        if (row_min_[y] <= row_max_[y]) {
            points_[count++] = {static_cast<std::int32_t>(row_min_[y]), static_cast<std::int32_t>(y)};
            if (row_max_[y] != row_min_[y])
                points_[count++] = {static_cast<std::int32_t>(row_max_[y]), static_cast<std::int32_t>(y)};
        }
        row_min_[y] = std::numeric_limits<std::uint32_t>::max();
        row_max_[y] = 0;
    }
    Rectangle box{};
    if (count < 2)
        return box;
    // Monotone-chain convex hull; collinear points are dropped.
    const auto turn = [](Corner o, Corner a, Corner b) {
        return static_cast<std::int64_t>(a.x - o.x) * (b.y - o.y)
             - static_cast<std::int64_t>(a.y - o.y) * (b.x - o.x);
    };
    std::size_t size = 0;
    for (std::size_t i = 0; i < count; ++i) {
        while (size >= 2 && turn(hull_[size - 2], hull_[size - 1], points_[i]) <= 0)
            --size;
        hull_[size++] = points_[i];
    }
    for (std::size_t i = count - 1, lower = size + 1; i-- > 0;) {
        while (size >= lower && turn(hull_[size - 2], hull_[size - 1], points_[i]) <= 0)
            --size;
        hull_[size++] = points_[i];
    }
    --size; // The last point repeats the first.
    // Rotating calipers, quadratic in the (small) hull size: the minimum-area
    // rectangle has one side collinear with a hull edge.
    double best_area = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < size; ++i) {
        const Corner a = hull_[i], b = hull_[(i + 1) % size];
        const double ex = b.x - a.x, ey = b.y - a.y, norm = std::hypot(ex, ey);
        if (norm == 0.0)
            continue;
        const double ux = ex / norm, uy = ey / norm;
        double u0 = std::numeric_limits<double>::infinity(), u1 = -u0, v0 = u0, v1 = -u0;
        for (std::size_t k = 0; k < size; ++k) {
            const double u = hull_[k].x * ux + hull_[k].y * uy;
            const double v = -hull_[k].x * uy + hull_[k].y * ux;
            u0 = std::min(u0, u);
            u1 = std::max(u1, u);
            v0 = std::min(v0, v);
            v1 = std::max(v1, v);
        }
        const double area = (u1 - u0) * (v1 - v0);
        if (area >= best_area)
            continue;
        best_area = area;
        const double um = (u0 + u1) * 0.5, vm = (v0 + v1) * 0.5;
        box.center = {static_cast<float>(um * ux - vm * uy), static_cast<float>(um * uy + vm * ux)};
        if (u1 - u0 >= v1 - v0) {
            box.axis = {static_cast<float>(ux), static_cast<float>(uy)};
            box.length = static_cast<float>(u1 - u0);
            box.thickness = static_cast<float>(v1 - v0);
        } else {
            box.axis = {static_cast<float>(-uy), static_cast<float>(ux)};
            box.length = static_cast<float>(v1 - v0);
            box.thickness = static_cast<float>(u1 - u0);
        }
        box.degenerate = false;
    }
    return box;
}

Blob Detect::Workspace::Measure(
    const Region& region, Rectangle box, Rect window, Color color, const ColorTable& table) const noexcept {
    Blob blob{};
    const double n = static_cast<double>(region.area);
    blob.area = static_cast<std::uint32_t>(region.area);
    blob.color = color;
    blob.bounds = {region.x0, region.y0, region.x1 - region.x0 + 1U, region.y1 - region.y0 + 1U};
    blob.lab = {static_cast<float>(region.l / n), static_cast<float>(region.a / n),
                static_cast<float>(region.b / n)};
    const double mx = region.sx / n, my = region.sy / n;
    blob.center = {static_cast<float>(mx), static_cast<float>(my)};
    // Second moments of unit pixels (each adds 1/12): a single pixel is round.
    const double xx = region.sxx / n - mx * mx + 1.0 / 12.0;
    const double yy = region.syy / n - my * my + 1.0 / 12.0;
    const double xy = region.sxy / n - mx * my;
    const double box_area = static_cast<double>(blob.bounds.width) * blob.bounds.height;
    blob.fill = static_cast<float>(std::min(1.0, n / box_area));
    // Undo the camera's horizontal pixel scale before judging roundness. A
    // square also has equal axes, so penalize fills away from a disk's pi/4.
    const double s = pixel_aspect_;
    const double sxx = xx * s * s, sxy = xy * s;
    const double spread = std::hypot(sxx - yy, 2.0 * sxy);
    const double axis_ratio = std::sqrt(std::max(0.0, sxx + yy - spread) / std::max(1e-9, sxx + yy + spread));
    const double disk = std::numbers::pi / 4.0;
    const double fill_score = std::min({1.0, blob.fill / disk, disk / std::max(1e-9, double{blob.fill})});
    blob.circularity = static_cast<float>(axis_ratio * fill_score);
    blob.aspect = static_cast<float>(blob.bounds.width * s / blob.bounds.height);
    if (box.degenerate || box.length == 0.0F) {
        const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
        box.axis = {static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle))};
        box.center = box.degenerate ? blob.center : box.center;
    }
    blob.length = box.length + 1.0F;
    blob.thickness = box.thickness + 1.0F;
    blob.axis = box.axis;
    blob.box_center = box.center;
    blob.oriented_fill = std::min(1.0F, static_cast<float>(n) / (blob.length * blob.thickness));
    // Color quality is a tuning score: chroma, then centrality in the profile.
    const float chroma_score =
        std::min(1.0F, blob.lab.chroma() / std::max(24.0F, 2.0F * table.config().min_chroma));
    blob.color_quality = chroma_score * (0.5F + 0.5F * table.range(color).centrality(blob.lab));
    blob.clipped = region.x0 <= window.x || region.y0 <= window.y
                || region.x1 + 1U >= window.x + window.width
                || region.y1 + 1U >= window.y + window.height;
    return blob;
}

std::expected<std::span<const Blob>, DetectionError> Detect::Workspace::Scan(
    const camera::FrameView& frame, const ColorTable& table, const Request& request,
    ScanCounters& counters) noexcept {
    const Rect window = request.window;
    if (!visited_ || !table.prepared())
        return std::unexpected(DetectionError::kNotPrepared);
    if (window.empty() || window.x >= frame.width || window.y >= frame.height
        || window.width > frame.width - window.x || window.height > frame.height - window.y
        || frame.width > max_width_ || frame.height > max_height_)
        return std::unexpected(DetectionError::kInvalidWindow);
    const std::uint32_t step = std::clamp(request.step, 1U, 8U);
    const std::uint32_t right = window.x + window.width, bottom = window.y + window.height;
    const float min_chroma = table.config().min_chroma;
    std::uint32_t count = 0, extracted = 0;
    for (std::size_t index = 0; index < kColorCount; ++index) {
        const auto color = static_cast<Color>(index);
        const ColorMask bit = mask_of(color);
        if ((request.colors & bit) == 0)
            continue;
        ClearVisited(window);
        for (std::uint32_t y = window.y; y < bottom; y += step) {
            const auto* row = frame.row(y);
            for (std::uint32_t x = window.x; x < right; x += step) {
                if (visited(x, y) || (table.classify(ColorTable::Rgb565(row, x)) & bit) == 0)
                    continue;
                Region region{};
                const bool filled = Fill(frame, table, window, bit, x, y, region);
                const Rectangle box = MinimumRectangle(region);
                if (!filled)
                    return std::unexpected(DetectionError::kWorkspaceExhausted);
                const std::uint64_t box_area =
                    std::uint64_t{region.x1 - region.x0 + 1U} * (region.y1 - region.y0 + 1U);
                if (region.area < request.min_area || box_area < request.min_area)
                    continue;
                ++counters.raw_blobs;
                if (++extracted > max_candidates_)
                    return std::unexpected(DetectionError::kWorkspaceExhausted);
                const double n = static_cast<double>(region.area);
                if (Lab{static_cast<float>(region.l / n), static_cast<float>(region.a / n),
                        static_cast<float>(region.b / n)}.chroma() < min_chroma) {
                    ++counters.rejected_color;
                    continue;
                }
                candidates_[count++] = Measure(region, box, window, color, table);
            }
        }
    }
    return std::span<const Blob>(candidates_.get(), count);
}

std::expected<void, DetectionError> Detect::Prepare(const camera::FrameInfo& geometry) noexcept {
    if (!Supports(geometry.format))
        return std::unexpected(DetectionError::kUnsupportedFormat);
    const Track probe{config_.light, config_.armor, config_.track, config_.roi};
    if (!probe.valid_config() || !ColorTable::ValidConfig(config_.color))
        return std::unexpected(DetectionError::kInvalidConfig);
    if (const auto prepared = workspace_.Prepare(
            geometry.width, geometry.height, geometry.pixel_aspect, config_.workspace); !prepared)
        return prepared;
    if (!colors_.Prepare(config_.color))
        return std::unexpected(DetectionError::kNoMemory);
    prepared_ = geometry;
    return {};
}

std::expected<DetectResult, DetectionError>
    Detect::update(const camera::FrameView& frame, Track& track) noexcept {
    if (!colors_.prepared() || prepared_.width == 0)
        return std::unexpected(DetectionError::kNotPrepared);
    if (frame.format != prepared_.format)
        return std::unexpected(DetectionError::kUnsupportedFormat);
    if (!frame.valid() || frame.width > prepared_.width || frame.height > prepared_.height)
        return std::unexpected(DetectionError::kInvalidFrame);
    const auto search = track.plan(frame);
    if (!search)
        return std::unexpected(search.error());
    DetectResult result{};
    result.source = {frame.sequence, frame.source_pts, frame.received, frame.source_pts_timebase_validated};
    result.scan.gap_reset = search->gap_reset;
    auto measured = measure(frame, search->window, *search, search->tracking, search->validate, track, result);
    if (measured && search->tracking && !result.scan.full_frame
        && (!result.light.valid || result.light.clipped || (result.armor.valid && result.armor.clipped))) {
        // A lost or edge-clipped tracked target is re-measured on the whole frame
        // in this cycle, still bound to its identity. Counters accumulate.
        result.light = result.armor = {};
        result.scan.rescanned = true;
        measured = measure(frame, Rect::full(frame.width, frame.height), *search, true, true, track, result);
    }
    if (!measured) {
        track.Abort();
        return std::unexpected(measured.error());
    }
    track.finish(result, search->tracking);
    return result;
}

std::expected<void, DetectionError> Detect::measure(
    const camera::FrameView& frame, Rect window, const Track::Search& search, bool tracking,
    bool validate, const Track& track, DetectResult& result) noexcept {
    auto& scan = result.scan;
    auto& counters = scan.counters;
    scan.phase = tracking ? DetectionPhase::kTrack : DetectionPhase::kDetect;
    scan.search_window = window;
    scan.full_frame = window.is_full(frame.width, frame.height);
    const std::uint32_t step = tracking ? 1U : config_.light.search_step;
    const auto lights = workspace_.Scan(
        frame, colors_, {window, config_.light.colors, step, config_.light.min_area}, counters);
    if (!lights)
        return std::unexpected(lights.error());
    counters.light_candidates += static_cast<std::uint32_t>(lights->size());

    const auto frame_area = std::uint64_t{frame.width} * frame.height;
    float best = kInvalidCost;
    for (const auto& blob : *lights) {
        switch (light_.Check(blob, frame_area)) {
        case Light::Rejection::kNone: break;
        case Light::Rejection::kSmall:
        case Light::Rejection::kArea: ++counters.rejected_light_area; continue;
        default: ++counters.rejected_light_shape; continue;
        }
        const Target candidate = Light::ToTarget(blob);
        // Acquisition takes the strongest lamp; tracking keeps the same identity.
        const float score = tracking ? track.light_cost(candidate, search.predicted_light)
                                     : -light_.Score(blob);
        if (!std::isfinite(score)) {
            ++counters.rejected_association;
            continue;
        }
        if (score < best) {
            best = score;
            result.light = candidate;
        }
    }

    validate = !tracking || validate || track.quality_dropped(result.light);
    scan.full_validation = validate;
    scan.armor_scanned = result.light.valid && (search.predicted_armor.valid || validate);
    if (scan.armor_scanned) {
        const Rect area = armor_.SearchWindow(result.light, frame.width, frame.height);
        if (!area.empty()) {
            const auto bars = workspace_.Scan(
                frame, colors_, {area, config_.armor.colors, 1U, config_.armor.area_floor}, counters);
            if (!bars)
                return std::unexpected(bars.error());
            counters.armor_candidates += static_cast<std::uint32_t>(bars->size());
            result.armor = armor_.Select(
                *bars,
                {result.light, search.predicted_armor, track.armor_reference(), track.association(),
                 frame, colors_},
                counters);
        }
    }
    scan.candidates = counters.light_candidates + counters.armor_candidates;
    return {};
}

class ParameterValue final {
public:
    static bool Assign(std::uint32_t& field, double value) noexcept {
        if (value < 0 || value > std::numeric_limits<std::uint32_t>::max() || std::floor(value) != value)
            return false;
        field = static_cast<std::uint32_t>(value);
        return true;
    }
    static bool Assign(float& field, double value) noexcept {
        if (std::fabs(value) > std::numeric_limits<float>::max())
            return false;
        field = static_cast<float>(value);
        return true;
    }
    static bool AssignMask(ColorMask& field, double value) noexcept {
        if (value < 1 || value > kAllColors || std::floor(value) != value)
            return false;
        field = static_cast<ColorMask>(value);
        return true;
    }
    static bool AssignLab(int& field, double value) noexcept {
        // Range order and channel limits are checked with the whole profile.
        if (value < -128 || value > 127 || std::floor(value) != value)
            return false;
        field = static_cast<int>(value);
        return true;
    }
    static bool AssignColor(ColorTable::Config& config, std::string_view name, double value) noexcept {
        constexpr std::array<std::string_view, kColorCount> kNames{"green", "red", "blue"};
        for (std::size_t index = 0; index < kNames.size(); ++index) {
            if (!name.starts_with(kNames[index]) || name.size() <= kNames[index].size()
                || name[kNames[index].size()] != '.')
                continue;
            const auto field = name.substr(kNames[index].size() + 1U);
            auto& range = config.ranges[index];
            if (field == "l_min") return AssignLab(range.l_min, value);
            if (field == "l_max") return AssignLab(range.l_max, value);
            if (field == "a_min") return AssignLab(range.a_min, value);
            if (field == "a_max") return AssignLab(range.a_max, value);
            if (field == "b_min") return AssignLab(range.b_min, value);
            if (field == "b_max") return AssignLab(range.b_max, value);
            return false;
        }
        return false;
    }
};

bool Detect::set_parameter(Config& config, std::string_view name, double value) noexcept {
    if (!std::isfinite(value))
        return false;
    // Names are the Config field paths; defaults live only in the Config types.
#define DART_DETECT_FIELD(path) \
    if (name == #path)          \
        return ParameterValue::Assign(config.path, value);
    DART_DETECT_FIELD(workspace.max_candidates)
    DART_DETECT_FIELD(workspace.max_spans)
    DART_DETECT_FIELD(color.min_chroma)
    DART_DETECT_FIELD(light.min_area)
    DART_DETECT_FIELD(light.far_area_threshold)
    DART_DETECT_FIELD(light.far_size_px)
    DART_DETECT_FIELD(light.search_step)
    DART_DETECT_FIELD(light.confirm_frames)
    DART_DETECT_FIELD(light.far_confirm_frames)
    DART_DETECT_FIELD(light.far_jump_k)
    DART_DETECT_FIELD(light.far_jump_min_px)
    DART_DETECT_FIELD(light.deadband_px)
    DART_DETECT_FIELD(light.min_fill)
    DART_DETECT_FIELD(light.min_circularity)
    DART_DETECT_FIELD(light.circularity_weight)
    DART_DETECT_FIELD(light.aspect_min)
    DART_DETECT_FIELD(light.aspect_max)
    DART_DETECT_FIELD(light.max_area_fraction)
    DART_DETECT_FIELD(armor.length_floor)
    DART_DETECT_FIELD(armor.area_floor)
    DART_DETECT_FIELD(armor.length_min_k)
    DART_DETECT_FIELD(armor.length_max_k)
    DART_DETECT_FIELD(armor.tiny_px)
    DART_DETECT_FIELD(armor.aspect_min)
    DART_DETECT_FIELD(armor.fill_min)
    DART_DETECT_FIELD(armor.flat_deg)
    DART_DETECT_FIELD(armor.parallel_deg)
    DART_DETECT_FIELD(armor.thickness_ratio)
    DART_DETECT_FIELD(armor.length_ratio)
    DART_DETECT_FIELD(armor.overlap)
    DART_DETECT_FIELD(armor.gap_max_k)
    DART_DETECT_FIELD(armor.diagonal_min_sin)
    DART_DETECT_FIELD(armor.border_penalty)
    DART_DETECT_FIELD(armor.window_width_k)
    DART_DETECT_FIELD(armor.window_height_k)
    DART_DETECT_FIELD(armor.window_gap_k)
    DART_DETECT_FIELD(armor.span_min_k)
    DART_DETECT_FIELD(armor.span_max_k)
    DART_DETECT_FIELD(armor.plate_up_lo_pct)
    DART_DETECT_FIELD(armor.plate_up_hi_pct)
    DART_DETECT_FIELD(armor.plate_offx_pct)
    DART_DETECT_FIELD(armor.association_k)
    DART_DETECT_FIELD(armor.association_min_px)
    DART_DETECT_FIELD(armor.track_hold)
    DART_DETECT_FIELD(armor.confirm_frames)
    DART_DETECT_FIELD(armor.max_lab_distance)
    DART_DETECT_FIELD(armor.max_gap_color_fraction)
    DART_DETECT_FIELD(track.association_k)
    DART_DETECT_FIELD(track.association_min_px)
    DART_DETECT_FIELD(track.min_size_ratio)
    DART_DETECT_FIELD(track.min_color_ratio)
    DART_DETECT_FIELD(track.max_lab_distance)
    DART_DETECT_FIELD(track.validation_interval)
    DART_DETECT_FIELD(track.quality_drop_ratio)
    DART_DETECT_FIELD(track.velocity_alpha)
    DART_DETECT_FIELD(track.prediction_max_ms)
    DART_DETECT_FIELD(roi.min_width)
    DART_DETECT_FIELD(roi.min_height)
    DART_DETECT_FIELD(roi.size_k)
    DART_DETECT_FIELD(roi.margin)
    DART_DETECT_FIELD(roi.grow_per_miss)
    DART_DETECT_FIELD(roi.full_area_fraction)
    DART_DETECT_FIELD(roi.reset_after_misses)
    DART_DETECT_FIELD(roi.full_scan_interval)
#undef DART_DETECT_FIELD
    if (name == "light.colors")
        return ParameterValue::AssignMask(config.light.colors, value);
    if (name == "armor.colors")
        return ParameterValue::AssignMask(config.armor.colors, value);
    if (name == "roi.max_gap") {
        std::uint32_t milliseconds = 0;
        if (!ParameterValue::Assign(milliseconds, value) || milliseconds == 0)
            return false;
        config.roi.max_gap = std::chrono::milliseconds{milliseconds};
        return true;
    }
    if (name.starts_with("color."))
        return ParameterValue::AssignColor(config.color, name.substr(6), value);
    return false;
}

} // namespace dart::detection
