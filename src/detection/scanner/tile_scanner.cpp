// ============================================================================
// TileScanner 实现：瓦片粗筛（RVV）+ 瓦片并块 + 只对最亮几块做精修。
//
// 二值图的极性约定（与 GraphicsUtils::save_pbm 一致）：>=128 即亮（255），<128 即暗（0）。
// 阈值化本身在 main.cpp 里由 GraphicsUtils::binarize 完成（RVV，写进池帧供录像/取证），
// 本文件**只读**二值图，不再碰原始 Y 平面 —— 这是"所有视觉处理都在二值图上"的边界。
// ============================================================================

#include "detection/scanner/tile_scanner.hpp"

#include <algorithm>
#include <cstring>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#define DART_HAVE_RVV 1
#endif

namespace dart::detection {

namespace {

// 亮/暗分界。与 GraphicsUtils::binarize 的产出（0/255）和 save_pbm 的极性判定一致。
constexpr uint8_t kBrightMin = 128;

inline uint32_t min_u32(uint32_t a, uint32_t b) { return a < b ? a : b; }

} // namespace

TileScanner::TileScanner(const ScannerConfig &cfg) : cfg_(cfg) {
    tile_ = cfg_.tile ? cfg_.tile : 16;
}

const char *TileScanner::name() const {
#if defined(DART_HAVE_RVV)
    return "rvv-tile";
#else
    return "scalar-tile";
#endif
}

void TileScanner::ensure_grid(uint32_t w, uint32_t h) {
    const uint32_t gw = (w + tile_ - 1) / tile_;
    const uint32_t gh = (h + tile_ - 1) / tile_;
    if (gw == gw_ && gh == gh_ && !count_.empty())
        return;

    gw_ = gw;
    gh_ = gh;
    const size_t n = static_cast<size_t>(gw_) * gh_;
    count_.assign(n, 0);
    xsum_.assign(n, 0);
    ysum_.assign(n, 0);
    comp_id_.assign(n, -1);
    bfs_.reserve(n);
    order_.reserve(n);
    comps_.reserve(cfg_.max_components ? cfg_.max_components : 96);
}

// ---------------------------------------------------------------------------
// 粗筛（RVV）：逐瓦片一条或多条向量。每个瓦片累计三件事 ——
// 命中像素数、命中像素的 x 坐标和、命中像素的 y 坐标和（y 那项用 行号×命中数）。
// 这样"面积/质心"在粗筛阶段就已经是精确值，后面并块只是把它们加起来。
//
// 本工具链（GCC 12.0.1 预发布版）的两个 RVV intrinsic 怪癖，写在这里免得下次有人踩：
//   ① 掩码类型名带 SEW 后缀：u8 + LMUL=1 是 vbool8_t（不是标准里的 vbool1_t），
//      于是比较/取位数长这样：vmsgtu_vx_u8m1_b8 / vcpop_m_b8。
//   ② 归约（vredsum/vwredsumu）是**三向量 + vl** 形式：vredsum_vs_u8m1_u8m1(vs2, vs1, vd, vl)，
//      比标准 intrinsics 多一个"被掩码关闭时保留谁"的向量参数（传零向量即可）。
//      另外这条工具链没有 vwredsumu_vs_u16m1_u8m1，所以下面的按位索引和用 u8 归约完成 ——
//      这也正是把每条向量限制在 16 个像素的原因：Σ(0..15)=120 < 256，绝不会回绕。
// ---------------------------------------------------------------------------
#if defined(DART_HAVE_RVV)
uint32_t TileScanner::tile_pass_rvv(const GrayFrame &f) {
    uint32_t tiles = 0;

    for (uint32_t ty = 0; ty < gh_; ++ty) {
        const uint32_t y0 = ty * tile_;
        const uint32_t y1 = min_u32(y0 + tile_, f.height);

        for (uint32_t y = y0; y < y1; ++y) {
            const uint8_t *row = f.pixels + static_cast<size_t>(y) * f.stride;

            for (uint32_t tx = 0; tx < gw_; ++tx) {
                const uint32_t x0 = tx * tile_;
                if (x0 >= f.width)
                    break;
                const uint32_t x1 = min_u32(x0 + tile_, f.width);

                uint32_t cnt_total = 0;
                uint32_t sum_local = 0; // Σ(命中像素在瓦片内的局部 x)

                for (uint32_t x = x0; x < x1;) {
                    size_t vl = vsetvl_e8m1(static_cast<size_t>(x1 - x));
                    if (vl > 16)
                        vl = 16; // 见上：u8 归约的安全上限（VLEN=128 时本来就是 16）
                    vuint8m1_t v = vle8_v_u8m1(row + x, vl);
                    // 无符号比较：二值图是 0/255，vmsgtu 判"亮"（vmsgt 是有符号比较，不能用）
                    vbool8_t   m = vmsgtu_vx_u8m1_b8(v, kBrightMin - 1, vl);
                    const uint32_t cnt = static_cast<uint32_t>(vcpop_m_b8(m, vl));
                    cnt_total += cnt;

                    if (cnt > 0) {
                        vuint8m1_t vzero = vmv_v_x_u8m1(0, vl);
                        vuint8m1_t vid = vid_v_u8m1(vl); // 0,1,2,...：命中像素在**本条向量内**的索引
                        // vmerge 语义 = mask ? vs1 : vs2（mask 放第一个参数，同 graphics_utils 的写法）
                        vuint8m1_t lm = vmerge_vvm_u8m1(m, vzero, vid, vl);
                        vuint8m1_t rs = vredsum_vs_u8m1_u8m1(lm, vzero, vzero, vl);
                        const uint32_t sum_in_vec = static_cast<uint32_t>(vmv_x_s_u8m1_u8(rs));
                        // 换算到"瓦片内局部索引"：本条向量的起点相对瓦片起点偏移 (x - x0)。
                        // 瓦片边长允许 >16（那时一个瓦片行有多条向量），这一项不能漏。
                        sum_local += cnt * (x - x0) + sum_in_vec;
                    }
                    x += static_cast<uint32_t>(vl);
                }

                if (cnt_total == 0)
                    continue; // 空瓦片是常态：跳过后续标量记账，这一步省下来的是大头
                const uint32_t idx = ty * gw_ + tx;
                count_[idx] += cnt_total;
                xsum_[idx] += x0 * cnt_total + sum_local;
                ysum_[idx] += cnt_total * y;
                ++tiles;
            }
        }
    }
    return tiles;
}
#endif // DART_HAVE_RVV

// ---------------------------------------------------------------------------
// 粗筛（标量参考路径）：宿主机（无 RVV）走这条；板端用它做开机自检的对照。
// 两条路径必须产出逐字段一致的瓦片累加结果。
// ---------------------------------------------------------------------------
uint32_t TileScanner::tile_pass_scalar(const GrayFrame &f) {
    uint32_t tiles = 0;
    for (uint32_t ty = 0; ty < gh_; ++ty) {
        const uint32_t y0 = ty * tile_;
        const uint32_t y1 = min_u32(y0 + tile_, f.height);

        for (uint32_t y = y0; y < y1; ++y) {
            const uint8_t *row = f.pixels + static_cast<size_t>(y) * f.stride;

            for (uint32_t tx = 0; tx < gw_; ++tx) {
                const uint32_t x0 = tx * tile_;
                if (x0 >= f.width)
                    break;
                const uint32_t x1 = min_u32(x0 + tile_, f.width);

                uint32_t cnt = 0;
                uint32_t sum_x = 0;
                for (uint32_t x = x0; x < x1; ++x) {
                    if (row[x] >= kBrightMin) {
                        ++cnt;
                        sum_x += x;
                    }
                }
                if (cnt == 0)
                    continue;
                const uint32_t idx = ty * gw_ + tx;
                count_[idx] += cnt;
                xsum_[idx] += sum_x;
                ysum_[idx] += cnt * y;
                ++tiles;
            }
        }
    }
    return tiles;
}

// ---------------------------------------------------------------------------
// 瓦片并块：8 邻域 BFS。面积/Σx/Σy 直接累加瓦片值 —— 因为"所有有亮像素的瓦片"
// 都在图里，所以这部分是**精确**的（不是估计），r = sqrt(A/π) 也就精确。
// ---------------------------------------------------------------------------
uint32_t TileScanner::merge_components() {
    comps_.clear();
    const uint32_t n = gw_ * gh_;
    const size_t   max_comp = cfg_.max_components ? cfg_.max_components : n;

    for (uint32_t i = 0; i < n; ++i) {
        if (count_[i] == 0 || comp_id_[i] != -1)
            continue;
        if (comps_.size() >= max_comp)
            break; // 上限：剩下的按面积截断（Top-K 只要最亮的几块）

        Component c{};
        c.tx0 = c.tx1 = static_cast<uint16_t>(i % gw_);
        c.ty0 = c.ty1 = static_cast<uint16_t>(i / gw_);

        const int32_t cid = static_cast<int32_t>(comps_.size());
        bfs_.clear();
        bfs_.push_back(i);
        comp_id_[i] = cid;

        while (!bfs_.empty()) {
            const uint32_t cur = bfs_.back();
            bfs_.pop_back();
            const uint32_t cx = cur % gw_;
            const uint32_t cy = cur / gw_;

            c.area += count_[cur];
            c.sx += xsum_[cur];
            c.sy += ysum_[cur];
            ++c.tiles;
            if (cx < c.tx0) c.tx0 = static_cast<uint16_t>(cx);
            if (cx > c.tx1) c.tx1 = static_cast<uint16_t>(cx);
            if (cy < c.ty0) c.ty0 = static_cast<uint16_t>(cy);
            if (cy > c.ty1) c.ty1 = static_cast<uint16_t>(cy);

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0)
                        continue;
                    const int nx = static_cast<int>(cx) + dx;
                    const int ny = static_cast<int>(cy) + dy;
                    if (nx < 0 || ny < 0 || nx >= static_cast<int>(gw_) || ny >= static_cast<int>(gh_))
                        continue;
                    const uint32_t ni = static_cast<uint32_t>(ny) * gw_ + static_cast<uint32_t>(nx);
                    if (count_[ni] == 0 || comp_id_[ni] != -1)
                        continue;
                    comp_id_[ni] = cid;
                    bfs_.push_back(ni);
                }
            }
        }
        comps_.push_back(c);
    }
    return static_cast<uint32_t>(comps_.size());
}

// ---------------------------------------------------------------------------
// 精修：对**一个组件占用的瓦片**逐像素重扫，算出紧致包围盒与 fill。
// 面积/质心本来就已经精确（粗筛的矩），这里只是把包围盒从瓦片粒度收紧到像素粒度 ——
// fill = area/包围盒 必须用紧致包围盒才有意义（瓦片粒度的包围盒会让小目标 fill 假性很低）。
// ---------------------------------------------------------------------------
bool TileScanner::refine(const GrayFrame &f, uint32_t ci, const Component &c, Blip *out) const {
    // 像素预算：组件占用的瓦片面积。超过就放弃精修（整片过曝时保住时间预算）。
    const uint64_t tiles_px = static_cast<uint64_t>(c.tiles) * tile_ * tile_;
    if (tiles_px > cfg_.refine_max_px)
        return false;

    uint32_t x0 = 0xffffffffu, y0 = 0xffffffffu, x1 = 0, y1 = 0;
    uint32_t area = 0;
    uint64_t sx = 0, sy = 0;
    uint64_t sxx = 0, syy = 0, sxy = 0; // 二阶矩（算圆度：等效椭圆轴比 + 填充率）

    for (uint32_t ty = c.ty0; ty <= c.ty1; ++ty) {
        const uint32_t ya = ty * tile_;
        const uint32_t yb = min_u32(ya + tile_, f.height);
        for (uint32_t tx = c.tx0; tx <= c.tx1; ++tx) {
            if (comp_id_[ty * gw_ + tx] != static_cast<int32_t>(ci))
                continue; // 不属于本组件（含被别的组件占用的瓦片）
            const uint32_t xa = tx * tile_;
            const uint32_t xb = min_u32(xa + tile_, f.width);
            for (uint32_t y = ya; y < yb; ++y) {
                const uint8_t *row = f.pixels + static_cast<size_t>(y) * f.stride;
                for (uint32_t x = xa; x < xb; ++x) {
                    if (row[x] < kBrightMin)
                        continue;
                    ++area;
                    sx += x;
                    sy += y;
                    sxx += static_cast<uint64_t>(x) * x;
                    syy += static_cast<uint64_t>(y) * y;
                    sxy += static_cast<uint64_t>(x) * y;
                    if (x < x0) x0 = x;
                    if (x > x1) x1 = x;
                    if (y < y0) y0 = y;
                    if (y > y1) y1 = y;
                }
            }
        }
    }

    if (area == 0)
        return false; // 理论上不可能（组件面积>0），留着是为了不让 0 除进到下面
    out->cx = static_cast<float>(static_cast<double>(sx) / area);
    out->cy = static_cast<float>(static_cast<double>(sy) / area);
    out->area = area;
    out->radius = radius_from_area(static_cast<float>(area));
    out->x0 = x0;
    out->y0 = y0;
    out->x1 = x1;
    out->y1 = y1;
    const uint64_t bw = static_cast<uint64_t>(x1 - x0) + 1;
    const uint64_t bh = static_cast<uint64_t>(y1 - y0) + 1;
    out->fill = static_cast<float>(static_cast<double>(area) / static_cast<double>(bw * bh));
    out->circularity =
        roundness_from_moments(area, static_cast<double>(sx), static_cast<double>(sy),
                               static_cast<double>(sxx), static_cast<double>(syy),
                               static_cast<double>(sxy), static_cast<double>(bw), static_cast<double>(bh));
    out->score = static_cast<float>(area); // 排序键在 run() 里按 circ_weight 统一算
    // 贴边标记：面积是下界、质心偏，启动确认器据此跳过（见 Blip::border）
    out->border = (x0 == 0) || (y0 == 0) || (x1 + 1u >= f.width) || (y1 + 1u >= f.height);
    return true;
}

// ---------------------------------------------------------------------------
// 一帧全图扫描的主流程
// ---------------------------------------------------------------------------
size_t TileScanner::run(const GrayFrame &f, Blip *out, size_t cap, bool use_rvv) {
    trace_ = Trace{};
    trace_.rvv = use_rvv;
    if (out == nullptr || cap == 0 || f.pixels == nullptr || f.width == 0 || f.height == 0)
        return 0;
    if (f.stride < f.width)
        return 0; // stride 不合法：宁可返回空，也不越界读

    tile_ = cfg_.tile ? cfg_.tile : 16;
    ensure_grid(f.width, f.height);

    std::fill(count_.begin(), count_.end(), 0);
    std::fill(xsum_.begin(), xsum_.end(), 0);
    std::fill(ysum_.begin(), ysum_.end(), 0);
    std::fill(comp_id_.begin(), comp_id_.end(), -1);

#if defined(DART_HAVE_RVV)
    trace_.tiles = use_rvv ? tile_pass_rvv(f) : tile_pass_scalar(f);
#else
    (void)use_rvv;
    trace_.tiles = tile_pass_scalar(f);
#endif
    trace_.seed_tiles = trace_.tiles;

    const uint32_t ncomp = merge_components();
    trace_.components = ncomp;
    if (ncomp == 0)
        return 0;

    // 面积门限：太小 = 噪点/坏点；太大 = 整片过曝（不是"小目标"）
    const uint64_t frame_px = static_cast<uint64_t>(f.width) * f.height;
    const uint64_t max_area =
        frame_px * static_cast<uint64_t>(cfg_.max_area_frac_pct ? cfg_.max_area_frac_pct : 100) / 100u;
    const uint32_t min_area = cfg_.min_area ? cfg_.min_area : 1;

    order_.clear();
    for (uint32_t i = 0; i < ncomp; ++i) {
        const uint32_t a = comps_[i].area;
        if (a < min_area || a > max_area)
            continue;
        order_.push_back(i);
    }
    if (order_.empty())
        return 0;

    // 先按面积粗排，决定"值得精修哪几块"。为什么要多精修一些（top_k 的 3 倍）：
    // 圆度只有精修之后才知道，而"面积最大的几块"未必包含"最圆的那块" ——
    // 反光长条常常面积很大。精修有像素预算（refine_max_px/块）封顶，多修几块代价可控。
    std::sort(order_.begin(), order_.end(), [this](uint32_t a, uint32_t b) {
        if (comps_[a].area != comps_[b].area)
            return comps_[a].area > comps_[b].area;
        return a < b; // 面积相同时按瓦片顺序，保证结果可复现（自检要逐字段比对）
    });

    const size_t top_k = cfg_.top_k ? cfg_.top_k : 1;
    const size_t refine_n = std::min<size_t>(order_.size(), std::max<size_t>(top_k * 3u, 8u));
    size_t n = 0;
    for (size_t k = 0; k < refine_n && n < cap; ++k) {
        const uint32_t ci = order_[k];
        const Component &c = comps_[ci];
        Blip cand{};

        if (refine(f, ci, c, &cand)) {
            ++trace_.refined;
        } else {
            // 不精修：面积/质心仍然精确（来自瓦片矩），包围盒停在瓦片粒度
            cand.cx = static_cast<float>(static_cast<double>(c.sx) / static_cast<double>(c.area));
            cand.cy = static_cast<float>(static_cast<double>(c.sy) / static_cast<double>(c.area));
            cand.area = c.area;
            cand.radius = radius_from_area(static_cast<float>(c.area));
            cand.x0 = static_cast<uint32_t>(c.tx0) * tile_;
            cand.y0 = static_cast<uint32_t>(c.ty0) * tile_;
            cand.x1 = min_u32((static_cast<uint32_t>(c.tx1) + 1) * tile_, f.width) - 1;
            cand.y1 = min_u32((static_cast<uint32_t>(c.ty1) + 1) * tile_, f.height) - 1;
            const uint64_t bw = static_cast<uint64_t>(cand.x1 - cand.x0) + 1;
            const uint64_t bh = static_cast<uint64_t>(cand.y1 - cand.y0) + 1;
            cand.fill = static_cast<float>(static_cast<double>(c.area) / static_cast<double>(bw * bh));
            // 没精修（包围盒预算超了）就没有二阶矩：用"包围盒长宽比（当轴比）×
            // 归一化填充率"估计圆度，避免它在排序键里被当成圆度 0 而直接排到最后。
            // 这类块本来就又大又不像小目标，估计得粗一点无所谓。
            const float ar = static_cast<float>(bw) / static_cast<float>(bh > 0 ? bh : 1);
            const float fill_norm = cand.fill / 0.7853981633974483f < 1.0f
                                        ? cand.fill / 0.7853981633974483f
                                        : 1.0f;
            cand.circularity = (ar < 1.0f ? ar : 1.0f / ar) * fill_norm;
            cand.score = static_cast<float>(c.area);
            cand.border = (cand.x0 == 0) || (cand.y0 == 0) || (cand.x1 + 1u >= f.width) ||
                          (cand.y1 + 1u >= f.height);
        }
        // 形状门限（圆度只有精修后才有；没精修的大块用估计值）
        if (cfg_.min_fill > 0.0f && cand.fill < cfg_.min_fill)
            continue;
        if (cfg_.min_circularity > 0.0f && cand.circularity < cfg_.min_circularity)
            continue;

        cand.continuity = 0.0f;                          // 滑窗命中率由启动确认器填
        cand.score = shape_score(static_cast<float>(cand.area), cand.circularity, cfg_.circ_weight);
        out[n++] = cand;
    }

    // 排序键 = 面积 × 圆度^w（见 ScannerConfig::circ_weight）：发光体是圆斑，
    // 反光/拖影是长条 —— 纯按面积排会让"更长更亮的光带"压过真正又小又圆的目标。
    std::sort(out, out + n, [](const Blip &a, const Blip &b) {
        if (a.score != b.score)
            return a.score > b.score;
        if (a.cy != b.cy)
            return a.cy < b.cy; // 同分时按位置定序，保证可复现（自检要逐字段比对）
        return a.cx < b.cx;
    });
    if (n > top_k)
        n = top_k; // 精修了更多块，但对外只交最亮最圆的前 K 个
    return n;
}

size_t TileScanner::scan(const GrayFrame &frame, Blip *out, size_t cap) {
#if defined(DART_HAVE_RVV)
    return run(frame, out, cap, true);
#else
    return run(frame, out, cap, false);
#endif
}

// ---------------------------------------------------------------------------
// 开机自检：向量路径 vs 标量参考路径。
// 板端没有 RISC-V 模拟器，这是唯一能证明向量代码没写错的办法。
// 合成图刻意含四种情况：穿瓦片边界的大块、瓦片内的方块、小于 min_area 的噪点、
// 以及"两个不同大小的目标"（验证 Top-K 排序）。
// ---------------------------------------------------------------------------
namespace {

void fill_rect(std::vector<uint8_t> &px, uint32_t w, uint32_t h, uint32_t x0, uint32_t y0, uint32_t rw,
               uint32_t rh) {
    for (uint32_t y = y0; y < y0 + rh && y < h; ++y)
        for (uint32_t x = x0; x < x0 + rw && x < w; ++x)
            px[static_cast<size_t>(y) * w + x] = 255;
}

void fill_disk(std::vector<uint8_t> &px, uint32_t w, uint32_t h, int cx, int cy, int r) {
    for (int y = cy - r; y <= cy + r; ++y) {
        for (int x = cx - r; x <= cx + r; ++x) {
            if (x < 0 || y < 0 || x >= static_cast<int>(w) || y >= static_cast<int>(h))
                continue;
            const int dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy <= r * r)
                px[static_cast<size_t>(y) * w + x] = 255;
        }
    }
}

} // namespace

bool TileScanner::selftest() {
    const uint32_t W = 128, H = 64;
    std::vector<uint8_t> px(static_cast<size_t>(W) * H, 0);
    fill_disk(px, W, H, 40, 30, 7);   // 最大：跨瓦片边界
    fill_rect(px, W, H, 90, 18, 9, 9); // 次大：正方形
    fill_disk(px, W, H, 100, 50, 2);   // 第三
    px[static_cast<size_t>(3) * W + 5] = 255; // 单像素噪点（应被 min_area 丢掉）

    GrayFrame f{};
    f.pixels = px.data();
    f.width = W;
    f.height = H;
    f.stride = W;

    ScannerConfig cfg;
    cfg.min_area = 3;
    cfg.top_k = 4;

    TileScanner va(cfg), vs(cfg);
    Blip ca[8]{}, cs[8]{};
    const size_t na = va.run(f, ca, 8, true);
    const size_t ns = vs.run(f, cs, 8, false);
    if (na != ns)
        return false;

    for (size_t i = 0; i < na; ++i) {
        if (ca[i].area != cs[i].area)
            return false;
        if (ca[i].x0 != cs[i].x0 || ca[i].x1 != cs[i].x1 || ca[i].y0 != cs[i].y0 || ca[i].y1 != cs[i].y1)
            return false;
        const float dcx = ca[i].cx - cs[i].cx;
        const float dcy = ca[i].cy - cs[i].cy;
        if (dcx > 1e-3f || dcx < -1e-3f || dcy > 1e-3f || dcy < -1e-3f)
            return false;
        const float dr = ca[i].radius - cs[i].radius;
        if (dr > 1e-3f || dr < -1e-3f)
            return false;
        const float dc = ca[i].circularity - cs[i].circularity;
        if (dc > 1e-3f || dc < -1e-3f)
            return false; // 圆度由边界长度算，两条路径必须一致
    }
    if (na < 3)
        return false; // 三个真目标必须都出来（少于 3 说明面积门限/并块有错）
    if (ca[0].area <= ca[1].area || ca[1].area <= ca[2].area)
        return false; // Top-K 必须按最亮排序
    return true;
}

} // namespace dart::detection
