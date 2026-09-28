// ============================================================================
// RunLengthMeasurer 实现：窗口内"游程 + 并查集"连通域，再按形状筛选取最大块。
//
// 为什么不用"整窗掩码 + BFS"（C 版 detect_color 的写法）：那需要一块 w×h 的
// 中间掩码，而跟踪态的窗口最大可以到整幅的一半 —— 板端用户堆只有 16MB，
// 这块内存得省。游程法的中间态只有"每行的游程表"，正常 ROI 只有几十条。
// 像素访问次数与掩码法一致（每像素一次），并查集的开销在游程级别（更少）。
//
// 两个出口共用同一段连通域（build_blobs）：
//   measure() —— 绿灯那一路：形状筛后取"最好的一个"（跟踪层的测量）
//   collect() —— 装甲板那一路：**全部**块（两片灯条 + 可能的干扰），
//                面积下限由调用方给（它的门限随尺度变，塞不进静态配置）
// ============================================================================

#include "detection/measure/roi_measure.hpp"

#include <algorithm>

namespace dart::detection {

namespace {

constexpr uint8_t kBrightMin = 128;

// 8 邻域下两段游程相接：区间有重叠，或只隔 1 个像素（对角相邻）
inline bool runs_touch(uint32_t a0, uint32_t a1, uint32_t b0, uint32_t b1) {
    return !(b0 > a1 + 1u || a0 > b1 + 1u);
}

} // namespace

RunLengthMeasurer::RunLengthMeasurer(const MeasureConfig &cfg) : cfg_(cfg) {}

int32_t RunLengthMeasurer::find_root(int32_t i) {
    // 路径减半：游程数最多几千，递归/全路径压缩都不必要，减半已足够平坦
    while (parent_[static_cast<size_t>(i)] != i) {
        parent_[static_cast<size_t>(i)] = parent_[static_cast<size_t>(parent_[static_cast<size_t>(i)])];
        i = parent_[static_cast<size_t>(i)];
    }
    return i;
}

bool RunLengthMeasurer::build_blobs(const GrayFrame &f, const RoiWindow &w) {
    trace_ = Trace{};

    // 非法窗口/非法 stride：返回"没测到"。调用方（状态机）会把它当成一次 miss ——
    // 比越界读或者断言崩掉都好，板端跑起来时"少测一帧"是可以接受的代价。
    if (f.pixels == nullptr || w.empty() || f.stride < f.width || w.right() > f.width ||
        w.bottom() > f.height) {
        return false;
    }

    parent_.clear();
    rank_.clear();
    run_x0_.clear();
    run_x1_.clear();
    run_y_.clear();
    run_sxx_.clear();
    run_sxy_.clear();
    prev_row_.clear();
    cur_row_.clear();
    prev_id_.clear();
    cur_id_.clear();
    blobs_.clear();

    const size_t max_runs = cfg_.max_runs ? cfg_.max_runs : 16384;

    // ------------------------------------------------------------------
    // 第 1 步：逐行提取游程，并与上一行的游程做关联（两行都按 x 升序 → 双指针）
    // ------------------------------------------------------------------
    for (uint32_t ry = 0; ry < w.h; ++ry) {
        const uint8_t *row = f.pixels + static_cast<size_t>(w.y + ry) * f.stride;
        cur_row_.clear();
        cur_id_.clear();

        uint32_t       x = w.x;
        const uint32_t xend = w.x + w.w;
        while (x < xend) {
            if (row[x] < kBrightMin) {
                ++x;
                continue;
            }
            const uint32_t start = x;
            while (x < xend && row[x] >= kBrightMin)
                ++x;

            if (run_x0_.size() >= max_runs) { // 病态画面（半幅椒盐）：本帧按没测到处理
                trace_.overflow = true;
                return false;
            }
            const int32_t id = static_cast<int32_t>(run_x0_.size());
            const uint32_t len = x - start;
            parent_.push_back(id);
            rank_.push_back(0);
            run_x0_.push_back(static_cast<uint16_t>(start - w.x));
            run_x1_.push_back(static_cast<uint16_t>(x - 1u - w.x));
            run_y_.push_back(static_cast<uint16_t>(ry));
            // 闭式矩：Σx（等差数列）、Σx²（连续整数平方和 S2(n)=n(n+1)(2n+1)/6）、Σxy=y·Σx。
            // 用闭式而不是逐像素累加：游程可能很长，这里每段只做几次乘除。
            {
                const uint64_t xa = start - w.x;
                const uint64_t xb = x - 1u - w.x;
                const uint64_t sum_x = (xa + xb) * (uint64_t)len / 2u;
                const auto s2 = [](uint64_t n) { // 0²+1²+…+n²
                    return n * (n + 1u) * (2u * n + 1u) / 6u;
                };
                run_sxx_.push_back(s2(xb) - (xa > 0 ? s2(xa - 1u) : 0u));
                run_sxy_.push_back(sum_x * ry);
            }
            cur_row_.push_back(Run{static_cast<uint16_t>(start - w.x), static_cast<uint16_t>(x - 1u - w.x)});
            cur_id_.push_back(id);
        }

        if (!prev_row_.empty() && !cur_row_.empty()) {
            size_t a = 0; // 上一行游程指针
            for (size_t b = 0; b < cur_row_.size(); ++b) {
                while (a < prev_row_.size() &&
                       static_cast<uint32_t>(prev_row_[a].x1) + 1u < cur_row_[b].x0)
                    ++a; // 完全落在左边：跳过（下一段更靠右，不可能再关联上）
                for (size_t k = a; k < prev_row_.size(); ++k) {
                    if (static_cast<uint32_t>(prev_row_[k].x0) > static_cast<uint32_t>(cur_row_[b].x1) + 1u)
                        break; // 后面的只会更靠右
                    if (!runs_touch(prev_row_[k].x0, prev_row_[k].x1, cur_row_[b].x0, cur_row_[b].x1))
                        continue;
                    int32_t ra = find_root(prev_id_[k]);
                    int32_t rb = find_root(cur_id_[b]);
                    if (ra == rb)
                        continue;
                    if (rank_[static_cast<size_t>(ra)] < rank_[static_cast<size_t>(rb)])
                        std::swap(ra, rb);
                    parent_[static_cast<size_t>(rb)] = ra; // 按秩合并
                    if (rank_[static_cast<size_t>(ra)] == rank_[static_cast<size_t>(rb)])
                        ++rank_[static_cast<size_t>(ra)];
                }
            }
        }

        std::swap(prev_row_, cur_row_);
        std::swap(prev_id_, cur_id_);
    }

    const size_t nruns = run_x0_.size();
    trace_.runs = static_cast<uint32_t>(nruns);
    if (nruns == 0)
        return false; // 整窗全黑：常态（启动阶段背景就是全黑的）

    // ------------------------------------------------------------------
    // 第 2 步：按根结算每个组件的面积/矩/包围盒
    //   面积 = Σ 游程长度（精确）
    //   Σx  = Σ (x0+x1)·len/2（等差数列求和，不必逐像素；该乘积恒为偶数）
    //   Σy  = Σ len·y
    // ------------------------------------------------------------------
    root_blob_.assign(nruns, -1);
    for (size_t i = 0; i < nruns; ++i) {
        const int32_t r = find_root(static_cast<int32_t>(i));
        int32_t       bi = root_blob_[static_cast<size_t>(r)];
        if (bi < 0) {
            root_blob_[static_cast<size_t>(r)] = static_cast<int32_t>(blobs_.size());
            blobs_.push_back(Blob{});
            bi = static_cast<int32_t>(blobs_.size()) - 1;
        }
        Blob          &b = blobs_[static_cast<size_t>(bi)];
        const uint32_t len = static_cast<uint32_t>(run_x1_[i] - run_x0_[i]) + 1u;
        b.area += len;
        b.sxx += run_sxx_[i];
        b.syy += static_cast<uint64_t>(len) * run_y_[i] * run_y_[i];
        b.sxy += run_sxy_[i];
        b.sx += static_cast<uint64_t>(run_x0_[i] + run_x1_[i]) * len / 2u;
        b.sy += static_cast<uint64_t>(len) * run_y_[i];
        if (b.area == len) { // 该组件的第一个游程：初始化包围盒
            b.x0 = b.x1 = run_x0_[i];
            b.y0 = b.y1 = run_y_[i];
        } else {
            if (run_x0_[i] < b.x0) b.x0 = run_x0_[i];
            if (run_x1_[i] > b.x1) b.x1 = run_x1_[i];
            if (run_y_[i] < b.y0) b.y0 = run_y_[i];
            if (run_y_[i] > b.y1) b.y1 = run_y_[i];
        }
    }
    return !blobs_.empty();
}

TargetMeasurement RunLengthMeasurer::measure(const GrayFrame &f, const RoiWindow &w) {
    TargetMeasurement m{};
    if (!build_blobs(f, w))
        return m;

    // ------------------------------------------------------------------
    // 第 3 步：形状筛选后取最大块（与 detect_color 同口径）
    // ------------------------------------------------------------------
    const Blob  *best = nullptr;
    float        best_score = -1.0f;
    for (const Blob &b : blobs_) {
        if (b.area < cfg_.min_area)
            continue;
        ++trace_.blobs;
        const uint32_t bw = static_cast<uint32_t>(b.x1 - b.x0) + 1u;
        const uint32_t bh = static_cast<uint32_t>(b.y1 - b.y0) + 1u;
        const float    fill = static_cast<float>(b.area) / static_cast<float>(bw * bh);
        const float    aspect = static_cast<float>(bw) / static_cast<float>(bh);
        const float    circ = roundness_from_moments(
            b.area, static_cast<double>(b.sx), static_cast<double>(b.sy), static_cast<double>(b.sxx),
            static_cast<double>(b.syy), static_cast<double>(b.sxy),
            static_cast<double>(bw), static_cast<double>(bh));
        if (fill < cfg_.min_fill || aspect < cfg_.aspect_lo || aspect > cfg_.aspect_hi ||
            (cfg_.min_circularity > 0.0f && circ < cfg_.min_circularity)) {
            ++trace_.rejected;
            continue;
        }
        // 取"面积 × 圆度^w"最大者（不再单纯取最大块）：ROI 里若混进一条更长的反光，
        // 纯按面积会一直挑错目标；圆度加权后，圆斑才压得住长条。
        const float score = shape_score(static_cast<float>(b.area), circ, cfg_.circ_weight);
        if (score > best_score) {
            best_score = score;
            best = &b;
        }
    }
    if (best == nullptr)
        return m; // 窗口内没有合格块 → roi = NoTargetInRoi（默认值）

    const bool touch = (best->x0 == 0) || (best->y0 == 0) || (best->x1 == w.w - 1u) ||
                       (best->y1 == w.h - 1u);

    m.valid = true;
    m.cx = static_cast<float>(w.x) + static_cast<float>(static_cast<double>(best->sx) / best->area);
    m.cy = static_cast<float>(w.y) + static_cast<float>(static_cast<double>(best->sy) / best->area);
    m.area = best->area;
    m.radius = radius_from_area(static_cast<float>(best->area));
    m.fill = static_cast<float>(best->area) /
             static_cast<float>((static_cast<uint32_t>(best->x1 - best->x0) + 1u) *
                                (static_cast<uint32_t>(best->y1 - best->y0) + 1u));
    m.circularity = roundness_from_moments(
        best->area, static_cast<double>(best->sx), static_cast<double>(best->sy),
        static_cast<double>(best->sxx), static_cast<double>(best->syy), static_cast<double>(best->sxy),
        static_cast<double>(best->x1 - best->x0) + 1.0,
        static_cast<double>(best->y1 - best->y0) + 1.0);
    // 贴框 = 面积只是下界 → 降质量 → 放大 R_scale（让运动模型说话）
    m.quality = touch ? cfg_.clip_quality : 1.0f;
    m.roi = touch ? RoiState::TargetOutOfRoi : RoiState::Hit;
    return m;
}

uint32_t RunLengthMeasurer::collect(const GrayFrame &f, const RoiWindow &w, uint32_t min_area,
                                  BlobInfo *out, uint32_t cap) {
    if (out == nullptr || cap == 0)
        return 0;
    if (!build_blobs(f, w))
        return 0;

    uint32_t n = 0;
    for (const Blob &b : blobs_) {
        if (b.area < min_area) {
            ++trace_.rejected;
            continue;
        }
        ++trace_.blobs;
        if (n >= cap) { // 不截断静默丢弃：置 overflow，调用方知道本帧候选被砍过
            trace_.overflow = true;
            break;
        }
        const uint32_t bw = static_cast<uint32_t>(b.x1 - b.x0) + 1u;
        const uint32_t bh = static_cast<uint32_t>(b.y1 - b.y0) + 1u;
        BlobInfo      &o = out[n++];
        o.x0 = static_cast<uint16_t>(w.x + b.x0);
        o.y0 = static_cast<uint16_t>(w.y + b.y0);
        o.x1 = static_cast<uint16_t>(w.x + b.x1);
        o.y1 = static_cast<uint16_t>(w.y + b.y1);
        o.area = b.area;
        // 贴**窗口**边：包围盒/面积只是下界（可能被窗口削掉一截）。
        // 这是仓库在启动阶段踩过的坑（贴边块面积是下界、质心是偏的），
        // 装甲板这一路同样要吃它：被削的灯条长度不可信，配对时该被罚分。
        o.border = ((b.x0 == 0) || (b.y0 == 0) || (b.x1 == w.w - 1u) || (b.y1 == w.h - 1u)) ? 1u : 0u;
        o.circularity = roundness_from_moments(
            b.area, static_cast<double>(b.sx), static_cast<double>(b.sy), static_cast<double>(b.sxx),
            static_cast<double>(b.syy), static_cast<double>(b.sxy),
            static_cast<double>(bw), static_cast<double>(bh));
        o.fill = static_cast<float>(b.area) / static_cast<float>(bw * bh);
        // 主轴：斜灯条的长宽比/填充率会同时失效，判据必须建在主轴上（与旋转无关）
        principal_axis(static_cast<double>(b.area), static_cast<double>(b.sx), static_cast<double>(b.sy),
                       static_cast<double>(b.sxx), static_cast<double>(b.syy), static_cast<double>(b.sxy),
                       &o.theta, &o.len_major, &o.len_minor);
    }
    return n;
}

} // namespace dart::detection
