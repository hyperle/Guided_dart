#pragma once

// ============================================================================
// §0 这个文件里为什么有两个类（职责边界）
//
// 这两个类的名字只差一个字母（armer / armor），历史上是两个文件，很容易被误读成
// "同一个东西的旧版与新版本"。它们**不是**版本关系，职责与代码几乎不重合：
//
//   §A StartupArmer   —— 绿灯（工况 1）的**启动确认器**：吃全图候选序列，
//                        3 帧滑窗反向关联 + 三道门（命中数/位移平滑性/物理性），
//                        确认成功后把整条轨迹交给卡尔曼初始化。
//                        不认识像素、不认识窗口、不认识装甲板。
//                        使用者：TargetTracker（tracker.hpp）——它每帧全图扫描时喂候选。
//
//   §B ArmorDetector  —— **装甲板**（识别层第二路输出）：吃二值图 + 绿灯锚，
//                        在窗口内做连通域 → 灯条判据 → 配对 → 交叉连线求交，
//                        并自带 detect/track 两种路径。
//                        不认识候选序列表、不认识滑窗、不认识卡尔曼。
//                        使用者：DetectionPipeline（pipeline.hpp）。
//
// 为什么放在一个文件：名字撞车只需一个文件就不再有"哪个是最新的"这个问题；
// 但**两个类互不引用、不共享状态、不共享基类**（不是 is-a 关系，硬造父类只会
// 得到互相泄漏的抽象）。文件的物理组织 = 下面 §A / §B 两段，各自的注释、依赖、
// 测试都分开：§A 的实现在 armor.cpp，§B 全在头文件里（可直接拿去单测）。
//
// 合并的代价（知情选择）：本文件被 tracker.hpp 间接包含，于是绿灯那条链编译时
// 也会连带编译 blob_measure（§B 需要它的 BlobInfo）。运行期没有任何交叉。
// ============================================================================

#include <cmath>
#include <cstdint>

#include "core/frame.hpp"
#include "detection/blob_measure.hpp"
#include "detection/config.hpp"
#include "detection/types.hpp"

namespace dart::detection {

// ============================================================================
// §A StartupArmer —— 绿灯启动确认器（实现在 armor.cpp）
// ============================================================================

// 每帧最多缓存多少个候选参与关联（扫描器 Top-K 上限 64，但滑窗里没必要全留）
constexpr uint32_t kArmMaxPerFrame = 16;

class StartupArmer {
public:
    struct Confirmation {
        bool           ok = false;
        LightCandidate cand{};   // 确认后的目标（位置/尺度取最新一帧，continuity 已填）
        uint32_t       hits = 0; // 滑窗内关联上的帧数
        float          travel_px = 0.0f;    // 首末帧直线距离
        float          max_accel_px = 0.0f; // 帧间位移差的最大模（平滑性指标）
        float          radius_rate = 0.0f;  // 尺度变化率（px/s）

        // 整条轨迹（时间升序：旧 → 新），交给 ScaleAwareKalman::init_from_track
        LightCandidate chain[kArmMaxWindow]{};
        uint64_t       times_us[kArmMaxWindow]{};
        size_t         n = 0;
    };

    explicit StartupArmer(const ArmConfig &cfg);

    void reset();

    // 推入本帧全图候选（按 score 降序最好，内部会再排一次保证确定性），
    // 返回本帧的确认结果。窗口没填满时 ok 恒为 false。
    Confirmation push(uint64_t mono_us, const LightCandidate *cands, size_t n);

    uint32_t window() const { return cfg_.window; }

    // 本帧被否掉的候选个数（≈ 随机闪烁坏点计数，写进状态行做取证）
    uint32_t last_reject() const { return last_reject_; }
    // 本帧被跳过的**贴画面边**候选个数（与"拒闪"分开统计：那是被边界切掉的块，
    // 不是闪烁坏点。两者混在一起会让状态行误导调参 —— round12 的教训）
    uint32_t last_border_skip() const { return last_border_skip_; }
    // 本帧是否做过确认尝试（窗口填满后才是 true）
    bool     ready() const { return filled_ >= cfg_.window; }

private:
    struct Slot {
        uint64_t       mono_us = 0;
        uint32_t       n = 0;
        LightCandidate cands[kArmMaxPerFrame]{};
    };

    const Slot &at_lag(uint32_t lag) const; // lag=0 最新

    ArmConfig cfg_;
    Slot      ring_[kArmMaxWindow];
    uint32_t  head_ = 0;   // 最新一帧所在下标
    uint32_t  filled_ = 0; // 已填入的帧数（<= window）
    uint32_t  last_reject_ = 0;
    uint32_t  last_border_skip_ = 0;
};


// ============================================================================
// §B ArmorDetector —— 装甲板识别（全在这一个头文件里，可直接单测）
// ============================================================================

class ArmorDetector {
public:
    explicit ArmorDetector(const ArmorConfig &cfg, const MeasureConfig &meas_cfg)
        : cfg_(cfg), meas_(meas_cfg) {}

    void reset() {
        a_ = Bar{};
        b_ = Bar{};
        have_pair_ = false;
        age_ = 0;
        scale_s_ = 0;
        ds_ = 0.0f;
        r_last_ = 0.0f;
        trace_ = Trace{};
    }

    // 一帧一拍。bin 与绿灯那一路是**同一张**二值图（所以装甲板天然是"黑白口径"）；
    // led 是本帧跟踪层的输出（found=false 时不出结果 —— 绿灯是窗口的锚，没锚不猜）。
    // tracker_us 为空时不自计时（主机测试注入假钟即可完全确定性）。
    void detect(const GrayFrame &bin, const TrackOutput &led, uint64_t now_us,
                ArmorTarget *out) {
        trace_ = Trace{};
        *out = ArmorTarget{};
        const uint64_t t0 = now_us;

        // 1) 锚：跟踪层说"本帧有目标"才动手。它已经含了确认/门限/丢帧语义，
        //    这里不再自己判"锁没锁"（那是 TargetTracker 的职责）。
        if (!led.found || led.radius <= 0.0f)
            return;
        trace_.anchored = true;

        const int32_t s = static_cast<int32_t>(2.0f * led.radius + 0.5f); // 灯尺
        update_scale(s);
        trace_.scale = s;
        trace_.far = (static_cast<float>(s) < cfg_.tiny_px);

        // 2) 本帧扫哪里：
        //    · 首次/重捕 → "绿灯上方整块"
        //    · 跟踪中且绿灯够大 → **每条灯条各一个小窗**（只需盖住它自己，与板跨度无关）
        //    · 跟踪中但目标还小 → 一个合窗
        RoiWindow full{};
        if (!led_window(bin, led, &full))
            return;
        RoiWindow         wins[2];
        const uint32_t    nwin = search_windows(full, led, bin, wins);
        trace_.windows = static_cast<uint8_t>(nwin);

        const uint32_t pxmin = bar_px_min(s);

        // 3) **track 优先**：已有跟踪状态 → 只"认亲"，一条形状判据都不做。
        //    判定"还是同一个目标"靠的就是"在它自己的窗里连续变化"：
        //    位置容差挡住跳到别处、尺寸容差挡住换了个大小的东西。
        //    （这是 detect-track 的分工：门限只在 detect 需要，track 只认。）
        if (have_pair_ && track_step(bin, wins, nwin, led, pxmin, out)) {
            trace_.cost_us = static_cast<uint32_t>(now_us - t0); // track 路径也要记耗时
            return;
        }

        // 4) detect：全门限（as_bar 长度/长宽比/填充率/圆度）+ O(n²) 配对 + 刚性先验
        //    （走到这里说明：没有跟踪状态，或者 track 认亲失败且保持到期）
        trace_.window_px = 0;
        trace_.blobs = 0;
        trace_.rejected = 0;
        uint32_t nbar = 0, nblob_total = 0, px_total = 0;
        for (uint32_t k = 0; k < nwin; ++k) {
            px_total += static_cast<uint32_t>(wins[k].pixels());
            const uint32_t nblob = meas_.collect(bin, wins[k], pxmin, blobs_, kArmorMaxBars);
            nblob_total += nblob;
            for (uint32_t i = 0; i < nblob && nbar < kArmorMaxBars; ++i) {
                Bar bar;
                if (!as_bar(blobs_[i], s, &bar)) {
                    ++trace_.rejected;
                    continue;
                }
                // 两个窗万一重叠，同一条灯条会被带出来两次 → 按包围盒去重
                // （不去重也不会配错：两条"同一条灯条"的间距为 0，会被配对门限挡掉，
                //   但白算一遍不划算，而且会把候选数灌水）
                bool dup = false;
                for (uint32_t j = 0; j < nbar; ++j)
                    if (bars_[j].x == bar.x && bars_[j].y == bar.y && bars_[j].w == bar.w &&
                        bars_[j].h == bar.h)
                        dup = true;
                if (!dup)
                    bars_[nbar++] = bar;
            }
        }
        trace_.blobs = nblob_total;
        trace_.bars = nbar;
        trace_.window_px = px_total;

        // 4) 检测-跟踪
        const bool ok = step(bars_, nbar, led, s, out);
        trace_.pairs = pair_count_;
        trace_.rejected_prior = prior_reject_;
        trace_.cost_us = static_cast<uint32_t>(now_us - t0);
        (void)ok;
        fill_window(out, wins, nwin);
    }

    // 板端调参用（对齐 DetectionStats / LightScanner::Trace 的口径）
    struct Trace {
        bool     anchored = false; // 本帧有绿灯锚
        bool     far = false;      // 处在远档（形状判据换成圆度）
        int32_t  scale = 0;        // 灯尺 s = 2r
        uint32_t blobs = 0;        // 窗口内候选块数
        uint32_t bars = 0;         // 过灯条判据的块数
        uint32_t pairs = 0;        // 通过配对门限的灯条对数
        uint32_t rejected = 0;     // 被灯条判据否掉的块数
        uint32_t rejected_prior = 0; // 被刚性几何先验否掉的对数
        uint32_t window_px = 0;    // 本帧实际扫的**合计**窗口像素数（ROI 效率：单条窗是 2 个之和）
        uint8_t  windows = 0;      // 本帧开了几个窗（1 = 合窗，2 = 每条灯条各一个）
        uint8_t  mode = 0;         // 0 = detect（全门限；首次/重捕），1 = track（只认亲）
        uint8_t  assoc_fail = 0;   // 1 = 本帧认亲失败（没认到 / 跑出锚的够得着范围）
        uint32_t cost_us = 0;
    };
    const Trace &last_trace() const { return trace_; }

    // ---- 以下纯函数：不碰图像，主机侧可逐条单测（tests/host/armor_host_test.cpp）----

    // 灯条（画面绝对坐标；px = 像素数；border = 贴窗口边 → 长度只是下界）
    //
    // len/thick/theta 来自**二阶矩主轴**（types.hpp::principal_axis），不是包围盒：
    // 一条 5x40 的灯条转 30°，包围盒是 24x37（长宽比 1.5、填充 0.22），
    // 而主轴给出 (≈40, ≈5, 30°) —— 判据与端点都必须建在主轴上，才能与旋转无关。
    struct Bar {
        int16_t  x = 0, y = 0, w = 0, h = 0;
        uint32_t px = 0;
        uint8_t  border = 0;
        float    len = 0.0f;    // 主轴长度（真实灯条长度）
        float    thick = 0.0f;  // 次轴长度（真实厚度）
        float    theta = 0.0f;  // 主轴方向（弧度，从 +x 轴起算，[-π/2, π/2]）
        int32_t  bbox_len() const { return w > h ? w : h; }   // 包围盒长边（只用于"远档"判断与打印）
        int32_t  cx() const { return x + w / 2; }
        int32_t  cy() const { return y + h / 2; }
        // 主轴与"竖直/水平"中较近者的夹角 —— **单位：度**，与 ArmorConfig 里的角度一致。
        // （踩过：这里先返回了弧度，于是 15° 的 0.26 被当成"0.26 度" < flat_deg=12，
        //   填充率门限又生效了，斜灯条照样被判死。角度一律用度，别混。）
        float axis_skew_deg() const {
            const float a = (theta < 0.0f ? -theta : theta) * 57.29577951308232f; // 0..90
            const float to_vert = 90.0f - a;
            return to_vert < a ? to_vert : a;
        }
    };

    // 灯条长轴的两个端点 = "上角点 / 下角点"。
    //
    // 正放（axis_skew <= flat_deg）时用包围盒边的中点 —— 与改造前**逐位一致**，
    // 竖直/水平这个已经跑通的工况零回归。
    // 斜的时候必须沿**主轴**取：包围盒上边中点根本不在灯条轴上（偏出半个包围盒宽），
    // 四条"角点"就围不成那块四边形，板心会整体偏、而且逐帧抖。
    // （轴方向 mod π 没有歧义：两个端点交换不影响两条对角线的交点。）
    static void bar_ends(const Bar &b, float flat_deg, int32_t *x0, int32_t *y0, int32_t *x1,
                         int32_t *y1) {
        if (b.axis_skew_deg() <= flat_deg || !(b.len > 0.0f)) {
            if (b.h >= b.w) {
                *x0 = b.cx(); *y0 = b.y;
                *x1 = b.cx(); *y1 = b.y + b.h - 1;
            } else {
                *x0 = b.x;      *y0 = b.cy();
                *x1 = b.x + b.w - 1; *y1 = b.cy();
            }
            return;
        }
        const float c = std::cos(b.theta), sn = std::sin(b.theta);
        const float half = b.len * 0.5f;
        *x0 = b.cx() + static_cast<int32_t>(lroundf(c * half));
        *y0 = b.cy() + static_cast<int32_t>(lroundf(sn * half));
        *x1 = b.cx() - static_cast<int32_t>(lroundf(c * half));
        *y1 = b.cy() - static_cast<int32_t>(lroundf(sn * half));
    }

    // 两条线段的交点：不平行、且交点同时落在两条线段之内才返回 true。
    // 全整数：t/u 是有符号整数，只有最后求坐标做一次四舍五入除法 ——
    // "交点在不在段内"因此是精确比较，不会因浮点误差把边界情形判反
    // （角点几乎重合恰好是噪声最大的地方）。
    static bool line_intersection(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                                  int32_t x2, int32_t y2, int32_t x3, int32_t y3,
                                  float min_sin, int32_t *ox, int32_t *oy) {
        const int64_t ax = static_cast<int64_t>(x1) - x0, ay = static_cast<int64_t>(y1) - y0;
        const int64_t bx = static_cast<int64_t>(x3) - x2, by = static_cast<int64_t>(y3) - y2;
        const int64_t d = ax * by - ay * bx;
        if (d == 0)
            return false;                       // 平行或共线：没有唯一交点

        // 夹角门限：|sin| = |d| / (|a|·|b|)，用整数百分比比。
        // 为什么不开方后用平方比较：d 最大 ~3.4e7（画幅边长量级），10000·d² ≈ 1.1e19
        // 会溢出 int64。两次 sqrt（double 在 3.4e7 内是精确的）代价可忽略。
        const double la = std::sqrt(static_cast<double>(ax * ax + ay * ay));
        const double lb = std::sqrt(static_cast<double>(bx * bx + by * by));
        if (la <= 0.0 || lb <= 0.0)
            return false;
        const double sin_abs = static_cast<double>(d < 0 ? -d : d) / (la * lb);
        if (sin_abs < static_cast<double>(min_sin))
            return false;                       // 近平行：交点对 1px 抖动极敏感，宁可不报

        int64_t tn = (static_cast<int64_t>(x2) - x0) * by - (static_cast<int64_t>(y2) - y0) * bx;
        int64_t un = (static_cast<int64_t>(x2) - x0) * ay - (static_cast<int64_t>(y2) - y0) * ax;
        int64_t dd = d;
        if (dd < 0) {
            dd = -dd;
            tn = -tn;
            un = -un;
        }
        if (tn < 0 || tn > dd || un < 0 || un > dd)
            return false;                       // 交点在延长线上 → 不是这个四边形的中心
        *ox = x0 + static_cast<int32_t>(rdiv(ax * tn, dd));
        *oy = y0 + static_cast<int32_t>(rdiv(ay * tn, dd));
        return true;
    }

    // 板心 = A上—B下 与 A下—B上 两条连线的交点。
    // **注意是交叉连线**：A 的长轴与 B 的长轴永远平行，拿它们求交没有交点（写错过）。
    static bool armor_center(const Bar &a, const Bar &b, float min_sin, float flat_deg,
                             int32_t *cx, int32_t *cy) {
        int32_t a0x, a0y, a1x, a1y, b0x, b0y, b1x, b1y;
        bar_ends(a, flat_deg, &a0x, &a0y, &a1x, &a1y);
        bar_ends(b, flat_deg, &b0x, &b0y, &b1x, &b1y);
        return line_intersection(a0x, a0y, b1x, b1y, a1x, a1y, b0x, b0y, min_sin, cx, cy);
    }

    // 板心与绿灯是不是同一个整体：径向距离 < near_k × 半径（平方比较，省开方）
    static bool same_body(const ArmorConfig &cfg, const TrackOutput &led, int32_t cx, int32_t cy) {
        const float r = led.radius;
        if (!(r > 0.0f))
            return true; // 半径没建立：没有可比的基准 → 放行
        const float dx = static_cast<float>(cx) - led.cx;
        const float dy = static_cast<float>(cy) - led.cy;
        float       lim = cfg.near_k * r;
        if (lim < cfg.near_min_px)
            lim = cfg.near_min_px; // 小半径下按"半径倍数"算会被量化噪声判死（见 ArmorConfig）
        return dx * dx + dy * dy <= lim * lim;
    }

    // 两条灯条中心在"垂直于长轴"方向上的距离 ∝ 灯尺（板上间距是固定的）
    static int32_t pair_span(const Bar &a, const Bar &b) {
        return a.h >= a.w ? abs_i(a.cx() - b.cx()) : abs_i(a.cy() - b.cy());
    }

    // 门限：灯条长度 = 灯尺的倍数 + 硬地板；像素下限由"最短长度"推出（不另设系数）。
    // s_lo/s_hi 是尺度包络（min/max(s, s_pred)）：远处目标长得快，单点尺度会把
    // 这一帧的目标判掉，所以下界取小的、上界取大的。
    void gate(int32_t s_lo, int32_t s_hi, int32_t *len_min, int32_t *len_max,
              int32_t *px_min) const {
        int32_t lo = static_cast<int32_t>(cfg_.len_min_k * static_cast<float>(s_lo));
        int32_t hi = static_cast<int32_t>(cfg_.len_max_k * static_cast<float>(s_hi));
        if (lo < static_cast<int32_t>(cfg_.len_floor))
            lo = static_cast<int32_t>(cfg_.len_floor);
        if (hi < lo)
            hi = lo;
        int32_t px = lo / 2;
        if (px < static_cast<int32_t>(cfg_.px_floor))
            px = static_cast<int32_t>(cfg_.px_floor);
        *len_min = lo;
        *len_max = hi;
        *px_min = px;
    }

private:
    static int32_t abs_i(int32_t v) { return v < 0 ? -v : v; }
    static int64_t rdiv(int64_t num, int64_t den) { // 四舍五入整除（den > 0）
        return (2 * num + den) / (2 * den);
    }

    // ---- 尺度：线性一步外推（与仓库"目标均匀变大"同口径）----
    void update_scale(int32_t s) {
        if (scale_s_ > 0) {
            const float d = static_cast<float>(s - scale_s_);
            ds_ = 0.5f * ds_ + 0.5f * d;
        }
        scale_s_ = s;
    }
    void scale_band(int32_t *s_lo, int32_t *s_hi) const {
        if (scale_s_ <= 0) {
            *s_lo = 0;
            *s_hi = 0;
            return;
        }
        float v = cfg_.scale_v_k * ds_;
        if (v > cfg_.scale_v_max)
            v = cfg_.scale_v_max;
        else if (v < -cfg_.scale_v_max)
            v = -cfg_.scale_v_max;
        const int32_t sp = scale_s_ + static_cast<int32_t>(v);
        *s_lo = sp < scale_s_ ? (sp > 0 ? sp : 0) : scale_s_;
        *s_hi = sp > scale_s_ ? sp : scale_s_;
    }

    int32_t bar_px_min(int32_t s) const {
        int32_t a, b, px;
        gate(s, s, &a, &b, &px);
        return px;
    }

    // ---- 灯条判据（尺度自适应）----
    // 近档（长边 ≥ tiny_px）：长宽比 + 填充率可信，照用。
    // 远档：几个像素的"长宽比"是量化噪声 → 改判与尺度无关的圆度（灯条不像圆）。
    // 只填字段，不做任何判据（track 阶段的"认亲"用它：位置 + 尺寸容差就是全部判据）
    static void fill_bar(const RoiBlobMeasurer::BlobInfo &b, Bar *out) {
        const int32_t w = static_cast<int32_t>(b.x1) - b.x0 + 1;
        const int32_t h = static_cast<int32_t>(b.y1) - b.y0 + 1;
        out->x = b.x0;
        out->y = b.y0;
        out->w = static_cast<int16_t>(w);
        out->h = static_cast<int16_t>(h);
        out->px = b.area;
        out->border = b.border;
        out->theta = b.theta;
        if (b.len_major > 0.0f && b.len_minor > 0.0f) {
            out->len = b.len_major;
            out->thick = b.len_minor;
        } else {
            out->len = static_cast<float>(w > h ? w : h);
            out->thick = static_cast<float>(w > h ? h : w);
            out->theta = (h >= w) ? 1.5707963267948966f : 0.0f;
        }
    }

    // detect 阶段的形状判据（track 阶段**不进这里**）
    bool as_bar(const RoiBlobMeasurer::BlobInfo &b, int32_t s, Bar *out) const {
        const int32_t w = static_cast<int32_t>(b.x1) - b.x0 + 1;
        const int32_t h = static_cast<int32_t>(b.y1) - b.y0 + 1;
        int32_t       lmin, lmax, pxmin;
        gate(s, s, &lmin, &lmax, &pxmin);
        if (b.area < static_cast<uint32_t>(pxmin) || w <= 0 || h <= 0)
            return false;
        out->x = b.x0;
        out->y = b.y0;
        out->w = static_cast<int16_t>(w);
        out->h = static_cast<int16_t>(h);
        out->px = b.area;
        out->border = b.border;
        out->theta = b.theta;
        // 主轴可用就用主轴（与旋转无关）；退化（1~2px 或病态矩）时退回包围盒口径
        if (b.len_major > 0.0f && b.len_minor > 0.0f) {
            out->len = b.len_major;
            out->thick = b.len_minor;
        } else {
            out->len = static_cast<float>(w > h ? w : h);
            out->thick = static_cast<float>(w > h ? h : w);
            out->theta = (h >= w) ? 1.5707963267948966f : 0.0f;
        }
        if (out->len < static_cast<float>(lmin) || out->len > static_cast<float>(lmax))
            return false;
        if (static_cast<float>(out->bbox_len()) >= cfg_.tiny_px) {
            // 近档：长宽比用**主轴**长短边（斜灯条也成立）
            if (out->len < cfg_.aspect_min * out->thick)
                return false;
            // 填充率只在"基本正放"时才是可信的：斜的时候包围盒被撑大，fill 天然变低
            // （30° 时 0.22），拿它筛等于筛掉真灯条。
            if (out->axis_skew_deg() <= cfg_.flat_deg && b.fill < cfg_.fill_min)
                return false;
        } else if (b.area > 1u && b.circularity > cfg_.circ_max_far) {
            return false; // 远档：太圆 → 不是细长灯条（这条与尺度无关，3px 也成立）
        }
        return true;
    }

    // ---- 配对：两条灯条像不像同一块装甲板（全部是必要条件）----
    // ① 同向 ② 长度比 ③ 沿长轴投影重叠（并排，不是首尾相接）
    // ④ 中心距 <= gap_max_k·长边，且横向不重叠
    // 分数 = 长度差百分比 + 沿长轴错位百分比 + 贴边罚分（被窗口削过的长度不可信）
    bool pair_score(const Bar &a, const Bar &b, int32_t *score) const {
        const bool a_vert = a.h >= a.w;
        if (a_vert != (b.h >= b.w))
            return false;               // 偏竖/偏横不能配（先粗筛，省掉下面的三角计算）

        // **倾角要一致**：这就是"两条灯条平行"这条已知事实的用法。只比"都竖"不够 ——
        // 两条都歪着、但歪的角度差很多，那不是一块板。轴是直线(mod π)，所以比 |cos Δθ|。
        const float dtheta = a.theta - b.theta;
        const float par = std::fabs(std::cos(dtheta));                 // |cos Δθ|（轴 mod π）
        if (par < std::cos(cfg_.parallel_deg * 0.017453292519943295f))  // parallel_deg 是度
            return false;

        // **两条一样粗**：同一块板上两条灯条厚度相同 → 厚度比卡一道。它能挡掉
        // "灯条上糊了一块亮斑"这种被撑变形的块（厚度被撑大，主轴分解一眼看出来）。
        {
            const float tmax = a.thick > b.thick ? a.thick : b.thick;
            const float tmin = a.thick > b.thick ? b.thick : a.thick;
            if (tmax > 0.0f && tmin < cfg_.thick_ratio * tmax)
                return false;
        }
        int32_t la, lb, sep, half, lo, hi, off;
        if (a_vert) {
            la = static_cast<int32_t>(a.len + 0.5f); lb = static_cast<int32_t>(b.len + 0.5f);
            sep = abs_i(a.cx() - b.cx());
            half = a.w + b.w;
            lo = a.y > b.y ? a.y : b.y;
            const int32_t hia = a.y + a.h, hib = b.y + b.h;
            hi = hia < hib ? hia : hib;
            off = abs_i(a.cy() - b.cy());
        } else {
            la = static_cast<int32_t>(a.len + 0.5f); lb = static_cast<int32_t>(b.len + 0.5f);
            sep = abs_i(a.cy() - b.cy());
            half = a.h + b.h;
            lo = a.x > b.x ? a.x : b.x;
            const int32_t hia = a.x + a.w, hib = b.x + b.w;
            hi = hia < hib ? hia : hib;
            off = abs_i(a.cx() - b.cx());
        }
        const int32_t lmax = la > lb ? la : lb;
        const int32_t lmin = la > lb ? lb : la;
        if (lmax <= 0)
            return false;
        if (static_cast<float>(lmin) < cfg_.len_ratio * static_cast<float>(lmax))
            return false;
        if (static_cast<float>(hi - lo) < cfg_.overlap * static_cast<float>(lmin))
            return false;
        if (sep * 2 <= half)
            return false; // 横向重叠 → 在二值图上本来就是同一块
        if (static_cast<float>(sep) > cfg_.gap_max_k * static_cast<float>(lmax))
            return false;
        *score = (lmax - lmin) * 100 / lmax + off * 100 / lmax +
                 cfg_.border_penalty * (a.border + b.border);
        return true;
    }

    // ---- 刚性几何先验：板心相对绿灯的位置、两灯条间距必须是灯尺的固定倍数 ----
    // 前提是"绿灯与装甲板固定在同一块板上"（已确认）。它**与距离无关**，
    // 所以远档形状判据全不可信时，这是唯一还在工作的判据。
    bool prior_ok(const TrackOutput &led, int32_t s, const Bar &a, const Bar &b,
                  int32_t cx, int32_t cy) const {
        // ① 是不是**同一个整体**：板心到灯心的径向距离 < near_k × 绿灯半径。
        //    用径向（不是矩形框）：距离与倾角无关，板斜着转也不会把真目标切掉。
        if (!same_body(cfg_, led, cx, cy))
            return false;
        if (s <= 0)
            return true; // 灯尺没建立：只剩①可比
        // ② 板的**尺寸**对不对：两灯条间距 ∝ 灯尺（板上间距是固定的）
        const float fs = static_cast<float>(s);
        const float span = static_cast<float>(pair_span(a, b));
        if (span < cfg_.span_lo_k * fs || span > cfg_.span_hi_k * fs)
            return false;
        return true;
    }

    // ---- 窗口：绿灯正上方那块（尺寸随灯尺；下沿贴绿灯上沿）----
    bool led_window(const GrayFrame &f, const TrackOutput &led, RoiWindow *win) const {
        const float r = led.radius;
        float       rw = cfg_.roi_w_k * 2.0f * r;
        float       rh = cfg_.roi_h_k * 2.0f * r;
        const float gap = cfg_.roi_gap_k * 2.0f * r;
        if (rw < cfg_.roi_min_side)
            rw = cfg_.roi_min_side;
        if (rh < cfg_.roi_min_side)
            rh = cfg_.roi_min_side;
        const float top = led.cy - r; // 绿灯上沿
        float       x = led.cx - rw * 0.5f;
        float       y = top - gap - rh;
        return clamp_window(x, y, rw, rh, f.width, f.height, cfg_.roi_min_side, win);
    }

    // margin 统一口径：**按单条灯条长度**给，并封顶。
    // 原来按"整个对的尺寸 × 0.8"给：近距离一个窗就十几万像素（128880 vs 35280）。
    int32_t bar_margin(float bar_len_px, int32_t pad) const {
        float m = cfg_.bar_margin_k * bar_len_px;
        if (m < cfg_.bar_margin_min)
            m = cfg_.bar_margin_min;
        else if (m > cfg_.bar_margin_max)
            m = cfg_.bar_margin_max;
        return static_cast<int32_t>(m) + pad;
    }

    // 尺度增长补偿：工况是**逐渐接近**，30fps 下目标每帧能长 10~35%。
    // 窗口是按"上次采到的那条灯条"开的，不补偿的话这一帧目标就顶出窗外 → 被切。
    // 实测（每帧 ×1.35 的接近序列）：量到的灯条长度只有真值的 0.58；补偿后 0.86。
    int32_t grow_pad(const TrackOutput &led, float bar_len_px) const {
        if (r_last_ <= 0.0f || !(led.radius > r_last_))
            return 0;
        return static_cast<int32_t>(bar_len_px * (led.radius / r_last_ - 1.0f) * 0.5f);
    }

    // 锚的"够得着"范围：[灯心 ± near_k·r] 的外接方框。
    // 单条窗**不再**被"绿灯上方整块"裁剪（那正是某些角度包不住装甲板的原因），
    // 安全性改由这条径向范围提供：窗口可以跟着灯条走，但不许跑出这个范围。
    bool reach_box(const TrackOutput &led, const GrayFrame &f, RoiWindow *out) const {
        const float half = cfg_.near_k * led.radius;
        return clamp_window(led.cx - half, led.cy - half, half * 2.0f, half * 2.0f, f.width,
                            f.height, 4.0f, out);
    }

    // 一条灯条自己的小窗：只需盖住它自己 → 板转多少度都不影响这个窗口够不够
    bool bar_window(const Bar &bar, const TrackOutput &led, const RoiWindow &reach,
                    const GrayFrame &f, RoiWindow *out) const {
        const int32_t m = bar_margin(bar.len, grow_pad(led, bar.len));
        const int32_t x = bar.x - m, y = bar.y - m;
        const int32_t w = bar.w + 2 * m, h = bar.h + 2 * m;
        // 与"够得着"范围取交（都是闭区间外的开区间表示：x..x+w-1）
        int32_t rx = x, ry = y, rw = w, rh = h;
        if (rx < static_cast<int32_t>(reach.x)) {
            rw -= (static_cast<int32_t>(reach.x) - rx);
            rx = static_cast<int32_t>(reach.x);
        }
        if (ry < static_cast<int32_t>(reach.y)) {
            rh -= (static_cast<int32_t>(reach.y) - ry);
            ry = static_cast<int32_t>(reach.y);
        }
        const int32_t rx1 = static_cast<int32_t>(reach.right());
        const int32_t ry1 = static_cast<int32_t>(reach.bottom());
        if (rx + rw > rx1)
            rw = rx1 - rx;
        if (ry + rh > ry1)
            rh = ry1 - ry;
        if (rw < 8 || rh < 8)
            return false;
        return clamp_window(static_cast<float>(rx), static_cast<float>(ry),
                            static_cast<float>(rw), static_cast<float>(rh), f.width, f.height,
                            8.0f, out);
    }

    // 合窗（远距离 / 首次 / 单条窗没建起来）：一个窗罩住上一对灯条，与整块取交。
    RoiWindow union_window(const RoiWindow &full, const TrackOutput &led) const {
        if (!have_pair_)
            return full;
        const int32_t x0 = a_.x < b_.x ? a_.x : b_.x;
        const int32_t y0 = a_.y < b_.y ? a_.y : b_.y;
        const int32_t ax1 = a_.x + a_.w, bx1 = b_.x + b_.w;
        const int32_t ay1 = a_.y + a_.h, by1 = b_.y + b_.h;
        const int32_t x1 = ax1 > bx1 ? ax1 : bx1;
        const int32_t y1 = ay1 > by1 ? ay1 : by1;
        const float   side = a_.len > b_.len ? a_.len : b_.len;
        const int32_t m = bar_margin(side, grow_pad(led, side));
        int32_t       sx = x0 - m, sy = y0 - m;
        int32_t       sw = (x1 - x0) + 2 * m, sh = (y1 - y0) + 2 * m;
        if (sx < static_cast<int32_t>(full.x)) {
            sw -= (static_cast<int32_t>(full.x) - sx);
            sx = static_cast<int32_t>(full.x);
        }
        if (sy < static_cast<int32_t>(full.y)) {
            sh -= (static_cast<int32_t>(full.y) - sy);
            sy = static_cast<int32_t>(full.y);
        }
        const int32_t fx1 = static_cast<int32_t>(full.right());
        const int32_t fy1 = static_cast<int32_t>(full.bottom());
        if (sx + sw > fx1)
            sw = fx1 - sx;
        if (sy + sh > fy1)
            sh = fy1 - sy;
        if (sw < 8 || sh < 8)
            return full; // 小窗被挤没了：退回整块（宁可多扫，不可漏掉）
        RoiWindow w{};
        w.x = static_cast<uint32_t>(sx);
        w.y = static_cast<uint32_t>(sy);
        w.w = static_cast<uint32_t>(sw);
        w.h = static_cast<uint32_t>(sh);
        w.full_frame = false;
        return w;
    }

    // 本帧扫哪 1~2 个窗（detail 见 ArmorConfig 里那段说明）
    uint32_t search_windows(const RoiWindow &full, const TrackOutput &led, const GrayFrame &f,
                            RoiWindow *out) const {
        out[0] = full;
        if (!have_pair_)
            return 1;
        if (!(led.radius >= cfg_.per_bar_r)) { // 远距离：合窗（单条窗太小不可靠）
            out[0] = union_window(full, led);
            return 1;
        }
        RoiWindow reach{};
        if (!reach_box(led, f, &reach)) {
            out[0] = union_window(full, led);
            return 1;
        }
        uint32_t n = 0;
        const Bar *bars[2] = {&a_, &b_};
        for (int i = 0; i < 2; ++i) {
            RoiWindow w{};
            if (bar_window(*bars[i], led, reach, f, &w))
                out[n++] = w;
        }
        if (n < 2) { // 有窗建不起来 → 整帧退回合窗（宁可多扫，不可漏）
            out[0] = union_window(full, led);
            return 1;
        }
        return n;
    }

    static bool clamp_window(float x, float y, float w, float h, uint32_t fw, uint32_t fh,
                             float min_side, RoiWindow *out) {
        int32_t ix = static_cast<int32_t>(x);
        int32_t iy = static_cast<int32_t>(y);
        int32_t iw = static_cast<int32_t>(w);
        int32_t ih = static_cast<int32_t>(h);
        if (ix < 0) {
            iw += ix;
            ix = 0;
        }
        if (iy < 0) {
            ih += iy;
            iy = 0;
        }
        if (ix + iw > static_cast<int32_t>(fw))
            iw = static_cast<int32_t>(fw) - ix;
        if (iy + ih > static_cast<int32_t>(fh))
            ih = static_cast<int32_t>(fh) - iy;
        if (iw < static_cast<int32_t>(min_side) || ih < static_cast<int32_t>(min_side))
            return false; // 窗口被裁得没意义：本帧不出装甲板结果（宁可不报）
        out->x = static_cast<uint32_t>(ix);
        out->y = static_cast<uint32_t>(iy);
        out->w = static_cast<uint32_t>(iw);
        out->h = static_cast<uint32_t>(ih);
        out->full_frame = false;
        return true;
    }

    // ---- track：只认亲（不做形状判据）。返回 true = 本帧已经给出结果 ----
    bool track_step(const GrayFrame &bin, const RoiWindow *wins, uint32_t nwin,
                    const TrackOutput &led, uint32_t pxmin, ArmorTarget *out) {
        Bar  na{}, nb{};
        bool ok = false;
        if (nwin == 2) { // 单条窗：每个窗里只认自己那条
            bool a_ok = false, b_ok = false;
            for (uint32_t k = 0; k < 2; ++k) {
                trace_.window_px += static_cast<uint32_t>(wins[k].pixels());
                const uint32_t n = meas_.collect(bin, wins[k], pxmin, blobs_, kArmorMaxBars);
                trace_.blobs += n;
                const Bar &ref = (k == 0) ? a_ : b_;
                Bar        tmp{};
                if (assoc(blobs_, n, ref, &tmp)) {
                    if (k == 0) {
                        na = tmp;
                        a_ok = true;
                    } else {
                        nb = tmp;
                        b_ok = true;
                    }
                }
            }
            ok = a_ok && b_ok;
        } else { // 合窗：两条 incumbent 在同一个池里各认各的
            trace_.window_px += static_cast<uint32_t>(wins[0].pixels());
            const uint32_t n = meas_.collect(bin, wins[0], pxmin, blobs_, kArmorMaxBars);
            trace_.blobs += n;
            const bool a_ok = assoc(blobs_, n, a_, &na);
            const bool b_ok = assoc(blobs_, n, b_, &nb);
            ok = a_ok && b_ok && !(na.x == nb.x && na.y == nb.y && na.w == nb.w && na.h == nb.h);
        }
        if (ok) {
            int32_t cx = 0, cy = 0;
            if (armor_center(na, nb, cfg_.diag_min_sin, cfg_.flat_deg, &cx, &cy) &&
                same_body(cfg_, led, cx, cy)) {
                a_ = na;
                b_ = nb;
                c_[0] = cx;
                c_[1] = cy;
                age_ = 0;
                r_last_ = led.radius;
                trace_.mode = 1;
                trace_.bars = 2;
                *out = mk(ArmorTarget::Fresh, 0);
                fill_window(out, wins, nwin);
                return true; // cost_us 由 detect() 统一算
            }
        }
        // 认不到（或认到的东西跑出锚的够得着范围）→ 记 miss，而不是静默给一个坏中心；
        // 连续超过 hold 帧就释放这一对、退回 detect 重捕。
        ++age_;
        trace_.assoc_fail = 1;
        if (age_ <= cfg_.hold) {
            trace_.mode = 1;
            *out = mk(ArmorTarget::Held, static_cast<uint8_t>(age_));
            fill_window(out, wins, nwin);
            return true;
        }
        a_ = Bar{};
        b_ = Bar{};
        have_pair_ = false;
        age_ = 0;
        return false; // 落回 detect
    }

    // ---- 一帧判决：先关联（上一对还在就不换人），再保持，最后才重选 ----
    // 关联优先是"静止画面下结果乱跳"的正解：每帧从零重选时，两对分数接近就逐帧换人。
    // 保持（Held）只给旧中心、并打标记 —— 不是把旧值冒充成新测量。
    bool step(const Bar *bars, uint32_t n, const TrackOutput &led, int32_t s, ArmorTarget *out) {
        pair_count_ = 0;
        prior_reject_ = 0;

        if (have_pair_) {
            const Bar *na = assoc(bars, n, a_);
            const Bar *nb = assoc(bars, n, b_);
            int32_t    cx, cy;
            if (na != nullptr && nb != nullptr && na != nb &&
                armor_center(*na, *nb, cfg_.diag_min_sin, cfg_.flat_deg, &cx, &cy)) {
                a_ = *na;
                b_ = *nb;
                c_[0] = cx;
                c_[1] = cy;
                age_ = 0;
                r_last_ = led.radius;   // 记下"这一对是在多大尺度下采到的"

                *out = mk(ArmorTarget::Fresh, 0);
                ++pair_count_;
                return true;
            }
            ++age_;
            if (age_ <= cfg_.hold) {
                *out = mk(ArmorTarget::Held, static_cast<uint8_t>(age_));
                return true; // 短暂丢失：先保持（旧中心 + Held 标记）
            }
            have_pair_ = false; // 保持到期 → 放弃这一对，重新 detect
            age_ = 0;
        }

        // 重选：两两配对，取分数最小者
        int32_t best_score = 0;
        Bar     best_a{}, best_b{};
        int32_t best_cx = 0, best_cy = 0, best_px = -1;
        bool    found = false;
        for (uint32_t i = 0; i < n; ++i) {
            for (uint32_t j = i + 1; j < n; ++j) {
                int32_t score;
                if (!pair_score(bars[i], bars[j], &score))
                    continue;
                int32_t cx, cy;
                if (!armor_center(bars[i], bars[j], cfg_.diag_min_sin, cfg_.flat_deg, &cx, &cy))
                    continue;
                ++pair_count_;
                if (!prior_ok(led, s, bars[i], bars[j], cx, cy)) {
                    ++prior_reject_;
                    continue;
                }
                const int32_t px = static_cast<int32_t>(bars[i].px + bars[j].px);
                if (!found || score < best_score || (score == best_score && px > best_px)) {
                    found = true;
                    best_score = score;
                    best_a = bars[i];
                    best_b = bars[j];
                    best_cx = cx;
                    best_cy = cy;
                    best_px = px;
                }
            }
        }
        if (!found)
            return false;
        a_ = best_a;
        b_ = best_b;
        c_[0] = best_cx;
        c_[1] = best_cy;
        have_pair_ = true;
        age_ = 0;
        r_last_ = led.radius;

        *out = mk(ArmorTarget::Fresh, 0);
        return true;
    }

    // track 阶段的"认亲"：在该窗捞到的块里找 ref 的同一条灯条。
    // 判据只有位置与尺寸容差 —— 这就是"在它自己的窗里连续变化"这件事的全部内容：
    //   位置容差挡住"跳到别处"，尺寸容差挡住"换了个大小的东西"。
    // 形状判据（长度/长宽比/填充率/圆度）一个都不做：那是 detect 阶段
    // "从零判断这是不是一条灯条"才需要的，track 阶段它只会让跟踪更容易掉。
    bool assoc(const RoiBlobMeasurer::BlobInfo *blobs, uint32_t n, const Bar &ref,
               Bar *out) const {
        const float tx = cfg_.assoc_k * static_cast<float>(ref.w) > cfg_.assoc_min
                             ? cfg_.assoc_k * static_cast<float>(ref.w)
                             : cfg_.assoc_min;
        const float ty = cfg_.assoc_k * static_cast<float>(ref.h) > cfg_.assoc_min
                             ? cfg_.assoc_k * static_cast<float>(ref.h)
                             : cfg_.assoc_min;
        bool    found = false;
        int64_t best = 0;
        for (uint32_t i = 0; i < n; ++i) {
            Bar b{};
            fill_bar(blobs[i], &b);
            if (static_cast<float>(abs_i(b.cx() - ref.cx())) > tx ||
                static_cast<float>(abs_i(b.cy() - ref.cy())) > ty ||
                static_cast<float>(abs_i(b.w - ref.w)) > tx ||
                static_cast<float>(abs_i(b.h - ref.h)) > ty)
                continue;
            const int64_t dx = b.cx() - ref.cx(), dy = b.cy() - ref.cy();
            const int64_t d2 = dx * dx + dy * dy;
            if (!found || d2 < best) {
                best = d2;
                found = true;
                *out = b;
            }
        }
        return found;
    }

    // 在候选里找 ref 的同一条灯条：位置和尺寸都在容差内，取中心最近的那个。
    // 容差按灯条尺寸给（远处 3px 的灯条和近处 100px 的灯条，容差不可能是同一个数）。
    const Bar *assoc(const Bar *bars, uint32_t n, const Bar &ref) const {
        const float tx = cfg_.assoc_k * static_cast<float>(ref.w) > cfg_.assoc_min
                             ? cfg_.assoc_k * static_cast<float>(ref.w)
                             : cfg_.assoc_min;
        const float ty = cfg_.assoc_k * static_cast<float>(ref.h) > cfg_.assoc_min
                             ? cfg_.assoc_k * static_cast<float>(ref.h)
                             : cfg_.assoc_min;
        const Bar  *found = nullptr;
        int64_t     best = 0;
        for (uint32_t i = 0; i < n; ++i) {
            const Bar &b = bars[i];
            if (static_cast<float>(abs_i(b.cx() - ref.cx())) > tx ||
                static_cast<float>(abs_i(b.cy() - ref.cy())) > ty ||
                static_cast<float>(abs_i(b.w - ref.w)) > tx ||
                static_cast<float>(abs_i(b.h - ref.h)) > ty)
                continue;
            const int64_t dx = b.cx() - ref.cx(), dy = b.cy() - ref.cy();
            const int64_t d2 = dx * dx + dy * dy;
            if (found == nullptr || d2 < best) {
                best = d2;
                found = &b;
            }
        }
        return found;
    }

    ArmorTarget mk(uint8_t mode, uint8_t held) const {
        ArmorTarget t{};
        t.cx = c_[0];
        t.cy = c_[1];
        t.mode = mode;
        t.held = held;
        t.a_cx = a_.cx();
        t.b_cx = b_.cx();
        t.a_len = static_cast<uint32_t>(a_.len + 0.5f);
        t.b_len = static_cast<uint32_t>(b_.len + 0.5f);
        return t;
    }
    // 多窗 → 一个并集矩形（日志/CSV/画框用；逐窗明细在 Trace.windows 里）
    static void fill_window(ArmorTarget *t, const RoiWindow *wins, uint32_t n) {
        if (n == 0 || wins[0].empty())
            return;
        uint32_t x0 = wins[0].x, y0 = wins[0].y, x1 = wins[0].right() - 1u,
                 y1 = wins[0].bottom() - 1u;
        for (uint32_t i = 1; i < n; ++i) {
            if (wins[i].empty())
                continue;
            if (wins[i].x < x0)
                x0 = wins[i].x;
            if (wins[i].y < y0)
                y0 = wins[i].y;
            if (wins[i].right() - 1u > x1)
                x1 = wins[i].right() - 1u;
            if (wins[i].bottom() - 1u > y1)
                y1 = wins[i].bottom() - 1u;
        }
        t->x0 = x0;
        t->y0 = y0;
        t->x1 = x1;
        t->y1 = y1;
    }

    ArmorConfig        cfg_;
    RoiBlobMeasurer    meas_;                          // 自己的实例：不与绿灯那一路抢 trace
    RoiBlobMeasurer::BlobInfo blobs_[kArmorMaxBars];    // 定长，运行期零分配
    Bar                bars_[kArmorMaxBars];
    Bar                a_{}, b_{};
    int32_t            c_[2] = {0, 0};
    bool               have_pair_ = false;
    uint8_t            age_ = 0;
    int32_t            scale_s_ = 0;
    float              ds_ = 0.0f;
    float              r_last_ = 0.0f; // 上次采到那一对时的绿灯半径（算尺度增长用）
    uint32_t           pair_count_ = 0;
    uint32_t           prior_reject_ = 0;
    Trace              trace_{};
};

} // namespace dart::detection
