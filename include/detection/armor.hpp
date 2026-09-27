#pragma once

// ============================================================================
// 装甲板识别（识别层第二路输出）：绿灯是锚，装甲板在它正上方那块窗口里。
//
// 目标特征：一对**平行的细长发光灯条**。把 A 条的上角点连到 B 条的下角点、
// A 条的下角点连到 B 条的上角点，两条连线的交点就是装甲板中心
// （这两条线正是那四个角点围成的四边形的两条对角线）。
//
// 接法（已接进现有识别链，见 pipeline.cpp）：
//   DetectionPipeline::detect() 里 tracker_.end_frame() **之后**调用 ——
//   那时同时拿得到 ① 绿灯锚（TrackOutput：cx/cy/radius/state）② 本帧二值图。
//   窗口由锚现算（下沿贴绿灯上沿），只在窗口内做连通域，代价是几十微秒量级。
//
//   ┌ RoiBlobMeasurer::collect()  ← 复用现有游程+并查集（不新写连通域）
//   │      ↓ 候选块（全部，不是"最好的一个"）
//   │  is_bar()    ← 尺度自适应门限（灯尺的倍数 + 硬地板；远档改判圆度）
//   │      ↓ 灯条
//   │  pick_pair() ← 两两配对门限 + 刚性几何先验 + 交叉连线求交
//   │      ↓
//   └ ArmorTracker  ← 先关联（上一对还在就不换人）→ 短暂丢先保持（标 Held）→ 到期释放
//
// 与 MicroPython 版（tools/armor_detect_k230.py）的三处差异，都是"用 C++ 该有的写法"：
//   ① 不重复实现绿灯的锁：仓库 TargetTracker 已有 启动/跟踪/丢失 + 3 帧滑窗确认 +
//      尺度自适应卡尔曼，直接用它的输出当锚，不再维护第二套绿灯状态机；
//   ② 远档的形状判据不是"整条跳过"，而是改用**与尺度无关**的圆度
//      （types.hpp::roundness_from_moments）—— 3px 的灯条照样能判"不像圆"；
//   ③ 几何全整数、无浮点；夹角门限改用两次 sqrt 的整数百分比比较，避免
//      MicroPython 版的 10000·d² 在 int64 上溢出（d 可达 3.4e7 → 1.1e19）。
//
// 门限随距离变：灯尺 s = 2·r（r 是跟踪层给的等效半径）。
//   s=4px（远，灯条 3×1px）→ 门限落到地板 (len 2..32, px>=2)
//   s=100px（近，灯条 140×14）→ 门限自动长到 (20..800, px>=10)
// ============================================================================

#include <cmath>
#include <cstdint>

#include "core/frame.hpp"
#include "detection/blob_measure.hpp"
#include "detection/config.hpp"
#include "detection/types.hpp"

namespace dart::detection {

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

        // 2) 本帧扫哪里：跟踪态只扫上一对灯条周围（省时间），否则扫"绿灯上方整块"
        RoiWindow full{};
        if (!led_window(bin, led, &full))
            return;
        RoiWindow win = track_window(full);
        trace_.window_px = static_cast<uint32_t>(win.pixels());

        // 3) 连通域（复用现有游程+并查集）+ 灯条判据
        const uint32_t nblob = meas_.collect(bin, win, bar_px_min(s), blobs_, kArmorMaxBars);
        trace_.blobs = nblob;
        uint32_t nbar = 0;
        for (uint32_t i = 0; i < nblob; ++i) {
            Bar bar;
            if (!as_bar(blobs_[i], s, &bar)) {
                ++trace_.rejected;
                continue;
            }
            bars_[nbar++] = bar;
        }
        trace_.bars = nbar;

        // 4) 检测-跟踪
        const bool ok = step(bars_, nbar, led, s, out);
        trace_.pairs = pair_count_;
        trace_.rejected_prior = prior_reject_;
        trace_.cost_us = static_cast<uint32_t>(now_us - t0);
        (void)ok;
        fill_window(out, win);
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
        uint32_t window_px = 0;    // 本帧实际扫的窗口像素数（ROI 效率）
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
        if (s <= 0)
            return true; // 灯尺还没建立：没有可比的基准 → 放行
        const float fs = static_cast<float>(s);
        const float span = static_cast<float>(pair_span(a, b));
        const float up = static_cast<float>(led.cy) - static_cast<float>(led.radius) -
                         static_cast<float>(cy); // 板心在绿灯上沿之上为正
        const float offx = static_cast<float>(abs_i(cx - static_cast<int32_t>(led.cx + 0.5f)));
        if (span < cfg_.span_lo_k * fs || span > cfg_.span_hi_k * fs)
            return false;
        if (up < cfg_.up_lo_k * fs || up > cfg_.up_hi_k * fs)
            return false;
        if (offx > cfg_.offx_k * fs)
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

    // 跟踪态：只在上一对灯条周围开小窗，并与"绿灯上方整块"取交 ——
    // 一个错的跟踪状态不能把搜索带到画面别处去。
    RoiWindow track_window(const RoiWindow &full) const {
        if (!have_pair_)
            return full;
        const int32_t x0 = a_.x < b_.x ? a_.x : b_.x;
        const int32_t y0 = a_.y < b_.y ? a_.y : b_.y;
        const int32_t ax1 = a_.x + a_.w, bx1 = b_.x + b_.w;
        const int32_t ay1 = a_.y + a_.h, by1 = b_.y + b_.h;
        const int32_t x1 = ax1 > bx1 ? ax1 : bx1;
        const int32_t y1 = ay1 > by1 ? ay1 : by1;
        const int32_t side = (x1 - x0) > (y1 - y0) ? (x1 - x0) : (y1 - y0);
        const int32_t m = static_cast<int32_t>(cfg_.track_k * static_cast<float>(side));
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
        *out = mk(ArmorTarget::Fresh, 0);
        return true;
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
    static void fill_window(ArmorTarget *t, const RoiWindow &w) {
        t->x0 = w.x;
        t->y0 = w.y;
        t->x1 = w.empty() ? w.x : w.right() - 1u;
        t->y1 = w.empty() ? w.y : w.bottom() - 1u;
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
    uint32_t           pair_count_ = 0;
    uint32_t           prior_reject_ = 0;
    Trace              trace_{};
};

} // namespace dart::detection
