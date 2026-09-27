#pragma once

// ============================================================================
// 检测/跟踪层的**值类型**：这里只放数据，不放算法，也不认识像素内存。
//
// 数据流向（谁产生 → 谁消费）：
//
//   二值图 ──① 全图扫描（RVV）──> LightCandidate[]  ──③ 启动确认 ──┐
//                                                                │ 确认
//   二值图 ──② ROI 扫描 ────────> TargetMeasurement ──④ 卡尔曼 ──>│
//                                                                ↓
//   ⑤ RoiWindow（下一帧扫哪里）<── ROI 预测（kp·ŝ + B_margin）<── 状态机
//
// 三张表都是 POD：可自由拷贝、入队、写 CSV，不持有任何资源 —— 板端取证通道
// （EvidenceWriter）与录像标注直接读它们，不需要额外的序列化层。
// ============================================================================

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "core/frame.hpp"

namespace dart::detection {

// ---------------------------------------------------------------------------
// 跟踪状态机（工况 1 → 工况 2 的切换只有这三种状态）
//   Startup ：还没确认目标。每帧全图扫描 + 3 帧滑窗确认（启动阶段，目标很小）
//   Tracking：已确认。每帧只在预测 ROI 内扫描，卡尔曼滤波持续更新
//   Lost    ：ROI 内连续 N 帧没测到。窗口逐帧放大，超过门限后回到全图扫描重新确认
// ---------------------------------------------------------------------------
enum class TrackState : uint8_t {
    Startup  = 0,
    Tracking = 1,
    Lost     = 2,
};

// 状态名（日志/CSV 用；检测层不自己打日志，只提供字符串）
inline const char *to_string(TrackState s) {
    switch (s) {
    case TrackState::Startup:
        return "启动";
    case TrackState::Tracking:
        return "跟踪";
    case TrackState::Lost:
        return "丢失";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// 扫描窗口（ROI）。out-of-frame 的部分不出现：构造时就夹进画面，
// 夹过就置 clamped，让上层知道"这一帧窗口被边界削过"（贴边目标要留神）。
// ---------------------------------------------------------------------------
struct RoiWindow {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t w = 0;
    uint32_t h = 0;
    bool     full_frame = true; // true = 本帧扫整幅（启动 / 丢失复位）
    bool     clamped = false;   // 预测窗口被画面边界夹过

    bool     empty() const { return w == 0 || h == 0; }
    uint32_t right() const { return x + w; }  // 开区间
    uint32_t bottom() const { return y + h; } // 开区间
    uint64_t pixels() const { return static_cast<uint64_t>(w) * h; }

    bool contains(float px, float py) const {
        return px >= static_cast<float>(x) && py >= static_cast<float>(y) &&
               px < static_cast<float>(right()) && py < static_cast<float>(bottom());
    }
};

// ---------------------------------------------------------------------------
// 一块亮斑（全图扫描的产出）。
// 二值图上没有灰度可言，所以这里的"亮度"就等于**面积**：score 默认 = area，
// 排序取 Top-K 用的就是它。将来若改用原始 Y 平面的灰度和做排序键，
// 只需要在扫描器里改 score 的填法，下游（Top-K、确认器）都不用动。
// ---------------------------------------------------------------------------
struct LightCandidate {
    float    cx = 0.0f;      // 质心 x（矩算出，亚像素）
    float    cy = 0.0f;      // 质心 y
    uint32_t area = 0;       // 命中像素数
    float    radius = 0.0f;  // 等效半径 r = sqrt(A/π)：跟踪层的尺度观测量
    uint32_t x0 = 0;         // 紧致包围盒（闭区间）
    uint32_t y0 = 0;
    uint32_t x1 = 0;
    uint32_t y1 = 0;
    float    fill = 0.0f;      // area / 包围盒像素数：形状连续性（散点噪声会很低）
    // **圆度**（见 roundness_from_moments）：完美圆≈1、细长条→0，且**与尺寸无关**。
    // 发光体成像是圆斑，反光/拖影是长条 —— 它比 fill 更能分开这两类：
    // 一条 40x3 的横条 fill=1.0（包围盒就是它本身！），圆度只有 ~0.08。
    float    circularity = 0.0f;
    float    continuity = 0.0f; // 滑窗命中率 hits/window（由启动确认器填）
    float    score = 0.0f;      // 排序键
    // 包围盒贴到画面边界（目标被画面切掉）。贴边的块：面积只是**下界**、质心是**偏的**
    // （真实中心可能在画面外），拿它起滤波必然几帧就丢。
    // 板端 round12 的实测教训：那一轮画面里没有目标，826 张二值图 91% 全黑，
    // 有亮像素的 74 张里 51 张的包围盒贴边（手/物体从镜头前扫过，r 达 22~86px），
    // 而 8 次"确认"全部落在这类块上 —— 于是"确认→3 帧→丢失"循环了一整轮。
    bool     border = false;
};

// ---------------------------------------------------------------------------
// 一帧里对目标的测量（ROI 扫描的产出）：卡尔曼滤波只吃这三个量。
// quality ∈ (0,1]：1 = 测量可信；越小 = 方差越大（目标贴着 ROI 边被削、
// 形状不合格等）。它直接喂给噪声模型，是"远距离大 R_scale"之外的第二个自适应入口。
// ---------------------------------------------------------------------------
struct TargetMeasurement {
    bool     valid = false;
    float    cx = 0.0f;
    float    cy = 0.0f;
    float    radius = 0.0f;
    uint32_t area = 0;
    float    fill = 0.0f;
    float    circularity = 0.0f; // 圆度（见 LightCandidate::circularity）
    float    quality = 1.0f;
    RoiState roi = RoiState::NoTargetInRoi;
    uint32_t cost_us = 0;
};

// ---------------------------------------------------------------------------
// 一帧扫描的原始结果：要么是"全图候选表"，要么是"ROI 里的一个测量"。
// 跟踪层只认这一个输入，于是它完全不依赖扫描器的实现（假件也喂得进来）。
// ---------------------------------------------------------------------------
struct ScanReport {
    const LightCandidate *cands = nullptr;
    size_t                count = 0;
    TargetMeasurement     measurement{};
    RoiWindow             window{};
    uint32_t              scan_us = 0; // 扫描耗时（不含状态机）

    bool full_scan() const { return window.full_frame; }
};

// ---------------------------------------------------------------------------
// 跟踪层每帧的输出（面面层负责把它翻译成 dart::DetectResult）
// ---------------------------------------------------------------------------
struct TrackOutput {
    TrackState state = TrackState::Startup;
    bool       found = false;
    float      cx = 0.0f;
    float      cy = 0.0f;
    float      radius = 0.0f;
    float      circularity = -1.0f; // 本帧目标块的圆度（4πA/P²），取证/调参用
    RoiState   roi = RoiState::NoTargetInRoi;
    RoiWindow  window{};
    bool       kf_rejected = false; // 本帧测量被马氏门限拒收（离群点）
};

// ---------------------------------------------------------------------------
// 计数器：全部只在状态机内部累加，用于板端 1Hz 状态行与"为什么切状态"的复盘。
// 不做日志输出 —— 检测层不认识 log（职责分离），谁来打由调用方决定。
// ---------------------------------------------------------------------------
struct TrackerCounters {
    uint64_t arm_gained = 0;      // 启动阶段确认成功次数（→ 跟踪）
    uint64_t to_lost = 0;         // 跟踪 → 丢失
    uint64_t to_tracking = 0;     // 丢失 → 跟踪（重捕）
    uint64_t to_startup = 0;      // 丢失 → 启动（硬复位）
    uint64_t kf_reject = 0;       // 马氏门限拒收次数
    uint64_t full_scans = 0;      // 全图扫描帧数
    uint64_t roi_scans = 0;       // ROI 扫描帧数
    uint64_t arm_frames = 0;      // 参与滑窗确认的帧数
    uint64_t arm_reject = 0;      // 被平滑性/命中数否掉的候选个数（闪烁坏点计数）
    uint64_t arm_border_skip = 0; // 被跳过的贴画面边候选个数（不是坏点，是被边界切掉的块）
    uint64_t out_of_roi = 0;      // 目标超框次数
    uint64_t reacquire_soft = 0;  // 丢失→跟踪 的软重捕（沿用滤波器状态）
    uint64_t full_confirm = 0;    // **跟踪态**又走了一次"全图+确认"（ROI 退化成整幅的直接症状）
    uint64_t reacquire_hard = 0;  // 丢失后重新初始化滤波器
};

// 圆度：取值 0~1，完美圆≈1、细长条→0。两件事一起看：
//   ① 等效椭圆轴比 sqrt(λmin/λmax)：把块的像素分布拟合成椭圆后的长短轴之比。
//      **与尺寸无关** —— 这条是关键：启动阶段目标只有几个像素，不能用"越小越吃亏"的度量。
//   ② 归一化填充率 min(1, (A/(W·H)) / (π/4))：包围盒填充率相对"圆应有的 0.785"，
//      用来挡住镂空/散点/怪形（它们的椭圆比可能不难看，但填充率低）。
//
// 为什么**不用**教科书里的 4πA/P²：数字化圆的边界是阶梯状，4 邻域周长被系统性高估
// （实测：r=12px 的干净圆盘只能算出 0.579，半径越小越差），
// 拿它当门限会把"又小又圆"的真目标判成不圆 —— 那是启动阶段最不能出错的地方。
inline float roundness_from_moments(double area, double sx, double sy, double sxx, double syy,
                                    double sxy, double bw, double bh) {
    if (area <= 0.0)
        return 0.0f;
    const double mx = sx / area, my = sy / area;
    const double cxx = sxx / area - mx * mx;
    const double cyy = syy / area - my * my;
    const double cxy = sxy / area - mx * my;
    const double tr = cxx + cyy;
    const double det = cxx * cyy - cxy * cxy;
    const double disc = tr * tr * 0.25 - det;
    const double sq = disc > 0.0 ? std::sqrt(disc) : 0.0;
    const double lmax = tr * 0.5 + sq;
    const double lmin = tr * 0.5 - sq;

    double elong;
    if (lmax <= 1e-9)
        elong = 0.6; // 单个像素：没有形状信息（该由 min_area 去挡），给中性值
    else if (lmin <= 0.0)
        elong = 0.0; // 严格共线 = 针
    else
        elong = std::sqrt(lmin / lmax);

    const double box = (bw * bh > 0.0) ? bw * bh : 1.0;
    double       fill_norm = (area / box) / 0.7853981633974483; // π/4
    if (fill_norm > 1.0)
        fill_norm = 1.0;
    return static_cast<float>(elong * fill_norm);
}

// 候选排序键：score = 面积 × 圆度^w。
//   w=0 → 退化成"纯面积"（改动前的行为，便于 A/B 对照）
//   w=1 → 面积与圆度等权：**两倍面积但圆度只有一半的块，会和圆的打平**；细长条被压下去
// 圆度取下限 0.05 只是防止"把候选直接归零"，不是判据（要硬筛请用 min_circularity）。
inline float shape_score(float area, float circularity, float w) {
    if (area <= 0.0f)
        return 0.0f;
    if (w <= 0.0f)
        return area;
    const float c = circularity > 0.05f ? circularity : 0.05f;
    return area * std::pow(c, w);
}

// r = sqrt(A/π)：面积与等效半径互转。板端二值图上的"目标大小"统一用 r 表示
// （面积的平方根对边缘闪烁更稳，而 r 与卡尔曼的尺度状态一一对应）。
inline float radius_from_area(float area) {
    return area > 0.0f ? std::sqrt(area / 3.14159265358979323846f) : 0.0f;
}

// 主轴分解（与 roundness_from_moments 同源的二阶矩，但**保留方向与尺度**）：
//   theta    = 主轴方向（弧度，从 +x 轴起算，落在 [-π/2, π/2]）。轴是直线，mod π 无歧义。
//   len_major/len_minor = 主/次轴上的等效矩形边长。实心矩形的二阶矩 λ = L²/12，
//                        所以 L = sqrt(12·λ)（6x60 的灯条 → λmax=300 → L=60 ✓）。
//
// 为什么需要它：**包围盒对旋转极其敏感**。一条 5x40 的灯条转 30°，包围盒变成 24x37 ——
// 长宽比 8.0→1.5、填充率 1.00→0.22，于是"长宽比门限"和"填充率门限"同时失效，
// 端点也会从"长轴端点"退化成"包围盒边中点"（偏出半个包围盒宽）。
// 而主轴长短边与方向**与旋转无关**，灯条平行但不竖直时它照样准。
inline void principal_axis(double area, double sx, double sy, double sxx, double syy, double sxy,
                           float *theta, float *len_major, float *len_minor) {
    if (theta != nullptr)
        *theta = 0.0f;
    if (len_major != nullptr)
        *len_major = 0.0f;
    if (len_minor != nullptr)
        *len_minor = 0.0f;
    if (area <= 0.0)
        return;
    const double mx = sx / area, my = sy / area;
    const double cxx = sxx / area - mx * mx;
    const double cyy = syy / area - my * my;
    const double cxy = sxy / area - mx * my;
    const double tr = cxx + cyy;
    const double det = cxx * cyy - cxy * cxy;
    const double disc = tr * tr * 0.25 - det;
    const double sq = disc > 0.0 ? std::sqrt(disc) : 0.0;
    const double lam_major = tr * 0.5 + sq;
    const double lam_minor = tr * 0.5 - sq;
    if (theta != nullptr)
        *theta = static_cast<float>(0.5 * std::atan2(2.0 * cxy, cxx - cyy));
    if (len_major != nullptr)
        *len_major = lam_major > 0.0 ? static_cast<float>(std::sqrt(12.0 * lam_major)) : 0.0f;
    if (len_minor != nullptr)
        *len_minor = lam_minor > 0.0 ? static_cast<float>(std::sqrt(12.0 * lam_minor)) : 0.0f;
}

inline float area_from_radius(float r) {
    return r > 0.0f ? 3.14159265358979323846f * r * r : 0.0f;
}

} // namespace dart::detection
