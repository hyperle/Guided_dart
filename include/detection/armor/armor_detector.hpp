#pragma once

// ============================================================================
// 装甲板识别 —— 识别层的**第二路输出**。
//
// 输入：二值图 + 本帧跟踪层的输出（绿灯当锚；没锚不猜）
// 输出：ArmorTarget（板心 = 两片灯条围出的几何中心）
//
// 它在这条链上的位置：绿灯那一路给的是"发光点的中心"，这一路给的是"板面上两片
// 灯条围出的几何中心"——两者都写进 DetectResult，谁用哪个由上层决定。
//
// 内部三段：
//   ① 窗口规划   —— armor_windows.hpp（本帧扫哪 1~2 个矩形）
//   ② detect 路径 —— 全门限（armor_rules.hpp 的灯条判据 + 配对 + 刚性先验）
//   ③ track 路径  —— 只认亲（位置/尺寸容差），一条形状判据都不做
//
// 本文件只放**声明**：几何在 armor_geometry.cpp，尺度门限在 armor_scale.cpp，
// 判据在 armor_rules.cpp，窗口在 armor_windows.cpp，本类自己的状态机在
// armor_detector.cpp。这样"纯逻辑"可以逐条单测，不必先起一个检测器。
// ============================================================================

#include <cstdint>

#include "core/frame.hpp"
#include "detection/armor/armor_geometry.hpp"
#include "detection/armor/armor_scale.hpp"
#include "detection/armor/armor_windows.hpp"
#include "detection/measure/roi_measure.hpp"
#include "detection/config.hpp"
#include "detection/types.hpp"

namespace dart::detection {

class ArmorDetector {
public:
    explicit ArmorDetector(const ArmorConfig &cfg, const MeasureConfig &meas_cfg);

    void reset();

    // 一帧一拍。bin 与绿灯那一路是**同一张**二值图（所以装甲板天然是"黑白口径"）；
    // led 是本帧跟踪层的输出（found=false 时不出结果 —— 绿灯是窗口的锚，没锚不猜）。
    // now_us 是调用方给的时钟：主机测试注入假钟即可完全确定性。
    void detect(const GrayFrame &bin, const TrackOutput &led, uint64_t now_us, ArmorTarget *out);

    // 板端调参用（对齐 DetectionStats / TileScanner::Trace 的口径）
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
        uint32_t windows = 0;      // 本帧开了几个窗（1 = 合窗，2 = 每条灯条各一个）
        uint8_t  mode = 0;         // 0 = detect（全门限；首次/重捕），1 = track（只认亲）
        uint8_t  assoc_fail = 0;   // 1 = 本帧认亲失败（没认到 / 跑出锚的够得着范围）
        uint32_t cost_us = 0;
    };
    const Trace &last_trace() const { return trace_; }

private:
    using Bar = armor::Bar;

    // 把成员状态包成窗口层要看的视图（借用，不拷贝）
    armor::PairView pair_view() const;

    // ---- track：只认亲（不做形状判据）。返回 true = 本帧已经给出结果 ----
    bool track_step(const GrayFrame &bin, const RoiWindow *wins, uint32_t nwin,
                    const TrackOutput &led, uint32_t pxmin, ArmorTarget *out);

    // ---- 一帧判决：先关联（上一对还在就不换人），再保持，最后才重选 ----
    // 关联优先是"静止画面下结果乱跳"的正解：每帧从零重选时，两对分数接近就逐帧换人。
    // 保持（Held）只给旧中心、并打标记 —— 不是把旧值冒充成新测量。
    bool decide(const Bar *bars, uint32_t n, const TrackOutput &led, int32_t s, ArmorTarget *out);

    // track 阶段的"认亲"：在该窗捞到的块里找 ref 的同一条灯条，判据只有位置与尺寸容差。
    bool assoc(const RunLengthMeasurer::BlobInfo *blobs, uint32_t n, const Bar &ref, Bar *out) const;

    // 在候选里找 ref 的同一条灯条：位置和尺寸都在容差内，取中心最近的那个。
    // 容差按灯条尺寸给（远处 3px 的灯条和近处 100px 的灯条，容差不可能是同一个数）。
    const Bar *assoc(const Bar *bars, uint32_t n, const Bar &ref) const;

    ArmorTarget make_target(uint8_t mode, uint8_t held) const;

    // 多窗 → 一个并集矩形（日志/CSV/画框用；逐窗明细在 Trace.windows 里）
    static void fill_window(ArmorTarget *t, const RoiWindow *wins, uint32_t n);

    ArmorConfig               cfg_;
    RunLengthMeasurer           meas_; // 自己的实例：不与绿灯那一路抢 trace
    RunLengthMeasurer::BlobInfo blobs_[kArmorBarMax]; // 定长，运行期零分配
    Bar                       bars_[kArmorBarMax];
    Bar                       a_{}, b_{};
    int32_t                   c_[2] = {0, 0};
    bool                      have_pair_ = false;
    uint8_t                   age_ = 0;
    armor::ScaleState         scale_;
    float                     r_last_ = 0.0f; // 上次采到那一对时的绿灯半径（算尺度增长用）
    uint32_t                  pair_count_ = 0;
    uint32_t                  prior_reject_ = 0;
    Trace                     trace_{};
};

} // namespace dart::detection
