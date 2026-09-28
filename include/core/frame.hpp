#pragma once

#include <cstdint>

namespace dart {

// 二值化后的帧视图：0/255，按行 stride 排列（stride >= width）
struct GrayFrame {
    uint8_t *pixels;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
};

// ROI 判定：命中 / 框内无目标 / 目标超框
enum class RoiState : uint8_t {
    Hit = 0,
    NoTargetInRoi = 1,
    TargetOutOfRoi = 2,
};

// 装甲板结果（识别层的**第二路**输出：绿灯上方那两片灯条的对角线交点）。
//
// 为什么放在 core 而不是 detection：DetectResult 是核心层的 POD，核心层不能反向
// 依赖识别层。这里只放"上游要看的东西"（坐标 + 状态 + 窗口四角），灯条明细/调参
// 明细留在 ArmorDetector::Trace（那是识别层内部的事，不往录像标注里塞）。
struct ArmorTarget {
    // mode：0 = 本帧没有；1 = 本帧实测（两条对角线的交点）；2 = 保持的旧中心。
    // **1 和 2 必须分得清**：保持值是"上一帧的结论"，当成新测量喂给外环等于凭空
    // 补了一条假观测（宁可不更新，不可乱更新）。
    enum : uint8_t { None = 0, Fresh = 1, Held = 2 };

    int32_t  cx = -1;
    int32_t  cy = -1;
    uint8_t  mode = None;
    uint8_t  held = 0;        // mode=Held 时：已经连续保持了这么多帧

    // 两片灯条的中心/长边长度（取证与调参用：比值 = 条长 / 灯尺，用来收紧刚性先验）
    int32_t  a_cx = -1;
    int32_t  b_cx = -1;
    uint32_t a_len = 0;
    uint32_t b_len = 0;

    // 本帧**实际扫的窗口**（闭区间四角，口径与 DetectResult::roi_* 完全一致：
    // x1/y1 是框内最后一个像素）。跟踪时它是收窄后的小窗，不是"绿灯上方整块"。
    uint32_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;

    bool found() const { return mode != None; }
};

// 识别输出：中心像素坐标，无目标即 (-1,-1)（所以不需要单独的 found 字段）
struct DetectResult {
    int32_t  cx = -1;
    int32_t  cy = -1;
    RoiState roi = RoiState::NoTargetInRoi;
    uint32_t cost_us = 0; // 识别耗时，写进录像标注

    // ---- 以下由 detection 层（ScaleAwareTracker）填写；DetectorStub 时代恒为默认值 ----
    // 尺度与状态必须跟着结果一起走：录像标注、CSV 取证、板端调参都只看 DetectResult，
    // 再开一条"从识别器里捞内部状态"的旁路只会让两边慢慢不一致。
    float    radius = -1.0f; // 等效半径 r = sqrt(A/π)（px），无目标 -1
    // 目标圆度 4πA/P²（完美圆=1、正方形≈0.785）：调"圆度权重"时就看这一列，
    // 它是这一帧被判为目标的那个块的圆度（启动态取确认到的候选，跟踪态取 ROI 里的测量）。
    float    circ = -1.0f;
    uint8_t  state = 0;      // dart::detection::TrackState：0 启动 / 1 跟踪 / 2 丢失
    // 本帧实际扫描窗口的**四个角像素坐标**（全图扫描时就是整幅：0,0,W-1,H-1）。
    // 闭区间：roi_x1/roi_y1 是框内最后一个像素（不是"右边界外一格"）。
    // 为什么给四个角而不是 (x,y,w,h)：画框、和 PBM 逐像素对照时角坐标可以直接用，
    // 不必再做一次 x+w-1 的换算 —— 那个 -1 恰恰是这类记录最容易出错的地方。
    uint32_t roi_x0 = 0;
    uint32_t roi_y0 = 0;
    uint32_t roi_x1 = 0;
    uint32_t roi_y1 = 0;

    // ---- 装甲板（detection/armor/armor_detector.hpp 填；绿灯是锚，装甲板出精确板心）----
    // 绿灯那一路给的是"发光点的中心"，装甲板那一路给的是"板面上两片灯条围出的
    // 几何中心"——两者都在 DetectResult 里，谁用哪个由上层决定（外环/撞点选择）。
    ArmorTarget armor;
};

// 池里的一帧：像素内存 + 元数据 + 这一帧自己的识别结果
struct Frame {
    GrayFrame    view{};
    uint64_t     seq = 0;      // 帧序号（从 1 开始）
    uint64_t     mono_ms = 0;  // 取帧时刻（CLOCK_MONOTONIC，毫秒）
    DetectResult result;
    int          slot = -1;    // 池内槽位，由 FramePool 维护
    // 取证用：源 VICAP 帧的时间戳（量**真实**帧间隔，而不是数自己的循环次数），
    // 以及二值图的稀疏哈希（判"是不是同一帧被重复返回"）。
    uint64_t src_pts = 0;
    uint32_t y_hash = 0;
};

} // namespace dart
