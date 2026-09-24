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
