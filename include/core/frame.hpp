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
