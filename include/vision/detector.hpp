#pragma once

#include "core/frame.hpp"

namespace dart {

// 特征识别接口：只吃二值化帧视图，只吐中心像素坐标 + 尺度 + ROI 状态。
// 当前实现是 dart::detection::DetectionPipeline（include/detection/pipeline.hpp）：
//   启动态全图 RVV 粗筛 + 3 帧滑窗确认；跟踪态动态 ROI + 尺度自适应卡尔曼。
// 本接口只承诺"一帧进、一帧出"，不关心内部有几个阶段 —— 换实现（颜色/形状/KPU/别的跟踪器）
// 时 main.cpp 的流程、内存、取证、录像都不用动。
class IDetector {
public:
    virtual ~IDetector() = default;

    // 返回 cx=cy=-1 表示无目标；roi 说明是"框内无目标"还是"目标超框"
    virtual DetectResult detect(const GrayFrame &frame) = 0;
};

} // namespace dart
