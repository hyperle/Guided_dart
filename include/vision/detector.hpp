#pragma once

#include "core/frame.hpp"

namespace dart {

// 特征识别接口：只吃二值化帧视图，只吐中心像素坐标 + ROI 状态。
// 实现（颜色阈值 / 形状 / KPU）由使用方提供，本层不关心，也不做任何封装。
class IDetector {
public:
    virtual ~IDetector() = default;

    // 返回 cx=cy=-1 表示无目标；roi 说明是"框内无目标"还是"目标超框"
    virtual DetectResult detect(const GrayFrame &frame) = 0;
};

} // namespace dart
