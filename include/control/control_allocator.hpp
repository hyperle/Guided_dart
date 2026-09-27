#pragma once

// 模块 ④：控制分配 —— 三个力矩轴 → 四片舵面，输出归一化舵偏 [-1, 1]。
//
// 几何全部走分配矩阵（代码里不写死任何 ± 号）：舵偏正负、构型、方位角都只是数据
// （CONTROL_TODO.md §4 要求实拍确认）。
//
// 十字/X 型尾翼的一个硬不变量：M_x = R·k·Σδᵢ 与方位角无关 ⇒ **滚转列必然全 1**。
// 你原稿那组的滚转列是 (-1,-1,1,1)，而且三列正好是 (roll, pitch, yaw) 的一个循环置换
// —— 真打成那样，打俯仰会去滚转、打滚转会去偏航。自检里有一条专门钉这个不变量。

#include "control/control_types.hpp"

namespace dart::control {

class ControlAllocator {
public:
    explicit ControlAllocator(const AllocatorParams &params)
        : params_(params) {}

    // moment 按 BodyAxis 索引：(roll, pitch, yaw)，归一化。
    ServoCommand Allocate(const double moment[kBodyAxisCount]) const;

    // 只做矩阵乘法，不限幅（测试要能单独验矩阵本身是线性的）。
    void ApplyMix(const double moment[kBodyAxisCount], double out[kFinCount]) const;

    const AllocatorParams &params() const { return params_; }

private:
    AllocatorParams params_;
};

} // namespace dart::control
