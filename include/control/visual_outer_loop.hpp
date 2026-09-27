#pragma once

// 模块 ②：视觉外环 —— **像素误差上的 PD 滑模**（带能量保护）。
//
// 滑模面取 PD 形式，到达律用带边界层的 sat：
//
//     s    = Ė + λ·E                （E = 有符号误差信号 [px]，Ė 是它的速度 [px/s]）
//     Δθ_c = kd · clamp(s, ±φ)      （Δθ_c 就是迎角指令 α_cmd）
//     Δθ_c ← clamp(Δθ_c, ±limit)    （能量保护：三重限幅取最小）
//     Δθ   = Δθ_c + (γ̂ - θ_base)    （把 α 指令换算成"相对基准姿态的偏置"）
//
// **误差信号 E 有两档，开关在 OuterLoopParams::surface_on_heading_error**：
//   E = ε·f（像素误差）或 E = σ·f = ε·f + α·f（航向误差）。
//   前者是"把机头压到目标上"，后者是"把速度压到视线上"——只有后者能让脱靶量趋零。
//
//   * **带内（|s| ≤ φ）**：Δθ = kd·Ė + kd·λ·E = **kd·Ė + kp·E** —— 就是你原文那两条
//     PD 表达式（Kp = kd·λ、Kd = kd）。所以你的 PD 是这套滑模律的边界层内行为。
//   * **带外（|s| > φ）**：Δθ 饱和在 kd·φ —— 这是滑模的切换（bang-bang）段，
//     也是"拉回轨道"最用力的那段。
//
// 能量保护限的是**偏置 Δθ**（也就是迎角指令），三重取最小：
//   ① 翼面失速 ② 过载/能耗（A·α ≤ a_max）③ 低速压缩（v 掉到 floor 以下就归零）。

#include "control/control_types.hpp"

namespace dart::control {

class VisualOuterLoop {
public:
    VisualOuterLoop(const OuterLoopParams &params, const AeroParams &aero)
        : params_(params)
        , aero_(aero) {}

    // 一条通道一拍。error_px 是你的 e_u/e_v，los_rate_px 是目标自身像速（已扣自旋，
    // 也就是 α-β 给的 pure_velocity；没有就给 0，此时 Ė 只剩自旋那一项）。
    // theta_base 由调用方给（预设基准姿态），用于把 α 指令换算成姿态偏置。
    AxisCommand Update(GuideAxis axis, const ChannelFeedback &feedback, double theta_base, double error_px,
                       double los_rate_px) const;

    // 本拍生效的偏置上限 [rad]（能量保护算出来的；测试与日志都用它，不另算一遍）。
    double OffsetLimit(double speed) const;

    const OuterLoopParams &params() const { return params_; }
    double pixel_sign(GuideAxis axis) const { return params_.pixel_sign[Index(axis)]; }

private:
    OuterLoopParams params_;
    AeroParams aero_;
};

} // namespace dart::control
