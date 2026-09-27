#pragma once

// 模块 ③：姿态内环 —— 离散时间滑模，一拍解出该轴的归一化力矩指令。
//
// 被控对象（连续系数口径，与 predictor.cpp 的按拍递推差一个 tick）：
//     θ⁺ = θ + tick·ω
//     ω⁺ = ω + tick·(Cmα·α + Cmδ·δ + b̂)
// 滑模面（PD 形式，相对阶 1）：
//     s = (ω - ω_cmd) + c·(θ - θ_cmd)
// 到达律：带边界层的 sat，**一次解出** δ（不是"先连续设计再离散化"）：
//     Cmδ·δ = (s_next - s)/tick - Cmα·α - b̂ - c·(ω - ω_cmd)
//     s_next = s - K·tick·sat(s/φ)
//
// 三件事值得单独说：
//   ① **b̂ 直接抵消**：观测器吃掉慢变/稳态，切换项只管未建模的快变 → 增益可以取小、抖振小。
//   ② **φ ≥ K·tick 是硬约束**（不是调参建议）：带内 s_next = s·(1 - K·tick/φ)，
//      违反它就是一个稳定的极限环 —— 500 Hz 下表现为"舵机嗡嗡响"。
//      代码里夹住并报 ill_posed，不许静默。
//   ③ **Cmδ 是除数**：舵效失效（动压太低）时不许解这个方程（会解出巨值再被限幅，
//      等于一个不可解释的 bang-bang），改走"只压角速度"的兜底并置 authority_lost。

#include "control/control_types.hpp"

namespace dart::control {

struct InnerLoopInput {
    double theta = 0.0;       // 姿态角 [rad]
    double omega = 0.0;       // 角速度 [rad/s]
    double alpha = 0.0;       // 迎角 [rad]（静稳定力矩的输入）
    double speed = 0.0;       // 前向速度 [m/s]（动压调度用）
    double theta_cmd = 0.0;   // 姿态指令 [rad]（= θ_base + Δθ）
    double omega_cmd = 0.0;   // 指令速率前馈 [rad/s]（没有就填 0）
    double disturbance = 0.0; // 集总扰动 b̂ [rad/s²]
    bool valid = false;
};

struct InnerLoopOutput {
    double moment = 0.0;   // 归一化力矩指令 [-1, 1]
    double sliding = 0.0;  // s
    double error = 0.0;    // θ - θ_cmd
    bool valid = false;         // false = 本拍没状态，力矩指令按 0 处理
    bool saturated = false;     // 力矩指令触限
    bool authority_lost = false; // 舵效失效，走了兜底
    bool ill_posed = false;     // φ < K·tick（离散化约束被破坏）
};

class AttitudeInnerLoop {
public:
    AttitudeInnerLoop(const InnerLoopParams &params, const AeroParams &aero)
        : params_(params)
        , aero_(aero) {}

    InnerLoopOutput Update(const InnerLoopInput &in, double tick) const;

private:
    InnerLoopParams params_;
    AeroParams aero_;
};

} // namespace dart::control
