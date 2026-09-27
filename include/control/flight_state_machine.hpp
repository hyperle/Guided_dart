#pragma once

// 模块 ①：状态机管理（你的三个模式）。
//
//   kFreefall  刚抛出：外环偏置按 0 处理，只闭环基准姿态（纯抛物线）
//   kGlide     姿态捕获与轨迹修正：外环工作，theta_cmd = theta_base + delta
//   kTerminal  末段姿态锁死：外环偏置强制清零，全力恢复基准姿态
//
// 切换条件就用你原来那两条（目标框尺度 / 高度），另外补一条剩余时间判据；
// kTerminal 是**吸收态**（进去不再出来），与你原文的行为一致。

#include "control/control_types.hpp"

namespace dart::control {

class FlightStateMachine {
public:
    explicit FlightStateMachine(const StateMachineParams &params)
        : params_(params) {}

    // 逐拍推进。全部输入都是"这一刻的事实"，本类不做任何外推。
    FlightMode Update(bool launched, bool vision_valid, double elapsed_s, double bbox_size,
                      double altitude, double time_to_go) {
        switch (mode_) {
            case FlightMode::kFreefall:
                if (launched && vision_valid && elapsed_s >= params_.freefall_s) {
                    mode_ = FlightMode::kGlide;
                }
                break;
            case FlightMode::kGlide:
                if (ReachedTerminal(bbox_size, altitude, time_to_go)) mode_ = FlightMode::kTerminal;
                break;
            case FlightMode::kTerminal:
                break; // 吸收态
        }
        return mode_;
    }

    void Reset() { mode_ = FlightMode::kFreefall; }
    FlightMode mode() const { return mode_; }

private:
    bool ReachedTerminal(double bbox_size, double altitude, double time_to_go) const {
        if (params_.bbox_terminal > 0.0 && bbox_size >= params_.bbox_terminal) return true;
        if (params_.altitude_terminal > 0.0 && altitude > 0.0 && altitude <= params_.altitude_terminal) {
            return true;
        }
        if (params_.t_go_terminal > 0.0 && time_to_go > 0.0 && time_to_go <= params_.t_go_terminal) {
            return true;
        }
        return false;
    }

    StateMachineParams params_;
    FlightMode mode_ = FlightMode::kFreefall;
};

} // namespace dart::control
