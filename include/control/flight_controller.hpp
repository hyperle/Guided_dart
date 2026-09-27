#pragma once

// 编排：把四个模块串起来，每拍输出一次舵机指令（归一化 [-1, 1]）。
//
//   ① 状态机      FlightStateMachine   → 这一刻用哪个模式（外环要不要工作）
//   ② 视觉外环    VisualOuterLoop      → Δθ、Δψ（PD 滑模 + 能量保护）
//   ③ 姿态内环    AttitudeInnerLoop    → 三个轴的归一化力矩
//   ④ 控制分配    ControlAllocator     → 四片舵面的归一化指令 → IServoOutput
//
// 本类**没有一行控制律**：取数、按时序调用、合成 θ_cmd、下发。
//
// 依赖注入：控制器只认下面三个抽象，由调用方（main.cpp 侧）写适配器接进来。
// 本模块**不 include** IMU / 预测器 / 检测层任何头，数据从哪来与它无关
// （换数据源不碰这个文件）。这就是依赖倒置 + 构造注入，没有单例、没有全局态。

#include <cstdint>

#include "control/attitude_inner_loop.hpp"
#include "control/control_allocator.hpp"
#include "control/control_types.hpp"
#include "control/flight_state_machine.hpp"
#include "control/visual_outer_loop.hpp"

namespace dart::control {

// —— 端口（窄接口，各管一件事）——

class IBodyStateSource {
public:
    virtual ~IBodyStateSource() = default;

    // 返回 false = 这一刻的姿态/速度不可用（IMU 未就绪、饱和、丢帧）。
    // 实现方负责把 EKF 的扰动状态换算成连续量填进 disturbance [rad/s²]；
    // 没有扰动估计就填 0（内环会退化成纯滑模，稳态偏差变大但不会发散）。
    virtual bool ReadBodyState(BodyState *out) const = 0;

    virtual const char *name() const = 0;
};

class IVisionSource {
public:
    virtual ~IVisionSource() = default;

    // 返回 false = 本拍没有有效视觉（未确认/丢帧）。控制器按"没观测"处理，**不外推**。
    // 实现方负责把像素误差算成"相对预设像素"的值（e = 目标像素 − 预设像素）。
    virtual bool ReadTarget(TargetObservation *out) const = 0;

    virtual const char *name() const = 0;
};

class IServoOutput {
public:
    virtual ~IServoOutput() = default;

    virtual void Write(const ServoCommand &command) = 0;

    virtual const char *name() const = 0;
};

class FlightController {
public:
    struct Ports {
        IBodyStateSource *body = nullptr;
        IVisionSource *vision = nullptr;
        IServoOutput *servo = nullptr;
    };

    FlightController(const ControllerParams &params, const Ports &ports);

    // 逐拍调用，返回这一拍下发的舵机指令（同时已经写给 IServoOutput）。
    const ServoCommand &Update(uint64_t now_us);

    void Reset();

    FlightMode mode() const { return state_machine_.mode(); }
    const char *mode_name() const { return ModeName(state_machine_.mode()); }
    const ControllerParams &params() const { return params_; }
    const AxisCommand &axis_command(GuideAxis axis) const { return axis_[Index(axis)]; }
    const InnerLoopOutput &inner(GuideAxis axis) const { return inner_[Index(axis)]; }

private:
    ControllerParams params_;
    Ports ports_;
    FlightStateMachine state_machine_;
    VisualOuterLoop outer_loop_;
    AttitudeInnerLoop inner_loop_;
    ControlAllocator allocator_;

    AxisCommand axis_[kGuideAxisCount];
    InnerLoopOutput inner_[kGuideAxisCount];
    ServoCommand command_;
};

} // namespace dart::control
