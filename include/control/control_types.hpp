#pragma once

// 滑模控制器的值类型与参数（无实现依赖：不 include IMU / 预测器 / 检测层）。
//
// 四个模块共用这里：状态机 / 视觉外环（PD 滑模）/ 姿态内环 / 控制分配。
// 约定：机体 x 前 / y 右 / z 下；像面 x 右 / y 下；但**外环的输入输出沿用你原来的
// 像素口径**（e 用 px、ė 用 px/s、增益 rad/px 与 rad·s/px），只有能量保护那一层换算成
// 角度。这样你原来那组 Kp/Kd 的数字含义不变。

#include <cmath>
#include <cstdint>

namespace dart::control {

// ---------------------------------------------------------------------------
// 轴
// ---------------------------------------------------------------------------

inline constexpr int kFinCount = 4;

// 弹体三轴：姿态内环、力矩指令、控制分配按它索引。
enum class BodyAxis { kRoll = 0, kPitch = 1, kYaw = 2 };
inline constexpr int kBodyAxisCount = 3;
inline constexpr BodyAxis kBodyAxes[kBodyAxisCount] = { BodyAxis::kRoll, BodyAxis::kPitch, BodyAxis::kYaw };

// 视觉两通道（你代码里的 u/v）：只有俯仰、偏航参与轨道修正。
enum class GuideAxis { kPitch = 0, kYaw = 1 };
inline constexpr int kGuideAxisCount = 2;
inline constexpr GuideAxis kGuideAxes[kGuideAxisCount] = { GuideAxis::kPitch, GuideAxis::kYaw };

inline constexpr int Index(BodyAxis axis) { return static_cast<int>(axis); }
inline constexpr int Index(GuideAxis axis) { return static_cast<int>(axis); }
inline constexpr BodyAxis ToBodyAxis(GuideAxis axis) {
    return axis == GuideAxis::kPitch ? BodyAxis::kPitch : BodyAxis::kYaw;
}

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------

inline double Clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double Saturate(double x) { return Clamp(x, -1.0, 1.0); }
inline double Sign(double x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }

struct Attitude {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

struct AngularRate {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

inline double Value(const Attitude &attitude, BodyAxis axis) {
    switch (axis) {
        case BodyAxis::kRoll: return attitude.roll;
        case BodyAxis::kPitch: return attitude.pitch;
        case BodyAxis::kYaw: return attitude.yaw;
    }
    return 0.0;
}

inline double Value(const AngularRate &rate, BodyAxis axis) {
    switch (axis) {
        case BodyAxis::kRoll: return rate.roll;
        case BodyAxis::kPitch: return rate.pitch;
        case BodyAxis::kYaw: return rate.yaw;
    }
    return 0.0;
}

// ---------------------------------------------------------------------------
// 气动参数：只保留可标定的三个量。
//
// 刻意不使用 predictor.cpp 的 k_m / k_delta：占位值 k_delta = -2.8 意味着
// "1 单位舵偏 = 2.8 rad 等效迎角"，非物理；而且那对值是按拍口径。
// 下面三个是**连续量口径**（[1/s²/rad]、[1/s²]），动压按 (v/v_ref)² 缩放。
// ---------------------------------------------------------------------------

struct AeroParams {
    double gain_per_alpha = 4.9;     // A   横向加速度 ÷ 迎角 [m/s²/rad]
    double moment_per_alpha = -0.8;  // Cmα 力矩 ÷ 迎角 [1/s²/rad]（负 = 静稳定）
    double moment_per_delta = 2.24;  // Cmδ 单位舵偏的角加速度 [1/s²]
    double v_ref = 20.0;             // 上面三个系数的标定速度 [m/s]
};

inline double DynamicPressureScale(double speed, double v_ref) {
    if (!(v_ref > 0.0)) return 1.0;
    const double ratio = speed / v_ref;
    return ratio * ratio;
}

inline double ScaledGainPerAlpha(const AeroParams &aero, double speed) {
    return aero.gain_per_alpha * DynamicPressureScale(speed, aero.v_ref);
}

inline double ScaledMomentPerAlpha(const AeroParams &aero, double speed) {
    return aero.moment_per_alpha * DynamicPressureScale(speed, aero.v_ref);
}

inline double ScaledMomentPerDelta(const AeroParams &aero, double speed) {
    return aero.moment_per_delta * DynamicPressureScale(speed, aero.v_ref);
}

// ---------------------------------------------------------------------------
// 反馈与观测（模块的输入）
// ---------------------------------------------------------------------------

// 一条通道的反馈。theta/omega 给内环；alpha/flight_path/speed 给外环的能量保护。
struct ChannelFeedback {
    double theta = 0.0;       // 姿态角 θ [rad]
    double omega = 0.0;       // 角速度 ω [rad/s]
    double alpha = 0.0;       // 迎角 α = θ - γ [rad]
    double flight_path = 0.0; // 航迹角 γ [rad]
    double speed = 0.0;       // 前向速度 v [m/s]
    double disturbance = 0.0; // 集总扰动 b̂ [rad/s²]（EKF 的扰动状态；没有就填 0）
    bool valid = false;
};

struct BodyState {
    Attitude attitude;
    AngularRate rate;
    ChannelFeedback channel[kBodyAxisCount]; // 按 BodyAxis 索引（滚转通道 alpha 填 0）
    double speed = 0.0;    // 前向速度 [m/s]
    double elapsed_s = 0.0; // 从发射算起 [s]（状态机的静默段判据用）
    double altitude = -1.0; // 高度 [m]；不提供就填负数，末段判据会跳过这一条
    bool launched = false;
    bool valid = false;
};

// 视觉观测：像素坐标与预设坐标的比较结果（你的 u,v,uc,vc,du_dt,dv_dt）。
struct TargetObservation {
    double error_px[kGuideAxisCount] = {};    // e = 目标像素 − 预设像素 [px]
    double los_rate_px[kGuideAxisCount] = {}; // 目标自身像速 λ̇·f [px/s]（已扣自旋）
    double time_to_go = 0.0;                  // [s]；<=0 或非有限 = 未知
    double bbox_size = 0.0;                   // 目标框尺度（末段判据用；不用就填 0）
    bool valid = false;
};

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------

// 视觉外环：**像素误差上的 PD 滑模**。
//
// 滑模面取 PD 形式 s = ė + λ·e（λ = kp/kd，单位 1/s），到达律用带边界层的 sat：
//   Δθ = kd · clamp(s, ±φ)
// 于是：**|s| ≤ φ 内 Δθ = kp·e + kd·ė —— 就是你原来的那两条 PD 表达式**；
//       带外 Δθ 饱和在 kd·φ，那正是滑模的切换（bang-bang）段。
// 你原来的 PD 是这套滑模律的**边界层内**行为，滑模多出来的是带外的饱和切换。
struct OuterLoopParams {
    double kp = 0.005;              // [rad/px]   比例（= kd·λ）
    double kd = 0.002;              // [rad·s/px] 微分
    double boundary_px_rate = 30.0; // φ：滑模面边界层宽 [px/s]；带外即切换段
    double dead_zone_px = 5.0;      // 像素死区（你原来的 DEAD_ZONE_PIX）
    double max_offset = 0.07;       // MAX_DELTA_AOA：姿态偏置上限 [rad]（4°）

    // —— 能量保护：三重限幅，取最小 ——
    double alpha_stall = 0.14;      // 翼面失速迎角 [rad]
    double gain_per_alpha = 4.9;    // A [m/s²/rad]，把过载上限折算成迎角
    double lateral_accel_max = 8.0; // 横向加速度上限 [m/s²]
    double speed_floor = 15.0;      // 低于此速度不再修正 [m/s]
    double speed_full = 20.0;       // 达到此速度不压缩 [m/s]

    // —— 像素 → 角度 ——
    // **逐通道符号不同**：机体 z 朝下、像面 y 朝下，于是
    //   偏航：目标在右 → 像 x 变大 → ε = λ-θ 与 e 同号 → +1
    //   俯仰：目标在上 → 像 y 变小 → ε = λ-θ 与 e 反号 → -1
    // 与 predictor.cpp 的 rot_sign（水平 -1、垂直 +1）互为相反数。
    // 用错号竖直通道会变成正反馈，自检里有专门的用例。
    double pixel_sign[kGuideAxisCount] = { -1.0, 1.0 };
    double pixels_per_rad = 554.0;

    // —— 滑模面取哪个误差信号（这是本模块唯一一个"策略开关"，两档都保留）——
    //
    // false：**像素误差** ε = λ - θ。你把机头压到目标上（像素归零），但机头对上了
    //        不代表速度对上了：此时 α = σ 永久非零，脱靶量 ≈ Z·sin σ。实测在
    //        20 m/s 下留 **0.6~0.9 m 的系统性脱靶**（与初始偏差大小、射程都基本无关）。
    // true ：**航向误差** σ = λ - γ = ε + α。多出来的就是那个 α 项——也就是
    //        "像素误差 + 迎角 = 航向误差"这个恒等式。驱动 σ → 0 才是把**速度**压到
    //        视线上，脱靶量才能真的趋零（实测同算例 → 厘米级）。
    // 两者的滑模结构、边界层、死区、能量保护完全一样，只换误差信号。
    bool surface_on_heading_error = true;
};

// 姿态内环：**离散时间滑模**，一拍解出舵面（等效控制 + 到达律 + b̂ 抵消）。
struct InnerLoopParams {
    double rate_gain = 6.0;      // c：滑模面 s = (ω-ω_cmd) + c·(θ-θ_cmd) 的斜率 [1/s]
    double switch_gain = 0.6;    // K：到达律增益 [rad/s²]
    double boundary = 0.02;      // φ：边界层宽度 [rad/s]（**必须 ≥ K·tick**）
    double min_authority = 1e-3; // |Cmδ| 低于此值即认为舵效失效
    double rate_noise = 0.01;    // 陀螺噪声 [rad/s]（失效兜底判据用）
};

// 控制分配：几何是数据，代码里不写死 ± 号。
struct AllocatorParams {
    // 4 片舵面 × 3 个力矩轴，列序 = (roll, pitch, yaw)。
    // 十字/X 型尾翼下 M_x = R·k·Σδᵢ **与方位角无关** ⇒ 滚转列必然全 1；
    // M_y ∝ -Σδᵢcos φᵢ、M_z ∝ -Σδᵢsin φᵢ，φ = 45/135/225/315 给下面这组。
    // ⚠️ 上板前必须做一次舵偏阶跃确认（CONTROL_TODO.md §4）。
    double mix[kFinCount][kBodyAxisCount] = {
        { 1.0, -1.0, -1.0 }, // 舵面 1（φ = 45°）
        { 1.0, 1.0, -1.0 },  // 舵面 2（φ = 135°）
        { 1.0, 1.0, 1.0 },   // 舵面 3（φ = 225°）
        { 1.0, -1.0, 1.0 },  // 舵面 4（φ = 315°）
    };
    double servo_limit = 1.0; // 每片舵面的归一化行程上限
};

// 状态机（你的三个模式）。
enum class FlightMode {
    kFreefall, // 刚抛出：不导引，只闭环基准姿态
    kGlide,    // 姿态捕获与轨迹修正（外环工作）
    kTerminal, // 末段姿态锁死：外环偏置清零
};

const char *ModeName(FlightMode mode);

struct StateMachineParams {
    double freefall_s = 0.15;       // 静默段时长；之后且有视觉才进 kGlide
    double bbox_terminal = 0.0;     // 目标框尺度超限即进末段（<=0 = 不用这条）
    double altitude_terminal = 0.0; // 高度低于此值即进末段（<=0 = 不用这条）
    // 剩余时间低于此值即进末段（<=0 = 不用这条）。**这个值是"撞姿 vs 脱靶"的旋钮**：
    // 进了末段外环偏置就清零、机头开始往 θ_base 回摆，而回摆要时间。
    // 实测（1.0 m 装订偏差 @ 60 m，σ 表面）：
    //   t_go_terminal  0.15 s → 脱靶 0.227 m、撞点那一刻撞姿差 0.066 rad（3.8°）
    //                  0.30 s → 0.228 m、0.053 rad
    //                  0.50 s → 0.232 m、0.033 rad（1.9°）
    //                  0.80 s → 0.254 m、0.006 rad（0.3°）
    // 提前锁几乎不花脱靶量（+2%~12%），却把撞姿改善一个数量级 ⇒ 默认取 0.5 s。
    double t_go_terminal = 0.5;
};

struct ControllerParams {
    double tick = 1.0 / 500.0; // 控制拍 [s]。按拍系数都建立在它之上
    AeroParams aero;
    OuterLoopParams outer;
    InnerLoopParams inner;
    AllocatorParams allocator;
    StateMachineParams state_machine;
    double roll_ref = 0.0;  // 预设滚转（严格 0，或你装订的配平滚转）
    double pitch_ref = 0.0; // 预设俯仰 θ_base：取标称撞点航迹角
    double yaw_ref = 0.0;   // 预设偏航 ψ_base
};

inline double ReferenceAttitude(const ControllerParams &params, BodyAxis axis) {
    switch (axis) {
        case BodyAxis::kRoll: return params.roll_ref;
        case BodyAxis::kPitch: return params.pitch_ref;
        case BodyAxis::kYaw: return params.yaw_ref;
    }
    return 0.0;
}

// ---------------------------------------------------------------------------
// 输出
// ---------------------------------------------------------------------------

// 舵机输出：四片舵面的归一化指令 [-1, 1]（与仓库既有约定一致；PWM 由板端另一层做）。
struct ServoCommand {
    double fin[kFinCount] = {};
    bool saturated = false; // 任一舵面触限
};

// 外环一条通道的输出（你原来的 delta_theta_cmd / delta_psi_cmd）。
struct AxisCommand {
    double offset = 0.0;     // Δθ：最终姿态偏置 [rad]（合成时 θ_cmd = θ_base + Δθ）
    double correction = 0.0;  // Δθ_c：其中的迎角指令部分（能量保护限的是它）
    double signal = 0.0;      // E：本拍用的误差信号 [px]（像素误差或航向误差）
    double surface = 0.0;    // s：滑模面 [px/s]（调试/上报用）
    double limit = 0.0;     // 本拍生效的偏置上限 [rad]（能量保护算出来的）
    bool switching = false; // 处于切换段（|s| > φ）
    bool limited = false;   // 被能量保护限幅
    bool in_dead_zone = false;
    bool valid = false;
};

} // namespace dart::control
