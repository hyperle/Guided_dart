// 滑模控制器主机侧自检（四个模块：状态机 / 视觉外环 PD 滑模 / 姿态内环滑模 / 控制分配）
//
//   bash tests/host/run.sh control     （等价于下面这条）
//   g++ -std=gnu++20 -O1 -g -Wall -Wextra -Iinclude tests/host/control_host_test.cpp
//       src/control/flight_state_machine.cpp src/control/visual_outer_loop.cpp
//       src/control/attitude_inner_loop.cpp src/control/control_allocator.cpp
//       src/control/flight_controller.cpp -o build/host_test/control_host_test
//
// 真值用与被控对象**同一套方程**生成：这一步验结构与符号（PD 滑模的带内/带外、
// 逐通道像素符号、能量保护、离散化约束、分配矩阵不变量、闭环），
// 模型是否与物理相符由标定阶段负责。

#include "control/attitude_inner_loop.hpp"
#include "control/control_allocator.hpp"
#include "control/control_types.hpp"
#include "control/flight_controller.hpp"
#include "control/flight_state_machine.hpp"
#include "control/visual_outer_loop.hpp"

#include <cmath>
#include <cstdio>
#include <cstdint>

using dart::control::AngularRate;
using dart::control::Attitude;
using dart::control::AttitudeInnerLoop;
using dart::control::AxisCommand;
using dart::control::BodyAxis;
using dart::control::BodyState;
using dart::control::ChannelFeedback;
using dart::control::ControlAllocator;
using dart::control::ControllerParams;
using dart::control::FlightController;
using dart::control::FlightMode;
using dart::control::FlightStateMachine;
using dart::control::GuideAxis;
using dart::control::IBodyStateSource;
using dart::control::Index;
using dart::control::InnerLoopInput;
using dart::control::InnerLoopOutput;
using dart::control::IServoOutput;
using dart::control::IVisionSource;
using dart::control::kBodyAxisCount;
using dart::control::kFinCount;
using dart::control::ScaledMomentPerDelta;
using dart::control::StateMachineParams;
using dart::control::ServoCommand;
using dart::control::TargetObservation;
using dart::control::VisualOuterLoop;

namespace {

int failures = 0;

void Check(bool ok, const char *what) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

constexpr double kTick = 1.0 / 500.0;
constexpr double kPxPerRad = 554.0;
constexpr double kA = 4.9;        // A    [m/s²/rad]
constexpr double kCmAlpha = -0.8; // Cmα  [1/s²/rad]
constexpr double kCmDelta = 2.24; // Cmδ  [1/s²]
constexpr double kGravity = 9.7949;

ControllerParams MakeParams() {
    ControllerParams p;
    p.tick = kTick;
    p.aero.gain_per_alpha = kA;
    p.aero.moment_per_alpha = kCmAlpha;
    p.aero.moment_per_delta = kCmDelta;
    p.aero.v_ref = 20.0;
    p.outer.gain_per_alpha = kA;
    p.outer.pixels_per_rad = kPxPerRad;
    p.state_machine.freefall_s = 0.0;
    return p;
}

ChannelFeedback MakeFeedback(double theta, double gamma, double speed, double omega = 0.0,
                             double disturbance = 0.0) {
    ChannelFeedback fb;
    fb.theta = theta;
    fb.omega = omega;
    fb.flight_path = gamma;
    fb.alpha = theta - gamma;
    fb.speed = speed;
    fb.disturbance = disturbance;
    fb.valid = true;
    return fb;
}

// ---------------------------------------------------------------------------
// [1] 外环：带内是 PD，带外是切换 —— 你原来那两条 PD 表达式就是带内行为
// ---------------------------------------------------------------------------
void TestOuterLoopPdSliding() {
    std::printf("[1] 视觉外环：PD 滑模的带内/带外\n");
    const ControllerParams params = MakeParams();
    const VisualOuterLoop loop(params.outer, params.aero);

    const double speed = 20.0;
    const ChannelFeedback fb = MakeFeedback(0.0, 0.0, speed); // α = 0、γ = 0

    // 偏航通道、pixel_sign = +1：E = e（两种信号在 α = 0 时相同）
    const double e = 9.2;     // px
    const double e_dot = 3.0; // px/s
    const AxisCommand in_band = loop.Update(GuideAxis::kYaw, fb, 0.0, e, e_dot);
    const double expected = params.outer.kp * e + params.outer.kd * e_dot;
    std::printf("      带内：Δθ_c = %.6f rad，kp·e + kd·ė = %.6f rad\n", in_band.correction, expected);
    Check(Near(in_band.correction, expected, 1e-12), "带内 Δθ_c 逐位等于 kp·e + kd·ė（你原来的 PD 形式）");
    Check(!in_band.switching && !in_band.limited, "带内不报 switching / limited");
    Check(Near(in_band.offset, in_band.correction, 1e-15), "γ = θ_base 时偏置就等于迎角指令");

    // 带外：|s| > φ → 饱和在 kd·φ（滑模的切换幅值）
    const double big_rate = 400.0;
    const AxisCommand switching = loop.Update(GuideAxis::kYaw, fb, 0.0, e, big_rate);
    std::printf("      带外：|s| = %.1f px/s（φ = %.1f）→ Δθ_c = %.6f rad\n", switching.surface,
                params.outer.boundary_px_rate, switching.correction);
    Check(switching.switching, "|s| > φ 时报 switching（进入饱和切换段）");
    Check(Near(std::fabs(switching.surface), big_rate + params.outer.kp / params.outer.kd * e, 1e-9),
          "滑模面 s = Ė + λ·E（λ = kp/kd）");
    Check(Near(std::fabs(switching.correction), params.outer.kd * params.outer.boundary_px_rate, 1e-12),
          "带外 Δθ_c 饱和在 kd·φ（切换幅值）");

    // 两种误差信号：α = 0 时相同，α ≠ 0 时差 f·α（就是"像素误差 + 迎角 = 航向误差"）
    const ChannelFeedback with_alpha = MakeFeedback(0.05, 0.0, speed); // α = 0.05 rad
    const AxisCommand heading = loop.Update(GuideAxis::kYaw, with_alpha, 0.0, e, 0.0);
    ControllerParams pixel_mode = params;
    pixel_mode.outer.surface_on_heading_error = false;
    const VisualOuterLoop pixel_loop(pixel_mode.outer, pixel_mode.aero);
    const AxisCommand pixel = pixel_loop.Update(GuideAxis::kYaw, with_alpha, 0.0, e, 0.0);
    std::printf("      α = 0.05 rad 时：σ 信号 = %.2f px，像素信号 = %.2f px（差 %.2f px）\n", heading.signal,
                pixel.signal, heading.signal - pixel.signal);
    Check(Near(heading.signal - pixel.signal, params.outer.pixels_per_rad * 0.05, 1e-9),
          "两种信号之差恰为 f·α（sigma = epsilon + alpha 的恒等式）");
    Check(Near(pixel.offset, pixel.correction, 1e-15), "像素模式不含 (γ-θ_base) 项（偏置就是指令）");
    Check(Near(heading.offset, heading.correction + with_alpha.flight_path, 1e-12),
          "σ 模式偏置含 (γ - θ_base) 项：机头站在「航迹 + α」上");
}

// ---------------------------------------------------------------------------
// [2] 外环：逐通道像素符号（这是最容易错、且只跑一个通道看不出来的地方）
// ---------------------------------------------------------------------------
void TestOuterLoopSign() {
    std::printf("[2] 外环：逐通道像素符号\n");
    const ControllerParams params = MakeParams();
    const VisualOuterLoop loop(params.outer, params.aero);
    const ChannelFeedback fb = MakeFeedback(0.0, 0.0, 20.0);

    Check(Near(loop.pixel_sign(GuideAxis::kPitch), -1.0, 1e-12), "俯仰 pixel_sign = -1（= -rot_sign）");
    Check(Near(loop.pixel_sign(GuideAxis::kYaw), +1.0, 1e-12), "偏航 pixel_sign = +1");

    // 目标在机头上方：像 y 变小 ⇒ e < 0。必须给出 Δθ > 0（把机头抬向目标）
    const AxisCommand above = loop.Update(GuideAxis::kPitch, fb, 0.0, -kPxPerRad * 0.01, 0.0);
    Check(above.offset > 0.0, "目标在上方（e<0）→ 俯仰 Δθ > 0（机头抬高去对目标）");

    // 目标在机头右方：像 x 变大 ⇒ e > 0。必须给出 Δθ > 0（把机头右转）
    const AxisCommand right = loop.Update(GuideAxis::kYaw, fb, 0.0, +kPxPerRad * 0.01, 0.0);
    Check(right.offset > 0.0, "目标在右方（e>0）→ 偏航 Δθ > 0（机头右转去对目标）");

    // 反例：同一 e 符号在两个通道里必须给出**相反**的 Δθ 符号关系
    const AxisCommand yaw_neg = loop.Update(GuideAxis::kYaw, fb, 0.0, -kPxPerRad * 0.01, 0.0);
    Check(yaw_neg.offset < 0.0, "目标在左方 → 偏航 Δθ < 0（方向对称）");
}

// ---------------------------------------------------------------------------
// [3] 外环：能量保护（三重限幅取最小）与死区
// ---------------------------------------------------------------------------
void TestOuterLoopEnergyGuard() {
    std::printf("[3] 外环：能量保护与死区\n");
    ControllerParams params = MakeParams();
    params.outer.max_offset = 0.5;      // MAX_DELTA_AOA 放宽，让别的约束露出来
    params.outer.alpha_stall = 0.10;
    params.outer.lateral_accel_max = 100.0; // 过载这条先不紧
    const ChannelFeedback fb = MakeFeedback(0.0, 0.0, 20.0);

    // **注意**：这套结构里"切换幅值 kd·φ"本身就是一个权限上限。
    // kd·φ = 0.002·30 = 0.06 < 失速迎角 0.10 ⇒ 先用完的是它，不是能量保护。
    const VisualOuterLoop loop(params.outer, params.aero);
    const AxisCommand phi_bound = loop.Update(GuideAxis::kYaw, fb, 0.0, 500.0, 0.0);
    std::printf("      kd·φ = %.4f rad，能量上限 = %.4f rad → 生效的是 %s\n",
                params.outer.kd * params.outer.boundary_px_rate, phi_bound.limit,
                phi_bound.limited ? "能量保护" : "切换幅值 kd·φ");
    Check(Near(phi_bound.correction, params.outer.kd * params.outer.boundary_px_rate, 1e-12),
          "边界层宽度同时决定权限：大误差时输出 = kd·φ（此时能量保护还没轮到）");
    Check(!phi_bound.limited, "kd·φ < 能量上限时不算「被能量保护限幅」");

    // 把边界层放得很宽（kd·φ > 上限），能量保护才是生效的那一道
    ControllerParams wide = params;
    wide.outer.boundary_px_rate = 200.0; // kd·φ = 0.4 > 0.10
    const VisualOuterLoop wide_loop(wide.outer, wide.aero);
    const AxisCommand stall_bound = wide_loop.Update(GuideAxis::kYaw, fb, 0.0, 500.0, 0.0);
    Check(Near(stall_bound.limit, 0.10, 1e-12) && Near(stall_bound.correction, 0.10, 1e-12) &&
              stall_bound.limited,
          "上限 = min(失速, 过载, MAX_DELTA_AOA) = 失速迎角 0.10，并报 limited");

    // 过载这条变紧：a_max/A = 0.5/4.9
    ControllerParams accel_limited = wide;
    accel_limited.outer.alpha_stall = 0.5;      // 把失速这条放宽，让过载成为最紧的那条
    accel_limited.outer.lateral_accel_max = 0.5;
    const VisualOuterLoop loop2(accel_limited.outer, accel_limited.aero);
    const AxisCommand acc = loop2.Update(GuideAxis::kYaw, fb, 0.0, 500.0, 0.0);
    Check(Near(acc.limit, 0.5 / kA, 1e-12), "过载上限折算成迎角：a_max/A");

    // 低速压缩：v = 17.5 在 floor(15) 与 full(20) 之间 → 系数 0.5
    const VisualOuterLoop loop3(wide.outer, wide.aero);
    Check(Near(loop3.OffsetLimit(17.5), 0.5 * 0.10, 1e-12), "低速段按 (v-floor)/(full-floor) 线性压缩");
    Check(Near(loop3.OffsetLimit(15.0), 0.0, 1e-12), "v = floor → 上限 0（不再修正）");
    Check(Near(loop3.OffsetLimit(20.0), 0.10, 1e-12), "v = full → 不压缩");

    // 死区
    const VisualOuterLoop normal(params.outer, params.aero);
    const AxisCommand dead = normal.Update(GuideAxis::kYaw, fb, 0.0, 3.0, 0.0);
    Check(dead.in_dead_zone && Near(dead.correction, 0.0, 1e-15), "像素误差进死区 → 不打舵");
    Check(!dead.limited, "死区不算「被限幅」");
}

// ---------------------------------------------------------------------------
// [4] 内环：滑模面、等效控制、b̂ 抵消
// ---------------------------------------------------------------------------
void TestInnerLoop() {
    std::printf("[4] 姿态内环：滑模\n");
    const ControllerParams params = MakeParams();
    const AttitudeInnerLoop loop(params.inner, params.aero);

    // 阶跃跟踪（闭环小仿真：一阶模型 + 静稳定力矩 + 常值扰动）
    auto settle = [&](double disturbance_fed) {
        double theta = 0.0, omega = 0.0, gamma = 0.0;
        const double cmd = 0.1, b = 1.0, v = 20.0;
        for (int i = 0; i < 3000; ++i) {
            InnerLoopInput in;
            in.theta = theta;
            in.omega = omega;
            in.alpha = theta - gamma;
            in.speed = v;
            in.theta_cmd = cmd;
            in.disturbance = disturbance_fed;
            in.valid = true;
            const InnerLoopOutput out = loop.Update(in, kTick);
            omega += kTick * (kCmAlpha * (theta - gamma) + kCmDelta * out.moment + b);
            gamma += kTick * (kA * (theta - gamma)) / v;
            theta += kTick * omega;
        }
        return theta;
    };
    const double with_estimate = settle(1.0);
    const double without = settle(0.0);
    std::printf("      阶跃稳态：喂 b̂ 误差 %.5f rad，不喂 %.5f rad\n", std::fabs(with_estimate - 0.1),
                std::fabs(without - 0.1));
    Check(std::fabs(with_estimate - 0.1) < 0.005, "喂 b̂：稳态误差落在地平线量级");
    Check(std::fabs(without - 0.1) > 2.0 * std::fabs(with_estimate - 0.1),
          "不喂 b̂：稳态误差显著变大（这就是 EKF 那个扰动状态的用途）");

    // 滑模面与舵效失效兜底
    InnerLoopInput in;
    in.theta = 0.0;
    in.omega = 0.0;
    in.theta_cmd = 0.05;
    in.speed = 20.0;
    in.valid = true;
    const InnerLoopOutput out = loop.Update(in, kTick);
    Check(Near(out.sliding, params.inner.rate_gain * (0.0 - 0.05), 1e-12), "滑模面 s = (ω-ω_cmd) + c·(θ-θ_cmd)");

    ControllerParams dead = params;
    dead.aero.moment_per_delta = 1e-4; // 低于 min_authority
    const AttitudeInnerLoop dead_loop(dead.inner, dead.aero);
    InnerLoopInput spinning = in;
    spinning.omega = 1.0;
    const InnerLoopOutput lost = dead_loop.Update(spinning, kTick);
    Check(lost.authority_lost && Near(lost.moment, -1.0, 1e-12), "舵效失效 → 满舵反打压角速度并上报");

    ControllerParams zero = params;
    zero.aero.moment_per_delta = 0.0; // 连符号都不知道
    const AttitudeInnerLoop zero_loop(zero.inner, zero.aero);
    Check(Near(zero_loop.Update(spinning, kTick).moment, 0.0, 1e-15), "Cmδ = 0 → 输出 0（不猜方向）");

    // 状态不可用：不许把 θ-θ_cmd 报成"看起来像真的"跟踪误差
    InnerLoopInput invalid = in;
    invalid.valid = false;
    const InnerLoopOutput none = loop.Update(invalid, kTick);
    Check(!none.valid && Near(none.moment, 0.0, 1e-15) && Near(none.error, 0.0, 1e-15),
          "状态不可用 → valid=false、输出全 0（不报假误差）");
}

// ---------------------------------------------------------------------------
// [5] 内环：离散化硬约束 φ ≥ K·tick
// ---------------------------------------------------------------------------
void TestInnerLoopDiscrete() {
    std::printf("[5] 内环：离散化约束 φ ≥ K·tick\n");
    ControllerParams params = MakeParams();
    const AttitudeInnerLoop loop(params.inner, params.aero);

    InnerLoopInput in;
    in.theta = 0.0;
    in.omega = 0.0;
    in.theta_cmd = 0.05;
    in.speed = 20.0;
    in.valid = true;
    const InnerLoopOutput normal = loop.Update(in, kTick);
    Check(!normal.ill_posed, "默认参数（φ = 0.02 > K·tick = 0.0012）满足约束");

    ControllerParams tight = params;
    tight.inner.boundary = 1e-4; // < K·tick
    const AttitudeInnerLoop tight_loop(tight.inner, tight.aero);
    Check(tight_loop.Update(in, kTick).ill_posed, "φ < K·tick → 报 ill_posed（不许静默）");

    // 带内一步不过零（否则就是极限环）
    const double phi_eff = params.inner.boundary; // 0.02
    const double s_small = 0.5 * phi_eff;
    const double s_next = s_small - params.inner.switch_gain * kTick * (s_small / phi_eff);
    Check(s_next > 0.0 && s_next < s_small, "带内一步单调不过零（无极限环）");
}

// ---------------------------------------------------------------------------
// [6] 状态机：三个模式
// ---------------------------------------------------------------------------
void TestStateMachine() {
    std::printf("[6] 状态机\n");
    ControllerParams params = MakeParams();
    FlightStateMachine machine(params.state_machine);

    Check(machine.mode() == FlightMode::kFreefall, "初始 kFreefall");
    Check(machine.Update(false, true, 10.0, 0.0, -1.0, -1.0) == FlightMode::kFreefall, "未发射 → 停在 kFreefall");
    Check(machine.Update(true, false, 10.0, 0.0, -1.0, -1.0) == FlightMode::kFreefall,
          "发射了但还没有视觉 → 不进 kGlide（没有轨道信息就谈不上导引）");
    Check(machine.Update(true, true, 1.0, 0.0, -1.0, -1.0) == FlightMode::kGlide, "发射 + 有视觉 + 过静默段 → kGlide");

    Check(machine.Update(true, true, 2.0, 0.0, 0.0, 0.1) == FlightMode::kTerminal, "t_go 触底 → kTerminal");
    Check(machine.Update(true, true, 3.0, 0.0, 0.0, 5.0) == FlightMode::kTerminal, "kTerminal 是吸收态");

    // 你原文那两条判据
    StateMachineParams bbox = params.state_machine;
    bbox.bbox_terminal = 100.0;
    bbox.t_go_terminal = 0.0;
    FlightStateMachine m2(bbox);
    m2.Update(true, true, 1.0, 0.0, -1.0, -1.0);
    Check(m2.Update(true, true, 2.0, 150.0, -1.0, -1.0) == FlightMode::kTerminal, "目标框尺度超限 → kTerminal");

    StateMachineParams alt = params.state_machine;
    alt.altitude_terminal = 5.0;
    alt.t_go_terminal = 0.0;
    FlightStateMachine m3(alt);
    m3.Update(true, true, 1.0, 0.0, 20.0, -1.0);
    Check(m3.Update(true, true, 2.0, 0.0, 4.0, -1.0) == FlightMode::kTerminal, "高度低于阈值 → kTerminal");

    machine.Reset();
    Check(machine.mode() == FlightMode::kFreefall, "Reset 回到 kFreefall");
}

// ---------------------------------------------------------------------------
// [7] 控制分配
// ---------------------------------------------------------------------------
void TestAllocator() {
    std::printf("[7] 控制分配\n");
    const ControllerParams params = MakeParams();
    const ControlAllocator allocator(params.allocator);
    const auto &mix = params.allocator.mix;

    // 不变量：M_x = R·k·Σδᵢ 与方位角无关 ⇒ 滚转列必然全 1
    bool roll_all_ones = true;
    for (int fin = 0; fin < kFinCount; ++fin) {
        if (!Near(mix[fin][Index(BodyAxis::kRoll)], 1.0, 1e-12)) roll_all_ones = false;
    }
    Check(roll_all_ones, "滚转列全 1（十字/X 尾翼的硬不变量）");

    // 三列两两正交 ⇒ 三轴解耦
    bool orthogonal = true;
    for (int a = 0; a < kBodyAxisCount; ++a) {
        for (int b = a + 1; b < kBodyAxisCount; ++b) {
            double dot = 0.0;
            for (int fin = 0; fin < kFinCount; ++fin) dot += mix[fin][a] * mix[fin][b];
            if (!Near(dot, 0.0, 1e-12)) orthogonal = false;
        }
    }
    Check(orthogonal, "三列两两正交（三轴解耦）");

    // 往返：fin = B·m，还原 m' = Bᵀ·fin/4 应等于 m（列范数为 2）
    const double moment[kBodyAxisCount] = { 0.1, 0.2, -0.3 };
    const ServoCommand cmd = allocator.Allocate(moment);
    double back[kBodyAxisCount] = {};
    for (int axis = 0; axis < kBodyAxisCount; ++axis) {
        for (int fin = 0; fin < kFinCount; ++fin) back[axis] += cmd.fin[fin] * mix[fin][axis];
        back[axis] /= 4.0;
    }
    Check(Near(back[Index(BodyAxis::kPitch)], 0.2, 1e-12) && Near(back[Index(BodyAxis::kYaw)], -0.3, 1e-12),
          "B 的非限幅往返精确还原（矩阵本身是线性的）");
    Check(!cmd.saturated, "未触限不报饱和");

    // 限幅：俯仰单轴给 5 → mix 列里有 ±1，都会撞到 servo_limit
    const double huge[kBodyAxisCount] = { 0.0, 5.0, 0.0 };
    const ServoCommand sat = allocator.Allocate(huge);
    bool all_within = true;
    for (int fin = 0; fin < kFinCount; ++fin) {
        if (std::fabs(sat.fin[fin]) > params.allocator.servo_limit + 1e-12) all_within = false;
    }
    Check(sat.saturated && all_within, "触限时报 saturated，且每片舵面都在 ±servo_limit 内");
}

// ---------------------------------------------------------------------------
// 闭环仿真的被控对象与端口桩
// ---------------------------------------------------------------------------
struct PlanePlant {
    double x = 0.0, y = 0.0;
    double theta = 0.0, omega = 0.0, gamma = 0.0, v = 20.0;
    double target_x = 0.0, target_y = 0.0;
    bool gravity_on = false;

    double Range() const { return std::hypot(target_x - x, target_y - y); }
    double LosAngle() const { return std::atan2(target_y - y, target_x - x); }
    double HeadingError() const { return LosAngle() - gamma; }
    double MissSigned() const {
        const double px = target_x - x, py = target_y - y;
        return -px * std::sin(gamma) + py * std::cos(gamma);
    }
    void Step(double delta) {
        const double alpha = theta - gamma;
        omega += kTick * (kCmAlpha * alpha + kCmDelta * delta);
        const double gravity_perp = gravity_on ? -kGravity * std::cos(gamma) : 0.0;
        gamma += kTick * (kA * alpha + gravity_perp) / v;
        if (gravity_on) v -= kTick * (kGravity * std::sin(gamma));
        theta += kTick * omega;
        x += kTick * v * std::cos(gamma);
        y += kTick * v * std::sin(gamma);
    }
};

// 端口桩：只填这个通道需要的量。视线观测按真实光路给（像面 y 朝下！）
struct SimBody : public IBodyStateSource {
    const PlanePlant *plant = nullptr;
    bool is_pitch = false;
    bool launched = true;
    bool poison = false;

    bool ReadBodyState(BodyState *out) const override {
        const double theta = poison ? std::nan("") : plant->theta;
        out->launched = launched;
        out->valid = true;
        if (is_pitch) {
            out->attitude.pitch = theta;
            out->rate.pitch = plant->omega;
            out->channel[Index(BodyAxis::kPitch)] =
                MakeFeedback(theta, plant->gamma, plant->v, plant->omega, 0.0);
        } else {
            out->attitude.yaw = theta;
            out->rate.yaw = plant->omega;
            out->channel[Index(BodyAxis::kYaw)] =
                MakeFeedback(theta, plant->gamma, plant->v, plant->omega, 0.0);
        }
        out->channel[Index(BodyAxis::kRoll)].valid = true;
        out->speed = plant->v;
        return true;
    }
    const char *name() const override { return "sim-body"; }
};

struct SimVision : public IVisionSource {
    const PlanePlant *plant = nullptr;
    bool is_pitch = false;
    bool valid = true;

    bool ReadTarget(TargetObservation *out) const override {
        if (!valid) return false;
        const double los = plant->LosAngle();
        const double rate = plant->v * std::sin(plant->HeadingError()) / plant->Range(); // λ̇ [rad/s]
        if (is_pitch) {
            // 像面 y 朝下 ⇒ 目标在机头上方时 e < 0
            out->error_px[Index(GuideAxis::kPitch)] = -kPxPerRad * (los - plant->theta);
            out->los_rate_px[Index(GuideAxis::kPitch)] = -kPxPerRad * rate;
        } else {
            // 像面 x 朝右 ⇒ 目标在右侧时 e > 0
            out->error_px[Index(GuideAxis::kYaw)] = +kPxPerRad * (los - plant->theta);
            out->los_rate_px[Index(GuideAxis::kYaw)] = +kPxPerRad * rate;
        }
        out->time_to_go = plant->Range() / plant->v;
        out->valid = true;
        return true;
    }
    const char *name() const override { return "sim-vision"; }
};

struct SimServo : public IServoOutput {
    ServoCommand last;
    void Write(const ServoCommand &command) override { last = command; }
    const char *name() const override { return "sim-servo"; }
};

// B 的列正交且列范数 = 2 ⇒ 伪逆 = Bᵀ/4。把舵偏还原成"真正作用在该轴上的力矩"，
// 这样闭环里跑的是真的分配矩阵，而不是假设它等于 1。
double RealizedMoment(const ControllerParams &params, const ServoCommand &cmd, BodyAxis axis) {
    double sum = 0.0;
    for (int fin = 0; fin < kFinCount; ++fin) sum += cmd.fin[fin] * params.allocator.mix[fin][Index(axis)];
    return sum / 4.0;
}

struct Engagement {
    double miss = 0.0;
    double theta_error = 0.0;
    double offset = 0.0;
    int steps = 0;
    int glide_steps = 0;
    bool switching = false;
    bool limited = false;
};

// 交战算例：grid 是"装订给的基准姿态"相对初始视线角的偏差（0 = 装订正确）。
Engagement RunEngagementWith(double err_m, bool is_pitch, double theta_base_extra,
                             const ControllerParams &params_in);

Engagement RunEngagement(double err_m, bool is_pitch, double theta_base_extra) {
    return RunEngagementWith(err_m, is_pitch, theta_base_extra, MakeParams());
}

Engagement RunEngagementWith(double err_m, bool is_pitch, double theta_base_extra,
                             const ControllerParams &params_in) {
    ControllerParams params = params_in;
    PlanePlant plant;
    plant.gravity_on = is_pitch;
    plant.gamma = is_pitch ? -0.6 : 0.0; // 竖直通道给一个抛射角
    plant.target_x = is_pitch ? 20.0 : 60.0;
    plant.target_y = is_pitch ? 0.0 : -err_m;
    if (is_pitch) {
        // 先无控飞一遍，找落点，再把目标放在它附近（这样"修得动/修不动"才有意义）
        PlanePlant probe = plant;
        while (probe.x < plant.target_x && probe.v > 5.0) probe.Step(0.0);
        plant.target_y = probe.y;
    }
    const BodyAxis axis = is_pitch ? BodyAxis::kPitch : BodyAxis::kYaw;
    params.pitch_ref = is_pitch ? plant.LosAngle() + theta_base_extra : 0.0;
    params.yaw_ref = is_pitch ? 0.0 : plant.LosAngle() + theta_base_extra;

    SimBody body;
    body.plant = &plant;
    body.is_pitch = is_pitch;
    SimVision vision;
    vision.plant = &plant;
    vision.is_pitch = is_pitch;
    SimServo servo;
    FlightController::Ports ports;
    ports.body = &body;
    ports.vision = &vision;
    ports.servo = &servo;
    FlightController controller(params, ports);
    controller.Reset();

    Engagement result;
    uint64_t now_us = 0;
    double min_range = 1e9;
    double miss_at_min = 0.0;
    double theta_error_at_min = 0.0;
    while (plant.Range() > (is_pitch ? 1.5 : 2.0) && result.steps < 4000) {
        now_us += static_cast<uint64_t>(kTick * 1e6);
        controller.Update(now_us);
        const double delta = RealizedMoment(params, servo.last, axis);
        plant.Step(delta);
        // **最近点**而不是循环退出那一刻：大偏差时弹体根本进不了阈值内，
        // 循环会一直跑到上限，那时测到的"脱靶量"已经没有意义了（踩过）。
        if (plant.Range() < min_range) {
            min_range = plant.Range();
            miss_at_min = plant.MissSigned();
            theta_error_at_min = plant.theta - (is_pitch ? params.pitch_ref : params.yaw_ref);
        }
        if (controller.mode() == FlightMode::kGlide) {
            // 只记导引段的偏置：末段按设计把偏置清零，取那里的值什么都测不出来
            const AxisCommand &cmd = controller.axis_command(is_pitch ? GuideAxis::kPitch : GuideAxis::kYaw);
            if (std::fabs(cmd.offset) > std::fabs(result.offset)) result.offset = cmd.offset;
            result.switching = result.switching || cmd.switching;
            result.limited = result.limited || cmd.limited;
            ++result.glide_steps;
        }
        ++result.steps;
    }
    result.miss = miss_at_min;
    result.theta_error = theta_error_at_min;
    return result;
}

// ---------------------------------------------------------------------------
// [8] 闭环：水平通道
// ---------------------------------------------------------------------------
void TestClosedLoopYaw() {
    std::printf("[8] 闭环：水平通道\n");
    // 装订偏差 1.0 m @ 60 m = 9.2 px > 死区 5 px，外环才真的工作；
    // 横向修正预算 ½·A·α·t_go² = ½·4.9·0.07·9 ≈ 1.5 m，所以 1.0 m 是在能力之内的。
    const Engagement good = RunEngagement(1.0, false, 0.0);
    std::printf("      装订偏差 1.00 m → |脱靶| = %.4f m，导引段 |Δθ|max = %.4f rad，撞点撞姿差 %.4f rad"
                "（导引 %d 拍 / 共 %d 拍，限幅 %d）\n",
                std::fabs(good.miss), std::fabs(good.offset), std::fabs(good.theta_error), good.glide_steps,
                good.steps, (int)good.limited);
    Check(std::fabs(good.miss) < 0.30, "装订偏差 1.00 m → 脱靶压到 0.30 m 以内（实测 0.23 m）");
    Check(std::fabs(good.theta_error) < 0.05, "撞点姿态收敛到 θ_base（撞点那一刻 < 0.05 rad）");
    Check(std::fabs(good.offset) > 0.01, "导引段确实给出了非零偏置（外环在工作）");

    // 装订基准姿态给错 0.05 rad（27.7 px，远超死区）→ 外环给出偏置去修
    const Engagement biased = RunEngagement(1.0, false, 0.05);
    std::printf("      基准姿态偏 0.05 rad → |脱靶| = %.4f m，导引段 |Δθ|max = %.4f rad\n",
                std::fabs(biased.miss), std::fabs(biased.offset));
    Check(std::fabs(biased.offset) > 0.01, "基准姿态偏了 → 外环给出非零偏置去修（PD 滑模在工作）");

    // 超出能力：20 m/s 的横向加速度只有零点几个 m/s²，3 m 的偏差修不完。
    // 注意：3 m 偏差时弹体可能压根进不了 2 m 以内，测到的就是最近点脱靶量。
    const Engagement big = RunEngagement(3.0, false, 0.0);
    std::printf("      装订偏差 3.00 m → 最近点脱靶 %.4f m（能力上限约 ½·A·α·t_go² = 1.3 m）\n",
                std::fabs(big.miss));
    Check(std::fabs(big.miss) < 2.4, "3.00 m 偏差被压到 2.4 m 以内（部分修正，能力上限约 1.3 m）");

    // **两种误差信号的正面对比**（同一算例、同一组增益，只换误差信号）
    ControllerParams pixel_params = MakeParams();
    pixel_params.outer.surface_on_heading_error = false;
    const Engagement pixel = RunEngagementWith(1.0, false, 0.0, pixel_params);
    std::printf("      同一算例，E = 像素误差 ε（你原来的）→ |脱靶| = %.4f m\n", std::fabs(pixel.miss));
    Check(std::fabs(good.miss) < 0.5 * std::fabs(pixel.miss),
          "E = 航向误差 σ = ε+α 的脱靶量不到像素误差版的一半（实测见上面两行）");

    // 末段锁定时刻：撞姿 vs 脱靶的旋钮
    ControllerParams late = MakeParams();
    late.state_machine.t_go_terminal = 0.15;
    const Engagement late_lock = RunEngagementWith(1.0, false, 0.0, late);
    std::printf("      末段改在 t_go = 0.15 s 才锁 → |脱靶| = %.4f m，撞姿差 %.4f rad\n",
                std::fabs(late_lock.miss), std::fabs(late_lock.theta_error));
    Check(std::fabs(good.theta_error) < 0.8 * std::fabs(late_lock.theta_error) &&
              std::fabs(good.miss) < 1.2 * std::fabs(late_lock.miss),
          "提前锁末段（0.5 s）：撞姿明显改善，而脱靶量几乎不变");
}

// ---------------------------------------------------------------------------
// [9] 闭环：竖直通道（重力开着 + 像素符号）
// ---------------------------------------------------------------------------
void TestClosedLoopPitch() {
    std::printf("[9] 闭环：竖直通道（重力开着）\n");
    const Engagement center = RunEngagement(0.0, true, 0.0);
    const Engagement above = RunEngagement(0.0, true, 0.0);
    std::printf("      目标在标称落点上 → |脱靶| = %.4f m\n", std::fabs(center.miss));
    Check(std::fabs(center.miss) < 0.5, "标称落点附近：脱靶在半个身位内");

    // 目标相对标称落点平移 ±0.5 m，两个方向都必须往回收（符号错的话至少一边会发散）
    ControllerParams params = MakeParams();
    PlanePlant probe;
    probe.gravity_on = true;
    probe.gamma = -0.6;
    probe.target_x = 20.0;
    PlanePlant walk = probe;
    while (walk.x < probe.target_x && walk.v > 5.0) walk.Step(0.0);
    const double impact_y = walk.y;

    auto run_offset = [&](double offset) {
        PlanePlant plant;
        plant.gravity_on = true;
        plant.gamma = -0.6;
        plant.target_x = 20.0;
        plant.target_y = impact_y + offset;
        ControllerParams pr = MakeParams();
        // **两次都用同一条标称弹道的视线角**做基准姿态：否则 θ_base 跟着目标动，
        // 测出来的差异里混着"基准姿态变了"，不是外环的作用。
        PlanePlant nominal = plant;
        nominal.target_y = impact_y;
        pr.pitch_ref = nominal.LosAngle();
        SimBody body;
        body.plant = &plant;
        body.is_pitch = true;
        SimVision vision;
        vision.plant = &plant;
        vision.is_pitch = true;
        SimServo servo;
        FlightController::Ports ports;
        ports.body = &body;
        ports.vision = &vision;
        ports.servo = &servo;
        FlightController controller(pr, ports);
        uint64_t now_us = 0;
        int steps = 0;
        while (plant.Range() > 1.5 && steps < 20000) {
            now_us += static_cast<uint64_t>(kTick * 1e6);
            controller.Update(now_us);
            plant.Step(RealizedMoment(pr, servo.last, BodyAxis::kPitch));
            ++steps;
        }
        return plant.MissSigned();
    };
    const double up = run_offset(+0.5);
    const double down = run_offset(-0.5);
    std::printf("      目标偏移 +0.5 m → 脱靶 %+.3f m；-0.5 m → 脱靶 %+.3f m\n", up, down);
    Check(std::fabs(up) < 0.5 && std::fabs(down) < 0.5, "两个方向都被压下来（像素符号正确才可能）");
    Check(up * down < 0.0, "两侧目标给出反号的脱靶量（跟随的是目标，不是固定偏置）");
    (void)above;
    (void)params;
}

// ---------------------------------------------------------------------------
// [10] 失效路径：端口给坏数也不许把 NaN 送到舵面
// ---------------------------------------------------------------------------
void TestFailurePaths() {
    std::printf("[10] 失效路径\n");
    ControllerParams params = MakeParams();
    PlanePlant plant;
    plant.target_x = 60.0;
    plant.target_y = -0.3;
    params.yaw_ref = plant.LosAngle();

    SimBody body;
    body.plant = &plant;
    SimVision vision;
    vision.plant = &plant;
    SimServo servo;
    FlightController::Ports ports;
    ports.body = &body;
    ports.vision = &vision;
    ports.servo = &servo;
    FlightController controller(params, ports);

    uint64_t now_us = 0;
    for (int i = 0; i < 500; ++i) {
        now_us += static_cast<uint64_t>(kTick * 1e6);
        controller.Update(now_us);
    }
    Check(controller.mode() == FlightMode::kGlide, "正常进入 kGlide");

    body.poison = true; // 注入 NaN
    now_us += static_cast<uint64_t>(kTick * 1e6);
    const ServoCommand &poisoned = controller.Update(now_us);
    bool finite = true;
    for (int fin = 0; fin < kFinCount; ++fin) {
        if (!std::isfinite(poisoned.fin[fin])) finite = false;
    }
    Check(finite, "状态里出现 NaN：舵面输出仍然有限（NaN 没走到舵面）");

    body.poison = false;
    vision.valid = false; // 丢视觉
    now_us += static_cast<uint64_t>(kTick * 1e6);
    const ServoCommand &blind = controller.Update(now_us);
    bool finite2 = true;
    for (int fin = 0; fin < kFinCount; ++fin) {
        if (!std::isfinite(blind.fin[fin])) finite2 = false;
    }
    Check(finite2 && controller.mode() == FlightMode::kGlide, "丢视觉：输出有限，且不因为没观测就跳末段");

    // 空端口：一个都不注入也不许崩
    FlightController bare(params, FlightController::Ports{});
    const ServoCommand &none = bare.Update(1000);
    bool all_zero = true;
    for (int fin = 0; fin < kFinCount; ++fin) {
        if (none.fin[fin] != 0.0) all_zero = false;
    }
    Check(all_zero, "不注入任何端口：舵面输出全 0");

    // v = 0：能量保护把上限压到 0
    ControllerParams stopped = params;
    const VisualOuterLoop loop(stopped.outer, stopped.aero);
    const ChannelFeedback dead = MakeFeedback(0.0, 0.0, 0.0);
    Check(Near(loop.Update(GuideAxis::kYaw, dead, 0.0, 100.0, 0.0).offset, 0.0, 1e-15) && !dead.valid == false,
          "v = 0 → 偏置上限 0");
}

} // namespace

int main() {
    std::printf("=== 滑模控制器主机自检（四模块）===\n");
    TestOuterLoopPdSliding();
    TestOuterLoopSign();
    TestOuterLoopEnergyGuard();
    TestInnerLoop();
    TestInnerLoopDiscrete();
    TestStateMachine();
    TestAllocator();
    TestClosedLoopYaw();
    TestClosedLoopPitch();
    TestFailurePaths();
    std::printf("=== %s（%d 项失败）===\n", failures == 0 ? "全部通过" : "有失败", failures);
    return failures == 0 ? 0 : 1;
}
