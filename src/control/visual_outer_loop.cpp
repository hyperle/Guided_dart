// 视觉外环实现：PD 滑模 + 能量保护。
//
// 符号是推出来的，不是猜的（你原文也标了"符号取决于坐标系定义"）：
//   机体 z 朝下、像面 y 朝下 ⇒
//     偏航通道：目标在右 → 像 x 变大 → ε = λ-θ 与 e 同号 → pixel_sign = +1
//     俯仰通道：目标在上 → 像 y **变小** → ε = λ-θ 与 e 反号 → pixel_sign = -1
//   与 predictor.cpp 的 rot_sign（水平 -1、垂直 +1）互为相反数。
// 目标在 +ε 一侧时要把机头转/抬向它（Δθ > 0），所以本律是 **+kd·s**；
// 你原文的 `-(Kp·e + Kd·ė)` 方向是反的（会把机头转离目标），自检里钉住了正确符号。
//
// 误差信号两档（surface_on_heading_error）：
//   E = ε·f             → 机头对目标（像素归零），但留 α = σ 的永久迎角 ⇒ 脱靶 ≈ Z·sin σ
//   E = (ε+α)·f = σ·f   → 速度对视线（航向误差归零）⇒ 脱靶趋零
// 差的就是"像素误差 + 迎角 = 航向误差"这一项。

#include "control/visual_outer_loop.hpp"

#include <algorithm>
#include <cmath>

namespace dart::control {

double VisualOuterLoop::OffsetLimit(double speed) const {
    if (!(speed > 0.0) || !std::isfinite(speed)) return 0.0;

    // ① 翼面失速
    const double stall = params_.alpha_stall;
    // ② 过载 / 诱导阻力能耗：A(v)·|α| ≤ a_max
    const double gain = ScaledGainPerAlpha(aero_, speed);
    const double accel = (gain > 0.0) ? params_.lateral_accel_max / gain : 0.0;
    // ③ 低速保护：v ≤ floor 就一点都不修（此时 q ∝ v²，舵效已经塌了）
    double speed_scale = 1.0;
    if (speed <= params_.speed_floor) {
        speed_scale = 0.0;
    } else if (speed < params_.speed_full && params_.speed_full > params_.speed_floor) {
        speed_scale = (speed - params_.speed_floor) / (params_.speed_full - params_.speed_floor);
    }

    const double cap = std::min({ stall, accel, params_.max_offset });
    return std::max(0.0, speed_scale * cap);
}

AxisCommand VisualOuterLoop::Update(GuideAxis axis, const ChannelFeedback &feedback, double theta_base,
                                    double error_px, double los_rate_px) const {
    AxisCommand out;
    const double f = params_.pixels_per_rad;
    if (!feedback.valid || !(f > 0.0) || !std::isfinite(f)) return out; // valid=false：调用方按模式处理
    out.valid = true;

    const double sign = params_.pixel_sign[Index(axis)];
    const double speed = (feedback.speed > 0.0 && std::isfinite(feedback.speed)) ? feedback.speed : 0.0;

    // ---- 1) 像素 → 有符号像素误差（正 = 目标在 +角度一侧）------------------
    double raw_error = std::isfinite(error_px) ? error_px : 0.0;
    out.in_dead_zone = std::fabs(raw_error) < params_.dead_zone_px;
    if (out.in_dead_zone) raw_error = 0.0; // 死区：低速机不能把噪声当误差打舵
    const double eps_px = sign * raw_error;

    // ---- 2) 误差信号与它的导数 ---------------------------------------------
    const double rate = std::isfinite(los_rate_px) ? los_rate_px : 0.0;
    const double gain = ScaledGainPerAlpha(aero_, speed);
    double signal = eps_px;
    double signal_dot = sign * rate - f * feedback.omega; // 默认：ε̇·f（ε̇ = λ̇ - θ̇）
    if (params_.surface_on_heading_error) {
        // σ = λ - γ = ε + α ⇒ 多一项 α·f；σ̇ = λ̇ - γ̇，γ̇ = A·α/v
        signal = eps_px + f * feedback.alpha;
        const double gamma_dot = (speed > 0.0) ? gain * feedback.alpha / speed : 0.0;
        signal_dot = sign * rate - f * gamma_dot;
    }
    out.signal = signal;

    // ---- 3) 滑模面（PD 形式）与到达律 --------------------------------------
    const double lambda = (params_.kd > 0.0) ? params_.kp / params_.kd : 0.0; // [1/s]
    const double s = signal_dot + lambda * signal;
    out.surface = s;

    const double phi = params_.boundary_px_rate;
    out.switching = std::fabs(s) > phi;
    double correction = params_.kd * Clamp(s, -phi, phi); // 带内 = kp·e + kd·ė；带外 = 切换

    // ---- 4) 能量保护（限的是迎角指令，也就是这个 correction）---------------
    const double limit = OffsetLimit(speed);
    out.limit = limit;
    if (std::fabs(correction) > limit) {
        correction = Sign(correction) * limit;
        out.limited = true;
    }
    out.correction = correction;

    // ---- 5) 换算成"相对基准姿态的偏置" --------------------------------------
    // 要用上迎角 α 就得让机头站在"航迹 + α"上：θ = γ + α，所以偏置里要有 (γ - θ_base)。
    out.offset = correction;
    if (params_.surface_on_heading_error) out.offset += (feedback.flight_path - theta_base);
    return out;
}

} // namespace dart::control
