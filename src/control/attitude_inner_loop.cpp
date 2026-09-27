// 姿态内环实现。逐项对照头文件里的推导：
//
//   s⁺ - s = tick·(Cmα·α + Cmδ·δ + b̂) + c·tick·(ω - ω_cmd)
//   令 s⁺ = s_next  ⇒  Cmδ·δ = (s_next - s)/tick - Cmα·α - b̂ - c·(ω - ω_cmd)

#include "control/attitude_inner_loop.hpp"

#include <algorithm>
#include <cmath>

namespace dart::control {

InnerLoopOutput AttitudeInnerLoop::Update(const InnerLoopInput &in, double tick) const {
    InnerLoopOutput out;

    // 状态不可用：**不要把 θ-θ_cmd 报成一个看起来像真的跟踪误差**，
    // 力矩指令按 0 处理，由调用方决定（本拍不驱动舵面）。valid=false 会如实上报。
    if (!in.valid || !(tick > 0.0) || !std::isfinite(tick)) return out;
    out.valid = true;
    out.error = in.theta - in.theta_cmd;

    const double speed = (in.speed > 0.0 && std::isfinite(in.speed)) ? in.speed : 0.0;
    const double m_alpha = ScaledMomentPerAlpha(aero_, speed);
    const double m_delta = ScaledMomentPerDelta(aero_, speed);

    // ---- 1) 滑模面 ---------------------------------------------------------
    const double rate_error = in.omega - in.omega_cmd;
    const double s = rate_error + params_.rate_gain * out.error;
    out.sliding = s;

    // ---- 2) 舵效失效的兜底：不解等效控制 ------------------------------------
    if (!(std::fabs(m_delta) > params_.min_authority)) {
        out.authority_lost = true;
        // 方向由标定过的 Cmδ 符号给出；角速度已在地平线以下就不打（省舵机）。
        if (std::fabs(in.omega) > params_.rate_noise) {
            out.moment = Saturate(-Sign(in.omega) * Sign(m_delta));
        }
        return out;
    }

    // ---- 3) 到达律（带边界层），并解出这一拍的 δ ----------------------------
    const double phi = std::max(params_.boundary, params_.switch_gain * tick);
    out.ill_posed = params_.boundary < params_.switch_gain * tick;
    const double s_next = s - params_.switch_gain * tick * Saturate(s / phi);

    const double command =
        ((s_next - s) / tick - m_alpha * in.alpha - in.disturbance - params_.rate_gain * rate_error) / m_delta;
    out.moment = Saturate(command);
    out.saturated = std::fabs(command) > 1.0;
    return out;
}

} // namespace dart::control
