// 编排实现：取数 → 状态机 → 外环 → 合成 θ_cmd → 内环 → 分配 → 下发。
//
// 端口边界上的两件事（都在这里一次做完，散在各处一定会漏）：
//   ① 非有限值（NaN/inf）判为状态不可用：NaN 会一路穿过限幅到舵面，
//      而且会让"限幅比较"为假，下一拍舵面可能跳一整程；
//   ② 视觉无效时**不要把误差当 0**：那样外环会以为已经对准了。上游如实告知，
//      本拍外环就不工作（偏置按 0 处理），模式机也不会因为你没观测就跳到末段。

#include "control/flight_controller.hpp"

#include <cmath>

namespace dart::control {

namespace {

bool StateIsFinite(const BodyState &state) {
    const double values[] = {
        state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
        state.rate.roll,     state.rate.pitch,     state.rate.yaw,
        state.speed,
    };
    for (double value : values) {
        if (!std::isfinite(value)) return false;
    }
    for (const ChannelFeedback &channel : state.channel) {
        if (!std::isfinite(channel.theta) || !std::isfinite(channel.omega) || !std::isfinite(channel.alpha) ||
            !std::isfinite(channel.flight_path) || !std::isfinite(channel.speed) ||
            !std::isfinite(channel.disturbance)) {
            return false;
        }
    }
    return true;
}

} // namespace

FlightController::FlightController(const ControllerParams &params, const Ports &ports)
    : params_(params)
    , ports_(ports)
    , state_machine_(params.state_machine)
    , outer_loop_(params.outer, params.aero)
    , inner_loop_(params.inner, params.aero)
    , allocator_(params.allocator) {}

void FlightController::Reset() {
    state_machine_.Reset();
    axis_[0] = AxisCommand{};
    axis_[1] = AxisCommand{};
    inner_[0] = InnerLoopOutput{};
    inner_[1] = InnerLoopOutput{};
    command_ = ServoCommand{};
}

const ServoCommand &FlightController::Update(uint64_t now_us) {
    (void)now_us; // 本层不用时间戳：拍长按 params.tick（按拍系数约定），相位判据用外部给的事实
    const double tick = (params_.tick > 0.0 && std::isfinite(params_.tick)) ? params_.tick : 1.0 / 500.0;

    // ---- 1) 取数 + 端口边界清洗 -------------------------------------------
    BodyState body;
    const bool body_ok = (ports_.body != nullptr) && ports_.body->ReadBodyState(&body);
    const bool state_ok = body_ok && body.valid && StateIsFinite(body);

    TargetObservation target;
    const bool vision_ok = (ports_.vision != nullptr) && ports_.vision->ReadTarget(&target) && target.valid;
    if (vision_ok) {
        for (int i = 0; i < kGuideAxisCount; ++i) {
            if (!std::isfinite(target.error_px[i])) target.error_px[i] = 0.0;
            if (!std::isfinite(target.los_rate_px[i])) target.los_rate_px[i] = 0.0;
        }
        if (!(target.time_to_go > 0.0) || !std::isfinite(target.time_to_go)) target.time_to_go = -1.0;
    }

    // ---- 2) 状态机 --------------------------------------------------------
    const FlightMode mode = state_machine_.Update(state_ok && body.launched, vision_ok, body.elapsed_s,
                                                  target.bbox_size, body.altitude, target.time_to_go);

    // ---- 3) 视觉外环（只有 kGlide 工作；其余模式偏置按 0）------------------
    const bool outer_active = (mode == FlightMode::kGlide) && state_ok && vision_ok;
    for (GuideAxis axis : kGuideAxes) {
        const int i = Index(axis);
        const BodyAxis body_axis = ToBodyAxis(axis);
        const ChannelFeedback &feedback = body.channel[Index(body_axis)];
        if (outer_active) {
            axis_[i] = outer_loop_.Update(axis, feedback, ReferenceAttitude(params_, body_axis),
                                          target.error_px[i], target.los_rate_px[i]);
        } else {
            axis_[i] = AxisCommand{}; // 偏置清零：kFreefall/kTerminal 就是"锁基准姿态"
        }
    }

    // ---- 4) 姿态内环：三轴同一套滑模 ----------------------------------------
    // 俯仰/偏航的指令 = 基准姿态 + 外环偏置；滚转不参与轨道修正，指令恒为 roll_ref。
    double moment[kBodyAxisCount] = {};
    for (BodyAxis body_axis : kBodyAxes) {
        const int i = Index(body_axis);
        double theta_cmd = ReferenceAttitude(params_, body_axis);
        if (body_axis == BodyAxis::kPitch) theta_cmd += axis_[Index(GuideAxis::kPitch)].offset;
        if (body_axis == BodyAxis::kYaw) theta_cmd += axis_[Index(GuideAxis::kYaw)].offset;

        InnerLoopInput in;
        in.theta = Value(body.attitude, body_axis);
        in.omega = Value(body.rate, body_axis);
        in.alpha = body.channel[i].alpha;
        in.speed = (body.channel[i].speed > 0.0) ? body.channel[i].speed : body.speed;
        in.theta_cmd = theta_cmd;
        in.disturbance = body.channel[i].disturbance;
        in.valid = state_ok;

        const InnerLoopOutput out = inner_loop_.Update(in, tick);
        moment[i] = out.moment;
        if (body_axis == BodyAxis::kPitch) inner_[Index(GuideAxis::kPitch)] = out;
        if (body_axis == BodyAxis::kYaw) inner_[Index(GuideAxis::kYaw)] = out;
    }

    // ---- 5) 控制分配 + 下发 ------------------------------------------------
    command_ = allocator_.Allocate(moment);
    if (ports_.servo != nullptr) ports_.servo->Write(command_);
    return command_;
}

} // namespace dart::control
