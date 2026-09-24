#pragma once

namespace dart::control {

struct BodyRate {
    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};

// ---------------------------------------------------------------------------
// 数据源接口：IMU/姿态解算模块（另一个文件里的另一个类）实现它。
// 不直接 include 那个模块，是为了让 ESO 不依赖任何具体传感器。
// ---------------------------------------------------------------------------
class IBodyRateSource {
public:
    virtual ~IBodyRateSource() = default;

    // 取一帧角速度。
    virtual void read_body_rate(BodyRate &out);

    virtual const char *name() const = 0; // 日志/取证
};

class ExtendedState {
public:
    // ---- 唯一入口：从数据源拉一帧，更新基础运动学状态 --------------------
    bool state_update(IBodyRateSource &src);

    // ---- 不带源对象的直接送值版本（主机自检 / 回放 / 上层已解算好的场合）----
    void state_update(const BodyRate &rate);

    double roll_velocity() const { return roll_velocity_; }
    double pitch_velocity() const { return pitch_velocity_; }
    double yaw_velocity() const { return yaw_velocity_; }

    // 线速度暂无数据源（后续接 GNSS/视觉/空速），如实暴露 0
    double velocity_x() const { return velocity_x_; }
    double velocity_y() const { return velocity_y_; }
    double velocity_z() const { return velocity_z_; }

private:
    void apply_body_rate(const BodyRate &rate);

    // 集总扰动
    double yaw_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double roll_disturbance = 0.0;

    // 未知气动参数 base on recent velocity
    double yaw_control_effectiveness = 0.0;
    double pitch_control_effectiveness = 0.0;
    double roll_control_effectiveness = 0.0;
    double incidence_angle = 0.0;
    double sideslip_angle = 0.0;

    // 基础运动学状态
    double yaw_velocity_ = 0.0;
    double pitch_velocity_ = 0.0;
    double roll_velocity_ = 0.0;
    double velocity_x_ = 0.0;
    double velocity_y_ = 0.0;
    double velocity_z_ = 0.0;

    bool has_state_ = false; // 从未拿到过有效角速度之前，状态是"未知"而不是"零"

    static constexpr double gravity = 9.7949;
};

// ---------------------------------------------------------------------------
// 实现（header-only：这里是纯赋值/转发，还没有值得单开 .cpp 的算法）
// ---------------------------------------------------------------------------
inline bool ExtendedState::state_update(IBodyRateSource &src) {
    BodyRate rate;
    apply_body_rate(rate);
    return true;
}

inline void ExtendedState::state_update(const BodyRate &rate) {
    apply_body_rate(rate);
}

inline void ExtendedState::apply_body_rate(const BodyRate &rate) {
    // 逐步施工：这一步只做"忠实搬运"，不做任何滤波/限幅/符号修正。（非本类职责）
    roll_velocity_  = rate.roll;
    pitch_velocity_ = rate.pitch;
    yaw_velocity_   = rate.yaw;
    has_state_      = true;
}


class IMU_calculation_operator {
public:

private:
};

} // namespace dart::control
