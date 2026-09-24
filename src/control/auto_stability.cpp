#include "capture/imu_manager.h"
#include "pid_calculator.hpp"

#include <cmath>
#include <limits>

namespace dart::control {

// 一个环的整定值
struct PidTuning {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double i_limit = 0.0;   // 积分项**贡献**上限（ki·I），不是积分状态上限
    double out_limit = 0.0; // 必须 >= 0
    double err_band = 0.0;  // |误差| 出带 → 积分清零（split 带）；0 = 不设带
};

// 依赖注入：整定值 → PidCalculator 字段。这种换算只在本文件里出现。
void inject_pid(const PidTuning& t, PidCalculator* pid) {
    pid->kp = t.kp;
    pid->ki = t.ki;
    pid->kd = t.kd;

    pid->output_min = -t.out_limit;
    pid->output_max = t.out_limit;

    // PidCalculator 限的是积分状态 I，整定时给的是它的贡献，所以反算。
    const double i_state = (t.ki != 0.0) ? t.i_limit / std::fabs(t.ki) : 0.0;
    pid->integral_min = -i_state;
    pid->integral_max = i_state;

    const double band = (t.err_band > 0.0) ? t.err_band : std::numeric_limits<double>::infinity();
    pid->integral_split_min = -band;
    pid->integral_split_max = band;

    pid->reset(); // 积分状态是在旧 ki 下累起来的，换了整定值就没有意义
}

// IMU 慢轨消费接口（传 imu_get_delta_angle_between）；注入而不是直连全局符号，主机上才能塞假源
using imu_delta_angle_fn = bool (*)(uint64_t, uint64_t, float*);

class RollStabilizer {
public:
    // 输入角度误差(rad) → 期望角速度(rad/s)，output_min/max 就是期望角速度限幅
    PidCalculator angle;
    // 输入角速度误差(rad/s) → 归一化指令，output_min/max 就是执行机构行程
    PidCalculator rate;

    void set_imu_source(imu_delta_angle_fn read_delta) { read_delta_ = read_delta; }

    // 整定值由外部注入，本类不写死 kp/ki/kd
    void set_angle_pid(const PidTuning& t) { inject_pid(t, &angle); }
    void set_rate_pid(const PidTuning& t) { inject_pid(t, &rate); }

    void reset() {
        angle.reset();
        rate.reset();
        last_tick_us_ = 0;
        out_ = 0.0;
    }

    // 角速度取自**本拍自己的时间窗** [上一拍, 这一拍] 的角度增量 ÷ 窗长：
    // 窗口跟着控制环走，所以 IMU 采样抖动和"控制环比 IMU 快/慢"都不用在本类里处理。
    // 姿态角不在 IMU 里（它来自四元数解算），仍由调用方给。
    double update(double target_roll, double roll, uint64_t timestamp_us) {
        if (last_tick_us_ == 0) { // 第一拍没有窗口，只记基准
            last_tick_us_ = timestamp_us;
            return out_;
        }
        if (timestamp_us <= last_tick_us_) return out_; // 时间戳没往前走：不是新的一拍

        const uint64_t start_us = last_tick_us_;
        last_tick_us_ = timestamp_us; // 失败也推进：不重试已经过去的那段，窗口才不会无限长

        float delta[3];
        if (read_delta_ == nullptr || !read_delta_(start_us, timestamp_us, delta)) return out_;

        const double dt = (double)(timestamp_us - start_us) * 1e-6;
        const double rate_cmd = angle.update(target_roll - roll);
        out_ = rate.update(rate_cmd - (double)delta[0] / dt);
        return out_;
    }

private:
    imu_delta_angle_fn read_delta_ = nullptr;
    uint64_t last_tick_us_ = 0;
    double out_ = 0.0;
};

} // namespace dart::control
