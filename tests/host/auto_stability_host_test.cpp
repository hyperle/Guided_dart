// roll 自稳的主机侧自检：IMU 慢轨消费 / 注入 / 按拍语义 / split 带 / 闭环收敛
//
//   g++ -std=gnu++20 -O1 -g -Wall -Wextra -Iinclude tests/host/auto_stability_host_test.cpp
//       -o build/host_test/auto_stability_host_test && ./build/host_test/auto_stability_host_test

#include "../../src/control/auto_stability.cpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace {

using dart::control::RollStabilizer;

int failures = 0;

void Check(bool ok, const char *what) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

// 主机上顶替 imu_get_delta_angle_between 的假慢轨。按"窗口内速率恒定"给增量；
// 真实模块是梯形积分，差别只有半个采样周期。
double g_rate = 0.0; // rad/s（roll 轴）
bool g_delta_ok = true;
uint64_t g_now_us = 0;

bool FakeDelta(uint64_t start_us, uint64_t end_us, float *out_delta) {
    if (!g_delta_ok) return false;
    out_delta[0] = (float)(g_rate * (double)(end_us - start_us) * 1e-6);
    out_delta[1] = 0.0f;
    out_delta[2] = 0.0f;
    return true;
}

uint64_t NextTick() { return g_now_us += 2000; } // 500Hz

// 500Hz（T=2ms）下的每拍增益：ki = ki_按时间·T，kd = kd_按时间/T
RollStabilizer MakeStabilizer() {
    RollStabilizer s;
    s.set_imu_source(FakeDelta);
    s.set_angle_pid({.kp = 4.0, .ki = 0.002, .kd = 0.0, .i_limit = 1.0, .out_limit = 4.0,
                     .err_band = 0.35}); // 期望角速度限幅 4 rad/s，积分贡献上限 1 rad/s
    s.set_rate_pid({.kp = 0.15, .ki = 1e-4, .kd = 5.0, .i_limit = 0.3, .out_limit = 1.0,
                    .err_band = 1.0}); // 行程 ±1，积分贡献上限 0.3
    return s;
}

// 弹体 roll 轴：执行机构作用于角加速度（双积分），bias 是恒定干扰力矩（推力偏心）
struct RollPlant {
    double roll = 0.0;
    double rate = 0.0;
    double b = 20.0;   // rad/s² per unit
    double bias = 0.5; // rad/s²
    void Step(double u, double dt) {
        rate += (b * u + bias) * dt;
        roll += rate * dt;
    }
};

void TestImuConsumption() {
    std::printf("[1] IMU 慢轨消费\n");
    RollStabilizer s;
    s.set_angle_pid({.kp = 1.0, .ki = 0.1, .i_limit = 1e9, .out_limit = 1e9});
    s.set_rate_pid({.kp = 1.0, .out_limit = 1e9});

    Check(s.update(1.0, 0.0, NextTick()) == 0.0, "未注入源：第一拍不出指令");

    s.set_imu_source(FakeDelta);
    g_now_us = 0;
    g_rate = 0.0;
    g_delta_ok = true;
    Check(s.update(1.0, 0.0, NextTick()) == 0.0, "第一拍：只记窗口基准，PID 不推进");

    const double u1 = s.update(1.0, 0.0, NextTick());
    g_delta_ok = false;
    const double u2 = s.update(1.0, 0.0, NextTick());
    Check(u2 == u1, "慢轨取不到（越界/丢数据）：指令保持，PID 不动");

    g_delta_ok = true;
    const uint64_t t = NextTick();
    const double u3 = s.update(1.0, 0.0, t);
    const double u4 = s.update(1.0, 0.0, t); // 同一拍重复调用
    Check(u4 == u3, "时间戳没往前走：不重复消费");

    const double u5 = s.update(1.0, 0.0, NextTick());
    std::printf("      u1=%.3f u2=%.3f u3=%.3f u4=%.3f u5=%.3f\n", u1, u2, u3, u4, u5);
    Check(u5 > u3, "新的一拍：恢复消费，积分继续累加");

    // 增量 ÷ 窗长 就是内环看到的角速度
    RollStabilizer s2;
    s2.set_imu_source(FakeDelta);
    s2.set_angle_pid({.out_limit = 1e9}); // 外环不出力 → rate_cmd = 0
    s2.set_rate_pid({.kp = 1.0, .out_limit = 1e9});
    g_now_us = 0;
    g_rate = 2.5;
    s2.update(0.0, 0.0, NextTick());
    const double u_rate = s2.update(0.0, 0.0, NextTick());
    std::printf("      慢轨 2.5 rad/s → u=%.6f\n", u_rate);
    Check(std::fabs(u_rate + 2.5) < 1e-6, "区间增量/窗长 直接进内环");
}

void TestInjection() {
    std::printf("[2] 注入与按拍语义\n");
    RollStabilizer s;
    s.set_imu_source(FakeDelta);
    s.set_angle_pid({.ki = 1e-3, .i_limit = 1e9, .out_limit = 1e9}); // err_band 0 = 不设带
    s.set_rate_pid({.kp = 1.0, .out_limit = 1e9});                   // 内环只做直通
    g_now_us = 0;
    g_rate = 0.0;
    g_delta_ok = true;

    double u = 0.0;
    s.update(1.0, 0.0, NextTick()); // 第一拍只记基准
    for (int i = 0; i < 1000; ++i) u = s.update(1.0, 0.0, NextTick()); // 角度误差恒为 1 rad
    // PidCalculator 先用旧积分、再把本拍误差并进去，所以 1000 拍只积了 999 次
    const double expected = 1e-3 * 999;
    std::printf("      1000 拍后=%+.6f（期望 %.6f）\n", u, expected);
    Check(std::fabs(u - expected) < 1e-12, "积分只跟拍数走（与时间无关），滞后一拍");

    // i_limit 是**贡献**上限：ki=1e-3、i_limit=0.5 → 贡献停在 0.5
    RollStabilizer s2;
    s2.set_imu_source(FakeDelta);
    s2.set_angle_pid({.ki = 1e-3, .i_limit = 0.5, .out_limit = 1e9});
    s2.set_rate_pid({.kp = 1.0, .out_limit = 1e9});
    g_now_us = 0;
    s2.update(1.0, 0.0, NextTick());
    for (int i = 0; i < 2000; ++i) u = s2.update(1.0, 0.0, NextTick());
    std::printf("      积分贡献饱和在 u=%+.6f\n", u);
    Check(std::fabs(u - 0.5) < 1e-12, "i_limit 按贡献限幅（不随 ki 漂）");

    s2.set_angle_pid({.ki = 1e-3, .i_limit = 0.5, .out_limit = 1e9}); // 重新注入
    Check(s2.update(1.0, 0.0, NextTick()) == 0.0, "重新注入 → 积分清零");
}

void TestSplitBand() {
    std::printf("[3] split 带抗饱和：带内积分、带外清零\n");
    RollStabilizer s;
    s.set_imu_source(FakeDelta);
    s.set_angle_pid({.ki = 1e-3, .i_limit = 1e9, .out_limit = 1e9, .err_band = 0.2});
    s.set_rate_pid({.kp = 1.0, .out_limit = 1e9});
    g_now_us = 0;
    g_rate = 0.0;
    g_delta_ok = true;

    double u = 0.0;
    s.update(0.1, 0.0, NextTick());
    for (int i = 0; i < 1000; ++i) u = s.update(0.1, 0.0, NextTick()); // 误差 0.1，在带内
    const double expected = 1e-3 * 999 * 0.1;
    std::printf("      带内 err=0.1、1000 拍：u=%+.6f（期望 %.6f）\n", u, expected);
    Check(std::fabs(u - expected) < 1e-12, "带内：积分照常累加");

    u = s.update(0.4, 0.0, NextTick()); // 误差 0.4，出带
    std::printf("      出带后：u=%+.6f\n", u);
    Check(u == 0.0, "出带：积分立刻清零（不会带着旧积分顶限幅）");
}

void TestSignAndLimit() {
    std::printf("[4] 符号、限幅、NaN\n");
    RollStabilizer s = MakeStabilizer();
    g_now_us = 0;
    g_rate = 0.0;
    g_delta_ok = true;

    s.update(0.0, 0.5, NextTick());
    const double u = s.update(0.0, 0.5, NextTick()); // roll=+0.5（右翼下沉）
    std::printf("      roll=+0.5 → u=%+.3f\n", u);
    Check(u < 0.0, "roll 偏正 → 指令为负");

    double u_max = 0.0;
    for (int i = 0; i < 10; ++i) u_max = s.update(10.0, 0.0, NextTick()); // 目标角 10 rad
    std::printf("      大误差：u=%+.3f\n", u_max);
    Check(std::fabs(u_max) <= s.rate.output_max + 1e-12, "指令不超过内环行程");

    // 慢轨给出非有限增量
    RollStabilizer s_nan;
    s_nan.set_imu_source(FakeDelta);
    s_nan.set_rate_pid({.kp = 1.0, .out_limit = 1e9});
    g_now_us = 0;
    g_delta_ok = true;
    g_rate = std::numeric_limits<double>::quiet_NaN();
    s_nan.update(0.0, 0.0, NextTick());
    Check(std::isnan(s_nan.update(0.0, 0.0, NextTick())), "非有限增量 → 输出 NaN（不假装是 0）");
}

void TestClosedLoop() {
    std::printf("[5] 闭环：0.3 rad 初始偏差 + 恒定干扰力矩（500Hz）\n");
    RollStabilizer s = MakeStabilizer();
    RollPlant plant;
    plant.roll = 0.3;

    const double dt = 0.002;
    g_now_us = 0;
    g_delta_ok = true;
    double max_abs_u = 0.0;
    for (int i = 0; i < 5000; ++i) { // 10s
        g_rate = plant.rate;         // 慢轨里存的是这一拍真实转过的角度
        const double u = s.update(0.0, plant.roll, NextTick());
        plant.Step(u, dt);
        if (std::fabs(u) > max_abs_u) max_abs_u = std::fabs(u);
    }
    std::printf("      10s 后 roll=%+.5f rad  rate=%+.5f rad/s  max|u|=%.3f\n", plant.roll, plant.rate,
                max_abs_u);
    Check(std::fabs(plant.roll) < 0.01, "角度收敛（< 0.01 rad ≈ 0.6°）");
    Check(max_abs_u <= s.rate.output_max + 1e-12, "全程指令在行程内");
    Check(std::isfinite(plant.roll) && std::isfinite(plant.rate), "全程无 NaN");

    // 对照：去掉积分 → 恒定干扰力矩下必然留稳态误差（正常整定走 set_*_pid，这里只做对照）
    RollStabilizer p_only = MakeStabilizer();
    p_only.angle.ki = 0.0;
    p_only.rate.ki = 0.0;
    RollPlant plant_p;
    plant_p.roll = 0.3;
    g_now_us = 0;
    for (int i = 0; i < 5000; ++i) {
        g_rate = plant_p.rate;
        plant_p.Step(p_only.update(0.0, plant_p.roll, NextTick()), dt);
    }
    std::printf("      对照（无积分）roll=%+.5f rad\n", plant_p.roll);
    Check(std::fabs(plant_p.roll) > std::fabs(plant.roll), "有积分比无积分残余偏差更小");
}

} // namespace

int main() {
    TestImuConsumption();
    TestInjection();
    TestSplitBand();
    TestSignAndLimit();
    TestClosedLoop();
    std::printf("\n%s（失败 %d 项）\n", failures == 0 ? "auto_stability: ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
