// 联合 EKF 的开销基准 + 结构指纹
//
//   g++ -std=gnu++20 -O2 -Iinclude tests/host/predictor_bench.cpp -o build/host_test/predictor_bench
//       && ./build/host_test/predictor_bench
//
// 场景按实机设定：控制拍 500Hz（kTick），视觉帧 100Hz（每 5 拍一帧，整数比）。
// 主机耗时只用于**相对比较**；板端预算见 CONTROL_PERF.md（CPU0 800MHz、无 RVV）。
//
// 与重构前的对照（同一台机器、同为 -O2；旧值记在 CONTROL_PERF.md）：
//   旧：每拍 predict 159.3 ns / 每帧（3 更新 + α-β）0.8 µs —— 但那是 5 态 + 2 态分离的代价，
//   换来的是互协方差恒为 0。新：8 态联合，多出来的开销主要在这里量出来。
//   checksum 是**结构指纹**（状态 + 协方差原始字节的哈希）：重构前后它必然不同，
//   它的用途是"同一份代码再改时数值有没有动"，不是跨版本比对。

#include "../../src/control/predictor.cpp"

#include <chrono>
#include <cstdint>
#include <cstdio>

using dart::control::ChannelCoeff;
using dart::control::JointEkf;
using dart::control::kHorizontal;
using dart::control::kMotionCount;
using dart::control::kOmega;
using dart::control::kResRate;
using dart::control::kStateCount;
using dart::control::kTheta;
using dart::control::kTick;
using dart::control::kTickUs;
using dart::control::kU;
using dart::control::kV;
using dart::control::kVertical;

namespace {

ChannelCoeff MakeCoeff() {
    ChannelCoeff c;
    c.k_m = -4e-6;
    c.k_f = 1e-5;
    c.k_d = 1e-6;
    c.k_delta = -2.8;
    return c;
}

// FNV-1a：把状态与协方差的**原始字节**哈希（同一份代码改动前后的数值指纹）
struct Hash {
    uint64_t h = 1469598103934665603ull;
    void feed(double v) {
        uint64_t bits;
        __builtin_memcpy(&bits, &v, sizeof bits);
        for (int i = 0; i < 8; ++i) {
            h ^= (bits >> (8 * i)) & 0xffu;
            h *= 1099511628211ull;
        }
    }
};

double NowNs() {
    using namespace std::chrono;
    return (double)duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

} // namespace

int main() {
    const ChannelCoeff c = MakeCoeff();
    JointEkf ekf(c, c);
    double motion[kMotionCount] = {};
    motion[kTheta] = 0.02;
    motion[kOmega] = 0.3;
    motion[kU] = 0.05;
    motion[kV] = 60.0;
    ekf.set_motion_state(kHorizontal, motion);
    ekf.set_motion_state(kVertical, motion);

    // ---- 1) 每拍：模型推进 + 协方差传递（观测器里最重的一项）----
    const int ticks = 400000;
    uint64_t tick_us = 0;
    double t0 = NowNs();
    for (int k = 0; k < ticks; ++k) {
        tick_us += kTickUs;
        ekf.predict(0.2, -0.1, tick_us);
    }
    double t1 = NowNs();
    const double ns_per_tick = (t1 - t0) / ticks;

    // ---- 2) 每帧：3 个 IMU 标量更新 + 1 个像素更新（100Hz 视觉 = 5 拍一帧）----
    const int frames = 40000;
    uint64_t now_us = 1000000;
    double pixel = 120.0;
    t0 = NowNs();
    for (int f = 0; f < frames; ++f) {
        for (int k = 0; k < 5; ++k) {
            now_us += kTickUs;
            ekf.predict(0.2, -0.1, now_us);
        }
        const double theta = ekf.state(kHorizontal, kTheta);
        const double omega = ekf.state(kHorizontal, kOmega);
        const double u = ekf.state(kHorizontal, kU);
        const double v = ekf.state(kHorizontal, kV);
        ekf.update_attitude(kHorizontal, theta, 1e-8);
        ekf.update_rate(kHorizontal, omega, 1e-8);
        ekf.update_velocity(kHorizontal, u, v, 1e-4);
        pixel += 0.4; // 目标匀速横移
        ekf.update_pixel(kHorizontal, pixel, 0.25, now_us);
    }
    t1 = NowNs();
    const double us_per_frame = (t1 - t0) / frames / 1000.0;

    // ---- 3) 自身运动前馈轨迹（含 libm 的 tan，控制律可选调用）----
    double dh[16];
    double dv[16];
    const int traj_calls = 200000;
    t0 = NowNs();
    for (int k = 0; k < traj_calls; ++k) ekf.ego_trajectory(16, dh, dv);
    t1 = NowNs();
    const double ns_per_traj = (t1 - t0) / traj_calls;

    // ---- 4) 未来像素预测（残差外推 + 自身前推，一次 tan）----
    const int pred_calls = 200000;
    double sink = 0.0;
    t0 = NowNs();
    for (int k = 0; k < pred_calls; ++k) sink += ekf.observed_pixel_ahead(kHorizontal, 25);
    t1 = NowNs();
    const double ns_per_ahead = (t1 - t0) / pred_calls;
    if (!(sink == sink)) return 1; // 防优化掉

    Hash hash;
    hash.feed(sink);
    for (int ch = 0; ch < 2; ++ch) {
        for (int i = 0; i < kStateCount; ++i) {
            hash.feed(ekf.state(ch, i));
            for (int j = 0; j < kStateCount; ++j) hash.feed(ekf.covariance(ch, i, j));
        }
    }
    hash.feed(ekf.residual_rate(kHorizontal));
    hash.feed(ekf.residual_rate_px(kHorizontal));

    // 板端预算（CPU0 800MHz，无 RVV；按 1 双精度 FP 操作 ≈ 1 周期估算）
    const double tick_cycles = ns_per_tick * 0.8;
    const double frame_cycles = us_per_frame * 1000.0 * 0.8;
    std::printf("每拍 predict()（两通道）        %8.1f ns    ≈ %6.0f 周期  (500Hz → %5.2f%% CPU0)\n",
                ns_per_tick, tick_cycles, 100.0 * tick_cycles * 500.0 / 8.0e8);
    std::printf("每帧（5 拍 + 3 IMU + 1 像素）   %8.1f µs    ≈ %6.0f 周期  (100Hz → %5.2f%% CPU0)\n",
                us_per_frame, frame_cycles, 100.0 * frame_cycles * 100.0 / 8.0e8);
    std::printf("ego_trajectory(16 拍)           %8.1f ns    ≈ %6.0f 周期  （含 32 次 tan，可选调用）\n",
                ns_per_traj, ns_per_traj * 0.8);
    std::printf("observed_pixel_ahead(25 拍)     %8.1f ns    ≈ %6.0f 周期\n", ns_per_ahead,
                ns_per_ahead * 0.8);
    std::printf("checksum %016llx\n", (unsigned long long)hash.h);
    return 0;
}
