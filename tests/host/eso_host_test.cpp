// ============================================================================
// ESO 状态更新用法示例 —— 就是"main 里怎么调用那个虚函数"
//
//   编译运行（x86 主机，不需要 SDK / 相机 / 板子）：
//     g++ -std=gnu++20 -O1 -g -Wall -Wextra -Iinclude tests/host/eso_host_test.cpp
//         -o build/host_test/eso_host_test && ./build/host_test/eso_host_test
//
// 这个文件同时当着"以后真机 IMU 源怎么写"的模板：三个源类都实现了同一个
// IBodyRateSource 接口，主循环里的调用代码完全一样（eso.state_update(src)），
// 换源 = 换一个对象，ESO 那边一行都不用改。
// ============================================================================

#include "control/eso.hpp"

#include <cstdio>

using dart::control::BodyRate;
using dart::control::ExtendedState;
using dart::control::IBodyRateSource;

// ---------------------------------------------------------------------------
// 源 A：假 IMU 四元数解算源（代替仓库里**还没有**的那个解算函数）
//   脚本：前 2 拍"解算未收敛" → 返回 false；之后给一段角速度。
// 真机落地时长这样（现在只能用假的）：
//
//   class ImuSolverSource final : public IBodyRateSource {
//   public:
//       bool read_body_rate(BodyRate &out) override {
//           Quat q;  Vec3 gyro;
//           if (!solver_.latest(q) || !solver_.latest_gyro(gyro)) return false; // 没新数据
//           out = rates_from_quaternion(q, gyro);   // 四元数 → 三轴角速度（缺失的那个函数）
//           return true;
//       }
//       const char *name() const override { return "imu_quat_solver"; }
//   };
// ---------------------------------------------------------------------------
class FakeImuSolver final : public IBodyRateSource {
public:
    bool read_body_rate(BodyRate &out) override {
        ++tick_;
        if (tick_ <= 2) return false; // 解算尚未收敛：本拍没有可信数据
        // 简单造一段运动：偏航匀速转，俯仰/滚转小抖动
        out.roll  = 0.01 * static_cast<double>(tick_);
        out.pitch = -0.02 * static_cast<double>(tick_);
        out.yaw   = 0.30; // rad/s
        return true;
    }

    const char *name() const override { return "fake_imu_quat_solver"; } // 虚函数

private:
    int tick_ = 0;
};

// ---------------------------------------------------------------------------
// 源 B：回放源（读一段录下来的角速度）。用来证明"换源不动 ESO"。
// ---------------------------------------------------------------------------
class ReplaySource final : public IBodyRateSource {
public:
    explicit ReplaySource(const BodyRate *frames, int n) : frames_(frames), n_(n) {}

    bool read_body_rate(BodyRate &out) override {
        if (i_ >= n_) return false; // 回放到底
        out = frames_[i_++];
        return true;
    }
    const char *name() const override { return "replay"; }

private:
    const BodyRate *frames_;
    int             n_;
    int             i_ = 0;
};

static void dump(const char *tag, const ExtendedState &eso) {
    std::printf("  [%s] has_state=%d  roll=%+.4f pitch=%+.4f yaw=%+.4f (rad/s)\n", tag,
                eso.has_state() ? 1 : 0, eso.roll_velocity(), eso.pitch_velocity(),
                eso.yaw_velocity());
}

int main() {
    ExtendedState eso;

    // ---- 1) 通过**基类引用**调用虚函数：编译期不知道是哪个源，运行期才分派 ----
    FakeImuSolver fake;
    IBodyRateSource &src = fake; // 绑定基类引用
    std::printf("source = %s\n", src.name()); // 虚函数调用 #1（观察者是谁）

    for (int t = 0; t < 5; ++t) {
        const bool got = eso.state_update(src); // 虚函数调用 #2（取数据）
        if (!got) {
            // 关键约定：没数据就当"这一拍没发生"，状态保持不动
            std::printf("t=%d  no fresh imu data -> state untouched\n", t);
        } else {
            std::printf("t=%d  state updated\n", t);
        }
        dump("after", eso);
    }

    // ---- 2) 换源：ESO 的调用代码一个字都没变（这就是虚函数在这儿的意义）----
    const BodyRate tape[3] = {{0.0, 0.0, 0.10}, {0.0, 0.0, 0.20}, {0.0, 0.0, 0.30}};
    ReplaySource replay(tape, 3);
    std::printf("source = %s\n", replay.name());
    while (eso.state_update(replay)) dump("replay", eso);
    std::printf("replay exhausted -> last state kept\n");
    dump("final", eso);

    // ---- 3) 不带源对象的直接送值版本（上层已解算好的场合 / 主机自检）----
    eso.state_update(BodyRate{0.0, 0.0, 0.0});
    dump("direct", eso);

    // 自检：回放最后一帧是 yaw = 0.30，直接送值把它清零
    return (eso.yaw_velocity() == 0.0) ? 0 : 1;
}
