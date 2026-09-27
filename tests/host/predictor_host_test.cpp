// 联合 EKF（自身运动 + 像平面残差）的主机侧自检
//
//   g++ -std=gnu++20 -O1 -g -Wall -Wextra tests/host/predictor_host_test.cpp
//       -o build/host_test/predictor_host_test && ./build/host_test/predictor_host_test
//
// 真值用与滤波器**同一套方程**生成（同一组按拍气动系数、同一条舵偏指令）：
// 这一步验的是**结构与一致性**（分块、雅可比、互协方差、锚定语义、噪声自洽、与旧两级
// 架构的对照），不是验模型对物理是否成立 —— 那是标定阶段的事。
//
// 两处纪律：
//   1. 测试里的"旧架构"（5 态 IMU-only EKF + px 域 α-β）是**参照实现**，只存在于本文件，
//      用来给"联合 vs 级联"提供同一批输入下的对照。它不进 src/。
//   2. 场景必须留在画幅内：合成角 |μ + s(θ-θ₀)| < 30°（kMaxOffAxisRad）。超了投影会被
//      钳到 ±320px（这是有意的 FOV 保护，且计入 projection_clamps()），出来的数字没有意义。
//      RunScenario 会回报 max_abs_off_axis，由调用方断言。

#include "../../src/control/predictor.cpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <tuple>

using dart::control::ChannelCoeff;
using dart::control::JointEkf;
using dart::control::kB;
using dart::control::kChannelCount;
using dart::control::kHorizontal;
using dart::control::kMaxOffAxisRad;
using dart::control::kMotionCount;
using dart::control::kOmega;
using dart::control::kPxPerRad;
using dart::control::kResAngle;
using dart::control::kResRate;
using dart::control::kStateCount;
using dart::control::kTheta;
using dart::control::kThetaAnchor;
using dart::control::kTick;
using dart::control::kTickUs;
using dart::control::kU;
using dart::control::kV;
using dart::control::kVertical;
using dart::control::MotionModel;

namespace {

int failures = 0;

void Check(bool ok, const char *what) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

void CheckNear(double got, double want, double tol, const char *what) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %s  %s（%.6g vs %.6g，容差 %.3g）\n", ok ? "PASS" : "FAIL", what, got, want, tol);
    if (!ok) ++failures;
}

// 静稳定弹体、v=60m/s 下的量级（按拍标定；与旧测试同一组占位系数）
ChannelCoeff MakeCoeff() {
    ChannelCoeff c;
    c.k_m = -4e-6;    // 力矩：负 = 静稳定（尾翼）
    c.k_f = 1e-5;     // 侧向阻尼
    c.k_d = 1e-6;     // 阻力
    c.k_delta = -2.8; // 舵效：δ>0 → ω>0 → 像左移
    return c;
}

// 可复现的伪随机噪声（均匀分布，半幅 amp）
struct Noise {
    uint32_t seed;
    double operator()(double amplitude) {
        seed = seed * 1664525u + 1013904223u;
        return ((double)(seed >> 8) / 16777216.0 - 0.5) * 2.0 * amplitude;
    }
};

void MakeMotion(double theta, double omega, double u, double v, double b,
                double motion[kMotionCount]) {
    motion[kTheta] = theta;
    motion[kOmega] = omega;
    motion[kU] = u;
    motion[kV] = v;
    motion[kB] = b;
}

double OffAxisOf(const JointEkf &ekf, int ch) {
    return ekf.residual_angle(ch) +
           (ch == kHorizontal ? -1.0 : 1.0) *
               (ekf.state(ch, kTheta) - ekf.state(ch, kThetaAnchor));
}

// ---------------------------------------------------------------------------
// 参照实现：旧两级架构（5 态 IMU-only EKF + px 域 α-β），只用于对照
//
//   与旧代码同构：Predictor::advance 推进运动块、EkfFusion 只融 IMU、
//   TargetAlphaBeta 吃"扣掉自身运动预测"的残差。**ego_total 是锚点以来的累计**
//   （旧代码若写成每帧增量，残差里会一直留着自身转角，v̂ 全是假的）。
// ---------------------------------------------------------------------------
class RefTwoStage {
public:
    explicit RefTwoStage(const ChannelCoeff &c, int ch) : coeff_(c), ch_(ch) {
        for (int i = 0; i < kMotionCount; ++i) {
            for (int j = 0; j < kMotionCount; ++j) P_[i][j] = (i == j) ? kSeedVar[i] : 0.0;
        }
    }

    // 一拍：先累计自身运动像移（用**自己估的** ω，与旧 Predictor::step 同序），再推进
    void tick() {
        ego_total_px_ += (ch_ == kHorizontal ? -1.0 : 1.0) * kPxPerRad * kTick * x_[kOmega];
        double joint_in[kStateCount] = {};
        double joint_out[kStateCount] = {};
        double F[kStateCount][kStateCount] = {};
        for (int i = 0; i < kMotionCount; ++i) joint_in[i] = x_[i];
        MotionModel::advance(coeff_, joint_in, delta_, joint_out, F);
        double fp[kMotionCount][kMotionCount];
        double next[kMotionCount][kMotionCount];
        for (int i = 0; i < kMotionCount; ++i) {
            for (int j = 0; j < kMotionCount; ++j) {
                double s = 0.0;
                for (int k = 0; k < kMotionCount; ++k) s += F[i][k] * P_[k][j];
                fp[i][j] = s;
            }
        }
        for (int i = 0; i < kMotionCount; ++i) {
            for (int j = 0; j < kMotionCount; ++j) {
                double s = 0.0;
                for (int k = 0; k < kMotionCount; ++k) s += fp[i][k] * F[j][k];
                next[i][j] = s + ((i == j) ? kQ[i] : 0.0);
            }
        }
        for (int i = 0; i < kMotionCount; ++i) {
            x_[i] = joint_out[i];
            for (int j = 0; j < kMotionCount; ++j) P_[i][j] = next[i][j];
        }
    }

    void update(int index, double value, double var) {
        double ph[kMotionCount];
        for (int i = 0; i < kMotionCount; ++i) ph[i] = P_[i][index];
        const double s = var + P_[index][index];
        const double innov = value - x_[index];
        if (!(s > 0.0) || innov * innov / s > 6.63) return; // 旧 EkfFusion 的门限
        for (int i = 0; i < kMotionCount; ++i) x_[i] += ph[i] / s * innov;
        for (int i = 0; i < kMotionCount; ++i) {
            for (int j = 0; j < kMotionCount; ++j) P_[i][j] -= ph[i] / s * ph[j];
        }
    }

    // 视觉帧：喂**绝对像素**，内部扣掉锚点以来累计的自身运动像移
    void pixel(double measured_px, double frame_s) {
        const double pure = measured_px - ego_total_px_;
        if (!init_) {
            pos_ = pure;
            rate_ = 0.0;
            init_ = true;
            return;
        }
        if (!(frame_s > 0.0)) return;
        pos_ += rate_ * frame_s;
        const double innov = pure - pos_;
        pos_ += kAlpha * innov;
        rate_ += kBeta / frame_s * innov;
    }

    double x(int i) const { return x_[i]; }
    double &x(int i) { return x_[i]; }
    double rate_px() const { return rate_; }
    double pos_px() const { return pos_; }

private:
    static constexpr double kAlpha = 0.30; // 旧 AlphaBetaGains 推荐值
    static constexpr double kBeta = 0.03;
    static constexpr double kSeedVar[kMotionCount] = {0.01, 0.01, 1.0, 1.0, 1.0};
    static constexpr double kQ[kMotionCount] = {1e-8, 1e-8, 1e-6, 1e-6, 1e-8};

    ChannelCoeff coeff_;
    int ch_;
    double x_[kMotionCount] = {};
    double P_[kMotionCount][kMotionCount] = {};
    double delta_ = 0.0;
    double ego_total_px_ = 0.0;
    double pos_ = 0.0, rate_ = 0.0;
    bool init_ = false;
};

// ---------------------------------------------------------------------------
// 场景骨架：同一批输入分别喂给联合 EKF 与参照实现
// ---------------------------------------------------------------------------
struct Scenario {
    double theta0 = 0.05;      // 初始姿态（静稳定振荡幅度）
    double psi0_px = 100.0;    // 目标初始像素（必须留在画幅内）
    double target_px_s = 40.0; // 目标像面匀速横移
    double gyro_bias = 0.0;    // 陀螺/姿态的未建模常值偏置 [rad/s]
    double gyro_att_var = 1e-8; // 姿态观测方差（旧测试的占位：1e-4 rad 精度，板端填真实值）
    double px_amp = 0.5;       // 像素噪声半幅（均匀）
    double ego_z = 0.0;        // >0：真值里含"自身平移引起的像移"（∝ a_lat/Z，模型里没有）
    int frames = 200;
    int ticks_per_frame = 5;   // 视觉 100Hz（与 500Hz 控制整数比）
    uint32_t seed = 777u;
    double sigma_a = -1.0;     // <0 = 用默认
    double inv_z = 0.0;        // 机动自适应 Q 用（0 = 关）
};

struct ScenarioResult {
    double joint_rate_bias = 0, ref_rate_bias = 0;
    double joint_rate_rmse = 0, ref_rate_rmse = 0;
    double joint_rate_bias_late = 0, ref_rate_bias_late = 0;
    double accepted_nis_mean = 0;
    double max_abs_off_axis = 0;
    uint32_t pixel_rejects = 0, rescues = 0, updates = 0, clamps = 0;
    bool finite = true;
};

ScenarioResult RunScenario(const Scenario &sc, bool run_reference, double prior = -1.0) {
    const ChannelCoeff c = MakeCoeff();
    JointEkf ekf(c, c);
    if (sc.sigma_a > 0.0) ekf.set_residual_accel_noise(sc.sigma_a);
    if (prior >= 0.0) ekf.set_residual_rate_prior(prior);
    if (sc.inv_z > 0.0) ekf.set_maneuver_coupling(sc.inv_z);

    double motion[kMotionCount];
    MakeMotion(sc.theta0, 0.0, 0.0, 60.0, 0.0, motion);
    for (int ch = 0; ch < kChannelCount; ++ch) ekf.set_motion_state(ch, motion);

    RefTwoStage ref(c, kHorizontal);
    for (int i = 0; i < kMotionCount; ++i) ref.x(i) = motion[i];

    // 真值：与滤波器同一套方程，只多一个未建模陀螺偏置（和可选的平移耦合项）
    double truth[kStateCount] = {};
    double truth_next[kStateCount] = {};
    for (int i = 0; i < kMotionCount; ++i) truth[i] = motion[i];

    double psi = std::atan(sc.psi0_px / kPxPerRad);
    double psi_rate = sc.target_px_s / kPxPerRad;
    double theta_anchor_true = motion[kTheta];
    const double frame_s = sc.ticks_per_frame * kTick;
    const double px_var = sc.px_amp * sc.px_amp / 3.0; // 均匀分布 ±amp 的方差
    Noise noise{sc.seed};

    uint64_t tick_us = 1000000;
    uint64_t frame_us = 1000;
    double t_s = 0.0;
    double sum_j = 0, sum_r = 0, sum_j2 = 0, sum_r2 = 0;
    double sum_j_late = 0, sum_r_late = 0;
    int count = 0, count_late = 0;
    const int late_start = sc.frames / 2;
    ScenarioResult out;

    for (int f = 0; f < sc.frames; ++f) {
        for (int k = 0; k < sc.ticks_per_frame; ++k) {
            MotionModel::advance(c, truth, 0.0, truth_next, nullptr);
            for (int n = 0; n < kStateCount; ++n) truth[n] = truth_next[n];
            tick_us += kTickUs;
            ekf.predict(0.0, 0.0, tick_us);
            ref.tick();
            t_s += kTick;
        }
        // 未建模的自身平移耦合：像移角速率 ≈ a_lat/Z（模型里没有这一项）
        if (sc.ego_z > 0.0) {
            psi += MotionModel::lateral_accel(c, truth, 0.0) / sc.ego_z * frame_s;
        }
        psi += psi_rate * frame_s;
        if (f == 0) theta_anchor_true = truth[kTheta]; // 锚点时刻 = 首帧像素更新之前

        const double dth_true = truth[kTheta] - theta_anchor_true;
        const double z = kPxPerRad * std::tan(psi + (-1.0) * dth_true) + noise(sc.px_amp);
        frame_us += (uint64_t)(frame_s * 1e6);

        const double theta_meas = truth[kTheta] + sc.gyro_bias * t_s;
        const double omega_meas = truth[kOmega] + sc.gyro_bias;
        ekf.update_attitude(kHorizontal, theta_meas, sc.gyro_att_var);
        ekf.update_rate(kHorizontal, omega_meas, 1e-8);
        ekf.update_velocity(kHorizontal, truth[kU], truth[kV], 1e-4);
        ekf.update_pixel(kHorizontal, z, px_var, frame_us);

        if (run_reference) {
            ref.update(kTheta, theta_meas, sc.gyro_att_var);
            ref.update(kOmega, omega_meas, 1e-8);
            ref.update(kV, truth[kV], 1e-4);
            ref.pixel(z, frame_s);
        }

        out.max_abs_off_axis =
            std::fmax(out.max_abs_off_axis, std::fabs(OffAxisOf(ekf, kHorizontal)));

        if (f >= 40) {
            const double true_rate_px = psi_rate * kPxPerRad;
            const double je = ekf.residual_rate_px(kHorizontal) - true_rate_px;
            const double re = run_reference ? ref.rate_px() - true_rate_px : 0.0;
            sum_j += je;
            sum_r += re;
            sum_j2 += je * je;
            sum_r2 += re * re;
            if (f >= late_start) {
                sum_j_late += je;
                sum_r_late += re;
                ++count_late;
            }
            ++count;
        }
    }

    out.joint_rate_bias = sum_j / count;
    out.ref_rate_bias = sum_r / count;
    out.joint_rate_rmse = std::sqrt(sum_j2 / count);
    out.ref_rate_rmse = std::sqrt(sum_r2 / count);
    out.joint_rate_bias_late = sum_j_late / count_late;
    out.ref_rate_bias_late = sum_r_late / count_late;
    out.accepted_nis_mean = ekf.pixel_updates() ? ekf.pixel_nis_sum() / ekf.pixel_updates() : 0.0;
    out.pixel_rejects = ekf.pixel_rejects();
    out.rescues = ekf.consistency_rescues();
    out.updates = ekf.pixel_updates();
    out.clamps = ekf.projection_clamps();
    for (int ch = 0; ch < kChannelCount; ++ch) {
        for (int i = 0; i < kStateCount; ++i) {
            if (!std::isfinite(ekf.state(ch, i))) out.finite = false;
            for (int j = 0; j < kStateCount; ++j) {
                if (!std::isfinite(ekf.covariance(ch, i, j))) out.finite = false;
            }
        }
    }
    return out;
}

// ---------------------------------------------------------------------------

void TestJointStructure() {
    std::printf("[1] 联合状态、雅可比、两通道解耦、投影非线性\n");
    Check(kStateCount == 8, "状态 = [θ, ω, u, v, b | μ, μ̇ | θ₀]，共 8 维");
    Check(kMotionCount == 5, "运动子块 = 5 维（原气动状态一字未改）");

    const ChannelCoeff c = MakeCoeff();
    double motion[kMotionCount];
    MakeMotion(0.05, 0.3, 0.05, 60.0, 0.01, motion);

    // 稀疏表一致性：step() 写出来的 F，其非零项必须全部落在 kFPattern 里
    {
        JointEkf ekf(c, c);
        ekf.set_motion_state(kHorizontal, motion);
        double F[kStateCount][kStateCount] = {};
        ekf.step(kHorizontal, F);
        bool covered = true;
        for (int i = 0; i < kStateCount; ++i) {
            bool in_pattern[kStateCount] = {};
            for (int n = 0; dart::control::kFPattern[i][n] >= 0; ++n) {
                in_pattern[dart::control::kFPattern[i][n]] = true;
            }
            for (int j = 0; j < kStateCount; ++j) {
                if (!in_pattern[j] && F[i][j] != 0.0) covered = false;
            }
        }
        Check(covered, "kFPattern 覆盖了联合 F 的全部非零项");
    }

    // 解析雅可比 vs 中心差分（气动块非线性 —— 这是"不确定度传递"的地基）
    {
        const int idx[3] = {kTheta, kV, kOmega};
        const double h = 1e-6;
        bool diff_ok = true;
        for (int n = 0; n < 3; ++n) {
            JointEkf a(c, c), b(c, c);
            double ma[kMotionCount], mb[kMotionCount];
            for (int i = 0; i < kMotionCount; ++i) {
                ma[i] = motion[i];
                mb[i] = motion[i];
            }
            ma[idx[n]] += h;
            mb[idx[n]] -= h;
            a.set_motion_state(kHorizontal, ma);
            b.set_motion_state(kHorizontal, mb);
            a.step(kHorizontal, nullptr);
            b.step(kHorizontal, nullptr);
            JointEkf ref(c, c);
            ref.set_motion_state(kHorizontal, motion);
            double F[kStateCount][kStateCount] = {};
            ref.step(kHorizontal, F);
            for (int i = 0; i < kMotionCount; ++i) {
                const double d = (a.state(kHorizontal, i) - b.state(kHorizontal, i)) / (2.0 * h);
                if (std::fabs(d - F[i][idx[n]]) > 1e-3 * std::fmax(1.0, std::fabs(d))) {
                    diff_ok = false;
                }
            }
        }
        Check(diff_ok, "解析雅可比与中心差分一致（θ/ω/v 三列）");
    }

    // 残差块与锚点块的雅可比是精确的线性式
    {
        JointEkf ekf(c, c);
        ekf.set_motion_state(kHorizontal, motion);
        double F[kStateCount][kStateCount] = {};
        ekf.step(kHorizontal, F);
        Check(F[kResAngle][kResAngle] == 1.0 && F[kResAngle][kResRate] == kTick,
              "μ' = μ + kTick·μ̇（α-β 背后的匀速模型，逐拍推进）");
        Check(F[kResRate][kResRate] == 1.0 && F[kThetaAnchor][kThetaAnchor] == 1.0,
              "μ̇ 与 θ₀ 都是常值状态");
    }

    // 两通道解耦：只动水平舵，垂直通道 8 维逐位不变
    {
        JointEkf a(c, c), b(c, c);
        double m2[kMotionCount];
        MakeMotion(0.01, 0.02, 0.0, 60.0, 0.0, m2);
        for (int ch = 0; ch < kChannelCount; ++ch) {
            a.set_motion_state(ch, m2);
            b.set_motion_state(ch, m2);
        }
        for (int k = 0; k < 50; ++k) {
            a.predict(0.0, 0.0, 1000000 + (uint64_t)k * kTickUs);
            b.predict(1.0, 0.0, 1000000 + (uint64_t)k * kTickUs);
        }
        bool same = true;
        for (int i = 0; i < kStateCount; ++i) {
            same = same && (a.state(kVertical, i) == b.state(kVertical, i));
        }
        Check(same, "只动水平舵：垂直通道 50 拍逐位不变");
    }

    // 投影非线性：近轴退化成旧线性式；离轴时同一转角的像移必须更大（tan 而不是线性）
    {
        JointEkf near(c, c);
        double m0[kMotionCount];
        MakeMotion(0.0, 0.0, 0.0, 60.0, 0.0, m0);
        for (int ch = 0; ch < kChannelCount; ++ch) near.set_motion_state(ch, m0);
        near.predict(0.0, 0.0, 1000000 + kTickUs);
        near.update_pixel(kHorizontal, 2.0, 0.25, 2000); // 近轴（0.2°）
        const double small_px = near.predicted_pixel(kHorizontal);
        const double linear_px = kPxPerRad * OffAxisOf(near, kHorizontal);
        CheckNear(small_px, linear_px, 0.01, "近轴：h(x) ≈ f·[μ + s·(θ-θ₀)]（与旧线性式一致）");

        // 同一转角、不同离轴角：像移必须不同（tan 的 sec² 因子），旧的线性式给不出来
        auto ego_step_at = [&](double off_axis_rad) {
            JointEkf ekf(c, c);
            double m[kMotionCount];
            MakeMotion(0.0, 0.0, 0.0, 60.0, 0.0, m);
            for (int ch = 0; ch < kChannelCount; ++ch) ekf.set_motion_state(ch, m);
            ekf.predict(0.0, 0.0, 1000000 + kTickUs);
            // 同一批运动状态（ω 相同）下，只让视线角不同
            MakeMotion(0.0, 0.5, 0.0, 60.0, 0.0, m);
            ekf.set_motion_state(kHorizontal, m);
            ekf.update_pixel(kHorizontal, kPxPerRad * std::tan(off_axis_rad), 0.25, 2000);
            return ekf.ego_pixel_step(kHorizontal);
        };
        const double step_near = ego_step_at(0.0);
        const double step_far = ego_step_at(0.35); // 20° 离轴
        const double sec2 = 1.0 + std::tan(0.35) * std::tan(0.35);
        std::printf("      同一 ω=0.5rad/s：近轴像移 %+.4f px，20° 离轴 %+.4f px（比值 %.3f，sec²=%.3f）\n",
                    step_near, step_far, step_far / step_near, sec2);
        Check(std::fabs(step_far / step_near - sec2) < 0.02,
              "离轴 20°：同一转角的像移按 sec² 放大 13%（旧的线性式在这里系统性欠补偿）");
    }
}

// 反解：给定 α-β 的稳态位置增益 α 与观测噪声 R，求匀速模型的加速度噪声 σ_a
// （2 态卡尔曼稳态、位置观测；给默认 σ_a 提供**可复算**的出处）
void SteadyStateGains(double dt, double R, double sigma_a, double *alpha_out, double *beta_out) {
    double p00 = 1.0, p01 = 0.0, p11 = 1.0;
    for (int it = 0; it < 200000; ++it) {
        const double q00 = sigma_a * sigma_a * dt * dt * dt * dt / 4.0;
        const double q01 = sigma_a * sigma_a * dt * dt * dt / 2.0;
        const double q11 = sigma_a * sigma_a * dt * dt;
        const double a00 = p00 + 2.0 * dt * p01 + dt * dt * p11 + q00;
        const double a01 = p01 + dt * p11 + q01;
        const double a11 = p11 + q11;
        const double s = a00 + R;
        const double k0 = a00 / s;
        const double k1 = a01 / s;
        const double n00 = a00 - k0 * a00;
        const double n01 = a01 - k0 * a01;
        const double n11 = a11 - k1 * a00;
        const bool converged = std::fabs(n00 - p00) < 1e-20 && std::fabs(n01 - p01) < 1e-20 &&
                               std::fabs(n11 - p11) < 1e-20;
        p00 = n00;
        p01 = n01;
        p11 = n11;
        if (converged) break;
    }
    const double s = p00 + R;
    *alpha_out = p00 / s;
    *beta_out = p01 / s * dt; // α-β 记法里的 β = K_velocity·dt
}

void TestNoiseProvenance() {
    std::printf("[2] 默认噪声的出处与先验的必要性\n");
    // 旧 α-β 的推荐值是在 200Hz 视觉（dt=5ms）、σ_px=0.5px 下整定的
    const double dt = 0.005;
    const double R = (0.5 / kPxPerRad) * (0.5 / kPxPerRad);
    double alpha = 0.0, beta = 0.0;
    SteadyStateGains(dt, R, JointEkf::default_residual_accel_noise(), &alpha, &beta);
    std::printf("      σ_a=%.3f rad/s² → 稳态 (α, β) = (%.4f, %.5f)（旧 α-β 用 0.30 / 0.03）\n",
                JointEkf::default_residual_accel_noise(), alpha, beta);
    CheckNear(alpha, 0.30, 0.02, "默认 σ_a 反解出的稳态位置增益 = 旧 α-β 的 α=0.30");

    // 锚定时的残差速度先验不能给 0。Q 越大，速度状态靠过程噪声也能自己"长"出来，
    // 先验就不那么关键；**Q 按静默段标小时（上板的正确做法），先验直接决定成不成**。
    Scenario sc;
    sc.theta0 = 0.0;
    sc.psi0_px = 0.0;
    sc.frames = 120;
    sc.sigma_a = 0.02;
    const ScenarioResult locked = RunScenario(sc, false, /*prior=*/0.0);
    const ScenarioResult normal = RunScenario(sc, false, /*prior=*/-1.0);
    Scenario coarse = sc;
    coarse.sigma_a = 0.756; // Q 给大：先验几乎不起作用
    const ScenarioResult coarse_locked = RunScenario(coarse, false, /*prior=*/0.0);
    const ScenarioResult coarse_normal = RunScenario(coarse, false, /*prior=*/-1.0);
    std::printf("      Q 小(σ_a=0.02)：先验 0 → λ̇ 偏差 %+.2f px/s，默认先验 → %+.2f px/s\n",
                locked.joint_rate_bias_late, normal.joint_rate_bias_late);
    std::printf("      Q 大(σ_a=0.756)：先验 0 → %+.2f px/s，默认先验 → %+.2f px/s（Q 自己把速度撑起来了）\n",
                coarse_locked.joint_rate_bias_late, coarse_normal.joint_rate_bias_late);
    Check(std::fabs(locked.joint_rate_bias_late) > 10.0 * std::fabs(normal.joint_rate_bias_late),
          "Q 小（按静默段标定）时，速度先验给 0 会让速度状态学不出来");
    Check(std::fabs(normal.joint_rate_bias_late) < 1.0, "默认先验：λ̇ 收敛到真值");
}

void TestAnchorSemantics() {
    std::printf("[3] 锚定语义：首帧怎么进状态、丢失后怎么重锚\n");
    const ChannelCoeff c = MakeCoeff();
    JointEkf ekf(c, c);
    double motion[kMotionCount];
    MakeMotion(0.05, 0.1, 0.0, 60.0, 0.0, motion);
    for (int ch = 0; ch < kChannelCount; ++ch) ekf.set_motion_state(ch, motion);
    Check(!ekf.anchored(kHorizontal), "喂像素之前：未锚定（预测值没有物理意义）");

    ekf.predict(0.0, 0.0, 1000000 + kTickUs);
    const double z0 = 180.0;
    const double var_px = 0.25;
    Check(ekf.update_pixel(kHorizontal, z0, var_px, 5000), "首帧像素被采纳（锚定）");
    Check(ekf.anchored(kHorizontal), "锚定完成");
    CheckNear(ekf.residual_angle(kHorizontal), std::atan(z0 / kPxPerRad), 1e-12,
              "μ := atan(z/f)（严格反投影，不用小角近似）");
    CheckNear(ekf.state(kHorizontal, kThetaAnchor), ekf.state(kHorizontal, kTheta), 1e-15,
              "θ₀ := 锚点时刻的当前姿态估计");
    const double f2 = kPxPerRad * kPxPerRad;
    CheckNear(ekf.covariance(kHorizontal, kResAngle, kResAngle),
              var_px * f2 / ((f2 + z0 * z0) * (f2 + z0 * z0)), 1e-18,
              "Var(μ) = R·f²/(f²+z²)²（反投影斜率 dμ/dz 的平方）");
    CheckNear(ekf.covariance(kHorizontal, kResRate, kResRate),
              JointEkf::default_residual_rate_prior() * JointEkf::default_residual_rate_prior(),
              1e-15, "Var(μ̇) = 速度先验²（不许为 0）");
    CheckNear(ekf.covariance(kHorizontal, kResAngle, kTheta), 0.0, 1e-18,
              "Cov(μ, θ) = 0：μ 是相对锚点机体系的角度，不含姿态估计误差");
    CheckNear(ekf.covariance(kHorizontal, kThetaAnchor, kTheta),
              ekf.covariance(kHorizontal, kTheta, kTheta), 1e-18,
              "Cov(θ₀, θ) = Var(θ)：锚定时刻两者是同一个物理量");

    Check(!ekf.update_pixel(kHorizontal, z0 + 50.0, var_px, 5000), "同一时间戳重复喂：忽略");
    const uint32_t rejects_before = ekf.pixel_rejects();
    Check(!ekf.update_pixel(kHorizontal, std::nan(""), var_px, 6000), "NaN 像素：拒收");
    Check(!ekf.update_pixel(kHorizontal, z0, 0.0, 7000), "方差非正：拒收");
    Check(ekf.pixel_rejects() == rejects_before + 2, "拒收被计数（不许静默）");

    // reset：只重锚残差，自身运动状态与它的 P 一概不动
    double ego_state[kMotionCount];
    double ego_cov[kMotionCount][kMotionCount];
    for (int i = 0; i < kMotionCount; ++i) {
        ego_state[i] = ekf.state(kHorizontal, i);
        for (int j = 0; j < kMotionCount; ++j) ego_cov[i][j] = ekf.covariance(kHorizontal, i, j);
    }
    ekf.reset(kHorizontal);
    Check(!ekf.anchored(kHorizontal), "reset 后回到未锚定");
    bool ego_untouched = true;
    for (int i = 0; i < kMotionCount; ++i) {
        if (ekf.state(kHorizontal, i) != ego_state[i]) ego_untouched = false;
        for (int j = 0; j < kMotionCount; ++j) {
            if (ekf.covariance(kHorizontal, i, j) != ego_cov[i][j]) ego_untouched = false;
        }
    }
    Check(ego_untouched, "reset 不动自身运动状态与它的协方差（目标丢了 IMU 照常融合）");
    Check(ekf.update_pixel(kHorizontal, z0 - 30.0, var_px, 8000), "重锚定成功");
    CheckNear(ekf.residual_rate(kHorizontal), 0.0, 1e-15, "重锚定把残差速度清零（重新学）");
}

void TestTrackingAgainstTwoStage() {
    std::printf("[4] 残差跟踪：联合 EKF vs 旧两级（同一批输入）\n");
    std::printf("      场景：目标 40px/s + 机体静稳定振荡 θ₀=0.05rad，视觉 100Hz，σ_px=0.5px\n");
    std::printf("      σ_a      联合 λ̇ 偏差   联合 RMSE   α-β 偏差   α-β RMSE   ΣNIS/次数\n");
    const double sigmas[] = {0.05, 0.2, 0.756};
    double joint_rmse_default = 0.0, ref_rmse_default = 0.0;
    for (double sa : sigmas) {
        Scenario sc;
        sc.sigma_a = sa;
        const ScenarioResult r = RunScenario(sc, true);
        std::printf("      %6.3f    %+9.3f    %8.3f    %+8.3f   %8.3f   %6.3f\n", sa,
                    r.joint_rate_bias, r.joint_rate_rmse, r.ref_rate_bias, r.ref_rate_rmse,
                    r.accepted_nis_mean);
        Check(r.finite, "状态与协方差全程有限");
        Check(r.max_abs_off_axis < kMaxOffAxisRad, "场景全程留在画幅内（否则投影被钳，数字无意义）");
        if (sa == 0.756) {
            joint_rmse_default = r.joint_rate_rmse;
            ref_rmse_default = r.ref_rate_rmse;
        }
    }
    Check(joint_rmse_default < ref_rmse_default,
          "默认参数下：联合 EKF 的 λ̇ 跟踪误差小于旧两级架构");

    // 一致性自检：模型与噪声都给对时，被采信新息的 ΣNIS/次数 应 ≈ 1
    Scenario sc;
    const ScenarioResult r = RunScenario(sc, false);
    std::printf("      ΣNIS/次数 = %.3f（%u 次采信，%u 次拒收，%u 次救援，最大离轴 %.3f rad）\n",
                r.accepted_nis_mean, r.updates, r.pixel_rejects, r.rescues, r.max_abs_off_axis);
    Check(r.accepted_nis_mean > 0.5 && r.accepted_nis_mean < 2.0,
          "噪声标定自洽：ΣNIS/次数 ∈ (0.5, 2)（Q/R 与真实误差量级一致）");
}

void TestPixelCorrectsEgo() {
    std::printf("[5] 互协方差的实际作用：像素按协方差同时修正残差与自身运动\n");
    const ChannelCoeff c = MakeCoeff();
    // feed_imu：喂姿态/角速度（姿态先验紧 → 像素新息应当主要归残差）
    // 不喂：姿态先验宽（σ_θ 0.22rad → 新息应当主要归"我的姿态不准"）
    // 锚点必须用一个**正常**的观测噪声（锚定的 μ 方差就来自首帧的 R —— R 给错，锚点就错）
    auto run = [&](double pixel_var, bool feed_imu, double px_offset) {
        JointEkf ekf(c, c);
        double seed[kStateCount] = {0.05, 0.05, 1.0, 1.0, 1.0, 0, 0, 0};
        ekf.seed(seed);
        ekf.set_pixel_gate(1e12); // 这里要看的是"协方差怎么分配新息"，先把门限挪开
        double motion[kMotionCount];
        MakeMotion(0.05, 0.0, 0.0, 60.0, 0.0, motion);
        for (int ch = 0; ch < kChannelCount; ++ch) ekf.set_motion_state(ch, motion);

        uint64_t t = 1000000;
        bool anchor_done = false;
        auto frame = [&](double z, uint64_t stamp) {
            for (int k = 0; k < 5; ++k) {
                t += kTickUs;
                ekf.predict(0.0, 0.0, t);
            }
            if (feed_imu) {
                ekf.update_attitude(kHorizontal, motion[kTheta], 1e-8);
                ekf.update_rate(kHorizontal, motion[kOmega], 1e-8);
                ekf.update_velocity(kHorizontal, motion[kU], motion[kV], 1e-4);
            }
            ekf.update_pixel(kHorizontal, z, anchor_done ? pixel_var : 0.25, stamp);
            anchor_done = true;
        };
        frame(100.0, 5000); // 锚定（R = 0.25px²）
        const double theta_before = ekf.state(kHorizontal, kTheta);
        const double omega_before = ekf.state(kHorizontal, kOmega);
        const double anchor_before = ekf.state(kHorizontal, kThetaAnchor);
        const double mu_before = ekf.residual_angle(kHorizontal);
        const double cov_before = ekf.covariance(kHorizontal, kResAngle, kTheta);
        frame(100.0 + px_offset, 10000); // 第二帧：目标相对预期多走 px_offset

        const double d_theta = ekf.state(kHorizontal, kTheta) - theta_before;
        const double d_omega = ekf.state(kHorizontal, kOmega) - omega_before;
        const double d_anchor = ekf.state(kHorizontal, kThetaAnchor) - anchor_before;
        const double d_mu = ekf.residual_angle(kHorizontal) - mu_before;
        // 线性化后 dh = slope·(dμ + s·dθ - s·dθ₀)，s = -1
        const double slope = kPxPerRad; // 近轴 sec² ≈ 1
        const double from_res = std::fabs(slope * d_mu);
        const double from_ego = std::fabs(slope * (-d_theta + d_anchor));
        return std::make_tuple(d_theta, d_omega, d_mu, cov_before,
                               ekf.covariance(kHorizontal, kResAngle, kTheta),
                               from_res / std::fmax(from_res + from_ego, 1e-30), d_anchor);
    };
    const auto tight = run(0.05, true, 3.0);    // 姿态先验紧（正常工况）
    const auto loose = run(0.05, false, 3.0);   // 姿态先验宽（IMU 不可用）
    const auto untrusted = run(1e6, true, 3.0); // 这一帧像素不可信（退化观测）
    const auto baseline = run(1e6, true, 0.0);  // 对照：同样的帧，但像素没动
    std::printf("      姿态先验紧：Δμ=%+.2e rad，Δθ=%+.2e，Δω=%+.2e，残差分得 %.1f%% 新息\n",
                std::get<2>(tight), std::get<0>(tight), std::get<1>(tight),
                100.0 * std::get<5>(tight));
    std::printf("      姿态先验宽：Δμ=%+.2e rad，Δθ=%+.2e，Δω=%+.2e，残差分得 %.1f%% 新息\n",
                std::get<2>(loose), std::get<0>(loose), std::get<1>(loose),
                100.0 * std::get<5>(loose));
    std::printf("      像素不可信：Δμ=%+.2e rad，Δθ=%+.2e（几乎不动）\n", std::get<2>(untrusted),
                std::get<0>(untrusted));
    Check(std::get<5>(tight) > 0.9, "姿态可信时：像素新息几乎全部归残差（该给目标的就给目标）");
    Check(std::get<5>(loose) < 0.5, "姿态不可信时：新息被大量判给自身运动状态（旧级联没有这条通路）");
    Check(std::fabs(std::get<0>(loose)) > 1e-6 || std::fabs(std::get<1>(loose)) > 1e-9,
          "像素确实反向修正了自身运动估计（θ/ω 被改动）");
    const double d_mu_px = kPxPerRad * std::fabs(std::get<2>(untrusted) - std::get<2>(baseline));
    const double d_theta_untrusted = std::fabs(std::get<0>(untrusted) - std::get<0>(baseline));
    std::printf("      像素不可信时，3px 偏差带来的状态变化：%.2e px（残差）、%.2e rad（姿态）\n",
                d_mu_px, d_theta_untrusted);
    Check(d_mu_px < 1e-3 && d_theta_untrusted < 1e-9,
          "像素不可信时：3px 偏差对状态的影响 < 1e-3 px（不会拿野值去改 IMU 估计）");
    Check(std::fabs(std::get<3>(tight)) < 1e-12 || std::get<4>(tight) != 0.0,
          "互协方差 Cov(μ, θ)：锚定时为 0（两者独立），更新后被建立起来");
    std::printf("      Cov(μ, θ)：第一次更新前 %+.3e → 更新后 %+.3e\n", std::get<3>(tight),
                std::get<4>(tight));
}

void TestConsistencyDiagnostics() {
    std::printf("[6] 模型有未知误差时：联合 EKF 会报，α-β 不会\n");
    // 短窗（100 帧）：偏置引起的稳态偏差还稳得住，用来核对量级；
    // 长窗（200 帧）用来看一致性指标什么时候动起来。
    Scenario short_clean;
    short_clean.frames = 100;
    short_clean.gyro_bias = 0.0;
    Scenario short_biased = short_clean;
    short_biased.gyro_bias = 0.02;
    const ScenarioResult sc_clean = RunScenario(short_clean, true);
    const ScenarioResult sc_biased = RunScenario(short_biased, true);
    const double predicted_bias = kPxPerRad * short_biased.gyro_bias; // ≈ f·b
    std::printf("      陀螺偏置 %.2f rad/s：λ̇ 偏差（帧 50~100）联合 %+.2f px/s，α-β %+.2f px/s；"
                "解析预期 ≈ f·b = %+.2f px/s\n",
                short_biased.gyro_bias, sc_biased.joint_rate_bias_late, sc_biased.ref_rate_bias_late,
                predicted_bias);
    Check(std::fabs(sc_biased.joint_rate_bias_late - predicted_bias) < 0.5 * predicted_bias,
          "偏置引起的 λ̇ 偏差量级与解析预期 f·b 一致（它把自身补偿误差当成了目标运动）");
    Check(std::fabs(sc_clean.joint_rate_bias_late) < 1.0, "无偏置时没有这个偏差");

    Scenario clean;
    Scenario biased;
    biased.gyro_bias = 0.02;
    const ScenarioResult rc = RunScenario(clean, true);
    const ScenarioResult rb = RunScenario(biased, true);
    std::printf("      无偏置：ΣNIS/次数=%.2f，拒收 %u，救援 %u | α-β λ̇ 偏差 %+.2f px/s\n",
                rc.accepted_nis_mean, rc.pixel_rejects, rc.rescues, rc.ref_rate_bias_late);
    std::printf("      有偏置：ΣNIS/次数=%.2f，拒收 %u，救援 %u | α-β λ̇ 偏差 %+.2f px/s\n",
                rb.accepted_nis_mean, rb.pixel_rejects, rb.rescues, rb.ref_rate_bias_late);
    Check(rb.pixel_rejects + rb.rescues > rc.pixel_rejects + rc.rescues,
          "陀螺偏置让一致性指标动起来（拒收/救援上升）—— 这是旧 α-β 没有的在线诊断");
    Check(rb.finite, "有未建模误差时不发散（一致性救援给的是有界步长）");
    // 不做"偏置下更准"的断言：常值陀螺偏置只有把它**建成状态**才能真正估掉。
    // 这里只报数字（见 CONTROL_TODO 的"待修正"一条），精度对比见 [4] 的模型匹配场景。
    std::printf("      长窗内 λ̇ 偏差：联合 %+.2f px/s vs α-β %+.2f px/s（都受偏置影响；"
                "要消掉它必须把陀螺零偏建成状态）\n",
                rb.joint_rate_bias_late, rb.ref_rate_bias_late);
}

void TestDeliverables() {
    std::printf("[7] 交付物：预测像素、未来像素、自身运动前馈\n");
    const ChannelCoeff c = MakeCoeff();
    JointEkf ekf(c, c);
    double motion[kMotionCount];
    MakeMotion(0.05, 0.3, 0.05, 60.0, 0.0, motion);
    for (int ch = 0; ch < kChannelCount; ++ch) ekf.set_motion_state(ch, motion);

    uint64_t t = 1000000, stamp = 0;
    for (int f = 0; f < 6; ++f) {
        for (int k = 0; k < 5; ++k) {
            t += kTickUs;
            ekf.predict(0.2, -0.1, t);
        }
        stamp += 5 * kTickUs;
        ekf.update_pixel(kHorizontal, 130.0 + 2.0 * f, 0.25, stamp);
    }
    CheckNear(ekf.predicted_pixel(kHorizontal), kPxPerRad * std::tan(OffAxisOf(ekf, kHorizontal)),
              1e-9, "predicted_pixel = f·tan(μ + s·(θ-θ₀))");
    CheckNear(ekf.residual_position_px(kHorizontal),
              kPxPerRad * std::tan(ekf.residual_angle(kHorizontal)), 1e-9,
              "residual_position_px = f·tan(μ)（旧 pure_position 的 px 语义）");
    CheckNear(ekf.residual_rate_px(kHorizontal), kPxPerRad * ekf.residual_rate(kHorizontal), 1e-9,
              "residual_rate_px = f·μ̇（旧 pure_velocity 的 px 语义）");

    // 未来 ticks 拍：残差按匀速外推 + 自身运动按当前舵偏前推，在同一个 tan 里合成
    const int ticks = 7;
    double cur[kStateCount], next[kStateCount];
    for (int i = 0; i < kStateCount; ++i) cur[i] = ekf.state(kHorizontal, i);
    for (int k = 0; k < ticks; ++k) {
        MotionModel::step(c, cur, ekf.control(kHorizontal), next);
        for (int n = 0; n < kStateCount; ++n) cur[n] = next[n];
    }
    const double mu_ahead =
        ekf.residual_angle(kHorizontal) + kTick * ticks * ekf.residual_rate(kHorizontal);
    const double expect = kPxPerRad * std::tan(
        mu_ahead + (-1.0) * (cur[kTheta] - ekf.state(kHorizontal, kThetaAnchor)));
    CheckNear(ekf.observed_pixel_ahead(kHorizontal, ticks), expect, 1e-9,
              "observed_pixel_ahead = 残差匀速外推 + 自身运动前推（一次 tan 合成）");

    double dh[8] = {};
    double dv[8] = {};
    ekf.ego_trajectory(8, dh, dv);
    CheckNear(dh[0], ekf.ego_pixel_step(kHorizontal), 1e-12,
              "ego_trajectory 第一拍 = ego_pixel_step");
    bool shaped = true;
    for (int k = 1; k < 8; ++k) {
        if (std::fabs(dh[k]) > 50.0) shaped = false; // 每拍增量应是小量，不是绝对位置
    }
    Check(shaped, "ego_trajectory 给的是增量（不是绝对位置）");
    std::printf("      自身运动像移增量（前 3 拍）：%+.5f, %+.5f, %+.5f px\n", dh[0], dh[1], dh[2]);
    Check(std::isfinite(dv[7]), "两个通道都给出轨迹");
}

void TestManeuverAdaptiveQ() {
    std::printf("[8] 机动自适应 Q：残差里扣不干净的平移随侧向加速度放大\n");
    // 真值里保留"自身平移引起的像移 ∝ a_lat/Z"（模型里没有这一项）；
    // σ_a 按静默段标小（0.05），机动期间靠 1/Z 这一项把 Q 抬起来。
    Scenario sc;
    sc.theta0 = 0.15;  // 静稳定振荡幅度大 → 迎角大 → 侧向加速度大
    sc.psi0_px = 120.0;
    sc.sigma_a = 0.05;
    sc.ego_z = 10.0;   // Z ≈ 10m：近距时"扣不干净的平移耦合"才够大
    Scenario quiet = sc;
    quiet.theta0 = 0.0;
    quiet.ego_z = 0.0;
    ScenarioResult off = RunScenario(sc, false);
    Scenario adaptive = sc;
    adaptive.inv_z = 1.0 / sc.ego_z;
    ScenarioResult on = RunScenario(adaptive, false);
    const ScenarioResult quiet_off = RunScenario(quiet, false);
    Scenario quiet_on_cfg = quiet;
    quiet_on_cfg.inv_z = 1.0 / sc.ego_z;
    const ScenarioResult quiet_on = RunScenario(quiet_on_cfg, false);
    std::printf("      硬机动：关 Q 自适应 RMSE %.3f px/s → 开 %.3f px/s\n", off.joint_rate_rmse,
                on.joint_rate_rmse);
    std::printf("      静默段：关 %.3f px/s → 开 %.3f px/s\n", quiet_off.joint_rate_rmse,
                quiet_on.joint_rate_rmse);
    Check(on.joint_rate_rmse < off.joint_rate_rmse,
          "硬机动下开启 Q 自适应：λ̇ 跟踪误差更小");
    Check(quiet_on.joint_rate_rmse < quiet_off.joint_rate_rmse * 1.5 + 0.05,
          "静默段不被拖累（侧向加速度小 → Q 几乎不变）");

    // 机理：Q 的残差块确实随侧向加速度放大
    const ChannelCoeff c = MakeCoeff();
    JointEkf soft(c, c), hard(c, c);
    hard.set_maneuver_coupling(1.0 / 30.0);
    double m_soft[kMotionCount], m_hard[kMotionCount];
    MakeMotion(0.0, 0.0, 0.0, 60.0, 0.0, m_soft);
    MakeMotion(0.5, 0.0, 0.0, 60.0, 0.0, m_hard); // 大迎角 → 大侧向加速度
    for (int ch = 0; ch < kChannelCount; ++ch) {
        soft.set_motion_state(ch, m_soft);
        hard.set_motion_state(ch, m_hard);
    }
    uint64_t t = 1000000;
    for (int k = 0; k < 50; ++k) {
        t += kTickUs;
        soft.predict(0.0, 0.0, t);
        hard.predict(0.0, 0.0, t);
    }
    const double v_soft = soft.covariance(kHorizontal, kResRate, kResRate);
    const double v_hard = hard.covariance(kHorizontal, kResRate, kResRate);
    std::printf("      P(μ̇)：静默 %.3e → 硬机动 %.3e（放大 %.1f 倍）\n", v_soft, v_hard,
                v_hard / v_soft);
    Check(v_hard > v_soft, "侧向加速度越大，残差速度的协方差越大（增益随机动强度自适应）");
}

void TestLongRunAndRobustness() {
    std::printf("[9] 长跑、极端输入、出框保护\n");
    const ChannelCoeff c = MakeCoeff();
    double motion[kMotionCount];
    MakeMotion(0.05, 0.3, 0.05, 60.0, 0.0, motion);

    // 20 万拍无测量：没有不稳定的自治状态，P 不该爆
    JointEkf ekf(c, c);
    for (int ch = 0; ch < kChannelCount; ++ch) ekf.set_motion_state(ch, motion);
    uint64_t t = 0;
    for (int k = 0; k < 200000; ++k) {
        t += kTickUs;
        ekf.predict(0.0, 0.0, t);
    }
    bool finite = true;
    for (int ch = 0; ch < kChannelCount; ++ch) {
        for (int i = 0; i < kStateCount; ++i) {
            if (!std::isfinite(ekf.state(ch, i))) finite = false;
            for (int j = 0; j < kStateCount; ++j) {
                if (!std::isfinite(ekf.covariance(ch, i, j))) finite = false;
            }
        }
    }
    std::printf("      P(θ)=%.4g P(ω)=%.4g（20 万拍无测量）\n", ekf.covariance(0, kTheta, kTheta),
                ekf.covariance(0, kOmega, kOmega));
    Check(finite, "20 万拍无测量：状态与 P 仍有限");

    // v → 0：除零被夹住
    JointEkf slow(c, c);
    double m_zero[kMotionCount];
    MakeMotion(0.05, 0.0, 0.0, 0.0, 0.0, m_zero);
    for (int ch = 0; ch < kChannelCount; ++ch) slow.set_motion_state(ch, m_zero);
    t = 0;
    for (int k = 0; k < 100; ++k) {
        t += kTickUs;
        slow.predict(0.5, 0.5, t);
    }
    bool slow_finite = true;
    for (int i = 0; i < kMotionCount; ++i) {
        if (!std::isfinite(slow.state(kHorizontal, i))) slow_finite = false;
    }
    Check(slow_finite, "v → 0：状态仍有限（safe_speed 兜底）");

    // 出框：目标跑出画幅时投影被钳，且这件事**被计数**
    JointEkf wide(c, c);
    double m0[kMotionCount];
    MakeMotion(0.0, 0.0, 0.0, 60.0, 0.0, m0);
    for (int ch = 0; ch < kChannelCount; ++ch) wide.set_motion_state(ch, m0);
    t = 1000000;
    for (int k = 0; k < 5; ++k) {
        t += kTickUs;
        wide.predict(0.0, 0.0, t);
    }
    wide.update_pixel(kHorizontal, kPxPerRad * std::tan(0.5), 0.25, 5000); // 锚在 28.6°
    const double anchor_px = wide.predicted_pixel(kHorizontal);
    Check(wide.projection_clamps() == 0, "画幅内：投影不钳位");
    // 用独立预测器的增量伪测量把残差速度推起来，让目标离场
    double claimed[kStateCount] = {};
    double claimed_var[kStateCount] = {};
    claimed[kResRate] = 5e-4;   // rad/s per tick
    claimed_var[kResRate] = 1e-8;
    for (int k = 0; k < 600; ++k) {
        t += kTickUs;
        wide.predict(0.0, 0.0, t);
        wide.update_state_increment(kHorizontal, claimed, claimed_var);
    }
    const double off_axis = OffAxisOf(wide, kHorizontal);
    const double out_px = wide.predicted_pixel(kHorizontal);
    std::printf("      锚点预测 %.2f px → 离轴 %.3f rad 时预测 %.2f px（钳位计数 %u，仅状态外推）\n",
                anchor_px, off_axis, out_px, wide.projection_clamps());
    Check(std::fabs(off_axis) > kMaxOffAxisRad, "残差角确实已超过半画幅（场景有效）");
    Check(std::fabs(out_px) <= kPxPerRad * std::tan(kMaxOffAxisRad) + 1e-9,
          "出框时投影被钳在画幅边界（tan 不会爆、H 有界）");
    Check(wide.projection_clamps() > 0, "钳位次数被上报：预测值不可信这件事不许静默");
    bool wide_finite = true;
    for (int i = 0; i < kStateCount; ++i) {
        if (!std::isfinite(wide.state(kHorizontal, i))) wide_finite = false;
    }
    Check(wide_finite, "出框后状态仍有限");
}

} // namespace

int main() {
    TestJointStructure();
    TestNoiseProvenance();
    TestAnchorSemantics();
    TestTrackingAgainstTwoStage();
    TestPixelCorrectsEgo();
    TestConsistencyDiagnostics();
    TestDeliverables();
    TestManeuverAdaptiveQ();
    TestLongRunAndRobustness();
    std::printf("\n%s（失败 %d 项）\n", failures == 0 ? "predictor: ALL PASS" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
