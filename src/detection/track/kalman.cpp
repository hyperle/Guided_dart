// ============================================================================
// ScaleAwareKalman 实现。
//
// 刻意**手写** 6 维状态转移与 3 维测量更新，而不是做一个通用卡尔曼框架：
//   * H 是"取第 0/1/4 行"这种平凡映射，写成通用矩阵反而更难读、更容易错；
//   * N/F 的稀疏结构写成循环后，P = F·P·Fᵀ + Q 退化成"加几行/几列 + 加 Q"，
//     6×6 的两次矩阵乘直接省掉（每帧几十次浮点运算）。
//   * 只有新息协方差 S（3×3）需要真正求逆，用 linalg::inverse。
// 这样 6 维状态与规格里的 [x, y, vx, vy, s, vs] 在代码里是一一对应的，
// 谁改公式都能一眼看出改的是哪一项。
// ============================================================================

#include "detection/track/kalman.hpp"

#include <cmath>

namespace dart::detection {

namespace {

// 状态下标：把"6 维状态向量"的分量写死成常量，避免到处出现裸数字
enum : int {
    kX = 0,
    kY = 1,
    kVx = 2,
    kVy = 3,
    kS = 4,
    kVs = 5,
};

// 观测下标 -> 状态下标（z = [cx, cy, r]）
constexpr int kObsIdx[3] = {kX, kY, kS};

inline double clamp_dt(double dt, const KfConfig &cfg) {
    if (dt < cfg.dt_min)
        dt = cfg.dt_min;
    if (dt > cfg.dt_max)
        dt = cfg.dt_max;
    return dt;
}

inline float quality_clamped(float q, float lo = 0.05f, float hi = 1.0f) {
    if (!(q > 0.0f))
        return lo; // NaN 也走这里：绝不让 NaN 进方差
    return q < lo ? lo : (q > hi ? hi : q);
}

} // namespace

// ---------------------------------------------------------------------------
// 噪声模型
// ---------------------------------------------------------------------------
float LinearScaleNoiseModel::base_scale_variance(float s_hat) const {
    const float sf = cfg_.s_far;
    const float sn = cfg_.s_near;
    if (!(sn > sf))
        return cfg_.r_scale_near; // 配置写反时不猜：按"近档"给（更信测量，问题更早暴露）
    float t = (s_hat - sf) / (sn - sf);
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    return cfg_.r_scale_far + t * (cfg_.r_scale_near - cfg_.r_scale_far);
}

float LinearScaleNoiseModel::scale_variance(float s_hat, float quality) const {
    const float q = quality_clamped(quality);
    // 方差 ∝ 1/q²：q=0.25（贴框）时方差放大 16 倍，运动模型立刻占主导
    float r = base_scale_variance(s_hat) / (q * q);
    if (r > cfg_.r_max)
        r = cfg_.r_max;
    return r;
}

float LinearScaleNoiseModel::position_variance(float /*s_hat*/, float quality) const {
    const float q = quality_clamped(quality);
    float       r = cfg_.r_pos / (q * q);
    if (r > cfg_.r_max)
        r = cfg_.r_max;
    return r;
}

const char *LinearScaleNoiseModel::band(float s_hat) const {
    if (s_hat <= cfg_.s_far)
        return "far";
    if (s_hat >= cfg_.s_near)
        return "near";
    return "mid";
}

// ---------------------------------------------------------------------------
// ScaleAwareKalman
// ---------------------------------------------------------------------------
ScaleAwareKalman::ScaleAwareKalman(const KfConfig &cfg, const IScaleNoiseModel *noise)
    : cfg_(cfg), owned_noise_(cfg), noise_(noise ? noise : &owned_noise_) {
    reset();
}

void ScaleAwareKalman::reset() {
    for (int i = 0; i < kN; ++i)
        xs_[i] = 0.0;
    P_ = linalg::Mat<kN, kN>::zero();
    init_ = false;
    r_scale_last_ = 0.0f;
    r_pos_last_ = 0.0f;
    d2_ = 0.0f;
    innov_[0] = innov_[1] = innov_[2] = 0.0f;
    reject_streak_ = 0;
}

// 发散保护：连续拒收就放大 P 的对角线。
// 为什么必须要有它：门限是按"模型对测量噪声的估计 R"设的。近距离 R_scale 很小
// （σ≈0.8px），目标一旦突然加速，新息会远超门限 → 测量被连续拒收 → 滤波器既不知道
// 要加速、又拒绝所有告诉它要加速的测量 → **锁死**（滞后越拉越大，最终把目标跟丢）。
// 放大 P 相当于承认"模型可能错了"：S = HPHᵀ + R 随之变大 → 门限自动放宽 → 几帧内
// 重新接受测量并追上；一旦有测量被接受，streak 立刻清零，不会长期虚高。
void ScaleAwareKalman::on_reject() {
    ++reject_streak_;
    if (cfg_.divergence_inflate > 1.0f && reject_streak_ <= cfg_.divergence_max_streak) {
        for (int i = 0; i < kN; ++i)
            P_(i, i) *= static_cast<double>(cfg_.divergence_inflate);
        P_.symmetrize();
    }
}

void ScaleAwareKalman::seed_covariance(float pos_var, float vel_var, float scale_var,
                                       float scale_vel_var) {
    P_ = linalg::Mat<kN, kN>::zero();
    P_(kX, kX) = pos_var;
    P_(kY, kY) = pos_var;
    P_(kVx, kVx) = vel_var;
    P_(kVy, kVy) = vel_var;
    P_(kS, kS) = scale_var;
    P_(kVs, kVs) = scale_vel_var;
}

void ScaleAwareKalman::init_from_measurement(float cx, float cy, float radius) {
    xs_[kX] = cx;
    xs_[kY] = cy;
    xs_[kVx] = 0.0;
    xs_[kVy] = 0.0;
    xs_[kS] = radius > cfg_.min_radius ? radius : cfg_.min_radius;
    xs_[kVs] = 0.0;
    seed_covariance(cfg_.init_pos_var, cfg_.init_vel_var, cfg_.init_scale_var,
                    cfg_.init_scale_vel_var);
    init_ = true;
}

namespace {

// 对 (t, v) 做最小二乘直线拟合，返回斜率（单位/秒）与"最后一个采样点上的拟合值"。
// 用拟合值而不是最后一点的观测值：3 帧数据下观测本身有量化噪声，
// 而滤波器初始化时最需要的是"当前位置"别被单帧噪声带偏。
bool fit_line(const float *v, const uint64_t *t_us, size_t n, float *slope_per_s, float *value_at_last) {
    if (n == 0)
        return false;
    if (n == 1) {
        *slope_per_s = 0.0f;
        *value_at_last = v[0];
        return true;
    }
    const double t_last = static_cast<double>(t_us[n - 1]) * 1e-6;
    double        st = 0.0, sv = 0.0, stt = 0.0, stv = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double ti = static_cast<double>(t_us[i]) * 1e-6 - t_last; // 以最后一帧为原点
        const double vi = static_cast<double>(v[i]);
        st += ti;
        sv += vi;
        stt += ti * ti;
        stv += ti * vi;
    }
    const double dn = static_cast<double>(n);
    const double det = dn * stt - st * st;
    if (std::fabs(det) < 1e-12) { // 时间戳全相同：没法估速度
        *slope_per_s = 0.0f;
        *value_at_last = static_cast<float>(sv / dn);
        return false;
    }
    const double slope = (dn * stv - st * sv) / det;      // 单位/秒
    const double inter = (sv - slope * st) / dn;          // t = t_last 处的值
    *slope_per_s = static_cast<float>(slope);
    *value_at_last = static_cast<float>(inter);
    return true;
}

} // namespace

void ScaleAwareKalman::init_from_track(const LightCandidate *chain, const uint64_t *times_us, size_t n) {
    init_from_measurement(chain ? chain[0].cx : 0.0f, chain ? chain[0].cy : 0.0f,
                          chain ? chain[0].radius : cfg_.min_radius);
    if (chain == nullptr || times_us == nullptr || n < 2)
        return;
    if (n > kArmMaxWindow)
        n = kArmMaxWindow; // 滑窗样本上限（编译期常数，栈上数组与之同尺寸）

    float vx = 0.0f, vy = 0.0f, rs = 0.0f, vs = 0.0f;
    float xs[kArmMaxWindow], ys[kArmMaxWindow], sr[kArmMaxWindow];
    for (size_t i = 0; i < n; ++i) {
        xs[i] = chain[i].cx;
        ys[i] = chain[i].cy;
        sr[i] = chain[i].radius;
    }
    const bool ok_x = fit_line(xs, times_us, n, &vx, &xs[n - 1]);
    const bool ok_y = fit_line(ys, times_us, n, &vy, &ys[n - 1]);
    fit_line(sr, times_us, n, &vs, &rs);

    xs_[kX] = xs[n - 1];
    xs_[kY] = ys[n - 1];
    xs_[kVx] = vx;
    xs_[kVy] = vy;
    if (rs > cfg_.min_radius)
        xs_[kS] = rs;
    xs_[kVs] = vs;

    // 拟合出来的速度比"两点差分"可信，所以速度方差给一半；
    // 若时间戳退化（ok_* 为 false），速度就是 0，方差给足别让滤波器过度自信。
    seed_covariance(cfg_.init_pos_var, ok_x && ok_y ? cfg_.init_vel_var * 0.5f : cfg_.init_vel_var,
                    cfg_.init_scale_var, cfg_.init_scale_vel_var);
    init_ = true;
}

void ScaleAwareKalman::init_from_track_at(const LightCandidate *chain, const uint64_t *times_us,
                                          size_t n, uint64_t now_us) {
    init_from_track(chain, times_us, n);
    if (!init_ || n == 0 || times_us == nullptr)
        return;
    const double dt = static_cast<double>(now_us > times_us[n - 1] ? now_us - times_us[n - 1] : 0) * 1e-6;
    if (dt > 0.0) // 把状态从"最后一帧观测时刻"外推到"本帧要开窗的时刻"
        predict(static_cast<float>(dt));
}

void ScaleAwareKalman::predict(float dt_s) {
    if (!init_)
        return;
    const double dt = clamp_dt(static_cast<double>(dt_s), cfg_);

    // 状态：x += vx·dt，y += vy·dt，s += vs·dt
    xs_[kX] += xs_[kVx] * dt;
    xs_[kY] += xs_[kVy] * dt;
    xs_[kS] += xs_[kVs] * dt;
    if (xs_[kS] < cfg_.min_radius) { // 尺度恒正（否则 R_scale(s) 会拿到负尺度）
        xs_[kS] = cfg_.min_radius;
        if (xs_[kVs] < 0.0)
            xs_[kVs] = 0.0; // 已经缩到下限还继续缩：把膨胀速率也压住，别让它继续往负方向跑
    }

    // P = F·P·Fᵀ + Q，F = I + dt·N。因为 N 只有 3 个非零元，
    // 右乘 N 就是"把父行/父列按 dt 加到子行/子列"，两次循环即可，无需矩阵乘。
    //   ① A = F·P ： A[i][j] = P[i][j] + dt·P[parent(i)][j]
    linalg::Mat<kN, kN> A = P_;
    for (int i = 0; i < kN; ++i) {
        const int p = (i == kX) ? kVx : (i == kY) ? kVy : (i == kS) ? kVs : -1;
        if (p < 0)
            continue;
        for (int j = 0; j < kN; ++j)
            A(i, j) = P_(i, j) + dt * P_(p, j);
    }
    //   ② P = A·Fᵀ ： P[i][j] = A[i][j] + dt·A[i][parent(j)]
    for (int i = 0; i < kN; ++i) {
        for (int j = 0; j < kN; ++j) {
            const int p = (j == kX) ? kVx : (j == kY) ? kVy : (j == kS) ? kVs : -1;
            P_(i, j) = (p < 0) ? A(i, j) : A(i, j) + dt * A(i, p);
        }
    }

    // Q：加速度白噪声模型（位置/尺度各一条 PSD）
    //   位置与速度的 2×2 块：[[σ²dt⁴/4, σ²dt³/2], [σ²dt³/2, σ²dt²]]
    const double d2 = dt * dt;
    const double d3 = d2 * dt;
    const double d4 = d2 * d2;
    const double qa = static_cast<double>(cfg_.sigma_a_pos) * cfg_.sigma_a_pos;
    const double qs = static_cast<double>(cfg_.sigma_a_scale) * cfg_.sigma_a_scale;
    for (int axis = 0; axis < 3; ++axis) {
        const int    ip = kObsIdx[axis];                        // 位置/尺度分量
        const int    iv = (axis == 0) ? kVx : (axis == 1) ? kVy : kVs;
        const double q = (axis == 2) ? qs : qa;
        P_(ip, ip) += q * d4 / 4.0;
        P_(ip, iv) += q * d3 / 2.0;
        P_(iv, ip) += q * d3 / 2.0;
        P_(iv, iv) += q * d2;
    }
    P_.symmetrize();
}

bool ScaleAwareKalman::update(const TargetMeasurement &m) {
    if (!m.valid)
        return false;
    if (!init_) {
        init_from_measurement(m.cx, m.cy, m.radius);
        return true;
    }

    // ---- 自适应 R：尺度那一档由**预测尺度**决定（远/近），再乘测量质量 ----
    const float s_hat = static_cast<float>(xs_[kS]);
    const float q = m.quality;
    const float r_pos = noise_->position_variance(s_hat, q);
    const float r_scale = noise_->scale_variance(s_hat, q);
    r_pos_last_ = r_pos;
    r_scale_last_ = r_scale;

    // ---- 新息 ν = z - H·x ----
    const double z[3] = {m.cx, m.cy, m.radius};
    for (int k = 0; k < 3; ++k)
        innov_[k] = static_cast<float>(z[k] - xs_[kObsIdx[k]]);

    // ---- S = H·P·Hᵀ + R（3×3，取 P 的 0/1/4 行列）----
    linalg::Mat<3, 3> S;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            S(i, j) = P_(kObsIdx[i], kObsIdx[j]);
    S(0, 0) += r_pos;
    S(1, 1) += r_pos;
    S(2, 2) += r_scale;

    linalg::Mat<3, 3> Sinv;
    if (!linalg::inverse(S, &Sinv)) {
        on_reject();
        return false; // 病态：拒更新（宁可不更新，不可乱更新）
    }

    // ---- 马氏距离门限：离群测量（更亮的干扰/闪烁）在进状态之前就被拦住 ----
    double d2 = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            d2 += static_cast<double>(innov_[i]) * Sinv(i, j) * static_cast<double>(innov_[j]);
    d2_ = static_cast<float>(d2);
    if (!(d2 <= cfg_.mahalanobis_gate)) {
        on_reject();
        return false;
    }
    reject_streak_ = 0; // 有测量被接受：发散保护立刻解除

    // ---- K = P·Hᵀ·S⁻¹（6×3）：P·Hᵀ 的第 k 列就是 P 的第 kObsIdx[k] 列 ----
    linalg::Mat<kN, 3> K;
    for (int i = 0; i < kN; ++i)
        for (int k = 0; k < 3; ++k) {
            double acc = 0.0;
            for (int j = 0; j < 3; ++j)
                acc += P_(i, kObsIdx[j]) * Sinv(j, k);
            K(i, k) = acc;
        }

    // ---- 状态更新 x += K·ν ----
    for (int i = 0; i < kN; ++i) {
        double acc = 0.0;
        for (int k = 0; k < 3; ++k)
            acc += K(i, k) * static_cast<double>(innov_[k]);
        xs_[i] += acc;
    }
    if (xs_[kS] < cfg_.min_radius)
        xs_[kS] = cfg_.min_radius;

    // ---- 协方差更新 P -= K·(H·P)（H·P 的 k 行就是 P 的第 kObsIdx[k] 行）----
    // 注意：必须先在**旧** P 上读完整的一遍再写回。就地一边读一边减会让后面的
    // (i,j) 用到已经被改过的 P，协方差会以"每帧一点"的方式悄悄错掉。
    const linalg::Mat<kN, kN> Pold = P_;
    for (int i = 0; i < kN; ++i)
        for (int j = 0; j < kN; ++j) {
            double acc = 0.0;
            for (int k = 0; k < 3; ++k)
                acc += K(i, k) * Pold(kObsIdx[k], j);
            P_(i, j) -= acc;
        }
    P_.symmetrize();
    for (int i = 0; i < kN; ++i)
        if (P_(i, i) < 0.0)
            P_(i, i) = 0.0; // 浮点误差不该产生负方差
    return true;
}

} // namespace dart::detection
