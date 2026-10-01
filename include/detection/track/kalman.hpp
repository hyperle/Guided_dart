#pragma once

// ============================================================================
// 工况 2 的核心：**带尺度预测的卡尔曼滤波**（Scale-Aware KF）
//
//   状态  x = [x, y, vx, vy, s, vs]^T
//          (x, y) 质心像素坐标；(vx, vy) 平移速度（px/s）
//          s      目标特征尺寸，取**等效半径** r = sqrt(A/π)（px）
//          vs     目标膨胀速率（px/s）—— 直接对应"逼近速度"
//
//   观测  z = [cx, cy, r]，H 取状态的第 0/1/4 行
//   模型  F = I + dt·N，N 只把 x←vx、y←vy、s←vs 连起来（匀速 + 匀速膨胀）
//   噪声  Q 用加速度白噪声模型；R_scale **随预测尺度自适应**（远→大、近→小）
//
// 为什么尺度要进状态：目标"均匀变大"意味着它的 ROI 大小本身是运动学量。
// 把 s 当独立测量去滤波（每帧重新按面积开窗）时，边缘过曝/闪烁会直接变成
// 窗口跳动，一跳就可能把目标切出 ROI；而 s 进状态后，膨胀速率 vs 会被
// 滤波器自己预测出来，窗口随之平滑增长 —— 这就是"带尺度预测"的收益。
//
// 工程约定（都是板端踩过坑之后加的）：
//   * **尺度恒正**：predict 之后 s < min_radius 就夹住，否则 R_scale(s) 会拿到负尺度
//   * **宁可不更新，不可乱更新**：新息马氏距离超门限的测量被拒收（返回 false），
//     调用方按"这一帧没测到"处理 —— 这条同时保护了"ROI 内出现更亮的干扰"
//   * **不许出 NaN**：S 不可逆时同样拒更新（linalg::inverse 返回 false）
//   * 每次更新后对称化 P：长跑几十万帧后 P 丢掉对称性会出现"负方差"
// ============================================================================

#include "detection/config/kalman.hpp"
#include "detection/config/confirmer.hpp"
#include "detection/linalg.hpp"
#include "detection/domain/observations.hpp"

namespace dart::detection {

// ---------------------------------------------------------------------------
// 尺度测量噪声模型：这是"远/近两个工况"在滤波器里的唯一差别所在。
//   远（目标小）：R_scale 大 → 测量权值低 → 靠运动模型抑制边缘过曝/闪烁带来的尺寸跳动
//   近（目标大）：R_scale 小 → 测量权值高 → 精确贴合轮廓变化（跟住膨胀）
// 做成接口是为了可替换（例如按"过曝程度/信噪比"整条曲线换掉，或改成按面积查表），
// 不是为了抽象而抽象：宿主机的对照测试会注入一条固定方差的假模型来验证滤波收敛。
// ---------------------------------------------------------------------------
class IScaleNoiseModel {
public:
    virtual ~IScaleNoiseModel() = default;

    // s_hat：预测尺度（px）；quality ∈ (0,1]：1 = 测量可信
    virtual float scale_variance(float s_hat, float quality) const = 0;
    virtual float position_variance(float s_hat, float quality) const = 0;
    virtual const char *name() const = 0;
};

// 线性过渡实现：s <= s_far 用 r_scale_far，s >= s_near 用 r_scale_near，中间线性插值。
// 质量 q 的作用是**放大方差**（q 越小越不信测量），与"远近"是两个独立的入口：
//   远处（大 R）来自尺度本身的不确定性；低质量（贴框/形状差）来自这一次测量的可信度。
class LinearScaleNoiseModel final : public IScaleNoiseModel {
public:
    explicit LinearScaleNoiseModel(const KfConfig &cfg) : cfg_(cfg) {}

    float scale_variance(float s_hat, float quality) const override;
    float position_variance(float /*s_hat*/, float quality) const override;
    const char *name() const override { return "linear(s_far→s_near)"; }

    // 只看尺度的那一档（取证/自检/板端调参）
    float base_scale_variance(float s_hat) const;
    // 当前处于远档/近档（日志用）
    const char *band(float s_hat) const;

private:
    KfConfig cfg_;
};

class ScaleAwareKalman {
public:
    static constexpr int kN = 6; // 状态维数：与"6 维状态向量"的规格一一对应

    // noise 为空时使用内置的 LinearScaleNoiseModel(cfg)（同一份配置）
    explicit ScaleAwareKalman(const KfConfig &cfg, const IScaleNoiseModel *noise = nullptr);

    void reset();
    bool initialized() const { return init_; }

    // 直接用一次测量起滤波：速度/膨胀率置 0，协方差按 init_*_var 给（很宽）
    void init_from_measurement(float cx, float cy, float radius);

    // 用启动确认的滑窗样本起滤波（推荐）：位置/尺度取最新一帧，
    // 速度/膨胀率用**最小二乘**拟合（比两点差分抗噪，3 帧数据也能用）。
    // chain/times 按**时间升序**（旧 → 新），n >= 2 才会估速度。
    void init_from_track(const Blip *chain, const uint64_t *times_us, size_t n);
    // 同上但带上时间戳，用于把状态直接外推到 now_us（避免"用旧时刻的状态开窗"）
    void init_from_track_at(const Blip *chain, const uint64_t *times_us, size_t n,
                            uint64_t now_us);

    // 状态转移。dt 由调用方给（时钟跳变/掉帧的夹取在 KfConfig.dt_min/dt_max 里）
    void predict(float dt_s);

    // 观测更新。返回 false = 这次测量被拒收（马氏门限/奇异），调用方按"没测到"处理
    bool update(const TargetMeasurement &m);

    // ---- 状态访问（命名访问器比 x[4] 可读得多，6 个字段手工展开不值得上通用向量类型）----
    float x() const { return static_cast<float>(xs_[0]); }
    float y() const { return static_cast<float>(xs_[1]); }
    float vx() const { return static_cast<float>(xs_[2]); }
    float vy() const { return static_cast<float>(xs_[3]); }
    float s() const { return static_cast<float>(xs_[4]); }
    float vs() const { return static_cast<float>(xs_[5]); }

    float sigma_x() const { return static_cast<float>(std::sqrt(var(0))); }
    float sigma_y() const { return static_cast<float>(std::sqrt(var(1))); }
    float sigma_s() const { return static_cast<float>(std::sqrt(var(4))); }

    // 取证/调参：最近一次更新的明细
    float mahalanobis2() const { return d2_; }
    float r_scale_last() const { return r_scale_last_; }
    float r_pos_last() const { return r_pos_last_; }
    float innovation_x() const { return innov_[0]; }
    float innovation_y() const { return innov_[1]; }
    float innovation_s() const { return innov_[2]; }
    // 连续被拒收的帧数：>0 说明滤波器正处在"模型跟不上"的状态（已按发散保护放大 P）。
    // 板端状态行里这个数持续 >0 就该回头看加速门限/过程噪声参数。
    uint32_t reject_streak() const { return reject_streak_; }
    float r_scale_for(float s_hat, float quality) const {
        return noise_->scale_variance(s_hat, quality);
    }
    const char        *noise_name() const { return noise_->name(); }
    const KfConfig    &config() const { return cfg_; }
    const linalg::Mat<kN, kN> &covariance() const { return P_; }

private:
    double var(int i) const { return P_(i, i) > 0.0 ? P_(i, i) : 0.0; }
    void   seed_covariance(float pos_var, float vel_var, float scale_var, float scale_vel_var);
    void   on_reject(); // 发散保护：连续拒收时放大 P（见 KfConfig.divergence_inflate）

    KfConfig                  cfg_;
    LinearScaleNoiseModel     owned_noise_; // noise_ 为空时用它
    const IScaleNoiseModel   *noise_ = nullptr;

    double                    xs_[kN]{}; // 状态向量
    linalg::Mat<kN, kN>       P_{};      // 状态协方差
    bool                      init_ = false;

    float                     r_scale_last_ = 0.0f;
    float                     r_pos_last_ = 0.0f;
    float                     d2_ = 0.0f;
    float                     innov_[3]{};
    uint32_t                  reject_streak_ = 0;
};

} // namespace dart::detection
