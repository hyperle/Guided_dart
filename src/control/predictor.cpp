// 联合 EKF：自身运动 + 像平面残差 —— 一个状态向量，一次估计
//
// 旧结构是**两步级联**：
//
//   IMU ──► 姿态预测器 Predictor/EkfFusion ──► 自身运动引起的像移 a_k ──┐
//                                                                      ├─► 相减 ─► 残差 ─► α-β
//   视觉像素 z_k ──────────────────────────────────────────────────────┘      (pure = z - a)
//
// 它有两个结构性缺陷，不是调参能补的：
//
//   ① 互协方差被强行清零。a_k 是**估计值**，带不确定性（陀螺噪声、气动系数标定误差、
//      未建模力矩）；相减把"目标真的动了"和"自身运动模型错了"混进同一份残差里，
//      而 α-β 只能看见合并后的那一份，无法把它们分开。旧实现里 P 是分块的
//      （姿态 5×5 与残差 2×2），交叉块 P(自身, 残差) 恒等于 0 —— 信息被拆开就回不来了。
//   ② 增益是常值。α-β 的 (α, β) 是按"残差信号有固定平滑特性"整定的常数，与当前机动
//      强度无关；最优增益应当随过程噪声（自己机动多猛、目标可不可预测）实时改变。
//
// 本文件把它改成**一个联合滤波器**：状态 = [自身运动 | 残差 | 锚点姿态]，观测是像平面上
// 两者的**非线性合成**，协方差矩阵里自然保留互协方差，增益由 K = P Hᵀ S⁻¹ 每拍现算。
// 关键是它**不重新推导视觉几何**：自身运动那一块就是把原来的气动按拍公式原封不动
// 放进 f(x,u) 的子块里，残差那一块就是 α-β 背后的匀速模型（μ' = μ + kTick·μ̇）。
//
// 状态（每通道 8 维；两通道物理上解耦，各自一套）：
//
//   x = [ θ, ω, u, v, b | μ, μ̇ | θ₀ ]
//         └── 自身运动 ──┘  └残差┘  └锚点┘
//
//   θ/ω/u/v/b ：姿态角、角速度、侧向速度、前向速度、未建模力矩偏差（**按拍**气动模型）
//   μ, μ̇     ：残差——目标自身运动 + 扣不干净的平移耦合，都归它。μ 是"视线相对**锚点**
//               机体系"的角偏差 [rad]，μ̇ 是它的变化率 [rad/s]（就是 PN 要的 λ̇）
//   θ₀        ：锚定时刻的机体系姿态 [rad]（常值，Q=0）。它让"锚点以来的转角"= θ - θ₀
//               成为**两个估计量之差**，因此姿态被修正时转角跟着修正 —— 这是旧结构做不到
//               的一件事（旧实现的 frame_px_ 是不可修正的确定性累加器）。
//
// 过程模型（按拍，不乘 dt；u = 舵偏指令）：
//
//   θ'  = θ + kTick·ω
//   ω'  = ω + k_m·v²·α_eff + b            ← 原样搬来的气动公式（含残余迎角）
//   u'  = u + k_f·v²·α_eff
//   v'  = v - k_d·v² - g·kTick·u/v
//   b'  = b
//   μ'  = μ + kTick·μ̇                     ← α-β 背后的匀速模型，逐拍推进
//   μ̇'  = μ̇
//   θ₀' = θ₀
//
//   F = ∂f/∂x 的稀疏结构见 kFPattern；气动那几行的偏导与旧 advance() 一字不差。
//
// 观测模型（像平面，非线性）：
//
//   z_k = f·tan( μ + s·(θ - θ₀) ) + v_k
//
//   s = rot_sign(ch)（机身右转 → 像左移，水平 -1；机头上抬 → 像下移，垂直 +1）。
//   这就是"自身运动与残差在像平面上的合成"：μ 是目标那一半，θ - θ₀ 是自己转的那一半，
//   两者在**同一个三角函数里**相加（不是先算一个再减另一个）。小角下退化成旧式
//   z ≈ f·μ + f·s·Δθ —— 与旧实现逐位一致；离轴角大时这个非线性是真的，不是装饰：
//   画幅边缘（30°）处斜率 sec² 比近轴大 1/3、位置比线性式差 10%，旧的线性自旋扣除
//   在这里会系统性欠补偿。
//
//   H = ∂h/∂x = f·sec²(·)·[ s(@θ), 0, 0, 0, 0, 1(@μ), 0, -s(@θ₀) ]
//
//   注意 H 在**自身运动块上非零**：像素观测会同时修正 θ（并经由 P 的互协方差修正
//   ω/u/v/b）与残差 μ —— 这正是旧级联做不到的"像素反向修正自身运动估计"。
//
// 时间：运动块与残差块都按**控制拍**推进（μ 逐拍走，不再按帧间隔跳），视觉帧只触发一次
//   像素更新。好处：帧间隔不必是拍的整数倍（旧 α-β 那条"200Hz/500Hz=2.5 拍"的误差没了），
//   而且"下一帧像素在哪"就是状态外推，调用方不用再传 dt。
//
// 约定：机体 x 前、y 右、z 下；像面 x 右、y 下；rad、m/s、px。
//   像素坐标是**绿灯中心的绝对像素**（锚点即首帧观测），预设像素的减法归调用方。
//
// 上板前必须做的三件事（其余见 CONTROL_TODO.md §11）：
//   ① 标定 σ_a（残差加速度噪声）与像素噪声 R：R 用检测层的实测像素噪声，σ_a 用静默段
//      残差的二阶差分统计；标定是否成功看 pixel_nis_sum()/pixel_updates() ≈ 1。
//   ② 目标丢失时调 reset(ch)（只重锚残差，自身运动状态不动）。
//   ③ 监控一致性指标：pixel_rejects()、consistency_rescues()、projection_clamps() 任一
//      持续增长都意味着"模型/噪声/几何有一处不对"，别当噪声忽略。
//   已知缺口：常值陀螺零偏没有建成状态，它会让 λ̇ 出现 ≈ f·b 的稳态偏差（实测与解析一致，
//   见测试 [6]）；要消掉它得在运动块上加一个零偏状态 —— 本滤波器就是它该待的地方。

#include <cmath>
#include <cstdint>

namespace dart::control {

// 角度→像素：f = (W/2)/tan(HFOV/2)，按画幅宽 640、水平视场 60° 得 ≈554。严格说它就是
// 针孔焦距 f [px]（u = f·tan(角)），只有小角下才等于"px/rad"。等标定改这一处。
constexpr double kPxPerRad = 554.0;
// 每拍的时间尺度（500Hz 控制回路）。它不是"时变源"，只是把 m/s、rad/s 换成"每拍的
// 位移、角位移"所需的单位换算。改回路频率时改这里 + 重标全部按拍系数。
constexpr double kTick = 1.0 / 500.0;
constexpr uint64_t kTickUs = (uint64_t)(kTick * 1e6);

constexpr int kChannelCount = 2;
constexpr int kHorizontal = 0; // yaw → 像 x
constexpr int kVertical = 1;   // pitch → 像 y

// 联合状态：自身运动 5 维 + 残差 2 维 + 锚点姿态 1 维
enum StateIndex {
    kTheta = 0,     // 姿态角 [rad]（按拍模型）
    kOmega,         // 角速度 [rad/s]
    kU,             // 该向线速度 [m/s]
    kV,             // 前向速度 [m/s]
    kB,             // 未建模力矩偏差 [rad/s per tick]
    kResAngle,      // 残差：视线角偏差 μ [rad]（相对锚点机体系）
    kResRate,       // 残差变化率 μ̇ [rad/s]
    kThetaAnchor,   // 锚点时刻的姿态 θ₀ [rad]（常值，Q = 0）
    kStateCount
};
constexpr int kMotionCount = kResAngle; // 运动子块长度 = 5（外部按块访问用）

// 离轴角钳位：tan 在 ±90° 爆掉，而画幅半宽只有 30°（f 就是按 60° 水平视场定的）。
// 钳到 π/6 → 投影像素恰好 ±320 px、sec² ≤ 1.33，H 有界。
// 钳的是**投影函数**，不是状态：状态可以跑到画幅外（目标出框时上游本来就不给观测）。
constexpr double kMaxOffAxisRad = 0.5235987755982988; // π/6 = 半画幅角

// 系数按拍标定，符号就是物理：k_m < 0 是静稳定（尾翼）、> 0 是静不稳定（鸭式）；
// k_f 是侧向阻尼（α 引起的力总是抵消侧向速度）；k_delta 的符号决定舵偏往哪边打。
struct ChannelCoeff {
    double k_m = 0.0;     // 力矩系数（∝v²·α_eff）
    double k_f = 0.0;     // 力系数（∝v²·α_eff）
    double k_d = 0.0;     // 阻力（∝v²）
    double k_delta = 0.0; // 舵偏 → 等效气动角（乘在 k_m/k_f 上）
    double g = 0.0;       // 重力项，只有垂直通道非 0
};

namespace {

// v 不该接近 0（无动力抛体一直在飞）；除零前就地夹住，别让 NaN 进状态
constexpr double kMinSpeed = 1.0;
double safe_speed(double v) {
    if (std::fabs(v) > kMinSpeed) return v;
    return v < 0.0 ? -kMinSpeed : kMinSpeed;
}

// 像面转动项符号：机身右转 → 像左移（水平 -1）；机头上抬 → 像下移（垂直 +1）。板端实拍确认。
double rot_sign(int ch) { return ch == kHorizontal ? -1.0 : 1.0; }

// F 的非零结构：行 → 非零列（升序，-1 结尾）。与 step() 的赋值一一对应；
// 乘法只走非零项，所以两边必须同步（测试里有一条断言盯着这张表）。
// 多留一格给结尾的 -1：满行的 -1 没有地方放，扫描就会越界（UBSan 抓过一次）。
constexpr int kFPattern[kStateCount][kStateCount + 1] = {
    {kTheta, kOmega, -1, -1, -1, -1, -1, -1, -1},      // θ'
    {kTheta, kOmega, kU, kV, kB, -1, -1, -1, -1},      // ω'（满 5 项）
    {kTheta, kU, kV, -1, -1, -1, -1, -1, -1},          // u'
    {kU, kV, -1, -1, -1, -1, -1, -1, -1},              // v'
    {kB, -1, -1, -1, -1, -1, -1, -1, -1},              // b'
    {kResAngle, kResRate, -1, -1, -1, -1, -1, -1, -1}, // μ'
    {kResRate, -1, -1, -1, -1, -1, -1, -1, -1},        // μ̇'
    {kThetaAnchor, -1, -1, -1, -1, -1, -1, -1, -1},    // θ₀'
};

// P ← F P Fᵀ + Q。P 与 Q 都对称、F 稀疏：只算上三角，只走非零项，最后镜像回下三角。
// Q 逐项加即可（上三角只有 8 个非零项，为它再养一张稀疏表不划算）。
void propagate_covariance(double P[kStateCount][kStateCount], const double F[kStateCount][kStateCount],
                          const double Q[kStateCount][kStateCount]) {
    double fp[kStateCount];
    double upper[kStateCount][kStateCount];
    for (int i = 0; i < kStateCount; ++i) {
        for (int col = 0; col < kStateCount; ++col) fp[col] = 0.0;
        for (int n = 0; kFPattern[i][n] >= 0; ++n) {
            const int k = kFPattern[i][n];
            const double f = F[i][k];
            for (int col = 0; col < kStateCount; ++col) fp[col] += f * P[k][col];
        }
        for (int j = i; j < kStateCount; ++j) {
            double sum = Q[i][j];
            for (int n = 0; kFPattern[j][n] >= 0; ++n) {
                const int k = kFPattern[j][n];
                sum += fp[k] * F[j][k];
            }
            upper[i][j] = sum;
        }
    }
    for (int i = 0; i < kStateCount; ++i) {
        for (int j = i; j < kStateCount; ++j) P[i][j] = P[j][i] = upper[i][j];
    }
}

// 投影：角度 → 像素（含钳位）。离轴超界时钳住，别让 tan 把 H 推到无穷。
// 钳位不是静默的：clamped 回传出去，交付路径会累计到 projection_clamps()（见类注释）。
double project_px(double off_axis_rad, bool *clamped = nullptr) {
    bool hit = false;
    if (off_axis_rad > kMaxOffAxisRad) {
        off_axis_rad = kMaxOffAxisRad;
        hit = true;
    } else if (off_axis_rad < -kMaxOffAxisRad) {
        off_axis_rad = -kMaxOffAxisRad;
        hit = true;
    }
    if (clamped != nullptr) *clamped = hit;
    return kPxPerRad * std::tan(off_axis_rad);
}

// 像平面合成角：残差 μ 与"锚点以来的转角" s·(θ - θ₀) 在同一个角里相加（观测模型的输入）
double off_axis_angle(int ch, const double x[kStateCount]) {
    return x[kResAngle] + rot_sign(ch) * (x[kTheta] - x[kThetaAnchor]);
}

// sec²(角)：投影的斜率。与 project_px 用同一套钳位，两者必须一致（否则 h 与 H 不匹配）。
double sec2_at(double off_axis_rad) {
    const double t = project_px(off_axis_rad) / kPxPerRad; // = tan(钳位后的角)
    return 1.0 + t * t;
}

} // namespace

// ---------------------------------------------------------------------------
// 自身运动模型：无状态、纯函数。状态由 JointEkf 持有（**一个**状态向量，不搞两份）。
//
// 它只管机体系物理（θ/ω/u/v/b 与舵偏、迎角、动压），与像素、与目标一概无关 ——
// 像素域的合成归 JointEkf。旧的 Predictor 把这两件事混在一个类里，拆开之后
// "自身运动模型只吃 IMU 与舵偏指令"这条纪律就是类型级的，不再靠注释约束。
// ---------------------------------------------------------------------------
struct MotionModel {
    // 气动角：舵偏先变成气动角，力才出来 —— "舵偏不是力"就落在这里
    static double alpha(const ChannelCoeff &c, const double x[kStateCount], double delta) {
        return x[kTheta] + c.k_delta * delta - x[kU] / safe_speed(x[kV]);
    }

    // 一拍：推进运动子块，并（可选）填 ∂f/∂x 的运动块。v、α、动压只算一次 ——
    // 分成两个函数时两处各算一遍，既是白开销，也埋着"模型和雅可比不一致"的隐患。
    // xin/xout 是**联合状态**数组（只读写前 kMotionCount 维），F 是联合 F。
    static void advance(const ChannelCoeff &c, const double xin[kStateCount], double delta,
                        double xout[kStateCount], double (*F)[kStateCount]) {
        const double v = safe_speed(xin[kV]);
        const double u = xin[kU];
        const double al = alpha(c, xin, delta);
        const double dyn = v * v;              // 动压比例项
        const double force = c.k_f * dyn * al; // 力/质量
        const double moment = c.k_m * dyn * al;

        for (int i = 0; i < kStateCount; ++i) xout[i] = xin[i];
        xout[kTheta] = xin[kTheta] + kTick * xin[kOmega];
        xout[kOmega] = xin[kOmega] + moment + xin[kB];
        xout[kU] = xin[kU] + force;
        xout[kV] = xin[kV] - c.k_d * dyn - c.g * kTick * u / v;

        if (F != nullptr) { // 只写 kFPattern 里的运动项；调用方负责清零
            F[kTheta][kTheta] = 1.0;
            F[kTheta][kOmega] = kTick;

            // ∂(k·v²·α)/∂x：α 对 θ、u、v 都有依赖，2kvα 那一项不能漏
            const double d_u = -v;
            const double d_v = 2.0 * v * al + u;
            F[kOmega][kTheta] = c.k_m * dyn;
            F[kOmega][kOmega] = 1.0;
            F[kOmega][kU] = c.k_m * d_u;
            F[kOmega][kV] = c.k_m * d_v;
            F[kOmega][kB] = 1.0;

            F[kU][kTheta] = c.k_f * dyn;
            F[kU][kU] = 1.0 + c.k_f * d_u;
            F[kU][kV] = c.k_f * d_v;

            F[kV][kU] = -c.g * kTick / v;
            F[kV][kV] = 1.0 - 2.0 * c.k_d * v + c.g * kTick * u / v / v;
        }
    }

    // 运动块一拍（无雅可比）
    static void step(const ChannelCoeff &c, const double xin[kStateCount], double delta,
                     double xout[kStateCount]) {
        advance(c, xin, delta, xout, nullptr);
    }

    // 当前侧向加速度 [m/s²]：机动强度的度量，机动自适应 Q 用它（见 JointEkf）
    static double lateral_accel(const ChannelCoeff &c, const double x[kStateCount], double delta) {
        const double v = safe_speed(x[kV]);
        return c.k_f * v * v * alpha(c, x, delta) / kTick;
    }
};

// ---------------------------------------------------------------------------
// 联合 EKF
//
// 噪声分工（本模块唯一要标定的东西，全部显式）：
//   Q 的运动块：气动模型的不确定性（系数标定误差、未建模力矩）→ 手工给
//   Q 的残差块：残差加速度噪声 σ_a [rad/s²] 按匀速模型展开成 2×2（含互项 σ_a²dt³/2）→
//               **它就是 α-β 里 (α, β) 的角色**，但按物理量给、且可随机动强度调度
//   R：IMU 各量与像素的观测噪声，按量分别给
// ---------------------------------------------------------------------------
class JointEkf {
public:
    JointEkf(const ChannelCoeff &horizontal, const ChannelCoeff &vertical)
        : coeff_{horizontal, vertical} {
        set_process_noise(kDefaultProcessNoise);
        set_residual_accel_noise(kDefaultResidualAccelNoise);
        seed(kDefaultSeedVariance);
    }

    // -----------------------------------------------------------------------
    // 噪声与先验
    // -----------------------------------------------------------------------

    // Q 的对角（运动块 + 残差的附加项）。残差主项由 set_residual_accel_noise 给，
    // 这里的 kResAngle/kResRate 是**额外**手工项（默认 0，别两边都填）。
    void set_process_noise(const double q[kStateCount]) {
        for (int i = 0; i < kStateCount; ++i) q_[i] = q[i];
    }

    // 残差加速度噪声 σ_a [rad/s²]：μ 的匀速模型过程噪声。
    // 按连续白噪声加速度展开成每拍 2×2：[[σ²dt⁴/4, σ²dt³/2], [σ²dt³/2, σ²dt²]]。
    void set_residual_accel_noise(double sigma_a_rad_s2) {
        sigma_a_ = sigma_a_rad_s2 > 0.0 ? sigma_a_rad_s2 : 0.0;
    }

    // 机动自适应 Q：残差里"扣不干净的自身平移"这一份，其像面角加速度 ≈ a_lat/Z。
    // inv_z = 1/Z_nominal [1/m]，**只是噪声定标常数**（不估计深度、不进状态）。
    // 默认 0（关）：Z 没标定前不许自动放大 Q。接线后给一个站位量级即可。
    void set_maneuver_coupling(double inv_z_per_m) { inv_z_ = inv_z_per_m > 0.0 ? inv_z_per_m : 0.0; }

    // 默认噪声的出处（测试里用它复算，防这两个数悄悄漂走）
    static constexpr double default_residual_accel_noise() { return kDefaultResidualAccelNoise; }
    static constexpr double default_residual_rate_prior() { return kDefaultResidualRatePrior; }

    // 锚定时"残差速度未知到什么程度" [rad/s]。
    // 这条是 α-β 与 EKF 最容易漏掉的一处差别：α-β 的常值 β 隐含了"残差速度先验很宽"，
    // 而卡尔曼滤波必须显式给出来。给 0 的后果不是"省事"，是**滤波器锁死**：它以为速度
    // 恒等于 0（P=0），像素新息再也灌不进速度状态，残差率永远学不出来（实测见测试 [2]）。
    void set_residual_rate_prior(double sigma_rate_rad_s) {
        rate_prior_ = sigma_rate_rad_s > 0.0 ? sigma_rate_rad_s : 0.0;
    }

    // 一致性守卫：连续拒收达 streak 次 → 判定"自己的不确定度被低估"（Q 给小了，或目标
    // 真的机动了），把这一次量测按**放大的 R** 吃进来：R_eff 取到"新息恰好落在门限上"，
    // 于是状态只朝量测走**有界的一步**（步长 ∝ 1/新息），协方差照常收缩一点。
    // 为什么不放大 P：放大 P 会让下一次更新接近"全信量测"，状态被噪声一把拽走，
    // 新息更大→再放大→发散（实测踩过）。放大 R 是单调有界的，不会自激。
    // streak = 0 等于关掉守卫（此时被拒的量测**永远不被采纳**，长此以往会锁死滤波器）。
    void set_consistency_guard(uint32_t streak) { consistency_streak_ = streak; }

    void set_gate(double mahalanobis2) { gate_ = mahalanobis2; }
    void set_pixel_gate(double mahalanobis2) { pixel_gate_ = mahalanobis2; }

    // 先验：重建 P，并**清掉锚点**（μ 与 θ₀ 重新变成"无意义"）。
    // 不动自身运动状态（那是调用方给的初值），也不动噪声设置。
    void seed() { seed(kDefaultSeedVariance); }

    void seed(const double var[kStateCount]) {
        for (int ch = 0; ch < kChannelCount; ++ch) {
            for (int i = 0; i < kStateCount; ++i) {
                for (int j = 0; j < kStateCount; ++j) P_[ch][i][j] = (i == j) ? var[i] : 0.0;
            }
            x_[ch][kResAngle] = 0.0;
            x_[ch][kResRate] = 0.0;
            x_[ch][kThetaAnchor] = 0.0;
            anchored_[ch] = false;
            last_pixel_us_[ch] = 0;
        }
    }

    // 自身运动块的初值（调用方给：出仓初速、当前姿态）。残差与锚点归 update_pixel/reset 管。
    void set_motion_state(int ch, const double motion[kMotionCount]) {
        for (int i = 0; i < kMotionCount; ++i) x_[ch][i] = motion[i];
    }

    // -----------------------------------------------------------------------
    // 递推：每拍一次（先调用它，再按需调用各个 update_*）
    // -----------------------------------------------------------------------

    // 一拍：模型推进 + 协方差传递。now_us 是这一拍的时间戳，增量伪测量要拿它对齐。
    void predict(double delta_horizontal, double delta_vertical, uint64_t now_us) {
        const double deltas[kChannelCount] = {delta_horizontal, delta_vertical};
        for (int ch = 0; ch < kChannelCount; ++ch) {
            delta_[ch] = deltas[ch];
            for (int i = 0; i < kStateCount; ++i) pre_[ch][i] = x_[ch][i];

            double F[kStateCount][kStateCount] = {};
            step(ch, F); // 推进 + 雅可比，一次评估

            for (int i = 0; i < kStateCount; ++i) pred_[ch][i] = x_[ch][i];
            // 机动自适应 Q：σ_a 随当前侧向加速度放大（inv_z_ = 0 时这一项恒为 0）
            double sigma = sigma_a_;
            if (inv_z_ > 0.0) {
                const double a_lat = MotionModel::lateral_accel(coeff_[ch], x_[ch], delta_[ch]);
                sigma += std::fabs(a_lat) * inv_z_;
            }
            build_q(ch, sigma);
            propagate_covariance(P_[ch], F, Q_[ch]);
        }
        tick_prev_us_ = tick_us_;
        tick_us_ = now_us;
    }

    // 一拍：推进联合状态 +（可选）填 ∂f/∂x。
    // 运动块走 MotionModel::advance（原气动公式）；残差块是匀速模型；θ₀ 常值。
    void step(int ch, double (*F)[kStateCount]) {
        double next[kStateCount];
        MotionModel::advance(coeff_[ch], x_[ch], delta_[ch], next, F);
        next[kResAngle] = x_[ch][kResAngle] + kTick * x_[ch][kResRate];
        next[kResRate] = x_[ch][kResRate];
        next[kThetaAnchor] = x_[ch][kThetaAnchor];
        if (F != nullptr) {
            F[kResAngle][kResAngle] = 1.0;
            F[kResAngle][kResRate] = kTick;
            F[kResRate][kResRate] = 1.0;
            F[kThetaAnchor][kThetaAnchor] = 1.0;
        }
        for (int i = 0; i < kStateCount; ++i) x_[ch][i] = next[i];
    }

    // -----------------------------------------------------------------------
    // 观测更新
    // -----------------------------------------------------------------------

    // 视觉像素（绝对像素坐标，绿灯中心）。首帧是**锚定**（见 anchor()），之后是标准更新。
    // 返回 true = 这一帧被用上了（锚定 / 更新 / 一致性救援）；false = 重复时间戳、非法值、
    // 或被门限拒收且还没到救援条件。
    bool update_pixel(int ch, double measured_px, double var_px, uint64_t frame_us) {
        if (frame_us != 0 && frame_us == last_pixel_us_[ch]) return false; // 同一帧喂两次：忽略
        if (!(var_px > 0.0) || !std::isfinite(measured_px)) {
            ++pixel_rejects_;
            return false;
        }
        last_pixel_us_[ch] = frame_us;
        if (!anchored_[ch]) {
            anchor(ch, measured_px, var_px);
            return true;
        }
        double H[kStateCount] = {};
        const double innovation = measured_px - pixel_measurement(ch, x_[ch], H);
        double s = 0.0;
        double nis = 0.0;
        if (!innovation_stats(ch, H, innovation, var_px, &s, &nis)) {
            ++pixel_rejects_;
            return false;
        }
        pixel_nis_ = nis;

        if (nis <= pixel_gate_) {
            apply_update(ch, H, innovation, s);
            pixel_nis_sum_ += nis; // 只统计被正常采信的（一致性的判据见 pixel_nis_sum 的注释）
            ++pixel_updates_;
            pixel_reject_streak_ = 0;
            reject_streak_ = 0;
            return true;
        }

        ++reject_streak_;
        ++pixel_rejects_;
        if (consistency_streak_ == 0 || ++pixel_reject_streak_ < consistency_streak_) return false;

        // 一致性救援：连续这么多次都超门限，说明"滤波器自己的协方差被低估了"（Q 给小了、
        // 或目标真在机动）。把这次量测按放大后的 R 吃进来 —— R_eff 取到"新息恰好落在门限
        // 上"，于是状态只走**有界的一步**（步长 ∝ 1/新息），协方差照常收缩一点。
        // 不放大 P 的原因见 set_consistency_guard 的注释（放大 P 会自激发散，实测踩过）。
        const double hpht = s - var_px;                       // H P Hᵀ
        const double var_eff = innovation * innovation / pixel_gate_ - hpht;
        double s_eff = 0.0;
        double nis_eff = 0.0;
        if (var_eff > 0.0 && innovation_stats(ch, H, innovation, var_eff, &s_eff, &nis_eff)) {
            apply_update(ch, H, innovation, s_eff);
            pixel_reject_streak_ = 0;
            ++consistency_rescues_;
            return true;
        }
        return false;
    }

    bool update_attitude(int ch, double theta, double var) {
        return update_by_index(ch, kTheta, theta - x_[ch][kTheta], var);
    }

    // 角速度是 IMU 最可靠的输出：不用积分、与视觉和深度都无关。
    bool update_rate(int ch, double omega, double var) {
        return update_by_index(ch, kOmega, omega - x_[ch][kOmega], var);
    }

    bool update_velocity(int ch, double u, double v, double var) {
        const bool ok_u = update_by_index(ch, kU, u - x_[ch][kU], var);
        const bool ok_v = update_by_index(ch, kV, v - x_[ch][kV], var);
        return ok_u && ok_v;
    }

    // 独立预测器对"刚刚这一拍状态变了多少"的加性伪测量。
    // 比的是**模型自己预测的增量**（pred_ − pre_），不是含修正后的增量 —— 否则测量
    // 修正会被误当成"物理上真的动了这么多"。必须先走满一拍，增量才有所指的区间。
    bool update_state_increment(int ch, const double delta[kStateCount],
                                const double var[kStateCount]) {
        if (tick_us_ <= tick_prev_us_) return false;
        bool any = false;
        for (int i = 0; i < kStateCount; ++i) {
            if (!(var[i] > 0.0)) continue;
            const double model_delta = pred_[ch][i] - pre_[ch][i];
            if (update_by_index(ch, i, delta[i] - model_delta, var[i])) any = true;
        }
        return any;
    }

    // -----------------------------------------------------------------------
    // 交付物
    // -----------------------------------------------------------------------

    // 锚定过了吗？没锚定之前残差与 θ₀ 没有物理意义，**别用下面的预测值**
    bool anchored(int ch) const { return anchored_[ch]; }

    // 丢掉锚点（目标丢失 / 跟踪器硬复位时调）。**自身运动状态与它的 P 一概不动** ——
    // 这正是联合滤波的收益：目标丢了，IMU 照常融合，重新捕获时不用重新收敛。
    void reset(int ch) {
        anchored_[ch] = false;
        last_pixel_us_[ch] = 0;
        x_[ch][kResAngle] = 0.0;
        x_[ch][kResRate] = 0.0;
        x_[ch][kThetaAnchor] = 0.0;
        for (int i = 0; i < kStateCount; ++i) {
            P_[ch][kResAngle][i] = P_[ch][i][kResAngle] = 0.0;
            P_[ch][kThetaAnchor][i] = P_[ch][i][kThetaAnchor] = 0.0;
        }
    }

    void reset() {
        for (int ch = 0; ch < kChannelCount; ++ch) reset(ch);
    }

    // 残差（目标那一半）：μ [rad] 与 μ̇ [rad/s]。μ̇ 就是 PN 要的 λ̇。
    double residual_angle(int ch) const { return x_[ch][kResAngle]; }
    double residual_rate(int ch) const { return x_[ch][kResRate]; }

    // 旧 α-β 的 px 域语义（迁移用）：μ 与 μ̇ 折算成"扣掉自身转动后的像面位置/速度"。
    // 位置用严格投影 f·tan(μ)；速度用**小角斜率** f·μ̇（与旧 pure_velocity 同定义）。
    double residual_position_px(int ch) const { return project_px(x_[ch][kResAngle]); }
    double residual_rate_px(int ch) const { return kPxPerRad * x_[ch][kResRate]; }

    // 锚点以来自身转动造成的像移 [px]（在当前视线角上取值：同一转角，离轴越远像移越大）
    double ego_pixel_shift(int ch) const { return predicted_pixel(ch) - residual_position_px(ch); }

    // 当前状态给出的像素预测 h(x̂) = 预测的绿灯中心像素。
    // 目标已出画幅（|合成角| > 半画幅）时投影被钳在 ±320px，并计入 projection_clamps()：
    // 这时它**不是**可信的预测值，控制律/ROI 必须按"目标出框"处理（见文件头）。
    double predicted_pixel(int ch) const {
        bool clamped = false;
        const double px = project_px(off_axis_angle(ch, x_[ch]), &clamped);
        if (clamped) ++projection_clamps_;
        return px;
    }

    // 未来 ticks 拍之后绿灯中心会在哪个像素：
    //   残差按匀速模型外推 + 自身运动按当前舵偏冻结前推，在同一个 tan 里合成
    double observed_pixel_ahead(int ch, int ticks) const {
        const double mu = x_[ch][kResAngle] + kTick * ticks * x_[ch][kResRate];
        double cur[kStateCount];
        double next[kStateCount];
        for (int i = 0; i < kStateCount; ++i) cur[i] = x_[ch][i];
        for (int i = 0; i < ticks; ++i) {
            MotionModel::step(coeff_[ch], cur, delta_[ch], next);
            for (int k = 0; k < kStateCount; ++k) cur[k] = next[k];
        }
        bool clamped = false;
        const double px = project_px(mu + rot_sign(ch) * (cur[kTheta] - x_[ch][kThetaAnchor]), &clamped);
        if (clamped) ++projection_clamps_;
        return px;
    }

    // 未来 ticks 拍里**自身转动**贡献的像移增量（相对现在；残差角冻结在当前值）。
    // 交给控制律当前馈：告诉外环"按现在的舵偏，像素本来会被自己带到哪去"。
    // 注意每次调用有 2·ticks 次 tan（libm）：它不在自稳/内环的热路径上，但调用点别放
    // 在 500Hz 的每拍上。
    void ego_trajectory(int ticks, double out_horizontal[], double out_vertical[]) const {
        const int chs[kChannelCount] = {kHorizontal, kVertical};
        double *outs[kChannelCount] = {out_horizontal, out_vertical};
        for (int i = 0; i < kChannelCount; ++i) {
            const int ch = chs[i];
            double cur[kStateCount];
            double next[kStateCount];
            for (int k = 0; k < kStateCount; ++k) cur[k] = x_[ch][k];
            const double s_sign = rot_sign(ch);
            bool clamped = false;
            const double base_px = project_px(x_[ch][kResAngle] + s_sign * (cur[kTheta] - x_[ch][kThetaAnchor]), &clamped);
            for (int k = 0; k < ticks; ++k) {
                MotionModel::step(coeff_[ch], cur, delta_[ch], next);
                for (int n = 0; n < kStateCount; ++n) cur[n] = next[n];
                const double ahead = x_[ch][kResAngle] + s_sign * (cur[kTheta] - x_[ch][kThetaAnchor]);
                outs[i][k] = project_px(ahead, &clamped) - base_px; // 增量，不是绝对位置
            }
            if (clamped) ++projection_clamps_;
        }
    }

    // 一拍的自身转动像移（ego_trajectory 的第一拍）
    double ego_pixel_step(int ch) const {
        double dh[1];
        double dv[1];
        ego_trajectory(1, dh, dv);
        return ch == kHorizontal ? dh[0] : dv[0];
    }

    // 当前气动角 [rad]（内环/能量预算/前馈都要它）
    double alpha(int ch) const { return MotionModel::alpha(coeff_[ch], x_[ch], delta_[ch]); }

    double control(int ch) const { return delta_[ch]; }

    // -----------------------------------------------------------------------
    // 访问与诊断
    // -----------------------------------------------------------------------

    double state(int ch, int i) const { return x_[ch][i]; }
    double covariance(int ch, int i, int j) const { return P_[ch][i][j]; }
    double nis() const { return nis_; }
    double pixel_nis() const { return pixel_nis_; }
    uint32_t reject_streak() const { return reject_streak_; }
    uint32_t pixel_rejects() const { return pixel_rejects_; }
    uint32_t pixel_reject_streak() const { return pixel_reject_streak_; }
    // 一致性救援次数：>0 就说明"Q 给小了"（或目标真在剧烈机动）—— 板端必须看得见
    uint32_t consistency_rescues() const { return consistency_rescues_; }
    // 投影被钳到画幅边界（±320px）的次数：>0 就是"目标已经出框，预测值不可信"，
    // 控制律/ROI 必须按出框处理 —— 这个数字不许被忽略
    uint32_t projection_clamps() const { return projection_clamps_; }
    // 像素一致性累计量：ΣNIS / 次数 ≈ 1 说明 Q/R 与真实误差量级自洽（标定靠它，不靠手感）。
    // 只统计**正常采信**的那些：被拒收的与救援进来的本身就是一致性异常的证据，
    // 混进统计量会把"越大越正常"变成结论。
    double pixel_nis_sum() const { return pixel_nis_sum_; }
    uint32_t pixel_updates() const { return pixel_updates_; }
    void reset_counters() {
        nis_ = pixel_nis_ = pixel_nis_sum_ = 0.0;
        reject_streak_ = pixel_rejects_ = pixel_updates_ = 0;
        pixel_reject_streak_ = consistency_rescues_ = 0;
        projection_clamps_ = 0;
    }

private:
    // 默认先验：姿态/角速度/速度/偏差沿用旧 EkfFusion 的占位；残差与 θ₀ 在锚定前无意义 → 0
    static constexpr double kDefaultSeedVariance[kStateCount] = {0.01, 0.01, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0};
    static constexpr double kDefaultProcessNoise[kStateCount] = {1e-8, 1e-8, 1e-6, 1e-6, 1e-8, 0.0, 0.0, 0.0};
    // 残差加速度噪声占位 [rad/s²]：由旧 α-β 的 α=0.30（200Hz、σ_px=0.5px）反解出的
    // "等效匀速模型过程噪声"，这样上板手感与旧值连续。反解算法与复算断言在
    // tests/host/predictor_host_test.cpp 的 [2]（同一个数在测试里复算，防它悄悄漂走）。
    // 标定后改这里：用实测残差的二阶差分统计，或看 pixel_nis_sum()/pixel_updates() ≈ 1。
    static constexpr double kDefaultResidualAccelNoise = 0.756;
    // 锚定时残差速度先验 [rad/s]：0.1 rad/s ≈ 55 px/s（目标横移几十 px/s 是常态）。
    // 它只影响**捕获后的前几帧**，稳态由 Q 与 R 定；给太小 → 前几帧学不出速度。
    static constexpr double kDefaultResidualRatePrior = 0.1;

    // h(x) 与 H = ∂h/∂x（H 为 nullptr 时只算 h）
    static double pixel_measurement(int ch, const double x[kStateCount], double H[kStateCount]) {
        const double s = rot_sign(ch);
        const double off_axis = off_axis_angle(ch, x);
        if (H != nullptr) {
            const double slope = kPxPerRad * sec2_at(off_axis);
            H[kTheta] = s * slope;
            H[kResAngle] = slope;
            H[kThetaAnchor] = -s * slope;
        }
        return project_px(off_axis);
    }

    // 首帧锚定：θ₀ := 当前姿态估计，μ := atan(z/f)；协方差按"这两个量此刻的定义"给。
    //   ① μ 的误差只有测量噪声：dμ/dz = f/(f²+z²) → Var(μ) = R·f²/(f²+z²)²
    //   ② μ̇ 完全未知 → 用先验宽度 rate_prior_（**不能给 0**，否则速度状态被锁死）
    //   ③ θ₀ 与 θ 在锚定时刻是**同一个物理量**（都在估"此刻的真实姿态"）→ θ₀ 的行/列
    //      直接复制 θ 的行/列（含 Var(θ₀)=Var(θ)、Cov(θ₀,θ)=Var(θ)）
    //   ④ μ 与运动块独立（μ 是"相对锚点机体系"的角度，不含姿态估计误差）→ 互协方差 0
    void anchor(int ch, double measured_px, double var_px) {
        x_[ch][kResAngle] = std::atan(measured_px / kPxPerRad); // 反投影（严格，不用小角近似）
        x_[ch][kResRate] = 0.0;
        x_[ch][kThetaAnchor] = x_[ch][kTheta];

        for (int i = 0; i < kStateCount; ++i) {
            P_[ch][kResAngle][i] = P_[ch][i][kResAngle] = 0.0;
            P_[ch][kResRate][i] = P_[ch][i][kResRate] = 0.0;
            P_[ch][kThetaAnchor][i] = P_[ch][i][kThetaAnchor] = 0.0;
        }
        const double f2 = kPxPerRad * kPxPerRad;
        const double z2 = measured_px * measured_px;
        P_[ch][kResAngle][kResAngle] = var_px * f2 / ((f2 + z2) * (f2 + z2));
        P_[ch][kResRate][kResRate] = rate_prior_ * rate_prior_;
        for (int i = 0; i < kStateCount; ++i) {
            P_[ch][kThetaAnchor][i] = P_[ch][kTheta][i];
            P_[ch][i][kThetaAnchor] = P_[ch][i][kTheta];
        }
        anchored_[ch] = true;
    }

    bool update_by_index(int ch, int index, double innovation, double var) {
        double H[kStateCount] = {};
        H[index] = 1.0;
        return scalar_update(ch, H, innovation, var);
    }

    // 标量更新的公共部分：算 P Hᵀ、新息方差 S 与 NIS（**不改状态**）。
    // 拆成"算统计量"与"落状态"两步，是为了让像素那条路能在超门限时用放大后的 R 重来一次
    // （一致性救援），而不是把拒收/救援的分支塞进更新本体里。
    bool innovation_stats(int ch, const double H[kStateCount], double innovation, double var,
                          double *s_out, double *nis_out) const {
        double ph = 0.0;
        for (int i = 0; i < kStateCount; ++i) {
            double row = 0.0;
            for (int j = 0; j < kStateCount; ++j) row += P_[ch][i][j] * H[j];
            ph += H[i] * row;
        }
        const double s = var + ph; // = H P Hᵀ + R
        if (!(s > 0.0) || !std::isfinite(s) || !std::isfinite(innovation)) return false;
        *s_out = s;
        *nis_out = innovation * innovation / s;
        return true;
    }

    // 落地：x += K·innovation，P -= K H P（对称，只算上三角再镜像）。S 由调用方给。
    void apply_update(int ch, const double H[kStateCount], double innovation, double s) {
        double ph[kStateCount];
        for (int i = 0; i < kStateCount; ++i) {
            double row = 0.0;
            for (int j = 0; j < kStateCount; ++j) row += P_[ch][i][j] * H[j];
            ph[i] = row;
        }
        const double inv_s = 1.0 / s;
        for (int i = 0; i < kStateCount; ++i) x_[ch][i] += ph[i] * inv_s * innovation;
        for (int i = 0; i < kStateCount; ++i) {
            for (int j = i; j < kStateCount; ++j) {
                P_[ch][i][j] = P_[ch][j][i] = P_[ch][i][j] - ph[i] * inv_s * ph[j];
            }
        }
    }

    // 带门限的标量更新（IMU 各量走这条；像素的正常路径在 update_pixel 里显式走上面两步）。
    // 一次一个测量，不用矩阵求逆。宁可不更新，不可乱更新。
    bool scalar_update(int ch, const double H[kStateCount], double innovation, double var) {
        double s = 0.0;
        double nis = 0.0;
        if (!innovation_stats(ch, H, innovation, var, &s, &nis)) {
            ++reject_streak_;
            return false;
        }
        nis_ = nis;
        if (nis > gate_) {
            ++reject_streak_;
            return false;
        }
        reject_streak_ = 0;
        apply_update(ch, H, innovation, s);
        return true;
    }

    // Q = 对角 + 残差匀速模型块（含 σ²dt³/2 的互项 —— 丢了这个互项，过程侧就不承认
    // "位置误差与速度误差相关"，残差的状态协方差会系统性偏小、增益偏激进）
    void build_q(int ch, double sigma_a) {
        for (int i = 0; i < kStateCount; ++i) {
            for (int j = 0; j < kStateCount; ++j) Q_[ch][i][j] = 0.0;
        }
        for (int i = 0; i < kStateCount; ++i) Q_[ch][i][i] = q_[i];
        const double dt2 = kTick * kTick;
        const double sa2 = sigma_a * sigma_a;
        Q_[ch][kResAngle][kResAngle] += sa2 * dt2 * dt2 / 4.0;
        Q_[ch][kResAngle][kResRate] += sa2 * dt2 * kTick / 2.0;
        Q_[ch][kResRate][kResAngle] += sa2 * dt2 * kTick / 2.0;
        Q_[ch][kResRate][kResRate] += sa2 * dt2;
        Q_[ch][kThetaAnchor][kThetaAnchor] = 0.0; // 锚点姿态是常值
    }

    ChannelCoeff coeff_[kChannelCount];
    double x_[kChannelCount][kStateCount] = {};
    double P_[kChannelCount][kStateCount][kStateCount] = {};
    double pre_[kChannelCount][kStateCount] = {};  // 推进前
    double pred_[kChannelCount][kStateCount] = {}; // 纯模型推进后（未含修正）
    double Q_[kChannelCount][kStateCount][kStateCount] = {};
    double q_[kStateCount] = {};
    double delta_[kChannelCount] = {};
    double sigma_a_ = 0.0;
    double inv_z_ = 0.0;
    double rate_prior_ = kDefaultResidualRatePrior;
    uint32_t consistency_streak_ = 5;
    bool anchored_[kChannelCount] = {};
    uint64_t last_pixel_us_[kChannelCount] = {};
    uint64_t tick_us_ = 0;
    uint64_t tick_prev_us_ = 0;
    double gate_ = 6.63;       // 马氏距离² χ²(1,0.99)：IMU 各量
    double pixel_gate_ = 6.63; // 同上：像素。上游检测层也有门限，这里再拦一道 ——
                               // 像素现在会修正自身运动状态，野值不再只是"残差不好看"
    double nis_ = 0.0;
    double pixel_nis_ = 0.0;
    double pixel_nis_sum_ = 0.0;
    uint32_t pixel_updates_ = 0;
    uint32_t reject_streak_ = 0;
    uint32_t pixel_rejects_ = 0;
    uint32_t pixel_reject_streak_ = 0;
    uint32_t consistency_rescues_ = 0;
    mutable uint32_t projection_clamps_ = 0; // const 交付函数里也要能计数
};

} // namespace dart::control
