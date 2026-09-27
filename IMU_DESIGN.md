// ============================================================================
// IMU 模块伪代码：单生产者（IMU 中断）→ 多消费者（飞控环 / 视觉 / 预测器）
//
// 职责：**积分在模块内**。
//   模块：标定 → 姿态（角速度积分）、线速度（比力积分 − 重力）→ 发布
//   消费侧：只消费结果（姿态角、角速度、线速度），或按区间要增量，或要未积分原始数据
//
// 为什么线速度也归模块：比力积分要减重力，而重力得用**姿态**转到机体系，姿态只在
//   模块里维护。放外面做，等于把姿态再导出一遍，还得保证两侧同拍。
//
// 解耦：姿态积分只用角速度，不看线速度；线速度积分读姿态（单向往），不写姿态。
//
// 两条轨道，因为需求根本不同：
//   latest  —— "现在是多少"：seqlock，读几个时钟周期
//   history —— "刚才那段转了多少 / 速度变了多少"：环形缓冲 + 时间戳精确可查
//
// 全模块无锁：ISR 不能阻塞，也不能让 50Hz 的日志任务拿锁去堵姿态环。
// ============================================================================

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#define IMU_HISTORY_CAPACITY 512u // 2 的幂；1kHz 下 0.5s，entry ~56B ≈ 28KB
#define IMU_HISTORY_MASK (IMU_HISTORY_CAPACITY - 1u)
#define IMU_SEQLOCK_RETRY 8u
#define IMU_GAP_US 20000u    // 相邻两帧间隔超过它 = 丢过数据
#define IMU_WINDOW_MAX 64u   // 单次区间查询最多跨这么多帧
#define IMU_ACC_RANGE 200.0f // 加速度计量程（按实机传感器填）
#define IMU_GRAVITY 9.7949f

typedef struct {
    float gyro[3];
    float acc[3];
    uint64_t timestamp;
    bool saturated;
} imu_raw_t;

typedef struct {
    float angle[3];
    float rate[3];
    float velocity[3];
    uint64_t timestamp;
    bool velocity_valid;
} imu_state_t;

typedef struct {
    imu_raw_t raw;      // 标定后的原始数据（含 saturated）
    float cum_angle[3]; // 开机以来累计转角（单调，不因 set_reference 归零）
    bool gap;           // 与前一帧之间有缺口
} imu_history_entry_t;

typedef struct __attribute__((aligned(64))) {
    _Atomic uint32_t sequence; // 用 _Atomic 而不是 volatile：对齐 32 位访问不撕裂是"隐性依赖"
    imu_state_t data;
} imu_latest_t;

typedef struct {
    // head 单独占一条 cache line：生产者每帧写它，消费者只读它
    _Atomic uint32_t head __attribute__((aligned(64)));
    imu_history_entry_t entries[IMU_HISTORY_CAPACITY];
} imu_history_t;

static imu_latest_t latest;
static imu_history_t history;
static imu_state_t state;
static imu_raw_t raw_latest;
static uint64_t last_ts_us;

static inline const imu_history_entry_t *at(uint32_t i) {
    return &history.entries[i & IMU_HISTORY_MASK];
}

// ---------------------------------------------------------------------------
// 生产者：只在 IMU 中断里调用，全模块唯一的写者
// ---------------------------------------------------------------------------
void imu_provider_post_sample(const float raw_gyro[3], const float raw_acc[3],
                              uint64_t timestamp_us) {
    imu_raw_t r;
    calibrate(raw_gyro, raw_acc, &r); // 零偏 → 温漂 → 相机系转机体系
    r.timestamp = timestamp_us;
    r.saturated = fabsf(raw_acc[0]) > IMU_ACC_RANGE || fabsf(raw_acc[1]) > IMU_ACC_RANGE ||
                  fabsf(raw_acc[2]) > IMU_ACC_RANGE;

    integrate(&r);
    publish_latest();
    publish_history(&r);
    raw_latest = r; // 未积分的给需要自己算的调用方
}

static void integrate(const imu_raw_t *r) {
    // 采样不等周期 → 模块内部必须用 dt（与控制律的"按拍、不乘 dt"是两码事）
    float dt = (float)(r->timestamp - last_ts_us) * 1e-6f;
    last_ts_us = r->timestamp;
    if (!(dt > 0.0f) || dt > (float)IMU_GAP_US * 1e-6f) return; // 时间戳异常/长缺口：不积

    // ① 姿态：只用角速度，与线速度完全解耦
    for (int a = 0; a < 3; ++a) {
        state.angle[a] += r->gyro[a] * dt;
        state.rate[a] = r->gyro[a];
    }

    // ② 线速度：比力 − 重力（重力按当前姿态转到机体系）；发射段量程打满 → 不积
    if (state.velocity_valid && !r->saturated) {
        float g_body[3];
        rotate_to_body(IMU_GRAVITY, state.angle, g_body);
        for (int a = 0; a < 3; ++a) state.velocity[a] += (r->acc[a] - g_body[a]) * dt;
    }
    state.timestamp = r->timestamp;
}

// ---------------------------------------------------------------------------
// 内部工具：发布与环形缓冲
// ---------------------------------------------------------------------------
static void publish_latest(void) {
    uint32_t seq = atomic_load_explicit(&latest.sequence, memory_order_relaxed);
    atomic_store_explicit(&latest.sequence, seq + 1, memory_order_relaxed); // 奇数 = 正在写
    atomic_thread_fence(memory_order_release);
    latest.data = state;
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&latest.sequence, seq + 2, memory_order_relaxed); // 偶数 = 写完
}

static void publish_history(const imu_raw_t *r) {
    uint32_t head = history.head;
    imu_history_entry_t *e = &history.entries[head & IMU_HISTORY_MASK];

    if (head == 0) {
        e->gap = false;
        e->cum_angle[0] = e->cum_angle[1] = e->cum_angle[2] = 0.0f;
    } else {
        const imu_history_entry_t *p = at(head - 1);
        e->gap = r->timestamp - p->raw.timestamp > IMU_GAP_US;
        float dt = (float)(r->timestamp - p->raw.timestamp) * 1e-6f;
        for (int a = 0; a < 3; ++a) { // 与前一帧做梯形积分
            e->cum_angle[a] = p->cum_angle[a] + 0.5f * (p->raw.gyro[a] + r->gyro[a]) * dt;
        }
    }

    e->raw = *r;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    history.head = head + 1; // head 动了这一格才可读
}

// 取 [start, end] 的窗口副本，并在副本上算：
//   越界/gap/区间过长/拷贝期间被覆盖 → false
static bool snapshot(uint64_t start_us, uint64_t end_us, imu_history_entry_t *window, uint32_t *n) {
    uint32_t head = history.head;
    if (head == 0) return false;

    uint32_t count = head < IMU_HISTORY_CAPACITY ? head : IMU_HISTORY_CAPACITY;
    uint32_t first = head - count;

    if (start_us < at(first)->raw.timestamp) return false; // 太老：已被覆盖
    uint64_t newest_us = at(head - 1)->raw.timestamp;
    if (end_us > newest_us) end_us = newest_us; // 未来：截断
    if (end_us <= start_us) return false;

    uint32_t i = head - 1;
    while (i > first && at(i)->raw.timestamp > start_us) --i;   // 停在 start 的左邻
    uint32_t j = i;
    while (j + 1 < head && at(j)->raw.timestamp < end_us) ++j;  // 停在覆盖 end 的那帧
    uint32_t cnt = j - i + 1;
    if (cnt > IMU_WINDOW_MAX) return false;

    for (uint32_t k = 0; k < cnt; ++k) window[k] = *at(i + k);
    if (history.head - first > IMU_HISTORY_CAPACITY) return false; // 拷贝期间被追上
    for (uint32_t k = 0; k < cnt; ++k) {
        if (window[k].gap) return false; // 这段里丢过数据
    }
    *n = cnt;
    return true;
}

// 时间戳落在两帧之间 → 在累计曲线上插值（等于对速率做梯形积分）
static bool cum_at(const imu_history_entry_t *w, uint32_t n, uint64_t t, const float *cum_first,
                   float *out) {
    for (uint32_t k = 0; k + 1 < n; ++k) {
        if (t > w[k + 1].raw.timestamp) continue;
        if (t < w[k].raw.timestamp) break;
        uint64_t span = w[k + 1].raw.timestamp - w[k].raw.timestamp;
        if (span == 0) return false;
        float ratio = (float)(t - w[k].raw.timestamp) / (float)span;
        *out = cum_first[k] + ratio * (cum_first[k + 1] - cum_first[k]);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// 参考系与初速度：发射前 / 出仓瞬间各调一次；两侧以这次调用为共同基准
// ---------------------------------------------------------------------------
void imu_set_reference(void) {
    for (int a = 0; a < 3; ++a) state.angle[a] = 0.0f; // 此刻姿态 = 初始坐标系
    // cum_angle 不清零：区间查询靠差分，基准跳变对它不可见
}

void imu_seed_initial_velocity(const float velocity[3]) {
    for (int a = 0; a < 3; ++a) state.velocity[a] = velocity[a];
    state.velocity_valid = true; // 之前 velocity 无意义，消费侧靠这个位判断
}

// ---------------------------------------------------------------------------
// 消费者 A（快轨）：最新积分结果。非阻塞、无锁、有界重试
// ---------------------------------------------------------------------------
bool imu_get_state(imu_state_t *out_state) {
    for (uint32_t retry = 0; retry < IMU_SEQLOCK_RETRY; ++retry) {
        uint32_t seq = atomic_load_explicit(&latest.sequence, memory_order_relaxed);
        if (seq & 1u) continue; // 正好撞上写入：重试

        atomic_thread_fence(memory_order_acquire);
        *out_state = latest.data;
        atomic_thread_fence(memory_order_acquire);

        if (atomic_load_explicit(&latest.sequence, memory_order_relaxed) == seq) return true;
    }
    return false; // 调用方按"这拍没有数据"处理
}

bool imu_get_raw_latest(imu_raw_t *out_raw) {
    // raw_latest 也是"最后写入者胜"，由生产者顺序写；消费侧只需要一个自洽快照
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *out_raw = raw_latest;
    return true;
}

// ---------------------------------------------------------------------------
// 消费者 B（慢轨）：区间量。越界、丢数据、时钟异常都在模块内部消化
//
//   陀螺与加速度计是两路：**加速度计量程打满不影响角度积分** —— 所以发射段的
//   角度查询照常可用，速度查询必须拒。
//
//   消费侧对应关系：
//     roll 自稳（auto_stability.cpp）—— 区间角度，除以窗长得窗口平均角速度
//     预测器（predictor.cpp）      —— 区间角度（写进 EKF 观 θ）+ 当前线速度（观 u/v）
//     视觉去自旋                   —— imu_get_delta_yaw_between 这一个包装
// ---------------------------------------------------------------------------
bool imu_get_delta_angle_between(uint64_t start_us, uint64_t end_us, float out_delta[3]) {
    imu_history_entry_t window[IMU_WINDOW_MAX];
    uint32_t n;
    if (!snapshot(start_us, end_us, window, &n)) return false;

    float cum[IMU_WINDOW_MAX][3];
    for (uint32_t k = 0; k < n; ++k) {
        for (int a = 0; a < 3; ++a) cum[k][a] = window[k].cum_angle[a];
    }

    float a[3], b[3];
    for (int ax = 0; ax < 3; ++ax) {
        if (!cum_at(window, n, start_us, &cum[0][ax], &a[ax])) return false;
        if (!cum_at(window, n, end_us, &cum[0][ax], &b[ax])) return false;
    }
    for (int ax = 0; ax < 3; ++ax) out_delta[ax] = b[ax] - a[ax];
    return true;
}

bool imu_get_delta_velocity_between(uint64_t start_us, uint64_t end_us, float out_delta[3]) {
    imu_history_entry_t window[IMU_WINDOW_MAX];
    uint32_t n;
    if (!snapshot(start_us, end_us, window, &n)) return false;

    for (uint32_t k = 0; k < n; ++k) {
        if (window[k].raw.saturated) return false; // 这段里量程打满过 → 速度不可信
    }
    // 历史里存的是当帧的 state.velocity（模块已积好的绝对值），差分即可
    for (int a = 0; a < 3; ++a) out_delta[a] = window[n - 1].velocity[a] - window[0].velocity[a];
    return true;
}

// 视觉只关心去自旋 → 三轴角度查询的 z 分量包装
bool imu_get_delta_yaw_between(uint64_t start_us, uint64_t end_us, float *out_delta_yaw) {
    float d[3];
    if (!imu_get_delta_angle_between(start_us, end_us, d)) return false;
    *out_delta_yaw = d[2];
    return true;
}

// ============================================================================
// 边界一览（全部在模块内部消化，调用方只看 true/false）
//
//   start 早于缓冲区最老一帧     → false
//   end 晚于最新一帧（时钟不同步）→ 截断到最新帧再算
//   end <= start                → false
//   区间内 gap                   → false（跨过一段未知时间）
//   区间跨帧数 > IMU_WINDOW_MAX  → false（调用方时间戳算错了）
//   区间内含加速度计饱和帧        → 角度查询照常，速度查询 false
//   seqlock 连续冲突超过重试上限  → false
//
// 待定（都已记进 CONTROL_TODO.md）：
//   1) 姿态积分现在是逐轴小角累加（样本内转角 ≪ 1，够用）；要严格就上四元数，
//      落在 eso.hpp 的 IMU_calculation_operator 里，对外接口不变。
//   2) 出仓初速度的量值由人给（发射段超量程）：imu_seed_initial_velocity() 的入参
//      要等实机弹道数据；x 正方向初速只是最省事的默认猜测。
//   3) IMU_ACC_RANGE / IMU_GAP_US 按实机传感器与丢包统计改。
//   4) 相机系→机体系的转换矩阵、零偏、温漂表都要实机标定。
// ============================================================================
