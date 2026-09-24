// ============================================================================
// IMU SPMC 伪代码：单生产者（IMU 中断）→ 多消费者（飞控环 / 视觉）
//
//   latest  —— "现在是多少"：seqlock，读几个时钟周期，零延迟
//   history —— "刚才那段转了多少"：环形缓冲 + 累计角位移曲线，时间戳可查
//
// 全模块无锁：ISR 不能阻塞，也不能让 50Hz 的日志任务拿锁去堵姿态环。
// ============================================================================

#include <stdbool.h>
#include <stdint.h>

#define IMU_HISTORY_CAPACITY 512u    // 2 的幂；1kHz 下 0.5s，entry 48B ≈ 24KB
#define IMU_HISTORY_MASK (IMU_HISTORY_CAPACITY - 1u)
#define IMU_SEQLOCK_RETRY 8u
#define IMU_GAP_US 20000u            // 相邻两帧间隔超过它 = 丢过数据
#define IMU_WINDOW_MAX 64u           // 单次区间查询最多跨这么多帧

typedef struct {
    float gyro[3];
    float acc[3];
    uint64_t timestamp;
} imu_sample_t;

typedef struct {
    imu_sample_t sample;
    float cum_angle[3];              // 开机以来的累计角位移，区间查询只需两次查表
    bool gap;                        // 与前一帧之间有缺口 → 跨它的区间查询必须降级
} imu_history_entry_t;

typedef struct __attribute__((aligned(64))) {
    volatile uint32_t sequence;
    imu_sample_t data;
} imu_latest_t;

typedef struct {
    // head 单独占一条 cache line：生产者每帧写它，消费者只读它
    volatile uint32_t head __attribute__((aligned(64)));
    imu_history_entry_t entries[IMU_HISTORY_CAPACITY];
} imu_history_t;

static imu_latest_t latest;
static imu_history_t history;

static inline const imu_history_entry_t *at(uint32_t i) {
    return &history.entries[i & IMU_HISTORY_MASK];
}

// ---------------------------------------------------------------------------
// 生产者：只在 IMU 中断里调用，全模块唯一的写者
// ---------------------------------------------------------------------------
void imu_provider_post_sample(const float raw_gyro[3], const float raw_acc[3],
                              uint64_t timestamp_us) {
    imu_sample_t s;
    calibrate(raw_gyro, raw_acc, &s);   // 零偏 → 温漂 → 相机系转机体系
    s.timestamp = timestamp_us;

    publish_latest(&s);
    publish_history(&s);
}

static void publish_latest(const imu_sample_t *s) {
    uint32_t seq = latest.sequence;

    latest.sequence = seq + 1;                          // 奇数 = 正在写
    __atomic_thread_fence(__ATOMIC_RELEASE);
    latest.data = *s;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    latest.sequence = seq + 2;                          // 偶数 = 写完
}

static void publish_history(const imu_sample_t *s) {
    uint32_t head = history.head;
    imu_history_entry_t *e = &history.entries[head & IMU_HISTORY_MASK];

    if (head == 0) {
        e->gap = false;
        e->cum_angle[0] = e->cum_angle[1] = e->cum_angle[2] = 0.0f;
    } else {
        const imu_history_entry_t *p = at(head - 1);
        e->gap = s->timestamp - p->sample.timestamp > IMU_GAP_US;
        float dt = (float)(s->timestamp - p->sample.timestamp) * 1e-6f;
        for (int a = 0; a < 3; ++a) {                   // 与前一帧做梯形积分
            e->cum_angle[a] = p->cum_angle[a] + 0.5f * (p->sample.gyro[a] + s->gyro[a]) * dt;
        }
    }

    e->sample = *s;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    history.head = head + 1;                            // head 动了这一格才可读
}

// ---------------------------------------------------------------------------
// 消费者 A：飞控姿态环 / 角速度环。非阻塞、无锁、有界重试
// ---------------------------------------------------------------------------
bool imu_get_latest(imu_sample_t *out_sample) {
    for (uint32_t retry = 0; retry < IMU_SEQLOCK_RETRY; ++retry) {
        uint32_t seq = latest.sequence;
        if (seq & 1u) continue;                         // 正好撞上写入：重试

        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        *out_sample = latest.data;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);

        if (latest.sequence == seq) return true;
    }
    return false;                                       // 调用方按"这拍没有数据"处理
}

// ---------------------------------------------------------------------------
// 消费者 B：视觉去自旋。区间查询，越界与时钟异常在模块内部消化
// ---------------------------------------------------------------------------
bool imu_get_delta_yaw_between(uint64_t start_time_us, uint64_t end_time_us,
                               float *out_delta_yaw) {
    uint32_t head = history.head;
    if (head == 0) return false;
    uint32_t count = head < IMU_HISTORY_CAPACITY ? head : IMU_HISTORY_CAPACITY;
    uint32_t first = head - count;

    if (start_time_us < at(first)->sample.timestamp) return false;   // 太老：已被覆盖
    uint64_t newest_us = at(head - 1)->sample.timestamp;
    if (end_time_us > newest_us) end_time_us = newest_us;            // 未来：截断
    if (end_time_us <= start_time_us) return false;

    // 从 start 的左邻帧拷到覆盖 end 的那帧；副本上算，避免读到写了一半的格子
    uint32_t i = head - 1;
    while (i > first && at(i)->sample.timestamp > start_time_us) --i;   // 停在 start 的左邻
    uint32_t j = i;
    while (j + 1 < head && at(j)->sample.timestamp < end_time_us) ++j;  // 停在覆盖 end 的那帧
    uint32_t n = j - i + 1;
    if (n > IMU_WINDOW_MAX) return false;                            // 间隔不合理

    imu_history_entry_t window[IMU_WINDOW_MAX];
    for (uint32_t k = 0; k < n; ++k) window[k] = *at(i + k);

    if (history.head - first > IMU_HISTORY_CAPACITY) return false;   // 拷贝期间被追上
    for (uint32_t k = 0; k < n; ++k) {
        if (window[k].gap) return false;                             // 这段里丢过数据
    }

    float yaw_start, yaw_end;
    if (!yaw_at(window, n, start_time_us, &yaw_start)) return false;
    if (!yaw_at(window, n, end_time_us, &yaw_end)) return false;

    *out_delta_yaw = yaw_end - yaw_start;
    return true;
}

// 时间戳落在两帧之间 → 在累计角位移曲线上插值（等于对速率做梯形积分）
static bool yaw_at(const imu_history_entry_t *w, uint32_t n, uint64_t t, float *out) {
    for (uint32_t k = 0; k + 1 < n; ++k) {
        if (t > w[k + 1].sample.timestamp) continue;
        if (t < w[k].sample.timestamp) break;

        uint64_t span = w[k + 1].sample.timestamp - w[k].sample.timestamp;
        if (span == 0) return false;
        float ratio = (float)(t - w[k].sample.timestamp) / (float)span;
        *out = w[k].cum_angle[2] + ratio * (w[k + 1].cum_angle[2] - w[k].cum_angle[2]);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// 消费者 C：控制环（roll 自稳）。同一个区间查询，输出换成三轴增量
//
// 控制环比视觉更在意"这个窗口的边界准不准"：它要的是**自己这两拍之间**转了多少，
// 而不是"当刻最新一帧"。用慢轨按自己的时间戳开窗，采样抖动、控制环比 IMU 快或慢
// 都不需要额外处理（真·时间基准）。角速度 = 增量 ÷ 窗长。
// imu_get_delta_yaw_between 就是它的 z 分量包装：视觉只关心去自旋，不想认识三轴。
// ---------------------------------------------------------------------------
bool imu_get_delta_angle_between(uint64_t start_time_us, uint64_t end_time_us,
                                 float out_delta[3]) {
    imu_history_entry_t window[IMU_WINDOW_MAX];
    uint32_t n;
    if (!snapshot(start_time_us, end_time_us, window, &n)) return false; // 同一套边界检查

    float a[3], b[3];
    if (!angle_at(window, n, start_time_us, a)) return false;
    if (!angle_at(window, n, end_time_us, b)) return false;

    for (int k = 0; k < 3; ++k) out_delta[k] = b[k] - a[k];
    return true;
}

// ============================================================================
// 边界一览（全部在模块内部消化，调用方只看 true/false）
//
//   start 早于缓冲区最老一帧   → false，视觉本次去自旋失败，按无 IMU 降级
//   end 晚于最新一帧（时钟不同步）→ 截断到最新帧再算
//   end <= start               → false
//   区间内 gap                 → false（累计角位移跨过一段未知时间）
//   区间跨帧数 > IMU_WINDOW_MAX → false（调用方时间戳算错了）
//   seqlock 连续冲突超过重试上限 → false
//
// 三处待定，等接线时定：
//   1) cum_angle[2] 当偏航角位移用：短曝光窗口（几 ms）内是标准近似；
//      要严格姿态角就得用四元数，那时这里换成四元数插值。
//   2) 标定系数（bias/温漂矩阵）由标定任务更新 → 换一份只读快照，原子换指针，
//      ISR 只读不写，标定任务自己保证新快照填完再发布。
//   3) history 深度按"最长回溯需求"定：现在 0.5s，只喂 200Hz 视觉的话 32 帧就够。
// ============================================================================
