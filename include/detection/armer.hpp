#pragma once

// ============================================================================
// 启动阶段确认器（工况 1 的第二步）：**3 帧滑窗 + 位移矢量平滑性**。
//
// 为什么必须有它：启动阶段目标只有几个像素，二值图上"闪烁的传感器坏点"
// 和"真目标"单帧看起来一模一样（都是 1~3 个白像素）。区分它们的不是单帧特征，
// 而是**时间上的一致性**：
//
//   真目标：连续 3 帧都在，位移矢量连续（帧间加速度有界），尺度变化物理合理
//   坏点  ：随机位置冒一下 —— 要么凑不出 3 帧命中，要么凑出来的"轨迹"在帧间乱跳
//
// 于是确认条件就是三道门（任一不过就丢，见 ArmConfig）：
//   ① 命中数：滑窗内 ≥ min_hits 帧被关联上（默认 3/3）
//   ② 平滑性：相邻两帧位移之差的模 ≤ max_accel_px（**主判据**）
//   ③ 物理性：累计位移 ≤ max_travel_px，尺度变化率 ≤ max_radius_rate
//
// 确认成功时把整条轨迹（时间升序）交给卡尔曼初始化：3 帧数据用最小二乘
// 估初速，比"两点差分"稳得多 —— 这就是"确认"顺带赚到的收益。
//
// 关联方式：从最新一帧往回串。位置门限用**匀速外推**（已知两点时），
// 只有一点时按"原地不动"外推（gate_px）。所以确认期间的目标可以有速度，
// 但加速度太大会被 ② 拦住。
// ============================================================================

#include <cstdint>

#include "detection/config.hpp"
#include "detection/types.hpp"

namespace dart::detection {

// 每帧最多缓存多少个候选参与关联（扫描器 Top-K 上限 64，但滑窗里没必要全留）
constexpr uint32_t kArmMaxPerFrame = 16;

class StartupArmer {
public:
    struct Confirmation {
        bool           ok = false;
        LightCandidate cand{};   // 确认后的目标（位置/尺度取最新一帧，continuity 已填）
        uint32_t       hits = 0; // 滑窗内关联上的帧数
        float          travel_px = 0.0f;    // 首末帧直线距离
        float          max_accel_px = 0.0f; // 帧间位移差的最大模（平滑性指标）
        float          radius_rate = 0.0f;  // 尺度变化率（px/s）

        // 整条轨迹（时间升序：旧 → 新），交给 ScaleAwareKalman::init_from_track
        LightCandidate chain[kArmMaxWindow]{};
        uint64_t       times_us[kArmMaxWindow]{};
        size_t         n = 0;
    };

    explicit StartupArmer(const ArmConfig &cfg);

    void reset();

    // 推入本帧全图候选（按 score 降序最好，内部会再排一次保证确定性），
    // 返回本帧的确认结果。窗口没填满时 ok 恒为 false。
    Confirmation push(uint64_t mono_us, const LightCandidate *cands, size_t n);

    uint32_t window() const { return cfg_.window; }

    // 本帧被否掉的候选个数（≈ 随机闪烁坏点计数，写进状态行做取证）
    uint32_t last_reject() const { return last_reject_; }
    // 本帧被跳过的**贴画面边**候选个数（与"拒闪"分开统计：那是被边界切掉的块，
    // 不是闪烁坏点。两者混在一起会让状态行误导调参 —— round12 的教训）
    uint32_t last_border_skip() const { return last_border_skip_; }
    // 本帧是否做过确认尝试（窗口填满后才是 true）
    bool     ready() const { return filled_ >= cfg_.window; }

private:
    struct Slot {
        uint64_t       mono_us = 0;
        uint32_t       n = 0;
        LightCandidate cands[kArmMaxPerFrame]{};
    };

    const Slot &at_lag(uint32_t lag) const; // lag=0 最新

    ArmConfig cfg_;
    Slot      ring_[kArmMaxWindow];
    uint32_t  head_ = 0;   // 最新一帧所在下标
    uint32_t  filled_ = 0; // 已填入的帧数（<= window）
    uint32_t  last_reject_ = 0;
    uint32_t  last_border_skip_ = 0;
};

} // namespace dart::detection
