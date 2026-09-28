#pragma once

// ============================================================================
// 绿灯（工况 1）的**启动确认器**。
//
//   吃：全图扫描出来的候选序列（每帧一批 LightCandidate）
//   做：3 帧滑窗反向关联 + 三道门（命中数 / 位移平滑性 / 物理性）
//   出：确认成功则把整条轨迹交给卡尔曼初始化
//
// 它**不认识**像素、不认识窗口、不认识装甲板 —— 只认候选表。
// 使用者：TargetTracker（tracker.hpp），它每帧全图扫描时喂候选。
//
// 与装甲板那一路的关系：无。装甲板的识别在 detection/armor/ 下
// （armor_detector.hpp 等），两者除了都属于"识别层第二路输出"之外没有任何耦合。
// ============================================================================

#include <cstddef>
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
