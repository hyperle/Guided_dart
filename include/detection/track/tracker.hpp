#pragma once

// ============================================================================
// 目标跟踪状态机（启动 / 跟踪 / 丢失）+ 滤波器 + ROI 预测的**唯一归属地**。
//
// 它不认识像素、不认识扫描器、也不认识日志：只吃"上一帧扫到什么"，
// 吐"下一帧该扫哪里"和"本帧结论"。这样跟踪逻辑可以在宿主机上用合成序列跑
// （tests/host/detection_host_test.cpp），板端只验证"RVV / 内存 / 时序"这些
// 真的只能上板才知道的事。
//
// 每帧两拍（顺序不能反：**先预测、再决定扫哪里、最后才吃测量**）：
//
//   begin_frame(now, w, h)  推进时钟 → KF 预测 → 算出本帧扫描窗口（Plan）
//        ↓  调用方按 Plan 去扫（全图 RVV 或 ROI 连通域）
//   end_frame(report)       吃下观察 → 状态机迁移 → KF 更新 → 产出结果
//
// 三态迁移规则：
//   Startup  --滑窗确认成功-->            Tracking
//   Tracking --连续 lost_after 帧无测量--> Lost
//   Lost     --ROI 内重测到---------->     Tracking（软重捕，沿用/KF 重初始化）
//   Lost     --全图滑窗再次确认------->     Tracking（重捕）
//   Lost     --连续 hard_reset_after-->    Startup（硬复位，当新目标重新确认）
// ============================================================================

#include <cstdint>

#include "detection/track/blip_confirmer.hpp"
#include "detection/config.hpp"
#include "detection/track/kalman.hpp"
#include "detection/track/roi_prediction.hpp"
#include "detection/types.hpp"

namespace dart::detection {

class TargetTracker {
public:
    // noise 为空时使用内置的 LinearScaleNoiseModel（与 KF 同一份配置）
    TargetTracker(const DetectionConfig &cfg, const IScaleNoiseModel *noise = nullptr);

    void reset();

    // 本帧的扫描计划（begin_frame 的产出，调用方据此选扫描器）
    struct Plan {
        TrackState state = TrackState::Startup;
        RoiWindow  window{};
        bool       full_scan = true;
        bool       kf_ready = false;
        float      pred_x = 0.0f; // 预测质心（取证：与测量值对照能看出预测偏差）
        float      pred_y = 0.0f;
        float      pred_s = 0.0f;
    };

    Plan       begin_frame(uint64_t mono_us, uint32_t frame_w, uint32_t frame_h);
    TrackOutput end_frame(const ScanReport &report);

    TrackState              state() const { return state_; }
    uint32_t                lost_frames() const { return lost_frames_; }
    float                   dt_last() const { return dt_last_; }
    const TrackerCounters  &counters() const { return cnt_; }
    const ScaleAwareKalman &kf() const { return kf_; }
    const BlipConfirmer     &confirmer() const { return confirmer_; }
    const DetectionConfig  &config() const { return cfg_; }
    const RoiPredictor     &predictor() const { return roi_; }

    // 上一帧因平滑性被否掉的候选数（闪烁坏点计数，供状态行取证）
    uint32_t last_reject() const { return last_reject_; }

private:
    void        enter(TrackState s);
    void        note_miss();
    TrackOutput result_from_state(bool found, float cx, float cy, float radius, RoiState roi,
                                  float circ = -1.0f) const;

    DetectionConfig    cfg_;
    ScaleAwareKalman   kf_;
    BlipConfirmer       confirmer_;
    RoiPredictor       roi_;
    // 噪声模型由 KF 持有（noise 为空时 KF 用内置实现），这里不再存第二份引用 ——
    // 两份引用迟早会有一份忘了更新。要看模型名字走 kf().noise_name()。

    TrackState         state_ = TrackState::Startup;
    uint32_t           lost_frames_ = 0;
    uint64_t           last_us_ = 0;
    float              dt_last_ = 0.0f;
    uint32_t           frame_w_ = 0;
    uint32_t           frame_h_ = 0;

    // begin_frame 与 end_frame 之间的交接（这一帧"怎么扫"）
    Plan               plan_{};
    bool               prev_full_ = true; // 上一帧是否全图扫描（模式切换时重置滑窗）
    bool               out_of_roi_last_ = false;
    uint32_t           last_reject_ = 0;

    TrackerCounters    cnt_{};
};

} // namespace dart::detection
