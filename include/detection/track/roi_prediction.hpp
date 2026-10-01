#pragma once

// ============================================================================
// 动态 ROI：跟踪阶段每帧"先预测、再决定扫哪里"。
//
//   规格公式：W_roi = kp · ŝ + B_margin
//   本实现  ：W_roi = (kp · ŝ + B_margin + k_σ·σ_pred) × 放大系数
//
// 为什么多一项 k_σ·σ_pred：只用 ŝ 定窗口时，窗口大小与**预测误差**无关。
// 目标刚加速、或刚从丢失里恢复（协方差很大）的那一帧，窗口相对预测误差太小 →
// 目标出框 → 这一帧测量缺失 → 反而把状态机推向"丢失"，形成恶性循环。
// k_σ 置 0 就严格退化成规格公式（宿主机的对照测试会验证这一点）。
//
// 窗口永远被夹进画面：贴边目标会让窗口向画面内侧平移（clamped=true），
// 于是目标必然落在窗口内 —— 但可能贴到窗口边，那时由测量器给出
// TargetOutOfRoi + 低质量，本类下一帧再按 out_of_roi_grow 放大一次。
// ============================================================================

#include <cstdint>

#include "detection/config/roi.hpp"
#include "detection/domain/geometry.hpp"

namespace dart::detection {

class RoiPredictor {
public:
    explicit RoiPredictor(const RoiConfig &cfg) : cfg_(cfg) {}

    // 整幅窗口（启动阶段与"放大到尽头"时用）
    static RoiWindow whole(uint32_t frame_w, uint32_t frame_h);

    // 跟踪/丢失态：按预测状态算出本帧要扫的窗口
    //   px, py   预测质心（只用于定窗口中心，不参与滤波）
    //   s        预测等效半径 ŝ
    //   sigma_*  预测标准差（用于 k_σ 项）
    //   lost_frames 连续丢失帧数（放大系数，0 = 跟踪态）
    //   out_of_roi_last 上一帧是否"目标超框"（再放大一档）
    RoiWindow for_tracking(float px, float py, float s, float sigma_x, float sigma_y,
                           uint32_t lost_frames, bool out_of_roi_last, uint32_t frame_w,
                           uint32_t frame_h) const;

    // 只看规格公式那一部分（自检/取证：验证 W = kp·s + margin）
    float side_for(float s, float sigma) const {
        const float base = cfg_.kp * (s > 0.0f ? s : 0.0f) + cfg_.margin;
        const float sigma_term = cfg_.k_sigma * (sigma > 0.0f ? sigma : 0.0f);
        return base + sigma_term;
    }

    // 当前放大系数（丢失帧数 + 超框提示）
    float grow_factor(uint32_t lost_frames, bool out_of_roi_last) const;

    const RoiConfig &config() const { return cfg_; }

private:
    RoiConfig cfg_;
};

} // namespace dart::detection
