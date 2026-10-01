#pragma once

// ============================================================================
// 灯尺（s = 2r）的状态与由它导出的灯条门限。
//
// 工况是"目标逐渐接近"：灯尺逐帧变大，门限必须跟着灯尺走 —— 远距离真灯条只有
// 2~6px，塞不进一组固定门限。这里只做"尺度 → 门限"的换算，不碰图像、不认识块。
//
// 拆成独立单元的理由：门限公式是识别正确性最敏感的地方（长度地板、像素地板、
// 尺度包络），单独放就能在主机上不开检测器直接单测。
// ============================================================================

#include <cstdint>

#include "detection/config/armor.hpp"

namespace dart::detection::armor {

// 尺度状态：与检测器的关联状态（上一对灯条/年龄）无关，因此单独持有。
struct ScaleState {
    int32_t s = 0;    // 上一次的灯尺
    float   ds = 0.0f; // 灯尺变化率（一阶低通）
};

void scale_reset(ScaleState *st);

// 线性一步外推：ds = 0.5·ds + 0.5·(s − s_prev)。
// 与仓库"目标均匀变大"同口径；首次（s_prev 未建立）只记值不外推。
void scale_update(ScaleState *st, int32_t s);

// 尺度包络 [s_lo, s_hi]：远处目标长得快，单点尺度会把这一帧的目标判掉，
// 所以下界取小的、上界取大的。
//
// ⚠ 现状：**本仓库没有任何调用点**（gate 的两处调用都传 s_lo = s_hi = s）。
//   连带的 ScaleState::ds、ArmorConfig::scale_v_k / scale_v_max 目前也是惰性的。
//   tools/armor_detect_k230.py 里保留了它的对照测试，所以这里先按原样迁过来；
//   是否接线或删除留待后续单独处理（阶段 1 只做搬迁，不改行为）。
void scale_band(const ArmorConfig &cfg, const ScaleState &st, int32_t *s_lo, int32_t *s_hi);

// 门限：灯条长度 = 灯尺的倍数 + 硬地板；像素下限由"最短长度"推出（不另设系数）。
// s_lo/s_hi 是尺度包络（min/max(s, s_pred)）：远处目标长得快，单点尺度会把
// 这一帧的目标判掉，所以下界取小的、上界取大的。
void scale_gate(const ArmorConfig &cfg, int32_t s_lo, int32_t s_hi, int32_t *len_min,
                int32_t *len_max, int32_t *px_min);

// 像素下限（等价于 scale_gate(s, s, ...) 只要最后一个出参）
int32_t bar_px_min(const ArmorConfig &cfg, int32_t s);

} // namespace dart::detection::armor
