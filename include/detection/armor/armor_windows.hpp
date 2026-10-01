#pragma once

// ============================================================================
// 装甲板的**窗口规划**：本帧到底扫哪 1~2 个矩形。
//
// 窗口是这一路唯一的"效率旋钮"，也是最容易出隐性 bug 的地方：窗口开小了目标
// 被切掉（面积只是下界 → 长度量短 → 判据失效），开大了 ROI 退化成整幅
// （板端就是白烧 RVV）。所以这一层单独成文件，且只做几何、不碰像素。
//
// 三种窗口：
//   · led_window    —— 绿灯正上方那一整块（首次 / 重捕）
//   · bar_window    —— 每条灯条各一个自己的小窗（跟踪中且绿灯够大）
//   · union_window  —— 一个合窗罩住上一对灯条（远距离 / 单条窗建不起来）
//
// 本层不持有检测器的关联状态，只看调用方借进来的 PairView（上一对灯条 + r_last）。
// ============================================================================

#include <cstdint>

#include "core/frame.hpp"
#include "detection/armor/armor_geometry.hpp"
#include "detection/config/armor.hpp"
#include "detection/domain/geometry.hpp"
#include "detection/domain/tracking.hpp"

namespace dart::detection::armor {

// 窗口规划要看的"上一对灯条"。借用，不持有 —— 持有会让检测器的复制语义变味。
struct PairView {
    const Bar *a = nullptr;       // have_pair 为真时有效
    const Bar *b = nullptr;       // have_pair 为真时有效
    bool       have_pair = false;
    float      r_last = 0.0f; // 上次采到那一对时的绿灯半径（尺度增长补偿用）
};

// ---------------------------------------------------------------------------
// 把浮点窗口夹进画面并取整；夹完小于 min_side 就返回 false（窗口没意义 → 本帧不报）。
// 闭区间外的开区间表示：窗口覆盖 x .. x+w-1。
// ---------------------------------------------------------------------------
bool clamp_window(float x, float y, float w, float h, uint32_t fw, uint32_t fh, float min_side,
                  RoiWindow *out);

// ---------------------------------------------------------------------------
// 绿灯正上方那块：尺寸随灯尺，下沿贴绿灯上沿。
// ---------------------------------------------------------------------------
bool led_window(const ArmorConfig &cfg, const GrayFrame &f, const TrackOutput &led,
                RoiWindow *win);

// ---------------------------------------------------------------------------
// 窗口外扩 margin，口径 = **按单条灯条长度**给并封顶，最后再加 pad。
// 原来按"整个对的尺寸 × 0.8"给：近距离一个窗就十几万像素（128880 vs 35280）。
// ---------------------------------------------------------------------------
int32_t bar_margin(const ArmorConfig &cfg, float bar_len_px, int32_t pad);

// ---------------------------------------------------------------------------
// 尺度增长补偿：工况是**逐渐接近**，30fps 下目标每帧能长 10~35%。
// 窗口是按"上次采到的那条灯条"开的，不补偿的话这一帧目标就顶出窗外 → 被切。
// 实测（每帧 ×1.35 的接近序列）：量到的灯条长度只有真值的 0.58；补偿后 0.86。
// ---------------------------------------------------------------------------
int32_t grow_pad(const PairView &pair, const TrackOutput &led, float bar_len_px);

// ---------------------------------------------------------------------------
// 锚的"够得着"范围：[灯心 ± near_k·r] 的外接方框。
// 单条窗**不再**被"绿灯上方整块"裁剪（那正是某些角度包不住装甲板的原因），
// 安全性改由这条径向范围提供：窗口可以跟着灯条走，但不许跑出这个范围。
// ---------------------------------------------------------------------------
bool reach_box(const ArmorConfig &cfg, const TrackOutput &led, const GrayFrame &f, RoiWindow *out);

// ---------------------------------------------------------------------------
// 一条灯条自己的小窗：只需盖住它自己 → 板转多少度都不影响这个窗口够不够。
// 与"够得着"范围取交；取交后小于 8px 返回 false。
// ---------------------------------------------------------------------------
bool bar_window(const ArmorConfig &cfg, const PairView &pair, const Bar &bar, const TrackOutput &led,
                const RoiWindow &reach, const GrayFrame &f, RoiWindow *out);

// ---------------------------------------------------------------------------
// 合窗（远距离 / 首次 / 单条窗没建起来）：一个窗罩住上一对灯条，与整块取交。
// 小窗被挤没了就退回整块 —— 宁可多扫，不可漏掉。
// ---------------------------------------------------------------------------
RoiWindow union_window(const ArmorConfig &cfg, const PairView &pair, const RoiWindow &full,
                       const TrackOutput &led);

// ---------------------------------------------------------------------------
// 本帧扫哪 1~2 个窗（out 至少 2 个元素）。返回窗个数。
// ---------------------------------------------------------------------------
uint32_t search_windows(const ArmorConfig &cfg, const PairView &pair, const RoiWindow &full,
                        const TrackOutput &led, const GrayFrame &f, RoiWindow *out);

} // namespace dart::detection::armor
