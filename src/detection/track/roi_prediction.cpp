// ============================================================================
// RoiPredictor 实现
// ============================================================================

#include "detection/track/roi_prediction.hpp"

#include <cmath>

namespace dart::detection {

namespace {

inline float max_f(float a, float b) { return a > b ? a : b; }

inline uint32_t max_u32(uint32_t a, uint32_t b) { return a > b ? a : b; }
inline uint32_t min_u32(uint32_t a, uint32_t b) { return a < b ? a : b; }

} // namespace

RoiWindow RoiPredictor::whole(uint32_t frame_w, uint32_t frame_h) {
    RoiWindow w;
    w.x = 0;
    w.y = 0;
    w.w = frame_w;
    w.h = frame_h;
    w.full_frame = true;
    w.clamped = false;
    return w;
}

float RoiPredictor::grow_factor(uint32_t lost_frames, bool out_of_roi_last) const {
    // 线性放大（不是指数）：丢失初期窗口温和长大，避免一帧就把整幅都算进来。
    // 封顶 grow_max 之后就交给"整幅扫描"这条路（rescan_frac 判定）。
    float g = 1.0f + cfg_.grow_per_lost * static_cast<float>(lost_frames);
    if (g > cfg_.grow_max)
        g = cfg_.grow_max;
    if (g < 1.0f)
        g = 1.0f;
    if (out_of_roi_last)
        g *= (cfg_.out_of_roi_grow > 0.0f ? cfg_.out_of_roi_grow : 1.0f);
    return g;
}

RoiWindow RoiPredictor::for_tracking(float px, float py, float s, float sigma_x, float sigma_y,
                                     uint32_t lost_frames, bool out_of_roi_last, uint32_t frame_w,
                                     uint32_t frame_h) const {
    RoiWindow w;
    if (frame_w == 0 || frame_h == 0) {
        w.full_frame = true;
        return w;
    }

    const float g = grow_factor(lost_frames, out_of_roi_last);
    // W/H 用各自的预测标准差（x/y 方向的误差不一定相同），边长下限兜住小目标
    float side_w = max_f(cfg_.min_side, side_for(s, sigma_x) * g);
    float side_h = max_f(cfg_.min_side, side_for(s, sigma_y) * g);

    // 单维上限：把**这一维**夹到画幅（不因此判整幅 —— 窄画幅上竖长条窗口仍划算得多）
    const float cap_w = static_cast<float>(frame_w) * cfg_.max_side_frac;
    const float cap_h = static_cast<float>(frame_h) * cfg_.max_side_frac;
    if (side_w > cap_w)
        side_w = cap_w;
    if (side_h > cap_h)
        side_h = cap_h;

    // 整幅回退：**按面积**判（见 RoiConfig::rescan_area_frac 的注释 —— 按单维判会让
    // 近距离大目标永远退化全图，那正是工况 2 最主要的使用场景）。
    const float frame_area = static_cast<float>(frame_w) * static_cast<float>(frame_h);
    if (side_w * side_h >= frame_area * cfg_.rescan_area_frac)
        return whole(frame_w, frame_h);

    uint32_t ww = static_cast<uint32_t>(side_w + 0.5f);
    uint32_t wh = static_cast<uint32_t>(side_h + 0.5f);
    ww = max_u32(ww, 1);
    wh = max_u32(wh, 1);
    if (ww > frame_w) {
        ww = frame_w;
        w.clamped = true;
    }
    if (wh > frame_h) {
        wh = frame_h;
        w.clamped = true;
    }

    // 以预测质心为中心，再整体平移到画面内（不是缩小窗口，而是平移 ——
    // 贴边目标若把窗口截一半，目标就有一半落进画面外，等于白扫）
    int32_t x0 = static_cast<int32_t>(px + 0.5f) - static_cast<int32_t>(ww / 2);
    int32_t y0 = static_cast<int32_t>(py + 0.5f) - static_cast<int32_t>(wh / 2);
    if (x0 < 0) {
        x0 = 0;
        w.clamped = true;
    }
    if (y0 < 0) {
        y0 = 0;
        w.clamped = true;
    }
    if (x0 + static_cast<int32_t>(ww) > static_cast<int32_t>(frame_w)) {
        x0 = static_cast<int32_t>(frame_w) - static_cast<int32_t>(ww);
        w.clamped = true;
    }
    if (y0 + static_cast<int32_t>(wh) > static_cast<int32_t>(frame_h)) {
        y0 = static_cast<int32_t>(frame_h) - static_cast<int32_t>(wh);
        w.clamped = true;
    }
    if (x0 < 0)
        x0 = 0; // 窗口比画面还大时上面两步会把 x0 推成负数，这里兜住
    if (y0 < 0)
        y0 = 0;

    w.x = static_cast<uint32_t>(x0);
    w.y = static_cast<uint32_t>(y0);
    w.w = ww;
    w.h = wh;
    w.full_frame = false;
    return w;
}

} // namespace dart::detection
