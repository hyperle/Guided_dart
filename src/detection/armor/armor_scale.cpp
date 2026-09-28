// ============================================================================
// armor_scale.cpp —— armor_scale.hpp 的实现（尺度状态 + 灯条门限换算）
// ============================================================================

#include "detection/armor/armor_scale.hpp"

namespace dart::detection::armor {

void scale_reset(ScaleState *st) {
    st->s = 0;
    st->ds = 0.0f;
}

void scale_update(ScaleState *st, int32_t s) {
    if (st->s > 0) {
        const float d = static_cast<float>(s - st->s);
        st->ds = 0.5f * st->ds + 0.5f * d;
    }
    st->s = s;
}

void scale_band(const ArmorConfig &cfg, const ScaleState &st, int32_t *s_lo, int32_t *s_hi) {
    if (st.s <= 0) {
        *s_lo = 0;
        *s_hi = 0;
        return;
    }
    float v = cfg.scale_v_k * st.ds;
    if (v > cfg.scale_v_max)
        v = cfg.scale_v_max;
    else if (v < -cfg.scale_v_max)
        v = -cfg.scale_v_max;
    const int32_t sp = st.s + static_cast<int32_t>(v);
    *s_lo = sp < st.s ? (sp > 0 ? sp : 0) : st.s;
    *s_hi = sp > st.s ? sp : st.s;
}

void scale_gate(const ArmorConfig &cfg, int32_t s_lo, int32_t s_hi, int32_t *len_min,
                int32_t *len_max, int32_t *px_min) {
    int32_t lo = static_cast<int32_t>(cfg.len_min_k * static_cast<float>(s_lo));
    int32_t hi = static_cast<int32_t>(cfg.len_max_k * static_cast<float>(s_hi));
    if (lo < static_cast<int32_t>(cfg.len_floor))
        lo = static_cast<int32_t>(cfg.len_floor);
    if (hi < lo)
        hi = lo;
    int32_t px = lo / 2;
    if (px < static_cast<int32_t>(cfg.px_floor))
        px = static_cast<int32_t>(cfg.px_floor);
    *len_min = lo;
    *len_max = hi;
    *px_min = px;
}

int32_t bar_px_min(const ArmorConfig &cfg, int32_t s) {
    int32_t a, b, px;
    scale_gate(cfg, s, s, &a, &b, &px);
    return px;
}

} // namespace dart::detection::armor
