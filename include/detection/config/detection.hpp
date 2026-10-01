#pragma once

#include "detection/config/armor.hpp"
#include "detection/config/confirmer.hpp"
#include "detection/config/kalman.hpp"
#include "detection/config/measure.hpp"
#include "detection/config/roi.hpp"
#include "detection/config/scanner.hpp"
#include "detection/config/tracker.hpp"

namespace dart::detection {

struct DetectionConfig {
    ScannerConfig   scan;
    MeasureConfig   measure;
    ConfirmerConfig confirm;
    KfConfig        kf;
    RoiConfig       roi;
    TrackerConfig   tracker;
    ArmorConfig     armor;
    bool            enable_tracking = true;

    // 保持旧 CLI 的行为，同时把所有边界约束集中在配置聚合层。
    void sanitize() {
        if (confirm.window < 1)
            confirm.window = 1;
        if (confirm.window > kBlipWindowMax)
            confirm.window = kBlipWindowMax;
        if (confirm.min_hits < 1)
            confirm.min_hits = 1;
        if (confirm.min_hits > confirm.window)
            confirm.min_hits = confirm.window;
        if (scan.top_k < 1)
            scan.top_k = 1;
        if (scan.top_k > 64)
            scan.top_k = 64;
        if (scan.tile < 1)
            scan.tile = 1;
        if (tracker.lost_after < 1)
            tracker.lost_after = 1;
        if (roi.kp < 0.0f)
            roi.kp = 0.0f;
        if (roi.k_sigma < 0.0f)
            roi.k_sigma = 0.0f;
        if (armor.len_min_k < 0.0f)
            armor.len_min_k = 0.0f;
        if (armor.len_max_k < armor.len_min_k)
            armor.len_max_k = armor.len_min_k;
        if (armor.aspect_min < 1.0f)
            armor.aspect_min = 1.0f;
        if (armor.diag_min_sin < 0.0f)
            armor.diag_min_sin = 0.0f;
        if (armor.diag_min_sin > 1.0f)
            armor.diag_min_sin = 1.0f;
        if (armor.hold > 32)
            armor.hold = 32;
        if (armor.roi_min_side < 4.0f)
            armor.roi_min_side = 4.0f;
        if (armor.bar_margin_min < 1.0f)
            armor.bar_margin_min = 1.0f;
        if (armor.bar_margin_max < armor.bar_margin_min)
            armor.bar_margin_max = armor.bar_margin_min;
        if (armor.near_k < 0.0f)
            armor.near_k = 0.0f;
    }
};

} // namespace dart::detection
