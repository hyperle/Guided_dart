// ============================================================================
// TargetTracker 实现：三态迁移 + KF 更新 + ROI 预测。
// 所有"为什么这么判"的理由都写在头文件与 config.hpp 里，这里只留实现要点。
// ============================================================================

#include "detection/tracker.hpp"

#include <cmath>

namespace dart::detection {

TargetTracker::TargetTracker(const DetectionConfig &cfg, const IScaleNoiseModel *noise)
    : cfg_(cfg), kf_(cfg.kf, noise), armer_(cfg.arm), roi_(cfg.roi) {}

void TargetTracker::reset() {
    state_ = TrackState::Startup;
    lost_frames_ = 0;
    last_us_ = 0;
    dt_last_ = 0.0f;
    out_of_roi_last_ = false;
    armer_last_reject_ = 0;
    prev_full_ = true;
    plan_ = Plan{};
    kf_.reset();
    armer_.reset();
}

void TargetTracker::enter(TrackState s) {
    if (state_ == s)
        return;
    if (s == TrackState::Tracking) {
        if (state_ == TrackState::Startup)
            ++cnt_.arm_gained;
        else
            ++cnt_.to_tracking;
    } else if (s == TrackState::Lost) {
        ++cnt_.to_lost;
    } else { // → Startup
        ++cnt_.to_startup;
    }
    state_ = s;
}

TargetTracker::Plan TargetTracker::begin_frame(uint64_t mono_us, uint32_t frame_w, uint32_t frame_h) {
    // ---- 时钟：dt 只在这里算一次（掉帧/暂停/时钟跳变都夹在 KfConfig.dt_min..dt_max）----
    dt_last_ = 0.0f;
    if (last_us_ != 0 && mono_us > last_us_) {
        float dt = static_cast<float>(static_cast<double>(mono_us - last_us_) * 1e-6);
        if (dt > cfg_.kf.dt_max)
            dt = cfg_.kf.dt_max;
        dt_last_ = dt;
    }
    last_us_ = mono_us;
    frame_w_ = frame_w;
    frame_h_ = frame_h;

    plan_ = Plan{};
    plan_.state = state_;

    if (!cfg_.enable_tracking) {
        // A/B 模式（--no-track）：永远走"全图 + 滑窗确认"，用来判断 ROI 门控/滤波
        // 是不是某次回归的元凶。这里**不能**去动滑窗（例如"没填满就 reset"会把滑窗
        // 永远清零，于是永远确认不了目标 —— 主机测试就是这么把这个坑抓出来的）。
        plan_.window = RoiPredictor::whole(frame_w, frame_h);
        plan_.full_scan = true;
        prev_full_ = true;
        return plan_;
    }

    // ---- 本帧"怎么扫"：三条路 ----
    //   ① 启动态：全图（还没有目标）
    //   ② 跟踪态：ROI（KF 预测）
    //   ③ 丢失态：先按放大后的 ROI，连续丢失超过 rescan_after 帧后回到全图重扫
    bool full = (state_ == TrackState::Startup) || !kf_.initialized() ||
                (state_ == TrackState::Lost && lost_frames_ >= cfg_.tracker.rescan_after);

    if (full) {
        plan_.window = RoiPredictor::whole(frame_w, frame_h);
        plan_.full_scan = true;
        plan_.kf_ready = kf_.initialized();
        if (kf_.initialized()) {
            plan_.pred_x = kf_.x();
            plan_.pred_y = kf_.y();
            plan_.pred_s = kf_.s();
        }
    } else {
        kf_.predict(dt_last_); // 只有"这一帧要看测量"时才推进模型（全图重扫帧不推进）
        plan_.kf_ready = true;
        plan_.pred_x = kf_.x();
        plan_.pred_y = kf_.y();
        plan_.pred_s = kf_.s();
        plan_.window = roi_.for_tracking(kf_.x(), kf_.y(), kf_.s(), kf_.sigma_x(), kf_.sigma_y(),
                                        state_ == TrackState::Lost ? lost_frames_ : 0,
                                        out_of_roi_last_, frame_w, frame_h);
        plan_.full_scan = plan_.window.full_frame;
    }

    // 扫描模式切换（ROI ↔ 全图）时把滑窗清掉：滑窗里存的候选是**上一次全图扫描**那一轮
    // 的位置，跟现在的场景早就对不上了；不清就会出现"用旧位置确认了新目标"的假确认。
    // 清掉的代价是重新确认要多花 window 帧，这正是重捕该有的代价。
    if (plan_.full_scan && !prev_full_)
        armer_.reset();
    prev_full_ = plan_.full_scan;

    if (plan_.full_scan)
        ++cnt_.full_scans;
    else
        ++cnt_.roi_scans;
    return plan_;
}

void TargetTracker::note_miss() {
    ++lost_frames_;
    if (state_ == TrackState::Tracking) {
        if (lost_frames_ >= cfg_.tracker.lost_after)
            enter(TrackState::Lost);
        return;
    }
    if (state_ == TrackState::Lost && lost_frames_ >= cfg_.tracker.hard_reset_after) {
        // 连续丢失这么久还没找回来：清空滤波器与滑窗，当成"一个新目标"重新确认。
        // 保留旧状态只会让预测越来越离谱，还会把真正的新位置当离群点拒掉。
        kf_.reset();
        armer_.reset();
        enter(TrackState::Startup);
    }
}

TrackOutput TargetTracker::result_from_state(bool found, float cx, float cy, float radius,
                                             RoiState roi_state, float circ) const {
    TrackOutput o;
    o.state = state_;
    o.window = plan_.window;
    o.found = found;
    if (found) {
        o.cx = cx;
        o.cy = cy;
        o.radius = radius;
        o.circularity = circ;
        o.roi = roi_state;
    } else {
        o.cx = o.cy = -1.0f;
        o.radius = -1.0f;
        o.roi = RoiState::NoTargetInRoi;
    }
    return o;
}

TrackOutput TargetTracker::end_frame(const ScanReport &report) {
    out_of_roi_last_ = false;

    // ================= 全图扫描帧：走启动确认逻辑（启动态与丢失重扫共用）=================
    if (report.full_scan()) {
        const StartupArmer::Confirmation a = armer_.push(last_us_, report.cands, report.count);
        ++cnt_.arm_frames;
        armer_last_reject_ = armer_.last_reject();
        cnt_.arm_reject += armer_last_reject_;
        cnt_.arm_border_skip += armer_.last_border_skip();

        if (!a.ok) {
            // 全图扫描没确认出目标，也算"这一帧没测到"。**这一句不能省**：
            // 丢失态超过 rescan_after 之后就一直在全图重扫，如果这里不记 miss，
            // lost_frames 会停住不动 —— 硬复位永远到不了，"连续丢失 N 帧"的日志
            // 也会一直显示同一个数（主机测试抓到的就是这个）。
            note_miss();
            return result_from_state(false, 0.0f, 0.0f, 0.0f, RoiState::NoTargetInRoi);
        }

        if (!cfg_.enable_tracking) {
            // A/B 模式：不给滤波，直接用确认到的候选（每帧重新确认）
            return result_from_state(true, a.cand.cx, a.cand.cy, a.cand.radius, RoiState::Hit,
                                 a.cand.circularity);
        }

        // 确认成功 → 用整条轨迹初始化滤波器（最小二乘估初速），并外推到"本帧时刻"。
        // 三种来路要分开计数（板端 run1 就是被混在一起骗过的：跟踪态每帧都走全图确认，
        // 却记成了 4271 次"重捕"）：
        //   启动 → 跟踪：首次确认      跟踪态 → 又全图确认：ROI 退化成了整幅
        //   丢失 → 跟踪：真正的重捕
        const TrackState prev = state_;
        kf_.init_from_track_at(a.chain, a.times_us, a.n, last_us_);
        lost_frames_ = 0;
        if (prev == TrackState::Lost)
            ++cnt_.reacquire_soft;
        else if (prev == TrackState::Tracking)
            ++cnt_.full_confirm;
        enter(TrackState::Tracking);
        return result_from_state(true, kf_.x(), kf_.y(), kf_.s(), RoiState::Hit, a.cand.circularity);
    }

    // ================= ROI 扫描帧：测量 → 滤波 =================
    const TargetMeasurement &m = report.measurement;

    if (!m.valid) {
        note_miss();
        return result_from_state(false, 0.0f, 0.0f, 0.0f, m.roi, m.circularity);
    }

    bool updated = false;
    if (state_ == TrackState::Lost && lost_frames_ > cfg_.kf.reinit_after_lost) {
        // 丢了很久又突然测到：预测态早就飘了，直接按这次测量重新起滤波
        kf_.init_from_measurement(m.cx, m.cy, m.radius);
        ++cnt_.reacquire_hard;
        updated = true;
    } else {
        const bool was_lost = (state_ == TrackState::Lost);
        updated = kf_.update(m);
        if (!updated) {
            ++cnt_.kf_reject; // 被马氏门限拒收：当成"这一帧没测到"
        } else if (was_lost) {
            ++cnt_.reacquire_soft; // 丢失期间预测态还够用：直接接着滤波就回来了
        }
    }

    if (m.roi == RoiState::TargetOutOfRoi) {
        out_of_roi_last_ = true; // 下一帧窗口放大（begin_frame 会读它）
        ++cnt_.out_of_roi;
    }

    if (!updated) {
        note_miss();
        return result_from_state(false, 0.0f, 0.0f, 0.0f, m.roi, m.circularity);
    }

    lost_frames_ = 0;
    if (state_ != TrackState::Tracking)
        enter(TrackState::Tracking);
    return result_from_state(true, kf_.x(), kf_.y(), kf_.s(), m.roi, m.circularity);
}

} // namespace dart::detection
