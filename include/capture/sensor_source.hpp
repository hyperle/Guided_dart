#pragma once

#include <cstdint>

#include "core/config.hpp"

#include "mpi_sensor_api.h"
#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_vicap_api.h"

namespace dart {

// 传感器取帧：VICAP / VB / sensor 探测的全部生命周期都封在这里，
// 上层只看到 capture()。单一职责：把"一帧原始 Y 平面"交出去，并在帧用完时归还。
class SensorSource {
public:
    // 一帧原始 YUV420SP。只暴露 Y 平面（二值化只用 Y），
    // 析构即 munmap + 归还 VICAP —— 帧的所有权边界就是它的作用域。
    class RawFrame {
    public:
        RawFrame() = default;
        RawFrame(const RawFrame &) = delete;
        RawFrame &operator=(const RawFrame &) = delete;
        RawFrame(RawFrame &&o) noexcept;
        ~RawFrame();

        const uint8_t *y = nullptr;
        uint32_t       width = 0;
        uint32_t       height = 0;
        uint32_t       stride = 0; // Y 平面行跨距（VICAP 有对齐填充，通常 > width）
        k_mod_id       mod_id = static_cast<k_mod_id>(0); // 源帧的 mod_id，供池帧照抄
        uint64_t       pts = 0; // 源 VICAP 帧的 pts（取证：真实帧间隔）

    private:
        friend class SensorSource;
        SensorSource      *owner_ = nullptr;
        k_video_frame_info info_{};
        void              *map_ = nullptr;
        uint32_t           map_size_ = 0;
    };

    explicit SensorSource(const Config &cfg);
    ~SensorSource();

    void start();
    void stop();

    RawFrame capture();

    // 实际生效的传感器模式（回读值；适配表匹配不到时会落到别的模式，绝不信请求值）
    uint32_t width() const { return acq_w_; }
    uint32_t height() const { return acq_h_; }
    uint32_t fps() const { return acq_fps_; }

private:
    void apply_exposure();

    const Config       cfg_;
    k_vicap_sensor_info sensor_{};
    k_s32              sensor_fd_ = -1;
    uint32_t           acq_w_ = 0;
    uint32_t           acq_h_ = 0;
    uint32_t           acq_fps_ = 0;
    uint32_t           dump_fails_ = 0;  // 连续取帧失败次数（用于限频打印错误码）
    uint32_t           frame_count_ = 0; // 已取到的帧数（第 1 帧打一条日志）
    bool               started_ = false;
};

} // namespace dart
