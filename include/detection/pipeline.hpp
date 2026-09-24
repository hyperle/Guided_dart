#pragma once

// ============================================================================
// 识别层门面：把"看哪里、怎么看、看到之后怎么跟"编成一个 dart::IDetector。
//
// 依赖注入（构造函数 Deps）：
//   IPointScanner           全图扫描器（默认 LightScanner：RVV 瓦片粗筛）
//   IRoiMeasurer            ROI 测量器（默认 RoiBlobMeasurer：游程连通域）
//   const IScaleNoiseModel  尺度噪声 R_scale(s)（默认 LinearScaleNoiseModel）
//   std::function now_us    时钟（默认 CLOCK_MONOTONIC；测试注入假时钟即可完全确定性地跑）
//
// 为什么这样切：上层（main.cpp）只认识 IDetector，不关心有几个阶段；
// 而这些"策略"在宿主机测试里要能换成假件（无 RVV、无相机、时间可控），
// 这样"启动确认 / 状态机 / 滤波"这些逻辑就能在 x86 上先咬死，
// 上板只剩"RVV 与内存时序"这类真正只能上板才知道的事。
//
// 时间：本层自己读单调钟（一次 detect 调用 = 一帧）。不用 Frame::mono_ms 是因为
// IDetector 的入参只有 GrayFrame（纯像素视图），而 dt 必须来自"帧到达本层"的时刻。
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "core/frame.hpp"
#include "detection/blob_measure.hpp"
#include "detection/config.hpp"
#include "detection/light.hpp"
#include "detection/tracker.hpp"
#include "detection/types.hpp"
#include "vision/detector.hpp"

namespace dart::detection {

// 板端 1Hz 状态行要的数字（检测层不自己打日志，谁用谁打）
struct DetectionStats {
    uint64_t       frames = 0;    // 本层处理的帧数
    uint64_t       hits = 0;      // 有目标的帧数
    uint64_t       misses = 0;    // 无目标的帧数
    uint64_t       cands_last = 0; // 上一帧全图候选数（启动态）
    uint32_t       last_scan_us = 0;  // 上一帧扫描耗时（纯扫描，不含状态机）
    uint32_t       last_total_us = 0; // 上一帧整条链路耗时
    TrackState     state = TrackState::Startup;
    RoiWindow      last_window{};
    TrackerCounters tracker{};
    LightScanner::Trace   scan_trace{};
    RoiBlobMeasurer::Trace measure_trace{};
};

class DetectionPipeline final : public IDetector {
public:
    struct Deps {
        IPointScanner          *scanner = nullptr;
        IRoiMeasurer           *measurer = nullptr;
        const IScaleNoiseModel *noise = nullptr;
        std::function<uint64_t()> now_us; // 空 = CLOCK_MONOTONIC
    };

    // 只传配置 = 全部用自带实现（板端就是这条路径）
    explicit DetectionPipeline(const DetectionConfig &cfg);
    // 需要替换策略/时钟时用它（宿主机测试走这条）
    DetectionPipeline(const DetectionConfig &cfg, const Deps &deps);
    ~DetectionPipeline() override;

    DetectionPipeline(const DetectionPipeline &) = delete;
    DetectionPipeline &operator=(const DetectionPipeline &) = delete;

    DetectResult detect(const GrayFrame &frame) override;

    void reset();

    const DetectionStats  &stats() const { return stats_; }
    TrackState             state() const { return tracker_.state(); }
    const TargetTracker   &tracker() const { return tracker_; }
    const DetectionConfig &config() const { return cfg_; }

    // 开机自检：RVV 全图扫描 vs 标量参考逐位比对 + ROI 测量器解析校验。
    // 板端没有 RISC-V 模拟器，这是唯一能证明向量代码没写错的办法
    // （与 GraphicsUtils::selftest 同一套路）。
    static bool selftest();

    // 参数/实现一行摘要（main 只负责打出来）。静态是因为它在"建队之前"就要打 ——
    // 板端日志里参数必须出现在任何一帧之前，否则复盘时说不清"这轮到底跑的是哪套参数"。
    static void describe(const DetectionConfig &cfg, char *buf, size_t n);

private:
    uint64_t now_us() const;

    DetectionConfig    cfg_;
    LightScanner       owned_scanner_;
    RoiBlobMeasurer    owned_measurer_;
    IPointScanner     *scanner_ = nullptr;
    IRoiMeasurer      *measurer_ = nullptr;
    std::function<uint64_t()> now_fn_; // 空 = 真实单调钟
    TargetTracker      tracker_;
    std::vector<LightCandidate> cands_;
    DetectionStats     stats_{};
};

} // namespace dart::detection
