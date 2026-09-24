#pragma once

// ============================================================================
// 工况 2 的"看一眼"：在**预测 ROI 窗口内**做连通域 + 形状筛选，取出目标测量。
//
// 这一步是"启动阶段确认完目标之后，开始使用现有的处理逻辑"的落点：
// 口径与 `apps/green_led_ai/detect_color.c` 完全一致（8 邻域连通域 → 填充率/长宽比
// 筛选 → 取最大块），区别只有两点：
//   ① 只在窗口内扫（窗口由 ROI 预测给出，通常是整幅的 1%~5%）；
//   ② 用"游程 + 并查集"代替"整窗掩码 + BFS"：省掉一块 w×h 的中间掩码
//      （板端用户堆只有 16MB，而掩码最大可以到 640×360），顺带把面积/矩
//      一次性算干净（每行游程的 Σx 是等差数列，不用逐像素累加）。
//
// 输出语义（复用 dart::RoiState）：
//   NoTargetInRoi  —— 窗口内没有合格块（状态机记一次 miss）
//   Hit            —— 块完整落在窗口内（可放心用于尺度更新）
//   TargetOutOfRoi —— 块贴着窗口边（面积只是下界 → quality 降低 → R_scale 放大）
// ============================================================================

#include <cstdint>
#include <vector>

#include "core/frame.hpp"
#include "detection/config.hpp"
#include "detection/types.hpp"

namespace dart::detection {

class IRoiMeasurer {
public:
    virtual ~IRoiMeasurer() = default;

    // 在 window 内测量目标（window 必须已经在画面内）。无目标时返回 valid=false。
    virtual TargetMeasurement measure(const GrayFrame &frame, const RoiWindow &window) = 0;
    virtual const char *name() const = 0;
};

class RoiBlobMeasurer final : public IRoiMeasurer {
public:
    explicit RoiBlobMeasurer(const MeasureConfig &cfg);

    TargetMeasurement measure(const GrayFrame &frame, const RoiWindow &window) override;
    const char       *name() const override { return "roi-rle-cc"; }

    // 上一帧的明细（板端调参：游程数 / 合格块数 / 被丢的原因）
    struct Trace {
        uint32_t runs = 0;
        uint32_t blobs = 0;      // 通过面积门限的块数
        uint32_t rejected = 0;   // 被形状门限（填充率/长宽比）丢掉的块数
        bool     overflow = false; // 游程超上限（本帧按"没测到"处理）
    };
    const Trace &last_trace() const { return trace_; }

private:
    // 一行的游程（窗口内相对坐标，闭区间）
    struct Run {
        uint16_t x0 = 0;
        uint16_t x1 = 0;
    };
    // 组件的累加器
    struct Blob {
        uint64_t sx = 0, sy = 0;
        uint32_t area = 0;
        uint64_t sxx = 0, syy = 0, sxy = 0; // 二阶矩（圆度用）
        uint16_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    };

    int32_t find_root(int32_t i);

    MeasureConfig cfg_;
    // ---- 游程 + 并查集（全部按帧复用，容量只增不减，运行期零分配）----
    std::vector<int32_t>  parent_;  // 并查集父指针（按游程 id）
    std::vector<uint8_t>  rank_;    // 按秩合并
    std::vector<uint16_t> run_x0_;  // 游程（窗口内相对坐标，闭区间）
    std::vector<uint16_t> run_x1_;
    std::vector<uint16_t> run_y_;
    std::vector<uint64_t> run_sxx_;   // 每个游程的 Σx²（闭式：连续整数平方和）
    std::vector<uint64_t> run_sxy_;   // 每个游程的 Σxy（= y·Σx）
    std::vector<Run>      prev_row_; // 上一行的游程（位置 + 游程 id 的对应）
    std::vector<Run>      cur_row_;
    std::vector<int32_t>  prev_id_;
    std::vector<int32_t>  cur_id_;
    // ---- 结算 ----
    std::vector<Blob>     blobs_;
    std::vector<int32_t>  root_blob_; // 根 -> blobs_ 下标（-1 表示还没建）
    Trace                 trace_{};
};

} // namespace dart::detection
