#pragma once

// ============================================================================
// 工况 1：启动阶段的**全图扫描**（RVV 向量 + 二值图）。
//
// 目标在这个阶段只有几个像素，"先找出来"比"算得准"重要，所以整条路径按
// "一遍读、尽量少的标量操作"设计：
//
//   ① 瓦片粗筛（RVV）：把二值图按 16×16 切成瓦片，每行每条向量算一次
//      比较 + vpopc（命中数）+ vwredsumu（命中像素的 x 索引和）。
//      一帧 640×360 = 14400 次向量迭代，板端量级 ~0.1ms（跟二值化同一个数量级）。
//   ② 瓦片并块：瓦片上做 8 邻域 BFS。**面积与质心在这一步就已经是精确值** ——
//      因为"有亮像素的瓦片"是全体，组件内像素恰好等于这些瓦片里的亮像素总和。
//   ③ 精修（只对最亮的前 K 块，标量）：算出紧致包围盒与 fill。
//      面积/质心不受影响；这一步只是把包围盒从"瓦片粒度"收紧到像素粒度。
//      有像素预算上限（refine_max_px），整片过曝时不会把时间吃光。
//
// 为什么不用"逐行游程 + 并查集"的经典连通域：那条路每行都要标量扫 640 个像素，
// 单帧 230K 次标量迭代，跟"RVV 一遍过"不是一个量级；而启动阶段只关心
// "最亮的几块"，瓦片粗筛的精度损失（两个目标落进同一瓦片会被并成一块）
// 在这个阶段无所谓（目标小、数量少），换来的速度才是这个阶段真正需要的。
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/frame.hpp"
#include "detection/config/scanner.hpp"
#include "detection/domain/observations.hpp"
#include "detection/math/shape_metrics.hpp"

namespace dart::detection {

// 全图扫描器接口。实现只回答一个问题：
//   "给定一张二值图，最亮的 K 块亮斑在哪、多大。"
// 做成接口是为了：① 换粗筛手段（如灰度投影）时下游不用动；② 宿主机测试注入假件；
// ③ 将来 KPU 出候选也能直接接进来。
class IBlipScanner {
public:
    virtual ~IBlipScanner() = default;

    // 把候选按 score 降序写入 out（最多 cap 个），返回实际写入个数。
    virtual size_t scan(const GrayFrame &frame, Blip *out, size_t cap) = 0;
    virtual const char *name() const = 0;
};

class TileScanner final : public IBlipScanner {
public:
    explicit TileScanner(const ScannerConfig &cfg);

    size_t       scan(const GrayFrame &frame, Blip *out, size_t cap) override;
    const char  *name() const override;

    // 开机自检：同一张合成图上把**向量路径**与**标量参考路径**各跑一遍，
    // 逐字段比对候选表。板端没有 RISC-V 模拟器，这是唯一能证明向量代码没写错的办法
    // （与 GraphicsUtils::selftest 同一套路，对应 DEBUG_GUIDE 里"首次执行 RVV 静默停住"的教训）。
    static bool selftest();

    // 上一帧的粗筛明细（板端调参用：瓦片数/种子瓦片数/组件数/精修块数）
    struct Trace {
        uint32_t tiles = 0;
        uint32_t seed_tiles = 0;
        uint32_t components = 0;
        uint32_t refined = 0;
        bool     rvv = false;
    };
    const Trace &last_trace() const { return trace_; }

private:
    // 瓦片级组件（面积/质心在粗筛阶段就已经精确）
    struct Component {
        uint64_t sx = 0; // Σx（像素坐标）
        uint64_t sy = 0; // Σy
        uint32_t area = 0;
        uint32_t tiles = 0;
        uint16_t tx0 = 0, ty0 = 0, tx1 = 0, ty1 = 0; // 组件占用瓦片的包围盒（含）
    };

    void   ensure_grid(uint32_t w, uint32_t h);
    size_t run(const GrayFrame &frame, Blip *out, size_t cap, bool use_rvv);
    uint32_t tile_pass_rvv(const GrayFrame &frame);
    uint32_t tile_pass_scalar(const GrayFrame &frame);
    uint32_t merge_components();
    bool     refine(const GrayFrame &f, uint32_t comp_index, const Component &c, Blip *out) const;

    ScannerConfig cfg_;
    uint32_t      tile_ = 16;
    uint32_t      gw_ = 0, gh_ = 0;

    // 每瓦片累加器（粗筛产出）
    std::vector<uint32_t> count_;
    std::vector<uint32_t> xsum_;
    std::vector<uint32_t> ysum_;
    // 每个瓦片归属的组件号（-1 = 无亮像素），并块阶段填，精修阶段用
    std::vector<int32_t> comp_id_;
    std::vector<uint32_t> bfs_;   // BFS 栈（存瓦片线性索引）
    std::vector<uint32_t> order_; // 排序索引（每帧复用，避免逐帧分配）
    std::vector<Component> comps_; // 组件表（每帧复用）

    Trace trace_{};
};

} // namespace dart::detection
