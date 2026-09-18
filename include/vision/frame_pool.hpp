#pragma once

#include <cstdint>
#include <mutex>

#include "core/frame.hpp"

#include "mpi_sys_api.h"
#include "mpi_vb_api.h"

namespace dart {

// 二值化帧池。两个关键约束决定了它的实现：
//   1) 内存来自 MMZ 的 VB 池 —— 录像是整块拷进自己的输入块再喂 VENC（硬件 DMA），
//      普通堆内存喂不了硬件；物理地址对未来接 KPU 识别器也是必需的。
//   2) 池帧布局就是 YUV420SP（Y = 二值图，UV 恒 128），于是"二值化 → 识别 → 录像拷一份"
//      全程只写一次内存；池帧永远不交给编码器，编码器只碰录像自己的块。
//
// 并发约定：主线程 acquire，录像线程拷完即 release，所以这两个操作带锁
// （锁只护住"哪一块空闲"这一点状态，每帧各一次，开销可忽略）。
class FramePool {
public:
    FramePool(uint32_t width, uint32_t height, uint32_t slots);
    ~FramePool();

    Frame *acquire(); // 无空闲槽返回 nullptr：调用方丢这一拍，绝不阻塞识别链路
    void   release(Frame *f);

    // 照抄源 VICAP 帧的 mod_id（老工程同款做法，录像输入块要用它构造帧描述）
    void set_mod_id(k_mod_id id);
    k_mod_id mod_id() const { return mod_id_; }

    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }
    uint32_t slots() const { return slots_n_; }

    // 空闲帧数：录像侧用它做"识别优先"的门限
    uint32_t free_count() const {
        std::lock_guard<std::mutex> lk(mtx_);
        uint32_t n = 0;
        for (uint32_t i = 0; i < slots_n_; ++i)
            if (!slots_[i].busy)
                ++n;
        return n;
    }

private:
    struct Slot {
        k_vb_blk_handle handle = VB_INVALID_HANDLE;
        uint8_t        *virt = nullptr;
        Frame           frame{};
        bool            busy = false;
    };

    const uint32_t width_;
    const uint32_t height_;
    const uint32_t stride_; // 32 字节对齐：VENC/ISP 对 stride 有对齐要求
    const uint32_t slots_n_;
    Slot          *slots_;
    uint32_t       next_ = 0;
    k_s32          pool_id_ = VB_INVALID_POOLID;
    k_mod_id       mod_id_ = static_cast<k_mod_id>(0);
    mutable std::mutex mtx_;
};

} // namespace dart
