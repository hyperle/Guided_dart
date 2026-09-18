#include "vision/frame_pool.hpp"

#include <cstdio>
#include <cstring>

#include "core/check.hpp"
#include "core/log.hpp"
#include "core/mpp_map.hpp"

namespace dart {

FramePool::FramePool(uint32_t width, uint32_t height, uint32_t slots)
    : width_(width), height_(height), stride_((width + 31u) & ~31u), slots_n_(slots), slots_(new Slot[slots]) {
    // 池帧必须紧凑排列：binarize 只收 width（无 stride 参数）
    check(stride_ == width, "画幅宽度必须是 32 的倍数（帧池按紧凑排列使用）");

    // 块大小按 4K 对齐：老工程（板端验证过）的输入块池就是这么建的
    const k_u32 blk_size =
        static_cast<k_u32>(VB_ALIGN_UP(static_cast<k_u64>(stride_) * height_ * 3 / 2, 4096));

    // NOCACHE：CPU 直接写、硬件直接读，省掉所有 cache flush/invalidate
    k_vb_pool_config pool{};
    pool.blk_cnt = slots_n_;
    pool.blk_size = blk_size;
    pool.mode = VB_REMAP_MODE_NOCACHE;
    pool_id_ = kd_mpi_vb_create_pool(&pool);
    check(pool_id_ != static_cast<k_s32>(VB_INVALID_POOLID), "创建帧池失败（MMZ 不足，见 core/config.hpp 的内存账）");
    log_line("帧池: %u 槽 x %u KB = %u KB\n", slots_n_, blk_size / 1024, slots_n_ * blk_size / 1024);

    for (uint32_t i = 0; i < slots_n_; ++i) {
        Slot &s = slots_[i];
        s.handle = kd_mpi_vb_get_block(static_cast<k_u32>(pool_id_), blk_size, nullptr);
        check(s.handle != VB_INVALID_HANDLE, "帧池取块失败");

        const k_u64 phys = kd_mpi_vb_handle_to_phyaddr(s.handle);
        // cached 持久映射：池帧现在只被 CPU 碰（RVV 二值化写、识别器读、取证打包读），
        // 没有任何 DMA 消费者 —— 录像那条路是"整块 memcpy 进自己的非 cache 输入块"再喂
        // 编码器，所以池帧不需要 flush。若将来把池帧直接交给 VENC/KPU，必须在送帧前
        // mpp_invalidate/flush，否则硬件会读到旧数据。
        s.virt = static_cast<uint8_t *>(mpp_map_persist_cached(phys, blk_size));
        check(s.virt != nullptr, "帧池 mmap 失败");

        // UV 平面恒 128：编码器看到的就是黑白画面。只填这一次，之后永不再动。
        const size_t y_bytes = static_cast<size_t>(stride_) * height_;
        std::memset(s.virt + y_bytes, 128, blk_size - y_bytes);

        s.frame.slot = static_cast<int>(i);
        s.frame.view = GrayFrame{s.virt, width_, height_, stride_};
    }
}

FramePool::~FramePool() {
    for (uint32_t i = 0; i < slots_n_; ++i) {
        if (slots_[i].handle == VB_INVALID_HANDLE)
            continue;
        // 不解映射：VA 由 mpp_map_persist* 统一持有到 mpp_map_shutdown()（映射一次、
        // 永不解映射是这块板子的铁律）
        kd_mpi_vb_release_block(slots_[i].handle);
    }
    if (pool_id_ != static_cast<k_s32>(VB_INVALID_POOLID))
        kd_mpi_vb_destory_pool(static_cast<k_u32>(pool_id_));
    delete[] slots_;
}

// 照抄源帧的 mod_id：老工程注释原话「VENC 收 VICAP 帧时就是这个值」。
// 录像的输入块要用它构造帧描述（缺这个字段驱动可能认不出来）。
void FramePool::set_mod_id(k_mod_id id) {
    if (mod_id_ == id)
        return;
    mod_id_ = id;
    log_line("帧池: mod_id 采用源帧的值 %d\n", static_cast<int>(id));
}

Frame *FramePool::acquire() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (uint32_t i = 0; i < slots_n_; ++i) {
        Slot &s = slots_[next_];
        next_ = (next_ + 1) % slots_n_;
        if (!s.busy) {
            s.busy = true;
            return &s.frame;
        }
    }
    return nullptr;
}

void FramePool::release(Frame *f) {
    std::lock_guard<std::mutex> lk(mtx_);
    slots_[f->slot].busy = false;
}

} // namespace dart
