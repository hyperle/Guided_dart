#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>

#include "core/config.hpp"
#include "core/frame.hpp"
#include "vision/frame_pool.hpp"

#include "mpi_vb_api.h"
#include "mpi_venc_api.h"

namespace dart {

// 黑白录像（H.264，左上角烧关键数据）。四条硬约束决定它长这样：
//
//   1) **识别优先**：submit() 只入队，队列满/池帧紧张就丢这一帧并计数，永不阻塞。
//      队列刻意做浅 —— 它同时是"录像最多能攥住几个识别池帧"的上限。
//
//   2) **录像私有输入块**：识别池帧拷一份进录像自己的 VB 块，池帧拷完立刻归还。
//      编码器只碰录像的块，三条流水（采集/识别、编码、写盘）彻底解耦。
//
//   3) **输入块启动时一次取满，运行期永不再 get_block**（老工程 record.c 板端验证过
//      的形态）：块状态机 free → inflight → free，只有**对应的码流取回来**才回收。
//      上一版每帧 get_block 又不 release，6 个块 7 帧就漏光 —— 这条是本次重构的核心。
//      块状态只在录像线程里改，因此**不需要锁**。
//
//   4) **码流走持久 cache 映射**：输出池每个块只 mmap 一次（块大小恒定），读之前
//      invalidate，运行期绝不做逐包 mmap/munmap（老工程"内核被打挂"的可疑路径，
//      也是这里最省的一次系统调用都没有的路径）。
//
// 帧所有权：主线程 submit() 即交出池帧，录像线程拷进输入块后立刻归还池。
class Recorder {
public:
    Recorder(const Config &cfg, FramePool &pool);
    ~Recorder();

    // 返回 false = 录像链路建不起来（池/通道/文件），**但进程不该因此死掉**：
    // 识别才是主功能，录像只是旁路。失败原因已经打进日志。
    bool start();
    void stop();

    void submit(Frame *f); // 非阻塞：入队失败即丢帧（计数），绝不等待

    // 计数器：录像线程写、主线程读，relaxed 原子（rv64 上是原生 8 字节原子，零开销）
    uint64_t frames() const { return frames_.load(std::memory_order_relaxed); }           // 送进编码器的帧
    uint64_t send_fail() const { return send_fail_.load(std::memory_order_relaxed); }     // 送帧失败/超时
    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }         // 队列满/预留不足丢的帧
    uint64_t blk_empty() const { return blk_empty_.load(std::memory_order_relaxed); }     // 输入块用光丢的帧
    uint64_t reclaimed() const { return reclaimed_.load(std::memory_order_relaxed); }     // 强制回收次数
    uint64_t underflow() const { return underflow_.load(std::memory_order_relaxed); }     // 出包多于送帧（编码器行为异常）
    uint64_t io_fail() const { return io_fail_.load(std::memory_order_relaxed); }         // 写文件失败次数
    uint64_t bytes() const { return bytes_.load(std::memory_order_relaxed); }
    uint32_t inflight() const { return inflight_n_; } // 在途块数（诊断用）

private:
    static constexpr uint32_t kQueueDepth = 3;    // 录像队列深度（= 最多攥住几个池帧）
    uint32_t reserve_frames_ = 4; // 池里至少给识别留这么多空闲帧（按帧池大小算，见 ctor）
    static constexpr uint32_t kInBlkMax = 8;      // 输入块数组上限（实际块数取自 cfg_.rec_in_blocks）

    struct InBlk {
        k_vb_blk_handle    h = VB_INVALID_HANDLE;
        k_u64              phys = 0;
        uint8_t           *va = nullptr; // 非 cache 持久映射
        k_video_frame_info info{};       // 填好的帧描述，直接送编码器
    };

    void run();                                  // 录像线程主体
    bool copy_and_send(Frame *src);              // 拷进空闲输入块 → 送编码器
    uint32_t drain_once(int32_t timeout_ms);     // 取一次码流并落盘；返回视频包数
    uint32_t write_packs(const k_venc_stream &st);
    const uint8_t *map_pack(const k_venc_pack &p);
    void write_bytes(const uint8_t *data, uint32_t len);
    void reclaim(uint32_t n);                    // 在途块 → 空闲（FIFO，与送帧同序）
    void maybe_flush(bool force);
    void log_venc_status(const char *why);
    bool probe_encoder();                    // --venc-probe：候选编码配置逐个试编
    bool setup_chn(uint32_t chn, k_payload_type type, uint32_t enc_h, uint32_t fps, uint32_t gop,
                   uint32_t kbps, uint32_t q_factor, uint32_t intbuf_mb);
    void fill_synth(uint32_t bi, uint32_t frame_no);
    bool prepare_in_blocks();
    bool open_file();
    bool setup_venc();
    void teardown_venc();
    void annotate(uint8_t *dst, const Frame *f);

    const Config cfg_;
    FramePool   &pool_;

    int32_t  venc_pool_id_ = -1;
    uint32_t chn_ = 0;              // 实际用上的 VENC 通道（cfg_.venc_chn 可能被占）
    int32_t  in_pool_id_ = -1;
    uint32_t out_blk_bytes_ = 0;
    uint32_t in_blk_bytes_ = 0;
    uint32_t y_bytes_ = 0;
    int32_t  last_ret_ = 0;

    InBlk    in_[kInBlkMax];
    uint32_t in_blk_cnt_ = 0;
    uint32_t enc_h_ = 0;               // 实际编码高度（≤ cfg_.height；探针可能把它降到 16 对齐）
    uint32_t probe_left_ = 0;          // 自检留下的卡死通道数（>0 时任何池都不能销毁）
    bool     chn_ready_ = false;       // 自检已建好并采纳了通道 → 主链路直接用它
    bool     adopt_mjpeg_ = false;     // 自检发现只有 MJPEG 能出码流 → 录 MJPEG（文件后缀换 .mjpg）
    bool     skip_teardown_ = false;   // 送过帧但一字节码流都没出 → 退出时不拆链（拆了会把内核带走）
    int16_t  free_[kInBlkMax] = {};     // 空闲块索引（栈）
    uint32_t free_n_ = 0;
    int16_t  inflight_[kInBlkMax] = {};    // 已送帧、等码流回收的块（FIFO）
    uint64_t inflight_ms_[kInBlkMax] = {}; // 每块送帧时刻：超时未回收就强收（防编码器丢帧把录像拖死）
    uint32_t inflight_n_ = 0;
    uint64_t last_status_ms_ = 0;      // 上次打编码器内部状态的时间

    uint32_t attempts_ = 0;             // 录像线程尝试送帧次数（只在前几次打点，防日志洪泛）
    uint8_t  hdr_[1024] = {};           // SPS/PPS（每个 I 帧前重发，单文件从中间也能解）
    uint32_t hdr_len_ = 0;

    std::FILE *file_ = nullptr;
    uint64_t   last_flush_ms_ = 0;
    bool       warned_pack_addr_ = false;

    std::thread             thread_;
    std::mutex              q_mtx_;
    std::condition_variable q_cv_;
    Frame                  *q_[kQueueDepth] = {};
    uint32_t                q_head_ = 0;
    uint32_t                q_count_ = 0;
    bool                    running_ = false;
    bool                    stopped_ = false; // 只停一次（main 与析构都会停）

    std::atomic<uint64_t> frames_{0};
    std::atomic<uint64_t> send_fail_{0};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<uint64_t> blk_empty_{0};
    std::atomic<uint64_t> reclaimed_{0};
    std::atomic<uint64_t> underflow_{0};
    std::atomic<uint64_t> io_fail_{0};
    std::atomic<uint64_t> bytes_{0};
};

} // namespace dart
