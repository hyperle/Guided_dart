// ============================================================================
// 主机侧 MPP 桩：在 x86 上把 Recorder / FramePool / mpp_map 真代码跑起来，
// 用断言盯住"上板才会暴露"的那几条不变量：
//
//   1) VB 块引用计数：get_block 只在启动时发生，块必须 release 才回空闲表
//   2) 空闲块不能被复用**在途**的块（同一块重复送进编码器 = 编码器读被覆写的内存）
//   3) 码流映射：整块持久映射，运行期绝不做逐包 mmap/munmap（长度还必须匹配）
//   4) 编码器输出池的块：送帧借出、release_stream 归还
//   5) 拆机顺序：stop_chn → destroy_chn → detach_vb_pool → destroy_pool(out)，
//      且输入块全部 release 之后才 destroy_pool(in)
// ============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace stub {

// 桩行为开关（测试里改）
struct Config {
    bool     send_fails = false;   // 送帧一律失败（模拟编码器永不接收）
    uint32_t stream_every = 1;     // 每 N 次送帧才吐 1 个码流（模拟编码器静默丢帧）
    uint32_t header_delay = 0;     // start 后第几次 get_stream 才给参数集包
    bool     create_chn_fails = false; // 所有 VENC 通道都建不起来（被别的子系统占着）
};

void reset(const Config &cfg);
void push_frame(const uint8_t *y, uint32_t len); // 测试侧给"编码器"喂内容（可选）

// 计数与断言
struct Stats {
    uint64_t get_block = 0;
    uint64_t release_block = 0;
    uint64_t send_ok = 0;
    uint64_t send_fail = 0;
    uint64_t streams = 0;
    uint64_t header_packs = 0;
    uint64_t mmap = 0;
    uint64_t mmap_cached = 0;
    uint64_t munmap = 0;
    uint64_t invalidate = 0;
    uint64_t blocks_allocated = 0; // 所有池里被 get_block 出来的块总数
};

const Stats &stats();
std::vector<std::string> &call_trace();

// 校验：调用过 munmap 的每个映射，长度必须与 map 时一致（老工程"长度不匹配打挂内核"）
void check_all(int *failures);

} // namespace stub
