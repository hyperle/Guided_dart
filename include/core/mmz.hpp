#pragma once

#include <cstdint>

namespace dart {

// MMZ（硬件缓冲的唯一来源）当前还能拿到多少内存：逐次申请 1MB 直到失败，然后立刻全部释放。
// 为什么要它：VENC 的内部缓冲（VPU 固件工作区、参考帧）也是从 MMZ 出的，如果我们的
// VB 池把 MMZ 吃光，编码器会**收帧但一个字节都不出**——这种失败在 API 返回值上看不出来，
// 只能靠这个数字。
// 注意：它会把剩余内存短暂占满再释放，只在启动阶段/异常排查时调用。
uint32_t mmz_free_mb();

// 同时探一下单块最大可分配值（碎片化程度），失败返回 0
uint32_t mmz_max_block_mb();

} // namespace dart
