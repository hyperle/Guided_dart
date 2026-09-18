#pragma once

#include "k_type.h"

namespace dart {

// 物理地址 → VA 的**持久**映射：映射只增不减，运行期永不解映射，退出时统一释放。
//
// 为什么必须这样（老工程 README 的板端实测结论）：
//   * `kd_mpi_sys_mmap/munmap` 与 `kd_mpi_venc_send_frame` / `get_stream` 在**多线程
//     并发**时会把 RT-Smart 打挂 —— 旧工程 29 次重启循环的来源；
//   * 同一物理地址若以不同长度 munmap，同样会打挂内核。
// 因此：谁都不许逐包/逐帧 mmap+munmap，一律从这里拿 VA。
//
// 两种映射：
//   mpp_map_persist()        非 cache。CPU 写、硬件读，写完立刻可见（VICAP 帧、录像输入块）
//   mpp_map_persist_cached() cache。CPU 读得快（码流落盘），但**每次读之前**必须
//                            mpp_invalidate()，否则读到的是 DMA 写之前的旧 cache 行
void *mpp_map_persist(k_u64 phys, k_u32 size);
void *mpp_map_persist_cached(k_u64 phys, k_u32 size);

// 作废 [phys, phys+size) 对应的 cache 行（回调自 kd_mpi_sys_mmz_invalidate_cache）
void mpp_invalidate(k_u64 phys, void *va, k_u32 size);

// 统一解映射（等价于老工程 mpp_shutdown()）：必须在所有线程 join 之后调用，
// 且严格按 map 时的长度释放。
void mpp_map_shutdown();

} // namespace dart
