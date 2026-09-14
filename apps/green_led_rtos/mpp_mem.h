/*
 * mpp_mem —— MPP 用户态内存映射的进程内串行化。
 *
 * 背景（实测）：kd_mpi_sys_mmap / kd_mpi_sys_munmap 在**多线程并发**使用时会
 * 把 RT-Smart 打挂（整机重启，不是进程收到 SIGSEGV）。板端日志定位到：
 *   识别线程刚 map 完一帧（还没 unmap），采集线程为了喂录像又 map 同一批
 *   VICAP buffer -> 立刻重启。
 * 官方示例都是单线程 dump->编码，所以从没暴露这个问题。
 *
 * 这里把"映射 -> 使用 -> 解映射"整段做成互斥：全进程任一时刻只允许存在一次
 * 有效映射。传输/编码调用（kd_mpi_venc_*）不在此列，它们内部有自己的缓冲。
 */
#ifndef GREEN_LED_MPP_MEM_H
#define GREEN_LED_MPP_MEM_H

#include <stdint.h>

#include "mpi_sys_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 进入/离开"MPP 映射临界区"（可重入？不可 —— 不要嵌套） */
void  mpp_mem_begin(void);
void  mpp_mem_end(void);

/* 必须在 mpp_mem_begin/end 之间调用 */
void *mpp_mem_map(k_u64 phys, k_u32 size);
void  mpp_mem_unmap(void *va, k_u32 size);

/* 自检统计：映射次数、因并发而等待的次数 */
void  mpp_mem_stats(uint64_t *maps, uint64_t *contended);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_MPP_MEM_H */
