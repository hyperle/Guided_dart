/*
 * rec_dir —— 录像目录管理（会话编号 + 容量上限淘汰）
 *
 * 单独成模块的原因：这段逻辑会**删掉用户的录像文件**，属于「出错代价最高」
 * 的一段代码，而它完全不依赖任何硬件（纯 POSIX 文件操作），因此可以在宿主机上
 * 直接跑单元测试（apps/green_led_ai/test/recdir_host_test.c）。
 *
 * 目录约定（与旧工程一致，便于沿用离线脚本）：
 *   <dir>/rec_%04u.h264   Annex-B H.264 裸流
 *   <dir>/rec_%04u.csv    逐帧识别结果
 * 同一 idx 的两个文件属于同一段会话，淘汰时**必须成对删除**（否则会留下孤儿文件）。
 */
#ifndef GREEN_LED_AI_REC_DIR_H
#define GREEN_LED_AI_REC_DIR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 扫描 dir，返回下一个会话序号（rec_<idx> 的最大 idx + 1；空目录返回 1） */
uint32_t rec_dir_next_index(const char *dir);

/*
 * 删除最旧会话，直到目录里 rec_*.h264/.csv 的总大小 <= cap_bytes。
 * 返回删除的会话数（-1 表示目录打不开）。
 *
 * 语义保证（单元测试覆盖）：
 *   - 只在「总量 > cap」时删，且从序号最小的会话开始删；
 *   - 同一会话的 .h264 与 .csv 一起删；
 *   - 只认 rec_<数字>.h264 / rec_<数字>.csv，其余文件（含 .ppm、日志）绝不碰。
 */
int rec_dir_evict(const char *dir, uint64_t cap_bytes);

/* 目录内 rec_*.h264/.csv 的总字节数（测试与日志用；-1 = 打不开） */
long long rec_dir_total_bytes(const char *dir);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_REC_DIR_H */
