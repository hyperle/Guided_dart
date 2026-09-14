/*
 * control —— 运行期控制接口（Unix socket）
 *
 * 目的：视觉与录像可以分别、单独启停，互不影响；外部（板端 msh / 主机脚本）
 * 用一条 socket 命令即可操作，不必重启进程（沿用旧工程的控制方式）。
 *
 *   vision on | off | status
 *   record on | off | status | cap <bytes> | dir <path>
 *   status                     # 一行汇总（含两侧统计）
 *   help
 */
#ifndef GREEN_LED_AI_CONTROL_H
#define GREEN_LED_AI_CONTROL_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int  control_start(void);
void control_stop(void);

/* 汇总状态行（周期日志与 status 命令共用同一口径） */
void control_status_line(char *buf, size_t n);

/* 探针行：rgb/lab、亮度 y=[min,max,mean]、阈值与 ROI（照旧工程的调参口径） */
void control_probe_line(char *buf, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* GREEN_LED_AI_CONTROL_H */
