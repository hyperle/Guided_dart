/* 宿主机测试桩：只提供 detect_color.c 需要的外部符号 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "app.h"

cfg_t g_cfg;
void app_log(const char *fmt, ...) { va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); }
void trace_log(uint64_t seq, const char *fmt, ...) { (void)seq; (void)fmt; }
uint64_t mono_us(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000ull + ts.tv_nsec/1000; }
static result_t g_r;
void result_publish(int32_t cx, int32_t cy, uint64_t seq, uint32_t fps){ g_r.center_x=cx; g_r.center_y=cy; g_r.seq=seq; g_r.fps=fps; }
void result_get(result_t *o){ *o = g_r; }
void mpp_enter(void) {} void mpp_leave(void) {}
void *mpp_map_persist(k_u64 p, k_u32 s) { (void)p; (void)s; return 0; }
void mpp_shutdown(void) {}
void mpp_get_stats(mpp_stats_t *st) { if (st) { st->calls=0; st->contended=0; st->max_hold_us=0; st->maps=0; } }
