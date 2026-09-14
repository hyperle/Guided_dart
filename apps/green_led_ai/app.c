/*
 * green_led_ai —— 日志、计时、最近结果、MPP 串行化与映射缓存实现
 */
#include "app.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

cfg_t g_cfg;

/* ============================ 日志 ============================ */

static int log_ready;

static void log_dir_ensure(void)
{
    mkdir(APP_LOG_DIR, 0777);
}

void app_log(const char *fmt, ...)
{
    char buf[640];
    va_list ap;
    int n;

    if (!log_ready) {
        log_dir_ensure();
        log_ready = 1;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if (n > (int)sizeof(buf) - 1)
        n = (int)sizeof(buf) - 1;

    /* stdout：launcher 会重定向到 launcher-child.log；msh 手动跑也可见 */
    fwrite(buf, 1, (size_t)n, stdout);
    fflush(stdout);

    /* 直写日志文件：open -> write -> fsync -> close，MTP 可随时读取 */
    int fd = open(APP_LOG_PATH, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0)
        return;
    ssize_t w = write(fd, buf, (size_t)n);
    fsync(fd);
    close(fd);
    (void)w;
}

void trace_log(uint64_t seq, const char *fmt, ...)
{
    char body[512];
    va_list ap;
    int n;

    if (g_cfg.trace_frames <= 0 || seq > (uint64_t)g_cfg.trace_frames)
        return;

    va_start(ap, fmt);
    n = vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    app_log("trace seq=%llu %s\n", (unsigned long long)seq, body);
}

/* ============================ 时间 ============================ */

uint64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/* ============================ 最近结果 ============================ */

static pthread_mutex_t g_res_lock = PTHREAD_MUTEX_INITIALIZER;
static result_t        g_res = { .center_x = -1, .center_y = -1, .seq = 0, .fps = 0, .found = 0 };

void result_publish(int32_t cx, int32_t cy, uint64_t seq, uint32_t fps)
{
    pthread_mutex_lock(&g_res_lock);
    g_res.center_x = cx;
    g_res.center_y = cy;
    g_res.seq = seq;
    g_res.fps = fps;
    g_res.found = (cx >= 0);
    pthread_mutex_unlock(&g_res_lock);
}

void result_get(result_t *out)
{
    if (!out)
        return;
    pthread_mutex_lock(&g_res_lock);
    *out = g_res;
    pthread_mutex_unlock(&g_res_lock);
}

/* ============================ MPP 串行化 ============================ */

/*
 * 全进程唯一的一把 MPP 锁：kd_mpi_sys_mmap/munmap、kd_mpi_vicap_dump_*、
 * kd_mpi_venc_send_frame/get_stream/release_stream 全部在它内部执行。
 * 实测这些调用并发会把 RT-Smart 打挂。
 */
static pthread_mutex_t g_mpp_lock = PTHREAD_MUTEX_INITIALIZER;
static mpp_stats_t     g_mpp;
static uint64_t        g_mpp_enter_us;

void mpp_enter(void)
{
    if (pthread_mutex_trylock(&g_mpp_lock) != 0) {
        g_mpp.contended++;      /* 记一笔：确实发生了并发，需要复盘调用点 */
        pthread_mutex_lock(&g_mpp_lock);
    }
    g_mpp.calls++;
    g_mpp_enter_us = mono_us();
}

void mpp_leave(void)
{
    uint64_t held = mono_us() - g_mpp_enter_us;
    if (held > g_mpp.max_hold_us)
        g_mpp.max_hold_us = held;
    pthread_mutex_unlock(&g_mpp_lock);
}

/* ---------------- 物理地址映射缓存（一次映射，永不逐帧解映射） ---------------- */

#define MPP_MAP_CACHE_N 512

typedef struct {
    k_u64    phys;
    k_u32    size;
    uint8_t *va;
    int      cached;      /* 0=非 cache 映射，1=cache 映射（两者可以并存） */
} map_entry_t;

static map_entry_t g_map_cache[MPP_MAP_CACHE_N];
static uint32_t    g_map_used;

static void *map_persist_common(k_u64 phys, k_u32 size, int cached)
{
    if (!phys || !size)
        return NULL;

    /*
     * 同一物理地址可能被以不同长度请求 —— VENC 码流包就是这样：池子里的块
     * 被反复复用，每包长度都不同（板端实测 29/45/24/55 字节…）。
     * 处理方式：length 覆盖得住就直接复用；覆盖不住就**为更大的长度另建一条
     * 映射**（映射只增不减、永不解映射，因此多一条映射是安全的；内核被打挂的
     * 是「并发 map/unmap + 长度不匹配的 munmap」，不是这里的多映射）。
     * 调用方用 blk_base 归一化（见 record.c）时通常不会走到这条路径。
     */
    for (uint32_t i = 0; i < g_map_used; ++i) {
        if (g_map_cache[i].phys == phys && g_map_cache[i].cached == cached &&
            g_map_cache[i].size >= size)
            return g_map_cache[i].va;
    }
    if (g_map_used >= MPP_MAP_CACHE_N) {
        app_log(APP_NAME_STR ": map cache full (%u)，丢弃本次映射 phys=0x%llx size=%u\n",
                g_map_used, (unsigned long long)phys, size);
        return NULL;
    }

    void *va = cached ? kd_mpi_sys_mmap_cached(phys, size) : kd_mpi_sys_mmap(phys, size);
    if (!va) {
        app_log(APP_NAME_STR ": mmap failed phys=0x%llx size=%u\n",
                (unsigned long long)phys, size);
        return NULL;
    }
    if (g_map_used && (g_map_used % 64u) == 0)
        app_log(APP_NAME_STR ": map cache 增长到 %u 条（同址多长度映射，正常但偏多）\n",
                g_map_used);
    g_map_cache[g_map_used].phys = phys;
    g_map_cache[g_map_used].size = size;
    g_map_cache[g_map_used].va = (uint8_t *)va;
    g_map_cache[g_map_used].cached = cached;
    g_map_used++;
    g_mpp.maps++;
    return va;
}

void *mpp_map_persist(k_u64 phys, k_u32 size)
{
    return map_persist_common(phys, size, 0);
}

void *mpp_map_persist_cached(k_u64 phys, k_u32 size)
{
    return map_persist_common(phys, size, 1);
}

void mpp_invalidate(k_u64 phys, void *va, k_u32 size)
{
    if (!va || !size)
        return;
    kd_mpi_sys_mmz_invalidate_cache(phys, va, size);
}

void mpp_shutdown(void)
{
    /* 必须按 map 时的长度 unmap，长度不匹配同样会打挂内核 */
    mpp_enter();
    for (uint32_t i = 0; i < g_map_used; ++i) {
        if (g_map_cache[i].va)
            kd_mpi_sys_munmap(g_map_cache[i].va, g_map_cache[i].size);
        g_map_cache[i].va = NULL;
    }
    g_map_used = 0;
    mpp_leave();
}

void mpp_get_stats(mpp_stats_t *st)
{
    if (!st)
        return;
    mpp_enter();
    *st = g_mpp;
    mpp_leave();
}
