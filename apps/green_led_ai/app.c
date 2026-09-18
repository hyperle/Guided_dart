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

/* ============================ MPP 分段串行化 ============================ */

/*
 * 分三段的理由（2025-09 板端两轮实测后定稿）：
 *
 *   1) 一把全局锁把「阻塞等帧」和「送帧/取流」串在一起，是上一轮 13fps /
 *      lat=70ms 的病根：dump 一帧要占锁 33ms（≈90% 时间），别的线程只能排队。
 *   2) 但"锁内绝不做阻塞等待"又会导致采集线程空转轮询 —— 而 SDK 的 dump
 *      超时路径会 printf 一次再返回 0xA0158010（见 vision.c 注释），轮询不可行。
 *   3) 所以按**模块**分段，而不是按"全局一把"：
 *        · g_mpp_lock  (mpp_enter/leave)  —— VB 池/块、sys mmap、cache 操作：
 *                                           全是微秒级调用，锁内禁止任何等待
 *        · g_venc_lock (venc_enter/leave) —— VENC 的 send/get/release：**只有
 *                                           录像线程一个使用者**，可以在锁内
 *                                           用阻塞超时等编码器事件
 *        · VICAP（dump/release）—— 本工程里**只由采集线程调用**，因此不加锁；
 *                                          用 vicap_enter/leave 做并发自检
 *                                          （真出现了并发就计数报警，绝不静默）
 *      三条流水之间不再互相阻塞：识别线程只碰 VICAP 和自己私有的拷贝缓冲。
 */
static pthread_mutex_t g_mpp_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_venc_lock = PTHREAD_MUTEX_INITIALIZER;
static mpp_stats_t     g_mpp;
static uint64_t        g_mpp_enter_us;
static uint64_t        g_venc_enter_us;
static volatile int    g_vicap_busy;

#define MPP_HOLD_WARN_US 2000ull   /* 单次持锁超过 2ms 就计数报警 */

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
    if (held > MPP_HOLD_WARN_US)
        g_mpp.long_holds++;     /* 状态行的 long= 就是它：>0 说明锁内有阻塞调用了 */
    pthread_mutex_unlock(&g_mpp_lock);
}

void venc_enter(void)
{
    pthread_mutex_lock(&g_venc_lock);
    g_mpp.venc_calls++;
    g_venc_enter_us = mono_us();
}

void venc_leave(void)
{
    uint64_t held = mono_us() - g_venc_enter_us;
    if (held > g_mpp.venc_max_hold_us)
        g_mpp.venc_max_hold_us = held;   /* 允许大：专用锁，锁内本来就在等编码器 */
    pthread_mutex_unlock(&g_venc_lock);
}

/* VICAP 段：设计上只允许采集线程进入；这里只检测，不阻塞 */
void vicap_enter(void)
{
    if (__atomic_test_and_set(&g_vicap_busy, __ATOMIC_ACQ_REL))
        g_mpp.vc_conflicts++;            /* 必须恒为 0：非 0 说明 VICAP 被并发调用了 */
}

void vicap_leave(void)
{
    __atomic_clear(&g_vicap_busy, __ATOMIC_RELEASE);
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
    st->vc_conflicts = g_mpp.vc_conflicts;   /* 原子读，锁外取一次即可 */
}
