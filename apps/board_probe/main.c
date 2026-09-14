/*
 * board_probe —— 板端环境探针（一次性回答「这块板子能给视觉流水多少资源」）
 *
 * 为什么需要它：业务代码在主机上只能编译、不能运行，下面这些量必须在板上实测：
 *   1) MMZ 可用总量（K230D 只有 60MB MMZ，乒乓缓冲/模型/VB 池都要从这里出）
 *   2) 用户态堆与 /sdcard 剩余空间
 *   3) CPU 实际频率（判断 RT-Smart 是否跑在 1.6GHz 的 RVV 大核上；按 U-Boot
 *      k230_boot_core() + TOC boot=0x3 推断是 CPU1，这里实测确认）
 *   4) RVV 向量单元是否真的可用、相对标量加速比
 *   5) /dev/gnne_device(KPU)、/dev/ai_2d_device(AI2D) 等设备节点是否存在
 *   6) PM 频率档位（必要时可把 CPU/KPU 锁到最高档）
 *
 * 输出：stdout + /sdcard/app/logs/board_probe.log（与业务应用同一套日志模式）
 *
 * 编译：bash scripts/build.sh board_probe
 * 运行：msh> /sdcard/app/board_probe        （前台跑，几十秒内结束）
 */
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <time.h>
#include <unistd.h>

#include "mpi_sys_api.h"
#include "mpi_pm_api.h"
#include "k_pm_comm.h"

#if defined(__riscv_vector)
#include <riscv_vector.h>
#define PROBE_HAVE_RVV 1
#endif

#define PROBE_LOG "/sdcard/app/logs/board_probe.log"

/* ============================ 日志 ============================ */

static void plog(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    int n;

    static int dir_ok;
    if (!dir_ok) {
        mkdir("/sdcard/app/logs", 0777);
        dir_ok = 1;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    if (n > (int)sizeof(buf) - 1)
        n = (int)sizeof(buf) - 1;

    fwrite(buf, 1, (size_t)n, stdout);
    fflush(stdout);

    int fd = open(PROBE_LOG, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd >= 0) {
        ssize_t w = write(fd, buf, (size_t)n);
        fsync(fd);
        close(fd);
        (void)w;
    }
}

static uint64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

/* ============================ 1) MMZ / 堆 ============================ */

static void probe_mmz(void)
{
    plog("probe: ---- MMZ ----\n");
    /* 逐档试探最大可分配块；每档成功即释放，测的是「单块最大值」 */
    static const uint32_t steps[] = {
        1u << 20, 2u << 20, 4u << 20, 8u << 20, 16u << 20,
        24u << 20, 32u << 20, 48u << 20, 64u << 20, 96u << 20, 128u << 20
    };
    uint32_t max_ok = 0;
    for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i) {
        k_u64 phys = 0;
        void  *virt = NULL;
        k_s32 ret = kd_mpi_sys_mmz_alloc(&phys, &virt, "probe", NULL, steps[i]);
        if (ret == K_SUCCESS && virt) {
            plog("probe: mmz single block %u MB OK phys=0x%llx\n",
                 (unsigned)(steps[i] >> 20), (unsigned long long)phys);
            max_ok = steps[i];
            kd_mpi_sys_mmz_free(phys, virt);
        } else {
            plog("probe: mmz single block %u MB FAILED ret=0x%08x\n",
                 (unsigned)(steps[i] >> 20), (unsigned)ret);
            break;
        }
    }

    /* 累计可分配量：一直申请 1MB 直到失败（上限 256 块，避免打满系统） */
    k_u64 phys[256];
    void  *virt[256];
    int     n = 0;
    for (; n < 256; ++n) {
        if (kd_mpi_sys_mmz_alloc(&phys[n], &virt[n], "probe", NULL, 1u << 20) != K_SUCCESS ||
            !virt[n]) {
            plog("probe: mmz total alloc stopped at %d MB (ret fail)\n", n);
            break;
        }
    }
    plog("probe: mmz cumulative allocatable >= %d MB (single max %u MB)\n",
         n, (unsigned)(max_ok >> 20));
    for (int i = 0; i < n; ++i)
        kd_mpi_sys_mmz_free(phys[i], virt[i]);
}

static void probe_fs(void)
{
    plog("probe: ---- filesystem ----\n");
    struct statvfs vfs;
    if (statvfs("/sdcard", &vfs) == 0) {
        uint64_t total = (uint64_t)vfs.f_blocks * vfs.f_frsize;
        uint64_t avail = (uint64_t)vfs.f_bavail * vfs.f_frsize;
        plog("probe: /sdcard total=%llu MB free=%llu MB\n",
             (unsigned long long)(total >> 20), (unsigned long long)(avail >> 20));
    } else {
        plog("probe: statvfs(/sdcard) failed: %s\n", strerror(errno));
    }
    mkdir("/sdcard/app/logs", 0777);
}

/* ============================ 2) CPU 频率 / 大核判定 ============================ */

static sigjmp_buf g_ill_jmp;
static volatile int g_ill_hit;

static void on_sigill(int sig)
{
    (void)sig;
    g_ill_hit = 1;
    siglongjmp(g_ill_jmp, 1);
}

static uint64_t try_rdcycle(void)
{
    uint64_t v = 0;
    if (sigsetjmp(g_ill_jmp, 1) == 0) {
        __asm__ volatile("rdcycle %0" : "=r"(v));
        return v;
    }
    return 0;   /* U 态不允许读 cycle 时返回 0 */
}

static uint64_t rdtime(void)
{
    uint64_t v = 0;
    __asm__ volatile("rdtime %0" : "=r"(v));
    return v;
}

/* 固定整数工作量，用于横向比较频率（返回值 = 每秒完成的迭代数） */
static uint64_t int_bench_1s(void)
{
    uint64_t iters = 0;
    uint32_t x = 12345;
    uint64_t t0 = mono_us();
    while (mono_us() - t0 < 200000ull) {
        for (int i = 0; i < 10000; ++i) {
            x = x * 1103515245u + 12345u;
            x ^= x >> 13;
        }
        iters += 10000;
    }
    uint64_t dt = mono_us() - t0;
    plog("probe: int loop %llu iters in %llu us, checksum=0x%08x\n",
         (unsigned long long)iters, (unsigned long long)dt, x);
    return dt ? (iters * 1000000ull / dt) : 0;
}

static void probe_cpu(void)
{
    plog("probe: ---- CPU ----\n");

    /* RDTIME 时基频率 */
    uint64_t t0 = rdtime();
    usleep(200000);
    uint64_t t1 = rdtime();
    plog("probe: rdtime delta=%llu over 200ms -> timebase ~%llu Hz\n",
         (unsigned long long)(t1 - t0), (unsigned long long)((t1 - t0) * 5ull));

    /* cycle 频率：rdcycle 在 U 态被禁时会 SIGILL，这里捕获后如实报告 */
    struct sigaction sa, old;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigill;
    sigaction(SIGILL, &sa, &old);

    uint64_t c0 = try_rdcycle();
    if (g_ill_hit) {
        plog("probe: rdcycle 在用户态不可用（SIGILL）-> 用整数基准间接判断\n");
    } else {
        usleep(200000);
        uint64_t c1 = try_rdcycle();
        if (c1 > c0)
            plog("probe: rdcycle delta=%llu over 200ms -> core ~%llu MHz\n",
                 (unsigned long long)(c1 - c0),
                 (unsigned long long)((c1 - c0) * 5ull / 1000000ull));
        else
            plog("probe: rdcycle 返回 0（U 态未开放），用整数基准间接判断\n");
    }
    sigaction(SIGILL, &old, NULL);

    uint64_t ips = int_bench_1s();
    plog("probe: int loop throughput ~%llu iters/s"
         "（参考：1.6GHz 大核应明显高于 800MHz 小核）\n",
         (unsigned long long)ips);
}

/* ============================ 3) RVV ============================ */

#if defined(PROBE_HAVE_RVV)
static void scalar_convert(const uint8_t *y, const uint8_t *uv, uint32_t w, uint32_t h,
                           uint32_t *out)
{
    for (uint32_t i = 0; i < w * h; i += 2) {
        uint32_t uvo = (i >> 1) * 2;
        int u = uv[uvo] - 128;
        int yy = y[i] - 16;
        (void)u;
        out[i] = (uint32_t)(298 * yy + 516 * u + 128) >> 8;
    }
}

static void rvv_convert(const uint8_t *y, const uint8_t *uv, uint32_t w, uint32_t h,
                        uint32_t *out)
{
    uint32_t n = w * h;
    uint32_t i = 0;
    while (i + 16 <= n) {
        size_t vl = vsetvl_e32m4(16);
        if (vl < 16)
            break;
        vuint8m1_t vy = vle8_v_u8m1(y + i, vl);          /* 简化：连续取，仅作吞吐对比 */
        vuint8m1_t vu = vlse8_v_u8m1(uv + i, 2, vl);
        vuint32m4_t wy = vzext_vf4_u32m4(vy, vl);
        vuint32m4_t wu = vzext_vf4_u32m4(vu, vl);
        vint32m4_t sy = vsub_vx_i32m4(vreinterpret_v_u32m4_i32m4(wy), 16, vl);
        vint32m4_t su = vsub_vx_i32m4(vreinterpret_v_u32m4_i32m4(wu), 128, vl);
        vint32m4_t r = vadd_vx_i32m4(vadd_vv_i32m4(vmul_vx_i32m4(sy, 298, vl),
                                                   vmul_vx_i32m4(su, 516, vl), vl),
                                     128, vl);
        r = vsra_vx_i32m4(r, 8, vl);
        vse32_v_i32m4((int32_t *)(out + i), r, vl);
        i += 16;
    }
    for (; i < n; ++i) {
        int u = uv[(i >> 1) * 2] - 128;
        int yy = y[i] - 16;
        out[i] = (uint32_t)((298 * yy + 516 * u + 128) >> 8);
    }
}

static void probe_rvv(void)
{
    plog("probe: ---- RVV ----\n");
    const uint32_t w = 640, h = 480;
    uint8_t *y = (uint8_t *)malloc(w * h);
    uint8_t *uv = (uint8_t *)malloc(w * h / 2);
    uint32_t *out = (uint32_t *)malloc(w * h * sizeof(uint32_t));
    if (!y || !uv || !out) {
        plog("probe: rvv bench malloc failed\n");
        free(y); free(uv); free(out);
        return;
    }
    for (uint32_t i = 0; i < w * h; ++i)
        y[i] = (uint8_t)(i & 0xFF);
    for (uint32_t i = 0; i < w * h / 2; ++i)
        uv[i] = (uint8_t)((i * 7) & 0xFF);

    const int reps = 20;
    uint64_t t0 = mono_us();
    for (int i = 0; i < reps; ++i)
        scalar_convert(y, uv, w, h, out);
    uint64_t t_scalar = mono_us() - t0;

    t0 = mono_us();
    for (int i = 0; i < reps; ++i)
        rvv_convert(y, uv, w, h, out);
    uint64_t t_rvv = mono_us() - t0;

    plog("probe: 640x480 YUV->R 算子 %d 次: scalar=%llu us, RVV=%llu us, speedup=%llux\n",
         reps, (unsigned long long)t_scalar, (unsigned long long)t_rvv,
         (unsigned long long)(t_rvv ? t_scalar / t_rvv : 0));
    plog("probe: RVV 可用（本二进制用 -march=rv64imafdcv 编译）\n");

    free(y); free(uv); free(out);
}
#endif

/* ============================ 4) 设备节点 ============================ */

static void probe_devices(void)
{
    plog("probe: ---- devices ----\n");
    static const char *paths[] = {
        "/dev/gnne_device",   /* KPU 推理设备（中断/poll 等待结果） */
        "/dev/ai_2d_device",  /* AI2D 硬件预处理（resize/pad/色彩空间） */
        "/dev/mem",
        "/dev/vb_device",
        "/dev/vicap_device",
        "/dev/venc_device",
        "/dev/sys_device",
    };
    for (unsigned i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i) {
        int fd = open(paths[i], O_RDWR);
        if (fd >= 0) {
            plog("probe: %-20s open OK\n", paths[i]);
            close(fd);
        } else {
            plog("probe: %-20s open failed: %s\n", paths[i], strerror(errno));
        }
    }
}

/* ============================ 5) PM 频率档位 ============================ */

static void probe_pm(void)
{
    plog("probe: ---- PM (频率档位) ----\n");
    static const char *names[] = { "CPU", "KPU", "DPU", "VPU", "DISPLAY", "MEDIA" };

    for (int d = 0; d < PM_DOMAIN_NR; ++d) {
        int32_t cur = -1;
        int     gret = kd_mpi_pm_get_profile((k_pm_domain)d, &cur);
        plog("probe: domain %-7s get_profile rc=%d cur=%d\n",
             (d < (int)(sizeof(names) / sizeof(names[0]))) ? names[d] : "?", gret, cur);

        k_pm_profile prof[16];
        uint32_t     count = 16;
        memset(prof, 0, sizeof(prof));
        if (kd_mpi_pm_get_profiles((k_pm_domain)d, &count, prof) == 0) {
            for (uint32_t i = 0; i < count && i < 16; ++i)
                plog("probe:   profile[%u] freq=%d Hz volt=%d uV%s\n",
                     i, prof[i].freq, prof[i].volt, ((int32_t)i == cur) ? "  <- current" : "");
        }
    }

    int32_t temp = 0, idx = 0;
    int     rc_th = kd_mpi_pm_get_thermal_protect(PM_DOMAIN_CPU, &temp, &idx);
    if (rc_th == 0)
        plog("probe: thermal protect temp=%d C profile=%d\n", temp, idx);
    else
        plog("probe: thermal protect 查询 rc=%d\n", rc_th);

    int32_t sh = 0;
    if (kd_mpi_pm_get_thermal_shutdown(&sh) == 0)
        plog("probe: thermal shutdown temp=%d C\n", sh);
}

/* ============================ main ============================ */

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    plog("\n==== board_probe start pid=%d ====\n", (int)getpid());
    probe_fs();
    probe_mmz();
    probe_cpu();
#if defined(PROBE_HAVE_RVV)
    probe_rvv();
#else
    plog("probe: 本二进制未启用 RVV（-march 缺 v）\n");
#endif
    probe_devices();
    probe_pm();
    plog("==== board_probe done ====\n");
    return 0;
}
