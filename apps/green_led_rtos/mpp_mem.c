#include "mpp_mem.h"

#include <pthread.h>

static pthread_mutex_t g_mem_lock = PTHREAD_MUTEX_INITIALIZER;
static uint64_t g_maps;
static uint64_t g_contended;

void mpp_mem_begin(void)
{
    if (pthread_mutex_trylock(&g_mem_lock) != 0) {
        g_contended++;          /* 记一笔：真的发生过多线程抢映射 */
        pthread_mutex_lock(&g_mem_lock);
    }
}

void mpp_mem_end(void)
{
    pthread_mutex_unlock(&g_mem_lock);
}

void *mpp_mem_map(k_u64 phys, k_u32 size)
{
    g_maps++;
    return kd_mpi_sys_mmap(phys, size);
}

void mpp_mem_unmap(void *va, k_u32 size)
{
    if (va)
        kd_mpi_sys_munmap(va, size);
}

void mpp_mem_stats(uint64_t *maps, uint64_t *contended)
{
    if (maps)
        *maps = g_maps;
    if (contended)
        *contended = g_contended;
}
