#include "core/mpp_map.hpp"

#include <cstdint>
#include <mutex>

#include "core/log.hpp"

#include "mpi_sys_api.h"

namespace dart {
namespace {

constexpr uint32_t kMaxMaps = 256; // VICAP 环形缓冲 + 录像输入块 + VENC 输出池块，几十条足够

struct MapEntry {
    k_u64    phys = 0;
    k_u32    size = 0;
    uint8_t *va = nullptr;
    bool     cached = false;
};

MapEntry   g_maps[kMaxMaps];
uint32_t   g_used = 0;
std::mutex g_mtx; // 主线程与录像线程都会调用 → 必须串行（这正是打挂内核的那个并发点）

void *map_common(k_u64 phys, k_u32 size, bool cached) {
    if (phys == 0 || size == 0)
        return nullptr;

    std::lock_guard<std::mutex> lk(g_mtx);

    // 同一物理块可能被以不同长度请求（码流包长度各不相同）：覆盖得住就直接复用。
    // 覆盖不住就为更大的长度另建一条映射 —— 映射只增不减，多一条映射是安全的；
    // 打挂内核的是「并发 map/unmap + 长度不匹配的 munmap」，不是这里的多映射。
    for (uint32_t i = 0; i < g_used; ++i)
        if (g_maps[i].phys == phys && g_maps[i].cached == cached && g_maps[i].size >= size)
            return g_maps[i].va;

    if (g_used >= kMaxMaps) {
        log_line("映射: 缓存满(%u)，本次映射失败 phys=0x%llx\n", g_used,
                 static_cast<unsigned long long>(phys));
        return nullptr;
    }

    void *va = cached ? kd_mpi_sys_mmap_cached(phys, size) : kd_mpi_sys_mmap(phys, size);
    if (va == nullptr) {
        log_line("映射: mmap%s 失败 phys=0x%llx size=%u\n", cached ? "_cached" : "",
                 static_cast<unsigned long long>(phys), size);
        return nullptr;
    }

    g_maps[g_used].phys = phys;
    g_maps[g_used].size = size;
    g_maps[g_used].va = static_cast<uint8_t *>(va);
    g_maps[g_used].cached = cached;
    ++g_used;
    return va;
}

} // namespace

void *mpp_map_persist(k_u64 phys, k_u32 size) { return map_common(phys, size, false); }

void *mpp_map_persist_cached(k_u64 phys, k_u32 size) { return map_common(phys, size, true); }

void mpp_invalidate(k_u64 phys, void *va, k_u32 size) {
    if (phys == 0 || va == nullptr || size == 0)
        return;
    kd_mpi_sys_mmz_invalidate_cache(phys, va, size);
}

void mpp_map_shutdown() {
    std::lock_guard<std::mutex> lk(g_mtx);
    for (uint32_t i = 0; i < g_used; ++i)
        if (g_maps[i].va != nullptr)
            kd_mpi_sys_munmap(g_maps[i].va, g_maps[i].size); // 必须按 map 时的长度
    g_used = 0;
}

} // namespace dart
