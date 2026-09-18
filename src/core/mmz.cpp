#include "core/mmz.hpp"

#include "mpi_sys_api.h"

namespace dart {
namespace {

constexpr uint32_t kChunkMb = 1;
constexpr uint32_t kMaxChunks = 512; // 上限保护：别把系统打满

} // namespace

uint32_t mmz_free_mb() {
    k_u64 phys[kMaxChunks];
    void *virt[kMaxChunks];

    uint32_t n = 0;
    for (; n < kMaxChunks; ++n) {
        if (kd_mpi_sys_mmz_alloc(&phys[n], &virt[n], "sgd_probe", nullptr, kChunkMb << 20) != K_SUCCESS ||
            virt[n] == nullptr) {
            break;
        }
    }
    for (uint32_t i = 0; i < n; ++i)
        kd_mpi_sys_mmz_free(phys[i], virt[i]);
    return n * kChunkMb;
}

uint32_t mmz_max_block_mb() {
    static const uint32_t steps[] = {1, 2, 4, 8, 16, 24, 32, 48, 64};
    uint32_t best = 0;

    for (uint32_t mb : steps) {
        k_u64 phys = 0;
        void *virt = nullptr;
        if (kd_mpi_sys_mmz_alloc(&phys, &virt, "sgd_probe", nullptr, mb << 20) != K_SUCCESS ||
            virt == nullptr) {
            break;
        }
        best = mb;
        kd_mpi_sys_mmz_free(phys, virt);
    }
    return best;
}

} // namespace dart
