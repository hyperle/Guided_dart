// 主机侧 MPP 桩的实现。见 stub_mpp.hpp 的不变量清单。
//
// MMZ 模型：每个 VB 池 = 一次独立的对齐分配，phys = 0x10000000 + 池序号*16MB + 块偏移。
// 于是"越界写块的末尾"会撞上 ASan 红区（池的分配正好等于 blk_cnt*blk_size），
// 这正是我们最想抓的那类 bug（标签/拷贝越长、步距算错）。
#include "stub_mpp.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <vector>

#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_venc_api.h"

namespace stub {
namespace {

constexpr k_u64 kPhysBase = 0x10000000ull;
constexpr k_u64 kPhysStride = 16ull << 20;

struct Blk {
    k_u64 phys = 0;
    bool  busy = false;  // 应用/模块持有
    int   enc_ref = 0;   // 编码器持有
};

struct Pool {
    k_u64             base = 0;
    uint8_t          *mem = nullptr;
    k_u32             blk_size = 0;
    k_u32             blk_cnt = 0;
    std::vector<Blk>  blks;
};

std::map<uint32_t, Pool> g_pools;
uint32_t                 g_next_pool = 1;
uint64_t                 g_pool_seq = 0;

struct Mapping {
    void  *va = nullptr;
    k_u32  size = 0;
};
std::map<k_u64, Mapping> g_maps;
std::map<void *, k_u64>  g_va_phys;

struct Job {
    uint32_t out_pool = 0;
    uint32_t out_idx = 0;
    uint32_t in_pool = 0;
    uint32_t in_idx = 0;
    uint64_t seq = 0;
};
std::vector<Job>             g_jobs;
std::map<uint32_t, uint32_t> g_attach;
bool     g_started = false;
bool     g_header_done = false;
uint32_t g_get_calls = 0;
uint32_t g_send_seq = 0;
Job      g_stream_job{};
bool     g_stream_active = false;

Config                   g_cfg;
Stats                    g_stats;
std::vector<std::string> g_trace;
int                      g_fail = 0;

void fail(const char *what) {
    std::fprintf(stderr, "  [桩断言失败] %s\n", what);
    ++g_fail;
}

uint32_t handle_make(uint32_t pool, uint32_t idx) { return (pool << 8) | idx; }
uint32_t handle_pool(uint32_t h) { return h >> 8; }
uint32_t handle_idx(uint32_t h) { return h & 0xffu; }
bool     handle_valid(uint32_t h) { return h != VB_INVALID_HANDLE && h != 0 && g_pools.count(handle_pool(h)) != 0; }

Blk *blk_of(uint32_t h) {
    if (!handle_valid(h))
        return nullptr;
    Pool &p = g_pools[handle_pool(h)];
    const uint32_t i = handle_idx(h);
    return i < p.blks.size() ? &p.blks[i] : nullptr;
}

Pool *pool_of_phys(k_u64 phys) {
    for (auto &kv : g_pools) {
        Pool &p = kv.second;
        if (phys >= p.base && phys < p.base + static_cast<k_u64>(p.blk_size) * p.blk_cnt)
            return &p;
    }
    return nullptr;
}

Blk *blk_by_phys(k_u64 phys, Pool **pool_out = nullptr) {
    Pool *p = pool_of_phys(phys);
    if (p == nullptr)
        return nullptr;
    if (pool_out)
        *pool_out = p;
    const k_u64 off = phys - p->base;
    return &p->blks[static_cast<size_t>(off / p->blk_size)];
}

uint8_t *host_ptr(k_u64 phys) {
    Pool *p = pool_of_phys(phys);
    return p != nullptr ? p->mem + (phys - p->base) : nullptr;
}

uint32_t take_free_out_block(uint32_t pool) {
    auto it = g_pools.find(pool);
    if (it == g_pools.end())
        return VB_INVALID_HANDLE;
    for (uint32_t i = 0; i < it->second.blks.size(); ++i) {
        if (!it->second.blks[i].busy) {
            it->second.blks[i].busy = true;
            return handle_make(pool, i);
        }
    }
    return VB_INVALID_HANDLE; // 输出池没空块：真 SDK 这里编码器就卡住了
}

void write_pack_bytes(uint8_t *dst, uint32_t len, uint8_t kind) {
    static const uint8_t sc[4] = {0x00, 0x00, 0x00, 0x01};
    std::memcpy(dst, sc, 4); // 伪 H.264：起始码 + 可识别的 NAL 类型字节
    for (uint32_t i = 4; i < len; ++i)
        dst[i] = kind;
}

} // namespace

void reset(const Config &cfg) {
    for (auto &kv : g_pools)
        std::free(kv.second.mem);
    g_pools.clear();
    g_next_pool = 1;
    g_pool_seq = 0;
    g_maps.clear();
    g_va_phys.clear();
    g_jobs.clear();
    g_attach.clear();
    g_started = false;
    g_header_done = false;
    g_get_calls = 0;
    g_send_seq = 0;
    g_stream_active = false;
    g_cfg = cfg;
    g_stats = Stats{};
    g_trace.clear();
    g_fail = 0;
}

const Stats &stats() { return g_stats; }
std::vector<std::string> &call_trace() { return g_trace; }
void check_all(int *failures) { *failures += g_fail; }

} // namespace stub

// ============================================================================
// 与 SDK 头文件声明一致的 C 链接桩
// ============================================================================

k_s32 kd_mpi_vb_create_pool(k_vb_pool_config *config) {
    if (config == nullptr || config->blk_size == 0 || config->blk_cnt == 0)
        return static_cast<k_s32>(VB_INVALID_POOLID);

    stub::Pool p;
    p.blk_size = static_cast<k_u32>(config->blk_size);
    p.blk_cnt = static_cast<k_u32>(config->blk_cnt);
    p.base = stub::kPhysBase + (stub::g_pool_seq++) * stub::kPhysStride;
    p.mem = static_cast<uint8_t *>(std::aligned_alloc(4096, static_cast<size_t>(p.blk_size) * p.blk_cnt));
    if (p.mem == nullptr)
        return static_cast<k_s32>(VB_INVALID_POOLID);

    for (k_u32 i = 0; i < p.blk_cnt; ++i) {
        stub::Blk b;
        b.phys = p.base + static_cast<k_u64>(i) * p.blk_size;
        p.blks.push_back(b);
    }
    const uint32_t id = stub::g_next_pool++;
    stub::g_pools[id] = p;
    return static_cast<k_s32>(id);
}

k_vb_blk_handle kd_mpi_vb_get_block(k_u32 pool_id, k_u64 blk_size, const k_char *mmz_name) {
    (void)mmz_name;
    auto it = stub::g_pools.find(pool_id);
    if (it == stub::g_pools.end() || blk_size == 0 || blk_size > it->second.blk_size)
        return VB_INVALID_HANDLE;
    for (uint32_t i = 0; i < it->second.blks.size(); ++i) {
        stub::Blk &b = it->second.blks[i];
        if (!b.busy && b.enc_ref == 0) {
            b.busy = true;
            ++stub::g_stats.get_block;
            ++stub::g_stats.blocks_allocated;
            return stub::handle_make(pool_id, i);
        }
    }
    return VB_INVALID_HANDLE; // 池里没有空块：真 SDK 立刻失败返回，不等待
}

k_s32 kd_mpi_vb_release_block(k_vb_blk_handle block) {
    stub::Blk *b = stub::blk_of(block);
    if (b == nullptr)
        return K_FAILED;
    if (!b->busy)
        stub::fail("release_block 作用在已空闲的块上（引用计数被打崩）");
    b->busy = false;
    ++stub::g_stats.release_block;
    return K_SUCCESS;
}

k_u64 kd_mpi_vb_handle_to_phyaddr(k_vb_blk_handle block) {
    stub::Blk *b = stub::blk_of(block);
    return b != nullptr ? b->phys : 0;
}

// 真 SDK 的语义：物理地址只要落在某个块里就返回那个块的 handle
// （内核实现里的断言是 pool_phys_addr <= phys_addr < pool_phys_addr + pool_size）。
// 上一版这里写成"必须等于块基址"，于是 Recorder 的归一化永远失败、退回逐包映射 ——
// 正好把要测的那条修复路径绕过去了。
k_vb_blk_handle kd_mpi_vb_phyaddr_to_handle(k_u64 phys_addr) {
    stub::Pool *p = nullptr;
    stub::Blk *b = stub::blk_by_phys(phys_addr, &p);
    if (b == nullptr || p == nullptr)
        return VB_INVALID_HANDLE;
    for (auto &kv : stub::g_pools)
        if (&kv.second == p)
            return stub::handle_make(kv.first, static_cast<uint32_t>(b - &p->blks[0]));
    return VB_INVALID_HANDLE;
}

k_s32 kd_mpi_vb_destory_pool(k_u32 pool_id) {
    auto it = stub::g_pools.find(pool_id);
    if (it == stub::g_pools.end())
        return K_FAILED;
    for (const auto &b : it->second.blks)
        if (b.busy || b.enc_ref > 0)
            stub::fail("销毁 VB 池时池里还有块被占着（应用/编码器没还引用）");
    std::free(it->second.mem);
    stub::g_pools.erase(it);
    return K_SUCCESS;
}

k_s32 kd_mpi_vb_set_config(const k_vb_config *config) { (void)config; return K_SUCCESS; }
k_s32 kd_mpi_vb_init(void) { return K_SUCCESS; }
k_s32 kd_mpi_vb_exit(void) { return K_SUCCESS; }
k_s32 kd_mpi_vb_get_config(k_vb_config *config) { (void)config; return K_SUCCESS; }

// ---------------------------------------------------------------- sys
namespace stub {
namespace {
void *mmap_impl(k_u64 phy_addr, k_u32 size) {
    uint8_t *host = host_ptr(phy_addr);
    if (host == nullptr || pool_of_phys(phy_addr + size - 1) != pool_of_phys(phy_addr)) {
        fail("mmap 跨越/落在已知 VB 池之外（地址或长度非法）");
        return nullptr;
    }
    auto it = g_maps.find(phy_addr);
    if (it != g_maps.end())
        return it->second.va;
    g_maps[phy_addr] = Mapping{host, size};
    g_va_phys[host] = phy_addr;
    return host;
}
} // namespace
} // namespace stub

void *kd_mpi_sys_mmap(k_u64 phy_addr, k_u32 size) {
    ++stub::g_stats.mmap;
    return stub::mmap_impl(phy_addr, size);
}

void *kd_mpi_sys_mmap_cached(k_u64 phy_addr, k_u32 size) {
    ++stub::g_stats.mmap_cached;
    return stub::mmap_impl(phy_addr, size);
}

k_s32 kd_mpi_sys_munmap(void *virt_addr, k_u32 size) {
    ++stub::g_stats.munmap;
    auto it = stub::g_va_phys.find(virt_addr);
    if (it == stub::g_va_phys.end()) {
        std::fprintf(stderr, "  [桩] munmap va=%p size=%u，当前活跃映射 %zu 条\n", virt_addr, size,
                     stub::g_maps.size());
        for (auto &kv : stub::g_va_phys)
            std::fprintf(stderr, "       活映射 va=%p phys=0x%llx\n", kv.first,
                         (unsigned long long)kv.second);
        stub::fail("munmap 作用在没映射过（或已解除）的 VA 上");
        return K_FAILED;
    }
    if (stub::g_maps[it->second].size != size)
        stub::fail("munmap 长度与 mmap 时不一致（老工程实测会打挂内核）");
    stub::g_maps.erase(it->second);
    stub::g_va_phys.erase(it);
    return K_SUCCESS;
}

// 假 MMZ 预算：mmz_free_mb() 逐块申请时用它记账（真实板子上这是唯一的内存来源）
namespace stub {
namespace {
constexpr uint32_t kFakeMmzMb = 24;
uint32_t g_mmz_used_mb = 0;
} // namespace
} // namespace stub

k_s32 kd_mpi_sys_mmz_alloc(k_u64 *phy_addr, void **virt_addr, const k_char *mmb, const k_char *zone,
                          k_u32 len) {
    (void)mmb;
    (void)zone;
    const uint32_t mb = (len + (1u << 20) - 1) >> 20;
    if (mb == 0 || stub::g_mmz_used_mb + mb > stub::kFakeMmzMb)
        return K_FAILED;
    void *p = std::malloc(len);
    if (p == nullptr)
        return K_FAILED;
    stub::g_mmz_used_mb += mb;
    if (virt_addr)
        *virt_addr = p;
    if (phy_addr)
        *phy_addr = stub::kPhysBase + stub::g_pool_seq * stub::kPhysStride + (mb << 20);
    return K_SUCCESS;
}

k_s32 kd_mpi_sys_mmz_free(k_u64 phy_addr, void *virt_addr) {
    (void)phy_addr;
    if (virt_addr != nullptr)
        std::free(virt_addr);
    if (stub::g_mmz_used_mb > 0)
        --stub::g_mmz_used_mb; // 按 1MB 块记账，够测试用了
    return K_SUCCESS;
}

k_s32 kd_mpi_sys_mmz_invalidate_cache(k_u64 phy_addr, void *virt_addr, k_u32 size) {
    (void)phy_addr;
    (void)virt_addr;
    (void)size;
    ++stub::g_stats.invalidate;
    return K_SUCCESS;
}

k_s32 kd_mpi_sys_mmz_flush_cache(k_u64 phy_addr, void *virt_addr, k_u32 size) {
    (void)phy_addr;
    (void)virt_addr;
    (void)size;
    return K_SUCCESS;
}

// ---------------------------------------------------------------- venc
// 头文件里 kd_mpi_venc_attach_vb_pool 是 static inline，钩它调用的 _ex 版本
k_s32 kd_mpi_venc_attach_vb_pool_ex(k_u32 chn_num, k_u32 pool_id, k_s32 buff_num) {
    (void)buff_num;
    stub::g_attach[chn_num] = pool_id;
    stub::g_trace.push_back("attach_vb_pool");
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_detach_vb_pool(k_u32 chn_num) {
    stub::g_trace.push_back("detach_vb_pool");
    stub::g_attach.erase(chn_num);
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_create_chn(k_u32 chn_num, k_venc_chn_attr *attr) {
    (void)chn_num;
    if (attr == nullptr || attr->venc_attr.pic_width == 0 || attr->venc_attr.pic_height == 0)
        return K_FAILED;
    if (stub::g_cfg.create_chn_fails)
        return K_FAILED;
    stub::g_trace.push_back("create_chn");
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_start_chn(k_u32 chn_num) {
    (void)chn_num;
    stub::g_started = true;
    stub::g_trace.push_back("start_chn");
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_stop_chn(k_u32 chn_num) {
    (void)chn_num;
    stub::g_started = false;
    stub::g_trace.push_back("stop_chn");
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_destroy_chn(k_u32 chn_num) {
    (void)chn_num;
    stub::g_trace.push_back("destroy_chn");
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_send_frame(k_u32 chn_num, k_video_frame_info *frame, k_s32 milli_sec) {
    (void)milli_sec;
    if (!stub::g_started || frame == nullptr) {
        ++stub::g_stats.send_fail;
        return K_FAILED;
    }

    stub::Pool *in_pool = nullptr;
    stub::Blk *in = stub::blk_by_phys(frame->v_frame.phys_addr[0], &in_pool);
    if (in == nullptr || in_pool == nullptr) {
        stub::fail("送帧的物理地址不在任何 VB 块里");
        ++stub::g_stats.send_fail;
        return K_FAILED;
    }
    if (stub::g_pools.find(frame->pool_id) == stub::g_pools.end() ||
        &stub::g_pools[frame->pool_id] != in_pool) {
        stub::fail("送帧的 pool_id 与物理地址所属池不一致");
        ++stub::g_stats.send_fail;
        return K_FAILED;
    }
    const auto pool_id_of = [&] {
        for (auto &kv : stub::g_pools)
            if (&kv.second == in_pool)
                return kv.first;
        return 0u;
    }();
    const uint32_t in_idx = static_cast<uint32_t>((frame->v_frame.phys_addr[0] - in_pool->base) / in_pool->blk_size);
    if (frame->v_frame.width == 0 || frame->v_frame.height == 0 ||
        frame->v_frame.stride[0] < frame->v_frame.width) {
        stub::fail("送帧的帧描述非法");
        ++stub::g_stats.send_fail;
        return K_FAILED;
    }

    // 这一块**必须不在在途队列里**：复用未回收的块 = 编码器正在读被覆写的内存
    for (const auto &j : stub::g_jobs)
        if (j.in_pool == pool_id_of && j.in_idx == in_idx)
            stub::fail("输入块在编码器还没吐码流时就被复用（在途块被覆写）");

    if (stub::g_cfg.send_fails) {
        ++stub::g_stats.send_fail;
        return K_FAILED; // 模拟编码器永不接收
    }

    auto at = stub::g_attach.find(chn_num);
    if (at == stub::g_attach.end()) {
        stub::fail("送帧时通道没挂输出池");
        ++stub::g_stats.send_fail;
        return K_FAILED;
    }
    const uint32_t ob = stub::take_free_out_block(at->second);
    if (ob == VB_INVALID_HANDLE) {
        ++stub::g_stats.send_fail;
        return K_FAILED; // 输出池没空块
    }

    stub::Job j;
    j.out_pool = stub::handle_pool(ob);
    j.out_idx = stub::handle_idx(ob);
    j.in_pool = pool_id_of;
    j.in_idx = in_idx;
    j.seq = ++stub::g_send_seq;
    in->enc_ref++; // 编码器持引用，直到它的码流被 release_stream
    stub::g_jobs.push_back(j);
    ++stub::g_stats.send_ok;
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_get_stream(k_u32 chn_num, k_venc_stream *stream, k_s32 milli_sec) {
    (void)milli_sec;
    if (stream == nullptr || stream->pack == nullptr || stream->pack_cnt == 0)
        return K_FAILED;
    stream->pack_cnt = 0;
    if (!stub::g_started)
        return K_FAILED;

    ++stub::g_get_calls;

    // 参数集只在码流开始时出现一次（老工程实测），header_delay 控制第几次取流才给
    if (!stub::g_header_done && stub::g_get_calls > stub::g_cfg.header_delay) {
        auto at = stub::g_attach.find(chn_num);
        const uint32_t ob = stub::take_free_out_block(at->second);
        if (ob == VB_INVALID_HANDLE)
            return K_FAILED;
        stub::g_stream_job = stub::Job{};
        stub::g_stream_job.out_pool = stub::handle_pool(ob);
        stub::g_stream_job.out_idx = stub::handle_idx(ob);
        stub::g_stream_job.in_pool = 0;
        stub::g_stream_job.in_idx = 0xffffffffu;
        stub::g_stream_active = true;

        const k_u64 phys = stub::g_pools[stub::handle_pool(ob)].blks[stub::handle_idx(ob)].phys + 0x100;
        stub::write_pack_bytes(stub::host_ptr(phys), 32, 0x67); // SPS
        stream->pack[0].phys_addr = phys;
        stream->pack[0].len = 32;
        stream->pack[0].type = K_VENC_HEADER;
        stream->pack_cnt = 1;
        stub::g_header_done = true;
        ++stub::g_stats.header_packs;
        return K_SUCCESS;
    }

    if (stub::g_jobs.empty())
        return K_FAILED;

    const stub::Job j = stub::g_jobs.front();
    stub::g_jobs.erase(stub::g_jobs.begin());

    // 模拟编码器静默丢帧：每 stream_every 帧才吐一个包，其余直接吞掉
    if (stub::g_cfg.stream_every > 1 && (j.seq % stub::g_cfg.stream_every) != 0) {
        stub::g_pools[j.out_pool].blks[j.out_idx].busy = false;
        stub::g_pools[j.in_pool].blks[j.in_idx].enc_ref--;
        return K_FAILED;
    }

    const k_u64 phys = stub::g_pools[j.out_pool].blks[j.out_idx].phys + 0x100;
    const uint8_t kind = (j.seq % 5 == 0) ? 0x65 : 0x41; // 每 5 帧一个 I 帧
    stub::write_pack_bytes(stub::host_ptr(phys), 64, kind);
    stream->pack[0].phys_addr = phys;
    stream->pack[0].len = 64;
    stream->pack[0].type = (kind == 0x65) ? K_VENC_I_FRAME : K_VENC_P_FRAME;
    stream->pack_cnt = 1;

    stub::g_stream_job = j;
    stub::g_stream_active = true;
    ++stub::g_stats.streams;
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_set_intbuf_size(k_u32 chn_num, k_u32 size) {
    (void)chn_num;
    if (size == 0)
        return K_FAILED;
    stub::g_trace.push_back("set_intbuf_size");
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_query_status(k_u32 chn_num, k_venc_chn_status *status) {
    (void)chn_num;
    if (status == nullptr)
        return K_FAILED;
    std::memset(status, 0, sizeof(*status));
    // 在途任务数就是"编码器里攒着几帧"，峰值的码流统计按已产出的包数给
    status->cur_packs = static_cast<k_u32>(stub::g_jobs.size());
    status->end_of_stream = K_FALSE;
    status->stream_info.u32PicCnt = static_cast<k_u32>(stub::g_stats.streams);
    status->stream_info.u32PicBytesNum = static_cast<k_u32>(stub::g_stats.streams * 64);
    return K_SUCCESS;
}

k_s32 kd_mpi_venc_release_stream(k_u32 chn_num, k_venc_stream *stream) {
    (void)chn_num;
    (void)stream;
    if (!stub::g_stream_active)
        return K_FAILED;
    stub::g_stream_active = false;

    stub::Job &j = stub::g_stream_job;
    if (j.in_idx != 0xffffffffu) {
        stub::Blk &in = stub::g_pools[j.in_pool].blks[j.in_idx];
        in.enc_ref--;
        if (in.enc_ref < 0)
            stub::fail("输入块引用计数被减到负数");
    }
    auto it = stub::g_pools.find(j.out_pool);
    if (it != stub::g_pools.end() && j.out_idx < it->second.blks.size())
        it->second.blks[j.out_idx].busy = false; // 码流取走后输出块归还
    return K_SUCCESS;
}
