// ============================================================================
// Recorder 主机侧回归测试（x86 上跑真代码 + 桩 MPP）
//
// 盯的是上板才会暴露、而且一暴露就是"录像静默停止"的那几条：
//   A 正常编码器（参数集晚到）：送帧=码流包 1:1，块不泄漏、运行期不再 get_block、
//     运行期零 mmap/munmap、文件里 SPS 在第一个 I 帧之前且每个 I 帧前重发
//   B 编码器永不接收：录像线程不许被钉死，块全部收回，停止干净
//   C 编码器每 3 帧只吐 1 包：在途块不许被复用（桩里硬断言），丢帧计数上涨但账目平衡
//   D 编码器彻底静默：2 秒后强收最老在途块，录像不永久停摆
//
// 编译运行: bash tests/host/run.sh
// ============================================================================
#include <cstdio>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "core/config.hpp"
#include "core/mpp_map.hpp"
#include "record/recorder.hpp"
#include "evidence/evidence_writer.hpp"
#include "vision/frame_pool.hpp"

#include "stub_mpp.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const std::string &what) {
    if (!ok) {
        std::fprintf(stderr, "  ✗ %s\n", what.c_str());
        ++g_failures;
    }
}

// 从 .h264 里按起始码切出 NAL 类型序列，验证参数集位置
bool h264_types(const std::string &path, std::vector<int> *types, long *size) {
    std::FILE *f = std::fopen(path.c_str(), "rb");
    if (f == nullptr)
        return false;
    std::vector<uint8_t> buf;
    uint8_t chunk[4096];
    size_t n = 0;
    while ((n = std::fread(chunk, 1, sizeof(chunk), f)) > 0)
        buf.insert(buf.end(), chunk, chunk + n);
    std::fclose(f);
    *size = static_cast<long>(buf.size());

    for (size_t i = 0; i + 4 < buf.size(); ++i) {
        if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 0 && buf[i + 3] == 1) {
            types->push_back(buf[i + 4]);
            i += 4;
        }
    }
    return true;
}

std::string out_dir_for(const char *name) {
    std::string d = std::string("/tmp/sgd_host_test/") + name;
    std::string cmd = "rm -rf " + d + " && mkdir -p " + d;
    (void)!std::system(cmd.c_str());
    return d;
}

struct Result {
    bool     started = false;
    uint64_t frames = 0, send_fail = 0, blk_empty = 0, dropped = 0, reclaimed = 0, io_fail = 0;
    uint32_t inflight = 0;
    std::string file;
    long         file_size = 0;
};

void check_teardown_order();

Result run_scenario(const char *name, const stub::Config &scfg, uint32_t nframes, uint32_t submit_us,
                    bool venc_probe = false) {
    stub::reset(scfg);

    dart::Config cfg;
    cfg.width = 640;
    cfg.height = 360;
    cfg.sensor_fps = 30;
    cfg.frame_slots = 16;
    const std::string dir = out_dir_for(name);
    cfg.out_dir = dir.c_str();
    cfg.venc_probe = venc_probe;

    Result r;

    {
        dart::FramePool pool(cfg.width, cfg.height, cfg.frame_slots);
        dart::Recorder  recorder(cfg, pool);

        r.started = recorder.start();

        const stub::Stats after_start = stub::stats();

        // 运行期不许再 get_block / mmap / munmap —— 这是本轮修复的核心。
        // 计数在循环里只累计最大值，循环外断言一次（避免刷屏）。
        uint64_t max_gb = after_start.get_block;
        uint64_t max_mm = after_start.mmap;
        uint64_t max_mmc = after_start.mmap_cached;
        uint64_t max_mu = after_start.munmap;
        uint32_t max_inflight = 0;

        for (uint32_t i = 0; i < nframes; ++i) {
            dart::Frame *f = pool.acquire();
            if (f == nullptr) { // 录像还攥着池帧：丢这一拍（真主循环同款行为）
                usleep(200);
                --i;
                continue;
            }
            // 填 Y 平面（模拟二值化结果）
            for (uint32_t y = 0; y < f->view.height; ++y)
                std::memset(f->view.pixels + static_cast<size_t>(y) * f->view.stride, (y + i) & 0xff,
                            f->view.width);
            f->seq = i + 1;
            f->mono_ms = i * 11;
            f->result.cx = 320;
            f->result.cy = 180;
            f->result.cost_us = 900;
            recorder.submit(f);

            if (submit_us)
                usleep(submit_us);

            const stub::Stats &s = stub::stats();
            max_gb = s.get_block;
            max_mm = s.mmap;
            max_mmc = s.mmap_cached;
            max_mu = s.munmap;
            if (recorder.inflight() > max_inflight)
                max_inflight = recorder.inflight();
        }

        check(max_gb == after_start.get_block, "运行期发生了 get_block（块应在启动时一次取满）");
        check(max_mm == after_start.mmap, "运行期发生了 mmap（应整块持久映射）");
        check(max_mu == after_start.munmap, "运行期发生了 munmap（老工程被打挂的那条路）");
        check(max_mmc <= after_start.mmap_cached + 8, "cache 映射数超过输出池块数");
        check(max_inflight <= 6, "在途块超过了输入块总数（账目错乱）");

        recorder.stop();

        r.frames = recorder.frames();
        r.send_fail = recorder.send_fail();
        r.blk_empty = recorder.blk_empty();
        r.dropped = recorder.dropped();
        r.reclaimed = recorder.reclaimed();
        r.io_fail = recorder.io_fail();
        r.inflight = recorder.inflight();

        check(r.inflight == 0, "停止后在途块没清零");
        if (r.started)
            check_teardown_order();

        // 完全释放后再解映射（与 main.cpp 的顺序一致）
    }
    dart::mpp_map_shutdown();

    std::string path = dir + "/rec_0001.h264";
    std::vector<int> types;
    if (h264_types(path, &types, &r.file_size))
        r.file = path;
    return r;
}

void check_teardown_order() {
    const std::vector<std::string> &tr = stub::call_trace();
    int stop = -1, destroy = -1, detach = -1;
    for (size_t i = 0; i < tr.size(); ++i) {
        if (tr[i] == "stop_chn" && stop < 0)
            stop = static_cast<int>(i);
        else if (tr[i] == "destroy_chn" && destroy < 0)
            destroy = static_cast<int>(i);
        else if (tr[i] == "detach_vb_pool" && detach < 0)
            detach = static_cast<int>(i);
    }
    check(stop >= 0 && destroy > stop && detach > destroy,
          "拆机顺序应为 stop_chn → destroy_chn → detach_vb_pool（上一版漏了 detach）");
}

void check_file_ordering(const std::string &path, long size, const char *tag) {
    std::vector<int> types;
    long n = 0;
    if (!h264_types(path, &types, &n)) {
        check(false, std::string(tag) + ": 文件打不开 " + path);
        return;
    }
    check(!types.empty() && types[0] == 0x67, std::string(tag) + ": 文件开头不是 SPS（参数集）");
    bool seen_hdr = false;
    int  idrs = 0;
    for (int t : types) {
        if (t == 0x67)
            seen_hdr = true;
        else if (t == 0x65) {
            ++idrs;
            check(seen_hdr, std::string(tag) + ": I 帧之前没有参数集");
        }
    }
    check(idrs > 0, std::string(tag) + ": 一个 I 帧都没有");
    check(size > 0, std::string(tag) + ": 文件是空的");
}

} // namespace

int main() {
    std::printf("=== Recorder 主机侧回归 ===\n");

    // ---------------- A 正常（参数集晚到一次）----------------
    {
        stub::Config sc;
        sc.header_delay = 1; // start() 那次取流拿不到参数集，运行期才补到
        Result r = run_scenario("A_normal", sc, 300, 1500);
        std::printf("A 正常: 送帧 %llu 码流包应有 %llu, 失败 %llu 无块 %llu 队列丢 %llu 强收 %llu IO错 %llu 文件 %ld 字节\n",
                    (unsigned long long)r.frames, (unsigned long long)stub::stats().streams,
                    (unsigned long long)r.send_fail, (unsigned long long)r.blk_empty,
                    (unsigned long long)r.dropped, (unsigned long long)r.reclaimed,
                    (unsigned long long)r.io_fail, r.file_size);
        check(r.frames > 0, "A: 一帧都没录上");
        check(r.send_fail == 0, "A: 出现送帧失败");
        check(r.blk_empty == 0, "A: 出现无块可用");
        check(r.io_fail == 0, "A: 出现写文件错误");
        check(r.reclaimed == 0, "A: 触发了强制回收");
        check(r.frames == stub::stats().streams, "A: 送帧数与码流包数不是 1:1");
        check_file_ordering(r.file, r.file_size, "A");
        check(stub::stats().munmap == stub::stats().mmap + stub::stats().mmap_cached,
              "A: 映射与解映射数量不匹配");
        check(stub::stats().release_block >= 22, "A: 输入块/帧池块没有全部 release");
    }

    // ---------------- B 编码器永不接收 ----------------
    {
        stub::Config sc;
        sc.send_fails = true;
        Result r = run_scenario("B_never_accepts", sc, 60, 1000);
        std::printf("B 永不接收: 送帧成功 %llu 失败 %llu 无块 %llu 强收 %llu\n",
                    (unsigned long long)r.frames, (unsigned long long)r.send_fail,
                    (unsigned long long)r.blk_empty, (unsigned long long)r.reclaimed);
        check(r.frames == 0, "B: 编码器没接收却记账成功");
        check(r.send_fail > 0, "B: 送帧失败没计数");
        check(r.blk_empty == 0, "B: 送帧失败时块没立刻收回（会导致 6 帧后彻底没块）");
    }

    // ---------------- C 编码器每 3 帧只吐 1 包 ----------------
    {
        stub::Config sc;
        sc.stream_every = 3;                                 // 每 3 帧只吐 1 包
        Result r = run_scenario("C_encoder_drops", sc, 900, 3000); // 跑满 2.7 秒，跨过强收阈值
        std::printf("C 编码器丢帧: 送帧 %llu 失败 %llu 无块 %llu 强收 %llu 文件 %ld 字节\n",
                    (unsigned long long)r.frames, (unsigned long long)r.send_fail,
                    (unsigned long long)r.blk_empty, (unsigned long long)r.reclaimed, r.file_size);
        check(r.blk_empty > 0, "C: 编码器丢帧时居然没有无块可用（在途块回收逻辑可疑）");
        check(r.reclaimed > 0, "C: 编码器丢帧后没有强收兜底，块会被永久占住");
        check(r.frames > 20, "C: 录像实际上停摆了（强收没把块救回来）");
        check_file_ordering(r.file, r.file_size, "C");
    }

    // ---------------- D 编码器彻底静默 → 强收兜底 ----------------
    {
        stub::Config sc;
        sc.stream_every = 1000000; // 永远不吐视频包
        Result r = run_scenario("D_silent_encoder", sc, 200, 20000); // 慢喂，给强收留 2 秒窗口
        std::printf("D 编码器静默: 送帧 %llu 无块 %llu 强收 %llu\n", (unsigned long long)r.frames,
                    (unsigned long long)r.blk_empty, (unsigned long long)r.reclaimed);
        check(r.reclaimed > 0, "D: 在途块卡死后没有强收兜底（录像会永久停摆）");
        check(r.blk_empty > 0, "D: 在途块满时没有正确丢帧");
    }

    // ---------------- E VENC 通道全被占：录像放弃，但进程不许死 ----------------
    {
        stub::Config sc;
        sc.create_chn_fails = true;
        Result r = run_scenario("E_no_venc_chn", sc, 40, 1000);
        std::printf("E 通道被占: start()=%s 送帧 %llu 队列丢 %llu\n", r.started ? "true" : "false",
                    (unsigned long long)r.frames, (unsigned long long)r.dropped);
        check(!r.started, "E: 通道建不起来却报告启动成功");
        check(r.frames == 0, "E: 没有通道却记账送帧成功");
        check(r.dropped > 0, "E: 录像没起来时池帧没有正常归还（识别会被饿死）");
        check(r.inflight == 0, "E: 放弃录像后在途块没清零");
    }

    // ---------------- F 编码器自检（--venc-probe）不能破坏正常录像 ----------------
    {
        stub::Config sc;
        Result r = run_scenario("F_venc_probe", sc, 120, 2000, /*venc_probe=*/true);
        std::printf("F 编码器自检: start=%s 送帧 %llu 失败 %llu 无块 %llu 在途 %u\n",
                    r.started ? "true" : "false", (unsigned long long)r.frames,
                    (unsigned long long)r.send_fail, (unsigned long long)r.blk_empty, r.inflight);
        check(r.started, "F: 自检之后录像链路起不来");
        check(r.frames > 0, "F: 自检之后一帧都没录上");
        check(r.send_fail == 0, "F: 自检之后出现送帧失败");
        check(r.blk_empty == 0, "F: 自检之后出现无块可用");
        check(r.inflight == 0, "F: 自检之后在途块没清零");
    }

    // ---------------- G 取证写线程：文件格式/极性/异步写 ----------------
    {
        const std::string dir = out_dir_for("G_evidence");
        {
            // 必须堆分配：对象内有 ~1.4MB 的入队缓冲（栈上放会直接爆栈，ASan 已抓到过）
            std::unique_ptr<dart::EvidenceWriter> evp(new dart::EvidenceWriter(dir.c_str()));
            dart::EvidenceWriter &ev = *evp;
            // 左黑右白：验证 PBM 极性（bit=1 表示黑）与 PGM 内容
            std::vector<uint8_t> bin(640 * 360);
            for (uint32_t y = 0; y < 360; ++y)
                for (uint32_t x = 0; x < 640; ++x)
                    bin[y * 640 + x] = (x < 320) ? 0 : 255;
            for (int i = 0; i < 3; ++i) {
                ev.submit_pbm(1 + i * 9, bin.data(), 640, 360, 640);
                ev.submit_pgm(i, bin.data(), 640, 360, 640);
                ev.submit_csv("1,0,0,0,250,-1,-1,1,0,90.0,1\n", 27);
            }
            // PGM 只有 2 个入队槽：第 3 张必然按"满则丢并计数"处理 —— 等它被记账
            for (int i = 0; i < 500 && (ev.written_pbm() < 3 || ev.written_pgm() + ev.dropped() < 3); ++i)
                usleep(2000);
            check(ev.written_pbm() == 3, "G: PBM 没有全部落盘");
            check(ev.written_pgm() + ev.dropped() == 3, "G: PGM 既没落盘也没计数（账目不平）");
            check(ev.dropped() >= 1, "G: 队列满时没有丢样本（会阻塞视觉链路）");
        }
        int fails_before = g_failures;
        std::vector<int> types;
        long sz = 0;
        check(h264_types(dir + "/f000001.pbm", &types, &sz), "G: PBM 文件不存在");
        check(sz == 11 + 80 * 360, "G: PBM 大小不对");
        // 极性/逐位正确性：左半是暗(0) → bit=1 → 0xFF；右半是亮(255) → bit=0 → 0x00
        if (std::FILE *pf = std::fopen((dir + "/f000001.pbm").c_str(), "rb")) {
            std::vector<uint8_t> row(11 + 80);
            (void)!std::fread(row.data(), 1, 11 + 80, pf);
            std::fclose(pf);
            check(row[0] == 'P' && row[1] == '4', "G: PBM 头不对");
            check(row[11] == 0xFF && row[11 + 39] == 0xFF, "G: PBM 左半（暗）应为 0xFF");
            check(row[11 + 40] == 0x00 && row[11 + 79] == 0x00, "G: PBM 右半（亮）应为 0x00");
        }
        std::FILE *f = std::fopen((dir + "/raw000000.pgm").c_str(), "rb");
        check(f != nullptr, "G: PGM 文件不存在");
        if (f != nullptr) {
            std::vector<uint8_t> buf(16);
            (void)!std::fread(buf.data(), 1, 16, f);
            std::fclose(f);
            check(buf[0] == 'P' && buf[1] == '5', "G: PGM 头不对");
        }
        std::FILE *c = std::fopen((dir + "/frames.csv").c_str(), "rb");
        check(c != nullptr, "G: CSV 不存在");
        if (c != nullptr) {
            char line[256];
            (void)!std::fgets(line, sizeof(line), c); // 表头
            const bool has_row = std::fgets(line, sizeof(line), c) != nullptr;
            std::fclose(c);
            check(has_row, "G: CSV 没有数据行");
        }
        std::printf("G 取证写线程: %s（断言失败 %d）\n", g_failures == fails_before ? "通过" : "失败",
                    g_failures - fails_before);
    }

    int stub_fail = 0;
    stub::check_all(&stub_fail);
    std::printf("=== 结果: %s（本地断言失败 %d，桩断言失败 %d）===\n",
                (g_failures + stub_fail) == 0 ? "全部通过" : "有失败", g_failures, stub_fail);
    return (g_failures + stub_fail) == 0 ? 0 : 1;
}
