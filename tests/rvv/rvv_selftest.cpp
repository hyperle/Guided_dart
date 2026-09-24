// ============================================================================
// RVV 路径的 qemu 回归（riscv64 侧的可执行体）
//
// 为什么有它：主机侧测试（tests/host/detection_host_test.cpp）跑的是**标量参考路径**，
// 板端才有真 RVV。中间这段空白过去只能靠"板端开机自检 + 读反汇编"补，代价是每错一次
// 就要浪费一轮上板。有了 qemu-riscv64（带 RVV 1.0），这一段可以在宿主机上先跑：
//
//   ① VLEN 核对：vsetvlmax_e8m1() 必须是 16（板端 C908 的 VLEN=128）
//   ② LightScanner::selftest()：**向量路径 vs 标量参考逐位比对**（和板端同一个函数）
//   ③ 解析校验：合成图上 RVV 扫出来的面积/质心/等效半径 vs 逐像素真值
//   ④ 整链：DetectionPipeline 在合成序列上跑 启动→跟踪，确认 ROI 门控/滤波在 riscv 上也对
//   ⑤ 数量级：RVV 全图扫描单帧耗时（**qemu 的时间不是板端的时间**，只看数量级）
//
// 构建与运行由 scripts/rvv_qemu.sh 负责（交叉编译 + 挑 qemu + 传正确的 -cpu 参数）。
// ============================================================================

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "detection/light.hpp"
#include "detection/pipeline.hpp"

#if defined(__riscv_vector)
#include <riscv_vector.h>
#define RVV_SELFTEST_HAVE_RVV 1
#endif

using namespace dart::detection;
using dart::DetectResult;
using dart::GrayFrame;
using dart::RoiState;

namespace {

int g_checks = 0;
int g_fails = 0;

#define CHECK(cond, ...)                                                                           \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_fails;                                                                             \
            std::printf("  [FAIL] %s:%d  ", __FILE__, __LINE__);                                   \
            std::printf(__VA_ARGS__);                                                              \
            std::printf("\n");                                                                     \
        }                                                                                          \
    } while (0)

void disk(std::vector<uint8_t> &px, uint32_t w, uint32_t h, float cx, float cy, float r) {
    for (int y = static_cast<int>(cy - r) - 1; y <= static_cast<int>(cy + r) + 1; ++y) {
        if (y < 0 || y >= static_cast<int>(h))
            continue;
        for (int x = static_cast<int>(cx - r) - 1; x <= static_cast<int>(cx + r) + 1; ++x) {
            if (x < 0 || x >= static_cast<int>(w))
                continue;
            const float dx = static_cast<float>(x) - cx;
            const float dy = static_cast<float>(y) - cy;
            if (dx * dx + dy * dy <= r * r)
                px[static_cast<size_t>(y) * w + x] = 255;
        }
    }
}

struct Truth {
    uint32_t area = 0;
    double   cx = 0, cy = 0;
};

Truth truth_of(const std::vector<uint8_t> &px, uint32_t w, uint32_t h) {
    Truth t;
    double sx = 0, sy = 0;
    for (uint32_t y = 0; y < h; ++y)
        for (uint32_t x = 0; x < w; ++x)
            if (px[static_cast<size_t>(y) * w + x] >= 128) {
                ++t.area;
                sx += x;
                sy += y;
            }
    if (t.area) {
        t.cx = sx / t.area;
        t.cy = sy / t.area;
    }
    return t;
}

} // namespace

int main() {
    std::printf("=== RVV 路径 qemu 回归（riscv64）===\n");

    // ① VLEN 核对：板端 C908 是 VLEN=128 → e8m1 的 VLMAX 必须是 16。
    // 我们的 RVV 循环按"一条向量 16 像素"写（u8 归约不回绕的前提），这里先把前提钉死。
#if defined(RVV_SELFTEST_HAVE_RVV)
    const size_t vmax = vsetvlmax_e8m1();
    std::printf("RVV: VLMAX(e8m1) = %zu（期望 16，对应 VLEN=128）\n", vmax);
    CHECK(vmax == 16, "VLMAX(e8m1) 必须等于 16（否则 -cpu ...vlen=128 没传对）");
    CHECK(vmax == 16, "向量路径与标量参考的比对前提成立");
#else
    std::printf("!! 本可执行体没有编译 RVV 路径（__riscv_vector 未定义）—— 检查 -march=rv64imafdcv\n");
    ++g_checks;
    ++g_fails;
#endif

    // ② 与板端同一个自检函数：向量路径 vs 标量参考逐字段比对
    const bool light_ok = LightScanner::selftest();
    CHECK(light_ok, "LightScanner::selftest()（RVV vs 标量逐位比对）必须通过");
    std::printf("  LightScanner::selftest() = %s\n", light_ok ? "PASS" : "FAIL");

    const bool pipe_ok = DetectionPipeline::selftest();
    CHECK(pipe_ok, "DetectionPipeline::selftest()（含 ROI 测量解析校验）必须通过");
    std::printf("  DetectionPipeline::selftest() = %s\n", pipe_ok ? "PASS" : "FAIL");

    // ③ 解析校验：RVV 扫出来的面积/质心/等效半径 vs 逐像素真值（板端尺寸 640x360）
    {
        const uint32_t W = 640, H = 360;
        std::vector<uint8_t> px(static_cast<size_t>(W) * H, 0);
        disk(px, W, H, 200.0f, 120.0f, 9.0f);   // 最大
        disk(px, W, H, 480.0f, 260.0f, 5.0f);   // 次之
        disk(px, W, H, 90.0f, 300.0f, 3.0f);    // 最小（启动阶段的量级）
        px[static_cast<size_t>(40) * W + 600] = 255; // 单像素坏点（应被 min_area 丢掉）

        const Truth truth = truth_of(px, W, H); // 三个目标的总和（用于交叉核对总量）

        GrayFrame f{};
        f.pixels = px.data();
        f.width = W;
        f.height = H;
        f.stride = W;

        ScannerConfig cfg;
        LightScanner  sc(cfg);
        LightCandidate out[8]{};
        const size_t   n = sc.scan(f, out, 8);
        CHECK(n == 3, "三个目标必须出 3 个候选（单像素坏点被 min_area 丢掉），实得 %zu", n);
        if (n == 3) {
            CHECK(out[0].area > out[1].area && out[1].area > out[2].area,
                  "Top-K 必须按面积降序：%u %u %u", out[0].area, out[1].area, out[2].area);

            // 最大的那块：面积/质心/等效半径逐项核对（RVV 路径的量化正确性）
            const float exp_area = 3.14159265f * 9.0f * 9.0f;
            CHECK(std::fabs(static_cast<float>(out[0].area) - exp_area) < exp_area * 0.08f,
                  "最大块面积应≈πr²=%.0f，实得 %u", static_cast<double>(exp_area), out[0].area);
            CHECK(std::fabs(out[0].cx - 200.0f) < 0.6f && std::fabs(out[0].cy - 120.0f) < 0.6f,
                  "最大块质心应≈(200,120)，实得 (%.2f,%.2f)", static_cast<double>(out[0].cx),
                  static_cast<double>(out[0].cy));
            const float r_exp = std::sqrt(static_cast<float>(out[0].area) / 3.14159265f);
            CHECK(std::fabs(out[0].radius - r_exp) < 1e-3f,
                  "等效半径必须满足 r=sqrt(A/π)：%.4f vs %.4f", static_cast<double>(out[0].radius),
                  static_cast<double>(r_exp));
            CHECK(out[0].fill > 0.6f, "圆盘 fill 应接近 0.78，实得 %.3f",
                  static_cast<double>(out[0].fill));
        }
        std::printf("  候选: ");
        for (size_t i = 0; i < n; ++i)
            std::printf("[%zu] area=%u r=%.2f 中心(%.2f,%.2f) ", i, out[i].area,
                        static_cast<double>(out[i].radius), static_cast<double>(out[i].cx),
                        static_cast<double>(out[i].cy));
        std::printf("（逐像素真值总量 %u 像素）\n", truth.area);
    }

    // ④ 整链：合成序列跑 启动 → 跟踪，确认 ROI 门控与滤波在 riscv 上同样成立
    {
        DetectionConfig cfg;
        DetectionPipeline::Deps deps;
        uint64_t now = 0;
        deps.now_us = [&now]() { return now; }; // 假时钟：结果必须可复现
        DetectionPipeline pipe(cfg, deps);

        const uint32_t W = 640, H = 360;
        std::vector<uint8_t> px(static_cast<size_t>(W) * H, 0);
        GrayFrame            f{};
        f.pixels = px.data();
        f.width = W;
        f.height = H;
        f.stride = W;

        float  tx = 200.0f, ty = 150.0f, tr = 6.0f;
        int    tracked = 0;
        bool   roi_only = true;
        float  worst_err = 0.0f;
        for (int k = 0; k < 40; ++k) {
            std::fill(px.begin(), px.end(), 0);
            disk(px, W, H, tx, ty, tr);
            const Truth        truth = truth_of(px, W, H);
            const dart::DetectResult r = pipe.detect(f);
            if (r.state == static_cast<uint8_t>(TrackState::Tracking)) {
                ++tracked;
                if (r.cx >= 0) {
                    const float err = std::fabs(static_cast<float>(r.cx) - static_cast<float>(truth.cx)) +
                                      std::fabs(static_cast<float>(r.cy) - static_cast<float>(truth.cy));
                    if (err > worst_err)
                        worst_err = err;
                }
                if (r.roi_x0 == 0 && r.roi_y0 == 0 && r.roi_x1 == W - 1 && r.roi_y1 == H - 1 && k > 5)
                    roi_only = false;
            }
            tx += 2.0f;
            ty += 0.8f;
            tr += 0.05f;
            now += 11111; // 90fps
        }
        CHECK(tracked >= 35, "40 帧里至少 35 帧应进入跟踪态，实得 %d", tracked);
        CHECK(worst_err < 3.0f, "跟踪位置误差应 <3px（最差 %.2f）", static_cast<double>(worst_err));
        CHECK(roi_only, "跟踪态必须只在 ROI 内扫描");
        std::printf("  整链: 跟踪 %d/40 帧，位置最差误差 %.2fpx，全图扫描 %llu 帧 / ROI 扫描 %llu 帧\n",
                    tracked, static_cast<double>(worst_err),
                    static_cast<unsigned long long>(pipe.stats().tracker.full_scans),
                    static_cast<unsigned long long>(pipe.stats().tracker.roi_scans));
    }

    // ⑤ 数量级参考：RVV 全图粗筛单帧耗时（640x360）。
    // **这不是板端性能**：qemu 是逐指令翻译执行，绝对值没有意义，只看"有没有数量级异常"。
    {
        const uint32_t W = 640, H = 360;
        std::vector<uint8_t> px(static_cast<size_t>(W) * H, 0);
        disk(px, W, H, 300.0f, 180.0f, 8.0f);
        GrayFrame f{};
        f.pixels = px.data();
        f.width = W;
        f.height = H;
        f.stride = W;

        ScannerConfig cfg;
        LightScanner  sc(cfg);
        LightCandidate out[8]{};
        for (int i = 0; i < 5; ++i)
            sc.scan(f, out, 8); // 预热
        const int    N = 50;
        const auto   t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < N; ++i)
            sc.scan(f, out, 8);
        const auto t1 = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / N;
        std::printf("  RVV 全图粗筛 640x360: %.0f us/帧（qemu 翻译执行，仅作数量级参考；"
                    "板端看日志的 扫描 avg/max）\n", us);
        CHECK(us < 20000.0, "单帧粗筛耗时不应出现数量级异常（实得 %.0fus）", us);
    }

    std::printf("\n=== %d 项检查，失败 %d 项 ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
