// ============================================================================
// 识别/跟踪层主机侧回归测试（x86，无 RVV、无相机、时间可控）
//
//   bash tests/host/run.sh
//
// 为什么要有它：这层的失败模式（启动确认收不收敛、坏点有没有被挡住、状态机会不会
// 卡在丢失、ROI 会不会把目标切出去、R_scale 到底有没有"远稳近跟"）全都只在板端
// 才暴露，而板端一次验证要拔卡、上电、再拔卡。这里用**合成的二值图 + 假时钟**
// 把整条链跑起来，把逻辑先咬死；上板只留下"RVV 与内存时序"这类真的只能上板才知道的事。
//
// 覆盖面：
//   1) linalg       矩阵乘/转置/求逆（含奇异）
//   2) TileScanner 全图粗筛：质心/面积/等效半径解析校验、Top-K 排序、门限、stride
//   3) RunLengthMeasurer 窗口连通域：绝对坐标、贴框语义、取最大块、空窗
//   4) BlipConfirmer 3 帧滑窗：平滑目标确认；随机闪烁/随机跳变/锯齿位移全部拒掉
//   5) ScaleAwareKalman 6 维滤波：初速拟合、匀速收敛、膨胀速率、马氏门限、
//                    **R_scale 自适应（远=模型主导、近=贴合测量）**
//   6) TargetTracker / DetectionPipeline 整链：启动→跟踪→丢失→重捕→硬复位，
//                    并验证"跟踪态只在 ROI 内扫描"、ROI = kp·ŝ+B_margin(+kσσ)
// ============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "detection/track/blip_confirmer.hpp"
#include "detection/measure/roi_measure.hpp"
#include "detection/track/kalman.hpp"
#include "detection/linalg.hpp"
#include "detection/scanner/tile_scanner.hpp"
#include "detection/pipeline.hpp"
#include "detection/track/roi_prediction.hpp"
#include "detection/track/tracker.hpp"

using namespace dart::detection;
using dart::DetectResult; // dart::DetectResult（核心层的结果类型，不是 detection:: 里的）
using dart::GrayFrame;
using dart::RoiState;

// ---------------------------------------------------------------------------
// 极简测试框架：失败只登记 + 打印，最后统一报总数（不做提前退出，
// 一次跑完能看到"这一改动同时坏了几处"）
// ---------------------------------------------------------------------------
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

#define SECTION(name) std::printf("\n== %s ==\n", name)

void ok(const char *what, const char *detail = "") { std::printf("  [ ok ] %s %s\n", what, detail); }

} // namespace

// ---------------------------------------------------------------------------
// 合成场景：一张二值图（0/255），画圆盘/方块
// ---------------------------------------------------------------------------
namespace {

struct Scene {
    uint32_t             w = 0, h = 0, stride = 0;
    std::vector<uint8_t> px;

    Scene(uint32_t w_, uint32_t h_, uint32_t stride_ = 0)
        : w(w_), h(h_), stride(stride_ ? stride_ : w_), px(static_cast<size_t>(stride_ ? stride_ : w_) * h_, 0) {}

    void clear() { std::fill(px.begin(), px.end(), 0); }

    void disk(float cx, float cy, float r) {
        const int x0 = static_cast<int>(std::floor(cx - r)), x1 = static_cast<int>(std::ceil(cx + r));
        const int y0 = static_cast<int>(std::floor(cy - r)), y1 = static_cast<int>(std::ceil(cy + r));
        for (int y = y0; y <= y1; ++y) {
            if (y < 0 || y >= static_cast<int>(h))
                continue;
            for (int x = x0; x <= x1; ++x) {
                if (x < 0 || x >= static_cast<int>(w))
                    continue;
                const float dx = static_cast<float>(x) - cx, dy = static_cast<float>(y) - cy;
                if (dx * dx + dy * dy <= r * r)
                    px[static_cast<size_t>(y) * stride + x] = 255;
            }
        }
    }

    void box(int bx, int by, int bw, int bh) {
        for (int y = by; y < by + bh; ++y)
            for (int x = bx; x < bx + bw; ++x)
                if (x >= 0 && y >= 0 && x < static_cast<int>(w) && y < static_cast<int>(h))
                    px[static_cast<size_t>(y) * stride + x] = 255;
    }

    GrayFrame view() {
        GrayFrame f{};
        f.pixels = px.data();
        f.width = w;
        f.height = h;
        f.stride = stride;
        return f;
    }
};

// 圆盘在离散像素网格上的真实面积/质心（用同一套判据自己数一遍，避免"期望值"写错）
struct Truth {
    uint32_t area = 0;
    double   cx = 0, cy = 0;
    float    radius() const { return radius_from_area(static_cast<float>(area)); }
};

Truth truth_of(const Scene &s) {
    Truth t;
    double sx = 0, sy = 0;
    for (uint32_t y = 0; y < s.h; ++y)
        for (uint32_t x = 0; x < s.w; ++x)
            if (s.px[static_cast<size_t>(y) * s.stride + x] >= 128) {
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

// ===========================================================================
// 1) linalg
// ===========================================================================
void test_linalg() {
    SECTION("1) linalg 小矩阵");
    linalg::Mat<2, 2> a;
    a(0, 0) = 4;
    a(0, 1) = 7;
    a(1, 0) = 2;
    a(1, 1) = 6;
    linalg::Mat<2, 2> inv;
    CHECK(linalg::inverse(a, &inv), "2x2 非奇异矩阵必须能求逆");
    const linalg::Mat<2, 2> prod = linalg::mul(a, inv);
    CHECK(std::fabs(prod(0, 0) - 1.0) < 1e-12 && std::fabs(prod(1, 1) - 1.0) < 1e-12 &&
              std::fabs(prod(0, 1)) < 1e-12 && std::fabs(prod(1, 0)) < 1e-12,
          "A·A⁻¹ 必须是单位阵: [%g %g; %g %g]", prod(0, 0), prod(0, 1), prod(1, 0), prod(1, 1));

    linalg::Mat<3, 3> sing;
    sing(0, 0) = 1;
    sing(0, 1) = 2;
    sing(0, 2) = 3;
    sing(1, 0) = 2;
    sing(1, 1) = 4;
    sing(1, 2) = 6;
    sing(2, 0) = 1;
    sing(2, 1) = 1;
    sing(2, 2) = 1;
    linalg::Mat<3, 3> dummy;
    CHECK(!linalg::inverse(sing, &dummy), "奇异矩阵必须返回 false（宁可不更新，不可出 NaN）");

    linalg::Mat<2, 3> r;
    r(0, 0) = 1;
    r(0, 1) = 2;
    r(0, 2) = 3;
    r(1, 0) = 4;
    r(1, 1) = 5;
    r(1, 2) = 6;
    const linalg::Mat<3, 2> rt = linalg::transpose(r);
    CHECK(rt(2, 1) == 6 && rt(0, 1) == 4, "转置下标必须对得上");
    ok("矩阵乘/转置/求逆/奇异检测");
}

// ===========================================================================
// 2) 全图扫描器
// ===========================================================================
void test_scanner() {
    SECTION("2) TileScanner 全图粗筛");

    ScannerConfig cfg;
    cfg.min_area = 3;
    cfg.top_k = 4;
    TileScanner sc(cfg);

    // ---- 解析校验：一个圆盘的面积/质心/等效半径必须与逐像素数出来的完全一致 ----
    Scene s(320, 200);
    s.disk(157.0f, 88.0f, 11.0f);
    const Truth t = truth_of(s);
    Blip out[8]{};
    size_t          n = sc.scan(s.view(), out, 8);
    CHECK(n == 1, "只有一个目标时只能出 1 个候选，实得 %zu", n);
    if (n == 1) {
        CHECK(out[0].area == t.area, "面积必须精确：扫到 %u，逐像素数出 %u", out[0].area, t.area);
        CHECK(std::fabs(out[0].cx - static_cast<float>(t.cx)) < 0.01f, "质心 x: %.3f vs %.3f",
              static_cast<double>(out[0].cx), t.cx);
        CHECK(std::fabs(out[0].cy - static_cast<float>(t.cy)) < 0.01f, "质心 y: %.3f vs %.3f",
              static_cast<double>(out[0].cy), t.cy);
        CHECK(std::fabs(out[0].radius - t.radius()) < 0.01f, "等效半径 r=sqrt(A/π): %.3f vs %.3f",
              static_cast<double>(out[0].radius), static_cast<double>(t.radius()));
        CHECK(out[0].fill > 0.6f, "圆盘 fill 应接近 0.78，实得 %.3f", static_cast<double>(out[0].fill));
    }
    ok("解析校验（面积/质心/等效半径/fill）");

    // ---- Top-K 排序：三个不同大小的目标必须按"最亮（面积大）在前" ----
    Scene m(320, 200);
    m.disk(40, 40, 5);
    m.disk(160, 100, 12);
    m.disk(260, 160, 8);
    n = sc.scan(m.view(), out, 8);
    CHECK(n == 3, "三个目标应出 3 个候选，实得 %zu", n);
    if (n == 3) {
        CHECK(out[0].area > out[1].area && out[1].area > out[2].area,
              "Top-K 必须按面积降序：%u %u %u", out[0].area, out[1].area, out[2].area);
        CHECK(std::fabs(out[0].cx - 160.0f) < 1.0f, "最大块应在 (160,100)，实得 %.1f",
              static_cast<double>(out[0].cx));
    }

    // ---- Top-K 截断 ----
    ScannerConfig cfg1 = cfg;
    cfg1.top_k = 2;
    TileScanner sc1(cfg1);
    n = sc1.scan(m.view(), out, 8);
    CHECK(n == 2, "top_k=2 时只出 2 个候选，实得 %zu", n);

    // ---- 面积门限：单像素噪点必须被丢掉 ----
    Scene tiny(160, 120);
    tiny.px[static_cast<size_t>(60) * 160 + 80] = 255;
    tiny.px[static_cast<size_t>(61) * 160 + 81] = 255;
    n = sc.scan(tiny.view(), out, 8);
    CHECK(n == 0, "单像素/两像素噪点必须被 min_area 丢掉，实得 %zu", n);

    // ---- 整片过曝：应被 max_area_frac 丢掉（它不是"小目标"）----
    Scene flood(160, 120);
    flood.box(0, 0, 160, 120);
    n = sc.scan(flood.view(), out, 8);
    CHECK(n == 0, "整幅过曝必须被 max_area_frac 丢掉，实得 %zu", n);

    // ---- stride > width：行填充不能把质心带偏 ----
    Scene pad(200, 120, 256);
    pad.disk(100.0f, 60.0f, 9.0f);
    Blip pout[4]{};
    n = sc.scan(pad.view(), pout, 4);
    CHECK(n == 1 && std::fabs(pout[0].cx - 100.0f) < 0.5f && std::fabs(pout[0].cy - 60.0f) < 0.5f,
          "stride>width 时质心仍须正确（实得 n=%zu cx=%.2f cy=%.2f）", n,
          n ? static_cast<double>(pout[0].cx) : -1.0, n ? static_cast<double>(pout[0].cy) : -1.0);

    // ---- 空图/非法图不能崩 ----
    Scene blank(64, 48);
    CHECK(sc.scan(blank.view(), out, 8) == 0, "全黑帧必须返回 0");
    GrayFrame bad{};
    CHECK(sc.scan(bad, out, 8) == 0, "空指针帧必须安全返回 0");

    // ---- 瓦片边长 > 16（一条向量吃不下一个瓦片行）也要精确 ----
    ScannerConfig cfgBig = cfg;
    cfgBig.tile = 32;
    TileScanner scBig(cfgBig);
    Scene big(320, 200);
    big.disk(157.0f, 88.0f, 11.0f);
    const Truth tb = truth_of(big);
    n = scBig.scan(big.view(), out, 8);
    CHECK(n == 1 && out[0].area == tb.area && std::fabs(out[0].cx - static_cast<float>(tb.cx)) < 0.01f,
          "tile=32 时面积/质心仍须精确（area %u vs %u）", n ? out[0].area : 0, tb.area);

    // ---- **圆度**：发光体是圆斑、反光/拖影是长条，排序要能分开 ----
    {
        // ① 圆度的量级：圆盘≈0.8~1、正方形≈0.785、细长条远小于圆
        ScannerConfig c0;
        TileScanner  sc0(c0);
        Scene d2(120, 120);
        d2.disk(60.0f, 60.0f, 18.0f);
        Scene sq2(120, 120);
        sq2.box(40, 40, 30, 30);
        Scene ln2(160, 60);
        ln2.box(20, 30, 40, 3);
        Blip od[4]{}, os[4]{}, ol[4]{};
        CHECK(sc0.scan(d2.view(), od, 4) == 1, "圆盘应出 1 个候选");
        CHECK(sc0.scan(sq2.view(), os, 4) == 1, "方块应出 1 个候选");
        CHECK(sc0.scan(ln2.view(), ol, 4) == 1, "横条应出 1 个候选");
        const float c_disk = od[0].circularity, c_sq = os[0].circularity, c_line = ol[0].circularity;
        // 期望值按实测校准（宿主/板端同一套离散化）：
        //   圆盘 r=18 → 0.938；紧凑方块 → 1.0（"紧凑"就该算圆，不能惩罚小目标）；
        //   40x3 横条 → 0.071（轴比 0.05 × 填充率 1.0）。
        CHECK(c_disk > 0.85f && c_disk <= 1.05f, "圆盘圆度应≈0.9（实得 %.3f）", static_cast<double>(c_disk));
        CHECK(c_sq > 0.9f, "紧凑方块应被当成'圆'（实得 %.3f）", static_cast<double>(c_sq));
        CHECK(c_line < 0.3f, "细长条圆度应远小于圆（实得 %.3f）", static_cast<double>(c_line));
        // 尺度无关性：小圆盘不能被判成"不圆"（旧公式 4πA/P² 在小目标上就是这个毛病）
        Scene d_small(60, 60);
        d_small.disk(30.0f, 30.0f, 3.0f); // 启动阶段目标的量级
        Blip osm[4]{};
        CHECK(sc0.scan(d_small.view(), osm, 4) == 1, "小圆盘应出 1 个候选");
        const float c_small = osm[0].circularity;
        CHECK(c_small > 0.6f, "r=3px 的小圆盘圆度仍应 >0.6（实得 %.3f）", static_cast<double>(c_small));
        std::printf("       圆度: 圆盘(r=18) %.3f / 小圆盘(r=3) %.3f / 方块 %.3f / 40x3 横条 %.3f\n",
                    static_cast<double>(c_disk), static_cast<double>(c_small), static_cast<double>(c_sq),
                    static_cast<double>(c_line));

        // ② 排序：一条"更长更亮"的光带 vs 一个更小但更圆的目标
        //   只按面积 → 光带赢；按 面积×圆度 → 圆的赢。这正是"提高圆度占比"要的效果。
        Scene mix(200, 120);
        mix.disk(60.0f, 60.0f, 12.0f);   // 圆目标：面积≈452
        mix.box(110, 58, 70, 6);         // 光带：面积 420（略小）；再拉长一点让它更大：
        mix.box(110, 96, 90, 6);         // 另一条更长：面积 540 > 452
        ScannerConfig c_area;
        c_area.circ_weight = 0.0f;       // 旧行为：纯面积
        ScannerConfig c_circ;
        c_circ.circ_weight = 1.0f;       // 新默认：面积 × 圆度
        TileScanner sa(c_area), sc(c_circ);
        Blip oa[4]{}, oc[4]{};
        const size_t na = sa.scan(mix.view(), oa, 4), nc = sc.scan(mix.view(), oc, 4);
        CHECK(na >= 2 && nc >= 2, "应至少出 2 个候选（实得 %zu / %zu）", na, nc);
        if (na >= 2 && nc >= 2) {
            const bool area_picks_band = oa[0].circularity < 0.5f;
            const bool circ_picks_disk = oc[0].circularity > 0.7f;
            CHECK(area_picks_band, "circ_weight=0 时应按面积选中光带（实得圆度 %.3f）",
                  static_cast<double>(oa[0].circularity));
            CHECK(circ_picks_disk, "circ_weight=1 时应选中圆目标（实得圆度 %.3f，面积 %u vs 光带 %u）",
                  static_cast<double>(oc[0].circularity), oc[0].area, oc[1].area);
        }

        // ③ 圆度硬门限：把细长条直接筛掉
        ScannerConfig c_gate = c_area;
        c_gate.min_circularity = 0.5f;
        TileScanner sg(c_gate);
        Blip og[4]{};
        const size_t ng = sg.scan(ln2.view(), og, 4);
        CHECK(ng == 0, "min_circularity=0.5 时细长条应被筛掉（实得 %zu）", ng);
        Blip og2[4]{};
        CHECK(sg.scan(d2.view(), og2, 4) == 1, "圆盘不该被圆度门限误伤");
        ok("圆度量级 / 排序权重 / 圆度门限");
    }

    // ---- 板端自检函数本身必须通过（RVV 路径与标量参考逐位比对）----
    CHECK(TileScanner::selftest(), "TileScanner::selftest() 必须通过");
    ok("Top-K 排序 / 门限 / stride / tile>16 / selftest");
}

// ===========================================================================
// 3) ROI 测量器
// ===========================================================================
void test_measurer() {
    SECTION("3) RunLengthMeasurer 窗口连通域");
    MeasureConfig cfg;
    RunLengthMeasurer meas(cfg);

    Scene s(320, 200);
    s.disk(150.0f, 100.0f, 10.0f);
    const Truth t = truth_of(s);

    RoiWindow w;
    w.x = 120;
    w.y = 70;
    w.w = 64;
    w.h = 64;
    w.full_frame = false;

    const TargetMeasurement m = meas.measure(s.view(), w);
    CHECK(m.valid, "窗口内含完整目标时必须测到");
    CHECK(m.area == t.area, "面积必须精确：%u vs %u", m.area, t.area);
    CHECK(std::fabs(m.cx - static_cast<float>(t.cx)) < 0.01f &&
              std::fabs(m.cy - static_cast<float>(t.cy)) < 0.01f,
          "**绝对坐标**必须正确（窗口偏移别忘了加）：(%.2f,%.2f) vs (%.2f,%.2f)",
          static_cast<double>(m.cx), static_cast<double>(m.cy), t.cx, t.cy);
    CHECK(m.roi == RoiState::Hit, "完整落在窗口内应为 Hit");
    CHECK(m.quality == 1.0f, "完整落在窗口内质量应为 1");

    // 贴框：窗口刚好卡住目标 → TargetOutOfRoi + 质量降低
    RoiWindow tight;
    tight.x = 140;
    tight.y = 90;
    tight.w = 20;
    tight.h = 20;
    tight.full_frame = false;
    const TargetMeasurement mt = meas.measure(s.view(), tight);
    CHECK(mt.valid && mt.roi == RoiState::TargetOutOfRoi && mt.quality < 1.0f,
          "贴框必须报 TargetOutOfRoi 并降低质量（valid=%d roi=%d q=%.2f）", static_cast<int>(mt.valid),
          static_cast<int>(mt.roi), static_cast<double>(mt.quality));

    // 取最大块：窗口里有一个大目标 + 一个小亮斑
    Scene two(320, 200);
    two.disk(150.0f, 100.0f, 10.0f);
    two.box(130, 80, 4, 4);
    const TargetMeasurement mbig = meas.measure(two.view(), w);
    CHECK(mbig.valid && std::fabs(mbig.radius - t.radius()) < 0.5f, "必须取最大块（r=%.2f vs %.2f）",
          static_cast<double>(mbig.radius), static_cast<double>(t.radius()));

    // ---- 圆度：ROI 里"更长的光带"不该抢走目标 ----
    {
        Scene mix(320, 200);
        mix.disk(150.0f, 100.0f, 12.0f); // 圆目标：面积≈452、长宽比 1
        // 光带：60x10（面积 600 > 452，但长宽比 6.0 仍在 aspect_hi=6 门限内 ——
        // 用 120x6 那种超长条会先被 aspect 门限筛掉，测不到圆度权重的作用）
        mix.box(110, 150, 60, 10);
        RoiWindow rw;
        rw.x = 80;
        rw.y = 60;
        rw.w = 200;
        rw.h = 120;
        rw.full_frame = false;
        MeasureConfig m_area;
        m_area.circ_weight = 0.0f;
        MeasureConfig m_circ;
        m_circ.circ_weight = 1.0f;
        RunLengthMeasurer ma(m_area), mc(m_circ);
        const TargetMeasurement ra = ma.measure(mix.view(), rw);
        const TargetMeasurement rc = mc.measure(mix.view(), rw);
        CHECK(ra.valid && rc.valid, "两种权重下都应测到目标");
        if (ra.valid && rc.valid) {
            CHECK(ra.circularity < 0.5f, "纯面积权重下会选中光带（圆度 %.3f）",
                  static_cast<double>(ra.circularity));
            CHECK(rc.circularity > 0.7f, "圆度加权后应选中圆目标（圆度 %.3f）",
                  static_cast<double>(rc.circularity));
        }
        // 圆度门限
        MeasureConfig m_gate;
        m_gate.min_circularity = 0.5f;
        RunLengthMeasurer mg(m_gate);
        Scene band(320, 200);
        band.box(100, 100, 120, 6); // 只有一条光带
        const TargetMeasurement rg = mg.measure(band.view(), rw);
        CHECK(!rg.valid, "min_circularity=0.5 时只有光带的窗口应返回无目标");
        std::printf("       ROI 取块: 纯面积→圆度 %.3f / 面积×圆度→圆度 %.3f\n",
                    static_cast<double>(ra.circularity), static_cast<double>(rc.circularity));
        ok("ROI 取块按圆度加权（不再单纯取最大块）");
    }

    // 空窗
    Scene blank(320, 200);
    CHECK(!meas.measure(blank.view(), w).valid, "全黑窗口必须返回无目标");

    // 非法窗口（越界）不能崩
    RoiWindow bad;
    bad.x = 300;
    bad.y = 190;
    bad.w = 64;
    bad.h = 64;
    bad.full_frame = false;
    CHECK(!meas.measure(blank.view(), bad).valid, "越界窗口必须安全返回无目标");
    ok("绝对坐标 / 贴框语义 / 取最大块 / 空窗 / 越界窗口");
}

// ===========================================================================
// 4) 启动确认器（3 帧滑窗 + 位移矢量平滑性）
// ===========================================================================
namespace {

Blip mk_cand(float x, float y, float r) {
    Blip c{};
    c.cx = x;
    c.cy = y;
    c.area = static_cast<uint32_t>(3.14159265f * r * r);
    c.radius = r;
    c.fill = 0.7f;
    c.score = static_cast<float>(c.area);
    c.x0 = static_cast<uint32_t>(x - r);
    c.y0 = static_cast<uint32_t>(y - r);
    c.x1 = static_cast<uint32_t>(x + r);
    c.y1 = static_cast<uint32_t>(y + r);
    return c;
}

} // namespace

void test_confirmer() {
    SECTION("4) BlipConfirmer 3 帧滑窗确认");
    const uint64_t dt = 11111; // 90fps ≈ 11.1ms

    // ---- ① 平滑运动的目标：第 3 帧必须确认，continuity=1，整条轨迹时间升序 ----
    {
        ConfirmerConfig cfg;
        BlipConfirmer confirmer(cfg);
        Blip c{};
        bool confirmed = false;
        uint64_t t = 0;
        for (int k = 0; k < 6; ++k) {
            c = mk_cand(100.0f + 4.0f * k, 80.0f, 3.0f);
            const BlipConfirmer::Confirmation cf = confirmer.push(t, &c, 1);
            if (k < 2)
                CHECK(!cf.ok, "滑窗未满（%d 帧）时不能确认", k + 1);
            if (cf.ok) {
                confirmed = true;
                CHECK(cf.hits == 3, "命中数应为 3，实得 %u", cf.hits);
                CHECK(std::fabs(cf.cand.continuity - 1.0f) < 1e-6f, "continuity 应为 1.0，实得 %.2f",
                      static_cast<double>(cf.cand.continuity));
                CHECK(cf.n == 3 && cf.times_us[0] < cf.times_us[2], "轨迹样本必须时间升序");
                CHECK(std::fabs(cf.chain[2].cx - (100.0f + 4.0f * k)) < 1e-3f,
                      "轨迹最后一帧应是本帧位置，实得 %.2f", static_cast<double>(cf.chain[2].cx));
                CHECK(cf.max_accel_px < 1e-3f, "匀速运动的帧间加速度应≈0，实得 %.3f",
                      static_cast<double>(cf.max_accel_px));
                break;
            }
            t += dt;
        }
        CHECK(confirmed, "平滑运动的目标必须在滑窗填满后立刻确认");
        ok("平滑目标确认（3/3 帧）");
    }

    // ---- ② 随机闪烁的坏点：一个都不许确认 ----
    {
        ConfirmerConfig cfg;
        BlipConfirmer confirmer(cfg);
        std::mt19937                     rng(1234);
        std::uniform_int_distribution<int> dx(5, 630), dy(5, 350);
        int                               confirms = 0;
        uint64_t                          t = 0;
        for (int k = 0; k < 300; ++k) {
            // 每帧两个随机位置的 2×2 亮斑（面积 4 ≥ min_area，所以能过扫描器）
            Blip cs[2];
            cs[0] = mk_cand(static_cast<float>(dx(rng)), static_cast<float>(dy(rng)), 1.2f);
            cs[1] = mk_cand(static_cast<float>(dx(rng)), static_cast<float>(dy(rng)), 1.2f);
            if (confirmer.push(t, cs, 2).ok)
                ++confirms;
            t += dt;
        }
        CHECK(confirms == 0, "随机闪烁坏点在 300 帧里被确认了 %d 次（必须为 0）", confirms);
        ok("随机闪烁坏点 0 次确认（300 帧）");
    }

    // ---- ③ 锯齿位移（能关联上、但加速度爆掉）：必须被平滑性门挡掉 ----
    {
        ConfirmerConfig cfg;
        BlipConfirmer confirmer(cfg);
        int           confirms = 0;
        uint64_t      t = 0;
        for (int k = 0; k < 30; ++k) {
            const float x = (k % 2 == 0) ? 200.0f : 202.0f; // 位移 ±2px 交替 → 帧间加速度 4px
            const Blip c = mk_cand(x, 150.0f, 3.0f);
            if (confirmer.push(t, &c, 1).ok)
                ++confirms;
            t += dt;
        }
        CHECK(confirms == 0, "锯齿位移必须被 max_accel_px 挡掉，实得 %d 次确认", confirms);
        CHECK(confirmer.last_reject() > 0, "被拒的候选应当被计数（last_reject=%u）", confirmer.last_reject());
        ok("锯齿位移被平滑性门挡掉");
    }

    // ---- ④ 随机跳变（关联门之外）：也不许确认 ----
    {
        ConfirmerConfig cfg;
        BlipConfirmer confirmer(cfg);
        std::mt19937                       rng(99);
        std::uniform_int_distribution<int> jx(20, 600), jy(20, 330);
        int                                confirms = 0;
        uint64_t                           t = 0;
        for (int k = 0; k < 100; ++k) {
            const Blip c = mk_cand(static_cast<float>(jx(rng)), static_cast<float>(jy(rng)), 4.0f);
            if (confirmer.push(t, &c, 1).ok)
                ++confirms;
            t += dt;
        }
        CHECK(confirms == 0, "随机跳变的目标必须被关联门挡掉，实得 %d 次确认", confirms);
        ok("随机跳变被关联门挡掉");
    }

    // ---- ⑤ 贴画面边的候选：必须跳过（round12 的实测教训）----
    // 场景：一个 r=25px 的大亮块贴着画面右边（手/物体从镜头前扫过），
    // 它连续 6 帧平滑移动 —— 没有这条门就会被确认，然后 3 帧内丢失。
    {
        ConfirmerConfig cfg;
        BlipConfirmer confirmer(cfg);
        int confirms = 0;
        uint64_t t = 0;
        for (int k = 0; k < 12; ++k) {
            Blip c = mk_cand(615.0f, 180.0f + 2.0f * k, 25.0f);
            c.x1 = 640;          // 包围盒顶到右边界 → 被画面切掉
            c.border = true;
            if (confirmer.push(t, &c, 1).ok)
                ++confirms;
            t += dt;
        }
        CHECK(confirms == 0, "贴边的候选必须被跳过，实得 %d 次确认", confirms);
        CHECK(confirmer.last_border_skip() == 1, "贴边跳过必须单独计数（实得 %u）", confirmer.last_border_skip());
        CHECK(confirmer.last_reject() == 0, "贴边跳过不该混进'拒闪'（实得 %u）", confirmer.last_reject());

        // 同一个块完整进画面（border=false）后必须能确认 —— 门不能把真目标也挡在门外
        ConfirmerConfig cfg2;
        BlipConfirmer arm2(cfg2);
        int confirms2 = 0;
        t = 0;
        for (int k = 0; k < 12; ++k) {
            Blip c = mk_cand(615.0f, 180.0f + 2.0f * k, 25.0f);
            c.border = false;
            if (arm2.push(t, &c, 1).ok)
                ++confirms2;
            t += dt;
        }
        CHECK(confirms2 > 0, "完整落进画面的同样一个块必须能确认（实得 %d）", confirms2);
        ok("贴边候选跳过（且不误伤进画面后的同一目标）");
    }

    // ---- ⑥ 全黑帧（无候选）与 reset 的行为 ----
    {
        ConfirmerConfig cfg;
        BlipConfirmer confirmer(cfg);
        Blip c = mk_cand(100.0f, 100.0f, 3.0f);
        confirmer.push(0, &c, 1);
        confirmer.push(dt, &c, 1);
        CHECK(!confirmer.push(2 * dt, nullptr, 0).ok, "空候选表不能确认");
        confirmer.reset();
        CHECK(!confirmer.ready(), "reset 之后滑窗必须重新计数");
        ok("空帧与 reset");
    }
}

// ===========================================================================
// 5) 尺度自适应卡尔曼
// ===========================================================================
void test_kalman() {
    SECTION("5) ScaleAwareKalman（6 维：x y vx vy s vs）");

    // ---- 5.1 噪声模型：远→大、近→小，质量 q 放大方差 ----
    {
        KfConfig        cfg;
        LinearScaleNoiseModel nm(cfg);
        CHECK(std::fabs(nm.base_scale_variance(cfg.s_far) - cfg.r_scale_far) < 1e-3f,
              "s=s_far 时 R_scale 应等于 r_scale_far");
        CHECK(std::fabs(nm.base_scale_variance(cfg.s_near) - cfg.r_scale_near) < 1e-3f,
              "s=s_near 时 R_scale 应等于 r_scale_near");
        const float mid = nm.base_scale_variance((cfg.s_far + cfg.s_near) * 0.5f);
        CHECK(mid < cfg.r_scale_far && mid > cfg.r_scale_near, "中间尺度应线性过渡（%.2f）",
              static_cast<double>(mid));
        // 用容差比较：插值公式在 t=0/1 处过一次浮点乘加，不保证逐位相等
        CHECK(std::fabs(nm.base_scale_variance(1.0f) - cfg.r_scale_far) < 1e-4f,
              "比 s_far 更远（更小）也不能超过远档：%.4f",
              static_cast<double>(nm.base_scale_variance(1.0f)));
        CHECK(std::fabs(nm.base_scale_variance(100.0f) - cfg.r_scale_near) < 1e-4f,
              "比 s_near 更近也不能小于近档：%.4f",
              static_cast<double>(nm.base_scale_variance(100.0f)));
        const float q_bad = nm.scale_variance(20.0f, 0.25f);
        const float q_ok = nm.scale_variance(20.0f, 1.0f);
        CHECK(q_bad > q_ok * 10.0f, "质量 0.25 应把方差放大约 16 倍（%.2f vs %.2f）",
              static_cast<double>(q_bad), static_cast<double>(q_ok));
        CHECK(nm.scale_variance(35.0f, 1e-9f) <= cfg.r_max, "方差必须被 r_max 封顶");
        ok("R_scale(s) 远近两档 + 质量缩放");
    }

    // ---- 5.2 用滑窗样本初始化：速度/膨胀率必须是拟合值，不是 0 ----
    {
        KfConfig          cfg;
        ScaleAwareKalman  kf(cfg);
        const uint64_t    dt = 11111;
        Blip    chain[3];
        uint64_t          times[3];
        // 目标以 3px/帧向右、1px/帧向下运动，半径 10→11→12（每帧 +1px）
        for (int i = 0; i < 3; ++i) {
            chain[i] = mk_cand(100.0f + 3.0f * i, 50.0f + 1.0f * i, 10.0f + 1.0f * i);
            times[i] = static_cast<uint64_t>(i) * dt;
        }
        kf.init_from_track(chain, times, 3);
        CHECK(kf.initialized(), "初始化后必须处于已初始化态");
        const float exp_vx = 3.0f / 0.011111f;
        const float exp_vy = 1.0f / 0.011111f;
        const float exp_vs = 1.0f / 0.011111f;
        CHECK(std::fabs(kf.vx() - exp_vx) < exp_vx * 0.02f, "vx 拟合值应≈%.0f，实得 %.1f",
              static_cast<double>(exp_vx), static_cast<double>(kf.vx()));
        CHECK(std::fabs(kf.vy() - exp_vy) < exp_vy * 0.02f, "vy 拟合值应≈%.0f，实得 %.1f",
              static_cast<double>(exp_vy), static_cast<double>(kf.vy()));
        CHECK(std::fabs(kf.vs() - exp_vs) < exp_vs * 0.02f, "vs 拟合值应≈%.0f，实得 %.1f",
              static_cast<double>(exp_vs), static_cast<double>(kf.vs()));
        CHECK(std::fabs(kf.s() - 12.0f) < 0.2f, "尺度应取最后一帧（≈12），实得 %.2f",
              static_cast<double>(kf.s()));
        ok("滑窗最小二乘初始化（位置/速度/尺度/膨胀率）");
    }

    // ---- 5.3 匀速 + 匀速膨胀：收敛后跟踪误差应很小 ----
    {
        KfConfig         cfg;
        ScaleAwareKalman kf(cfg);
        kf.init_from_measurement(100.0f, 100.0f, 6.0f);
        const float dt_s = 1.0f / 90.0f;
        const float vx = 200.0f, vy = 60.0f, vs = 25.0f; // px/s
        float       tx = 100.0f, ty = 100.0f, tr = 6.0f;
        float       err_x = 0, err_y = 0, err_r = 0;
        const int   N = 200;
        for (int k = 0; k < N; ++k) {
            tx += vx * dt_s;
            ty += vy * dt_s;
            tr += vs * dt_s;
            kf.predict(dt_s);
            TargetMeasurement m{};
            m.valid = true;
            m.cx = tx;
            m.cy = ty;
            m.radius = tr;
            m.quality = 1.0f;
            CHECK(kf.update(m), "正常测量不该被门限拒收（第 %d 帧）", k);
            if (k > N - 40) { // 只看后半程（已收敛）
                err_x += std::fabs(kf.x() - tx);
                err_y += std::fabs(kf.y() - ty);
                err_r += std::fabs(kf.s() - tr);
            }
        }
        err_x /= 40;
        err_y /= 40;
        err_r /= 40;
        CHECK(err_x < 1.0f && err_y < 1.0f, "收敛后位置误差应 <1px：x=%.3f y=%.3f", static_cast<double>(err_x),
              static_cast<double>(err_y));
        CHECK(err_r < 0.5f, "收敛后尺度误差应 <0.5px，实得 %.3f", static_cast<double>(err_r));
        CHECK(std::fabs(kf.vx() - vx) < vx * 0.05f && std::fabs(kf.vs() - vs) < vs * 0.2f,
              "速度/膨胀速率应收敛到真值：vx=%.1f(真%.0f) vs=%.1f(真%.0f)", static_cast<double>(kf.vx()),
              static_cast<double>(vx), static_cast<double>(kf.vs()), static_cast<double>(vs));
        ok("匀速+匀速膨胀收敛（位置 <1px、尺度 <0.5px）");
    }

    // ---- 5.4 马氏门限：离群测量必须被拒收，且状态不变 ----
    {
        KfConfig         cfg;
        ScaleAwareKalman kf(cfg);
        kf.init_from_measurement(100.0f, 100.0f, 10.0f);
        for (int k = 0; k < 40; ++k) { // 先收敛
            kf.predict(1.0f / 90.0f);
            TargetMeasurement m{};
            m.valid = true;
            m.cx = 100.0f;
            m.cy = 100.0f;
            m.radius = 10.0f;
            m.quality = 1.0f;
            kf.update(m);
        }
        const float x0 = kf.x();
        kf.predict(1.0f / 90.0f);
        const float x_pred = kf.x();
        TargetMeasurement out{};
        out.valid = true;
        out.cx = x_pred + 80.0f; // 突然跳到 80px 之外
        out.cy = kf.y();
        out.radius = 10.0f;
        out.quality = 1.0f;
        const bool accepted = kf.update(out);
        CHECK(!accepted, "80px 的离群测量必须被马氏门限拒收（d²=%.1f）", static_cast<double>(kf.mahalanobis2()));
        CHECK(std::fabs(kf.x() - x_pred) < 1e-4f, "被拒之后状态不能被动过：%.4f vs %.4f",
              static_cast<double>(kf.x()), static_cast<double>(x_pred));
        CHECK(kf.reject_streak() == 1, "拒收应被连续计数（streak=%u）", kf.reject_streak());
        // 下一帧来一个正常测量：streak 必须清零（发散保护只在"连续"拒收时起作用）
        kf.predict(1.0f / 90.0f);
        TargetMeasurement good{};
        good.valid = true;
        good.cx = kf.x();
        good.cy = kf.y();
        good.radius = 10.0f;
        good.quality = 1.0f;
        CHECK(kf.update(good), "随后的正常测量必须被接受");
        CHECK(kf.reject_streak() == 0, "接受测量后连续拒收计数必须清零");
        (void)x0;
        ok("马氏门限拒收离群测量 + streak 计数");
    }

    // ---- 5.5 **R_scale 自适应**：远距离靠模型抑制尺寸抖动，近距离贴合真实轮廓变化 ----
    // 两个子项分别对应规格里的两句话：
    //   a) 抖动抑制：测量里带**同一幅度**的尺寸噪声（白噪声）→ 远档（R 大）必须更平滑
    //   b) 贴合变化：目标"开始逼近"（膨胀速率突然变化）+ 很小的噪声 → 近档（R 小）滞后更小
    // 尺度取 4px（远档）与 45px（近档）：后者就是板端实测的近距离目标
    // （DEBUG_GUIDE §4.1：稳定期单帧 6725 个白像素 → r≈46px）。
    {
        KfConfig     cfg;
        const float  dt_s = 1.0f / 90.0f;
        const float  S_FAR = 4.0f, S_NEAR = 45.0f;
        const float  noise_sigma = 0.8f; // 同一个绝对噪声，两个工况都要能通过门限

        std::mt19937                    rng(20250918);
        std::normal_distribution<float> noise(0.0f, noise_sigma);

        // a) 抖动抑制
        auto ripple_run = [&](float s_true, float *ripple_out, int *rejected) {
            ScaleAwareKalman kf(cfg);
            kf.init_from_measurement(300.0f, 180.0f, s_true);
            std::vector<float> hist;
            *rejected = 0;
            const int N = 500;
            for (int k = 0; k < N; ++k) {
                const float z = s_true + noise(rng);
                kf.predict(dt_s);
                TargetMeasurement m{};
                m.valid = true;
                m.cx = kf.x();
                m.cy = kf.y();
                m.radius = z;
                m.quality = 1.0f;
                if (!kf.update(m))
                    ++*rejected;
                if (k >= N - 150)
                    hist.push_back(kf.s());
            }
            double mean = 0;
            for (float v : hist)
                mean += v;
            mean /= static_cast<double>(hist.size());
            double var = 0;
            for (float v : hist)
                var += (v - mean) * (v - mean);
            *ripple_out = static_cast<float>(std::sqrt(var / static_cast<double>(hist.size())));
        };

        float ripple_far = 0, ripple_near = 0;
        int   rej_far = 0, rej_near = 0;
        ripple_run(S_FAR, &ripple_far, &rej_far);
        ripple_run(S_NEAR, &ripple_near, &rej_near);
        CHECK(rej_far == 0 && rej_near == 0, "噪声在主滤波带宽内时不该被拒（远 %d / 近 %d）", rej_far,
              rej_near);
        CHECK(ripple_far < ripple_near * 0.5f,
              "远距离（大 R_scale）必须把尺寸抖动压得更平：ripple %.4f vs 近 %.4f",
              static_cast<double>(ripple_far), static_cast<double>(ripple_near));

        // b) 贴合真实变化：先恒定，再突然以固定速率变大（"开始逼近"），比较滞后
        auto lag_run = [&](float s0, float *lag_out) {
            ScaleAwareKalman kf(cfg);
            kf.init_from_measurement(300.0f, 180.0f, s0);
            const float rate = 60.0f; // px/s：膨胀速率从 0 突跳到 60px/s
            float       truth = s0;
            float       lag = 0.0f;
            int         n = 0;
            const int   N = 260;
            for (int k = 0; k < N; ++k) {
                if (k >= 160)
                    truth += rate * dt_s;
                kf.predict(dt_s);
                TargetMeasurement m{};
                m.valid = true;
                m.cx = kf.x();
                m.cy = kf.y();
                m.radius = truth + noise(rng) * 0.2f; // 很小的测量噪声
                m.quality = 1.0f;
                kf.update(m);
                if (k >= 160 && k < 240) { // 速率突跳后的适应期
                    lag += (truth - kf.s());
                    ++n;
                }
            }
            *lag_out = n ? lag / static_cast<float>(n) : 0.0f;
        };

        float lag_far = 0, lag_near = 0;
        lag_run(S_FAR, &lag_far);
        lag_run(S_NEAR, &lag_near);
        CHECK(lag_near < lag_far,
              "近距离（小 R_scale）必须更快跟上真实膨胀：适应期滞后 %.3f vs 远 %.3fpx",
              static_cast<double>(lag_near), static_cast<double>(lag_far));

        std::printf("       抖动抑制(白噪声 σ=%.1fpx)：远 ripple %.4fpx / 近 ripple %.4fpx\n",
                    static_cast<double>(noise_sigma), static_cast<double>(ripple_far),
                    static_cast<double>(ripple_near));
        std::printf("       贴合变化(速率 0→60px/s)：远 适应期滞后 %.3fpx / 近 %.3fpx\n",
                    static_cast<double>(lag_far), static_cast<double>(lag_near));
        ok("R_scale 远稳/近跟（本项就是两个工况的核心差别）");
    }

    // ---- 5.7 发散保护：连续被拒时 P 必须长大（否则滤波器会锁死在"跟不上"的状态）----
    // 场景：目标突然加速，而 R_scale 很小（近距离），新息远超门限。
    // 期望：连续拒收几步之后门限被 P 撑开放宽，测量重新被接受并追上 —— 而不是一直拒到底。
    {
        KfConfig         cfg;
        ScaleAwareKalman kf(cfg);
        kf.init_from_measurement(300.0f, 180.0f, 45.0f); // 近档：R_scale=0.6（σ≈0.8px）
        // 先让滤波器收敛（尺度稳定、P 收紧）—— 突变必须发生在"已经稳定"之后，
        // 否则初始的宽松协方差会把突跳照单全收，测不出发散保护。
        for (int k = 0; k < 120; ++k) {
            kf.predict(1.0f / 90.0f);
            TargetMeasurement m{};
            m.valid = true;
            m.cx = kf.x();
            m.cy = kf.y();
            m.radius = 45.0f;
            m.quality = 1.0f;
            kf.update(m);
        }
        CHECK(kf.reject_streak() == 0 && kf.sigma_s() < 1.0f, "突变前必须先收敛（σ_s=%.3f）",
              static_cast<double>(kf.sigma_s()));

        const float rate = 60.0f; // 膨胀速率突然变成 60px/s
        float       truth = 45.0f;
        int         accepted_after_maneuver = -1;
        int         rejects = 0;
        const float p_before = static_cast<float>(kf.covariance()(4, 4));
        for (int k = 0; k < 40; ++k) {
            truth += rate / 90.0f;
            kf.predict(1.0f / 90.0f);
            TargetMeasurement m{};
            m.valid = true;
            m.cx = kf.x();
            m.cy = kf.y();
            m.radius = truth;
            m.quality = 1.0f;
            const bool ok2 = kf.update(m);
            if (!ok2)
                ++rejects;
            if (ok2 && accepted_after_maneuver < 0 && k > 0)
                accepted_after_maneuver = k;
        }
        CHECK(rejects > 0, "突变初期必须出现过拒收（否则测的不是发散保护，实得 %d）", rejects);
        CHECK(accepted_after_maneuver >= 0 && accepted_after_maneuver < 20,
              "发散保护必须在 20 帧内让测量重新被接受（实得第 %d 帧）", accepted_after_maneuver);
        CHECK(std::fabs(kf.s() - truth) < 1.5f, "追上之后尺度必须贴合真值：%.2f vs %.2f",
              static_cast<double>(kf.s()), static_cast<double>(truth));
        std::printf("       突变膨胀 60px/s（P 尺度方差 收敛后 %.4f）：拒收 %d 帧，第 %d 帧重新咬住，"
                    "末态尺度 %.2f（真值 %.2f）\n",
                    static_cast<double>(p_before), rejects, accepted_after_maneuver,
                    static_cast<double>(kf.s()), static_cast<double>(truth));
        ok("拒收发散保护（连续拒收 → 放大 P → 重新咬住目标）");
    }

    // ---- 5.6 **抗差性**：尺度测量连续大幅跳变时，估计不能被带偏 ----
    // 这条固化的是一段真实观察（调试期用仪表打出来的）：交替的大幅尺度跳变不会被
    // 马氏门限拒掉，而是被 vs 吸收 —— 因为 vs 在状态里，能把"有规律的变化"预测掉。
    // 这正是把 s/vs 放进 6 维状态（而不是每帧按面积重新开窗）的收益之一。
    {
        KfConfig         cfg;
        ScaleAwareKalman kf(cfg);
        kf.init_from_measurement(300.0f, 180.0f, 30.0f);
        double sum = 0;
        int    n = 0;
        for (int k = 0; k < 200; ++k) {
            kf.predict(1.0f / 90.0f);
            TargetMeasurement m{};
            m.valid = true;
            m.cx = kf.x();
            m.cy = kf.y();
            m.radius = 30.0f + ((k % 2 == 0) ? 4.5f : -4.5f); // ±15% 的交替跳变
            m.quality = 1.0f;
            kf.update(m);
            if (k >= 100) {
                sum += kf.s();
                ++n;
            }
        }
        const double mean = sum / n;
        CHECK(std::fabs(mean - 30.0) < 2.0, "估计均值必须仍贴着真值 30（实得 %.3f）", mean);
        std::printf("       ±15%% 交替尺度跳变 200 帧后，估计均值 %.3f（真值 30），vs=%.1fpx/s\n", mean,
                    static_cast<double>(kf.vs()));
        ok("连续大幅跳变不把尺度估计带偏（vs 吸收可预测部分）");
    }
}

// ===========================================================================
// 6) ROI 预测公式
// ===========================================================================
void test_roi() {
    SECTION("6) RoiPredictor 动态 ROI");
    RoiConfig        cfg;
    RoiPredictor     rp(cfg);
    const uint32_t   W = 640, H = 360;

    // 规格公式：kσ=0 时必须严格等于 kp·s + B_margin
    RoiConfig exact = cfg;
    exact.k_sigma = 0.0f;
    RoiPredictor rp_exact(exact);
    for (float s : {2.0f, 5.0f, 12.0f, 30.0f}) {
        const float expect = cfg.kp * s + cfg.margin;
        CHECK(std::fabs(rp_exact.side_for(s, 7.0f) - expect) < 1e-4f,
              "kσ=0 时边长必须严格等于 kp·s+B_margin：s=%.1f 得 %.3f 期望 %.3f", static_cast<double>(s),
              static_cast<double>(rp_exact.side_for(s, 7.0f)), static_cast<double>(expect));
    }

    // kσ>0 时窗口随预测不确定度变大
    CHECK(rp.side_for(10.0f, 5.0f) > rp.side_for(10.0f, 0.5f), "kσ 项应随 σ_pred 增长");

    // 面面内的窗口：中心跟随预测点、尺寸正确
    const RoiWindow w = rp.for_tracking(320.0f, 180.0f, 10.0f, 0.5f, 0.5f, 0, false, W, H);
    CHECK(!w.full_frame, "跟踪态应是 ROI 而不是整幅");
    CHECK(std::fabs(static_cast<float>(w.w) - rp.side_for(10.0f, 0.5f)) <= 1.0f,
          "窗口宽应≈side_for（%u vs %.2f）", w.w, static_cast<double>(rp.side_for(10.0f, 0.5f)));
    CHECK(w.x <= 320 && 320 < w.right() && w.y <= 180 && 180 < w.bottom(), "预测点必须在窗口内");

    // 贴边：窗口必须被整体平移进画面（不是被截掉一半）
    const RoiWindow edge = rp.for_tracking(4.0f, 4.0f, 12.0f, 0.5f, 0.5f, 0, false, W, H);
    CHECK(edge.clamped, "贴边时应标记 clamped");
    CHECK(edge.x == 0 && edge.y == 0, "贴边窗口应被平移进画面（x=%u y=%u）", edge.x, edge.y);
    CHECK(edge.right() <= W && edge.bottom() <= H, "窗口不得越界");
    const RoiWindow edge2 = rp.for_tracking(638.0f, 358.0f, 12.0f, 0.5f, 0.5f, 0, false, W, H);
    CHECK(edge2.right() <= W && edge2.bottom() <= H, "右下贴边也不得越界（%u,%u）", edge2.right(),
          edge2.bottom());

    // ---- 近距离目标（板端 run1 的实测场景）：**必须仍然是 ROI** ----
    // 板端 640x360 上目标 r≈40px 时窗口 = kp·ŝ+margin ≈ 224px；旧实现按"任一维 ≥ 0.6×画幅"
    // 判整幅，高方向阈值只有 216px → 一律退化成全图（run1 实测 全图 7350 / ROI 196 帧）。
    // 这条用例就是那次回归的守门人。
    {
        const RoiWindow near = rp.for_tracking(320.0f, 180.0f, 40.0f, 1.0f, 1.0f, 0, false, W, H);
        CHECK(!near.full_frame, "r=40px 的近距离目标必须仍用 ROI，不能退化成全幅");
        const float area_frac = static_cast<float>(near.w) * static_cast<float>(near.h) /
                                (static_cast<float>(W) * static_cast<float>(H));
        CHECK(area_frac < cfg.rescan_area_frac,
              "近距离窗口面积占比 %.1f%% 应小于回退阈值 %.0f%%（否则等于没省）",
              static_cast<double>(area_frac) * 100.0, static_cast<double>(cfg.rescan_area_frac) * 100.0);
        CHECK(near.w >= 200 && near.h >= 200, "窗口边长应约 kp·ŝ+margin≈224px（实得 %ux%u）", near.w, near.h);

        // 丢失后放大到一定程度转整幅是**设计行为**（放大是为了把"跑出窗口"和"真没了"分开），
        // 这里只守住"放大确实起作用"，别把回退阈值调成"一丢就整幅"：
        const RoiWindow near_lost = rp.for_tracking(320.0f, 180.0f, 40.0f, 1.0f, 1.0f, 1, false, W, H);
        CHECK(near_lost.w >= near.w && near_lost.h >= near.h, "丢失态窗口只许变大（%ux%u → %ux%u）",
              near.w, near.h, near_lost.w, near_lost.h);

        // 但真正的大块（round12 的手：r≈88px → 窗口 464px，占掉半个画面）仍应回退整幅
        const RoiWindow huge = rp.for_tracking(320.0f, 180.0f, 88.0f, 1.0f, 1.0f, 0, false, W, H);
        CHECK(huge.full_frame, "r=88px 的巨块仍应回退成整幅扫描");

        // 单维夹取但面积仍小 → 不该整幅（"按面积判"相对"按单维判"的收益）：
        // x 方向不确定度极大（窗口宽被夹到画幅），y 方向很确定 → 仍是条窄带，值得只扫它
        const RoiWindow band = rp.for_tracking(320.0f, 180.0f, 5.0f, 300.0f, 5.0f, 0, false, W, H);
        CHECK(!band.full_frame, "宽被夹到画幅、高很小的一条带不该整幅（%ux%u）", band.w, band.h);
        CHECK(band.bottom() <= H && band.right() <= W, "夹取后仍不得越界");
        std::printf("       近距离 ROI: r=40 → %ux%u（占整幅 %.1f%%）；r=88 → %s\n", near.w, near.h,
                    static_cast<double>(area_frac) * 100.0, huge.full_frame ? "整幅" : "ROI");
    }

    // 丢失态：窗口逐帧放大，放大到尽头直接整幅
    const RoiWindow l0 = rp.for_tracking(320.0f, 180.0f, 10.0f, 0.5f, 0.5f, 0, false, W, H);
    const RoiWindow l3 = rp.for_tracking(320.0f, 180.0f, 10.0f, 0.5f, 0.5f, 3, false, W, H);
    CHECK(l3.w > l0.w, "丢失态窗口必须逐帧放大（%u → %u）", l0.w, l3.w);
    const RoiWindow l99 = rp.for_tracking(320.0f, 180.0f, 10.0f, 0.5f, 0.5f, 99, false, W, H);
    CHECK(l99.full_frame, "放大到尽头必须转成整幅扫描");
    // 超框提示：下一帧再放大一档
    const RoiWindow oor = rp.for_tracking(320.0f, 180.0f, 10.0f, 0.5f, 0.5f, 0, true, W, H);
    CHECK(oor.w > l0.w, "上一帧超框时窗口应放大（%u → %u）", l0.w, oor.w);
    ok("W=kp·s+B_margin(+kσ·σ) / 贴边平移 / 丢失放大 / 整幅回退");
}

// ===========================================================================
// 7) 整链：启动 → 跟踪 → 丢失 → 重捕 → 硬复位
// ===========================================================================
namespace {

// 合成一段"目标逼近"的序列：既能跑真二值图，又不依赖 RVV/相机/真实时钟。
struct Sim {
    Scene                             scene;
    uint64_t                          now_us = 0;
    uint64_t                          dt_us;
    int                               frame = 0;
    dart::detection::DetectionPipeline pipe;

    Sim(uint32_t w, uint32_t h, const DetectionConfig &cfg, uint64_t dt_us_ = 11111)
        : scene(w, h), dt_us(dt_us_), pipe(cfg, deps()) {}

    // 假时钟：测试必须完全确定性，绝不能依赖真实单调钟
    dart::detection::DetectionPipeline::Deps deps() {
        dart::detection::DetectionPipeline::Deps d;
        d.now_us = [this]() { return now_us; };
        return d;
    }

    DetectResult step() {
        now_us += dt_us;
        ++frame;
        return pipe.detect(scene.view());
    }
};

struct Recorder_ {
    std::vector<std::pair<int, TrackState>> trans;
    TrackState                              last = TrackState::Startup;
    void note(int frame, TrackState s) {
        if (frame == 1) {
            last = s;
            return;
        }
        if (s != last) {
            trans.emplace_back(frame, s);
            last = s;
        }
    }
    bool has(TrackState to) const {
        for (const auto &t : trans)
            if (t.second == to)
                return true;
        return false;
    }
    int first(TrackState to) const {
        for (const auto &t : trans)
            if (t.second == to)
                return t.first;
        return -1;
    }
};

} // namespace

void test_chain() {
    SECTION("7) 整链：启动→跟踪→丢失→重捕→复位");

    DetectionConfig cfg; // 默认参数（与板端一致）
    const uint32_t  W = 640, H = 360;
    Sim             sim(W, H, cfg);
    Recorder_       rec;

    // 目标状态：以固定速度平移 + 匀速膨胀（"目标会动，并会均匀变大"）
    float tx = 100.0f, ty = 80.0f, tr = 5.0f;
    const float vx = 2.0f, vy = 0.8f, vr = 0.05f; // px/帧

    int   track_frames = 0, roi_frames_at_track = 0, full_at_track = 0;
    float worst_pos_err = 0.0f, worst_r_err = 0.0f;
    bool  roi_contains_target = true;
    std::vector<float> roi_ws; // 跟踪态每帧的窗口宽（用来验证"目标变大 → ROI 变大"）

    // ---- 阶段 A：目标出现并持续逼近（150 帧）----
    for (int k = 0; k < 150; ++k) {
        sim.scene.clear();
        sim.scene.disk(tx, ty, tr);
        const Truth truth = truth_of(sim.scene);
        const DetectResult r = sim.step();
        rec.note(k, static_cast<TrackState>(r.state));

        if (r.state == static_cast<uint8_t>(TrackState::Tracking)) {
            ++track_frames;
            if (r.cx >= 0) {
                const float perr = std::fabs(static_cast<float>(r.cx) - static_cast<float>(truth.cx)) +
                                   std::fabs(static_cast<float>(r.cy) - static_cast<float>(truth.cy));
                worst_pos_err = perr > worst_pos_err ? perr : worst_pos_err;
                const float rerr = std::fabs(r.radius - truth.radius());
                worst_r_err = rerr > worst_r_err ? rerr : worst_r_err;
            } else {
                worst_pos_err = 999.0f; // 跟踪态却没目标：直接判失败
            }
            const bool is_full = (r.roi_x0 == 0 && r.roi_y0 == 0 && r.roi_x1 == W - 1 && r.roi_y1 == H - 1);
            if (is_full) {
                ++full_at_track; // 跟踪态不该整幅扫描
            } else {
                ++roi_frames_at_track;
                roi_ws.push_back(static_cast<float>(r.roi_x1 - r.roi_x0 + 1));
                // 窗口是闭区间：目标质心必须落在 [x0,x1] × [y0,y1] 内
                if (!(static_cast<float>(r.roi_x0) <= truth.cx &&
                      truth.cx <= static_cast<float>(r.roi_x1) &&
                      static_cast<float>(r.roi_y0) <= truth.cy &&
                      truth.cy <= static_cast<float>(r.roi_y1)))
                    roi_contains_target = false;
            }
        }
        tx += vx;
        ty += vy;
        tr += vr;
    }

    // 前 20 帧 / 后 20 帧的平均窗口宽（目标半径从 5px 长到 ~12px）
    float roi_w_first = 0.0f, roi_w_last = 0.0f;
    if (roi_ws.size() >= 40) {
        for (size_t i = 0; i < 20; ++i) {
            roi_w_first += roi_ws[i];
            roi_w_last += roi_ws[roi_ws.size() - 1 - i];
        }
        roi_w_first /= 20.0f;
        roi_w_last /= 20.0f;
    }

    CHECK(rec.first(TrackState::Tracking) >= 0, "阶段 A 必须进入跟踪态");
    CHECK(rec.first(TrackState::Tracking) <= static_cast<int>(cfg.confirm.window) + 1,
          "滑窗确认应在 %u 帧内完成，实得第 %d 帧", cfg.confirm.window, rec.first(TrackState::Tracking));
    CHECK(track_frames >= 140, "150 帧里至少 140 帧应处于跟踪态，实得 %d", track_frames);
    CHECK(roi_frames_at_track >= 140, "跟踪态必须只在 ROI 内扫描（ROI 帧 %d）", roi_frames_at_track);
    // 允许 1 次全幅：**确认成功的那一帧本身就是全图扫描**（那时还不知道目标在哪），
    // 它的结果状态已经是"跟踪"。除此之外跟踪态不该再整幅扫。
    CHECK(full_at_track <= 1, "跟踪态除确认帧外不该出现全幅扫描（出现 %d 次）", full_at_track);
    CHECK(roi_contains_target, "跟踪态的目标必须始终落在扫描窗口内");
    CHECK(worst_pos_err < 3.0f, "跟踪位置误差应 <3px（最差 %.2f）", static_cast<double>(worst_pos_err));
    CHECK(worst_r_err < 2.0f, "跟踪尺度误差应 <2px（最差 %.2f）", static_cast<double>(worst_r_err));
    CHECK(roi_w_last > roi_w_first + 5.0f,
          "目标变大时 ROI 必须跟着变大（前 %.1fpx → 后 %.1fpx）", static_cast<double>(roi_w_first),
          static_cast<double>(roi_w_last));
    std::printf("       跟踪 %d 帧，位置最差误差 %.2fpx，尺度最差误差 %.2fpx，ROI 宽 %.0f→%.0fpx\n",
                track_frames, static_cast<double>(worst_pos_err), static_cast<double>(worst_r_err),
                static_cast<double>(roi_w_first), static_cast<double>(roi_w_last));

    // ---- 阶段 B：目标消失（8 帧）→ 必须进丢失态 ----
    for (int k = 0; k < 8; ++k) {
        sim.scene.clear();
        const DetectResult r = sim.step();
        rec.note(150 + k, static_cast<TrackState>(r.state));
    }
    CHECK(rec.has(TrackState::Lost), "目标消失后必须进入丢失态");
    CHECK(rec.first(TrackState::Lost) >= 150 + static_cast<int>(cfg.tracker.lost_after) - 1,
          "丢失不应早于 lost_after=%u 帧（第 %d 帧就丢了）", cfg.tracker.lost_after,
          rec.first(TrackState::Lost));

    // ---- 阶段 C：目标回到原处 → 应在有限帧内重捕 ----
    int reacquire_frame = -1;
    for (int k = 0; k < 24; ++k) {
        sim.scene.clear();
        sim.scene.disk(tx, ty, tr);
        const DetectResult r = sim.step();
        rec.note(158 + k, static_cast<TrackState>(r.state));
        if (r.state == static_cast<uint8_t>(TrackState::Tracking) && reacquire_frame < 0) {
            reacquire_frame = k;
            CHECK(r.cx >= 0, "重捕当帧就该给出位置");
        }
        tx += vx;
        ty += vy;
        tr += vr;
    }
    CHECK(reacquire_frame >= 0, "目标回来后必须能重捕（24 帧内没回到跟踪态）");
    CHECK(sim.pipe.stats().tracker.reacquire_soft + sim.pipe.stats().tracker.reacquire_hard > 0,
          "重捕必须被计数（软 %llu / 硬 %llu）",
          static_cast<unsigned long long>(sim.pipe.stats().tracker.reacquire_soft),
          static_cast<unsigned long long>(sim.pipe.stats().tracker.reacquire_hard));
    std::printf("       重捕在第 %d 帧（目标回来后）\n", reacquire_frame);

    // ---- 阶段 D：长时间消失 → 必须硬复位回启动态 ----
    for (int k = 0; k < 70; ++k) {
        sim.scene.clear();
        const DetectResult r = sim.step();
        rec.note(182 + k, static_cast<TrackState>(r.state));
    }
    CHECK(rec.has(TrackState::Startup), "长时间丢失必须硬复位回启动态");
    CHECK(sim.pipe.stats().tracker.to_startup > 0, "硬复位必须被计数（%llu）",
          static_cast<unsigned long long>(sim.pipe.stats().tracker.to_startup));

    // ---- 阶段 E：再出现 → 启动态确认后重新进入跟踪 ----
    const int startup_first = rec.first(TrackState::Startup);
    int       retrack_frame = -1;
    for (int k = 0; k < 12; ++k) {
        sim.scene.clear();
        sim.scene.disk(tx, ty, tr);
        const DetectResult r = sim.step();
        rec.note(252 + k, static_cast<TrackState>(r.state));
        if (r.state == static_cast<uint8_t>(TrackState::Tracking) && retrack_frame < 0)
            retrack_frame = k;
        tx += vx;
        ty += vy;
        tr += vr;
    }
    CHECK(startup_first > 0, "阶段 D 之后应记录到启动态");
    CHECK(retrack_frame >= 0 && retrack_frame <= static_cast<int>(cfg.confirm.window) + 2,
          "硬复位后重新确认应在 %u 帧内完成（实得 %d）", cfg.confirm.window, retrack_frame);

    std::printf("       状态迁移链：");
    for (const auto &t : rec.trans)
        std::printf(" [第%d帧→%s]", t.first, to_string(t.second));
    std::printf("\n");
    ok("启动→跟踪→丢失→重捕→硬复位→再跟踪");

    // ---- 8) A/B 模式：关掉跟踪就永远走全图扫描（回归对照用）----
    SECTION("8) --no-track 对照（全图扫描 + 每帧确认）");
    DetectionConfig cfg2;
    cfg2.enable_tracking = false;
    Sim sim2(W, H, cfg2);
    int found_after = -1;
    for (int k = 0; k < 12; ++k) {
        sim2.scene.clear();
        sim2.scene.disk(200.0f + 2.0f * k, 150.0f, 6.0f);
        const DetectResult r = sim2.step();
        CHECK(static_cast<TrackState>(r.state) == TrackState::Startup,
              "关掉跟踪后状态必须一直是启动态（第 %d 帧得到 %s）", k, to_string(static_cast<TrackState>(r.state)));
        CHECK(r.roi_x0 == 0 && r.roi_y0 == 0 && r.roi_x1 == W - 1 && r.roi_y1 == H - 1,
              "关掉跟踪后必须每帧整幅扫描（角坐标应为 (0,0)-(%u,%u)，实得 (%u,%u)-(%u,%u)）", W - 1, H - 1,
              r.roi_x0, r.roi_y0, r.roi_x1, r.roi_y1);
        if (r.cx >= 0 && found_after < 0)
            found_after = k;
    }
    CHECK(found_after == static_cast<int>(cfg2.confirm.window) - 1,
          "全图模式应在第 %u 帧给出目标（实得第 %d 帧）", cfg2.confirm.window, found_after);
    CHECK(sim2.pipe.stats().tracker.roi_scans == 0, "A/B 模式下不该有 ROI 扫描（%llu）",
          static_cast<unsigned long long>(sim2.pipe.stats().tracker.roi_scans));
    ok("A/B 对照模式");
}

int main() {
    std::printf("=== 识别/跟踪层主机侧回归（无 RVV / 无相机 / 假时钟）===\n");
    test_linalg();
    test_scanner();
    test_measurer();
    test_confirmer();
    test_kalman();
    test_roi();
    test_chain();
    std::printf("\n=== 合计 %d 项检查，失败 %d 项 ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
