// ============================================================================
// 装甲板识别（detection/armor.hpp）主机侧回归：合成二值图 + 假锚，不需要相机/RVV/SDK。
//
//   bash tests/host/run.sh armor
//
// 为什么先写它：这套东西的坑几乎全在"纯逻辑"里 —— 交叉连线接反、交点在延长线上还
// 当真、远档拿量化噪声当形状判据、跟踪期逐帧换人、把保持值当成新测量。
// 这些在板子上只表现为"偶尔跳一下"，在主机上却能一条一条钉死。
// ============================================================================

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "core/frame.hpp"
#include "detection/armor.hpp"
#include "detection/config.hpp"
#include "detection/pipeline.hpp"

using namespace dart;
using namespace dart::detection;

namespace {

// ArmorConfig::flat_deg 的默认值：手工构造的 Bar 没有主轴（len=0）时，
// bar_ends/armor_center 会走包围盒口径 —— 那正是"竖直工况零回归"的路径。
constexpr float kFlat = 12.0f;

int g_fail = 0;

void check(const char *name, bool ok, const std::string &extra = std::string()) {
    std::printf("  %-52s %s %s\n", name, ok ? "PASS" : "FAIL", extra.c_str());
    if (!ok)
        ++g_fail;
}

// ---- 合成二值图：把"两条竖灯条"画进一张全黑图（值 255）----
struct Scene {
    uint32_t              w = 640, h = 480;
    std::vector<uint8_t>  px;
    Scene() : px(static_cast<size_t>(640) * 480, 0) {}
    void rect(int x0, int y0, int x1, int y1) {
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                px[static_cast<size_t>(y) * w + x] = 255;
    }
    // 画一个**真正旋转**的矩形（不是"包围盒近似"）：长边从竖直起算偏 deg 度。
    // 这是"灯条平行但不竖直"这个工况的唯一诚实造法 —— 包围盒会自己变成 24x37 那种。
    void rot_rect(float ccx, float ccy, float L, float t, float deg) {
        const float r = deg * 3.14159265358979f / 180.0f;
        const float ux = std::sin(r), uy = -std::cos(r); // 长边方向
        const float vx = std::cos(r), vy = std::sin(r);  // 厚度方向
        const int   i0 = static_cast<int>(ccx - L), i1 = static_cast<int>(ccx + L);
        const int   j0 = static_cast<int>(ccy - L), j1 = static_cast<int>(ccy + L);
        for (int y = (j0 < 0 ? 0 : j0); y <= j1 && y < static_cast<int>(h); ++y) {
            for (int x = (i0 < 0 ? 0 : i0); x <= i1 && x < static_cast<int>(w); ++x) {
                const float dx = static_cast<float>(x) + 0.5f - ccx;
                const float dy = static_cast<float>(y) + 0.5f - ccy;
                if (std::fabs(dx * ux + dy * uy) <= L * 0.5f &&
                    std::fabs(dx * vx + dy * vy) <= t * 0.5f)
                    px[static_cast<size_t>(y) * w + x] = 255;
            }
        }
    }
    GrayFrame view() {
        GrayFrame f{};
        f.pixels = px.data();
        f.width = w;
        f.height = h;
        f.stride = w;
        return f;
    }
};

// 一次完整的"检测 + 跟踪"：绿灯锚放在 (cx, cy)，灯尺 = 2r
ArmorTarget run(ArmorDetector &det, const GrayFrame &f, float cx, float cy, float r,
                bool fresh_input = true) {
    TrackOutput led{};
    led.found = fresh_input;
    led.state = TrackState::Tracking;
    led.cx = cx;
    led.cy = cy;
    led.radius = r;
    ArmorTarget t{};
    det.detect(f, led, 0, &t);
    return t;
}

} // namespace

int main() {
    std::printf("=== 装甲板识别：主机侧回归 ===\n");

    // ------------------------------------------------------------------
    // 1) 纯几何：板心 = 两条**交叉**连线的交点
    // ------------------------------------------------------------------
    {
        // A(300,100 6x40) B(340,100 6x40)：A上(303,100) B下(343,139) → 交点 (323,119~120)
        ArmorDetector::Bar a{}, b{};
        a.x = 300; a.y = 100; a.w = 6; a.h = 40;
        b.x = 340; b.y = 100; b.w = 6; b.h = 40;
        int32_t cx = 0, cy = 0;
        const bool ok = ArmorDetector::armor_center(a, b, 0.30f, kFlat, &cx, &cy);
        check("等高竖灯条 → 板心 = 两灯条中点", ok && cx == 323 && (cy == 119 || cy == 120),
              "得到 (" + std::to_string(cx) + "," + std::to_string(cy) + ")");

        // 长度不等：交点仍在两段之内（不能跑到延长线上）
        ArmorDetector::Bar b2{};
        b2.x = 340; b2.y = 110; b2.w = 6; b2.h = 40;
        const bool ok2 = ArmorDetector::armor_center(a, b2, 0.30f, kFlat, &cx, &cy);
        check("长度不等 → 交点仍在两灯条之间", ok2 && cx == 323 && cy > 100 && cy < 150,
              "得到 (" + std::to_string(cx) + "," + std::to_string(cy) + ")");

        // 各自长轴求交：永远平行 → 必须没有交点（这条写错过一次）
        int32_t ax0, ay0, ax1, ay1, bx0, by0, bx1, by1;
        ArmorDetector::bar_ends(a, kFlat, &ax0, &ay0, &ax1, &ay1);
        ArmorDetector::bar_ends(b, kFlat, &bx0, &by0, &bx1, &by1);
        check("拿各自长轴求交 → 无交点(平行)",
              !ArmorDetector::line_intersection(ax0, ay0, ax1, ay1, bx0, by0, bx1, by1, 0.30f,
                                                &cx, &cy));

        // 共线 / 近平行 → 无交点；交点在延长线上 → 无交点
        check("共线两段 → 无交点",
              !ArmorDetector::line_intersection(0, 0, 10, 0, 20, 0, 30, 0, 0.30f, &cx, &cy));
        check("近平行两段 → 无交点",
              !ArmorDetector::line_intersection(0, 0, 100, 0, 0, 1, 100, 1, 0.30f, &cx, &cy));
        check("交点在段外 → 无交点",
              !ArmorDetector::line_intersection(0, 0, 10, 0, 20, 0, 20, 10, 0.30f, &cx, &cy));

        // 横灯条：长轴端点取左右边中点
        ArmorDetector::Bar hb{};
        hb.x = 100; hb.y = 50; hb.w = 40; hb.h = 6;
        ArmorDetector::bar_ends(hb, kFlat, &ax0, &ay0, &ax1, &ay1);
        check("横灯条端点 = 左右边中点", ax0 == 100 && ay0 == 53 && ax1 == 139 && ay1 == 53);
    }

    // ------------------------------------------------------------------
    // 2) 变阈值：同一判据、同一 blobs，远档与近档结论不同
    // ------------------------------------------------------------------
    {
        DetectionConfig cfg;
        ArmorConfig     ac = cfg.armor;
        ArmorDetector   det(ac, cfg.measure);

        TrackOutput led{};
        led.found = true;
        led.state = TrackState::Tracking;

        // 近档：灯尺 s=2r=120 → 门限 len 24..960；6x60 的长度 60 通过，20x40 的长度 40 通过
        // 但 20x40 的圆度 0.95 在近档不看圆度、看长宽比（40/20=2 >= 2）→ 也过。
        // 这里只验"门限随尺度变"这件事本身：远档 6x60 会被 len_max 挡住。
        Scene far;
        far.rect(300, 100, 305, 159);  // 6x60
        far.rect(340, 100, 345, 159);
        led.cx = 323.0f; led.cy = 260.0f; led.radius = 2.0f;  // 灯尺 s=4 → 远档
        const ArmorTarget tf = run(det, far.view(), led.cx, led.cy, led.radius);
        check("远档(s=4): 6x60 的大亮条超出 len_max(8·s=32) → 不算灯条",
              !tf.found(), "bars=" + std::to_string(det.last_trace().bars));

        det.reset();
        Scene near;
        near.rect(300, 100, 305, 159);
        near.rect(340, 100, 345, 159);
        led.cx = 323.0f; led.cy = 260.0f; led.radius = 30.0f; // 灯尺 s=60 → 近档
        const ArmorTarget tn = run(det, near.view(), led.cx, led.cy, led.radius);
        check("近档(s=60): 同一对灯条 → 配成对并出板心",
              tn.found() && tn.mode == ArmorTarget::Fresh,
              "bars=" + std::to_string(det.last_trace().bars) +
                  " 中心=(" + std::to_string(tn.cx) + "," + std::to_string(tn.cy) + ")");
    }

    // ------------------------------------------------------------------
    // 3) 远距离也能识别：灯条只有 3×1 px（"类似噪点"那一档）
    // ------------------------------------------------------------------
    {
        DetectionConfig cfg;
        ArmorDetector   det(cfg.armor, cfg.measure);
        Scene           sc;
        const int       y0 = 300, len = 3;
        sc.rect(300, y0, 300, y0 + len - 1);       // 1x3
        sc.rect(308, y0, 308, y0 + len - 1);
        // 灯尺 s=4（r=2），绿灯上沿 y=320：窗口 = 绿灯上方一块（地板 24px）
        const ArmorTarget t = run(det, sc.view(), 304.0f, 322.0f, 2.0f);
        check("远档: 3x1 px 的两片灯条 → 仍能配成对",
              t.found() && t.mode == ArmorTarget::Fresh,
              "bars=" + std::to_string(det.last_trace().bars) +
                  " 板心=(" + std::to_string(t.cx) + "," + std::to_string(t.cy) + ")");
        check("远档: 窗口落到地板尺寸(不随灯尺缩小到 0)",
              det.last_trace().window_px >= 24u * 24u,
              "window_px=" + std::to_string(det.last_trace().window_px));
    }

    // ------------------------------------------------------------------
    // 4) 灯条被窗口边界削到 → 长度只是下界：配对打分要罚它
    // ------------------------------------------------------------------
    {
        DetectionConfig cfg;
        ArmorDetector   det(cfg.armor, cfg.measure);
        Scene           sc;
        // 一条完整灯条 + 一条贴着窗口下沿被削掉的灯条（画到窗口外）
        sc.rect(300, 200, 305, 320);
        sc.rect(340, 200, 345, 320);
        const ArmorTarget t = run(det, sc.view(), 323.0f, 327.0f, 30.0f);
        check("配对仍能给出结果（贴边只是罚分，不是硬否）", t.found(),
              "bars=" + std::to_string(det.last_trace().bars));
    }

    // ------------------------------------------------------------------
    // 5) 检测-跟踪：静止画面下不许丢、不许换人；�leaving短暂丢失先保持
    // ------------------------------------------------------------------
    {
        DetectionConfig cfg;
        ArmorDetector   det(cfg.armor, cfg.measure);
        Scene           sc;
        sc.rect(300, 100, 305, 159);
        sc.rect(340, 100, 345, 159);
        const GrayFrame f = sc.view();
        const ArmorTarget t1 = run(det, f, 323.0f, 260.0f, 30.0f);
        check("首帧 → Fresh(本帧实测)", t1.found() && t1.mode == ArmorTarget::Fresh);

        // 复制一份"同样内容"的帧，保证输入不随跟踪状态变化
        Scene sc2;
        sc2.rect(300, 100, 305, 159);
        sc2.rect(340, 100, 345, 159);
        const GrayFrame f2 = sc2.view();
        bool all_fresh = true, shrinking = true;
        uint32_t prev_px = det.last_trace().window_px;
        for (int k = 0; k < 5; ++k) {
            const ArmorTarget t = run(det, f2, 323.0f, 260.0f, 30.0f);
            if (!t.found() || t.mode != ArmorTarget::Fresh)
                all_fresh = false;
            if (det.last_trace().window_px > prev_px)
                shrinking = false;
            prev_px = det.last_trace().window_px;
        }
        check("静止 5 帧 → 全部 Fresh(不丢)", all_fresh);
        check("跟踪态搜索窗比整块小(省时间)", shrinking && prev_px > 0,
              "window_px=" + std::to_string(prev_px));

        // 灯条全消失 → 先 Held（带年龄），超过 hold 帧后转为 None
        Scene empty;
        const GrayFrame fe = empty.view();
        const ArmorTarget h1 = run(det, fe, 323.0f, 260.0f, 30.0f);
        check("灯条消失第 1 帧 → Held(标年龄，不当新测量)",
              h1.found() && h1.mode == ArmorTarget::Held && h1.held == 1,
              "held=" + std::to_string(h1.held));
        ArmorTarget h2 = h1;
        for (uint32_t k = 0; k < cfg.armor.hold; ++k)
            h2 = run(det, fe, 323.0f, 260.0f, 30.0f);
        check("超过 hold 帧 → 释放(None)，不许无限保持", !h2.found(),
              "held=" + std::to_string(h2.held));
    }

    // ------------------------------------------------------------------
    // 5b) 斜灯条（"两条灯条平行但不竖直"）：主轴口径必须替代包围盒口径
    // ------------------------------------------------------------------
    {
        DetectionConfig cfg;
        ArmorDetector   det(cfg.armor, cfg.measure);
        const float     L = 40.0f, t = 5.0f, gap = 44.0f;
        const float     cy_bar = 150.0f, led_y = 240.0f;   // 绿灯在上方窗口之下
        const float     r = 30.0f;                          // 灯尺 s = 60 → 门限 len 12..480
        for (float deg : {0.0f, 15.0f, 30.0f, 40.0f}) {
            det.reset();
            Scene sc;
            sc.rot_rect(320.0f - gap * 0.5f, cy_bar, L, t, deg);
            sc.rot_rect(320.0f + gap * 0.5f, cy_bar, L, t, deg);
            const ArmorTarget at = run(det, sc.view(), 320.0f, led_y, r);
            const bool ok = at.found() && at.mode == ArmorTarget::Fresh &&
                            std::abs(at.cx - 320) <= 2 && std::abs(at.cy - static_cast<int>(cy_bar)) <= 2;
            char extra[128];
            std::snprintf(extra, sizeof(extra), "%g°: bars=%u 板心=(%d,%d) 期望=(320,%d)",
                          static_cast<double>(deg), det.last_trace().bars, at.cx, at.cy,
                          static_cast<int>(cy_bar));
            check("斜灯条: 平行但不竖直 → 仍配成对且板心正确", ok, extra);
        }
        // 反例：倾角差太大（0° 与 30°）不成对；厚度差太大不成对
        det.reset();
        Scene sc2;
        sc2.rot_rect(298.0f, cy_bar, L, t, 0.0f);
        sc2.rot_rect(342.0f, cy_bar, L, t, 30.0f);
        const ArmorTarget at2 = run(det, sc2.view(), 320.0f, led_y, r);
        check("平行判据: 两条倾角差 30° > 15° → 不成对", !at2.found(),
              "bars=" + std::to_string(det.last_trace().bars));
        det.reset();
        Scene sc3;
        sc3.rot_rect(298.0f, cy_bar, L, t, 20.0f);
        sc3.rot_rect(342.0f, cy_bar, L, 25.0f, 20.0f);   // 同样倾斜但粗 5 倍
        const ArmorTarget at3 = run(det, sc3.view(), 320.0f, led_y, r);
        check("厚度判据: 两条粗细差 5 倍 → 不成对", !at3.found(),
              "bars=" + std::to_string(det.last_trace().bars));
    }

    // ------------------------------------------------------------------
    // 6) 锚的语义：绿灯没锚就不出结果（宁可不报，也不凭空开窗）
    // ------------------------------------------------------------------
    {
        DetectionConfig cfg;
        ArmorDetector   det(cfg.armor, cfg.measure);
        Scene           sc;
        sc.rect(300, 100, 305, 159);
        sc.rect(340, 100, 345, 159);
        const ArmorTarget t = run(det, sc.view(), 0.0f, 0.0f, 0.0f, /*fresh_input=*/false);
        check("绿灯没锚 → 装甲板不出结果", !t.found());
    }

    // ------------------------------------------------------------------
    // 6b) --armor-off：整段不跑（不做无谓的连通域，也不出结果）
    // ------------------------------------------------------------------
    {
        DetectionConfig cfg;
        cfg.armor.enable = false;
        DetectionPipeline pipe(cfg);
        Scene             sc;
        sc.rect(300, 100, 305, 159);
        sc.rect(340, 100, 345, 159);
        const GrayFrame f = sc.view();
        const DetectResult r = pipe.detect(f);
        check("armor.enable=false → 装甲板整段不跑（trace 为空）",
              !r.armor.found() && pipe.stats().armor_trace.bars == 0 &&
                  pipe.stats().armor_trace.cost_us == 0);
    }

    // ------------------------------------------------------------------
    // 7) 与既有测量器同口径：collect() 出来的块 = measure() 看的块
    // ------------------------------------------------------------------
    {
        MeasureConfig    mcfg;
        RoiBlobMeasurer  meas(mcfg);
        Scene            sc;
        sc.rect(300, 100, 305, 159);   // 6x60
        sc.rect(340, 100, 345, 159);   // 6x60
        sc.rect(200, 300, 209, 309);   // 10x10 方块（不该算灯条）
        GrayFrame   f = sc.view();
        RoiWindow   win{};
        win.x = 100; win.y = 50; win.w = 400; win.h = 300; win.full_frame = false;
        RoiBlobMeasurer::BlobInfo out[8];
        const uint32_t n = meas.collect(f, win, 2, out, 8);
        check("collect(): 三块都带出来（不是只带最好的那块）", n == 3,
              "n=" + std::to_string(n));
        bool abs_ok = true;
        for (uint32_t i = 0; i < n; ++i)
            if (out[i].x1 >= f.width || out[i].y1 >= f.height)
                abs_ok = false;
        check("collect(): 坐标是**画面绝对坐标**（不用再加窗口偏移）", abs_ok);
        const TargetMeasurement m = meas.measure(f, win);
        // measure() 取的是"面积 × 圆度^circ_weight"最大者（既有行为，不是最大块）：
        // 10x10 方块(100×0.9) 压过 6x60 长条(360×0.15) —— 圆斑优先，长条让位。
        // 我这次重构 blob_measure 只动了"多一个出口"，这个语义必须原样不变。
        check("measure(): 既有语义不变（圆度加权取优 → 方块胜长条）", m.valid && m.area == 100,
              "area=" + std::to_string(m.area));
    }

    std::printf("---\n");
    if (g_fail) {
        std::printf("装甲板回归: FAIL %d 项\n", g_fail);
        return 1;
    }
    std::printf("装甲板回归: 全部 PASS\n");
    return 0;
}
