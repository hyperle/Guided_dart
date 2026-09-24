#!/usr/bin/env python3
"""识别/跟踪结果复盘：frames.csv 的最后一公里。

板端跑完一轮后，最值钱的证据是 `dart/frames.csv`（每帧一行）。这个工具把它读成
**可核对的判据**，而不是让人去几百行日志里找结论：

  * 状态机时间线：启动/跟踪/丢失 各段从第几帧到第几帧、各占多少秒
  * 跟踪质量：跟踪态命中率、位置轨迹的连续性、等效半径 r 的膨胀速率（"均匀变大"拟合）
  * ROI 门控：ROI 尺寸分布、占整幅比例、是否真的一直只扫 ROI、**是否始终框住目标**
  * 时序健康：真实帧率（源帧 pts 间隔）、断档、重复帧
  * 耗时：识别整链 cost_us 的分位数 vs 帧预算
  * 矛盾检查：state=跟踪 却 cx=-1、质心落在 ROI 之外、ROI 越界 …

用法：
  python3 scripts/detect_review.py /media/$USER/SDCARD/dart            # 出报告
  python3 scripts/detect_review.py <卡>/dart --csv-only                # 只看判据表
  python3 scripts/detect_review.py --selftest                          # 合成数据自测

列（main.cpp 的 emit_csv 写、evidence_writer 写表头）：
  seq, mono_ms, src_pts, d_pts_us, exp_us, cx, cy, roi, cost_us, captured_fps, pbm,
  radius, state, roi_x, roi_y, roi_w, roi_h
  # state: 0=启动 1=跟踪 2=丢失；radius 是等效半径 r=sqrt(A/π)，无目标为 -1
  # 老版 CSV 没有最后 6 列（DetectorStub 时代），本工具会自动跳过相关判据
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import tempfile
from pathlib import Path

STATE_NAME = {0: "启动", 1: "跟踪", 2: "丢失"}
FRAME_W_DEFAULT = 640  # 画幅（--frame 可改，报告里按 CSV 里的 ROI 推断，这里只做兜底）


# --------------------------------------------------------------------------- IO

def _num(row: dict, key: str, cast=float):
    v = row.get(key, "")
    if v is None or v == "":
        return None
    try:
        return cast(v)
    except (TypeError, ValueError):
        return None


def find_csv(directory: Path) -> Path:
    """frames.csv 现在写在取证根目录（图像在 <dir>/img/），老归档里也可能同层或 img/ 下"""
    for cand in (directory / "frames.csv", directory / "img" / "frames.csv",
                 directory / "csv" / "frames.csv"):
        if cand.exists():
            return cand
    return directory / "frames.csv"


def load_rows(directory: Path) -> list[dict]:
    path = find_csv(directory)
    if not path.exists():
        return []
    out = []
    with path.open(newline="") as fh:
        for i, row in enumerate(csv.DictReader(fh)):
            if not row.get("seq"):
                continue
            rec = {
                "seq": _num(row, "seq", int),
                "mono_ms": _num(row, "mono_ms", int),
                "src_pts": _num(row, "src_pts", int),
                "d_pts_us": _num(row, "d_pts_us", int),
                "cx": _num(row, "cx", int),
                "cy": _num(row, "cy", int),
                "roi": _num(row, "roi", int),
                "cost_us": _num(row, "cost_us", int),
                "radius": _num(row, "radius"),
                "circ": _num(row, "circ"), # 圆度（4πA/P² 的替代定义，见 types.hpp::roundness_from_moments）
                "state": _num(row, "state", int),
                # 扫描窗口：新 CSV 直接给**四个角像素坐标**（闭区间）；老 CSV 是 (x,y,w,h)。
                # 这里统一换算成角坐标，后面的分析只认角（归档里的 round11/round12 都是老格式）。
                "roi_x0": _num(row, "roi_x0", int),
                "roi_y0": _num(row, "roi_y0", int),
                "roi_x1": _num(row, "roi_x1", int),
                "roi_y1": _num(row, "roi_y1", int),
                "_roi_x": _num(row, "roi_x", int),
                "_roi_y": _num(row, "roi_y", int),
                "_roi_w": _num(row, "roi_w", int),
                "_roi_h": _num(row, "roi_h", int),
                "fps_field": _num(row, "captured_fps"),
            }
            rec["_i"] = i
            # 老格式换算（roi_x/roi_y 是左上角，roi_w/roi_h 是边长 → 闭区间右/下角 = x+w-1）
            if rec["roi_x0"] is None and rec["_roi_x"] is not None and rec["_roi_w"] is not None:
                rec["roi_x0"] = rec["_roi_x"]
                rec["roi_y0"] = rec["_roi_y"]
                rec["roi_x1"] = rec["_roi_x"] + rec["_roi_w"] - 1
                rec["roi_y1"] = rec["_roi_y"] + rec["_roi_h"] - 1
            out.append(rec)
    # 时间轴：用源帧 pts 间隔累加出来的微秒轴（毫秒粒度不够看 11ms 帧间隔的抖动）
    t = 0.0
    for rec in out:
        d = rec["d_pts_us"] or 0
        rec["t_us"] = t
        t += d
    # 没有 d_pts_us（第一帧恒为 0）时退回 mono_ms
    if out and all((r["d_pts_us"] or 0) == 0 for r in out[1:]):
        t0 = out[0]["mono_ms"] or 0
        for rec in out:
            rec["t_us"] = float((rec["mono_ms"] or 0) - t0) * 1000.0
    return out


def roi_box(r: dict):
    """本帧扫描窗口的四角（闭区间）(x0,y0,x1,y1)；没有该信息返回 None"""
    if r.get("roi_x0") is None:
        return None
    return (r["roi_x0"], r["roi_y0"], r["roi_x1"], r["roi_y1"])


def roi_size(r: dict) -> tuple[int, int]:
    b = roi_box(r)
    return (b[2] - b[0] + 1, b[3] - b[1] + 1) if b else (0, 0)


def frame_size(rows: list[dict]) -> tuple[int, int]:
    """推断画幅：**取所有帧**里窗口角坐标的最大值。
    不能只看跟踪态 —— 启动态（以及丢失重扫）的窗口就是整幅 (0,0)-(W-1,H-1)，
    所以"取全部帧的最大值"才是画幅；只看跟踪态会把启动态的全幅窗口误判成越界。"""
    boxes = [b for b in (roi_box(r) for r in rows) if b]
    if not boxes:
        return (0, 0)
    return (max(b[2] for b in boxes) + 1, max(b[3] for b in boxes) + 1)


def has_track_columns(rows: list[dict]) -> bool:
    return any(r["state"] is not None for r in rows)


# ---------------------------------------------------------------------- 计算

def segments(rows: list[dict]) -> list[tuple[int, int, int]]:
    """连续同状态段 -> [(state, 起始下标, 结束下标闭区间)]"""
    out = []
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["state"] != rows[start]["state"]:
            out.append((rows[start]["state"], start, i - 1))
            start = i
    return out


def linfit(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """最小二乘 y = a + b x，返回 (a, b, R²)"""
    n = len(xs)
    if n < 3:
        return (0.0, 0.0, 0.0)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0:
        return (my, 0.0, 0.0)
    b = sxy / sxx
    a = my - b * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    r2 = 0.0 if ss_tot <= 0 else max(0.0, 1.0 - ss_res / ss_tot)
    return (a, b, r2)


def pct(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


# ---------------------------------------------------------------------- 报告

class Report:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.verdicts: list[tuple[str, str, str]] = []  # (级别, 判据, 说明)

    def say(self, text: str = "") -> None:
        self.lines.append(text)

    def check(self, ok: bool, name: str, detail: str, warn_only: bool = False) -> None:
        level = "ok" if ok else ("注意" if warn_only else "FAIL")
        self.verdicts.append((level, name, detail))

    def dump(self, csv_only: bool) -> int:
        if not csv_only:
            for line in self.lines:
                print(line)
        print()
        print("=== 判据 ===")
        width = max((len(n) for _, n, _ in self.verdicts), default=0)
        for level, name, detail in self.verdicts:
            print(f"  [{level:>4}] {name:<{width}}  {detail}")
        fails = sum(1 for lv, _, _ in self.verdicts if lv == "FAIL")
        print(f"\n=== {len(self.verdicts)} 项判据，FAIL {fails} 项 ===")
        return 1 if fails else 0


def analyse(directory: Path, csv_only: bool) -> int:
    rows = load_rows(directory)
    rep = Report()
    if not rows:
        print(f"!! 没有 CSV 数据：{directory / 'frames.csv'} 缺失或为空")
        return 1

    have_track = has_track_columns(rows)
    n = len(rows)
    t_end_s = rows[-1]["t_us"] / 1e6
    dts = [r["d_pts_us"] for r in rows[1:] if r["d_pts_us"]]
    fps = (1e6 / statistics.median(dts)) if dts else 0.0

    rep.say(f"=== 识别/跟踪复盘: {directory}（CSV: {find_csv(directory)}）===")
    rep.say(f"帧数 {n}，seq {rows[0]['seq']}..{rows[-1]['seq']}，时长 {t_end_s:.1f}s，"
            f"真实帧率 {fps:.1f} fps（源帧 pts 间隔中位数 {statistics.median(dts) if dts else 0:.0f}us）")
    if not have_track:
        rep.say("!! 这份 CSV 没有跟踪列（radius/state/roi_*）—— 是 DetectorStub 时代的旧数据，"
                "只能出帧率/耗时判据")
        rep.check(fps >= 70, "真实帧率≥70fps", f"{fps:.1f} fps（round10 基线 82~88）", warn_only=True)
        costs = [r["cost_us"] for r in rows if r["cost_us"] is not None]
        if costs:
            rep.say(f"识别整链 cost_us: p50={pct(costs,50):.0f} p95={pct(costs,95):.0f} max={max(costs)}")
        return rep.dump(csv_only)

    # ---- 状态时间线 ----
    segs = segments(rows)
    rep.say()
    rep.say("状态时间线（段内为该状态连续帧）：")
    for state, a, b in segs:
        dur = (rows[b]["t_us"] - rows[a]["t_us"]) / 1e6
        rep.say(f"  第{rows[a]['seq']:>5}~{rows[b]['seq']:<5} 帧  {STATE_NAME.get(state,'?'):<4} "
                f"{b-a+1:>5} 帧 / {dur:6.2f}s" +
                (f"   r={rows[a]['radius']:.2f}→{rows[b]['radius']:.2f}px"
                 if rows[a]["radius"] is not None and rows[b]["radius"] is not None else ""))
    counts = {}
    for state, a, b in segs:
        counts[state] = counts.get(state, 0) + (b - a + 1)
    share = "  ".join(f"{STATE_NAME.get(k,'?')} {v/n*100:.1f}%" for k, v in sorted(counts.items()))
    rep.say(f"  占空比：{share}")

    tracking = [r for r in rows if r["state"] == 1]
    startup = [r for r in rows if r["state"] == 0]
    first_track = next((r for r in rows if r["state"] == 1), None)

    rep.check(bool(tracking), "进入过跟踪态",
              f"跟踪帧 {len(tracking)}" if tracking else "整轮都在启动/丢失 —— 目标没被确认（看日志的【全图候选/拒闪】）",
              warn_only=True)
    if first_track is not None:
        confirm_frames = first_track["_i"]
        rep.check(confirm_frames <= 8, "确认延迟（启动阶段）",
                  f"第 {confirm_frames + 1} 帧进入跟踪（滑窗 3 帧 + 关联，≤8 帧属正常）", warn_only=True)

    # ---- 命中率 ----
    def hit_rate(rs):
        if not rs:
            return 0.0
        return sum(1 for r in rs if (r["cx"] or -1) >= 0) / len(rs) * 100.0

    rep.say()
    rep.say(f"命中率：跟踪态 {hit_rate(tracking):.1f}%（{len(tracking)} 帧）  "
            f"启动态 {hit_rate(startup):.1f}%（{len(startup)} 帧）")
    if tracking:
        rep.check(hit_rate(tracking) >= 95.0, "跟踪态命中率≥95%", f"{hit_rate(tracking):.1f}%")

    # ---- 位置轨迹连续性 + 速度 ----
    if tracking:
        pos = [(r["t_us"], r["cx"], r["cy"]) for r in tracking if (r["cx"] or -1) >= 0]
        steps = []
        for (t0, x0, y0), (t1, x1, y1) in zip(pos, pos[1:]):
            dt = (t1 - t0) / 1e6
            if dt > 1e-6:
                steps.append(math.hypot(x1 - x0, y1 - y0) / dt)
        if steps:
            med = statistics.median(steps)
            jumps = [s for s in steps if s > max(4 * med, 50.0)]
            rep.say(f"跟踪速度：中位数 {med:.1f} px/s（最大 {max(steps):.1f}）")
            rep.check(len(jumps) <= max(2, len(steps) // 100), "位置轨迹无异常跳变",
                      f"{len(jumps)}/{len(steps)} 次超过中位数 4 倍（多数说明有离群测量混进来）",
                      warn_only=True)

        # 圆度（调 min_circularity 的依据）：目标块的圆度分布
        cvals = [r["circ"] for r in tracking if r["circ"] is not None and r["circ"] >= 0.0]
        if cvals:
            cvals.sort()
            med_c = cvals[len(cvals) // 2]
            rep.say(f"圆度（目标块，0~1）：中位 {med_c:.2f}  p10 {cvals[len(cvals)//10]:.2f}  "
                    f"最小 {cvals[0]:.2f}（细长反光通常 <0.3，圆目标 0.75~0.95）")
            rep.check(med_c > 0.6, "目标块的圆度中位 >0.6",
                      f"中位 {med_c:.2f}；若这一列普遍偏低，说明挑到的不是圆斑（可加大 --circ-weight，"
                      f"或把 --min-circ 设成 {max(0.2, med_c * 0.6):.2f} 把长条直接筛掉）", warn_only=True)

        rvals = [r["radius"] for r in tracking if r["radius"] is not None and r["radius"] > 0]
        if len(rvals) >= 3:
            ts = [r["t_us"] / 1e6 for r in tracking if r["radius"] is not None and r["radius"] > 0]
            a, b, r2 = linfit(ts, rvals)
            rep.say(f"尺度 r：{rvals[0]:.2f} → {rvals[-1]:.2f}px，线性拟合斜率 {b:+.2f} px/s（R²={r2:.3f}）")
            rep.check(b > 0.05, "尺度在均匀变大", f"斜率 {b:+.2f} px/s（正=逼近；R² {r2:.2f} 表示是否均匀）",
                      warn_only=True)

    # ---- ROI 门控 ----
    if tracking:
        widths = [roi_size(r)[0] for r in tracking]
        heights = [roi_size(r)[1] for r in tracking]
        areas = [w * h for w, h in (roi_size(r) for r in tracking)]
        full = [r for r in tracking if roi_size(r)[0] >= 600 and roi_size(r)[1] >= 340]
        frame_w, frame_h = frame_size(rows)
        frame_px = frame_w * frame_h
        rep.say()
        rep.say(f"ROI（四个角像素坐标，闭区间）：宽 {min(widths)}~{max(widths)}px，"
                f"中位 {pct([float(w) for w in widths], 50):.0f}px；高 {min(heights)}~{max(heights)}px；"
                f"面积占整幅中位 {pct([float(a) for a in areas], 50) / frame_px * 100:.1f}%")
        b = roi_box(tracking[0])
        rep.say(f"  首帧窗口 ({b[0]},{b[1]})-({b[2]},{b[3]})")
        # 确认那一帧必然是整幅扫描（那时还不知道目标在哪），只允许每段跟踪的首帧
        firsts = {a for _, a, _ in segs if rows[a]["state"] == 1}
        bad_full = [r for r in full if r["_i"] not in firsts]
        rep.check(not bad_full, "跟踪态只在 ROI 内扫描",
                  f"除每段首帧外出现 {len(bad_full)} 次全幅扫描" + (f"（首个第 {bad_full[0]['seq']} 帧）" if bad_full else ""))
        outside = []
        for r in tracking:
            b = roi_box(r)
            if (r["cx"] or -1) < 0 or b is None:
                continue
            if not (b[0] <= r["cx"] <= b[2] and b[1] <= r["cy"] <= b[3]):
                outside.append(r)
        rep.check(not outside, "目标始终落在扫描窗口内",
                  f"{len(outside)} 帧的目标质心在窗口外" + (f"（首个第 {outside[0]['seq']} 帧）" if outside else ""))
        oob = []
        for r in rows:
            b = roi_box(r)
            if b is None:
                continue
            if b[2] >= frame_w or b[3] >= frame_h or b[0] > b[2] or b[1] > b[3]:
                oob.append(r)
        rep.check(not oob, "扫描窗口不越界", f"{len(oob)} 帧越界（索引越界会读到别的内存）")

    # ---- 矛盾/异常 ----
    rep.say()
    dup = sum(1 for a, b in zip(rows, rows[1:]) if a["src_pts"] and a["src_pts"] == b["src_pts"])
    gaps = [r["seq"] for r in rows[1:] if (r["d_pts_us"] or 0) > 1e6]
    rep.check(dup == 0, "无重复帧（src_pts 判据）", f"{dup} 处相邻帧同一 pts")
    rep.check(len(gaps) <= 2, "无长断档", f"{len(gaps)} 次间隔 >1s（首个第 {gaps[0]} 帧）" if gaps else "无 >1s 间隔",
              warn_only=True)
    neg_when_track = sum(1 for r in tracking if (r["cx"] or -1) < 0)
    rep.check(neg_when_track <= max(3, len(tracking) // 50), "跟踪态无目标的比例很低",
              f"{neg_when_track}/{len(tracking)} 帧（跟踪态丢测量会走丢失流程，偶发正常）", warn_only=True)

    # ---- 耗时 ----
    costs = [r["cost_us"] for r in rows if r["cost_us"] is not None]
    if costs:
        budget = 1e6 / fps if fps > 0 else 0
        rep.say(f"识别整链 cost_us：p50={pct(costs,50):.0f} p95={pct(costs,95):.0f} max={max(costs):.0f}"
                f"（帧预算 {budget:.0f}us @ {fps:.0f}fps）")
        rep.check(pct(costs, 95) < budget * 0.5 if budget else True, "识别耗时 < 帧预算的一半",
                  f"p95 {pct(costs,95):.0f}us vs 预算 {budget:.0f}us", warn_only=True)

    rep.check(fps >= 70, "真实帧率≥70fps", f"{fps:.1f} fps（round10 基线 82~88；掉了说明识别吃了预算）",
              warn_only=True)
    return rep.dump(csv_only)


# -------------------------------------------------------------------- selftest

def selftest() -> int:
    """造一份合成 frames.csv（含 启动→跟踪→丢失→跟踪 与"均匀变大"），校验工具判据。"""
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp)
        with (d / "frames.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["seq", "mono_ms", "src_pts", "d_pts_us", "exp_us", "cx", "cy", "roi",
                        "cost_us", "captured_fps", "pbm", "radius", "state",
                        "roi_x", "roi_y", "roi_w", "roi_h"])
            t = 0
            pts = 0
            seq = 0
            r = 4.0
            x, y = 200.0, 150.0
            plan = [(0, 2), (1, 60), (2, 6), (1, 40)]  # 状态脚本
            for state, cnt in plan:
                for _ in range(cnt):
                    seq += 1
                    dt = 11111
                    pts += dt
                    t += 11
                    if state == 1:
                        x += 3.0
                        y += 1.0
                        r += 0.06           # 0.06px/帧 ≈ 5.4px/s 的均匀膨胀
                    found = state == 1
                    w.writerow([seq, t, pts, dt if seq > 1 else 0, 250,
                                int(x) if found else -1, int(y) if found else -1,
                                0 if found else 1, 150, 90.0, 0,
                                f"{r:.2f}" if found else "-1",
                                state, int(x) - 30 if found else 0, int(y) - 30 if found else 0,
                                60 if found else 640, 60 if found else 360])

        rows = load_rows(d)
        ok = True

        def expect(cond, what):
            nonlocal ok
            print(("  [ ok ] " if cond else "  [FAIL] ") + what)
            ok = ok and cond

        expect(len(rows) == 108, f"读入 {len(rows)} 行（期望 108）")
        expect(has_track_columns(rows), "识别出跟踪列存在")
        segs = segments(rows)
        expect([s[0] for s in segs] == [0, 1, 2, 1], f"状态分段 {[s[0] for s in segs]}（期望 0,1,2,1）")
        ts = [r["t_us"] / 1e6 for r in rows if r["state"] == 1]
        rs = [r["radius"] for r in rows if r["state"] == 1]
        _, slope, r2 = linfit(ts, rs)
        expect(4.5 < slope < 6.5, f"膨胀速率拟合 {slope:.2f} px/s（期望 ≈5.4）")
        expect(r2 > 0.99, f"均匀变大 R²={r2:.4f}")
        fps = 1e6 / statistics.median([r["d_pts_us"] for r in rows[1:]])
        expect(abs(fps - 90.0) < 0.5, f"帧率 {fps:.1f}（期望 90）")
        # 判据表本身必须全绿
        import io
        from contextlib import redirect_stdout
        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = analyse(d, csv_only=True)
        expect(rc == 0, "合成数据判据全绿（analyse 返回 0）")
        for line in buf.getvalue().splitlines():
            print("      " + line)
    print("\n=== selftest " + ("通过" if ok else "失败") + " ===")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="识别/跟踪结果复盘（frames.csv）")
    ap.add_argument("directory", nargs="?", type=Path, help="含 frames.csv 的目录（板端 /sdcard/dart）")
    ap.add_argument("--csv-only", action="store_true", help="只打判据表")
    ap.add_argument("--selftest", action="store_true", help="用合成数据自测")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.directory:
        ap.error("请给一个含 frames.csv 的目录，或用 --selftest")
    return analyse(args.directory, args.csv_only)


if __name__ == "__main__":
    sys.exit(main())
