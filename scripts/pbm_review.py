#!/usr/bin/env python3
"""板端取证数据（PBM 序列 + frames.csv）的主机侧回放与校验。

板端那台机器上 VENC 不可用（手工送帧两条编码路径都拿不到输出，PicCnt 恒为 0），
所以"录像"改由应用自己落盘：每 N 帧一张二值化 PBM 图 + 每帧一行 CSV。这个工具
把这两样东西合成为可回放的 mp4，并给出**可核对的量**：

  * 真实帧率：来自源 VICAP 帧的 pts 间隔（d_pts_us），不是"自己数循环次数"
  * 重复帧：相邻帧稀疏哈希相同 → 同一帧被重复返回
  * 断档：seq / 文件名不连续
  * 二值化效果：PBM 直接就是识别器看到的图（可选叠加识别结果的十字与文字）

用法：
  # 只出报告
  python3 scripts/pbm_review.py /media/hyperlee/SDCARD/dart --check
  # 合成 mp4（默认叠加识别结果）
  python3 scripts/pbm_review.py /media/hyperlee/SDCARD/dart --mp4 review.mp4
  # 自测：造一份合成数据把整条管线跑一遍
  python3 scripts/pbm_review.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import shutil
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    from PIL import Image, ImageDraw
except ImportError:  # --check 不需要 PIL；只有合成 mp4 才需要
    Image = None
    ImageDraw = None


# --------------------------------------------------------------------------- IO

def load_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline="") as fh:
        return [row for row in csv.DictReader(fh) if row.get("seq")]


def pbm_files(directory: Path) -> list[tuple[int, Path]]:
    out = []
    for p in sorted(directory.glob("f*.pbm")):
        try:
            out.append((int(p.stem[1:]), p))
        except ValueError:
            continue
    return out


# ------------------------------------------------------------------------ check

def cmd_check(directory: Path, fps_override: float | None) -> int:
    rows = load_csv(directory / "frames.csv")
    pbms = pbm_files(directory)
    print(f"目录: {directory}")
    print(f"CSV 行数: {len(rows)}    PBM 张数: {len(pbms)}")

    if not rows:
        print("!! 没有 CSV 数据（frames.csv 缺失或为空）")
        return 1

    seqs = [int(r["seq"]) for r in rows]
    pbm_flag = sum(1 for r in rows if r.get("pbm") == "1")
    print(f"帧号范围: {seqs[0]} .. {seqs[-1]}    标记存图的行: {pbm_flag}")
    gaps = [b - a for a, b in zip(seqs, seqs[1:]) if b - a != 1]
    print(f"帧号断档: {len(gaps)} 处" + (f"（例如 {gaps[:5]}）" if gaps else ""))

    # 真实帧间隔来自源帧 pts
    dpts = [int(r["d_pts_us"]) for r in rows if r.get("d_pts_us") and int(r["d_pts_us"]) > 0]
    if dpts:
        mean = statistics.fmean(dpts)
        dpts_sorted = sorted(dpts)
        p99 = dpts_sorted[min(len(dpts_sorted) - 1, int(len(dpts_sorted) * 0.99))]
        print(f"源帧间隔 pts: 均值 {mean:.1f} us → {1e6 / mean:.2f} fps"
              f"    最小 {min(dpts)} us（{1e6 / min(dpts):.1f} fps）"
              f"    最大 {max(dpts)} us    p99 {p99} us")
        jitter = statistics.pstdev(dpts) if len(dpts) > 1 else 0.0
        print(f"抖动(标准差): {jitter:.1f} us")
    else:
        print("!! CSV 里没有可用的 d_pts_us（源帧 pts 为 0？）")

    loops = [float(r["captured_fps"]) for r in rows if r.get("captured_fps")]
    if loops:
        print(f"自数循环帧率 captured_fps: 均值 {statistics.fmean(loops):.2f} fps"
              f"（对照上面的 pts 帧率，两者接近才说明没丢帧/没重复）")

    # 真·重复帧：src_pts 与上一帧相同 = 同一张采集帧被返回两次
    # （不要用稀疏哈希判：二值图内容恒定时哈希必然相同，那是误报）
    pts = [r.get("src_pts", "") for r in rows]
    dup = sum(1 for a, b in zip(pts, pts[1:]) if a and a == b)
    print(f"真·重复帧(src_pts 相同): {dup} 处" + ("  ← 有重复" if dup else "  ✓ 每帧都是新的"))

    # 曝光分档（--exposure-sweep 时会填 exp_us）
    exps = [r.get("exp_us", "") for r in rows if r.get("exp_us")]
    if exps and any(e not in ("", "0") for e in exps):
        from collections import Counter
        cnt = Counter(exps)
        print("曝光分档(us: 帧数): " + "  ".join(f"{k}:{v}" for k, v in sorted(cnt.items(), key=lambda kv: int(kv[0]))))

    if pbms:
        pseq = [s for s, _ in pbms]
        step = statistics.median([b - a for a, b in zip(pseq, pseq[1:])]) if len(pseq) > 1 else 1
        missing = [s for s in pseq if s not in set(seqs)]
        print(f"PBM 帧号: {pseq[0]} .. {pseq[-1]}，步长中位数 {step:g}")
        print(f"PBM 未被 CSV 覆盖: {len(missing)} 张")
        size = pbms[0][1].stat().st_size
        print(f"单张 PBM 字节: {size}（640x360 应为 11 + 640/8*360 = {11 + 80 * 360}）")
        if dpts and step:
            print(f"推出 PBM 序列帧率 ≈ {1e6 / (statistics.fmean(dpts) * step):.2f} fps"
                  f"（源帧率 / 步长 {step:g}）")
    return 0


# ----------------------------------------------------------------------- render

def _draw_overlay(img, row: dict) -> None:
    draw = ImageDraw.Draw(img)
    try:
        cx, cy = int(row["cx"]), int(row["cy"])
    except (KeyError, ValueError):
        cx = cy = -1
    if cx >= 0 and cy >= 0:  # 识别中心：十字
        draw.line((cx - 12, cy, cx + 12, cy), fill=128)
        draw.line((cx, cy - 12, cx, cy + 12), fill=128)
        draw.rectangle((cx - 16, cy - 16, cx + 16, cy + 16), outline=128)
    text = (f"F{int(row['seq']):06d} T{int(row['mono_ms']):09d} "
            f"X{cx:04d} Y{cy:04d} R{row.get('roi', '?')} D{row.get('cost_us', '?')}")
    draw.text((2, 2), text, fill=255)


def cmd_render(directory: Path, out: Path, fps: float | None, overlay: bool,
               dump_overlay: Path | None = None) -> int:
    if Image is None:
        print("!! 需要 Pillow（pip install pillow）才能合成 mp4")
        return 1
    rows = load_csv(directory / "frames.csv")
    by_seq = {int(r["seq"]): r for r in rows}
    pbms = pbm_files(directory)
    if not pbms:
        print("!! 没有 PBM 图，无法合成 mp4")
        return 1

    if fps is None:
        fps = 10.0
        dpts = [int(r["d_pts_us"]) for r in rows if r.get("d_pts_us") and int(r["d_pts_us"]) > 0]
        pseq = [s for s, _ in pbms]
        if dpts and len(pseq) > 1:
            step = statistics.median([b - a for a, b in zip(pseq, pseq[1:])])
            if step:
                fps = 1e6 / (statistics.fmean(dpts) * step)
    print(f"合成 mp4: {out}   {len(pbms)} 帧 @ {fps:.2f} fps   叠加识别结果: {'是' if overlay else '否'}")

    first = Image.open(pbms[0][1])
    w, h = first.size

    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        print("!! 找不到 ffmpeg")
        return 1

    cmd = [ffmpeg, "-y", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "gray", "-s", f"{w}x{h}", "-r", f"{fps:.6f}",
           "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
           "-movflags", "+faststart", str(out)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    for idx, (seq, path) in enumerate(pbms):
        # PBM 的 1 = 黑（我们的 save_pbm 就是这么写的），PIL 读 "1" 模式后 0/255，
        # 其中 255 表示白 —— 直接转 "L" 就是识别器看到的图（亮处白）。
        img = Image.open(path).convert("L")
        if img.size != (w, h):
            img = img.resize((w, h))
        if overlay and seq in by_seq:
            _draw_overlay(img, by_seq[seq])
        if idx == 0 and dump_overlay is not None:
            img.save(dump_overlay)  # 第一帧存成 PNG：方便直接肉眼核对二值化与标注
            print(f"已导出首帧带标注 PNG: {dump_overlay}")
        proc.stdin.write(img.tobytes())
    proc.stdin.close()
    rc = proc.wait()
    if rc != 0:
        print(f"!! ffmpeg 返回 {rc}")
        return rc
    print(f"完成：{out}（{out.stat().st_size / 1024:.0f} KB）")
    return 0


# ---------------------------------------------------------------------- selftest

def _write_pbm(path: Path, w: int, h: int, bright_left: bool) -> None:
    """按板上 save_pbm 的规则写：bit=1 表示暗像素。"""
    row_bytes = (w + 7) // 8
    with path.open("wb") as fh:
        fh.write(f"P4\n{w} {h}\n".encode())
        for _ in range(h):
            packed = bytearray(row_bytes)
            for x in range(w):
                dark = (x >= w // 2) if bright_left else (x < w // 2)
                if dark:
                    packed[x >> 3] |= 0x80 >> (x & 7)
            fh.write(packed)


def cmd_selftest() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="pbm_review_selftest_"))
    w, h, n, step = 640, 360, 24, 9
    dpts = 11111  # ~90 fps
    for i in range(n):
        seq = 1 + i * step
        _write_pbm(tmp / f"f{seq:06d}.pbm", w, h, bright_left=(i % 2 == 0))
    with (tmp / "frames.csv").open("w", newline="") as fh:
        wtr = csv.writer(fh)
        wtr.writerow(["seq", "mono_ms", "src_pts", "d_pts_us", "cx", "cy", "roi", "cost_us",
                      "y_hash", "captured_fps", "pbm"])
        for i in range(n * step):
            seq = i + 1
            saved = 1 if (seq % step) == 1 else 0
            wtr.writerow([seq, i * 11, i * dpts, 0 if i == 0 else dpts, 320, 180, 0, 900,
                          f"{i & 0xffff:08x}", 90.0, saved])

    print("=== 自测：合成数据 ===")
    rc = cmd_check(tmp, None)
    if rc != 0:
        return rc

    out = tmp / "review.mp4"
    rc = cmd_render(tmp, out, None, True, dump_overlay=tmp / "first_frame.png")
    if rc != 0:
        return rc
    if out.stat().st_size < 4096:
        print("!! mp4 太小，可疑")
        return 1

    # 极性检查：第一帧左半亮（255）右半暗（0）
    img = Image.open(tmp / "f000001.pbm").convert("L")
    left = img.getpixel((w // 4, h // 2))
    right = img.getpixel((w * 3 // 4, h // 2))
    ok_pol = left > 200 and right < 50
    print(f"极性检查: 左半(亮)={left} 右半(暗)={right} → {'✓ 正确' if ok_pol else '✗ 反了'}")

    # 叠加检查：导出的首帧 PNG 里中心附近应出现标记（灰 128）
    over = Image.open(tmp / "first_frame.png").convert("L")
    marked = sum(1 for x in range(320 - 20, 320 + 21) if over.getpixel((x, 180)) == 128)
    print(f"叠加标记像素数: {marked}（>0 即画上了）")
    print(f"临时目录: {tmp}")
    return 0 if (ok_pol and marked > 0) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="板端 PBM+CSV 取证数据的回放与校验")
    ap.add_argument("directory", nargs="?", type=Path, help="含 f*.pbm 与 frames.csv 的目录")
    ap.add_argument("--check", action="store_true", help="只出校验报告")
    ap.add_argument("--mp4", type=Path, help="合成 mp4 的输出路径")
    ap.add_argument("--fps", type=float, help="输出帧率（默认按 pts 与步长推断）")
    ap.add_argument("--no-overlay", action="store_true", help="不叠加识别结果")
    ap.add_argument("--dump-overlay", type=Path, help="把第一帧（带标注）另存为 PNG，便于肉眼核对")
    ap.add_argument("--selftest", action="store_true", help="用合成数据自测整条管线")
    args = ap.parse_args()

    if args.selftest:
        return cmd_selftest()
    if args.directory is None:
        ap.error("需要目录参数（或用 --selftest）")
    if not args.directory.is_dir():
        print(f"!! 目录不存在: {args.directory}")
        return 1

    rc = 0
    if args.check or args.mp4 is None:
        rc |= cmd_check(args.directory, args.fps)
    if args.mp4 is not None:
        rc |= cmd_render(args.directory, args.mp4, args.fps, not args.no_overlay, args.dump_overlay)
    return rc


if __name__ == "__main__":
    sys.exit(main())
