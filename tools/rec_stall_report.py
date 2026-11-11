#!/usr/bin/env python3
"""Summarise recording stall behaviour and recorder timing from one or more sessions.

Separates input gaps, gaps between successfully recorded source frames, and
buffer-full drops. CSV receipt times describe capture, not disk completion;
recorder timing and queue counters in the application log describe storage.

    python3 tools/rec_stall_report.py /data/app/records/rec_0016
    python3 tools/rec_stall_report.py records/rec_00* --log logs/dart_app.log

Only the standard library is used; copy `rec_NNNN/` and `dart_app.log` back from
the board and run it on the host.
"""

from __future__ import annotations

import argparse
import csv
import io
import statistics
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path


def read_frames(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    header: dict[str, str] = {}
    rows: list[dict[str, str]] = []
    text = path.read_text(encoding="utf-8", errors="replace").splitlines()
    data: list[str] = []
    for line in text:
        if line.startswith("#"):
            for token in line[1:].split():
                if "=" in token:
                    key, value = token.split("=", 1)
                    header[key] = value
            continue
        if line.strip():
            data.append(line)
    if data:
        rows = list(csv.DictReader(io.StringIO("\n".join(data) + "\n")))
    return header, rows


@dataclass
class Summary:
    name: str
    format_note: str = ""
    seconds: float = 0.0
    rows: int = 0
    attempts: int = 0
    accepted: int = 0
    decisions: Counter = field(default_factory=Counter)
    stall_seconds: float = 0.0
    stalls: list[float] = field(default_factory=list)
    output_gaps: list[float] = field(default_factory=list)
    output_gap_limit_ms: float = 0.0
    target_fps: int = 0
    mean_frame_bytes: float = 0.0
    bytes_per_second: float = 0.0
    container_size: int = 0
    container_indexed: int = 0

    @property
    def missing_rows(self) -> int:
        return max(0, self.attempts - self.rows)

    @property
    def effective_fps(self) -> float:
        return self.accepted / self.seconds if self.seconds else 0.0

    @property
    def active_fps(self) -> float:
        active = self.seconds - self.stall_seconds
        return self.accepted / active if active > 0 else 0.0

    @property
    def stall_pct(self) -> float:
        return 100.0 * self.stall_seconds / self.seconds if self.seconds else 0.0


def summarise(record: Path, stall_us: int) -> Summary:
    header, rows = read_frames(record / "logs" / "frames.csv")
    result = Summary(name=record.name)
    if not rows:
        return result
    result.rows = len(rows)
    result.target_fps = int(header.get("fps", "0"))
    result.attempts = max(int(row["input_index"]) for row in rows)
    result.decisions = Counter(row["decision"] for row in rows)
    result.accepted = result.decisions.get("accepted", 0)

    times = sorted(int(row["received_us"]) for row in rows)
    result.seconds = (times[-1] - times[0]) / 1e6
    gaps = [b - a for a, b in zip(times, times[1:])]
    result.stalls = [gap / 1000.0 for gap in gaps if gap > stall_us]
    result.stall_seconds = sum(gap for gap in gaps if gap > stall_us) / 1e6

    accepted_items = [row for row in rows if row["decision"] == "accepted"]
    accepted_times = sorted(int(row["received_us"]) for row in accepted_items)
    # Two target periods avoid treating an intentionally low recording FPS as
    # a stall. The input threshold still applies to all submission attempts.
    output_limit_us = max(stall_us, 2_000_000 / result.target_fps if result.target_fps > 0 else 0)
    result.output_gap_limit_ms = output_limit_us / 1000
    result.output_gaps = [
        (b - a) / 1000 for a, b in zip(accepted_times, accepted_times[1:]) if b - a > output_limit_us
    ]
    sizes = [int(row["file_bytes"]) for row in accepted_items if int(row["file_bytes"]) > 0]
    if sizes:
        result.mean_frame_bytes = statistics.mean(sizes)
        result.bytes_per_second = sum(sizes) / result.seconds if result.seconds else 0.0
        image = accepted_items[0].get("image_file", "")
        result.format_note = Path(image).name if image else ""

    for suffix in ("pngs", "h264", "dbv", "pgm", "pbm"):
        container = record / "img" / f"recording.{suffix}"
        if container.is_file():
            result.container_size = container.stat().st_size
            result.format_note = container.name
            result.container_indexed = max(
                (int(row["file_offset"]) + int(row["file_bytes"]) for row in accepted_items),
                default=0,
            )
            break
    return result


def log_timing(path: Path, limit: int = 0) -> dict[str, list[float]]:
    """Collects the recorder/timing fields the app writes once per status window."""
    wanted = {
        "telemetry_us", "telemetry_max_us", "acquire_max_us", "acquire_us",
        "pack_us", "io_us", "io_max_us", "space_us", "write_us",
        "rec_busy", "rec_queued", "rec_event_dropped", "rec_acc", "rec_down",
        "detect_us", "detect_max_us", "pipeline_us", "pipeline_max_us", "pack_max_us",
        "buffer_copy_us", "buffer_copy_max_us", "open_us", "close_us", "drain_us",
        "rec_buffered", "rec_buffer_high_water", "rec_buffer_dropped", "rec_encoded", "rec_enqueued",
        "rec_written", "rec_timed_out", "rec_write_fail", "rec_session_event_dropped",
        "elapsed_s", "fps",
    }
    series: dict[str, list[float]] = {name: [] for name in wanted}
    if not path.is_file():
        return series
    windows: list[dict[str, float]] = []
    pending: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if " op=" not in line:
            continue
        fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
        for key, value in fields.items():
            if key not in wanted:
                continue
            if key == "fps" and fields.get("op") != "timing":
                continue
            try:
                pending[key] = float(value)
            except ValueError:
                continue
        if fields.get("op") == "timing" and "window_ms" in fields:
            windows.append(pending)
            pending = {}
    # Old logs without timing markers still expose their last counter snapshot.
    if not windows and pending:
        windows.append(pending)
    for window in windows[limit:]:
        for key, value in window.items():
            series[key].append(value)
    return series


def format_series(values: list[float], unit: str) -> str:
    if not values:
        return "-"
    ordered = sorted(values)
    median = statistics.median(ordered)
    return f"median {median:.0f}{unit} p95 {ordered[int(len(ordered) * 0.95)]:.0f}{unit} max {ordered[-1]:.0f}{unit}"


def print_summary(item: Summary, stall_ms: int) -> None:
    print(f"== {item.name} ({item.format_note or '?'})")
    print(
        f"   duration {item.seconds:6.2f}s   rows {item.rows:5d}   attempts {item.attempts:5d}"
        f"   missing_rows {item.missing_rows}"
    )
    decided = ", ".join(f"{key}={value}" for key, value in sorted(item.decisions.items()))
    print(f"   decisions: {decided}")
    print(
        f"   frames:    effective {item.effective_fps:5.1f} fps   active {item.active_fps:5.1f} fps"
    )
    if item.stalls:
        print(
            f"   input gaps: {len(item.stalls)} gaps > {stall_ms} ms, {item.stall_seconds:.2f}s"
            f" ({item.stall_pct:.0f}% of wall time), median {statistics.median(item.stalls):.0f} ms,"
            f" max {max(item.stalls):.0f} ms"
        )
    else:
        print(f"   input gaps: none above {stall_ms} ms")
    if item.output_gaps:
        print(f"   recorded source gaps: {len(item.output_gaps)} above {item.output_gap_limit_ms:.0f} ms, "
              f"max {max(item.output_gaps):.0f} ms")
    else:
        print(f"   recorded source gaps: none above {item.output_gap_limit_ms:.0f} ms")
    print(f"   buffer-full drops: {item.decisions.get('buffer_full', 0)} (CSV rows)")
    if item.missing_rows:
        print("   missing metadata can hide decisions or create apparent input gaps; compare rec_event_dropped")
    if item.mean_frame_bytes:
        print(
            f"   storage:   {item.mean_frame_bytes / 1024:6.1f} KiB/frame   "
            f"{item.bytes_per_second / 1e6:.2f} MB/s"
        )
    if item.container_size:
        extra = item.container_size - item.container_indexed
        print(
            f"   container: {item.container_size} B, CSV indexes {item.container_indexed} B,"
            f" trailing {extra} B"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("records", nargs="+", type=Path, help="rec_NNNN directory")
    parser.add_argument(
        "--log", type=Path, action="append", default=[], help="dart_app.log of a run (repeatable)"
    )
    parser.add_argument("--stall-ms", type=int, default=80, help="gap threshold (default 80)")
    parser.add_argument("--skip-windows", type=int, default=2, help="log windows to ignore at start (default 2)")
    args = parser.parse_args()
    if args.stall_ms <= 0 or args.skip_windows < 0:
        parser.error("--stall-ms must be positive; --skip-windows must be nonnegative")

    summaries = [summarise(record, args.stall_ms * 1000) for record in args.records]
    for item in summaries:
        print_summary(item, args.stall_ms)
        print()

    for log_path in args.log:
        series = log_timing(log_path, limit=args.skip_windows)
        print(f"== {log_path}")
        for key, unit in (
            ("telemetry_us", "us"),
            ("telemetry_max_us", "us"),
            ("acquire_max_us", "us"),
            ("detect_max_us", "us"),
            ("pipeline_max_us", "us"),
            ("pack_us", "us"),
            ("pack_max_us", "us"),
            ("buffer_copy_us", "us"),
            ("io_us", "us"),
            ("io_max_us", "us"),
            ("write_us", "us"),
            ("rec_busy", ""),
            ("rec_queued", ""),
            ("rec_buffered", ""),
            ("rec_buffer_high_water", ""),
            ("rec_buffer_dropped", ""),
            ("rec_event_dropped", ""),
            ("rec_write_fail", ""),
            ("drain_us", "us"),
            ("fps", ""),
        ):
            if series[key]:
                print(f"   {key:18s} {format_series(series[key], unit)}")
        if not series["telemetry_max_us"]:
            print("   no op=timing telemetry fields found (not a dart_app log?)")
        noisy = [value for value in series["telemetry_us"] if value > 5000]
        if noisy:
            print(
                f"   WARNING: {len(noisy)} observed status reports spent >5 ms on the capture thread"
                f" (max {max(noisy):.0f} us); inspect scheduling, formatting and queue drops."
            )
        elif series["telemetry_us"]:
            print("   capture-thread log cost: all status windows < 5 ms")

    if len(summaries) > 1:
        print("== comparison")
        for item in summaries:
            print(
                f"   {item.name:>10s}  effective {item.effective_fps:5.1f} fps  "
                f"active {item.active_fps:5.1f} fps  stall {item.stall_pct:4.0f}%  "
                f"dropped_rows {item.missing_rows:3d}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
