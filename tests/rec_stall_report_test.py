#!/usr/bin/env python3
"""Regression for tools/rec_stall_report.py using a synthetic session."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / "tools" / "rec_stall_report.py"
CSV_HEADER = (
    "# recorder format=2 width=640 height=480 fps=30 max_file_bytes=500000000 "
    "file_limit_bytes=500000000 session=rec_0001\n"
    "input_index,source_sequence,source_pts,source_pts_timebase_validated,received_us,"
    "decision,image_index,image_file,file_offset,file_bytes\n"
)


def load_tool():
    spec = importlib.util.spec_from_file_location("rec_stall_report", TOOL)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # Dataclasses resolve their module while the class body is executed.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_session(root: Path) -> Path:
    record = root / "rec_0001"
    (record / "logs").mkdir(parents=True)
    (record / "img").mkdir(parents=True)
    lines = []
    # 100 submissions every 21 ms plus one extra 210 ms before the 51st frame.
    time_us = 0
    image_index = 0
    accepted_bytes = 0
    for index in range(1, 101):
        if index == 51:
            time_us += 210_000
        time_us += 21_000
        accepted = index % 4 != 0
        if accepted:
            image_index += 1
            offset = 16 + accepted_bytes
            size = 400 + image_index
            lines.append(
                f"{index},{index},{time_us},0,{time_us},accepted,{image_index},"
                f"img/recording.pngs,{offset},{size}\n"
            )
            accepted_bytes += size
        else:
            lines.append(f"{index},{index},{time_us},0,{time_us},downsampled,0,,0,0\n")
    (record / "logs" / "frames.csv").write_text(CSV_HEADER + "".join(lines))
    # One extra frame the CSV does not index: the tool must report the bytes.
    (record / "img" / "recording.pngs").write_bytes(b"\0" * (16 + accepted_bytes + 517))
    return record


def main() -> int:
    module = load_tool()
    with tempfile.TemporaryDirectory(prefix="dart-stall-report-") as scratch:
        record = write_session(Path(scratch))
        summary = module.summarise(record, 80_000)
        assert summary.rows == 100
        assert summary.attempts == 100
        assert summary.missing_rows == 0, summary.missing_rows
        assert summary.accepted == 75, summary.accepted
        assert summary.decisions["downsampled"] == 25, summary.decisions
        assert abs(summary.seconds - (99 * 0.021 + 0.21)) < 1e-6, summary.seconds
        assert len(summary.stalls) == 1, summary.stalls
        assert abs(summary.stalls[0] - 231.0) < 1.0, summary.stalls
        assert abs(summary.stall_seconds - 0.231) < 1e-6, summary.stall_seconds
        assert abs(summary.effective_fps - 75 / summary.seconds) < 1e-6
        assert abs(summary.active_fps - 75 / (summary.seconds - 0.231)) < 1e-6
        assert summary.container_size - summary.container_indexed == 517
        assert 400 < summary.mean_frame_bytes < 500, summary.mean_frame_bytes
        assert summary.target_fps == 30
        assert summary.output_gaps

        # A session without stalls must report an effective rate at the target.
        clean = Path(scratch) / "rec_0002"
        (clean / "logs").mkdir(parents=True)
        lines = []
        for index in range(1, 31):
            time_us = index * 33_000
            lines.append(f"{index},{index},{time_us},0,{time_us},accepted,{index},,0,0\n")
        (clean / "logs" / "frames.csv").write_text(CSV_HEADER + "".join(lines))
        clean_summary = module.summarise(clean, 80_000)
        assert clean_summary.stalls == [], clean_summary.stalls
        assert abs(clean_summary.effective_fps - clean_summary.active_fps) < 1e-9
        # Duration spans 29 intervals of 33 ms, so 30 frames read as ~31.3 fps.
        assert abs(clean_summary.effective_fps - 30 / (29 * 0.033)) < 0.01, clean_summary.effective_fps

        # Dense capture with a backed-up writer creates recorded-source gaps,
        # while input remains continuous. H.264 shares the CSV byte index.
        h264 = Path(scratch) / "rec_0003"
        (h264 / "logs").mkdir(parents=True)
        (h264 / "img").mkdir()
        lines = []
        image_index = 0
        for index in range(1, 51):
            if index <= 5 or index >= 45:
                image_index += 1
                lines.append(f"{index},{index},0,0,{index * 10000},accepted,{image_index},"
                             f"img/recording.h264,{(image_index - 1) * 100},100\n")
            else:
                lines.append(f"{index},{index},0,0,{index * 10000},buffer_full,0,,0,0\n")
        (h264 / "logs" / "frames.csv").write_text(CSV_HEADER + "".join(lines))
        (h264 / "img" / "recording.h264").write_bytes(b"\0" * (image_index * 100))
        buffered = module.summarise(h264, 80_000)
        assert buffered.stalls == []
        assert buffered.output_gaps == [400.0]
        assert buffered.decisions["buffer_full"] == 39
        assert buffered.format_note == "recording.h264"
        assert buffered.container_size == buffered.container_indexed

        # A window spans all status rows up to op=timing, rather than one line.
        log = Path(scratch) / "app.log"
        log.write_text(
            "[record] ts_ms=1 op=recorder_start fps=30\n"
            "[record] ts_ms=1 op=buffer_status rec_buffered=3 rec_buffer_dropped=4\n"
            "[app] ts_ms=1 op=timing window_ms=1000 fps=20 telemetry_max_us=100\n"
            "[record] ts_ms=2 op=buffer_status rec_buffered=5 rec_buffer_dropped=8\n"
            "[record] ts_ms=2 op=recorder_timing pack_us=900 io_max_us=20000\n"
            "[app] ts_ms=2 op=timing window_ms=1000 fps=25 telemetry_max_us=200\n"
        )
        series = module.log_timing(log, limit=1)
        assert series["fps"] == [25.0]
        assert series["rec_buffered"] == [5.0]
        assert series["rec_buffer_dropped"] == [8.0]
        assert series["pack_us"] == [900.0]

    print("rec_stall_report: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
