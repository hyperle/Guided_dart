#!/usr/bin/env python3
"""Regression test for tools/rec_extract_pngs.py using a synthetic container."""

from __future__ import annotations

import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
EXTRACT = REPO / "tools" / "rec_extract_pngs.py"


def make_png(width: int, height: int, pixels: bytes) -> bytes:
    def chunk(kind: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
        )

    raw = b"".join(b"\x00" + pixels[y * width : (y + 1) * width] for y in range(height))
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def main() -> int:
    width, height = 2, 1
    first = make_png(width, height, bytes([0, 255]))
    second = make_png(width, height, bytes([10, 20]))
    with tempfile.TemporaryDirectory(prefix="dart-pngs-extract-") as scratch:
        root = Path(scratch)
        (root / "img").mkdir()
        (root / "logs").mkdir()
        header = struct.pack("<4sHHII", b"DPNG", 1, 16, width, height)
        (root / "img" / "recording.pngs").write_bytes(header + first + second)
        (root / "logs" / "frames.csv").write_text(
            "# recorder format=2 width=2 height=1 fps=30 "
            "max_file_bytes=500000000 file_limit_bytes=500000000 session=/tmp/rec_0001\n"
            "input_index,source_sequence,source_pts,source_pts_timebase_validated,"
            "received_us,decision,image_index,image_file,file_offset,file_bytes\n"
            f"1,1,0,0,1000,accepted,1,img/recording.pngs,16,{len(first)}\n"
            "2,2,0,0,2000,downsampled,0,,0,0\n"
            f"3,3,0,0,3000,accepted,2,img/recording.pngs,{16 + len(first)},{len(second)}\n"
        )
        result = subprocess.run(
            [sys.executable, str(EXTRACT), str(root), "--strict"],
            text=True,
            capture_output=True,
        )
        assert result.returncode == 0, result.stderr
        assert (root / "img" / "00000001.png").read_bytes() == first
        assert (root / "img" / "00000002.png").read_bytes() == second
        assert "frames=2 failed=0" in result.stdout, result.stdout

        # A damaged tail must be reported and rejected in strict mode.
        (root / "img" / "recording.pngs").write_bytes(header + first + second[: len(second) // 2])
        result = subprocess.run(
            [sys.executable, str(EXTRACT), str(root), "--strict"],
            text=True,
            capture_output=True,
        )
        assert result.returncode == 1, (result.returncode, result.stdout, result.stderr)
        assert "outside file size" in result.stderr or "not a complete PNG" in result.stderr
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
