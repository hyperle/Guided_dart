#!/usr/bin/env python3
"""Extract the PNG frames stored in a dart_app `--rec-format pngs` container.

The container is a 16-byte header followed by concatenated, complete PNG files.
`logs/frames.csv` is the index: every accepted row carries the byte offset and
length of its PNG.  This tool only reads those offsets, validates the header and
PNG boundaries, and writes ordinary per-frame PNGs so existing viewers and
`rec_to_video.py` can consume the session.

Container layout (little-endian):

    0   char magic[4] = "DPNG"
    4   u16  version = 1
    6   u16  header_bytes = 16
    8   u32  width
    12  u32  height

Usage:
    python3 tools/rec_extract_pngs.py records/rec_0001
    python3 tools/rec_extract_pngs.py records/rec_0001 --output /tmp/frames
"""

from __future__ import annotations

import argparse
import csv
import io
import struct
import sys
from pathlib import Path

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
CONTAINER_HEADER = struct.Struct("<4sHHII")
CONTAINER_MAGIC = b"DPNG"
CONTAINER_VERSION = 1


def read_comment_header(path: Path) -> dict[str, str]:
    header: dict[str, str] = {}
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if not line.startswith("#"):
                break
            for token in line[1:].split():
                if "=" in token:
                    key, value = token.split("=", 1)
                    header[key] = value
    return header


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        data_lines = [line for line in stream if line.strip() and not line.startswith("#")]
    if not data_lines:
        return []
    return list(csv.DictReader(io.StringIO("".join(data_lines))))


def as_int(row: dict[str, str], name: str, location: str) -> int:
    try:
        value = int(row[name])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{location}: invalid {name}") from error
    if value < 0:
        raise ValueError(f"{location}: negative {name}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", type=Path, help="rec_NNNN directory")
    parser.add_argument(
        "--container", type=Path, default=None,
        help="container path (default: <record>/img/recording.pngs)")
    parser.add_argument(
        "--frames", type=Path, default=None,
        help="frames.csv path (default: <record>/logs/frames.csv)")
    parser.add_argument(
        "--output", type=Path, default=None,
        help="output directory (default: <record>/img; 8-digit names match the Qt player)")
    parser.add_argument(
        "--strict", action="store_true",
        help="return 1 when a frame is missing/truncated")
    args = parser.parse_args()

    record = args.record
    container_path = args.container or (record / "img" / "recording.pngs")
    frames_path = args.frames or (record / "logs" / "frames.csv")
    output = args.output or (record / "img")
    # Flat copies keep recording.pngs/frames.csv beside the session directory.
    if args.container is None and not container_path.is_file():
        if (record / "recording.pngs").is_file():
            container_path = record / "recording.pngs"
    if args.frames is None and not frames_path.is_file():
        if (record / "frames.csv").is_file():
            frames_path = record / "frames.csv"

    if not container_path.is_file():
        print(f"missing container: {container_path}", file=sys.stderr)
        return 2
    if not frames_path.is_file():
        print(f"missing frames CSV: {frames_path}", file=sys.stderr)
        return 2

    try:
        csv_header = read_comment_header(frames_path)
        rows = read_csv_rows(frames_path)
    except OSError as error:
        print(f"cannot read {frames_path}: {error}", file=sys.stderr)
        return 2

    file_size = container_path.stat().st_size
    if file_size < CONTAINER_HEADER.size:
        print(f"{container_path}: container header is incomplete", file=sys.stderr)
        return 2
    with container_path.open("rb") as stream:
        raw_header = stream.read(CONTAINER_HEADER.size)
        magic, version, header_bytes, width, height = CONTAINER_HEADER.unpack(raw_header)
        if magic != CONTAINER_MAGIC or version != CONTAINER_VERSION:
            print(f"{container_path}: unsupported container header", file=sys.stderr)
            return 2
        if header_bytes < CONTAINER_HEADER.size or header_bytes > file_size:
            print(f"{container_path}: invalid header_bytes={header_bytes}", file=sys.stderr)
            return 2
        if csv_header.get("width") and int(csv_header["width"]) != width:
            print(f"{container_path}: width {width} != CSV {csv_header['width']}", file=sys.stderr)
            return 2
        if csv_header.get("height") and int(csv_header["height"]) != height:
            print(
                f"{container_path}: height {height} != CSV {csv_header['height']}",
                file=sys.stderr,
            )
            return 2

        output.mkdir(parents=True, exist_ok=True)
        accepted: list[tuple[int, int, int, str]] = []
        for line_number, row in enumerate(rows, start=2):
            if row.get("decision") != "accepted":
                continue
            location = f"{frames_path}:{line_number}"
            name = row.get("image_file", "")
            if name and not name.endswith("recording.pngs"):
                print(f"{location}: not a pngs frame: {name}", file=sys.stderr)
                return 2
            index = as_int(row, "image_index", location)
            offset = as_int(row, "file_offset", location)
            size = as_int(row, "file_bytes", location)
            if index == 0 or size == 0:
                print(f"{location}: empty accepted frame", file=sys.stderr)
                return 2
            accepted.append((index, offset, size, location))

        if not accepted:
            print(f"{frames_path}: no accepted frames", file=sys.stderr)
            return 2

        accepted.sort(key=lambda item: item[0])
        seen: set[int] = set()
        written = 0
        failures = 0
        expected_offset = header_bytes
        for index, offset, size, location in accepted:
            if index in seen:
                print(f"{location}: duplicate image_index {index}", file=sys.stderr)
                return 2
            seen.add(index)
            if offset != expected_offset:
                print(
                    f"{location}: frame {index} offset {offset} != expected {expected_offset}",
                    file=sys.stderr,
                )
                failures += 1
            expected_offset = offset + size
            if offset < header_bytes or offset + size > file_size:
                message = (
                    f"{location}: frame {index} offset/size {offset}+{size} "
                    f"outside file size {file_size}"
                )
                print(message, file=sys.stderr)
                failures += 1
                continue
            stream.seek(offset)
            data = stream.read(size)
            if len(data) != size or not data.startswith(PNG_SIGNATURE):
                print(f"{location}: frame {index} is not a complete PNG", file=sys.stderr)
                failures += 1
                continue
            if data[-8:-4] != b"IEND":
                print(f"{location}: frame {index} has no IEND chunk", file=sys.stderr)
                failures += 1
                continue
            try:
                png_width, png_height = struct.unpack(">II", data[16:24])
            except struct.error:
                print(f"{location}: frame {index} has no valid IHDR", file=sys.stderr)
                failures += 1
                continue
            if png_width != width or png_height != height:
                print(
                    f"{location}: frame {index} is {png_width}x{png_height}, "
                    f"expected {width}x{height}",
                    file=sys.stderr,
                )
                failures += 1
                continue
            target = output / f"{index:08d}.png"
            target.write_bytes(data)
            written += 1

    print(
        f"container={container_path} size={file_size} nominal_fps={csv_header.get('fps', '?')} "
        f"geometry={width}x{height} frames={written} failed={failures} output={output}"
    )
    if failures:
        return 1 if args.strict else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
