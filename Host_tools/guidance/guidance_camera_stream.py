#!/usr/bin/env python3
from __future__ import annotations

import argparse
import configparser
import contextlib
import http.server
import json
import socketserver
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised only on machines without pyserial.
    serial = None
    list_ports = None


REPO_ROOT = Path(__file__).resolve().parents[2]
OPENMV_PROJECT_DIR = REPO_ROOT / "OpenMV_guidance"
DEFAULT_CONFIG_PATH = OPENMV_PROJECT_DIR / "openmv.ini"
DEFAULT_STREAM_SCRIPT = OPENMV_PROJECT_DIR / "src" / "stream_debug.py"
DEFAULT_BAUDRATE = 115200
DEFAULT_HTTP_HOST = "127.0.0.1"
DEFAULT_HTTP_PORT = 8081

CTRL_C = b"\x03"
CTRL_D = b"\x04"
RAW_REPL_ENTER = b"\x01"
RAW_REPL_EXIT = b"\x02"
RAW_REPL_PROMPT = b"raw REPL; CTRL-B to exit\r\n>"
FRAME_MAGIC = b"OMVJ"
PASSIVE_PREVIEW_REQUEST = b"OMVP\n"
FRAME_HEADER_STRUCT = struct.Struct(">IHHI")
METADATA_MAGIC = b"OMVM"
METADATA_HEADER_STRUCT = struct.Struct(">I")


@dataclass(frozen=True)
class CameraFrame:
    jpeg: bytes
    width: int
    height: int
    ticks_ms: int
    received_at_s: float
    metadata: dict[str, object] | None = None


class LatestFrameStore:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._sequence = 0
        self._frame: Optional[CameraFrame] = None

    def update(self, frame: CameraFrame) -> int:
        with self._condition:
            self._sequence += 1
            self._frame = frame
            self._condition.notify_all()
            return self._sequence

    def get(self) -> tuple[int, Optional[CameraFrame]]:
        with self._condition:
            return self._sequence, self._frame

    def wait_for_next(self, last_sequence: int, timeout_s: float = 1.0) -> tuple[int, Optional[CameraFrame]]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._sequence == last_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return self._sequence, self._frame
                self._condition.wait(remaining)
            return self._sequence, self._frame


class ThreadingHttpServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class QuietMjpegHandler(http.server.BaseHTTPRequestHandler):
    frame_store: LatestFrameStore
    verbose_logs = False

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API.
        if self.path in ("/", "/index.html"):
            self._serve_index()
            return
        if self.path == "/snapshot.jpg":
            self._serve_snapshot()
            return
        if self.path == "/stream.mjpg":
            self._serve_stream()
            return
        self.send_error(404, "not found")

    def log_message(self, fmt: str, *args: object) -> None:
        if self.verbose_logs:
            super().log_message(fmt, *args)

    def _serve_index(self) -> None:
        html = b"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OpenMV USB Preview</title>
<style>
:root { color-scheme: dark; }
html, body { margin: 0; height: 100%; background: #151719; color: #f1f5f9; font: 14px system-ui, sans-serif; }
body { display: grid; grid-template-rows: auto 1fr; }
header { padding: 10px 14px; border-bottom: 1px solid #2b3036; background: #1f2328; }
main { min-height: 0; display: grid; place-items: center; padding: 12px; }
img { max-width: 100%; max-height: 100%; object-fit: contain; background: #050608; }
</style>
</head>
<body>
<header>OpenMV USB Preview</header>
<main><img src="/stream.mjpg" alt="OpenMV stream"></main>
</body>
</html>
"""
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(html)

    def _serve_snapshot(self) -> None:
        _, frame = self.frame_store.get()
        if frame is None:
            self.send_error(503, "no frame received yet")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame.jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(frame.jpeg)

    def _serve_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=openmv")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()

        sequence = 0
        while True:
            sequence, frame = self.frame_store.wait_for_next(sequence, timeout_s=1.0)
            if frame is None:
                continue
            try:
                self.wfile.write(b"--openmv\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame.jpeg)}\r\n".encode("ascii"))
                self.wfile.write(f"X-Frame-Width: {frame.width}\r\n".encode("ascii"))
                self.wfile.write(f"X-Frame-Height: {frame.height}\r\n".encode("ascii"))
                self.wfile.write(b"\r\n")
                self.wfile.write(frame.jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                return


class OpenMvUsbStream:
    def __init__(self, port: str, baudrate: int, read_timeout_s: float = 0.1) -> None:
        if serial is None:
            raise RuntimeError("pyserial is required: python3 -m pip install -r OpenMV_guidance/requirements.txt")
        self._port_name = port
        self._baudrate = int(baudrate)
        self._read_timeout_s = float(read_timeout_s)
        self._port = None
        self._stream_started = False
        self._passive_preview_requested = False
        self._last_passive_request_s = 0.0

    def __enter__(self) -> "OpenMvUsbStream":
        self._port = serial.Serial(self._port_name, self._baudrate, timeout=self._read_timeout_s, write_timeout=2.0)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._port is not None:
            self._port.close()
            self._port = None

    def _ensure_open(self):
        if self._port is None:
            raise RuntimeError("OpenMV USB port is not open")
        return self._port

    def _write_all(self, payload: bytes) -> None:
        port = self._ensure_open()
        port.write(payload)
        port.flush()

    def _read_exact(self, size: int, timeout_s: float) -> bytes:
        port = self._ensure_open()
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"timeout reading {size} bytes, got {len(data)}")
            chunk = port.read(size - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def _read_until(self, marker: bytes, timeout_s: float) -> bytes:
        port = self._ensure_open()
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while time.monotonic() < deadline:
            self._send_passive_preview_request()
            chunk = port.read(1)
            if not chunk:
                continue
            data.extend(chunk)
            if data.endswith(marker):
                return bytes(data)
        raise TimeoutError(f"timeout waiting for {marker!r}, got {bytes(data)!r}")

    def _drain(self, timeout_s: float = 0.2) -> bytes:
        port = self._ensure_open()
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = port.read(1024)
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def request_passive_preview(self) -> None:
        self._passive_preview_requested = True
        self._send_passive_preview_request(force=True)

    def _send_passive_preview_request(self, force: bool = False) -> None:
        if not self._passive_preview_requested:
            return
        now = time.monotonic()
        if force or now - self._last_passive_request_s >= 0.5:
            self._write_all(PASSIVE_PREVIEW_REQUEST)
            self._last_passive_request_s = now

    def enter_raw_repl(self) -> None:
        self._drain(0.2)
        self._write_all(CTRL_C)
        time.sleep(0.1)
        self._write_all(CTRL_C)
        time.sleep(0.1)
        self._write_all(RAW_REPL_EXIT)
        time.sleep(0.05)
        self._drain(0.2)
        self._write_all(RAW_REPL_ENTER)
        self._read_until(RAW_REPL_PROMPT, 3.0)
        self._write_all(CTRL_D)
        self._read_until(RAW_REPL_PROMPT, 5.0)

    def start_stream(self, stream_script: Path, stream_options: dict[str, object]) -> None:
        source = stream_script.read_text(encoding="utf-8")
        command = source + "\nrun_usb_stream(**%r)\n" % stream_options
        self._write_all(command.encode("utf-8"))
        self._write_all(CTRL_D)
        self._read_until(b"OK", 5.0)
        self._stream_started = True

    def read_frame(
        self,
        timeout_s: float,
        max_frame_bytes: int,
        expect_metadata: bool = False,
        max_metadata_bytes: int = 16 * 1024,
    ) -> CameraFrame:
        self._sync_to_magic(timeout_s)
        header = self._read_exact(FRAME_HEADER_STRUCT.size, timeout_s)
        length, width, height, ticks_ms = FRAME_HEADER_STRUCT.unpack(header)
        if length <= 0 or length > max_frame_bytes:
            raise ValueError(f"invalid frame length {length}")
        jpeg = self._read_exact(length, timeout_s)
        if not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("received payload is not a JPEG frame")
        metadata = self._read_metadata(timeout_s, max_metadata_bytes) if expect_metadata else None
        return CameraFrame(
            jpeg=jpeg,
            width=int(width),
            height=int(height),
            ticks_ms=int(ticks_ms),
            received_at_s=time.monotonic(),
            metadata=metadata,
        )

    def _read_metadata(self, timeout_s: float, max_metadata_bytes: int) -> dict[str, object]:
        magic = self._read_exact(len(METADATA_MAGIC), timeout_s)
        if magic != METADATA_MAGIC:
            raise ValueError(f"invalid metadata magic {magic!r}")
        header = self._read_exact(METADATA_HEADER_STRUCT.size, timeout_s)
        (length,) = METADATA_HEADER_STRUCT.unpack(header)
        if length <= 0 or length > max_metadata_bytes:
            raise ValueError(f"invalid metadata length {length}")
        payload = self._read_exact(length, timeout_s)
        decoded = json.loads(payload.decode("utf-8"))
        if not isinstance(decoded, dict):
            raise ValueError("metadata payload is not a JSON object")
        return decoded

    def _sync_to_magic(self, timeout_s: float) -> None:
        port = self._ensure_open()
        deadline = time.monotonic() + timeout_s
        window = bytearray()
        while time.monotonic() < deadline:
            self._send_passive_preview_request()
            chunk = port.read(1)
            if not chunk:
                continue
            window.extend(chunk)
            if len(window) > len(FRAME_MAGIC):
                del window[0 : len(window) - len(FRAME_MAGIC)]
            if bytes(window) == FRAME_MAGIC:
                return
        raise TimeoutError("timeout waiting for OpenMV frame header")

    def stop_and_reset(self, reset: bool = True) -> None:
        if self._port is None:
            return
        with contextlib.suppress(Exception):
            if self._stream_started:
                self._write_all(CTRL_C)
                time.sleep(0.3)
                self._drain(0.5)
            if reset:
                self._write_all(b"import machine\nmachine.reset()\n")
                self._write_all(CTRL_D)
                time.sleep(0.2)
            else:
                self._write_all(RAW_REPL_EXIT)
                time.sleep(0.1)


def load_config_port(config_path: Path) -> tuple[str, int]:
    parser = configparser.ConfigParser()
    if not parser.read(config_path):
        return "", DEFAULT_BAUDRATE
    port = parser.get("serial", "port", fallback="").strip()
    baudrate = parser.getint("serial", "baudrate", fallback=DEFAULT_BAUDRATE)
    return port, baudrate


def serial_port_rows() -> list[tuple[str, str, str]]:
    if list_ports is None:
        return []
    rows: list[tuple[str, str, str]] = []
    for port in list_ports.comports():
        rows.append((str(port.device), str(port.description or ""), str(port.hwid or "")))
    return sorted(rows, key=lambda row: row[0])


def format_serial_ports() -> str:
    rows = serial_port_rows()
    if not rows:
        return "no USB serial devices detected"
    return "\n".join(f"  {device}\t{description or 'n/a'}\t{hwid or 'n/a'}" for device, description, hwid in rows)


def auto_detect_port() -> str:
    rows = serial_port_rows()
    openmv_rows = [row for row in rows if "openmv" in " ".join(row).lower()]
    if len(openmv_rows) == 1:
        return openmv_rows[0][0]

    usb_rows = [
        row
        for row in rows
        if row[0].startswith("/dev/ttyACM")
        or row[0].startswith("/dev/ttyUSB")
        or "usb" in " ".join(row).lower()
    ]
    if len(usb_rows) == 1:
        return usb_rows[0][0]

    raise RuntimeError("unable to auto-select OpenMV USB port; pass --port\n" + format_serial_ports())


def resolve_port(args: argparse.Namespace) -> tuple[str, int]:
    config_port, config_baudrate = load_config_port(Path(args.config).resolve())
    port = args.port or config_port or auto_detect_port()
    baudrate = int(args.baudrate or config_baudrate or DEFAULT_BAUDRATE)
    return port, baudrate


def build_stream_options(args: argparse.Namespace) -> dict[str, object]:
    return {
        "frame_size": str(args.framesize).upper(),
        "width": int(args.width),
        "height": int(args.height),
        "pixformat": str(args.pixformat).upper(),
        "quality": int(args.quality),
        "fps_limit": int(args.fps),
        "auto_gain": bool(args.auto_gain),
        "auto_whitebal": bool(args.auto_whitebal),
        "auto_exposure": bool(args.auto_exposure),
        "exposure_us": int(args.exposure_us),
        "annotate": bool(args.annotate),
        "debug_detector": bool(args.debug_detector),
        "emit_metadata": bool(metadata_logging_enabled(args)),
        "control_uart": bool(args.control_uart),
        "control_uart_port": int(args.control_uart_port),
        "control_uart_baudrate": int(args.control_uart_baudrate),
    }


def make_handler(frame_store: LatestFrameStore, verbose_logs: bool):
    class Handler(QuietMjpegHandler):
        pass

    Handler.frame_store = frame_store
    Handler.verbose_logs = verbose_logs
    return Handler


def start_http_server(host: str, port: int, frame_store: LatestFrameStore, verbose_logs: bool):
    server = ThreadingHttpServer((host, int(port)), make_handler(frame_store, verbose_logs))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def metadata_logging_enabled(args: argparse.Namespace) -> bool:
    return bool(args.metadata_jsonl or args.record_metadata)


def resolve_metadata_jsonl_path(args: argparse.Namespace) -> Path | None:
    if args.metadata_jsonl:
        return Path(args.metadata_jsonl)
    if args.record_metadata:
        if not args.record_mjpeg:
            raise RuntimeError("--record-metadata requires --record-mjpeg or --metadata-jsonl")
        return Path(args.record_mjpeg).with_suffix(".jsonl")
    return None


def _metadata_int(value, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _point_touches_roi_edge(center_x: int, center_y: int, radius: int, roi) -> bool:
    if not isinstance(roi, (list, tuple)) or len(roi) < 4:
        return False
    rx, ry, rw, rh = (_metadata_int(roi[0]), _metadata_int(roi[1]), _metadata_int(roi[2]), _metadata_int(roi[3]))
    radius = max(1, int(radius))
    target_left = center_x - radius
    target_top = center_y - radius
    target_right = center_x + radius
    target_bottom = center_y + radius
    roi_right = rx + rw - 1
    roi_bottom = ry + rh - 1
    overlaps_roi = (
        target_right >= rx
        and target_left <= roi_right
        and target_bottom >= ry
        and target_top <= roi_bottom
    )
    return overlaps_roi and (
        target_left <= rx
        or target_top <= ry
        or target_right >= roi_right
        or target_bottom >= roi_bottom
    )


def _target_search_from_metadata(metadata: dict[str, object] | None) -> str:
    if not metadata:
        return "metadata_missing"
    status = str(metadata.get("target_search") or "")
    if status:
        return status
    if not bool(metadata.get("detector_available", True)):
        return "detector_unavailable"
    if bool(metadata.get("debug_detector", True)) is False:
        return "detector_disabled"
    if not bool(metadata.get("detected", False)):
        return "target_not_found"
    selected_scan = str(metadata.get("selected_scan") or "")
    if selected_scan == "roi" or bool(metadata.get("target_in_search_roi", False)):
        return "search_roi"
    if bool(metadata.get("full_target_selected", False)):
        center_x = _metadata_int(metadata.get("center_x", -1), -1)
        center_y = _metadata_int(metadata.get("center_y", -1), -1)
        radius = _metadata_int(metadata.get("radius", 1), 1)
        if center_x >= 0 and center_y >= 0 and _point_touches_roi_edge(center_x, center_y, radius, metadata.get("search_roi")):
            return "search_roi_too_small"
    return "full_frame_search"


def write_metadata_line(handle, sequence: int, frame: CameraFrame) -> None:
    metadata = frame.metadata or {}
    record: dict[str, object] = {
        "frame_index": _metadata_int(metadata.get("frame_index"), max(0, int(sequence) - 1)),
        "target_search": _target_search_from_metadata(metadata),
    }
    handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")))
    handle.write("\n")
    handle.flush()


def write_optional_frame_files(args: argparse.Namespace, sequence: int, frame: CameraFrame) -> None:
    if args.raw_dump:
        with open(args.raw_dump, "ab") as handle:
            handle.write(FRAME_MAGIC)
            handle.write(FRAME_HEADER_STRUCT.pack(len(frame.jpeg), frame.width, frame.height, frame.ticks_ms))
            handle.write(frame.jpeg)
    if args.record_mjpeg:
        output_path = Path(args.record_mjpeg)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "ab") as handle:
            handle.write(frame.jpeg)
    if args.save_dir:
        output_dir = Path(args.save_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"openmv_{sequence:06d}.jpg"
        output_path.write_bytes(frame.jpeg)


def run_preview(args: argparse.Namespace) -> int:
    stream_script = Path(args.openmv_script).resolve()
    if args.raw_repl:
        if not stream_script.is_file():
            raise RuntimeError(f"OpenMV stream script not found: {stream_script}")
        if args.control_uart and not args.debug_detector:
            raise RuntimeError("--control-uart requires the OpenMV detector; remove --no-debug-detector")

    port, baudrate = resolve_port(args)
    frame_store = LatestFrameStore()
    server = None
    if not args.snapshot:
        server = start_http_server(args.http_host, args.http_port, frame_store, args.http_logs)
        print(f"OpenMV preview: http://{args.http_host}:{args.http_port}/")
    print(f"Using OpenMV USB device: {port}")
    if args.raw_repl:
        print("Raw REPL mode: interrupting OpenMV and running the host stream helper")
        if args.control_uart:
            print(
                f"Forwarding detector measurements to control UART{args.control_uart_port} "
                f"at {args.control_uart_baudrate} baud"
            )
    else:
        print("Passive mode: listening to /flash/main.py without interrupting OpenMV")
        if args.control_uart:
            print("Passive mode keeps UART1 under /flash/main.py; --control-uart is ignored")

    metadata_jsonl_path = resolve_metadata_jsonl_path(args)
    stream_options = build_stream_options(args) if args.raw_repl else {}
    max_frame_bytes = max(4096, int(args.max_frame_bytes))
    max_metadata_bytes = max(256, int(args.max_metadata_bytes))
    status_interval_s = max(0.2, float(args.status_interval))
    sequence = 0
    last_status_at = time.monotonic()
    bytes_since_status = 0
    frames_since_status = 0
    expect_metadata = bool(metadata_jsonl_path is not None or not args.raw_repl)

    metadata_handle = None
    try:
        if metadata_jsonl_path is not None:
            metadata_jsonl_path.parent.mkdir(parents=True, exist_ok=True)
            metadata_mode = "a" if args.record_metadata and not args.metadata_jsonl else "w"
            metadata_handle = open(metadata_jsonl_path, metadata_mode, encoding="utf-8")
            print(f"OpenMV metadata log: {metadata_jsonl_path}")
        with OpenMvUsbStream(port, baudrate) as stream:
            try:
                if args.raw_repl:
                    stream.enter_raw_repl()
                    stream.start_stream(stream_script, stream_options)
                else:
                    stream.request_passive_preview()
                while True:
                    frame = stream.read_frame(
                        timeout_s=float(args.frame_timeout),
                        max_frame_bytes=max_frame_bytes,
                        expect_metadata=expect_metadata,
                        max_metadata_bytes=max_metadata_bytes,
                    )
                    sequence = frame_store.update(frame)
                    write_optional_frame_files(args, sequence, frame)
                    if metadata_handle is not None:
                        write_metadata_line(metadata_handle, sequence, frame)

                    if args.snapshot:
                        output_path = Path(args.snapshot)
                        output_path.parent.mkdir(parents=True, exist_ok=True)
                        output_path.write_bytes(frame.jpeg)
                        print(f"Saved snapshot: {output_path} ({frame.width}x{frame.height}, {len(frame.jpeg)} bytes)")
                        return 0

                    frames_since_status += 1
                    bytes_since_status += len(frame.jpeg)
                    now = time.monotonic()
                    if now - last_status_at >= status_interval_s:
                        elapsed = max(0.001, now - last_status_at)
                        fps = frames_since_status / elapsed
                        kbps = (bytes_since_status * 8.0 / 1000.0) / elapsed
                        sys.stdout.write(
                            f"\rframes={sequence} fps={fps:4.1f} link={kbps:6.1f}kbps "
                            f"last={frame.width}x{frame.height} {len(frame.jpeg)}B   "
                        )
                        sys.stdout.flush()
                        last_status_at = now
                        frames_since_status = 0
                        bytes_since_status = 0
            finally:
                if args.raw_repl:
                    stream.stop_and_reset(reset=not args.no_reset)
    finally:
        if metadata_handle is not None:
            metadata_handle.close()
        if server is not None:
            server.shutdown()
            server.server_close()
        if not args.snapshot:
            sys.stdout.write("\n")
    return 0

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Preview OpenMV camera frames over USB-C, optionally forwarding detector measurements to the control-board UART"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="OpenMV config path used for default port/baudrate")
    parser.add_argument("--port", default="", help="OpenMV USB CDC device, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=0, help="USB CDC baudrate setting; default comes from openmv.ini")
    parser.add_argument("--openmv-script", default=str(DEFAULT_STREAM_SCRIPT), help="OpenMV-side USB stream helper source")
    parser.add_argument("--http-host", default=DEFAULT_HTTP_HOST, help="local preview HTTP bind host")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT, help="local preview HTTP port")
    parser.add_argument("--http-logs", action="store_true", help="print HTTP request logs")
    parser.add_argument("--snapshot", default="", help="capture one frame to this JPEG path and exit")
    parser.add_argument("--save-dir", default="", help="optional directory to save every received JPEG frame")
    parser.add_argument("--record-mjpeg", default="", help="append received JPEG frames to this MJPEG file for video-tuner")
    parser.add_argument("--record-metadata", action="store_true", help="write a compact JSONL sidecar next to --record-mjpeg with frame_index and target_search per frame")
    parser.add_argument("--metadata-jsonl", default="", help="write one compact target-search JSON object per received frame to this host-side path")
    parser.add_argument("--raw-dump", default="", help="optional binary dump path using the OMVJ frame format")
    parser.add_argument("--framesize", default="QVGA", choices=["QQVGA", "QVGA", "VGA"], help="OpenMV sensor frame size")
    parser.add_argument("--width", type=int, default=640, help="sensor window width; use 0 to keep full frame")
    parser.add_argument("--height", type=int, default=480, help="sensor window height; use 0 to keep full frame")
    parser.add_argument("--pixformat", default="RGB565", choices=["RGB565", "GRAYSCALE"], help="OpenMV sensor pixel format")
    parser.add_argument("--quality", type=int, default=70, help="JPEG quality, clamped on the OpenMV side")
    parser.add_argument("--fps", type=int, default=12, help="maximum frame send rate; use 0 for no limit")
    parser.add_argument("--auto-gain", action="store_true", help="enable OpenMV auto gain for preview")
    parser.add_argument("--auto-whitebal", action="store_true", help="enable OpenMV auto white balance for preview")
    parser.add_argument("--auto-exposure", action="store_true", help="enable OpenMV auto exposure for preview")
    parser.add_argument("--exposure-us", type=int, default=2500, help="manual exposure time when auto exposure is disabled")
    parser.add_argument("--annotate", action="store_true", help="draw OpenMV-side pixel size and FPS text when detector overlay is disabled")
    parser.add_argument("--debug-detector", dest="debug_detector", action="store_true", default=True, help="run GreenLightDetector on the OpenMV preview stream and draw target overlays")
    parser.add_argument("--no-debug-detector", dest="debug_detector", action="store_false", help="stream raw preview frames without detector overlays")
    parser.add_argument("--control-uart", action="store_true", help="raw-REPL mode only: forward detector x/y/area to the STM32 over OpenMV UART1")
    parser.add_argument("--control-uart-port", type=int, default=1, help="OpenMV UART port used by --control-uart")
    parser.add_argument("--control-uart-baudrate", type=int, default=115200, help="baudrate used by --control-uart")
    parser.add_argument("--raw-repl", action="store_true", help="legacy mode: interrupt OpenMV and run src/stream_debug.py from RAM")
    parser.add_argument("--frame-timeout", type=float, default=5.0, help="seconds to wait for a frame before failing")
    parser.add_argument("--max-frame-bytes", type=int, default=512 * 1024, help="reject frames larger than this many bytes")
    parser.add_argument("--max-metadata-bytes", type=int, default=16 * 1024, help="reject per-frame metadata payloads larger than this many bytes before compacting")
    parser.add_argument("--status-interval", type=float, default=1.0, help="seconds between terminal status updates")
    parser.add_argument("--no-reset", action="store_true", help="legacy raw-REPL mode only: leave OpenMV in REPL instead of resetting back to main.py on exit")
    return parser


def main(argv: list[str]) -> int:
    args = build_arg_parser().parse_args(argv)
    return run_preview(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\nStopped")
        raise SystemExit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
