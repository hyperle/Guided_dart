#!/usr/bin/env python3
"""JPEG upload receiver and FFmpeg H.264/HLS backend. Python standard library only."""

import argparse
import base64
import binascii
import hashlib
from collections import deque
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import secrets
import socket
import shutil
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlsplit


MAX_JPEG_BYTES = 2 * 1024 * 1024
VIEWER = Path(__file__).with_name("viewer.html")


def jpeg_size(data):
    """Read JPEG SOF dimensions before handing the bounded input to FFmpeg."""
    if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise ValueError("expected a complete JPEG")
    offset = 2
    while offset + 4 <= len(data):
        if data[offset] != 0xFF:
            break
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            continue
        if marker in (0xD9, 0xDA):
            break
        length = int.from_bytes(data[offset:offset + 2], "big")
        if length < 2 or offset + length > len(data):
            break
        if marker in (0xC0, 0xC1, 0xC2):
            if length < 8:
                break
            height = int.from_bytes(data[offset + 3:offset + 5], "big")
            width = int.from_bytes(data[offset + 5:offset + 7], "big")
            if not (1 <= width <= 1920 and 1 <= height <= 1080):
                raise ValueError("JPEG dimensions exceed 1920x1080 or are zero")
            return width, height
        offset += length
    raise ValueError("JPEG has no supported SOF dimensions")


class VideoEncoder:
    def __init__(self, *, fps=5, width=320, height=240, ffmpeg="ffmpeg"):
        self.fps, self.width, self.height = fps, width, height
        self.directory = tempfile.TemporaryDirectory(prefix="canmv-video-")
        self.root = Path(self.directory.name)
        self.lock = threading.Lock()
        self.ready = threading.Event()
        self.stopped = threading.Event()
        self.latest = None
        self.received_frames = 0
        self.encoded_frames = 0
        self.last_received = None
        self.started = time.monotonic()
        self.stream_id = secrets.token_hex(8)
        self.first_received = None
        self.upload_times = deque(maxlen=4096)
        self.max_upload_gap = 0.0
        self.error = None
        self.log = (self.root / "ffmpeg.log").open("wb")
        command = [
            ffmpeg, "-hide_banner", "-loglevel", "warning", "-y",
            "-f", "image2pipe", "-framerate", str(fps), "-vcodec", "mjpeg",
            "-probesize", "32", "-analyzeduration", "0", "-threads", "1", "-i", "pipe:0",
            "-an", "-vf", (f"scale={width}:{height}:force_original_aspect_ratio=decrease,"
                            f"pad={width}:{height}:(ow-iw)/2:(oh-ih)/2,setsar=1"),
            "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
            "-pix_fmt", "yuv420p", "-profile:v", "baseline", "-crf", "24",
            "-threads", "1", "-g", str(fps), "-keyint_min", str(fps), "-sc_threshold", "0",
            "-f", "hls", "-hls_start_number_source", "epoch", "-hls_time", "1", "-hls_list_size", "6",
            "-hls_flags", "delete_segments+independent_segments+omit_endlist+temp_file",
            "-hls_delete_threshold", "2", "-hls_segment_filename", str(self.root / "frame_%09d.ts"),
            str(self.root / "index.m3u8"),
        ]
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                            stdout=subprocess.DEVNULL, stderr=self.log)
        except BaseException:
            self.log.close()
            self.directory.cleanup()
            raise
        self.worker = threading.Thread(target=self._encode, name="video-encoder", daemon=True)
        self.worker.start()

    def submit(self, jpeg):
        jpeg_size(jpeg)
        with self.lock:
            if self.error or self.process.poll() is not None:
                raise RuntimeError("FFmpeg encoder stopped; inspect backend terminal/log")
            # One latest frame bounds memory and avoids an ever-growing video delay.
            now = time.monotonic()
            if self.last_received is not None:
                self.max_upload_gap = max(self.max_upload_gap, now - self.last_received)
            if self.first_received is None:
                self.first_received = now
            self.latest = jpeg
            self.received_frames += 1
            self.last_received = now
            self.upload_times.append(now)
        self.ready.set()

    def _encode(self):
        while not self.stopped.is_set():
            if self.ready.wait(0.2):
                break
        next_frame = time.monotonic()
        while not self.stopped.is_set():
            with self.lock:
                jpeg = self.latest
            if jpeg is None:
                break
            try:
                self.process.stdin.write(jpeg)
                self.process.stdin.flush()
            except (BrokenPipeError, OSError, ValueError) as error:
                if not self.stopped.is_set():
                    with self.lock:
                        self.error = str(error)
                return
            with self.lock:
                self.encoded_frames += 1
            # Resample latest uploads to a constant video clock. Slow Wi-Fi repeats
            # the latest image; it does not slow playback or queue stale frames.
            next_frame = max(next_frame + 1 / self.fps, time.monotonic())
            self.stopped.wait(max(0, next_frame - time.monotonic()))

    def status(self):
        with self.lock:
            now = time.monotonic()
            age = None if self.last_received is None else now - self.last_received
            while self.upload_times and self.upload_times[0] < now - 60:
                self.upload_times.popleft()
            error = self.error
            if self.process.poll() is not None and not self.stopped.is_set():
                error = error or "FFmpeg exited; inspect backend log"
            return {"received_frames": self.received_frames, "encoded_frames": self.encoded_frames,
                    "age_seconds": None if age is None else round(age, 2),
                    "online": age is not None and age < 5 and not error,
                    "hls_ready": (self.root / "index.m3u8").is_file(),
                    "fps": self.fps, "width": self.width, "height": self.height, "stream_id": self.stream_id,
                    "error": error, "uptime_seconds": round(now - self.started, 2),
                    "last_minute_frames": len(self.upload_times),
                    "upload_fps": round(sum(stamp >= now - 10 for stamp in self.upload_times) /
                                        min(10, max(0.1, now - self.first_received)), 2)
                                  if self.first_received is not None else 0.0,
                    "max_upload_gap_seconds": round(self.max_upload_gap, 2)}

    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        self.stopped.set()
        self.ready.set()
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        self.worker.join(timeout=3)
        self.process.stdin.close()
        self.log.close()
        self.directory.cleanup()


class VideoServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, encoder, token):
        self.encoder, self.token = encoder, token
        self.relay_lock = threading.Lock()
        self.relay_status = {}
        super().__init__(address, VideoHandler)


class VideoHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(15)

    def log_message(self, fmt, *args):
        # Per-frame access logs would overwhelm the useful startup/error messages.
        pass

    def respond(self, status, body=b"", content_type="text/plain; charset=utf-8", *, close=False):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        if close:
            self.send_header("Connection", "close")
            self.close_connection = True
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_POST(self):
        if urlsplit(self.path).path != "/api/frame":
            self.respond(404, b"Not found", close=True)
            return
        expected = "Bearer " + self.server.token
        authorization = self.headers.get("Authorization", "")
        if not hmac.compare_digest(authorization.encode(), expected.encode()):
            self.respond(401, b"Invalid upload token", close=True)
            return
        # Reject unsupported framing rather than reading beyond this request.
        lengths = self.headers.get_all("Content-Length", [])
        if self.headers.get("Transfer-Encoding") or len(lengths) != 1 or not lengths[0].isdigit():
            self.respond(411, b"One Content-Length required", close=True)
            return
        size = int(lengths[0])
        if not 0 < size <= MAX_JPEG_BYTES:
            self.respond(413, b"JPEG exceeds 2 MiB", close=True)
            return
        if self.headers.get_content_type() != "image/jpeg":
            self.respond(415, b"Content-Type must be image/jpeg", close=True)
            return
        try:
            jpeg = self.rfile.read(size)
            if len(jpeg) != size:
                self.respond(400, b"Incomplete JPEG body", close=True)
                return
            self.server.encoder.submit(jpeg)
        except ValueError as error:
            self.respond(400, str(error).encode(), close=True)
            return
        except RuntimeError as error:
            self.respond(503, str(error).encode(), close=True)
            return
        except TimeoutError:
            self.respond(408, b"Upload timed out", close=True)
            return
        # Acknowledges receipt, not completion of the asynchronous video encoding.
        self.respond(204)

    def websocket_frame(self, opcode, data):
        self.wfile.write(bytes((0x80 | opcode, len(data))) + data)
        self.wfile.flush()

    def websocket_upload(self, browser_auth=False):
        expected = "Bearer " + self.server.token
        if not browser_auth and not hmac.compare_digest(self.headers.get("Authorization", "").encode(), expected.encode()):
            self.respond(401, b"Invalid upload token", close=True)
            return
        key = self.headers.get("Sec-WebSocket-Key", "")
        try:
            valid_key = len(base64.b64decode(key, validate=True)) == 16
        except (ValueError, binascii.Error):
            valid_key = False
        connection = {part.strip().lower() for part in self.headers.get("Connection", "").split(",")}
        if (self.headers.get("Upgrade", "").lower() != "websocket" or
                "upgrade" not in connection or not valid_key or
                self.headers.get("Sec-WebSocket-Version") != "13" or
                self.headers.get("Transfer-Encoding") or self.headers.get("Content-Length")):
            self.respond(400, b"Valid WebSocket upgrade required", close=True)
            return
        accept = base64.b64encode(hashlib.sha1(
            key.encode("ascii") + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest()).decode("ascii")
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.wfile.flush()
        self.close_connection = True
        # Frames are individual complete JPEGs; no growing queue or message
        # reassembly. Cumulative ACKs are independent of the encoding worker.
        received = 0
        authenticated = not browser_auth
        relay_id = None
        if browser_auth:
            self.connection.settimeout(5)

        def read_exact(size):
            data = self.rfile.read(size)
            if len(data) != size:
                raise EOFError
            return data

        def close(code):
            self.websocket_frame(8, code.to_bytes(2, "big"))

        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            while True:
                first, second = read_exact(2)
                opcode, size = first & 0x0f, second & 0x7f
                if first & 0xf0 != 0x80 or not second & 0x80 or opcode not in ((1, 2, 8, 9, 10) if browser_auth else (2, 8, 9, 10)):
                    close(1002)
                    return
                if opcode >= 8 and size > 125:
                    close(1002)
                    return
                if size == 126:
                    size = int.from_bytes(read_exact(2), "big")
                    if size < 126:
                        close(1002)
                        return
                elif size == 127:
                    size = int.from_bytes(read_exact(8), "big")
                    if size <= 65535 or size >> 63:
                        close(1002)
                        return
                if not authenticated and opcode == 2:
                    close(1002)
                    return
                if size > (MAX_JPEG_BYTES if authenticated and opcode == 2 else 512):
                    close(1009)
                    return
                mask = read_exact(4)
                data = bytearray(read_exact(size))
                for index in range(size):
                    data[index] ^= mask[index & 3]
                if opcode == 8:
                    if size == 1:
                        close(1002)
                    else:
                        self.websocket_frame(8, data)
                    return
                if opcode == 9:
                    self.websocket_frame(10, data)
                    continue
                if opcode == 10:
                    continue
                if not authenticated:
                    try:
                        supplied = json.loads(data).get("token", "")
                        valid = isinstance(supplied, str) and hmac.compare_digest(
                            supplied.encode(), self.server.token.encode())
                    except (ValueError, AttributeError, UnicodeError):
                        valid = False
                    if not valid:
                        close(1008)
                        return
                    authenticated = True
                    relay_id = secrets.token_hex(4)
                    with self.server.relay_lock:
                        self.server.relay_status = {"id": relay_id, "connected": True, "state": "authenticated", "updated_at": time.time()}
                    self.connection.settimeout(15)
                    self.websocket_frame(1, b'{"ready":true}')
                    continue
                if opcode == 1:
                    try:
                        report = json.loads(data)
                        state = report.get("state")
                        if state not in ("connecting", "streaming", "local_closed", "local_error", "local_stalled", "hidden"):
                            raise ValueError("invalid relay state")
                        metrics = {name: report[name] for name in ("local_frames", "sent_frames", "buffered_bytes")}
                        if any(type(value) is not int or not 0 <= value <= 2 ** 32 for value in metrics.values()):
                            raise ValueError("invalid relay metrics")
                    except (ValueError, AttributeError, KeyError, UnicodeError):
                        close(1007)
                        return
                    with self.server.relay_lock:
                        if self.server.relay_status.get("id") == relay_id:
                            self.server.relay_status = {"id": relay_id, "connected": True,
                                "state": state, "updated_at": time.time(), **metrics}
                    continue
                try:
                    self.server.encoder.submit(bytes(data))
                except ValueError:
                    close(1007)
                    return
                except RuntimeError:
                    close(1011)
                    return
                received += 1
                self.websocket_frame(2, received.to_bytes(4, "big"))
        except (EOFError, OSError):
            # Dropped mobile links are routine; the board reconnects itself.
            return
        finally:
            if relay_id is not None:
                with self.server.relay_lock:
                    if self.server.relay_status.get("id") == relay_id:
                        self.server.relay_status = {**self.server.relay_status,
                            "connected": False, "updated_at": time.time()}

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/frame/ws":
            self.websocket_upload()
        elif path == "/api/relay/ws":
            self.websocket_upload(browser_auth=True)
        elif path == "/":
            self.respond(200, VIEWER.read_bytes(), "text/html; charset=utf-8")
        elif path == "/api/status":
            status = self.server.encoder.status()
            with self.server.relay_lock:
                status["relay"] = self.server.relay_status.copy()
            self.respond(200, json.dumps(status).encode(), "application/json")
        elif path == "/assets/hls-1.5.18.min.js":
            self.respond(200, VIEWER.with_name("vendor").joinpath("hls-1.5.18.min.js").read_bytes(), "application/javascript")
        elif path == "/snapshot.jpg":
            jpeg = self.server.encoder.snapshot()
            self.respond(200 if jpeg else 503, jpeg or b"No frame received", "image/jpeg" if jpeg else "text/plain")
        elif path == "/live/index.m3u8" or re.fullmatch(r"/live/frame_\d{9,20}\.ts", path):
            file = self.server.encoder.root / path.rsplit("/", 1)[-1]
            try:
                body = file.read_bytes()
            except FileNotFoundError:
                self.respond(404, b"Waiting for video segment")
            else:
                self.respond(200, body, "application/vnd.apple.mpegurl" if path.endswith(".m3u8") else "video/mp2t")
        else:
            self.respond(404, b"Not found")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--token", default=os.environ.get("CANMV_UPLOAD_TOKEN"), help="default: generate a random upload token")
    args = parser.parse_args()
    if not (1 <= args.port <= 65535 and 1 <= args.fps <= 30 and
            32 <= args.width <= 1920 and 2 <= args.height <= 1080 and
            args.width % 2 == 0 and args.height % 2 == 0):
        parser.error("invalid port/fps/dimensions; width and height must be even")
    if not shutil.which(args.ffmpeg):
        parser.error("FFmpeg not found; install FFmpeg with the libx264 encoder")
    token = args.token or secrets.token_urlsafe(24)
    if any(c in token for c in "\r\n"):
        parser.error("token cannot contain CR/LF")
    encoder = VideoEncoder(fps=args.fps, width=args.width, height=args.height, ffmpeg=args.ffmpeg)
    try:
        server = VideoServer((args.bind, args.port), encoder, token)
        try:
            print(f"Viewer: http://127.0.0.1:{args.port}/", flush=True)
            print(f"Upload paths: /api/frame/ws (WebSocket), /api/frame (HTTP)   UPLOAD_TOKEN = {token!r}", flush=True)
            print(f"Video: H.264/HLS {args.width}x{args.height} {args.fps} FPS; FFmpeg log: {encoder.root / 'ffmpeg.log'}", flush=True)
            print("For the phone hotspot, expose this port with a public tunnel and use its HTTPS URL.", flush=True)
            server.serve_forever()
        finally:
            server.server_close()
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        encoder.close()


if __name__ == "__main__":
    main()
