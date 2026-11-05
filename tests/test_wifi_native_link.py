#!/usr/bin/env python3
"""Native C JPEG/upload -> Rust receiver -> real FFmpeg integration.

Build host CMake targets and tools/wifi_receiver first. Uses loopback sockets,
FFmpeg and ffprobe; deliberately separate from hardware-free CTest contracts.
"""
import argparse
import base64
import hashlib
import http.client
import json
import os
from pathlib import Path
import re
import socket
import struct
import subprocess
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
TOKEN = "native-integration-token"


def request_http(port, method, path, data=None, headers=None):
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
    try:
        connection.request(method, path, data, headers or {})
        response = connection.getresponse()
        return response.status, response.read()
    finally:
        connection.close()


def exact(client, length):
    data = bytearray()
    while len(data) < length:
        part = client.recv(length - len(data))
        assert part, "unexpected EOF"
        data.extend(part)
    return bytes(data)


def ws_open(port, path, bearer=False):
    client = socket.create_connection(("127.0.0.1", port), timeout=3)
    key = base64.b64encode(os.urandom(16)).decode()
    authorization = f"Authorization: Bearer {TOKEN}\r\n" if bearer else ""
    client.sendall((f"GET {path} HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n"
                    "Connection: Upgrade\r\nUpgrade: websocket\r\n"
                    f"Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: {key}\r\n"
                    f"{authorization}\r\n").encode())
    headers = bytearray()
    while not headers.endswith(b"\r\n\r\n"):
        headers.extend(exact(client, 1))
    assert b" 101 " in headers
    accept = base64.b64encode(hashlib.sha1(
        (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
    assert accept in headers
    return client


def ws_send(client, opcode, data):
    length = len(data)
    extension = b""
    if length >= 65536:
        extension, length = struct.pack("!Q", length), 127
    elif length >= 126:
        extension, length = struct.pack("!H", length), 126
    mask = os.urandom(4)
    masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(data))
    client.sendall(bytes([0x80 | opcode, 0x80 | length]) + extension + mask + masked)


def ws_read(client):
    first, length = exact(client, 2)
    assert not length & 0x80
    length &= 0x7f
    if length == 126:
        length = struct.unpack("!H", exact(client, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", exact(client, 8))[0]
    return first & 15, exact(client, length)


def protocol_cases(probe):
    responses = [
        (b"HTTP/1.1 204 No Content\r\n\r\n", True),
        (b"HTTP/1.1 202 Accepted\r\nContent-Length: 2\r\n\r\n{}", True),
        (b"HTTP/1.1 401 Unauthorized\r\nContent-Length: 0\r\n\r\n", False),
        (b"HTTP/1.1 200 OK\r\nContent-Length: 99999\r\n\r\n", False),
        (b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\nContent-Length: 0\r\n\r\n", False),
        (b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n0\r\n\r\n", False),
        (b"HTTP/1.1 200 OK\r\nContent-Length: 10\r\n\r\n{}", False),
        (b"HTTP/1.1 204 No Content\r\nX-Fill: " + b"x" * 4096 + b"\r\n\r\n", False),
        (b"", False),
    ]
    for response, success in responses:
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            port = listener.getsockname()[1]

            def peer():
                connection, _ = listener.accept()
                with connection:
                    connection.settimeout(3)
                    reader = connection.makefile("rb")
                    reader.readline()
                    headers = {}
                    while (line := reader.readline()) != b"\r\n":
                        name, value = line.split(b":", 1)
                        headers[name.lower()] = value.strip()
                    reader.read(int(headers[b"content-length"]))
                    if not response:
                        time.sleep(1.2)
                    try:
                        for offset in range(0, len(response), 7):
                            connection.sendall(response[offset:offset + 7])
                            time.sleep(0.001)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    reader.close()

            worker = threading.Thread(target=peer, daemon=True)
            worker.start()
            result = subprocess.run([str(probe), str(port), TOKEN, "1", "-"],
                                    capture_output=True, timeout=3)
            worker.join(timeout=3)
            assert not worker.is_alive()
            assert (result.returncode == 0) == success, (response, result.stderr)
    print(f"HTTP response framing/timeouts: {len(responses)} scenarios passed")


def end_to_end(receiver, probe, directory):
    log = directory / "receiver.log"
    with log.open("wb") as terminal:
        process = subprocess.Popen([str(receiver), "--port", "0", "--token", TOKEN,
                                    "--output", str(directory / "video")], stderr=terminal)
        try:
            deadline = time.monotonic() + 10
            port = None
            while time.monotonic() < deadline:
                text = log.read_text()
                found = re.search(r"address=127\.0\.0\.1:(\d+)", text)
                if found:
                    port = int(found.group(1))
                    break
                assert process.poll() is None, text
                time.sleep(0.05)
            assert port is not None, log.read_text()
            assert request_http(port, "POST", "/api/frame", b"bad")[0] == 401
            auth = {"Authorization": f"Bearer {TOKEN}", "Content-Type": "image/jpeg"}
            assert request_http(port, "POST", "/api/frame", b"bad", auth)[0] == 422
            assert request_http(port, "GET", "/")[0] == 200
            assert request_http(port, "GET", "/assets/hls-1.5.18.min.js")[0] == 200
            fixture = directory / "frame.jpg"
            result = subprocess.run([str(probe), str(port), TOKEN, "60", str(fixture)],
                                    capture_output=True, timeout=15)
            assert result.returncode == 0, result.stderr
            data = fixture.read_bytes()
            for path, bearer in (("/api/frame/ws", True), ("/api/relay/ws", False)):
                with ws_open(port, path, bearer) as client:
                    if not bearer:
                        ws_send(client, 1, json.dumps({"token": TOKEN}).encode())
                        assert ws_read(client) == (1, b'{"ready":true}')
                    for count in (1, 2):
                        ws_send(client, 2, data)
                        assert ws_read(client) == (2, count.to_bytes(4, "big"))
                    ws_send(client, 8, b"\x03\xe8")
            status = json.loads(request_http(port, "GET", "/api/status")[1])
            assert status["received_frames"] == 64, status
            assert status["encoded_frames"] >= 40, status
            assert status["hls_ready"], status
            code, playlist = request_http(port, "GET", "/live/index.m3u8")
            assert code == 200 and b"#EXTM3U" in playlist
            segments = [line for line in playlist.decode().splitlines() if line.endswith(".ts")]
            assert segments and request_http(port, "GET", "/live/" + segments[-1])[0] == 200
        finally:
            process.terminate()
            try:
                result = process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise
    assert result == 0, log.read_text()
    recording, = (directory / "video").glob("*/recording.mp4")
    details = json.loads(subprocess.check_output([
        "ffprobe", "-v", "error", "-show_streams", "-of", "json", str(recording)]))
    stream, = details["streams"]
    assert (stream["codec_name"], stream["width"], stream["height"]) == ("h264", 320, 240)
    subprocess.run(["ffmpeg", "-v", "error", "-i", str(recording), "-f", "null", "-"],
                   check=True, timeout=10)
    print(f"C -> Rust -> HLS/MP4 passed; received=64, encoded={status['encoded_frames']}")
    # Waiting for a camera and stopping before the first JPEG is a normal exit.
    empty_log = directory / "empty.log"
    with empty_log.open("wb") as terminal:
        waiting = subprocess.Popen([str(receiver), "--port", "0", "--token", TOKEN,
                                    "--output", str(directory / "empty")], stderr=terminal)
        try:
            deadline = time.monotonic() + 5
            while b"op=start" not in empty_log.read_bytes():
                assert waiting.poll() is None
                assert time.monotonic() < deadline
                time.sleep(0.02)
            waiting.terminate()
            assert waiting.wait(timeout=10) == 0, empty_log.read_text()
        finally:
            if waiting.poll() is None:
                waiting.kill()
                waiting.wait()
    failed = subprocess.run([str(receiver), "--port", "0", "--token", TOKEN,
                             "--ffmpeg", "/bin/false", "--output", str(directory / "failed")],
                            capture_output=True, timeout=10)
    assert failed.returncode != 0 and b"FFmpeg exited" in failed.stderr
    print("FFmpeg failure propagation and graceful shutdown passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receiver", type=Path,
                        default=ROOT / "tools/wifi_receiver/target/debug/dart-wifi-receiver")
    parser.add_argument("--probe", type=Path, default=ROOT / "build/tests/wifi_link_probe")
    args = parser.parse_args()
    protocol_cases(args.probe.resolve())
    with tempfile.TemporaryDirectory(prefix="dart-wifi-link-") as path:
        end_to_end(args.receiver.resolve(), args.probe.resolve(), Path(path))


if __name__ == "__main__":
    main()
