"""Real HTTP and FFmpeg checks for the JPEG -> H.264/HLS backend."""

import http.client
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "tools/wifi/video_server.py"
spec = importlib.util.spec_from_file_location("wifi_video_server", SCRIPT)
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)


def sample_jpeg():
    return subprocess.run([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
        "testsrc2=size=64x48:rate=1", "-frames:v", "1", "-c:v", "mjpeg",
        "-threads", "1", "-f", "image2pipe", "pipe:1",
    ], check=True, capture_output=True, timeout=10).stdout


@unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"), "FFmpeg/ffprobe required")
class VideoServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.jpeg = sample_jpeg()

    def setUp(self):
        self.encoder = server_module.VideoEncoder(fps=5)
        try:
            self.server = server_module.VideoServer(("127.0.0.1", 0), self.encoder, "test-token")
        except BaseException:
            self.encoder.close()
            raise
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=5)

    def tearDown(self):
        self.client.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=3)
        self.encoder.close()

    def upload(self, body=None, **headers):
        self.client.request("POST", "/api/frame", self.jpeg if body is None else body, headers={
            "Authorization": "Bearer test-token", "Content-Type": "image/jpeg", **headers})
        response = self.client.getresponse()
        data = response.read()
        return response.status, data

    def get(self, path):
        self.client.request("GET", path)
        response = self.client.getresponse()
        data = response.read()
        return response.status, data

    def test_http_uploads_become_decodable_h264_hls_video(self):
        self.assertEqual(self.get("/")[0], 200)
        self.assertIn(b"/assets/hls-1.5.18.min.js", self.get("/")[1])
        self.assertIn(b"Hls", self.get("/assets/hls-1.5.18.min.js")[1])
        self.assertEqual(self.get("/assets/../wifi_test.py")[0], 404)
        self.assertEqual(self.upload()[0], 204)
        self.assertEqual(self.upload()[0], 204)  # same persistent HTTP connection
        # Exercise the actual MicroPython uploader against the real HTTP backend.
        board_spec = importlib.util.spec_from_file_location("board_uploader", SCRIPT.with_name("wifi_test.py"))
        board = importlib.util.module_from_spec(board_spec)
        board_spec.loader.exec_module(board)
        board.UPLOAD_FALLBACK_URLS = ()
        board.STREAM_FPS = 5
        board.UPLOAD_TRANSPORT = "http"
        board.UPLOAD_URL = f"http://127.0.0.1:{self.server.server_port}/api/frame"
        board.UPLOAD_TOKEN = "test-token"
        board.STREAM_SECONDS = 1
        board.time = SimpleNamespace(ticks_ms=lambda: int(time.monotonic() * 1000),
                                     ticks_diff=lambda end, start: end - start,
                                     sleep_ms=lambda ms: time.sleep(ms / 1000))
        camera = SimpleNamespace(open=lambda: None, close=lambda: None,
                                 capture=lambda: SimpleNamespace(bytearray=lambda: self.jpeg))
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("127.0.0.1",))
        result = {"frames": 0, "stream": "FAIL"}
        with patch.object(board, "StreamCamera", return_value=camera), \
             patch.object(os, "exitpoint", lambda *args: None, create=True), \
             patch.object(os, "EXITPOINT_ENABLE", 1, create=True), \
             patch.object(os, "EXITPOINT_ENABLE_SLEEP", 2, create=True):
            board.stream_camera(wlan, "127.0.0.1", result)
        self.assertEqual((result["frames"], result["stream"]), (5, "PASS"))
        self.assertEqual(self.get("/snapshot.jpg"), (200, self.jpeg))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            status, playlist = self.get("/live/index.m3u8")
            if status == 200 and b".ts" in playlist:
                break
            time.sleep(0.1)
        else:
            self.fail("No HLS output: " + (self.encoder.root / "ffmpeg.log").read_text())
        segments = [line for line in playlist.decode().splitlines() if line.endswith(".ts")]
        self.assertTrue(segments)
        status, video = self.get("/live/" + segments[0])
        self.assertEqual(status, 200)
        with tempfile.TemporaryDirectory() as directory:
            segment = Path(directory) / "video.ts"
            segment.write_bytes(video)
            probe = subprocess.run(["ffprobe", "-v", "error", "-show_streams", "-of", "json", str(segment)],
                                   check=True, capture_output=True, timeout=10)
            stream = json.loads(probe.stdout)["streams"][0]
            self.assertEqual((stream["codec_name"], stream["width"], stream["height"]), ("h264", 320, 240))
            decoded = subprocess.run(["ffmpeg", "-v", "error", "-i", str(segment), "-frames:v", "1",
                                      "-threads", "1", "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1"],
                                     check=True, capture_output=True, timeout=10).stdout
            self.assertEqual(len(decoded), 320 * 240 * 3)
            self.assertGreater(max(decoded) - min(decoded), 100)
        status, data = self.get("/api/status")
        metrics = json.loads(data)
        self.assertEqual(metrics["received_frames"], 7)
        self.assertTrue(metrics["hls_ready"])
        self.assertGreaterEqual(metrics["encoded_frames"], 5)
        self.assertFalse(metrics["error"])
        self.assertEqual(metrics["last_minute_frames"], 7)
        self.assertGreater(metrics["uptime_seconds"], 0)
        self.assertGreaterEqual(metrics["max_upload_gap_seconds"], 0)

    def board_module(self):
        spec = importlib.util.spec_from_file_location("websocket_board", SCRIPT.with_name("wifi_test.py"))
        board = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(board)
        board.UPLOAD_URL = f"http://127.0.0.1:{self.server.server_port}/api/frame"
        board.UPLOAD_FALLBACK_URLS = ()
        board.UPLOAD_TOKEN = "test-token"
        board.time = SimpleNamespace(ticks_ms=lambda: int(time.monotonic() * 1000),
                                     ticks_diff=lambda end, start: end - start,
                                     sleep_ms=lambda ms: time.sleep(ms / 1000))
        return board

    def browser_socket(self, path="/api/relay/ws", port=None):
        client = socket.create_connection(("127.0.0.1", port or self.server.server_port), timeout=5)
        client.sendall(("GET %s HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                        "Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
                        "Sec-WebSocket-Key: MDEyMzQ1Njc4OWFiY2RlZg==\r\n\r\n" % path).encode())
        reader = client.makefile("rb")
        self.assertIn(b"101", reader.readline())
        while reader.readline() != b"\r\n":
            pass
        self.addCleanup(reader.close)
        self.addCleanup(client.close)
        return client, reader

    def browser_send(self, client, opcode, data):
        size, mask = len(data), b"mask"
        header = bytes((0x80 | opcode, 0x80 | size)) if size < 126 else bytes((0x80 | opcode, 0xfe)) + size.to_bytes(2, "big")
        client.sendall(header + mask + bytes(value ^ mask[index & 3] for index, value in enumerate(data)))

    def browser_receive(self, reader):
        first, size = reader.read(2)
        if size == 126:
            size = int.from_bytes(reader.read(2), "big")
        elif size == 127:
            size = int.from_bytes(reader.read(8), "big")
        return first & 15, reader.read(size)

    def test_browser_relay_authenticates_without_custom_headers_and_uploads_jpeg(self):
        client, reader = self.browser_socket()
        self.browser_send(client, 1, b'{"token":"test-token"}')
        opcode, ready = self.browser_receive(reader)
        self.assertEqual((opcode, json.loads(ready)), (1, {"ready": True}))
        self.browser_send(client, 1, json.dumps({"state": "streaming", "local_frames": 3,
                                               "sent_frames": 3, "buffered_bytes": 0}).encode())
        for count in range(1, 4):
            self.browser_send(client, 2, self.jpeg)
            self.assertEqual(self.browser_receive(reader), (2, count.to_bytes(4, "big")))
        self.assertEqual(self.encoder.status()["received_frames"], 3)
        self.assertEqual(self.encoder.snapshot(), self.jpeg)
        self.assertEqual(self.server.relay_status["state"], "streaming")
        self.assertEqual(self.server.relay_status["local_frames"], 3)
        self.browser_send(client, 8, b"\x03\xe8")
        self.assertEqual(self.browser_receive(reader), (8, b"\x03\xe8"))

    def test_browser_relay_rejects_wrong_token_or_unauthenticated_images(self):
        for opcode, data, code in ((1, b'{"token":"wrong"}', 1008),
                                   (1, b'[]', 1008), (2, self.jpeg, 1002),
                                   (1, b"x" * 513, 1009)):
            with self.subTest(opcode=opcode, size=len(data)):
                client, reader = self.browser_socket()
                self.browser_send(client, opcode, data)
                self.assertEqual(self.browser_receive(reader), (8, code.to_bytes(2, "big")))
        self.assertEqual(self.encoder.status()["received_frames"], 0)

    def test_phone_relay_local_server_forwards_real_jpegs_and_counts_backend_receipts(self):
        spec = importlib.util.spec_from_file_location("phone_relay", SCRIPT.with_name("phone_relay.py"))
        relay = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(relay)
        board = self.board_module()
        board.STREAM_SECONDS = 2
        cfg = vars(board).copy()
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            cfg["PHONE_RELAY_PORT"] = reservation.getsockname()[1]
        cfg["STREAM_FPS"] = 15
        cfg["StreamCamera"] = lambda: SimpleNamespace(open=lambda: None, close=lambda: None,
                    capture=lambda: SimpleNamespace(bytearray=lambda: self.jpeg))
        cfg["connect_stream_wlan"] = lambda wlan, force=False: "127.0.0.1"
        ready = threading.Event()
        cfg["log"] = lambda stage, status, message="": ready.set() if status == "READY" else None
        result = {"frames": 0}
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("127.0.0.1",))
        failures = []
        original_open = open

        def relay_page(path, *args, **kwargs):
            return original_open(SCRIPT.with_name("phone_relay.html") if path == "/sdcard/phone_relay.html" else path, *args, **kwargs)

        def run():
            try:
                relay.stream_via_phone(wlan, "127.0.0.1", result, cfg)
            except BaseException as error:
                failures.append(error)

        with patch.object(time, "ticks_ms", board.time.ticks_ms, create=True), \
             patch.object(time, "ticks_diff", board.time.ticks_diff, create=True), \
             patch.object(time, "sleep_ms", board.time.sleep_ms, create=True), \
             patch.object(os, "exitpoint", lambda *args: None, create=True), \
             patch.object(os, "EXITPOINT_ENABLE", 1, create=True), \
             patch.object(os, "EXITPOINT_ENABLE_SLEEP", 2, create=True), \
             patch("builtins.open", relay_page):
            thread = threading.Thread(target=run, daemon=True)
            thread.start()
            try:
                self.assertTrue(ready.wait(2), failures)
                local_http = http.client.HTTPConnection("127.0.0.1", cfg["PHONE_RELAY_PORT"], timeout=2)
                local_http.request("GET", "/")
                response = local_http.getresponse()
                page = response.read()
                local_http.close()
                self.assertEqual(response.status, 200)
                self.assertIn(b"test-token", page)
                self.assertNotIn(b"__CANMV_CONFIG__", page)
                self.assertIn(b"/api/relay/ws", page)
                backend, backend_reader = self.browser_socket()
                self.browser_send(backend, 1, b'{"token":"test-token"}')
                self.assertEqual(self.browser_receive(backend_reader)[0], 1)
                camera, camera_reader = self.browser_socket("/camera", cfg["PHONE_RELAY_PORT"])
                for count in range(1, 4):
                    opcode, jpeg = self.browser_receive(camera_reader)
                    self.assertEqual((opcode, jpeg), (2, self.jpeg))
                    self.browser_send(backend, 2, jpeg)
                    opcode, accepted = self.browser_receive(backend_reader)
                    self.assertEqual((opcode, int.from_bytes(accepted, "big")), (2, count))
                    self.browser_send(camera, 2, count.to_bytes(4, "big") + accepted)
                self.browser_send(camera, 8, b"\x03\xe8")
                self.browser_send(backend, 8, b"\x03\xe8")
            finally:
                thread.join(5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual((result["frames"], result["stream"]), (3, "PASS"))
        self.assertEqual(self.encoder.status()["received_frames"], 3)

    def test_websocket_continues_sending_while_receipt_ack_is_delayed(self):
        board = self.board_module()
        release_ack = threading.Event()
        original = server_module.VideoHandler.websocket_frame

        def delayed_ack(handler, opcode, data):
            if opcode == 2:
                release_ack.wait(3)
            original(handler, opcode, data)

        with patch.object(os, "exitpoint", lambda *args: None, create=True), \
             patch.object(server_module.VideoHandler, "websocket_frame", delayed_ack):
            connection = board.JpegWebSocket.open("127.0.0.1", board.parse_upload_url(board.UPLOAD_URL))
            try:
                started = time.monotonic()
                for _ in range(8):
                    self.assertEqual(connection.wait_capacity(), 0)
                    connection.send_jpeg(self.jpeg)
                    time.sleep(0.025)
                # All eight independent images were sent without waiting for
                # the first acknowledgment to make a network round trip.
                self.assertLess(time.monotonic() - started, 1)
                self.assertEqual((connection.sent, connection.acknowledged), (8, 0))
                release_ack.set()
                self.assertEqual(connection.wait_capacity(drain=True), 8)
                self.assertEqual(self.encoder.status()["received_frames"], 8)
                self.assertEqual(self.encoder.snapshot(), self.jpeg)
                connection.send_frame(9, b"ping")
                connection.send_jpeg(self.jpeg)
                self.assertEqual(connection.wait_capacity(drain=True), 1)
            finally:
                release_ack.set()
                connection.close()

    def test_websocket_stream_loop_counts_only_confirmed_jpegs(self):
        board = self.board_module()
        board.STREAM_SECONDS = 1
        board.STREAM_FPS = 10
        result = {"frames": 0, "stream": "FAIL"}
        camera = SimpleNamespace(open=lambda: None, close=lambda: None,
                                 capture=lambda: SimpleNamespace(bytearray=lambda: self.jpeg))
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("127.0.0.1",))
        with patch.object(board, "StreamCamera", return_value=camera), \
             patch.object(os, "exitpoint", lambda *args: None, create=True), \
             patch.object(os, "EXITPOINT_ENABLE", 1, create=True), \
             patch.object(os, "EXITPOINT_ENABLE_SLEEP", 2, create=True):
            board.stream_camera(wlan, "127.0.0.1", result)
        self.assertEqual(result["stream"], "PASS")
        self.assertGreaterEqual(result["frames"], 9)
        self.assertEqual(result["frames"], self.encoder.status()["received_frames"])
        self.assertEqual(result["stream_stats"]["unconfirmed_frames"], 0)

    def test_websocket_stream_reconnects_after_the_server_drops_tcp(self):
        board = self.board_module()
        board.STREAM_SECONDS = 2
        board.STREAM_FPS = 20
        board.STREAM_MAX_IN_FLIGHT = 2
        board.STREAM_IO_TIMEOUT_MS = 100
        board.STREAM_RETRY_BASE_MS = board.STREAM_RETRY_MAX_MS = 10
        original = server_module.VideoHandler.websocket_frame
        dropped = threading.Event()

        def drop_once(handler, opcode, data):
            original(handler, opcode, data)
            if opcode == 2 and int.from_bytes(data, "big") == 3 and not dropped.is_set():
                dropped.set()
                handler.connection.shutdown(socket.SHUT_RDWR)

        result = {"frames": 0, "stream": "FAIL"}
        camera = SimpleNamespace(open=lambda: None, close=lambda: None,
                                 capture=lambda: SimpleNamespace(bytearray=lambda: self.jpeg))
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("127.0.0.1",))
        with patch.object(board, "StreamCamera", return_value=camera), \
             patch.object(server_module.VideoHandler, "websocket_frame", drop_once), \
             patch.object(os, "exitpoint", lambda *args: None, create=True), \
             patch.object(os, "EXITPOINT_ENABLE", 1, create=True), \
             patch.object(os, "EXITPOINT_ENABLE_SLEEP", 2, create=True):
            board.stream_camera(wlan, "127.0.0.1", result)
        self.assertTrue(dropped.is_set())
        self.assertEqual(result["stream"], "PASS")
        self.assertGreater(result["frames"], 20)
        self.assertGreaterEqual(result["stream_stats"]["upload_errors"], 1)
        self.assertGreaterEqual(self.encoder.status()["received_frames"], result["frames"])
        self.assertEqual(result["stream_stats"]["wifi_attempts"], 0)

    def test_websocket_auth_protocol_and_jpeg_limits(self):
        board = self.board_module()
        with patch.object(os, "exitpoint", lambda *args: None, create=True):
            board.UPLOAD_TOKEN = "wrong"
            with self.assertRaisesRegex(RuntimeError, "401"):
                board.JpegWebSocket.open("127.0.0.1", board.parse_upload_url(board.UPLOAD_URL))
            board.UPLOAD_TOKEN = "test-token"
            for invalid, code in ((b"not a jpeg", 1007), (self.jpeg[:-2], 1007),
                                  (b"x" * (server_module.MAX_JPEG_BYTES + 1), 1009)):
                connection = board.JpegWebSocket.open("127.0.0.1", board.parse_upload_url(board.UPLOAD_URL))
                try:
                    if code == 1009:
                        # Send the oversized declaration, not the rejected body.
                        board.send_all(connection.client, b"\x82\xff" + len(invalid).to_bytes(8, "big"))
                        connection.sent = 1
                    else:
                        connection.send_jpeg(invalid)
                    with self.assertRaisesRegex(RuntimeError, str(code)):
                        connection.wait_capacity(drain=True)
                finally:
                    connection.close()
        self.assertEqual(self.encoder.status()["received_frames"], 0)

    def test_websocket_requires_masked_client_frames(self):
        board = self.board_module()
        with patch.object(os, "exitpoint", lambda *args: None, create=True):
            connection = board.JpegWebSocket.open("127.0.0.1", board.parse_upload_url(board.UPLOAD_URL))
            try:
                board.send_all(connection.client, b"\x82\x04jpeg")
                connection.sent = 1
                with self.assertRaisesRegex(RuntimeError, "1002"):
                    connection.wait_capacity(drain=True)
            finally:
                connection.close()
        self.assertEqual(self.encoder.status()["received_frames"], 0)

    def test_token_body_and_path_validation(self):
        self.assertEqual(self.upload(Authorization="Bearer wrong")[0], 401)
        self.assertEqual(self.encoder.status()["received_frames"], 0)
        self.assertEqual(self.upload(b"not a jpeg")[0], 400)
        self.assertEqual(self.upload(**{"Content-Type": "text/plain"})[0], 415)
        self.assertEqual(self.upload(**{"Content-Length": str(server_module.MAX_JPEG_BYTES + 1)})[0], 413)
        for path in ("/live/../../etc/passwd", "/live/ffmpeg.log", "/live/frame_fake.ts"):
            self.assertEqual(self.get(path)[0], 404)
        self.assertFalse(self.encoder.status()["hls_ready"])

    def test_missing_snapshot_is_not_an_old_or_fake_frame(self):
        self.assertEqual(self.get("/snapshot.jpg")[0], 503)
        state = json.loads(self.get("/api/status")[1])
        self.assertEqual(state["received_frames"], 0)
        self.assertFalse(state["online"])

    def test_jpeg_header_validation(self):
        self.assertEqual(server_module.jpeg_size(self.jpeg), (64, 48))
        for data in (b"", b"\xff\xd8\xff\xd9", self.jpeg[:-2], b"\xff\xd8\xff\xc0\x00\x01\xff\xd9"):
            with self.subTest(length=len(data)), self.assertRaises(ValueError):
                server_module.jpeg_size(data)


if __name__ == "__main__":
    unittest.main()
