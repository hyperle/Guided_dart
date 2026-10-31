"""Host simulations for the standalone CanMV Wi-Fi diagnostic (no real radio)."""

import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "tools/wifi/wifi_test.py"


class FakeWLAN:
    def __init__(self):
        self.enabled = True
        self.connected = False
        self.join = True
        self.accepted = True
        self.scan_error = None
        self.scan_calls = 0
        self.scan_sequence = None
        self.join_error = None
        self.disconnects = 0
        self.calls = []
        self.ip = "192.168.10.42"
        self.results = [SimpleNamespace(ssid=b"Test AP", bssid=b"\x00\x01\x02\x03\x04\x05",
                                        channel=6, rssi=-42, security=0x400004)]

    def active(self, value=None):
        return self.enabled

    def config(self, key):
        return b"\x00\x01\x02\x03\x04\x05"

    def scan(self):
        self.scan_calls += 1
        if self.scan_sequence is not None:
            return next(self.scan_sequence)
        if self.scan_error:
            raise self.scan_error
        return self.results

    def connect(self, *args):
        self.calls.append(args)
        if self.join_error:
            raise self.join_error
        self.connected = self.join
        return self.accepted

    def isconnected(self):
        return self.connected

    def ifconfig(self):
        return (self.ip, "255.255.255.0", "192.168.10.1", "192.168.10.1")

    def status(self, key):
        return -42

    def disconnect(self):
        self.disconnects += 1
        self.connected = False
        return True


class WifiTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location("wifi_under_test", SCRIPT)
        self.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.script)
        # Keep tests independent from the user's local SSID/password settings.
        self.script.SSID = ""
        self.script.STREAM_ENABLED = False
        self.script.STREAM_MODE = "direct"
        self.script.PASSWORD = ""
        self.script.WLAN_DEVICE = "sdio"
        self.script.SCAN_ATTEMPTS = 3
        self.script.SCAN_RETRY_DELAY_MS = 2000
        self.script.DISCONNECT_ON_EXIT = False
        self.script.REUSE_CONNECTION = False
        self.script.TCP_TEST_HOST = ""
        self.script.TCP_TEST_PORT = 8000
        self.script.TCP_TIMEOUT_S = 3
        self.wlan = FakeWLAN()
        self.constructors = []
        self.network = SimpleNamespace(STA_IF=0, WLAN_SDIO=2, WLAN_AUTO=0,
                                       WLAN=self.construct, get_dev_list=lambda: ["wlan0"])
        self.clock = 0
        self.script.time = SimpleNamespace(ticks_ms=lambda: self.clock,
                                           ticks_diff=lambda end, start: end - start,
                                           sleep_ms=self.sleep)
        self.script.CONNECT_TIMEOUT_MS = 1000
        self.script.CONNECT_ATTEMPTS = 1
        self.script.CONNECT_RETRY_DELAY_MS = 2000
        self.script.MONITOR_SECONDS = 2
        self.output = io.StringIO()

    def construct(self, *args):
        self.constructors.append(args)
        return self.wlan

    def sleep(self, milliseconds):
        self.clock += milliseconds

    def run_probe(self):
        with patch.dict(sys.modules, network=self.network), contextlib.redirect_stdout(self.output):
            return self.script.main()

    def connect_mode(self):
        self.script.SSID = "Test AP"
        self.script.PASSWORD = "password123"

    def test_canmv_object_scan_selects_sdio_and_preserves_connection(self):
        self.wlan.connected = True
        result = self.run_probe()
        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["scan_count"], 1)
        self.assertEqual(self.constructors, [(0, 2)])
        self.assertIn("Test AP", self.output.getvalue())
        self.assertEqual(self.wlan.disconnects, 0)
        self.assertFalse(self.wlan.calls)

    def test_legacy_api_and_tuple_scan_with_invalid_utf8(self):
        del self.network.WLAN_SDIO
        self.wlan.results = [(b"\xff", b"\x00" * 6, 1, -80, 3, False)]
        self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertEqual(self.constructors, [(0,)])
        self.assertIn("legacy API", self.output.getvalue())

    def test_empty_scan_is_inconclusive(self):
        self.wlan.results = []
        self.assertEqual(self.run_probe()["result"], "INCONCLUSIVE")

    def test_empty_scan_retries_and_reports_duration(self):
        self.wlan.results = []
        result = self.run_probe()
        self.assertEqual(result["result"], "INCONCLUSIVE")
        self.assertEqual(self.wlan.scan_calls, 3)
        self.assertEqual(self.clock, 4000)
        self.assertIn("attempt=3/3 elapsed_ms=0 count=0", self.output.getvalue())

    def test_scan_recovers_after_empty_result_and_stops_retrying(self):
        self.wlan.scan_sequence = iter([[], self.wlan.results])
        self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertEqual(self.wlan.scan_calls, 2)
        self.assertEqual(self.clock, 2000)
        self.assertIn("scan PASS attempt=2/3", self.output.getvalue())

    def test_scan_duration_handles_ticks_wrap(self):
        modulus = 1 << 30
        self.clock = modulus - 100
        self.script.time.ticks_ms = lambda: self.clock % modulus
        self.script.time.ticks_diff = lambda end, start: ((end - start + modulus // 2) % modulus
                                                         - modulus // 2)
        def scan():
            self.clock += 1456
            return self.wlan.results
        self.wlan.scan = scan
        self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertIn("elapsed_ms=1456 count=1", self.output.getvalue())

    def test_invalid_retry_settings_rejected_before_radio_use(self):
        for key, value in (("SCAN_ATTEMPTS", 0), ("SCAN_ATTEMPTS", 1.5),
                           ("SCAN_ATTEMPTS", 11), ("SCAN_RETRY_DELAY_MS", -1),
                           ("SCAN_RETRY_DELAY_MS", 10001)):
            with self.subTest(key=key, value=value):
                previous = getattr(self.script, key)
                setattr(self.script, key, value)
                self.assertEqual(self.run_probe()["result"], "FAIL")
                self.assertFalse(self.constructors)
                setattr(self.script, key, previous)

    def test_no_scan_results_is_failure(self):
        self.wlan.results = None
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertEqual(self.wlan.scan_calls, 1)
        self.assertIn("driver returned no scan result", self.output.getvalue())

    def test_scan_error_is_failure(self):
        self.wlan.scan_error = OSError("radio unavailable")
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertIn("scan FAIL", self.output.getvalue())

    def test_missing_firmware_api_or_inactive_radio_fails(self):
        del self.network.WLAN
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertIn("network.WLAN unavailable", self.output.getvalue())
        self.network.WLAN = self.construct
        self.wlan.enabled = False
        self.assertEqual(self.run_probe()["result"], "FAIL")

    def test_connection_and_ip_monitoring(self):
        self.connect_mode()
        self.assertEqual(self.run_probe(), {"result": "PASS", "scan_count": 1,
                                           "ip": "192.168.10.42", "tcp": "SKIP",
                                           "stream": "SKIP", "frames": 0})
        self.assertIn("monitor PASS 2/2", self.output.getvalue())
        self.assertEqual(self.wlan.calls, [("Test AP", "password123")])
        self.assertNotIn("password123", self.output.getvalue())
        self.assertEqual(self.wlan.disconnects, 0)

    def test_open_network_omits_password_and_accepts_none_return(self):
        self.script.SSID = "Open AP"
        self.wlan.accepted = None
        self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertEqual(self.wlan.calls, [("Open AP",)])

    def test_old_association_cannot_satisfy_new_test(self):
        self.connect_mode()
        self.wlan.connected = True
        self.wlan.join = False
        self.wlan.accepted = None
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertEqual(self.wlan.disconnects, 2)
        self.assertIn("link/IP timeout", self.output.getvalue())

    def test_zero_ip_times_out_and_cancels_join(self):
        self.connect_mode()
        self.wlan.ip = "0.0.0.0"
        result = self.run_probe()
        self.assertEqual(result["result"], "FAIL")
        self.assertIsNone(result["ip"])
        self.assertEqual(self.clock, 1000)
        self.assertEqual(self.wlan.disconnects, 1)

    def test_driver_rejection_and_password_redaction(self):
        self.connect_mode()
        self.wlan.accepted = False
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.wlan.accepted = True
        self.wlan.join_error = OSError("failed with password123")
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertNotIn("password123", self.output.getvalue())
        self.assertIn("<redacted>", self.output.getvalue())

    def test_interrupt_cancels_incomplete_join(self):
        self.connect_mode()
        self.wlan.join_error = KeyboardInterrupt()
        self.assertEqual(self.run_probe()["result"], "STOPPED")
        self.assertEqual(self.wlan.disconnects, 1)

    def test_link_loss_during_monitoring_is_failure(self):
        self.connect_mode()
        def lose_link(milliseconds):
            self.clock += milliseconds
            self.wlan.connected = False
        self.script.time.sleep_ms = lose_link
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertIn("link/IP lost", self.output.getvalue())

    def test_timeout_poll_handles_ticks_wrap(self):
        self.connect_mode()
        self.wlan.join = False
        modulus = 1 << 30
        self.clock = modulus - 400
        self.script.time.ticks_ms = lambda: self.clock % modulus
        self.script.time.ticks_diff = lambda end, start: ((end - start + modulus // 2) % modulus
                                                         - modulus // 2)
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertEqual(self.clock, modulus + 600)

    def test_invalid_config_fails_before_network_use(self):
        self.script.SSID = "中" * 11  # 33 bytes
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertFalse(self.constructors)

    def test_optional_tcp_binds_to_wlan_and_closes_on_failure(self):
        self.connect_mode()
        self.script.TCP_TEST_HOST = "192.168.10.2"
        events = []
        class Socket:
            def settimeout(self, seconds): events.append(("timeout", seconds))
            def bind(self, address): events.append(("bind", address))
            def connect(self, address):
                events.append(("connect", address))
                if fail:
                    raise OSError("connection refused")
            def close(self): events.append(("close",))
        socket = SimpleNamespace(AF_INET=2, SOCK_STREAM=1, socket=lambda *args: Socket())
        for fail in (False, True):
            with patch.dict(sys.modules, socket=socket):
                result = self.run_probe()
            self.assertEqual(result["result"], "FAIL" if fail else "PASS")
            self.assertEqual(result["tcp"], "FAIL" if fail else "PASS")
            self.assertIn(("bind", ("192.168.10.42", 0)), events)
            self.assertEqual(events[-1], ("close",))

    def test_same_ssid_ip_can_be_reused_without_another_join(self):
        self.connect_mode()
        self.script.REUSE_CONNECTION = True
        self.wlan.connected = True
        self.wlan.status = lambda key: SimpleNamespace(ssid=b"Test AP") if key == "ap" else -42
        self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertFalse(self.wlan.calls)
        self.assertEqual(self.wlan.disconnects, 0)
        self.assertIn("association/password not retested", self.output.getvalue())

    def test_reuse_rejects_other_or_unknown_ssid_and_missing_ip(self):
        self.connect_mode()
        self.script.REUSE_CONNECTION = True
        for ap, ip in ((SimpleNamespace(ssid=b"Other AP"), self.wlan.ip),
                       (-42, self.wlan.ip), (SimpleNamespace(ssid=b"Test AP"), "0.0.0.0")):
            with self.subTest(ap=ap, ip=ip):
                self.wlan.connected = True
                self.wlan.status = lambda key: ap
                self.wlan.ip = ip
                self.assertFalse(self.script.reusable_connection(self.wlan))

    def test_explicit_disconnect_also_cleans_up_reused_connection(self):
        self.connect_mode()
        self.script.REUSE_CONNECTION = True
        self.script.DISCONNECT_ON_EXIT = True
        self.wlan.connected = True
        self.wlan.status = lambda key: SimpleNamespace(ssid=b"Test AP") if key == "ap" else -42
        self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertEqual(self.wlan.disconnects, 1)
        self.assertFalse(self.wlan.calls)

    def test_rejected_join_is_cancelled_before_retry_and_can_recover(self):
        self.connect_mode()
        self.script.CONNECT_ATTEMPTS = 3
        events = []
        def connect(*args):
            events.append("connect")
            self.wlan.connected = events.count("connect") > 1
            return self.wlan.connected
        def disconnect():
            events.append("disconnect")
            self.wlan.connected = False
            return True
        self.wlan.connect = connect
        self.wlan.disconnect = disconnect
        result = self.run_probe()
        self.assertEqual(result["result"], "PASS")
        self.assertEqual(events, ["connect", "disconnect", "connect"])
        self.assertEqual(self.clock, 4000)  # 2 s retry + 2 s monitor
        self.assertIn("attempt=2/3", self.output.getvalue())
        self.assertNotIn("password123", self.output.getvalue())

    def test_persistent_rejection_is_bounded_and_never_claims_an_ip(self):
        self.connect_mode()
        self.script.CONNECT_ATTEMPTS = 3
        self.wlan.accepted = False
        self.wlan.join = False
        result = self.run_probe()
        self.assertEqual(result["result"], "FAIL")
        self.assertEqual(len(self.wlan.calls), 3)
        self.assertEqual(self.wlan.disconnects, 3)  # two retries + final cleanup
        self.assertEqual(self.clock, 4000)
        self.assertIsNone(result["ip"])
        self.assertEqual(result["stream"], "SKIP")
        self.assertIn("firmware supplied no reason code", self.output.getvalue())

    def test_cancel_failure_aborts_retries(self):
        self.connect_mode()
        self.script.CONNECT_ATTEMPTS = 3
        self.wlan.accepted = False
        self.wlan.disconnect = lambda: False
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertEqual(len(self.wlan.calls), 1)
        self.assertIn("cannot cancel rejected association", self.output.getvalue())

    def test_driver_exception_and_interrupt_are_not_retried(self):
        self.connect_mode()
        self.script.CONNECT_ATTEMPTS = 3
        for error, outcome in ((OSError("password123"), "FAIL"), (KeyboardInterrupt(), "STOPPED")):
            with self.subTest(outcome=outcome):
                self.wlan.calls.clear()
                self.wlan.join_error = error
                self.assertEqual(self.run_probe()["result"], outcome)
                self.assertEqual(len(self.wlan.calls), 1)
                self.assertNotIn("password123", self.output.getvalue())

    def test_retry_duration_handles_ticks_wrap(self):
        self.connect_mode()
        self.script.CONNECT_ATTEMPTS = 2
        self.wlan.accepted = False
        self.wlan.join = False
        modulus = 1 << 30
        self.clock = modulus - 100
        self.script.time.ticks_ms = lambda: self.clock % modulus
        self.script.time.ticks_diff = lambda end, start: (end - start + modulus // 2) % modulus - modulus // 2
        original = self.wlan.connect
        def slow_connect(*args):
            self.clock += 6300
            return original(*args)
        self.wlan.connect = slow_connect
        self.assertEqual(self.run_probe()["result"], "FAIL")
        self.assertEqual(self.output.getvalue().count("elapsed_ms=6300"), 2)

    def test_invalid_connect_retry_settings_fail_before_radio_use(self):
        for name, value in (("CONNECT_ATTEMPTS", 0), ("CONNECT_ATTEMPTS", 6),
                            ("CONNECT_ATTEMPTS", 1.5), ("CONNECT_RETRY_DELAY_MS", -1),
                            ("CONNECT_RETRY_DELAY_MS", 10001)):
            with self.subTest(name=name, value=value):
                original = getattr(self.script, name)
                setattr(self.script, name, value)
                self.assertEqual(self.run_probe()["result"], "FAIL")
                self.assertFalse(self.constructors)
                setattr(self.script, name, original)

    def test_stream_mode_does_not_scan_an_established_link(self):
        self.connect_mode()
        self.script.STREAM_ENABLED = True
        self.script.UPLOAD_FALLBACK_URLS = ()
        self.script.UPLOAD_IPV4_HINTS = {}
        self.script.STREAM_FPS = 5
        self.script.UPLOAD_TRANSPORT = "http"
        self.script.UPLOAD_URL = "http://192.168.10.2:8080/api/frame"
        self.script.UPLOAD_TOKEN = "test-token"
        self.script.REUSE_CONNECTION = True
        self.wlan.connected = True
        self.wlan.status = lambda key: SimpleNamespace(ssid=b"Test AP") if key == "ap" else -42
        def stream(wlan, ip, result):
            result.update(stream="PASS", frames=1, ip=ip)
        with patch.object(self.script, "stream_camera", side_effect=stream):
            self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertEqual(self.wlan.scan_calls, 0)
        self.assertFalse(self.wlan.calls)
        self.assertEqual(self.wlan.disconnects, 0)

    def test_explicit_disconnect_after_success(self):
        self.connect_mode()
        self.script.DISCONNECT_ON_EXIT = True
        self.assertEqual(self.run_probe()["result"], "PASS")
        self.assertEqual(self.wlan.disconnects, 1)


class UploadTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location("wifi_upload_test", SCRIPT)
        self.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.script)
        self.clock = 0
        self.script.time = SimpleNamespace(ticks_ms=lambda: self.clock,
                                           ticks_diff=lambda end, start: end - start,
                                           sleep_ms=self.sleep)
        self.os = SimpleNamespace(exitpoint=lambda *args: None, EXITPOINT_ENABLE=1,
                                  EXITPOINT_ENABLE_SLEEP=2)
        self.os_patch = patch.dict(sys.modules, os=self.os)
        self.os_patch.start()
        self.addCleanup(self.os_patch.stop)
        self.output = io.StringIO()
        self.stdout = contextlib.redirect_stdout(self.output)
        self.stdout.__enter__()
        self.addCleanup(self.stdout.__exit__, None, None, None)
        self.script.UPLOAD_FALLBACK_URLS = ()
        self.script.UPLOAD_IPV4_HINTS = {}
        self.script.STREAM_FPS = 5
        self.script.UPLOAD_TRANSPORT = "http"
        self.script.UPLOAD_URL = "http://192.168.10.2:8080/api/frame"
        self.script.UPLOAD_TOKEN = "test-token"
        self.script.SSID = "Test AP"
        self.script.PASSWORD = "password123"

    def sleep(self, milliseconds):
        self.clock += milliseconds

    def test_urls_and_invalid_settings(self):
        self.assertEqual(self.script.parse_upload_url("https://demo.example/api/frame"),
                         ("https", "demo.example", 443, "demo.example", "/api/frame"))
        for url in ("ftp://host", "http://u:p@host/", "http://host:0", "http://host:abc",
                    "http://host/\r\nInjected:yes", "http://host/#fragment"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                self.script.parse_upload_url(url)
        for key, value in (("STREAM_FPS", 0), ("STREAM_WIDTH", 319), ("JPEG_QUALITY", 101),
                           ("UPLOAD_TOKEN", ""), ("UPLOAD_TOKEN", "x\r\ny"),
                           ("CAMERA_BACKEND", "unknown")):
            previous = getattr(self.script, key)
            setattr(self.script, key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.script.check_settings()
            setattr(self.script, key, previous)

    def test_partial_sends_and_would_block_preserve_every_byte(self):
        writes = bytearray()
        calls = 0
        def write(data):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise OSError(11)
            count = min(3, len(data))
            writes.extend(data[:count])
            return count
        self.script.send_all(SimpleNamespace(write=write), b"0123456789")
        self.assertEqual(writes, b"0123456789")
        self.assertGreater(self.clock, 0)

    def test_stalled_send_times_out_across_ticks_wrap(self):
        self.script.STREAM_IO_TIMEOUT_MS = 10
        modulus = 1 << 30
        self.clock = modulus - 4
        self.script.time.ticks_ms = lambda: self.clock % modulus
        self.script.time.ticks_diff = lambda end, start: (end - start + modulus // 2) % modulus - modulus // 2
        with self.assertRaises(OSError):
            self.script.send_all(SimpleNamespace(write=lambda data: None), b"jpeg")
        self.assertEqual(self.clock, modulus + 6)

    def test_zero_write_is_disconnection(self):
        with self.assertRaises(OSError):
            self.script.send_all(SimpleNamespace(write=lambda data: 0), b"jpeg")

    def test_fragmented_ack_and_rejection(self):
        chunks = iter([None, b"HTTP/1.1 204 No", b" Content\r\nContent-Length: 0\r\n", b"\r\n"])
        self.script.read_upload_response(SimpleNamespace(read=lambda size: next(chunks)))
        with self.assertRaisesRegex(RuntimeError, "401"):
            self.script.read_upload_response(SimpleNamespace(read=lambda size: b"HTTP/1.1 401 No\r\n\r\n"))

    def test_empty_ack_times_out(self):
        self.script.STREAM_IO_TIMEOUT_MS = 20
        with self.assertRaises(OSError):
            self.script.read_upload_response(SimpleNamespace(read=lambda size: b""))

    def test_websocket_fragmented_ack_confirms_only_received_count(self):
        chunks = iter([b"\x82", b"\x04\x00\x00", b"\x00\x02", None])
        connection = self.script.JpegWebSocket(SimpleNamespace(read=lambda size: next(chunks)))
        connection.sent = 2
        self.assertEqual(connection.poll(), 2)
        self.assertEqual(connection.acknowledged, 2)
        connection.buffer = bytearray(b"\x82\x04\x00\x00\x00\x03")
        with self.assertRaisesRegex(RuntimeError, "count"):
            connection.poll()

    def test_websocket_flight_window_waits_for_ack_before_another_capture(self):
        sent = False
        def read(size):
            nonlocal sent
            if self.clock < 6 or sent:
                return None
            sent = True
            return b"\x82\x04\x00\x00\x00\x02"
        self.script.STREAM_MAX_IN_FLIGHT = 2
        connection = self.script.JpegWebSocket(SimpleNamespace(read=read))
        connection.sent = 2
        self.assertEqual(connection.wait_capacity(), 2)
        self.assertEqual(self.clock, 6)
        self.assertEqual(connection.acknowledged, 2)

    def test_websocket_missing_ack_times_out_across_ticks_wrap(self):
        modulus = 1 << 30
        self.clock = modulus - 4
        self.script.STREAM_IO_TIMEOUT_MS = 10
        self.script.time.ticks_ms = lambda: self.clock % modulus
        self.script.time.ticks_diff = lambda end, start: (end - start + modulus // 2) % modulus - modulus // 2
        connection = self.script.JpegWebSocket(SimpleNamespace(read=lambda size: None))
        connection.sent = 1
        with self.assertRaisesRegex(OSError, "stalled"):
            connection.wait_capacity(drain=True)
        self.assertEqual(self.clock, modulus + 6)

    def fake_camera(self):
        events = []
        class Camera:
            def open(self): events.append("open")
            def capture(self):
                events.append("capture")
                return SimpleNamespace(bytearray=lambda: b"\xff\xd8jpeg\xff\xd9")
            def close(self): events.append("close")
        return Camera, events

    def test_stream_reuses_connection_and_counts_acknowledged_frames(self):
        camera, events = self.fake_camera()
        clients = []
        writes = bytearray()
        client = SimpleNamespace(write=lambda data: writes.extend(data) or len(data),
                                 read=lambda size: b"HTTP/1.1 204 No Content\r\n\r\n",
                                 close=lambda: events.append("socket-close"))
        def connect(ip, target):
            clients.append(ip)
            return client
        self.script.STREAM_SECONDS = 1
        result = {"frames": 0, "stream": "FAIL"}
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("192.168.10.42",))
        with patch.object(self.script, "StreamCamera", camera), patch.object(self.script, "open_upload", connect):
            self.script.stream_camera(wlan, "192.168.10.42", result)
        self.assertEqual((result["frames"], result["stream"]), (5, "PASS"))
        self.assertEqual(clients, ["192.168.10.42"])
        self.assertEqual(writes.count(b"POST /api/frame HTTP/1.1"), 5)
        self.assertIn(b"Authorization: Bearer test-token\r\n", writes)
        self.assertEqual(events[-2:], ["socket-close", "close"])
        self.assertNotIn("test-token", self.output.getvalue())

    def test_dart_camera_startup_refreshes_radio_once_before_upload(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                self.clock = 0
                self.script.STREAM_SECONDS = 1
                self.script.STREAM_RADIO_RECOVERY = enabled
                camera, events = self.fake_camera()
                camera.raw = object()
                client = SimpleNamespace(write=lambda data: len(data),
                                         read=lambda size: b"HTTP/1.1 204 OK\r\n\r\n", close=lambda: None)
                def recover(wlan, force=False):
                    self.assertTrue(force)
                    self.assertEqual(events, ["open"])
                    return "ip"
                wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("ip",))
                result = {"frames": 0, "stream": "FAIL"}
                with patch.object(self.script, "StreamCamera", camera), \
                     patch.object(self.script, "open_upload", return_value=client), \
                     patch.object(self.script, "connect_stream_wlan", side_effect=recover) as refresh:
                    self.script.stream_camera(wlan, "ip", result)
                self.assertEqual(refresh.call_count, int(enabled))
                self.assertEqual(result["frames"], 5)
                self.assertEqual(events.count("open"), 1)

    def test_sparse_acks_do_not_prevent_recovery_after_repeated_stalls(self):
        camera, events = self.fake_camera()
        class Client:
            def __init__(self): self.writes = 0
            def write(self, data):
                self.writes += 1
                if self.writes > 2:
                    raise OSError(104)
                return len(data)
            def read(self, size): return b"HTTP/1.1 204 OK\r\n\r\n"
            def close(self): pass
        self.script.STREAM_SECONDS = 3
        self.script.STREAM_RETRY_BASE_MS = self.script.STREAM_RETRY_MAX_MS = 10
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("ip",))
        result = {"frames": 0, "stream": "FAIL"}
        with patch.object(self.script, "StreamCamera", camera), \
             patch.object(self.script, "open_upload", side_effect=lambda *args: Client()), \
             patch.object(self.script, "connect_stream_wlan", return_value="ip") as refresh:
            self.script.stream_camera(wlan, "ip", result)
        self.assertGreater(result["frames"], 3)
        self.assertGreaterEqual(result["stream_stats"]["upload_errors"], 3)
        self.assertGreaterEqual(refresh.call_count, 1)
        for call in refresh.call_args_list:
            self.assertEqual(call.kwargs, {"force": True})

    def test_outage_does_not_exit_after_three_failures_or_claim_upload(self):
        camera, events = self.fake_camera()
        attempts = []
        def fail(ip, target):
            attempts.append(ip)
            raise OSError(111)
        self.script.STREAM_SECONDS = 1
        self.script.STREAM_RETRY_BASE_MS = 50
        self.script.STREAM_RETRY_MAX_MS = 100
        result = {"frames": 0, "stream": "FAIL"}
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("ip",))
        with patch.object(self.script, "StreamCamera", camera), \
             patch.object(self.script, "open_upload", fail), \
             patch.object(self.script, "connect_stream_wlan", return_value="ip"):
            self.script.stream_camera(wlan, "ip", result)
        self.assertGreater(len(attempts), 3)
        self.assertEqual(result["frames"], 0)
        self.assertEqual(result["stream"], "INCONCLUSIVE")
        self.assertEqual(result["stream_stats"]["dropped_frames"], len(attempts))
        self.assertEqual(events[-1], "close")
        self.assertLess(self.clock, 1200)

    def test_wifi_drop_with_new_ip_rebinds_and_continues(self):
        camera, events = self.fake_camera()
        addresses = []
        state = {"connected": True, "ip": "old-ip", "dropped": False}
        def sleep(milliseconds):
            self.clock += milliseconds
            if not state["dropped"]:
                state.update(connected=False, dropped=True)
        self.script.time.sleep_ms = sleep
        def recover(wlan, force=False):
            state.update(connected=True, ip="new-ip")
            return "new-ip"
        def connect(ip, target):
            addresses.append(ip)
            return SimpleNamespace(write=lambda data: len(data),
                                   read=lambda size: b"HTTP/1.1 204 OK\r\n\r\n",
                                   close=lambda: events.append("socket-close"))
        wlan = SimpleNamespace(isconnected=lambda: state["connected"], ifconfig=lambda: (state["ip"],))
        self.script.STREAM_SECONDS = 1
        result = {"frames": 0, "stream": "FAIL"}
        with patch.object(self.script, "StreamCamera", camera), \
             patch.object(self.script, "open_upload", connect), \
             patch.object(self.script, "connect_stream_wlan", side_effect=recover):
            self.script.stream_camera(wlan, "old-ip", result)
        self.assertEqual(addresses, ["old-ip", "new-ip"])
        self.assertEqual(result["frames"], 5)
        self.assertEqual(result["ip"], "new-ip")
        self.assertEqual(events.count("open"), 1)
        self.assertEqual(events.count("close"), 1)

    def test_three_transport_failures_trigger_association_refresh_then_recover(self):
        camera, events = self.fake_camera()
        calls = []
        def connect(ip, target):
            calls.append(ip)
            if len(calls) <= 3:
                raise OSError(110)
            return SimpleNamespace(write=lambda data: len(data),
                                   read=lambda size: b"HTTP/1.1 204 OK\r\n\r\n",
                                   close=lambda: events.append("socket-close"))
        self.script.STREAM_SECONDS = 2
        self.script.STREAM_RETRY_BASE_MS = 50
        self.script.STREAM_RETRY_MAX_MS = 100
        result = {"frames": 0, "stream": "FAIL"}
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("ip",))
        with patch.object(self.script, "StreamCamera", camera), \
             patch.object(self.script, "open_upload", connect), \
             patch.object(self.script, "connect_stream_wlan", return_value="ip") as recover:
            self.script.stream_camera(wlan, "ip", result)
        recover.assert_called_once_with(wlan, force=True)
        self.assertEqual(result["stream"], "PASS")
        self.assertGreater(result["frames"], 0)
        self.assertEqual(result["stream_stats"]["upload_errors"], 3)
        self.assertEqual(result["stream_stats"]["dropped_frames"], 3)
        self.assertGreater(events.count("capture"), result["frames"])

    def test_http_transient_retries_but_bad_token_is_fatal(self):
        with self.assertRaises(OSError):
            self.script.read_upload_response(SimpleNamespace(read=lambda size: b"HTTP/1.1 503 Busy\r\n\r\n"))
        with self.assertRaisesRegex(RuntimeError, "401"):
            self.script.read_upload_response(SimpleNamespace(read=lambda size: b"HTTP/1.1 401 No\r\n\r\n"))
        self.assertFalse(self.script.read_upload_response(SimpleNamespace(
            read=lambda size: b"HTTP/1.1 204 OK\r\nConnection: close\r\n\r\n")))

    def test_camera_errors_are_fatal_and_release_camera(self):
        camera, events = self.fake_camera()
        def fail(self):
            raise RuntimeError("camera timeout")
        camera.capture = fail
        result = {"frames": 0, "stream": "FAIL"}
        wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("ip",))
        with patch.object(self.script, "StreamCamera", camera), self.assertRaisesRegex(RuntimeError, "camera timeout"):
            self.script.stream_camera(wlan, "ip", result)
        self.assertEqual(events[-1], "close")
        self.assertEqual(result["stream_stats"]["upload_errors"], 0)

    def test_ide_stop_during_recovery_releases_camera(self):
        camera, events = self.fake_camera()
        self.script.time.sleep_ms = lambda ms: (_ for _ in ()).throw(KeyboardInterrupt())
        result = {"frames": 0, "stream": "FAIL"}
        wlan = SimpleNamespace(isconnected=lambda: False)
        with patch.object(self.script, "StreamCamera", camera), \
             patch.object(self.script, "connect_stream_wlan", side_effect=OSError(100)), \
             self.assertRaises(KeyboardInterrupt):
            self.script.stream_camera(wlan, None, result)
        self.assertEqual(events[-1], "close")

    def test_reassociation_clears_stale_lease_renews_dhcp_and_selects_wifi_route(self):
        events = []
        class LeaseWLAN(FakeWLAN):
            def ifconfig(self, value=None):
                if isinstance(value, tuple):
                    self.ip = value[0]
                    events.append("clear")
                    return True
                if value == "dhcp":
                    self.ip = "192.168.10.99"
                    events.append("dhcp")
                    return True
                return super().ifconfig()
            def connect(self, *args):
                events.append("join")
                self.asserted_ip = self.ip
                return super().connect(*args)
        wlan = LeaseWLAN()
        def route(device):
            self.assertIs(device, wlan)
            events.append("route")
            return True
        network = SimpleNamespace(set_default_dev=route, get_default_dev=lambda: "wlan0")
        with patch.dict(sys.modules, network=network):
            ip = self.script.connect_stream_wlan(wlan)
        self.assertEqual(ip, "192.168.10.99")
        self.assertEqual(wlan.asserted_ip, "0.0.0.0")
        self.assertEqual(events, ["clear", "join", "dhcp", "route"])

    def test_tls_handshake_is_nonblocking_and_first_write_has_a_deadline(self):
        calls = []
        class RawSocket:
            def settimeout(self, timeout): calls.append(("timeout", timeout))
            def bind(self, address): pass
            def connect(self, address): calls.append(("connect", address))
            def setblocking(self, value): calls.append(("blocking", value))
            def close(self): pass
        raw = RawSocket()
        tls = SimpleNamespace(setblocking=lambda value: calls.append(("tls-blocking", value)),
                              write=lambda data: None, close=lambda: None)
        def wrap(sock, **options):
            self.assertIs(sock, raw)
            self.assertEqual(calls[-1], ("blocking", False))
            self.assertEqual(options, {"server_hostname": "backend.test", "do_handshake": False})
            return tls
        module = SimpleNamespace(AF_INET=2, SOCK_STREAM=1, socket=lambda *args: raw,
                                 getaddrinfo=lambda *args: [(2, 1, 0, "", ("192.0.2.9", 443))])
        self.script.STREAM_IO_TIMEOUT_MS = 10
        with patch.dict(sys.modules, socket=module, ssl=SimpleNamespace(wrap_socket=wrap)):
            client = self.script.open_upload("local-ip", self.script.parse_upload_url("https://backend.test/api/frame"))
            with self.assertRaisesRegex(OSError, "stalled"):
                self.script.send_all(client, b"websocket handshake request")
        self.assertEqual(self.clock, 10)

    def test_public_address_hint_skips_the_hotspot_dns_but_keeps_hostname_target(self):
        self.script.UPLOAD_IPV4_HINTS = {"backend.test": ("192.0.2.7",)}
        connected = []
        client = SimpleNamespace(settimeout=lambda value: None, bind=lambda address: None,
                                 connect=lambda address: connected.append(address),
                                 setblocking=lambda value: None, close=lambda: None)
        module = SimpleNamespace(AF_INET=2, SOCK_STREAM=1, socket=lambda *args: client,
                                 getaddrinfo=lambda *args: self.fail("address hint should skip DNS"))
        target = self.script.parse_upload_url("http://backend.test:8080/api/frame")
        with patch.dict(sys.modules, socket=module):
            self.script.open_upload("local-ip", target).close()
        self.assertEqual(connected, [("192.0.2.7", 8080)])
        self.assertEqual(target[1], "backend.test")

    def test_reconnecting_reuses_successful_dns_result(self):
        lookups = []
        def lookup(*args):
            lookups.append(args)
            return [(2, 1, 0, "", ("192.0.2.8", 8080))]
        client = SimpleNamespace(settimeout=lambda value: None, bind=lambda address: None,
                                 connect=lambda address: None, setblocking=lambda value: None,
                                 close=lambda: None)
        module = SimpleNamespace(AF_INET=2, SOCK_STREAM=1, socket=lambda *args: client,
                                 getaddrinfo=lookup)
        target = self.script.parse_upload_url("http://backend.test:8080/api/frame")
        with patch.dict(sys.modules, socket=module):
            self.script.open_upload("local-ip", target).close()
            self.script.open_upload("local-ip", target).close()
        self.assertEqual(len(lookups), 1)

    def test_failed_route_switches_but_bad_token_is_not_hidden_by_fallback(self):
        self.script.UPLOAD_TRANSPORT = "websocket"
        self.script.UPLOAD_FALLBACK_URLS = ("http://backup.test/api/frame",)
        self.script.STREAM_SECONDS = 1
        self.script.STREAM_RETRY_BASE_MS = self.script.STREAM_RETRY_MAX_MS = 10
        for error in (OSError(110), self.script.UploadConfigurationError(404),
                      self.script.UploadConfigurationError(401)):
            with self.subTest(error=error):
                self.clock = 0
                camera, events = self.fake_camera()
                hosts = []
                class Client:
                    sent = 0
                    acknowledged = 0
                    def wait_capacity(self, drain=False): return 0
                    def send_jpeg(self, data): self.sent += 1
                    def poll(self):
                        count = self.sent - self.acknowledged
                        self.acknowledged = self.sent
                        return count
                    def close(self): pass
                def connect(ip, target):
                    hosts.append(target[1])
                    if len(hosts) == 1:
                        raise error
                    return Client()
                result = {"frames": 0, "stream": "FAIL"}
                wlan = SimpleNamespace(isconnected=lambda: True, ifconfig=lambda: ("ip",))
                with patch.object(self.script, "StreamCamera", camera), \
                     patch.object(self.script.JpegWebSocket, "open", side_effect=connect):
                    if getattr(error, "status", None) == 401:
                        with self.assertRaisesRegex(RuntimeError, "401"):
                            self.script.stream_camera(wlan, "ip", result)
                        self.assertEqual(hosts, ["192.168.10.2"])
                    else:
                        self.script.stream_camera(wlan, "ip", result)
                        self.assertEqual(hosts, ["192.168.10.2", "backup.test"])
                        self.assertGreater(result["frames"], 0)
                        self.assertEqual(result["stream"], "PASS")

    def test_backend_dns_tries_next_ipv4_and_closes_failed_socket(self):
        events = []
        addresses = [(2, 1, 0, "", ("192.0.2.1", 8080)), (2, 1, 0, "", ("192.0.2.2", 8080))]
        class Socket:
            def settimeout(self, value): pass
            def bind(self, address): events.append(("bind", address))
            def connect(self, address):
                events.append(("connect", address))
                if address[0] == "192.0.2.1":
                    raise OSError(113)
            def setblocking(self, value): events.append(("blocking", value))
            def close(self): events.append(("close",))
        module = SimpleNamespace(AF_INET=2, SOCK_STREAM=1, socket=lambda *args: Socket(),
                                 getaddrinfo=lambda *args: addresses)
        with patch.dict(sys.modules, socket=module):
            client = self.script.open_upload("local-ip", self.script.parse_upload_url(self.script.UPLOAD_URL))
            client.close()
        self.assertEqual([event for event in events if event[0] == "connect"],
                         [("connect", ("192.0.2.1", 8080)), ("connect", ("192.0.2.2", 8080))])
        self.assertEqual(events.count(("close",)), 2)

    def test_initial_network_failure_keeps_stream_waiting_until_test_deadline(self):
        camera, events = self.fake_camera()
        self.script.STREAM_SECONDS = 1
        result = {"frames": 0, "stream": "FAIL"}
        wlan = SimpleNamespace(isconnected=lambda: False)
        with patch.object(self.script, "StreamCamera", camera), \
             patch.object(self.script, "connect_stream_wlan", side_effect=OSError(100)):
            self.script.stream_camera(wlan, None, result)
        self.assertNotIn("open", events)
        self.assertEqual(result["frames"], 0)
        self.assertEqual(result["stream"], "INCONCLUSIVE")
        self.assertIn("RECOVER", self.output.getvalue())

    def test_forced_recovery_stops_temporary_ap_on_success_failure_and_interrupt(self):
        self.os.urandom = lambda count: b"\xab" * count
        ap_events = []
        credentials = []
        ap = SimpleNamespace(active=lambda value: True,
                             config=lambda **kwargs: credentials.append(kwargs) or ap_events.append("start") or True,
                             stop=lambda: ap_events.append("stop") or True)
        module = SimpleNamespace(AP_IF=1, WLAN_SDIO=2, WLAN=lambda *args: ap)
        for outcome in (True, False, KeyboardInterrupt()):
            with self.subTest(outcome=outcome):
                ap_events.clear()
                wlan = FakeWLAN()
                def connect(*args, **kwargs):
                    if isinstance(outcome, BaseException):
                        raise outcome
                    wlan.connected = outcome
                    return outcome
                wlan.connect = connect
                with patch.dict(sys.modules, network=module):
                    if outcome is True:
                        self.assertEqual(self.script.connect_stream_wlan(wlan, force=True), wlan.ip)
                    else:
                        with self.assertRaises(KeyboardInterrupt if isinstance(outcome, BaseException) else OSError):
                            self.script.connect_stream_wlan(wlan, force=True)
                self.assertEqual(ap_events, ["start", "stop"])
                self.assertEqual(len(credentials[-1]["key"]), 32)
                self.assertNotIn(credentials[-1]["key"], self.output.getvalue())

    def test_recovery_mode_start_failure_still_stops_ap(self):
        self.os.urandom = lambda count: b"a" * count
        stopped = []
        ap = SimpleNamespace(active=lambda value: True, config=lambda **kwargs: False,
                             stop=lambda: stopped.append(True))
        module = SimpleNamespace(AP_IF=1, WLAN_SDIO=2, WLAN=lambda *args: ap)
        with patch.dict(sys.modules, network=module), self.assertRaises(OSError):
            self.script.prepare_radio_recovery()
        self.assertEqual(stopped, [True])

    def test_legacy_firmware_or_disabled_recovery_does_not_start_an_ap(self):
        with patch.dict(sys.modules, network=SimpleNamespace()):
            self.assertIsNone(self.script.prepare_radio_recovery())
        self.script.STREAM_RADIO_RECOVERY = False
        with patch.dict(sys.modules, network=SimpleNamespace(WLAN_SDIO=2)):
            self.assertIsNone(self.script.prepare_radio_recovery())

    def test_raw_zero_brightness_is_valid_and_compression_keeps_shared_buffer(self):
        buffer = bytearray(640 * 480)
        events = []
        preview = SimpleNamespace(bytearray=lambda: buffer,
                                  compressed=lambda **kwargs: events.append(kwargs) or "jpeg")
        raw_camera = SimpleNamespace(snapshot_into=lambda pixels, timeout: 0,
                                     close=lambda: events.append("closed"))
        raw = SimpleNamespace(PIXEL_FORMAT="gray8", Camera=lambda *args: raw_camera)
        image = SimpleNamespace(Image=lambda *args, **kwargs: preview, GRAYSCALE=1, ALLOC_HEAP=2)
        with patch.dict(sys.modules, _dart_raw=raw, image=image):
            camera = self.script.StreamCamera()
            camera.open()
            self.assertEqual(camera.capture(), "jpeg")
            self.assertIs(camera.pixels, buffer)
            camera.close()
        self.assertEqual(events[0], {"quality": 50, "x_scale": 0.5, "y_scale": 0.5})
        self.assertEqual(events[-1], "closed")


if __name__ == "__main__":
    unittest.main()
