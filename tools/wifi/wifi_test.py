"""CanMV MicroPython Wi-Fi diagnostic and JPEG upload to a video backend.

Turn on the board and Wi-Fi mechanical switches before powering/booting it.
Empty SSID: scan only. Fill SSID/PASSWORD to connect and stream the camera.
Set UPLOAD_URL to the backend URL; the phone watches the backend website.
SDIO0/SDIO1 and pinmux belong to firmware; Python selects the SDIO radio.
"""

import time

# ---- Edit these values, then run this file in the CanMV VS Code extension. ----
SSID = "hyperlee"                      # Empty = scan only; RTL8189 uses a 2.4 GHz AP.
PASSWORD = "20060519Ff"                  # Empty = open AP. Never printed by this script.
WLAN_DEVICE = "sdio"            # "sdio" or "auto"; SDIO1 is selected in firmware.
SCAN_ATTEMPTS = 3              # Retry empty scans; stop as soon as an AP is found.
SCAN_RETRY_DELAY_MS = 2000      # Wait between empty scan results.
CONNECT_TIMEOUT_MS = 20000      # IP polling limit; driver calls have own timeouts.
CONNECT_ATTEMPTS = 3            # Retry explicit driver rejection, not password exceptions.
CONNECT_RETRY_DELAY_MS = 2000    # Settle after disconnect before another association.
MONITOR_SECONDS = 5             # Check link/IP once per second after connection.
DISCONNECT_ON_EXIT = False      # Keep a successful connection for other scripts.
REUSE_CONNECTION = True         # Reuse verified same-SSID IPv4; False = fresh association test.

# Optional LAN check: run `python3 -m http.server 8000 --bind 0.0.0.0` on a PC.
# Fill its LAN IPv4 address here. This tests a TCP handshake, not Internet access.
TCP_TEST_HOST = ""
TCP_TEST_PORT = 8000
TCP_TIMEOUT_S = 3


# Camera -> JPEG uploads -> PC backend -> H.264/HLS -> phone browser.
# Run video_server.py on the PC; use a public tunnel when on different networks.
STREAM_ENABLED = True          # False = original Wi-Fi diagnostic only.
STREAM_MODE = "phone_relay"     # Phone forwards LAN JPEGs; "direct" uploads from the board.
PHONE_RELAY_PORT = 8081         # Open the printed HTTP address on the hotspot phone.
UPLOAD_URL = "https://seek-med-delayed-enhancement.trycloudflare.com/api/frame"
UPLOAD_FALLBACK_URLS = ()      # Add only currently active routes to this same backend.
# Optional hostname-specific address hints; TLS still uses the original SNI.
UPLOAD_IPV4_HINTS = {}         # Leave empty for current hotspot DNS; optional host -> IPv4 tuple.
_UPLOAD_ADDRESS_CACHE = {}
UPLOAD_TOKEN = "ecTRTGRI0DnqzTSf7SkBfQgY4fTIMzbX"  # Never logged; clear before sharing.
TLS_CA_FILE = ""               # Optional board PEM CA file; empty = TLS without CA verification.
UPLOAD_TRANSPORT = "websocket" # Continuous JPEGs with asynchronous receipt acknowledgments.
STREAM_MAX_IN_FLIGHT = 16      # Bound network backlog; never queue captured images.
UPLOAD_CONNECT_TIMEOUT_S = 15
STREAM_WIDTH = 320             # JPEG output; sensor capture stays at 640x480.
STREAM_HEIGHT = 240
STREAM_FPS = 15                 # Match video_server.py --fps; actual upload rate may be lower.
JPEG_QUALITY = 50              # 1..100; lower values use less uplink bandwidth.
CAMERA_BACKEND = "raw"         # Dart V1 SC038HGS requires the RAW tuning interpreter.
RAW_EXPOSURE_LINES = 400       # Only for _dart_raw; Sensor uses firmware exposure.
STREAM_SECONDS = 0             # 0 = until stopped from IDE.
STREAM_IO_TIMEOUT_MS = 10000   # Per-stall limit; stream recovers after connection failures.
STREAM_RETRY_BASE_MS = 1000     # Exponential backoff; failed JPEGs are dropped.
STREAM_RETRY_MAX_MS = 15000     # No retry-count limit in streaming mode; IDE stop still works.
STREAM_RADIO_RECOVERY = True   # Dart SDIO: force driver reinit after repeated failures.
MAX_RESPONSE_BYTES = 4096
SEND_CHUNK_BYTES = 1024


def log(stage, result, message=""):
    print("[wifi] %s %s%s" % (stage, result, (" " + message) if message else ""))


def text(value):
    if isinstance(value, (bytes, bytearray)):
        try:
            return value.decode("utf-8")
        except UnicodeError:
            return repr(value)
    return str(value)


def mac_text(value):
    if isinstance(value, (bytes, bytearray)):
        return ":".join("%02x" % byte for byte in value)
    return str(value)


def scan_fields(item):
    # CanMV returns WLAN info objects. Standard MicroPython returns 6-tuples.
    if hasattr(item, "ssid"):
        return (text(item.ssid), mac_text(item.bssid), item.channel,
                int(item.rssi), item.security)
    return (text(item[0]), mac_text(item[1]), item[2], int(item[3]), item[4])


def ipv4_ready(config):
    return bool(config and config[0] not in (None, "", "0.0.0.0", "255.255.255.255"))


def check_settings():
    if WLAN_DEVICE not in ("sdio", "auto"):
        raise ValueError("WLAN_DEVICE must be sdio or auto")
    if not isinstance(SSID, str) or len(SSID.encode("utf-8")) > 32:
        raise ValueError("SSID must contain at most 32 UTF-8 bytes")
    if not isinstance(PASSWORD, str):
        raise ValueError("PASSWORD must be a string")
    password_length = len(PASSWORD.encode("utf-8"))
    if SSID and password_length and not 8 <= password_length <= 64:
        raise ValueError("password must contain 8..64 bytes, or be empty for an open AP")
    if not isinstance(SCAN_ATTEMPTS, int) or not 1 <= SCAN_ATTEMPTS <= 10:
        raise ValueError("SCAN_ATTEMPTS must be an integer in 1..10")
    if not isinstance(SCAN_RETRY_DELAY_MS, int) or not 0 <= SCAN_RETRY_DELAY_MS <= 10000:
        raise ValueError("SCAN_RETRY_DELAY_MS must be an integer in 0..10000")
    if not isinstance(CONNECT_ATTEMPTS, int) or not 1 <= CONNECT_ATTEMPTS <= 5:
        raise ValueError("CONNECT_ATTEMPTS must be an integer in 1..5")
    if not isinstance(CONNECT_RETRY_DELAY_MS, int) or not 0 <= CONNECT_RETRY_DELAY_MS <= 10000:
        raise ValueError("CONNECT_RETRY_DELAY_MS must be an integer in 0..10000")
    if CONNECT_TIMEOUT_MS <= 0 or MONITOR_SECONDS < 0 or TCP_TIMEOUT_S <= 0:
        raise ValueError("timeouts must be positive and MONITOR_SECONDS nonnegative")
    if TCP_TEST_HOST:
        parts = TCP_TEST_HOST.split(".")
        if (len(parts) != 4 or
                any(not part.isdigit() or not 0 <= int(part) <= 255 for part in parts)):
            raise ValueError("TCP_TEST_HOST must be a numeric LAN IPv4 address")
        if not 1 <= TCP_TEST_PORT <= 65535:
            raise ValueError("TCP_TEST_PORT must be 1..65535")
        if not SSID:
            raise ValueError("fill SSID before enabling the TCP test")

    if not isinstance(REUSE_CONNECTION, bool):
        raise ValueError("REUSE_CONNECTION must be True or False")
    if not isinstance(STREAM_ENABLED, bool):
        raise ValueError("STREAM_ENABLED must be True or False")
    if not isinstance(STREAM_RADIO_RECOVERY, bool):
        raise ValueError("STREAM_RADIO_RECOVERY must be True or False")
    if STREAM_MODE not in ("direct", "phone_relay"):
        raise ValueError("STREAM_MODE must be direct or phone_relay")
    if not isinstance(PHONE_RELAY_PORT, int) or not 1 <= PHONE_RELAY_PORT <= 65535:
        raise ValueError("PHONE_RELAY_PORT must be an integer in 1..65535")
    if STREAM_ENABLED and UPLOAD_URL:
        parse_upload_url(UPLOAD_URL)
        if not isinstance(UPLOAD_FALLBACK_URLS, (tuple, list)) or len(UPLOAD_FALLBACK_URLS) > 4:
            raise ValueError("UPLOAD_FALLBACK_URLS must contain at most four URLs")
        for url in UPLOAD_FALLBACK_URLS:
            parse_upload_url(url)
        for host, hints in UPLOAD_IPV4_HINTS.items():
            if not isinstance(host, str) or not isinstance(hints, (tuple, list)):
                raise ValueError("UPLOAD_IPV4_HINTS must map hostnames to IPv4 lists")
            for address in hints:
                parts = address.split(".")
                if len(parts) != 4 or any(not part.isdigit() or not 0 <= int(part) <= 255 for part in parts):
                    raise ValueError("invalid upload IPv4 hint")
        if UPLOAD_TRANSPORT not in ("http", "websocket"):
            raise ValueError("UPLOAD_TRANSPORT must be http or websocket")
        if not UPLOAD_TOKEN or any(c in UPLOAD_TOKEN for c in "\r\n"):
            raise ValueError("fill UPLOAD_TOKEN with the token printed by video_server.py")
        if UPLOAD_CONNECT_TIMEOUT_S <= 0:
            raise ValueError("UPLOAD_CONNECT_TIMEOUT_S must be positive")
        for name, value, lower, upper in (
                ("STREAM_WIDTH", STREAM_WIDTH, 32, 640),
                ("STREAM_HEIGHT", STREAM_HEIGHT, 2, 480),
                ("STREAM_FPS", STREAM_FPS, 1, 30),
                ("STREAM_MAX_IN_FLIGHT", STREAM_MAX_IN_FLIGHT, 1, 64),
                ("JPEG_QUALITY", JPEG_QUALITY, 1, 100),
                ("STREAM_SECONDS", STREAM_SECONDS, 0, 86400),
                ("STREAM_IO_TIMEOUT_MS", STREAM_IO_TIMEOUT_MS, 1, 60000),
                ("STREAM_RETRY_BASE_MS", STREAM_RETRY_BASE_MS, 1, 60000),
                ("STREAM_RETRY_MAX_MS", STREAM_RETRY_MAX_MS, 1, 60000)):
            if not isinstance(value, int) or not lower <= value <= upper:
                raise ValueError("%s must be an integer in %d..%d" % (name, lower, upper))
        if STREAM_RETRY_MAX_MS < STREAM_RETRY_BASE_MS:
            raise ValueError("STREAM_RETRY_MAX_MS must be >= STREAM_RETRY_BASE_MS")
        if STREAM_WIDTH % 8 or STREAM_HEIGHT % 2:
            raise ValueError("STREAM_WIDTH must be a multiple of 8; STREAM_HEIGHT must be even")
        if CAMERA_BACKEND not in ("auto", "raw", "sensor"):
            raise ValueError("CAMERA_BACKEND must be auto, raw or sensor")
        if not isinstance(RAW_EXPOSURE_LINES, int) or not (
                RAW_EXPOSURE_LINES == 0 or 4 <= RAW_EXPOSURE_LINES <= 65509):
            raise ValueError("RAW_EXPOSURE_LINES must be 0 or 4..65509")


def new_wlan(network):
    if not hasattr(network, "WLAN"):
        raise RuntimeError("network.WLAN unavailable; firmware was built without the WLAN API. "
                           "Rebuild firmware with CONFIG_ENABLE_NETWORK_RT_WLAN=y "
                           "and the RTL8189 SDIO1 kernel driver")
    role = getattr(network, "STA_IF", 0)
    constant = "WLAN_SDIO" if WLAN_DEVICE == "sdio" else "WLAN_AUTO"
    if hasattr(network, constant):
        wlan = network.WLAN(role, getattr(network, constant))
        log("device", "INFO", "transport=" + WLAN_DEVICE + "; SDIO host is set by firmware")
    else:
        # Older CanMV firmware only exposes the board's default WLAN interface.
        wlan = network.WLAN(role)
        log("device", "INFO", "legacy API: using firmware default radio")
    return wlan



def scan_networks(wlan):
    for attempt in range(1, SCAN_ATTEMPTS + 1):
        log("scan", "START", "attempt=%d/%d" % (attempt, SCAN_ATTEMPTS))
        started = time.ticks_ms()
        raw = wlan.scan()
        elapsed = time.ticks_diff(time.ticks_ms(), started)
        if raw is None:
            raise RuntimeError("driver returned no scan result")
        networks = [scan_fields(item) for item in raw]
        networks.sort(key=lambda item: item[3], reverse=True)
        # An empty list alone cannot distinguish no APs from a HAL timeout.
        log("scan", "PASS" if networks else "INCONCLUSIVE",
            "attempt=%d/%d elapsed_ms=%d count=%d" %
            (attempt, SCAN_ATTEMPTS, elapsed, len(networks)))
        if networks:
            return networks
        if attempt < SCAN_ATTEMPTS:
            log("scan", "RETRY", "empty result; wait_ms=%d" % SCAN_RETRY_DELAY_MS)
            time.sleep_ms(SCAN_RETRY_DELAY_MS)
    log("scan", "INFO", "no APs reported; verify a nearby visible 2.4 GHz hotspot. "
        "If needed, switch board/Wi-Fi power off and on, then rerun")
    return []

def wait_for_ip(wlan):
    started = time.ticks_ms()
    last_report = -1000
    while True:
        connected = wlan.isconnected()
        config = wlan.ifconfig() if connected else None
        if connected and ipv4_ready(config):
            return config
        elapsed = time.ticks_diff(time.ticks_ms(), started)
        if elapsed >= CONNECT_TIMEOUT_MS:
            raise RuntimeError("link/IP timeout; check 2.4 GHz SSID, password and DHCP")
        if elapsed - last_report >= 1000:
            log("connect", "WAIT", "link=%s elapsed_ms=%d" % (connected, elapsed))
            last_report = elapsed
        time.sleep_ms(200)


def reusable_connection(wlan):
    if not REUSE_CONNECTION or not wlan.isconnected():
        return False
    try:
        ap = wlan.status("ap")
        return text(ap.ssid) == SSID and ipv4_ready(wlan.ifconfig())
    except (AttributeError, OSError, ValueError):
        # Unknown SSID or missing introspection is insufficient evidence to reuse.
        return False


def wait_for_disconnect(wlan):
    started = time.ticks_ms()
    while wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), started) >= CONNECT_TIMEOUT_MS:
            raise RuntimeError("previous association did not disconnect")
        time.sleep_ms(100)


def connect_with_retries(wlan):
    for attempt in range(1, CONNECT_ATTEMPTS + 1):
        log("connect", "TRY", "attempt=%d/%d" % (attempt, CONNECT_ATTEMPTS))
        started = time.ticks_ms()
        # CanMV requires omitting the password argument for an open network.
        # Driver exceptions remain errors; only an explicit False is retried.
        accepted = wlan.connect(SSID, PASSWORD) if PASSWORD else wlan.connect(SSID)
        elapsed = time.ticks_diff(time.ticks_ms(), started)
        if accepted is not False:
            log("connect", "ACCEPTED", "attempt=%d/%d elapsed_ms=%d; waiting for IPv4" %
                (attempt, CONNECT_ATTEMPTS, elapsed))
            return
        log("connect", "REJECTED", "attempt=%d/%d elapsed_ms=%d; firmware supplied no reason code" %
            (attempt, CONNECT_ATTEMPTS, elapsed))
        if attempt == CONNECT_ATTEMPTS:
            raise RuntimeError("driver rejected connection after %d attempt(s); "
                               "check hotspot client limit/MAC blocking, credentials and radio state. "
                               "Try toggling the hotspot and fully power-cycling the board" % CONNECT_ATTEMPTS)
        # isconnected() can be False while a previous join is still pending.
        # Explicitly cancel it before retrying; do not just reissue connect().
        if wlan.disconnect() is False:
            raise RuntimeError("cannot cancel rejected association before retry")
        wait_for_disconnect(wlan)
        log("connect", "RETRY", "join cancelled; wait_ms=%d" % CONNECT_RETRY_DELAY_MS)
        time.sleep_ms(CONNECT_RETRY_DELAY_MS)


def tcp_test(ip):
    import socket

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.settimeout(TCP_TIMEOUT_S)
        # Bind to WLAN's IP so another network adapter cannot satisfy this test.
        sock.bind((ip, 0))
        sock.connect((TCP_TEST_HOST, TCP_TEST_PORT))
        log("tcp", "PASS", "%s:%d via %s" % (TCP_TEST_HOST, TCP_TEST_PORT, ip))
    finally:
        sock.close()


class StreamCamera:
    """One camera owner; keep the writable RAW image alive during capture."""

    def __init__(self):
        self.raw = None
        self.sensor = None
        self.preview = None
        self.pixels = None

    def open(self):
        import image

        raw_module = None
        if CAMERA_BACKEND != "sensor":
            try:
                import _dart_raw
                raw_module = _dart_raw
            except ImportError:
                if CAMERA_BACKEND == "raw":
                    raise RuntimeError("_dart_raw unavailable; use auto/sensor or the RAW tuning firmware")
        if raw_module is not None:
            if getattr(raw_module, "PIXEL_FORMAT", None) != "gray8":
                raise RuntimeError("_dart_raw requires the gray8 tuning firmware")
            self.preview = image.Image(640, 480, image.GRAYSCALE, alloc=image.ALLOC_HEAP)
            self.pixels = self.preview.bytearray()
            self.raw = raw_module.Camera(RAW_EXPOSURE_LINES, 50, 0)
            log("camera", "PASS", "backend=raw 640x480 grayscale; exposure_lines=%d" %
                RAW_EXPOSURE_LINES)
        else:
            from media.sensor import Sensor

            # SC038HGS has a VGA mode; Sensor's FHD probe defaults cannot find it.
            self.sensor = Sensor(width=640, height=480, fps=0)
            self.sensor.reset()
            self.sensor.set_framesize(width=640, height=480)
            self.sensor.set_pixformat(Sensor.GRAYSCALE)
            self.sensor.run()
            log("camera", "PASS", "backend=sensor 640x480 grayscale via ISP")

    def capture(self):
        try:
            if self.raw is not None:
                # None is a timeout; zero bright pixels is a valid frame.
                if self.raw.snapshot_into(self.pixels, 1000) is None:
                    raise RuntimeError("RAW camera frame timeout")
                frame = self.preview
            else:
                frame = self.sensor.snapshot(timeout=1000, auto_reboot=False)
            # compressed() copies: compress() would corrupt the shared RAW image.
            return frame.compressed(quality=JPEG_QUALITY, x_scale=STREAM_WIDTH / 640,
                                    y_scale=STREAM_HEIGHT / 480)
        except Exception as error:
            # Camera errors must fail the test, not look like browser disconnects.
            raise RuntimeError("camera capture/encoding failed: %s" % error)

    def close(self):
        try:
            if self.raw is not None:
                self.raw.close()
        finally:
            if self.sensor is not None:
                self.sensor.stop()


def socket_errno(error):
    return getattr(error, "errno", None) or (error.args[0] if error.args else None)


def send_all(client, data):
    # Firmware blocking sends have short timeouts. Handle TCP backpressure and
    # short writes explicitly without copying a complete JPEG for each chunk.
    import os

    view = memoryview(data)
    offset = 0
    progress = time.ticks_ms()
    write = getattr(client, "write", None) or client.send
    while offset < len(view):
        os.exitpoint()
        try:
            sent = write(view[offset:offset + SEND_CHUNK_BYTES])
            if sent == 0:
                raise OSError(104, "client closed")
            if sent:
                offset += sent
                progress = time.ticks_ms()
                continue
        except OSError as error:
            if socket_errno(error) not in (11, 110, 115, -26880, -26752):
                raise
        if time.ticks_diff(time.ticks_ms(), progress) >= STREAM_IO_TIMEOUT_MS:
            raise OSError(110, "client send stalled")
        time.sleep_ms(2)


def parse_upload_url(url):
    if any(c in url for c in "\r\n #") or "://" not in url:
        raise ValueError("UPLOAD_URL must be an http(s) URL without spaces or fragments")
    scheme, rest = url.split("://", 1)
    authority, sep, path = rest.partition("/")
    if scheme not in ("http", "https") or not authority or "@" in authority:
        raise ValueError("UPLOAD_URL must be an http(s) URL without user/password")
    if ":" in authority:
        host, port = authority.rsplit(":", 1)
        if not port.isdigit() or not 1 <= int(port) <= 65535:
            raise ValueError("invalid UPLOAD_URL port")
        port = int(port)
    else:
        host, port = authority, 443 if scheme == "https" else 80
    if not host or any(c in host for c in ":[]?\\"):
        raise ValueError("UPLOAD_URL requires a DNS hostname or IPv4 address")
    return scheme, host, port, authority, ("/" + path if sep and path else "/api/frame")


def open_upload(ip, target):
    import socket

    scheme, host, port, _, _ = target
    cache_key = (host, port)
    addresses = _UPLOAD_ADDRESS_CACHE.get(cache_key)
    if addresses is None:
        hints = UPLOAD_IPV4_HINTS.get(host, ())
        if hints:
            addresses = [(socket.AF_INET, socket.SOCK_STREAM, 0, "", (address, port)) for address in hints]
        else:
            addresses = socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_STREAM)
        if addresses:
            _UPLOAD_ADDRESS_CACHE[cache_key] = addresses
    if not addresses:
        raise OSError(113, "backend DNS returned no IPv4 address")
    last_error = None
    for address in addresses:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(UPLOAD_CONNECT_TIMEOUT_S)
            sock.bind((ip, 0))
            sock.connect(address[-1])
            try:
                # Native RT-Smart/lwIP values; this firmware omits the Python
                # constants. Keep bounded TLS writes as separate TCP packets.
                sock.setsockopt(getattr(socket, "IPPROTO_TCP", 6),
                                getattr(socket, "TCP_NODELAY", 1), 1)
            except (AttributeError, OSError):
                pass
            if scheme == "https":
                import ssl

                # Blocking mbedTLS handshakes can loop forever on WANT_READ
                # after the NIC's short receive timeout. Let the first bounded
                # nonblocking send drive the handshake instead.
                sock.setblocking(False)
                options = {"server_hostname": host, "do_handshake": False}
                if TLS_CA_FILE:
                    with open(TLS_CA_FILE, "rb") as ca:
                        options["cadata"] = ca.read()
                    options["cert_reqs"] = ssl.CERT_REQUIRED
                sock = ssl.wrap_socket(sock, **options)
            sock.setblocking(False)
            return sock
        except OSError as error:
            sock.close()
            last_error = error
        except BaseException:
            sock.close()
            raise
    _UPLOAD_ADDRESS_CACHE.pop(cache_key, None)
    raise last_error


def read_upload_response(client):
    import os

    response = bytearray()
    started = time.ticks_ms()
    read = getattr(client, "read", None) or client.recv
    while len(response) < MAX_RESPONSE_BYTES:
        os.exitpoint()
        try:
            chunk = read(min(256, MAX_RESPONSE_BYTES - len(response)))
            if chunk:
                response.extend(chunk)
                if b"\r\n\r\n" in response:
                    line = bytes(response).split(b"\r\n", 1)[0].split()
                    if len(line) < 2 or not line[1].isdigit():
                        raise RuntimeError("invalid backend HTTP status")
                    status = int(line[1])
                    if status in (408, 425, 429) or 500 <= status <= 599:
                        raise OSError(16, "backend temporarily unavailable: HTTP %d" % status)
                    if status != 204:
                        raise RuntimeError("backend rejected JPEG: HTTP %d; check upload URL/token" % status)
                    return b"connection: close" not in bytes(response).lower()
            # Some CanMV sockets return b"" for would-block; bound EOF waits too.
        except OSError as error:
            if socket_errno(error) not in (11, 110, -26880, -26752):
                raise
        if time.ticks_diff(time.ticks_ms(), started) >= STREAM_IO_TIMEOUT_MS:
            raise OSError(110, "backend response timeout")
        time.sleep_ms(10)
    raise RuntimeError("backend HTTP response headers too large")




class UploadConfigurationError(RuntimeError):
    def __init__(self, status):
        self.status = status
        super().__init__("WebSocket rejected: HTTP %d; check backend URL/token" % status)


class JpegWebSocket:
    """RFC 6455 binary JPEGs; the server acknowledges cumulative receipt counts.

    A bounded flight window hides Internet round-trip latency without holding
    old camera frames in an application queue. Only acknowledged frames count
    as uploaded. No third-party MicroPython package is required.
    """

    def __init__(self, client):
        self.client = client
        self.buffer = bytearray()
        self.sent = 0
        self.acknowledged = 0
        self.progress = time.ticks_ms()
        self.poller = None
        try:
            import select
            self.poller = select.poll()
            self.poller.register(client, select.POLLIN)
        except (ImportError, AttributeError, TypeError, ValueError):
            self.poller = None

    @classmethod
    def open(cls, ip, target):
        import binascii
        import hashlib
        import os

        client = open_upload(ip, target)
        connection = cls(client)
        try:
            key = binascii.b2a_base64(os.urandom(16)).strip()
            accept = binascii.b2a_base64(hashlib.sha1(
                key + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest()).strip()
            path = target[4].rstrip("/") + "/ws"
            header = ("GET %s HTTP/1.1\r\nHost: %s\r\n"
                      "Authorization: Bearer %s\r\nUpgrade: websocket\r\n"
                      "Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
                      "Sec-WebSocket-Key: %s\r\n\r\n") % (
                          path, target[3], UPLOAD_TOKEN, key.decode())
            send_all(client, header.encode())
            started = time.ticks_ms()
            while b"\r\n\r\n" not in connection.buffer:
                os.exitpoint()
                connection.receive()
                if len(connection.buffer) > MAX_RESPONSE_BYTES:
                    raise RuntimeError("WebSocket response headers too large")
                if time.ticks_diff(time.ticks_ms(), started) >= STREAM_IO_TIMEOUT_MS:
                    raise OSError(110, "WebSocket handshake timeout")
                time.sleep_ms(2)
            headers, extra = bytes(connection.buffer).split(b"\r\n\r\n", 1)
            lines = headers.split(b"\r\n")
            status = lines[0].split()
            if len(status) < 2 or not status[1].isdigit():
                raise RuntimeError("invalid WebSocket HTTP status")
            code = int(status[1])
            if code in (408, 425, 429) or 500 <= code <= 599:
                raise OSError(16, "backend temporarily unavailable: HTTP %d body=%r" % (code, extra[:160]))
            if code != 101:
                raise UploadConfigurationError(code)
            fields = {}
            for line in lines[1:]:
                name, _, value = line.partition(b":")
                fields[name.lower()] = value.strip()
            if (fields.get(b"sec-websocket-accept") != accept or
                    fields.get(b"upgrade", b"").lower() != b"websocket" or
                    b"upgrade" not in [part.strip() for part in fields.get(b"connection", b"").lower().split(b",")]):
                raise RuntimeError("invalid WebSocket upgrade response")
            connection.buffer = bytearray(extra)
            connection.progress = time.ticks_ms()
            return connection
        except BaseException:
            client.close()
            raise

    def receive(self):
        if self.poller is not None and not self.poller.poll(0):
            return False
        read = getattr(self.client, "read", None) or self.client.recv
        try:
            chunk = read(1024)
            if chunk:
                self.buffer.extend(chunk)
                return True
            # Some firmware sockets return b"" as would-block. The pending ACK
            # timeout also bounds this ambiguous EOF case.
        except OSError as error:
            if socket_errno(error) not in (11, 110, -26880, -26752):
                raise
        return False

    def send_frame(self, opcode, data):
        import os

        length = len(data)
        if length < 126:
            header = bytes((0x80 | opcode, 0x80 | length))
        elif length <= 65535:
            header = bytes((0x80 | opcode, 0xfe)) + length.to_bytes(2, "big")
        else:
            header = bytes((0x80 | opcode, 0xff)) + length.to_bytes(8, "big")
        mask = os.urandom(4)
        # Combine each small JPEG into one TLS write, avoiding a TCP delay
        # between its WebSocket header and payload. Larger JPEGs stay bounded.
        view = memoryview(data)
        offset = 0
        while offset < length or (not length and offset == 0):
            count = min(SEND_CHUNK_BYTES - len(header) - 4 if offset == 0 else SEND_CHUNK_BYTES, length - offset)
            block = bytearray(count)
            for index in range(count):
                block[index] = view[offset + index] ^ mask[(offset + index) & 3]
            if offset == 0:
                send_all(self.client, header + mask + block)
            else:
                send_all(self.client, block)
            offset += count
            if not length:
                break

    def poll(self):
        previous = self.acknowledged
        # Drain immediately available ACKs, but cap work so a peer cannot starve
        # camera capture. Server frames are tiny and are never fragmented.
        for _ in range(32):
            while len(self.buffer) >= 2:
                first, second = self.buffer[0], self.buffer[1]
                size = second & 0x7f
                if first & 0xf0 != 0x80 or second & 0x80 or size > 125:
                    raise RuntimeError("invalid WebSocket acknowledgment frame")
                if len(self.buffer) < size + 2:
                    break
                payload = bytes(self.buffer[2:2 + size])
                self.buffer = self.buffer[2 + size:]
                opcode = first & 0x0f
                if opcode == 2 and size == 4:
                    count = int.from_bytes(payload, "big")
                    if not self.acknowledged <= count <= self.sent:
                        raise RuntimeError("invalid WebSocket acknowledgment count")
                    if count > self.acknowledged:
                        self.acknowledged = count
                        self.progress = time.ticks_ms()
                elif opcode == 9:
                    self.send_frame(10, payload)
                elif opcode == 10:
                    pass
                elif opcode == 8:
                    code = int.from_bytes(payload[:2], "big") if size >= 2 else 1000
                    if code in (1002, 1003, 1007, 1008, 1009):
                        raise RuntimeError("backend rejected JPEG stream: WebSocket %d" % code)
                    raise OSError(16, "backend closed JPEG stream: WebSocket %d" % code)
                else:
                    raise RuntimeError("unexpected WebSocket acknowledgment opcode")
            if not self.receive():
                break
        if self.sent > self.acknowledged and time.ticks_diff(
                time.ticks_ms(), self.progress) >= STREAM_IO_TIMEOUT_MS:
            raise OSError(110, "WebSocket receipt acknowledgment stalled")
        return self.acknowledged - previous

    def wait_capacity(self, drain=False):
        import os

        count = self.poll()
        limit = 1 if drain else STREAM_MAX_IN_FLIGHT
        while self.sent - self.acknowledged >= limit:
            os.exitpoint()
            time.sleep_ms(2)
            count += self.poll()
        return count

    def send_jpeg(self, data):
        if self.sent == self.acknowledged:
            self.progress = time.ticks_ms()
        self.send_frame(2, data)
        self.sent += 1

    def close(self):
        self.client.close()


def prepare_radio_recovery():
    """Dart driver restarts STA hardware when joining from active AP mode."""
    import network
    import os

    if not STREAM_RADIO_RECOVERY or WLAN_DEVICE != "sdio" or not hasattr(network, "WLAN_SDIO"):
        return None
    ap = network.WLAN(network.AP_IF, network.WLAN_SDIO)
    try:
        ap.active(True)
        # This AP exists only until the following station join restarts the
        # vendor radio. Give it a fresh password; never write it to logs/disk.
        key = "".join("%02x" % byte for byte in os.urandom(16))
        if ap.config(ssid="CanMV-recover", key=key) is False:
            raise OSError(100, "cannot start radio recovery mode")
        log("radio", "RECOVER", "reinitializing SDIO radio before station join")
        return ap
    except BaseException:
        ap.stop()
        raise


def select_stream_route(wlan):
    try:
        import network
    except ImportError:
        return
    setter = getattr(network, "set_default_dev", None)
    if setter is None:
        return
    if setter(wlan) is False:
        raise OSError(113, "cannot select Wi-Fi as the default uplink")
    getter = getattr(network, "get_default_dev", None)
    if getter is not None:
        log("route", "PASS", "uplink=" + text(getter()))


def connect_stream_wlan(wlan, force=False):
    """One controlled association attempt; the stream owns retry/backoff."""
    if not force and reusable_connection(wlan):
        select_stream_route(wlan)
        return wlan.ifconfig()[0]
    # Avoid racing a firmware reconnect worker with explicit association.
    try:
        wlan.config(auto_reconnect=False)
    except (AttributeError, TypeError, OSError, ValueError):
        pass
    if wlan.disconnect() is False:
        raise OSError(100, "cannot cancel previous Wi-Fi association")
    try:
        wait_for_disconnect(wlan)
    except RuntimeError:
        raise OSError(110, "previous Wi-Fi association did not stop")
    # A radio restart can leave the previous IPv4 lease visible. Clear it
    # before joining, then explicitly restart DHCP after association.
    renew_dhcp = False
    try:
        if wlan.ifconfig(("0.0.0.0", "0.0.0.0", "0.0.0.0", "0.0.0.0")) is False:
            raise OSError(100, "cannot clear previous IPv4 lease")
        renew_dhcp = True
    except (TypeError, AttributeError):
        pass
    time.sleep_ms(500)
    raw = wlan.scan()
    if raw is None:
        raise OSError(100, "Wi-Fi scan returned no result")
    matches = [item for item in raw if hasattr(item, "ssid") and text(item.ssid) == SSID]
    # Preserve native CanMV scan metadata rather than asking the driver to scan
    # a second time after the diagnostic scan cache has already been cleared.
    recovery_ap = prepare_radio_recovery() if force else None
    try:
        if matches:
            ap = max(matches, key=lambda item: item.rssi)
            log("connect", "TARGET", "bssid=%s channel=%s rssi=%s" %
                (mac_text(ap.bssid), ap.channel, ap.rssi))
            try:
                accepted = wlan.connect(info=ap, key=PASSWORD) if PASSWORD else wlan.connect(info=ap)
            except TypeError:
                accepted = wlan.connect(SSID, PASSWORD) if PASSWORD else wlan.connect(SSID)
        else:
            accepted = wlan.connect(SSID, PASSWORD) if PASSWORD else wlan.connect(SSID)
    finally:
        if recovery_ap is not None:
            recovery_ap.stop()
    if accepted is False:
        wlan.disconnect()
        raise OSError(100, "Wi-Fi driver rejected association")
    if renew_dhcp and wlan.ifconfig("dhcp") is False:
        raise OSError(100, "cannot restart Wi-Fi DHCP")
    try:
        config = wait_for_ip(wlan)
    except RuntimeError:
        wlan.disconnect()
        raise OSError(110, "Wi-Fi IPv4 timeout")
    select_stream_route(wlan)
    log("connect", "PASS", "ip=%s netmask=%s gateway=%s dns=%s" % tuple(config))
    return config[0]


def stream_pause(milliseconds, started):
    import os

    # Small sleep intervals keep IDE stop and finite test runs responsive.
    while milliseconds > 0:
        os.exitpoint()
        if STREAM_SECONDS and time.ticks_diff(time.ticks_ms(), started) >= STREAM_SECONDS * 1000:
            return
        step = min(200, milliseconds)
        time.sleep_ms(step)
        milliseconds -= step


def safe_error(error):
    message = str(error)
    for secret in (PASSWORD, UPLOAD_TOKEN):
        if secret:
            message = message.replace(secret, "<redacted>")
    return message


def stream_camera(wlan, ip, result):
    import gc
    import os

    camera = StreamCamera()
    camera_open = False
    client = None
    jpeg_bytes = 0
    targets = [parse_upload_url(url) for url in (UPLOAD_URL,) + tuple(UPLOAD_FALLBACK_URLS)]
    target_index = 0
    target = targets[0]
    rejected_targets = set()
    rate_started = time.ticks_ms()
    rate_frames = result["frames"]
    started = time.ticks_ms()
    report = started
    report_frames = result["frames"]
    last_sent = None
    failures = 0
    stalls_since_recovery = 0
    wifi_needed = ip is None
    radio_needed = False
    if ip is not None:
        result["ip"] = ip
    stats = {"upload_errors": 0, "wifi_attempts": 0, "dropped_frames": 0,
             "max_gap_ms": 0, "startup_ms": None, "elapsed_ms": 0,
             "sent_frames": 0, "unconfirmed_frames": 0,
             "captured_frames": 0, "capture_ms": 0, "send_ms": 0, "ack_wait_ms": 0}
    result["stream_stats"] = stats

    def received(count):
        nonlocal last_sent, failures, report, report_frames, stalls_since_recovery
        if not count:
            return
        now = time.ticks_ms()
        if last_sent is not None:
            stats["max_gap_ms"] = max(stats["max_gap_ms"], time.ticks_diff(now, last_sent))
        else:
            stats["startup_ms"] = time.ticks_diff(now, started)
        last_sent = now
        failures = 0
        result["frames"] += count
        result["stream"] = "PASS"
        elapsed = time.ticks_diff(now, report)
        if elapsed >= 5000:
            if (result["frames"] - report_frames) * 1000 / elapsed >= STREAM_FPS / 2:
                stalls_since_recovery = 0
            log("stream", "PASS", "uploaded_frames=%d fps=%.1f max_gap_ms=%d upload_errors=%d wifi_attempts=%d" % (
                result["frames"], (result["frames"] - report_frames) * 1000 / elapsed,
                stats["max_gap_ms"], stats["upload_errors"], stats["wifi_attempts"]))
            count = max(1, stats["captured_frames"])
            log("stream", "TIMING", "capture_avg_ms=%d send_avg_ms=%d ack_wait_ms=%d" % (
                stats["capture_ms"] // count, stats["send_ms"] // count, stats["ack_wait_ms"]))
            report, report_frames = now, result["frames"]

    def close_client():
        nonlocal client
        if client is not None:
            if UPLOAD_TRANSPORT == "websocket":
                stats["unconfirmed_frames"] += client.sent - client.acknowledged
            client.close()
            client = None

    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        result["stream"] = "WAITING"
        log("stream", "START", "JPEG upload -> backend H.264/HLS; %dx%d at <=%d FPS; automatic recovery" %
            (STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS))
        log("stream", "INFO", "transport=%s max_in_flight=%d" % (UPLOAD_TRANSPORT, STREAM_MAX_IN_FLIGHT))
        while not STREAM_SECONDS or time.ticks_diff(time.ticks_ms(), started) < STREAM_SECONDS * 1000:
            os.exitpoint()
            frame_started = time.ticks_ms()
            jpeg = None
            pixels = None
            try:
                connected = wlan.isconnected()
                config = wlan.ifconfig() if connected else None
                if not ipv4_ready(config) or config[0] != ip:
                    wifi_needed = True
                if wifi_needed:
                    close_client()
                    stats["wifi_attempts"] += 1
                    ip = connect_stream_wlan(wlan, force=radio_needed or failures >= 3)
                    radio_needed = False
                    stalls_since_recovery = 0
                    result["ip"] = ip
                    wifi_needed = False
                if not camera_open:
                    try:
                        camera.open()
                    except OSError as error:
                        raise RuntimeError("camera initialization failed: %s" % error)
                    camera_open = True
                    # On Dart, a fresh radio initialization after RAW camera
                    # startup restored full throughput in the on-board test.
                    # Do this once, before opening the upload connection.
                    if STREAM_RADIO_RECOVERY and WLAN_DEVICE == "sdio" and getattr(camera, "raw", None) is not None:
                        wifi_needed = True
                        radio_needed = True
                        log("radio", "START", "initialize SDIO link after RAW camera startup")
                        continue
                # WebSocket connects/waits before capture so backpressure never
                # creates a queue of old camera frames.
                if UPLOAD_TRANSPORT == "websocket":
                    wait_started = time.ticks_ms()
                    if client is None:
                        select_stream_route(wlan)
                        log("stream", "CONNECT", "host=" + target[1])
                        client = JpegWebSocket.open(ip, target)
                        rate_started, rate_frames = time.ticks_ms(), result["frames"]
                    received(client.wait_capacity())
                    stats["ack_wait_ms"] += time.ticks_diff(time.ticks_ms(), wait_started)
                capture_started = time.ticks_ms()
                jpeg = camera.capture()
                pixels = jpeg.bytearray()
                stats["capture_ms"] += time.ticks_diff(time.ticks_ms(), capture_started)
                stats["captured_frames"] += 1
                send_started = time.ticks_ms()
                if UPLOAD_TRANSPORT == "websocket":
                    client.send_jpeg(pixels)
                    stats["sent_frames"] += 1
                    jpeg_bytes += len(pixels)
                    received(client.poll())
                else:
                    if client is None:
                        client = open_upload(ip, target)
                    header = ("POST %s HTTP/1.1\r\nHost: %s\r\n"
                              "Authorization: Bearer %s\r\nContent-Type: image/jpeg\r\n"
                              "Content-Length: %d\r\nConnection: keep-alive\r\n\r\n") % (
                                  target[4], target[3], UPLOAD_TOKEN, len(pixels))
                    send_all(client, header.encode())
                    send_all(client, pixels)
                    stats["sent_frames"] += 1
                    keep_alive = read_upload_response(client)
                    if keep_alive is False:
                        close_client()
                    jpeg_bytes += len(pixels)
                    received(1)
                stats["send_ms"] += time.ticks_diff(time.ticks_ms(), send_started)
                rate_elapsed = time.ticks_diff(time.ticks_ms(), rate_started)
                if UPLOAD_TRANSPORT == "websocket" and len(targets) > 1 and rate_elapsed >= 15000:
                    rate = (result["frames"] - rate_frames) * 1000 / rate_elapsed
                    if rate < STREAM_FPS / 4:
                        raise OSError(110, "upload rate stayed below one quarter of target; retry route")
                    rate_started, rate_frames = time.ticks_ms(), result["frames"]
            except UploadConfigurationError as error:
                if error.status not in (404, 410):
                    raise
                close_client()
                rejected_targets.add(target_index)
                if len(rejected_targets) == len(targets):
                    raise
                while target_index in rejected_targets:
                    target_index = (target_index + 1) % len(targets)
                target = targets[target_index]
                result["stream"] = "RECOVERING"
                log("stream", "RECOVER", "expired/missing route disabled; next_host=" + target[1])
                continue
            except OSError as error:
                close_client()
                if socket_errno(error) != 16:
                    _UPLOAD_ADDRESS_CACHE.pop((target[1], target[2]), None)
                if len(rejected_targets) < len(targets):
                    target_index = (target_index + 1) % len(targets)
                    while target_index in rejected_targets:
                        target_index = (target_index + 1) % len(targets)
                    target = targets[target_index]
                failures += 1
                stats["upload_errors"] += 1
                if jpeg is not None:
                    stats["dropped_frames"] += 1
                result["stream"] = "RECOVERING"
                # Several TCP failures with a stale-but-present IP can require
                # a new association/DHCP lease. Never reset a healthy link for
                # a single server timeout.
                if socket_errno(error) != 16:
                    stalls_since_recovery += 1
                    if stalls_since_recovery >= 3:
                        wifi_needed = True
                        radio_needed = True
                delay = min(STREAM_RETRY_MAX_MS, STREAM_RETRY_BASE_MS * (2 ** min(failures - 1, 8)))
                log("stream", "RECOVER", "failures=%d wait_ms=%d next_host=%s error=%s" % (failures, delay, target[1], safe_error(error)))
                pixels = None
                jpeg = None
                gc.collect()
                stream_pause(delay, started)
                continue
            finally:
                pixels = None
                jpeg = None
            gc.collect()
            delay = 1000 // STREAM_FPS - time.ticks_diff(time.ticks_ms(), frame_started)
            if delay > 0:
                stream_pause(delay, started)
        if UPLOAD_TRANSPORT == "websocket" and client is not None:
            try:
                received(client.wait_capacity(drain=True))
            except OSError as error:
                stats["upload_errors"] += 1
                result["stream"] = "INCONCLUSIVE"
                log("stream", "WARN", "final receipt acknowledgment failed: " + safe_error(error))
        if result["stream"] != "PASS":
            result["stream"] = "INCONCLUSIVE"
    finally:
        stats["elapsed_ms"] = time.ticks_diff(time.ticks_ms(), started)
        try:
            close_client()
        finally:
            try:
                camera.close()
            finally:
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
                time.sleep_ms(100)
                log("stream", "INFO", "uploaded_frames=%d jpeg_bytes=%d elapsed_ms=%d max_gap_ms=%d upload_errors=%d" % (
                    result["frames"], jpeg_bytes, stats["elapsed_ms"], stats["max_gap_ms"], stats["upload_errors"]))

def main():
    wlan = None
    stage = "settings"
    outcome = "FAIL"
    connection_started = False
    completed = False
    result = {"scan_count": None, "ip": None, "tcp": "SKIP", "stream": "SKIP", "frames": 0}
    try:
        check_settings()
        log("start", "INFO", "mode=" + ("connect" if SSID else "scan"))
        stage = "device"
        import network

        wlan = new_wlan(network)
        if wlan.active(True) is False or not wlan.active():
            raise RuntimeError("WLAN inactive; check Wi-Fi switch, driver and SDIO1 firmware config")
        log(stage, "PASS", "WLAN active")
        # These diagnostics are optional on older firmware.
        for name, operation in (("mac", lambda: mac_text(wlan.config("mac"))),
                                ("netdevs", lambda: network.get_dev_list())):
            try:
                log(name, "INFO", str(operation()))
            except (AttributeError, OSError, ValueError):
                log(name, "SKIP", "not available on this firmware")

        if SSID and STREAM_ENABLED and UPLOAD_URL:
            # Association scanning belongs to the recovery loop. In particular,
            # do not scan/disconnect an already verified link before streaming.
            stage = "stream"
            connection_started = not reusable_connection(wlan)
            ip = wlan.ifconfig()[0] if not connection_started else None
            if STREAM_MODE == "phone_relay":
                from phone_relay import stream_via_phone
                stream_via_phone(wlan, ip, result, globals())
            else:
                stream_camera(wlan, ip, result)
            outcome = "PASS" if result["stream"] == "PASS" else "INCONCLUSIVE"
            completed = True
            return result

        stage = "scan"
        networks = scan_networks(wlan)
        result["scan_count"] = len(networks)
        for index, item in enumerate(networks):
            ssid, bssid, channel, rssi, security = item
            log("ap", "INFO", "%d ssid=%r bssid=%s channel=%s rssi=%d security=%s" %
                (index + 1, ssid, bssid, channel, rssi, security))
        if not SSID:
            outcome = "PASS" if networks else "INCONCLUSIVE"
            log("connect", "SKIP", "SSID is empty; fill SSID/PASSWORD to test link and IP")
            completed = True
            return result

        stage = "connect"
        log(stage, "START", "ssid=%r" % SSID)
        if reusable_connection(wlan):
            log("connect", "REUSE", "same SSID and valid IPv4; association/password not retested")
        else:
            connection_started = True
            # A different/unknown association must not satisfy this request.
            if wlan.isconnected():
                if wlan.disconnect() is False:
                    raise RuntimeError("cannot disconnect the previous association")
                wait_for_disconnect(wlan)
            connect_with_retries(wlan)
        stage = "ip"
        config = wait_for_ip(wlan)
        result["ip"] = config[0]
        log("connect", "PASS")
        log(stage, "PASS", "ip=%s netmask=%s gateway=%s dns=%s" % tuple(config))
        try:
            log("rssi", "INFO", str(wlan.status("rssi")))
        except (AttributeError, OSError, ValueError):
            pass

        stage = "monitor"
        for second in range(MONITOR_SECONDS):
            time.sleep_ms(1000)
            if not wlan.isconnected() or not ipv4_ready(wlan.ifconfig()):
                raise RuntimeError("link/IP lost during monitoring")
            log(stage, "PASS", "%d/%d seconds" % (second + 1, MONITOR_SECONDS))

        stage = "tcp"
        if TCP_TEST_HOST:
            result["tcp"] = "FAIL"
            tcp_test(wlan.ifconfig()[0])
            result["tcp"] = "PASS"
        else:
            log(stage, "SKIP", "TCP_TEST_HOST is empty; no LAN/Internet reachability claim")
        log("stream", "SKIP", "fill UPLOAD_URL/UPLOAD_TOKEN to upload JPEGs" if STREAM_ENABLED else "STREAM_ENABLED=False")
        outcome = "PASS"
        completed = True
    except KeyboardInterrupt:
        outcome = "STOPPED"
        if stage == "stream":
            result["stream"] = "STOPPED"
        log(stage, "STOPPED", "interrupted from IDE")
    except Exception as error:
        # A driver error may include its arguments; redact password defensively.
        message = str(error)
        for secret in (PASSWORD, UPLOAD_TOKEN):
            if secret:
                message = message.replace(secret, "<redacted>")
        if stage == "stream":
            result["stream"] = "FAIL"
        log(stage, "FAIL", message)
    finally:
        # Cancel incomplete joins. A successful link stays up unless requested.
        if wlan is not None and ((connection_started and not completed) or
                                 (DISCONNECT_ON_EXIT and (connection_started or result["ip"] is not None))):
            try:
                if wlan.disconnect() is False:
                    raise RuntimeError("driver rejected disconnect")
                log("disconnect", "PASS")
            except Exception:
                log("disconnect", "WARN", "driver cleanup failed")
                if completed:
                    outcome = "FAIL"
        result["result"] = outcome
        log("summary", outcome, "scan_count=%s ip=%s tcp=%s stream=%s frames=%d" %
            (result["scan_count"], result["ip"], result["tcp"], result["stream"], result["frames"]))
    return result


if __name__ == "__main__":
    main()
