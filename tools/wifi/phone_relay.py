"""CanMV local JPEG WebSocket server; the phone relays to the video backend."""


def stream_via_phone(wlan, ip, result, cfg):
    import binascii
    import gc
    import hashlib
    import json
    import os
    import socket
    import time

    log, send_all = cfg["log"], cfg["send_all"]
    camera = cfg["StreamCamera"]()
    listener = client = None
    camera_open = False
    started = time.ticks_ms()
    heartbeat = started
    failures = 0
    source = cfg["parse_upload_url"](cfg["UPLOAD_URL"])
    relay_url = source[0].replace("http", "ws") + "://" + source[3] + "/api/relay/ws"
    page_config = {"upload": relay_url, "viewer": source[0] + "://" + source[3] + "/", "token": cfg["UPLOAD_TOKEN"]}
    with open("/sdcard/phone_relay.html") as page:
        html = page.read().replace("__CANMV_CONFIG__", json.dumps(page_config).replace("<", "\\u003c")).encode()
    stats = {"sent_frames": 0, "wifi_attempts": 0, "upload_errors": 0}
    result["stream_stats"] = stats
    os.exitpoint(os.EXITPOINT_ENABLE)

    def read_available(connection, buffer):
        try:
            chunk = connection.recv(1024)
            if chunk == b"":
                raise OSError(104, "phone disconnected")
            if chunk:
                buffer.extend(chunk)
        except OSError as error:
            if cfg["socket_errno"](error) not in (11, 110):
                raise

    def send_frame(connection, opcode, payload):
        size = len(payload)
        if size < 126:
            header = bytes((0x80 | opcode, size))
        elif size <= 65535:
            header = bytes((0x80 | opcode, 126)) + size.to_bytes(2, "big")
        else:
            header = bytes((0x80 | opcode, 127)) + size.to_bytes(8, "big")
        # Send the header with payload bytes to avoid a tiny standalone TCP write.
        prefix = min(len(payload), cfg["SEND_CHUNK_BYTES"] - len(header))
        send_all(connection, header + payload[:prefix])
        if prefix < len(payload):
            send_all(connection, memoryview(payload)[prefix:])

    def serve(connection):
        buffer = bytearray()
        deadline = time.ticks_ms()
        while b"\r\n\r\n" not in buffer:
            os.exitpoint()
            read_available(connection, buffer)
            if len(buffer) > 4096 or time.ticks_diff(time.ticks_ms(), deadline) > 5000:
                raise OSError(110, "phone HTTP request timeout")
            time.sleep_ms(2)
        raw, remaining = bytes(buffer).split(b"\r\n\r\n", 1)
        lines = raw.split(b"\r\n")
        request = lines[0].split()
        if len(request) != 3 or request[0] != b"GET":
            return
        if request[1] != b"/camera":
            body = html if request[1] == b"/" else b"Not found"
            status = "200 OK" if request[1] == b"/" else "404 Not Found"
            header = ("HTTP/1.1 %s\r\nContent-Type: text/html; charset=utf-8\r\n"
                      "Content-Length: %d\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n") % (status, len(body))
            send_all(connection, header.encode())
            send_all(connection, body)
            return
        fields = {}
        for line in lines[1:]:
            name, _, value = line.partition(b":")
            fields[name.lower()] = value.strip()
        key = fields.get(b"sec-websocket-key", b"")
        if fields.get(b"upgrade", b"").lower() != b"websocket" or len(key) != 24:
            return
        accept = binascii.b2a_base64(hashlib.sha1(key + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest()).strip()
        send_all(connection, b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " + accept + b"\r\n\r\n")
        log("relay", "CONNECTED", "phone joined; JPEGs -> phone -> backend H.264")
        pending = bytearray(remaining)
        sent = received = confirmed = 0
        progress = report = time.ticks_ms()
        report_frames = result["frames"]
        while not cfg["STREAM_SECONDS"] or time.ticks_diff(time.ticks_ms(), started) < cfg["STREAM_SECONDS"] * 1000:
            os.exitpoint()
            tick = time.ticks_ms()
            if not wlan.isconnected():
                raise OSError(100, "Wi-Fi link lost")
            read_available(connection, pending)
            if len(pending) > 4096:
                raise OSError(104, "phone control buffer too large")
            while len(pending) >= 2:
                first, second = pending[0], pending[1]
                size = second & 127
                if first & 0xf0 != 0x80 or not second & 0x80 or size > 125:
                    raise OSError(104, "invalid phone WebSocket control frame")
                if len(pending) < size + 6:
                    break
                mask = pending[2:6]
                payload = bytearray(pending[6:size + 6])
                pending = pending[size + 6:]
                for index in range(size):
                    payload[index] ^= mask[index & 3]
                opcode = first & 15
                if opcode == 8:
                    log("relay", "DISCONNECTED", "phone WebSocket closed")
                    return
                if opcode == 9:
                    send_frame(connection, 10, payload)
                elif opcode == 2 and size == 8:
                    seen = int.from_bytes(payload[:4], "big")
                    accepted = int.from_bytes(payload[4:], "big")
                    if not received <= seen <= sent or not confirmed <= accepted <= seen:
                        raise OSError(104, "invalid phone acknowledgment")
                    if seen > received:
                        received = seen
                        progress = time.ticks_ms()
                    if accepted > confirmed:
                        result["frames"] += accepted - confirmed
                        confirmed = accepted
                        result["stream"] = "PASS"
                elif opcode != 10:
                    raise OSError(104, "unexpected phone control message")
            if time.ticks_diff(time.ticks_ms(), report) >= 5000:
                now = time.ticks_ms()
                log("relay", "PASS" if confirmed else "WAIT", "uploaded_frames=%d fps=%.1f local_sent=%d phone_received=%d" % (
                    result["frames"], (result["frames"] - report_frames) * 1000 / time.ticks_diff(now, report), sent, received))
                report, report_frames = now, result["frames"]
            if sent - received >= 4:
                if time.ticks_diff(time.ticks_ms(), progress) >= cfg["STREAM_IO_TIMEOUT_MS"]:
                    raise OSError(110, "phone not receiving local camera frames")
                time.sleep_ms(2)
                continue
            jpeg = camera.capture()
            send_frame(connection, 2, jpeg.bytearray())
            jpeg = None
            sent += 1
            stats["sent_frames"] += 1
            gc.collect()
            delay = 1000 // cfg["STREAM_FPS"] - time.ticks_diff(time.ticks_ms(), tick)
            if delay > 0:
                time.sleep_ms(delay)

    try:
        result["stream"] = "WAITING"
        while not cfg["STREAM_SECONDS"] or time.ticks_diff(time.ticks_ms(), started) < cfg["STREAM_SECONDS"] * 1000:
            os.exitpoint()
            try:
                if listener is None or not wlan.isconnected() or wlan.ifconfig()[0] != ip:
                    if listener is not None:
                        listener.close()
                        listener = None
                    stats["wifi_attempts"] += 1
                    ip = cfg["connect_stream_wlan"](wlan, force=failures >= 3)
                    result["ip"] = ip
                    if not camera_open:
                        camera.open()
                        camera_open = True
                    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    listener.bind((ip, cfg["PHONE_RELAY_PORT"]))
                    listener.listen(2)
                    listener.setblocking(False)
                    log("relay", "READY", "open on phone: http://%s:%d/ ; keep page in foreground" % (ip, cfg["PHONE_RELAY_PORT"]))
                    failures = 0
                try:
                    client, address = listener.accept()
                except OSError as error:
                    if cfg["socket_errno"](error) in (11, 110):
                        if time.ticks_diff(time.ticks_ms(), heartbeat) >= 15000:
                            heartbeat = time.ticks_ms()
                            log("relay", "WAIT", "no phone camera connection; open http://%s:%d/" % (ip, cfg["PHONE_RELAY_PORT"]))
                        time.sleep_ms(20)
                        continue
                    raise
                client.setblocking(False)
                try:
                    client.setsockopt(6, 1, 1)  # Native IPPROTO_TCP/TCP_NODELAY.
                except OSError:
                    pass
                try:
                    serve(client)
                finally:
                    client.close()
                    client = None
            except OSError as error:
                failures += 1
                stats["upload_errors"] += 1
                log("relay", "RECOVER", cfg["safe_error"](error))
                if not wlan.isconnected() and listener is not None:
                    listener.close()
                    listener = None
                time.sleep_ms(200)
    finally:
        if client is not None:
            client.close()
        if listener is not None:
            listener.close()
        camera.close()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
