import time
from pyb import USB_VCP

_SEND_INTERVAL_MS = 500
_PING_TIMEOUT_MS = 3000


class DebugStreamer:
    def __init__(self):
        self._last_send = 0
        self._vcp = USB_VCP()
        self._last_ping = 0

    def poll(self):
        if self._vcp.any():
            self._vcp.read(min(self._vcp.any(), 64))
            self._last_ping = time.ticks_ms()

    def send_frame(self, img):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_ping) > _PING_TIMEOUT_MS:
            return
        if time.ticks_diff(now, self._last_send) < _SEND_INTERVAL_MS:
            return
        self._last_send = now
        jpeg = img.compress(quality=10)
        if jpeg is None:
            return
        import ubinascii
        b64 = ubinascii.b2a_base64(jpeg).decode('ascii').rstrip()
        print('FRAME ' + b64)
