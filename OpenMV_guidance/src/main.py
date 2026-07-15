import sensor
import time
import ustruct
from pyb import UART, USB_VCP
from camera_config import (
    SD_RECORD_FLAG,
    SD_RECORD_SEGMENT_DURATION_MS,
    SD_RECORD_MAX_SEGMENTS,
    SD_RECORD_SYNC_INTERVAL_MS,
    SD_RECORD_JPEG_QUALITY,
    SD_RECORD_MIN_FREE_BYTES,
)

try:
    import _thread
except Exception:
    _thread = None

try:
    from pyb import LED
    _has_led = True
except (ImportError, AttributeError):
    LED = None
    _has_led = False

if _has_led:
    red_led = LED(1)
    green_led = LED(2)
    red_led.off()
    green_led.off()
else:
    red_led = None
    green_led = None


def set_led_state(error=False, target_detected=False):
    if red_led is None or green_led is None:
        return

    if error:
        green_led.off()
        red_led.on()
    elif target_detected:
        red_led.off()
        green_led.on()
    else:
        red_led.off()
        green_led.off()


def blink_startup_indicator():
    if red_led is None or green_led is None:
        return

    red_led.off()
    green_led.on()
    time.sleep_ms(500)
    green_led.off()


time.sleep(3)


SENSOR_WIDTH = 320
SENSOR_HEIGHT = 240
UART_PORT = 1
UART_BAUDRATE = 115200
MEASUREMENT_HEADER = 0x5A
MEASUREMENT_LEGACY_LENGTH = 0x06
MEASUREMENT_EXTENDED_LENGTH = 0x0A
MEASUREMENT_LEGACY_FRAME_LENGTH = MEASUREMENT_LEGACY_LENGTH + 3
MEASUREMENT_EXTENDED_FRAME_LENGTH = MEASUREMENT_EXTENDED_LENGTH + 3
MEASUREMENT_NO_TARGET_COORDINATE = 0xFFFF
MEASUREMENT_IMAGE_SIZE_STARTUP_FRAMES = 8
MEASUREMENT_IMAGE_SIZE_REFRESH_MS = 0
MEASUREMENT_ASYNC_UART_ENABLED = True
MEASUREMENT_SENDER_IDLE_MS = 1
MANUAL_EXPOSURE_US = 1000
USB_DEBUG_POLL_INTERVAL_MS = 250
USB_PREVIEW_JPEG_QUALITY = 70
USB_PREVIEW_FPS_LIMIT = 12
USB_FRAME_MAGIC = b"OMVJ"
USB_FRAME_HEADER_FORMAT = ">IHHI"
USB_METADATA_MAGIC = b"OMVM"
USB_METADATA_HEADER_FORMAT = ">I"
USB_PREVIEW_START_REQUEST = b"OMVP"
USB_PREVIEW_REQUEST_READ_BYTES = 64
OVERLAY_X = 2
OVERLAY_STATUS_Y = 2
OVERLAY_TARGET_Y = 14
OVERLAY_ROI_Y = 26
COLOR_OK = (0, 255, 0)
COLOR_ERROR = (255, 0, 0)
COLOR_TEXT = (255, 255, 255)

recorder = None
measurement_sender = None


def target_search_status(result, debug):
    debug = debug or {}
    status = debug.get("target_search", "")
    if status:
        return status
    if result is None:
        return "target_not_found"
    selected_scan = debug.get("selected_scan", "")
    if selected_scan == "roi" or debug.get("target_in_search_roi", False):
        return "search_roi"
    return "full_frame_search"


def make_frame_metadata(detector, result, image_width, image_height, now_ms, frame_index):
    debug = detector.last_debug or {}
    metadata = {
        "frame_index": int(frame_index),
        "target_search": target_search_status(result, debug),
        "detected": result is not None,
        "image_width": int(image_width),
        "image_height": int(image_height),
        "ticks_ms": int(now_ms),
    }
    for key in (
        "selected_scan",
        "roi_active",
        "target_in_search_roi",
        "search_roi",
        "fallback_used",
        "fallback_reason",
        "full_target_selected",
    ):
        if key in debug:
            metadata[key] = debug[key]
    if result is not None:
        metadata["center_x"] = int(result["center_x"])
        metadata["center_y"] = int(result["center_y"])
        metadata["area"] = int(result["area"])
        metadata["radius"] = int(result.get("radius", 0))
    return metadata


def payload_length(payload):
    try:
        return len(payload)
    except TypeError:
        return payload.size()


def json_escape(value):
    text = str(value)
    text = text.replace(chr(92), chr(92) + chr(92))
    return text.replace(chr(34), chr(92) + chr(34))


def json_value(value):
    if value is None:
        return "null"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return str(value)
    if isinstance(value, tuple) or isinstance(value, list):
        parts = []
        for item in value:
            parts.append(json_value(item))
        return "[" + ",".join(parts) + "]"
    if isinstance(value, dict):
        parts = []
        for key, item in value.items():
            parts.append("\"%s\":%s" % (json_escape(key), json_value(item)))
        return "{" + ",".join(parts) + "}"
    return "\"%s\"" % json_escape(value)


def try_draw(callable_obj, *args, **kwargs):
    try:
        callable_obj(*args, **kwargs)
    except Exception:
        pass


def draw_text(img, y, text, color):
    try_draw(img.draw_string, OVERLAY_X, y, text, color=color)


def runtime_roi_text(debug):
    if not debug or not debug.get("roi_active", False):
        return "roi inactive"
    roi = debug.get("search_roi", None)
    if debug.get("fallback_used", False):
        reason = debug.get("fallback_reason", "unknown")
        if debug.get("full_target_selected", False):
            return "fallback full: %s roi=%s" % (reason, roi)
        return "fallback kept roi: %s roi=%s" % (reason, roi)
    if debug.get("target_in_search_roi", False):
        return "roi ok %s" % (roi,)
    return "target outside roi %s" % (roi,)


def draw_preview_overlay(img, result, fps, debug):
    if result is None:
        draw_text(img, OVERLAY_STATUS_Y, "pix=%dx%d fps=%0.1f" % (img.width(), img.height(), fps), COLOR_ERROR)
        draw_text(img, OVERLAY_TARGET_Y, "target x=-1 y=-1", COLOR_ERROR)
        draw_text(img, OVERLAY_ROI_Y, runtime_roi_text(debug), COLOR_ERROR)
        return

    x = int(result["center_x"])
    y = int(result["center_y"])
    radius = int(result.get("radius", 1))
    area = int(result["area"])
    try_draw(img.draw_circle, x, y, radius, color=COLOR_OK)
    try_draw(img.draw_line, x - 6, y, x + 6, y, color=COLOR_TEXT)
    try_draw(img.draw_line, x, y - 6, x, y + 6, color=COLOR_TEXT)
    draw_text(img, OVERLAY_STATUS_Y, "pix=%dx%d fps=%0.1f" % (img.width(), img.height(), fps), COLOR_OK)
    draw_text(img, OVERLAY_TARGET_Y, "target x=%d y=%d a=%d" % (x, y, area), COLOR_OK)
    draw_text(img, OVERLAY_ROI_Y, runtime_roi_text(debug), COLOR_OK)


def write_usb_preview_frame(vcp, img, quality, metadata=None):
    jpeg = img.compress(quality=quality)
    if jpeg is None:
        return False
    header = ustruct.pack(
        USB_FRAME_HEADER_FORMAT,
        payload_length(jpeg),
        img.width(),
        img.height(),
        time.ticks_ms() & 0xFFFFFFFF,
    )
    try:
        vcp.write(USB_FRAME_MAGIC)
        vcp.write(header)
        vcp.write(jpeg)
        if metadata is not None:
            payload = json_value(metadata).encode("utf-8")
            vcp.write(USB_METADATA_MAGIC)
            vcp.write(ustruct.pack(USB_METADATA_HEADER_FORMAT, payload_length(payload)))
            vcp.write(payload)
    except Exception:
        return False
    return True


def poll_usb_preview_request(vcp, current_active):
    try:
        pending = vcp.any()
    except Exception:
        return current_active
    if not pending:
        return current_active
    try:
        data = vcp.read(min(int(pending), USB_PREVIEW_REQUEST_READ_BYTES))
    except Exception:
        return current_active
    if data and USB_PREVIEW_START_REQUEST in data:
        return True
    return current_active


def should_send_measurement_image_size(now_ms, last_image_size_ms, frame_index):
    if frame_index < MEASUREMENT_IMAGE_SIZE_STARTUP_FRAMES:
        return True
    if MEASUREMENT_IMAGE_SIZE_REFRESH_MS <= 0:
        return False
    return time.ticks_diff(now_ms, last_image_size_ms) >= MEASUREMENT_IMAGE_SIZE_REFRESH_MS


def measurement_clamp_u16(value):
    value = int(value)
    if value < 0:
        return 0
    if value > 0xFFFF:
        return 0xFFFF
    return value


def measurement_put_u16_be(packet, index, value):
    value = measurement_clamp_u16(value)
    packet[index] = (value >> 8) & 0xFF
    packet[index + 1] = value & 0xFF


def measurement_sender_worker(sender):
    sender.run()


class MeasurementSender:
    def __init__(self, uart):
        self._uart = uart
        self._lock = None
        if MEASUREMENT_ASYNC_UART_ENABLED and _thread is not None:
            try:
                self._lock = _thread.allocate_lock()
            except Exception:
                self._lock = None
        self._running = False
        self._failed = False
        self._thread_start_attempted = False
        self._latest_ready = False
        self._latest_seq = 0
        self._latest_x = MEASUREMENT_NO_TARGET_COORDINATE
        self._latest_y = MEASUREMENT_NO_TARGET_COORDINATE
        self._latest_area = 0
        self._latest_image_width = 0
        self._latest_image_height = 0
        self._latest_include_image_size = False
        self._sync_short_packet = bytearray(MEASUREMENT_LEGACY_FRAME_LENGTH)
        self._sync_extended_packet = bytearray(MEASUREMENT_EXTENDED_FRAME_LENGTH)

    def start(self):
        if self._thread_start_attempted:
            return False
        self._thread_start_attempted = True
        if self._lock is None:
            return False
        self._running = True
        self._failed = False
        try:
            _thread.start_new_thread(measurement_sender_worker, (self,))
        except Exception:
            self._running = False
            self._failed = True
            return False
        return True

    def stop(self):
        self._running = False

    def publish(self, x, y, area, image_width, image_height, include_image_size):
        if self._running and not self._failed and self._lock is not None:
            self._lock.acquire()
            self._latest_x = x
            self._latest_y = y
            self._latest_area = area
            self._latest_image_width = image_width
            self._latest_image_height = image_height
            self._latest_include_image_size = include_image_size
            self._latest_seq = (self._latest_seq + 1) & 0x3FFFFFFF
            self._latest_ready = True
            self._lock.release()
            return True
        return self._send_with_buffers(
            x,
            y,
            area,
            image_width,
            image_height,
            include_image_size,
            self._sync_short_packet,
            self._sync_extended_packet,
        )

    def run(self):
        short_packet = bytearray(MEASUREMENT_LEGACY_FRAME_LENGTH)
        extended_packet = bytearray(MEASUREMENT_EXTENDED_FRAME_LENGTH)
        last_seq = -1
        failed = False

        try:
            while self._running:
                self._lock.acquire()
                ready = self._latest_ready
                seq = self._latest_seq
                if (not ready) or (seq == last_seq):
                    self._lock.release()
                    time.sleep_ms(MEASUREMENT_SENDER_IDLE_MS)
                    continue
                x = self._latest_x
                y = self._latest_y
                area = self._latest_area
                image_width = self._latest_image_width
                image_height = self._latest_image_height
                include_image_size = self._latest_include_image_size
                self._lock.release()

                if not self._send_with_buffers(
                    x,
                    y,
                    area,
                    image_width,
                    image_height,
                    include_image_size,
                    short_packet,
                    extended_packet,
                ):
                    failed = True
                    break
                last_seq = seq
        except Exception:
            failed = True

        if failed:
            self._failed = True
        self._running = False

    def _send_with_buffers(self, x, y, area, image_width, image_height, include_image_size, short_packet, extended_packet):
        if include_image_size:
            packet = extended_packet
            packet[0] = MEASUREMENT_HEADER
            packet[1] = MEASUREMENT_EXTENDED_LENGTH
            measurement_put_u16_be(packet, 2, x)
            measurement_put_u16_be(packet, 4, y)
            measurement_put_u16_be(packet, 6, area)
            measurement_put_u16_be(packet, 8, image_width)
            measurement_put_u16_be(packet, 10, image_height)
        else:
            packet = short_packet
            packet[0] = MEASUREMENT_HEADER
            packet[1] = MEASUREMENT_LEGACY_LENGTH
            measurement_put_u16_be(packet, 2, x)
            measurement_put_u16_be(packet, 4, y)
            measurement_put_u16_be(packet, 6, area)

        checksum = 0
        last_index = len(packet) - 1
        for index in range(last_index):
            checksum = (checksum + packet[index]) & 0xFF
        packet[last_index] = checksum

        try:
            written = self._uart.write(packet)
        except Exception:
            return False
        if written is not None and written != len(packet):
            return False
        return True


def main():
    global recorder, measurement_sender

    blink_startup_indicator()

    from green_light_detector import GreenLightDetector

    def usb_debug_connected():
        try:
            return usb_vcp.isconnected()
        except Exception:
            return False

    def make_sd_recorder():
        if not SD_RECORD_FLAG:
            return None
        from video_recorder import RollingMjpegRecorder
        return RollingMjpegRecorder(
            segment_duration_ms=SD_RECORD_SEGMENT_DURATION_MS,
            max_segments=SD_RECORD_MAX_SEGMENTS,
            sync_interval_ms=SD_RECORD_SYNC_INTERVAL_MS,
            jpeg_quality=SD_RECORD_JPEG_QUALITY,
            min_free_bytes=SD_RECORD_MIN_FREE_BYTES,
        )

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing((SENSOR_WIDTH, SENSOR_HEIGHT))
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=MANUAL_EXPOSURE_US)
    sensor.skip_frames(time=500)

    uart = UART(UART_PORT, UART_BAUDRATE, timeout_char=1000)
    usb_vcp = USB_VCP()
    clock = time.clock()
    detector = GreenLightDetector()
    measurement_sender = MeasurementSender(uart)
    recorder = make_sd_recorder()
    usb_debug_active = False
    usb_preview_requested = False
    last_usb_debug_poll_ms = 0
    next_usb_preview_frame_ms = time.ticks_ms()
    usb_preview_min_interval_ms = max(1, 1000 // USB_PREVIEW_FPS_LIMIT)
    frame_index = 0
    last_measurement_image_size_ms = time.ticks_ms()

    while True:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_usb_debug_poll_ms) >= USB_DEBUG_POLL_INTERVAL_MS:
            last_usb_debug_poll_ms = now_ms
            usb_preview_requested = poll_usb_preview_request(usb_vcp, usb_preview_requested)
            next_usb_debug_active = usb_debug_connected() or usb_preview_requested
            if next_usb_debug_active and not usb_debug_active:
                next_usb_preview_frame_ms = now_ms
            usb_debug_active = next_usb_debug_active

        clock.tick()
        img = sensor.snapshot()
        image_width = img.width()
        image_height = img.height()

        result = detector.process_frame(img)
        include_image_size = should_send_measurement_image_size(
            now_ms,
            last_measurement_image_size_ms,
            frame_index,
        )
        if include_image_size:
            last_measurement_image_size_ms = now_ms

        if not include_image_size:
            measurement_sender.start()

        if result is None:
            set_led_state(target_detected=False)
            measurement_x = MEASUREMENT_NO_TARGET_COORDINATE
            measurement_y = MEASUREMENT_NO_TARGET_COORDINATE
            measurement_area = 0
        else:
            set_led_state(target_detected=True)
            measurement_x = result["center_x"]
            measurement_y = result["center_y"]
            measurement_area = result["area"]

        measurement_sender.publish(
            measurement_x,
            measurement_y,
            measurement_area,
            image_width,
            image_height,
            include_image_size,
        )

        frame_metadata = None
        if recorder is not None or usb_debug_active:
            frame_metadata = make_frame_metadata(detector, result, image_width, image_height, now_ms, frame_index)
        if recorder is not None:
            recorder.add_frame(img, frame_metadata)

        if usb_debug_active and time.ticks_diff(now_ms, next_usb_preview_frame_ms) >= 0:
            next_usb_preview_frame_ms = time.ticks_add(now_ms, usb_preview_min_interval_ms)
            debug = detector.last_debug or {}
            draw_preview_overlay(img, result, clock.fps(), debug)
            if not write_usb_preview_frame(usb_vcp, img, USB_PREVIEW_JPEG_QUALITY, frame_metadata):
                usb_preview_requested = False
                usb_debug_active = False

        frame_index = (frame_index + 1) & 0x3FFFFFFF


try:
    main()
except Exception:
    try:
        if measurement_sender is not None:
            measurement_sender.stop()
    except Exception:
        pass
    try:
        if recorder is not None:
            recorder.close(quiet=True)
    except Exception:
        pass
    set_led_state(error=True)
    while True:
        time.sleep(1)
