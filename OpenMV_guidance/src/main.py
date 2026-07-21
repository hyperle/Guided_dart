import sensor
import time
import ustruct
from pyb import UART, USB_VCP
from camera_config import (
    TARGET_EXPOSURE_SWITCH_FLAG,
    SD_RECORD_FLAG,
    SD_RECORD_SEGMENT_DURATION_MS,
    SD_RECORD_MAX_SEGMENTS,
    SD_RECORD_SYNC_INTERVAL_MS,
    SD_RECORD_JPEG_QUALITY,
    SD_RECORD_MIN_FREE_BYTES,
)

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
MEASUREMENT_IMAGE_SIZE_STARTUP_FRAMES = 30
MANUAL_EXPOSURE_US = 250
SEARCH_EXPOSURE_US = 1000
LOCKED_EXPOSURE_US = 300
LOCK_EXPOSURE_AREA_THRESHOLD = 500
LOCKED_EXPOSURE_MAX_MISSED_FRAMES = 50
EXPOSURE_SETTLE_FRAMES = 3
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


def should_send_measurement_image_size(frame_index):
    return frame_index < MEASUREMENT_IMAGE_SIZE_STARTUP_FRAMES


def main():
    global recorder

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

    target_exposure_switch_enabled = TARGET_EXPOSURE_SWITCH_FLAG
    initial_exposure_us = SEARCH_EXPOSURE_US if target_exposure_switch_enabled else MANUAL_EXPOSURE_US

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing((SENSOR_WIDTH, SENSOR_HEIGHT))
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=initial_exposure_us)
    sensor.skip_frames(time=500)
    sensor.set_transpose(True)
    sensor.set_hmirror(True)

    uart = UART(UART_PORT, UART_BAUDRATE, timeout_char=1000)
    usb_vcp = USB_VCP()
    clock = time.clock()
    detector = GreenLightDetector()
    detector_params = detector.params
    detector_default_max_missed_frames = detector_params["max_missed_frames"]
    set_detector_full_scan_fallback = detector.set_full_scan_fallback_enabled
    recorder = make_sd_recorder()
    sensor_snapshot = sensor.snapshot
    process_frame = detector.process_frame
    send_measurement = detector.send_measurement
    set_led = set_led_state
    set_fixed_exposure_api = sensor.set_auto_exposure
    no_target_coordinate = 0xFFFF
    startup_image_size_frames = MEASUREMENT_IMAGE_SIZE_STARTUP_FRAMES
    cached_image_width = 0
    cached_image_height = 0
    search_exposure_us = SEARCH_EXPOSURE_US
    locked_exposure_us = LOCKED_EXPOSURE_US
    lock_area_threshold = LOCK_EXPOSURE_AREA_THRESHOLD
    locked_max_missed_frames = LOCKED_EXPOSURE_MAX_MISSED_FRAMES
    exposure_settle_frame_count = EXPOSURE_SETTLE_FRAMES
    usb_debug_active = False
    usb_preview_requested = False
    last_usb_debug_poll_ms = 0
    next_usb_preview_frame_ms = time.ticks_ms()
    usb_preview_min_interval_ms = max(1, 1000 // USB_PREVIEW_FPS_LIMIT)
    exposure_locked = False
    exposure_missed_frames = 0
    exposure_settle_frames = 0
    frame_index = 0

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
        img = sensor_snapshot()
        if cached_image_width == 0:
            cached_image_width = img.width()
            cached_image_height = img.height()

        result = process_frame(img)
        include_image_size = frame_index < startup_image_size_frames

        if result is None:
            if target_exposure_switch_enabled:
                if exposure_settle_frames > 0:
                    exposure_settle_frames -= 1
                elif exposure_locked:
                    exposure_missed_frames += 1
                    if exposure_missed_frames > locked_max_missed_frames:
                        set_detector_full_scan_fallback(True)
                        detector_params["max_missed_frames"] = detector_default_max_missed_frames
                        set_fixed_exposure_api(False, exposure_us=search_exposure_us)
                        exposure_locked = False
                        exposure_missed_frames = 0
                        exposure_settle_frames = exposure_settle_frame_count
            set_led(target_detected=False)
            send_measurement(
                uart,
                no_target_coordinate,
                no_target_coordinate,
                0,
                cached_image_width,
                cached_image_height,
                include_image_size=include_image_size,
            )
        else:
            area = result["area"]
            if target_exposure_switch_enabled:
                if exposure_settle_frames > 0:
                    exposure_settle_frames -= 1
                elif exposure_locked:
                    exposure_missed_frames = 0
                elif area > lock_area_threshold:
                    set_fixed_exposure_api(False, exposure_us=locked_exposure_us)
                    set_detector_full_scan_fallback(False)
                    detector_params["max_missed_frames"] = locked_max_missed_frames
                    exposure_locked = True
                    exposure_missed_frames = 0
                    exposure_settle_frames = exposure_settle_frame_count
            set_led(target_detected=True)
            send_measurement(
                uart,
                result["center_x"],
                result["center_y"],
                area,
                cached_image_width,
                cached_image_height,
                include_image_size=include_image_size,
            )

        frame_metadata = None
        if recorder is not None or usb_debug_active:
            frame_metadata = make_frame_metadata(detector, result, cached_image_width, cached_image_height, now_ms, frame_index)
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
        if recorder is not None:
            recorder.close(quiet=True)
    except Exception:
        pass
    set_led_state(error=True)
    while True:
        time.sleep(1)
