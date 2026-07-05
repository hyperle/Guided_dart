import sensor
import time
import ustruct
from pyb import USB_VCP

from adaptive_exposure import TargetExposureController

_MAGIC = b"OMVJ"
_HEADER_FORMAT = ">IHHI"
_DEFAULT_EXPOSURE_US = 2500
_OVERLAY_X = 2
_OVERLAY_STATUS_Y = 2
_OVERLAY_TARGET_Y = 14
_COLOR_OK = (0, 255, 0)
_COLOR_ERROR = (255, 0, 0)
_COLOR_TEXT = (255, 255, 255)


def _clamp(value, min_value, max_value):
    value = int(value)
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def _sensor_framesize(name):
    name = (name or "QVGA").upper()
    if name == "QQVGA":
        return sensor.QQVGA
    if name == "VGA":
        return sensor.VGA
    return sensor.QVGA


def _sensor_pixformat(name):
    name = (name or "RGB565").upper()
    if name == "GRAYSCALE":
        return sensor.GRAYSCALE
    return sensor.RGB565


def _try_call(callable_obj, *args, **kwargs):
    try:
        callable_obj(*args, **kwargs)
    except Exception:
        pass


def configure_sensor(frame_size="QVGA", width=640, height=480, pixformat="RGB565",
                     auto_gain=False, auto_whitebal=False, auto_exposure=False,
                     exposure_us=_DEFAULT_EXPOSURE_US, adaptive_exposure=True):
    sensor.reset()
    sensor.set_pixformat(_sensor_pixformat(pixformat))
    sensor.set_framesize(_sensor_framesize(frame_size))

    width = int(width or 0)
    height = int(height or 0)
    if width > 0 and height > 0:
        _try_call(sensor.set_windowing, (width, height))

    sensor.skip_frames(time=1200)
    _try_call(sensor.set_auto_gain, bool(auto_gain))
    _try_call(sensor.set_auto_whitebal, bool(auto_whitebal))
    exposure_controller = None
    if auto_exposure:
        _try_call(sensor.set_auto_exposure, True)
    else:
        _try_call(sensor.set_auto_exposure, False, exposure_us=int(exposure_us or _DEFAULT_EXPOSURE_US))
        if adaptive_exposure and pixformat.upper() == "RGB565":
            exposure_controller = TargetExposureController(initial_exposure_us=int(exposure_us or _DEFAULT_EXPOSURE_US))
            exposure_controller.apply()
    sensor.skip_frames(time=300)
    return exposure_controller


def _payload_length(payload):
    try:
        return len(payload)
    except TypeError:
        return payload.size()


def _draw_cross(img, x, y, color):
    _try_call(img.draw_line, x - 6, y, x + 6, y, color=color)
    _try_call(img.draw_line, x, y - 6, x, y + 6, color=color)


def _draw_text(img, y, text, color):
    _try_call(img.draw_string, _OVERLAY_X, y, text, color=color)


def _draw_stream_status(img, fps, color):
    _draw_text(img, _OVERLAY_STATUS_Y, "pix=%dx%d fps=%0.1f" % (img.width(), img.height(), fps), color)


def _draw_detector_overlay(img, result, fps):
    if result is None:
        _draw_stream_status(img, fps, _COLOR_ERROR)
        _draw_text(img, _OVERLAY_TARGET_Y, "target x=-1 y=-1", _COLOR_ERROR)
        return

    x = int(result["center_x"])
    y = int(result["center_y"])
    radius = int(result["radius"])
    area = int(result["area"])
    _try_call(img.draw_circle, x, y, radius, color=_COLOR_OK)
    _draw_cross(img, x, y, _COLOR_TEXT)
    _draw_stream_status(img, fps, _COLOR_OK)
    _draw_text(img, _OVERLAY_TARGET_Y, "target x=%d y=%d a=%d" % (x, y, area), _COLOR_OK)


def _write_frame(vcp, img, quality):
    width = img.width()
    height = img.height()
    jpeg = img.compress(quality=quality)
    if jpeg is None:
        return False
    header = ustruct.pack(
        _HEADER_FORMAT,
        _payload_length(jpeg),
        width,
        height,
        time.ticks_ms() & 0xFFFFFFFF,
    )
    vcp.write(_MAGIC)
    vcp.write(header)
    vcp.write(jpeg)
    return True


def run_usb_stream(frame_size="QVGA", width=320, height=240, pixformat="RGB565",
                   quality=70, fps_limit=12, auto_gain=False, auto_whitebal=False,
                   auto_exposure=False, exposure_us=_DEFAULT_EXPOSURE_US, annotate=False,
                   debug_detector=True, adaptive_exposure=True):
    exposure_controller = configure_sensor(
        frame_size=frame_size,
        width=width,
        height=height,
        pixformat=pixformat,
        auto_gain=auto_gain,
        auto_whitebal=auto_whitebal,
        auto_exposure=auto_exposure,
        exposure_us=exposure_us,
        adaptive_exposure=adaptive_exposure,
    )

    detector = None
    if debug_detector:
        try:
            from green_light_detector import GreenLightDetector
            detector = GreenLightDetector()
        except Exception:
            detector = None

    vcp = USB_VCP()
    clock = time.clock()
    quality = _clamp(quality, 1, 95)
    fps_limit = int(fps_limit or 0)
    min_interval_ms = 0
    if fps_limit > 0:
        min_interval_ms = max(1, 1000 // fps_limit)
    next_frame_ms = time.ticks_ms()

    try:
        while True:
            if min_interval_ms > 0:
                now = time.ticks_ms()
                wait_ms = time.ticks_diff(next_frame_ms, now)
                if wait_ms > 0:
                    time.sleep_ms(wait_ms)
                next_frame_ms = time.ticks_add(time.ticks_ms(), min_interval_ms)

            clock.tick()
            img = sensor.snapshot()
            fps = clock.fps()
            if detector is not None:
                result = detector.process_frame(img)
                if exposure_controller is not None:
                    exposure_controller.update(img, result)
                _draw_detector_overlay(img, result, fps)
            elif debug_detector:
                if exposure_controller is not None:
                    exposure_controller.update(img, None)
                _draw_stream_status(img, fps, _COLOR_ERROR)
                _draw_text(img, _OVERLAY_TARGET_Y, "detector import failed", _COLOR_ERROR)
            elif annotate:
                if exposure_controller is not None:
                    exposure_controller.update(img, None)
                _draw_stream_status(img, fps, _COLOR_TEXT)
            elif exposure_controller is not None:
                exposure_controller.update(img, None)
            _write_frame(vcp, img, quality)
    except KeyboardInterrupt:
        pass
