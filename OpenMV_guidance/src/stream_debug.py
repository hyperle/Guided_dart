import sensor
import time
from pyb import UART, USB_VCP
from preview_stream import (
    COLOR_ERROR,
    COLOR_TEXT,
    draw_preview_overlay,
    draw_stream_status,
    draw_text,
    make_frame_metadata,
    write_usb_preview_frame,
)

try:
    from pyb import LED
    _status_red_led = LED(1)
    _status_green_led = LED(2)
    _status_red_led.off()
    _status_green_led.off()
except Exception:
    _status_red_led = None
    _status_green_led = None

try:
    from adaptive_exposure import TargetExposureController
except Exception:
    TargetExposureController = None

_DEFAULT_EXPOSURE_US = 1000
_CONTROL_UART_PORT = 1
_CONTROL_UART_BAUDRATE = 115200
_MEASUREMENT_HEADER = 0x5A
_MEASUREMENT_LEGACY_LENGTH = 0x06
_MEASUREMENT_EXTENDED_LENGTH = 0x0A
_MEASUREMENT_LENGTH = _MEASUREMENT_EXTENDED_LENGTH
_CONTROL_IMAGE_SIZE_STARTUP_FRAMES = 8
_CONTROL_IMAGE_SIZE_REFRESH_MS = 1000
_NO_TARGET_COORDINATE = 0xFFFF


def _set_status_led(error=False, target_detected=False):
    if _status_red_led is None or _status_green_led is None:
        return
    try:
        if error:
            _status_green_led.off()
            _status_red_led.on()
        elif target_detected:
            _status_red_led.off()
            _status_green_led.on()
        else:
            _status_red_led.off()
            _status_green_led.off()
    except Exception:
        pass


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
        if adaptive_exposure and pixformat.upper() == "RGB565" and TargetExposureController is not None:
            exposure_controller = TargetExposureController(initial_exposure_us=int(exposure_us or _DEFAULT_EXPOSURE_US))
            exposure_controller.apply()
    sensor.skip_frames(time=300)
    return exposure_controller



def _u16(value):
    value = int(value)
    if value < 0:
        return 0
    if value > 0xFFFF:
        return 0xFFFF
    return value


def _append_u16_be(packet, value):
    value = _u16(value)
    packet.append((value >> 8) & 0xFF)
    packet.append(value & 0xFF)


def _checksum_bytes(packet):
    checksum = 0
    for byte in packet:
        checksum = (checksum + byte) & 0xFF
    return checksum


def _send_control_measurement(uart, result, image_width, image_height, include_image_size=True):
    if uart is None:
        return True

    if result is None:
        x = _NO_TARGET_COORDINATE
        y = _NO_TARGET_COORDINATE
        area = 0
    else:
        x = result["center_x"]
        y = result["center_y"]
        area = result["area"]

    packet = bytearray()
    packet.append(_MEASUREMENT_HEADER)
    packet.append(_MEASUREMENT_EXTENDED_LENGTH if include_image_size else _MEASUREMENT_LEGACY_LENGTH)
    _append_u16_be(packet, x)
    _append_u16_be(packet, y)
    _append_u16_be(packet, area)
    if include_image_size:
        _append_u16_be(packet, image_width)
        _append_u16_be(packet, image_height)
    packet.append(_checksum_bytes(packet))
    try:
        uart.write(packet)
    except Exception:
        return False
    return True


def _should_send_control_image_size(now_ms, last_image_size_ms, frame_index):
    if frame_index < _CONTROL_IMAGE_SIZE_STARTUP_FRAMES:
        return True
    if _CONTROL_IMAGE_SIZE_REFRESH_MS <= 0:
        return False
    return time.ticks_diff(now_ms, last_image_size_ms) >= _CONTROL_IMAGE_SIZE_REFRESH_MS



def run_usb_stream(frame_size="QVGA", width=320, height=240, pixformat="RGB565",
                   quality=70, fps_limit=12, auto_gain=False, auto_whitebal=False,
                   auto_exposure=False, exposure_us=_DEFAULT_EXPOSURE_US, annotate=False,
                   debug_detector=True, adaptive_exposure=True, emit_metadata=False,
                   control_uart=False, control_uart_port=_CONTROL_UART_PORT,
                   control_uart_baudrate=_CONTROL_UART_BAUDRATE):
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

    control_uart_device = None
    if control_uart:
        if detector is None:
            raise RuntimeError("control UART requires GreenLightDetector")
        control_uart_device = UART(
            int(control_uart_port or _CONTROL_UART_PORT),
            int(control_uart_baudrate or _CONTROL_UART_BAUDRATE),
            timeout_char=1000,
        )

    vcp = USB_VCP()
    clock = time.clock()
    quality = _clamp(quality, 1, 95)
    fps_limit = int(fps_limit or 0)
    min_interval_ms = 0
    if fps_limit > 0:
        min_interval_ms = max(1, 1000 // fps_limit)
    next_frame_ms = time.ticks_ms()
    frame_index = 0
    last_control_image_size_ms = time.ticks_ms()

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
            now_ms = time.ticks_ms()
            result = None
            debug = None
            if detector is not None:
                result = detector.process_frame(img)
                debug = detector.last_debug
                if exposure_controller is not None:
                    exposure_controller.update(img, result)
                _set_status_led(target_detected=(result is not None))
                draw_preview_overlay(img, result, fps, debug)
            elif debug_detector:
                if exposure_controller is not None:
                    exposure_controller.update(img, None)
                _set_status_led(error=True)
                draw_stream_status(img, fps, COLOR_ERROR)
                draw_text(img, 14, "detector import failed", COLOR_ERROR)
            elif annotate:
                if exposure_controller is not None:
                    exposure_controller.update(img, None)
                draw_stream_status(img, fps, COLOR_TEXT)
            elif exposure_controller is not None:
                exposure_controller.update(img, None)
            if control_uart_device is not None:
                include_image_size = _should_send_control_image_size(
                    now_ms,
                    last_control_image_size_ms,
                    frame_index,
                )
                if include_image_size:
                    last_control_image_size_ms = now_ms
                if not _send_control_measurement(
                    control_uart_device,
                    result,
                    img.width(),
                    img.height(),
                    include_image_size,
                ):
                    control_uart_device = None
            metadata = None
            if emit_metadata:
                metadata = make_frame_metadata(
                    result,
                    debug,
                    img.width(),
                    img.height(),
                    now_ms,
                    frame_index,
                    fps,
                    debug_detector,
                    detector is not None,
                )
            if not write_usb_preview_frame(vcp, img, quality, metadata):
                break
            frame_index += 1
    except KeyboardInterrupt:
        pass
    finally:
        _set_status_led()
