import sensor
import time
from pyb import UART, USB_VCP

time.sleep(3)

try:
    from pyb import LED
    _has_led = True
except (ImportError, AttributeError):
    LED = None
    _has_led = False

from green_light_detector import GreenLightDetector
from video_recorder import RollingMjpegRecorder


SENSOR_WIDTH = 320
SENSOR_HEIGHT = 240
UART_PORT = 1
UART_BAUDRATE = 115200
MANUAL_EXPOSURE_US = 800
RECORDING_SEGMENT_DURATION_MS = 15000
RECORDING_MAX_SEGMENTS = 12
RECORDING_SYNC_INTERVAL_MS = 1000
RECORDING_JPEG_QUALITY = 30
RECORDING_MIN_FREE_BYTES = 4 * 1024 * 1024
USB_DEBUG_POLL_INTERVAL_MS = 250
DEBUG_FPS_PRINT_INTERVAL_MS = 1000


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
if _has_led:
    red_led = LED(1)
    green_led = LED(2)
    green_led.on()
else:
    red_led = None
    green_led = None

def usb_debug_connected():
    try:
        return usb_vcp.isconnected()
    except Exception:
        return False


def make_recorder():
    return RollingMjpegRecorder(
        segment_duration_ms=RECORDING_SEGMENT_DURATION_MS,
        max_segments=RECORDING_MAX_SEGMENTS,
        sync_interval_ms=RECORDING_SYNC_INTERVAL_MS,
        jpeg_quality=RECORDING_JPEG_QUALITY,
        min_free_bytes=RECORDING_MIN_FREE_BYTES,
    )


recorder = None
usb_debug_active = False
last_usb_debug_poll_ms = 0
last_debug_fps_print_ms = 0
heartbeat = False
while True:
    detector.poll_params(uart)

    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, last_usb_debug_poll_ms) >= USB_DEBUG_POLL_INTERVAL_MS:
        last_usb_debug_poll_ms = now_ms
        next_usb_debug_active = usb_debug_connected()
        if next_usb_debug_active and not usb_debug_active:
            recorder = make_recorder()
            last_debug_fps_print_ms = now_ms
        elif (not next_usb_debug_active) and usb_debug_active:
            if recorder is not None:
                recorder.close(quiet=True)
                recorder = None
        usb_debug_active = next_usb_debug_active

    clock.tick()
    img = sensor.snapshot()
    image_width = img.width()
    image_height = img.height()

    if recorder is not None:
        recorder.add_frame(img)

    result = detector.process_frame(img)

    if result is None:
        detector.send_measurement(uart, 0xFFFF, 0xFFFF, 0, image_width, image_height)
        heartbeat = not heartbeat
        if red_led is not None:
            if heartbeat:
                red_led.on()
            else:
                red_led.off()
    else:
        if red_led is not None:
            red_led.on()
        detector.send_measurement(uart, result["center_x"], result["center_y"], result["area"], image_width, image_height)

    if usb_debug_active and time.ticks_diff(now_ms, last_debug_fps_print_ms) >= DEBUG_FPS_PRINT_INTERVAL_MS:
        last_debug_fps_print_ms = now_ms
        print(clock.fps())
