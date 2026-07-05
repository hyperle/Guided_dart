import sensor
import time
from pyb import UART

time.sleep(3)

try:
    from pyb import LED
    _has_led = True
except (ImportError, AttributeError):
    LED = None
    _has_led = False

from adaptive_exposure import TargetExposureController
from green_light_detector import GreenLightDetector
from video_recorder import RollingMjpegRecorder


SENSOR_WIDTH = 320
SENSOR_HEIGHT = 240
UART_PORT = 1
UART_BAUDRATE = 115200
ADAPTIVE_EXPOSURE_ENABLED = False
MANUAL_EXPOSURE_US = 5000
RECORDING_SEGMENT_DURATION_MS = 15000
RECORDING_MAX_SEGMENTS = 12
RECORDING_SYNC_INTERVAL_MS = 1000
RECORDING_JPEG_QUALITY = 70
RECORDING_MIN_FREE_BYTES = 4 * 1024 * 1024


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((SENSOR_WIDTH, SENSOR_HEIGHT))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=MANUAL_EXPOSURE_US)
exposure_controller = None
if ADAPTIVE_EXPOSURE_ENABLED:
    exposure_controller = TargetExposureController(initial_exposure_us=MANUAL_EXPOSURE_US)
    exposure_controller.apply()
sensor.skip_frames(time=500)

uart = UART(UART_PORT, UART_BAUDRATE, timeout_char=1000)
clock = time.clock()
detector = GreenLightDetector()
if _has_led:
    red_led = LED(1)
    green_led = LED(2)
    green_led.on()
else:
    red_led = None
    green_led = None

recorder = RollingMjpegRecorder(
    segment_duration_ms=RECORDING_SEGMENT_DURATION_MS,
    max_segments=RECORDING_MAX_SEGMENTS,
    sync_interval_ms=RECORDING_SYNC_INTERVAL_MS,
    jpeg_quality=RECORDING_JPEG_QUALITY,
    min_free_bytes=RECORDING_MIN_FREE_BYTES,
)

heartbeat = False
while True:
    detector.poll_params(uart)

    clock.tick()
    img = sensor.snapshot()
    image_width = img.width()
    image_height = img.height()
    recorder.add_frame(img)

    result = detector.process_frame(img)
    if exposure_controller is not None:
        exposure_controller.update(img, result)

    if result is None:
        detector.send_measurement(uart, 0xFFFF, 0xFFFF, 0, image_width, image_height)
        print(clock.fps())
        heartbeat = not heartbeat
        if red_led is not None:
            if heartbeat:
                red_led.on()
            else:
                red_led.off()
        continue

    if red_led is not None:
        red_led.on()
    detector.send_measurement(uart, result["center_x"], result["center_y"], result["area"], image_width, image_height)
    print(clock.fps())
