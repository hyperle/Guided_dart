import sensor
import time
import sys
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
from preview_stream import (
    drain_usb_input,
    draw_preview_overlay,
    make_frame_metadata,
    usb_cable_connected,
    write_usb_preview_frame,
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
MEASUREMENT_IMAGE_SIZE_REFRESH_MS = 1000
MANUAL_EXPOSURE_US = 250
SEARCH_EXPOSURE_US = 1000
LOCKED_EXPOSURE_US = 300
LOCK_EXPOSURE_AREA_THRESHOLD = 500
LOCKED_EXPOSURE_MAX_MISSED_FRAMES = 50
EXPOSURE_SETTLE_FRAMES = 3
USB_DEBUG_POLL_INTERVAL_MS = 250
USB_PREVIEW_JPEG_QUALITY = 70
USB_PREVIEW_FPS_LIMIT = 12
WORK_FPS_EMA_NUM = 15
WORK_FPS_EMA_DEN = 100
ERROR_LOG_PATH = "/flash/openmv_last_error.txt"
SD_RECORD_STARTUP_DELAY_MS = 5000
recorder = None


def update_work_fps_x100(work_fps_x100, elapsed_us):
    if elapsed_us <= 0:
        return work_fps_x100
    inst_fps_x100 = 100000000 // elapsed_us
    if work_fps_x100 <= 0:
        return inst_fps_x100
    keep = WORK_FPS_EMA_DEN - WORK_FPS_EMA_NUM
    return ((work_fps_x100 * keep) + (inst_fps_x100 * WORK_FPS_EMA_NUM)) // WORK_FPS_EMA_DEN


def should_send_measurement_image_size(now_ms, last_image_size_ms, frame_index):
    if frame_index < MEASUREMENT_IMAGE_SIZE_STARTUP_FRAMES:
        return True
    return time.ticks_diff(now_ms, last_image_size_ms) >= MEASUREMENT_IMAGE_SIZE_REFRESH_MS


def write_last_error(exc):
    try:
        print("OMV_ERROR")
        sys.print_exception(exc)
    except Exception:
        pass

    try:
        f = open(ERROR_LOG_PATH, "w")
        sys.print_exception(exc, f)
        f.close()
        try:
            import os
            os.sync()
        except Exception:
            pass
    except Exception:
        pass


def close_sd_recorder(candidate):
    if candidate is None:
        return None
    try:
        candidate.close(quiet=True)
    except Exception:
        pass
    return None


def main():
    global recorder

    blink_startup_indicator()

    from green_light_detector import GreenLightDetector

    def make_sd_recorder():
        if not SD_RECORD_FLAG:
            return None
        try:
            from video_recorder import RollingMjpegRecorder
            candidate = RollingMjpegRecorder(
                segment_duration_ms=SD_RECORD_SEGMENT_DURATION_MS,
                max_segments=SD_RECORD_MAX_SEGMENTS,
                sync_interval_ms=SD_RECORD_SYNC_INTERVAL_MS,
                jpeg_quality=SD_RECORD_JPEG_QUALITY,
                min_free_bytes=SD_RECORD_MIN_FREE_BYTES,
            )
            if candidate.is_enabled():
                return candidate
            close_sd_recorder(candidate)
        except Exception:
            pass
        return None

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
    detector = GreenLightDetector()
    detector_params = detector.params
    detector_default_max_missed_frames = detector_params["max_missed_frames"]
    set_detector_full_scan_fallback = detector.set_full_scan_fallback_enabled
    recorder = None
    sd_recording_enabled = bool(SD_RECORD_FLAG)
    sd_recording_start_due_ms = time.ticks_add(time.ticks_ms(), SD_RECORD_STARTUP_DELAY_MS)
    sensor_snapshot = sensor.snapshot
    process_frame = detector.process_frame
    send_measurement = detector.send_measurement
    set_led = set_led_state
    set_fixed_exposure_api = sensor.set_auto_exposure
    no_target_coordinate = 0xFFFF
    last_image_size_ms = time.ticks_ms()
    cached_image_width = 0
    cached_image_height = 0
    search_exposure_us = SEARCH_EXPOSURE_US
    locked_exposure_us = LOCKED_EXPOSURE_US
    lock_area_threshold = LOCK_EXPOSURE_AREA_THRESHOLD
    locked_max_missed_frames = LOCKED_EXPOSURE_MAX_MISSED_FRAMES
    exposure_settle_frame_count = EXPOSURE_SETTLE_FRAMES
    usb_preview_active = False
    last_usb_debug_poll_ms = 0
    next_usb_preview_frame_ms = time.ticks_ms()
    usb_preview_min_interval_ms = max(1, 1000 // USB_PREVIEW_FPS_LIMIT)
    work_fps_x100 = 0
    last_loop_mark_us = 0
    preview_cost_us = 0
    exposure_locked = False
    exposure_missed_frames = 0
    exposure_settle_frames = 0
    frame_index = 0
    while True:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_usb_debug_poll_ms) >= USB_DEBUG_POLL_INTERVAL_MS:
            last_usb_debug_poll_ms = now_ms
            was_preview_active = usb_preview_active
            usb_preview_active = usb_cable_connected(usb_vcp)
            if usb_preview_active:
                drain_usb_input(usb_vcp)
            if usb_preview_active and not was_preview_active:
                next_usb_preview_frame_ms = now_ms

        loop_mark_us = time.ticks_us()
        if last_loop_mark_us != 0:
            period_us = time.ticks_diff(loop_mark_us, last_loop_mark_us) - preview_cost_us
            work_fps_x100 = update_work_fps_x100(work_fps_x100, period_us)
        last_loop_mark_us = loop_mark_us
        preview_cost_us = 0
        work_fps = work_fps_x100 / 100.0

        img = sensor_snapshot()
        if cached_image_width == 0:
            cached_image_width = img.width()
            cached_image_height = img.height()

        result = process_frame(img)
        include_image_size = should_send_measurement_image_size(now_ms, last_image_size_ms, frame_index)
        if include_image_size:
            last_image_size_ms = now_ms

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

        if sd_recording_enabled and recorder is None and time.ticks_diff(now_ms, sd_recording_start_due_ms) >= 0:
            recorder = make_sd_recorder()
            if recorder is None:
                sd_recording_enabled = False

        frame_metadata = None
        if recorder is not None or usb_preview_active:
            frame_metadata = make_frame_metadata(
                result,
                detector.last_debug,
                cached_image_width,
                cached_image_height,
                now_ms,
                frame_index,
                work_fps,
            )
        if recorder is not None:
            try:
                recorder.add_frame(img, frame_metadata)
                if not recorder.is_enabled():
                    recorder = close_sd_recorder(recorder)
                    sd_recording_enabled = False
            except Exception:
                recorder = close_sd_recorder(recorder)
                sd_recording_enabled = False

        if usb_preview_active and time.ticks_diff(now_ms, next_usb_preview_frame_ms) >= 0:
            next_usb_preview_frame_ms = time.ticks_add(now_ms, usb_preview_min_interval_ms)
            preview_t0 = time.ticks_us()
            draw_preview_overlay(img, result, work_fps, detector.last_debug or {})
            write_usb_preview_frame(usb_vcp, img, USB_PREVIEW_JPEG_QUALITY, frame_metadata)
            preview_cost_us = time.ticks_diff(time.ticks_us(), preview_t0)

        frame_index = (frame_index + 1) & 0x3FFFFFFF


try:
    main()
except Exception as exc:
    write_last_error(exc)
    try:
        if recorder is not None:
            recorder.close(quiet=True)
    except Exception:
        pass
    set_led_state(error=True)
    while True:
        time.sleep(1)
