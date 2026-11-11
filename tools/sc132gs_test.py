"""SC132GS color capture test for the Dart V1 CanMV MicroPython firmware.

Run from the CanMV VS Code extension, with other camera users stopped.
CHECK_CHIP_ID=False skips Python ID checks, not native driver / ISP ID access.
PROBE_ONLY checks I2C without starting MIPI. TEST_SECONDS=0 previews until stopped.
USE_VGA_MODE selects the sensor crop; both modes output a 640x480 center crop.
Exposure is in microseconds, analog gain is a linear multiplier.
Click two opposite corners in the IDE preview to sample a color region.
RGB565 frames are previewed without saving images; color features can be saved as JSON.
The default TEST_SECONDS=0 keeps the preview running until stopped in the IDE.
"""

import os
import time


PROBE_ONLY = False
CHECK_CHIP_ID = False
I2C_ADDRESS = 0x30
USE_VGA_MODE = False
PREVIEW_ENABLED = True
TEST_SECONDS = 1200
# Keep manual -: automatic exposure still fails in the tested firmware.
EXPOSURE_US = 5000
ANALOG_GAIN = 1.0
FRAME_TIMEOUT_MS = 1000
DIAGNOSTICS_ENABLED = True
COLOR_SAMPLING_ENABLED = True
COLOR_ROI = None                 # Optional (x, y, width, height) in the 640x480 image.
COLOR_SAMPLE_FRAMES = 30         # New live frames after selection, not a frozen IDE frame.
COLOR_PERCENTILES = (0.05, 0.95)
COLOR_THRESHOLD_MARGIN = (3, 5, 5)  # Extra tolerance for the L, a and b channels.
COLOR_RESULT_PATH = "/sdcard/sc132gs_color_sample.json"  # Empty string: console only.


def validate_config():
    for name, value in (("PROBE_ONLY", PROBE_ONLY),
                        ("CHECK_CHIP_ID", CHECK_CHIP_ID),
                        ("USE_VGA_MODE", USE_VGA_MODE),
                        ("PREVIEW_ENABLED", PREVIEW_ENABLED),
                        ("DIAGNOSTICS_ENABLED", DIAGNOSTICS_ENABLED),
                        ("COLOR_SAMPLING_ENABLED", COLOR_SAMPLING_ENABLED)):
        if type(value) is not bool:
            raise ValueError("%s must be True or False" % name)
    for name, value, minimum in (("TEST_SECONDS", TEST_SECONDS, 0),
                                 ("EXPOSURE_US", EXPOSURE_US, 1),
                                 ("FRAME_TIMEOUT_MS", FRAME_TIMEOUT_MS, 1),
                                 ("COLOR_SAMPLE_FRAMES", COLOR_SAMPLE_FRAMES, 1)):
        if type(value) is not int or value < minimum:
            raise ValueError("%s must be an integer >= %d" % (name, minimum))
    if type(ANALOG_GAIN) not in (int, float) or not 1.0 <= ANALOG_GAIN <= 28.546875:
        raise ValueError("ANALOG_GAIN must be within 1.0..28.546875")
    if type(I2C_ADDRESS) is not int or I2C_ADDRESS not in range(0x30, 0x34):
        raise ValueError("I2C_ADDRESS must be within 0x30..0x33")
    if COLOR_ROI is not None:
        if (not isinstance(COLOR_ROI, (tuple, list)) or len(COLOR_ROI) != 4
                or any(type(value) is not int for value in COLOR_ROI)):
            raise ValueError("COLOR_ROI must be None or (x, y, width, height)")
        x, y, width, height = COLOR_ROI
        if x < 0 or y < 0 or width < 4 or height < 4 or x + width > 640 or y + height > 480:
            raise ValueError("COLOR_ROI must be at least 4x4 and fit inside 640x480")
    if (not isinstance(COLOR_PERCENTILES, (tuple, list)) or len(COLOR_PERCENTILES) != 2
            or any(type(value) not in (int, float) for value in COLOR_PERCENTILES)
            or not 0 <= COLOR_PERCENTILES[0] < COLOR_PERCENTILES[1] <= 1):
        raise ValueError("COLOR_PERCENTILES must satisfy 0 <= low < high <= 1")
    if (not isinstance(COLOR_THRESHOLD_MARGIN, (tuple, list))
            or len(COLOR_THRESHOLD_MARGIN) != 3
            or any(type(value) is not int or value < 0 for value in COLOR_THRESHOLD_MARGIN)):
        raise ValueError("COLOR_THRESHOLD_MARGIN must contain three nonnegative integers")
    if not isinstance(COLOR_RESULT_PATH, str):
        raise ValueError("COLOR_RESULT_PATH must be a path string or an empty string")


class ColorSampler:
    """Bounded multi-frame Lab statistics from the unannotated RGB565 image."""

    def __init__(self):
        self.touch = None
        self.corner = None
        self.roi = None
        self.result = None
        self.samples = 0
        if COLOR_ROI is not None:
            self.select(COLOR_ROI)

    def start(self):
        if PREVIEW_ENABLED:
            try:
                from machine import TOUCH

                self.touch_down = TOUCH.EVENT_DOWN
                self.touch = TOUCH(TOUCH.DEV_IDE, range_x=640, range_y=480)
            except (ImportError, AttributeError, OSError, RuntimeError) as error:
                if str(error) == "IDE interrupt":
                    raise
                print("[COLOR] IDE touch unavailable: %s; set COLOR_ROI to sample" % error)
        if self.touch is not None:
            print("[COLOR] Click two opposite corners in the IDE preview; "
                  "the next pair starts a new sample")
        if self.roi is None and self.touch is None:
            print("[COLOR] Waiting for a configured COLOR_ROI; preview/capture continues")
        print("[COLOR] Sampling ISP output with fixed exposure/gain; "
              "ISP white balance is not locked by this script")

    def select(self, roi):
        self.roi = tuple(roi)
        self.corner = None
        self.result = None
        self.samples = 0
        self.sums = [0.0, 0.0, 0.0]
        self.second_moments = [0.0, 0.0, 0.0]
        self.low = [100, 127, 127]
        self.high = [0, -128, -128]
        print("[COLOR] roi=%s; collecting %d live frames; keep the target inside the box" %
              (self.roi, COLOR_SAMPLE_FRAMES))

    def select_point(self, x, y):
        point = (max(0, min(639, x)), max(0, min(479, y)))
        if self.corner is None:
            # Cancel the old sample immediately so two regions are never mixed.
            self.roi = None
            self.result = None
            self.samples = 0
            self.corner = point
            print("[COLOR] first corner=%s; click the opposite corner" % (point,))
            return
        x0, y0 = self.corner
        width, height = abs(point[0] - x0) + 1, abs(point[1] - y0) + 1
        if width < 4 or height < 4:
            print("[COLOR] Region must be at least 4x4; click the opposite corner again")
            return
        self.select((min(x0, point[0]), min(y0, point[1]), width, height))

    def process(self, frame, frame_index):
        if self.touch is not None:
            for point in self.touch.read(5):
                # The IDE sends DOWN/UP pairs for a click. Count each click once.
                if point.event == self.touch_down:
                    self.select_point(point.x, point.y)
        if self.roi is None or self.samples >= COLOR_SAMPLE_FRAMES:
            return
        if self.samples == 0:
            self.started = time.ticks_ms()
            self.first_frame = frame_index
        histogram = frame.get_histogram(roi=self.roi)
        stats = histogram.get_statistics()
        mean = (stats.l_mean(), stats.a_mean(), stats.b_mean())
        stdev = (stats.l_stdev(), stats.a_stdev(), stats.b_stdev())
        low = histogram.get_percentile(COLOR_PERCENTILES[0])
        high = histogram.get_percentile(COLOR_PERCENTILES[1])
        lower = (low.l_value(), low.a_value(), low.b_value())
        upper = (high.l_value(), high.a_value(), high.b_value())
        for channel in range(3):
            self.sums[channel] += mean[channel]
            # Equal-size ROIs: E[x^2] = variance + mean^2 includes between-frame drift.
            self.second_moments[channel] += stdev[channel] ** 2 + mean[channel] ** 2
            self.low[channel] = min(self.low[channel], lower[channel])
            self.high[channel] = max(self.high[channel], upper[channel])
        self.samples += 1
        if self.samples == COLOR_SAMPLE_FRAMES:
            self.finish(frame_index)

    def finish(self, frame_index):
        import json

        mean = [value / self.samples for value in self.sums]
        stdev = [max(0.0, self.second_moments[i] / self.samples - mean[i] ** 2) ** 0.5
                 for i in range(3)]
        threshold = []
        for i, (minimum, maximum) in enumerate(((0, 100), (-128, 127), (-128, 127))):
            threshold.extend((max(minimum, self.low[i] - COLOR_THRESHOLD_MARGIN[i]),
                              min(maximum, self.high[i] + COLOR_THRESHOLD_MARGIN[i])))
        self.result = {
            "version": 1, "kind": "roi_color_feature", "sensor": "SC132GS",
            "sensor_mode": "vga" if USE_VGA_MODE else "full", "frame_size": (640, 480),
            "pixel_format": "RGB565", "color_space": "CanMV Lab",
            "roi": self.roi, "sample_frames": self.samples,
            "sample_pixels": self.samples * self.roi[2] * self.roi[3],
            "first_frame": self.first_frame, "last_frame": frame_index,
            "elapsed_ms": time.ticks_diff(time.ticks_ms(), self.started),
            "requested_exposure_us": EXPOSURE_US, "requested_analog_gain": ANALOG_GAIN,
            "white_balance": "firmware ISP; not locked",
            "lab_mean": mean, "lab_stdev": stdev,
            "percentiles": COLOR_PERCENTILES,
            "percentile_envelope_low": self.low, "percentile_envelope_high": self.high,
            "threshold_margin": COLOR_THRESHOLD_MARGIN, "lab_threshold": threshold,
        }
        print("[COLOR] complete: roi=%s frames=%d Lab mean=%s stdev=%s" %
              (self.roi, self.samples, mean, stdev))
        print("COLOR_THRESHOLD = %s  # (L_min, L_max, a_min, a_max, b_min, b_max)" %
              (tuple(threshold),))
        print("[COLOR] result_json=%s" % json.dumps(self.result))
        if COLOR_RESULT_PATH:
            try:
                with open(COLOR_RESULT_PATH, "w") as output:
                    json.dump(self.result, output)
                print("[COLOR] saved: %s" % COLOR_RESULT_PATH)
            except OSError as error:
                print("[COLOR] Save failed: %s; use result_json from the console" % error)

    def draw(self, frame):
        if self.corner is not None:
            frame.draw_cross(self.corner[0], self.corner[1], color=(255, 255, 0))
            text = "Click the opposite corner"
        elif self.roi is not None:
            color = (0, 255, 0) if self.result is not None else (255, 255, 0)
            frame.draw_rectangle(self.roi, color=color, thickness=2)
            if self.result is not None:
                text = "Lab mean: %.1f %.1f %.1f; click twice to resample" % tuple(
                    self.result["lab_mean"])
            else:
                text = "Color sample: %d/%d" % (self.samples, COLOR_SAMPLE_FRAMES)
        elif self.touch is not None:
            text = "Color sample: click two opposite corners"
        else:
            return
        frame.draw_string_advanced(8, 8, 16, text, color=(255, 255, 255))

    def deinit(self):
        if self.roi is not None and self.samples < COLOR_SAMPLE_FRAMES:
            print("[COLOR] Incomplete sample: %d/%d frames; no result saved" %
                  (self.samples, COLOR_SAMPLE_FRAMES))
        if self.touch is not None:
            self.touch.deinit()


def check_firmware(board):
    if getattr(board, "CAMERA_NAME", None) != "SC132GS":
        raise RuntimeError("SC132GS firmware required: flash the Dart V1 build "
                           "from kRTOSSDK branch micropython, not SC038HGS firmware")
    if not hasattr(board, "camera_sensor") or (
            (CHECK_CHIP_ID or PROBE_ONLY) and not hasattr(board, "camera_probe")):
        raise RuntimeError("Update the firmware's dart_board.py camera helpers")


def probe_camera(board):
    check_firmware(board)
    address, chip_id = board.camera_probe()
    if address not in range(0x30, 0x34) or chip_id != 0x0132:
        raise RuntimeError("Unexpected sensor: address=0x%02x chip_id=0x%04x; "
                           "expected SC132GS ID 0x0132 at 0x30..0x33" %
                           (address, chip_id))
    print("[PASS] probe: I2C%d address=0x%02x chip_id=0x%04x CSI%d" %
          (board.CAMERA_I2C, address, chip_id, board.CAMERA_CSI))


def report_sensor_mode(sensor, board):
    expected_type = board.CAMERA_VGA_MODE if USE_VGA_MODE else board.CAMERA_MODE
    expected_size = (640, 480) if USE_VGA_MODE else (1080, 1280)
    actual_type = sensor.get_type()
    info = sensor._dev_attr.sensor_info
    if actual_type != expected_type or info.type != expected_type:
        raise RuntimeError("SC132GS firmware mode mismatch: expected type=%d, "
                           "selected=%d, metadata=%d" %
                           (expected_type, actual_type, info.type))
    if (info.width, info.height) != expected_size:
        raise RuntimeError("SC132GS firmware mode mismatch: expected %dx%d, got %dx%d" %
                           (expected_size[0], expected_size[1], info.width, info.height))
    if info.data_type != 0x2b:
        raise RuntimeError("SC132GS RAW10 required: firmware datatype=0x%02x" % info.data_type)
    print("[INFO] firmware sensor_mode=%dx%d %s type=%d RAW10 nominal_fps=%d "
          "lanes_enum=%d output=640x480 RGB565" %
          (info.width, info.height, "crop" if USE_VGA_MODE else "full sensor",
           actual_type, info.fps, info.mipi_lanes))
    print("[INFO] Mode metadata is not proof of sensor initialization or received frames")


def diagnose_startup(board, error):
    if "vicap init failed(-1609203696)" in str(error):
        print("[DIAG] -1609203696 = 0xa0158010 = K_ERR_VICAP_NOTREADY; "
              "VICAP initialization failed before frame capture")
    device_path = "/dev/sensor_sc132gs_csi%d" % board.CAMERA_CSI
    try:
        with open(device_path, "r+b"):
            pass
        print("[DIAG] sensor device opens: %s; registration alone does not "
              "prove current I2C communication" % device_path)
    except Exception as error:
        print("[DIAG] sensor device unavailable: %s: %s" % (device_path, error))
        print("[HINT] A configured Sensor type does not register a kernel device; "
              "check SC132GS kernel probe / fixed binding. A MicroPython "
              "soft reboot does not restart RT-Smart")
    print("[DIAG] Startup I2C recheck skipped: native failure cleanup may already "
          "have powered off the sensor")
    print("[HINT] Preserve the first native sensor / ISP error on UART0; "
          "a post-failure I2C read cannot establish the startup failure cause")
    database_name = "sc132gs-%s" % ("640x480" if USE_VGA_MODE else "1080x1280")
    for suffix in (".bin", ".xml", "_auto.json", "_manual.json"):
        path = "/bin/" + database_name + suffix
        try:
            size = os.stat(path)[6]
            if size <= 0:
                raise RuntimeError("empty file")
            print("[DIAG] ISP file: %s bytes=%d" % (path, size))
        except Exception as error:
            print("[DIAG] ISP file unavailable: %s: %s" % (path, error))


def diagnose_capture(sensor, board):
    print("[DIAG] failed(3) wraps kd_mpi_vicap_dump_frame failure; "
          "it is not the original MPI error code")
    try:
        from mpp.vicap import VICAP_MIPI_4LANE

        info = sensor._dev_attr.sensor_info
        print("[DIAG] receiver: %dx%d fps=%d csi_enum=%d lanes_enum=%d "
              "expected_4lane_enum=%d phy_enum=%d datatype=0x%02x" %
              (info.width, info.height, info.fps, info.csi_num, info.mipi_lanes,
               VICAP_MIPI_4LANE, info.phy_freq, info.data_type))
    except Exception as error:
        print("[DIAG] receiver readback unavailable: %s" % error)
    try:
        from machine import I2C

        address = I2C_ADDRESS
        if CHECK_CHIP_ID:
            address, chip_id = board.camera_probe()
            if chip_id != 0x0132 or address not in range(0x30, 0x34):
                raise RuntimeError("SC132GS no longer responds")
        bus = I2C(board.CAMERA_I2C, freq=400000)
        values = []
        registers = {}
        for register in (0x0100, 0x3018, 0x3019, 0x3031, 0x3037, 0x303f, 0x3208,
                         0x3209, 0x320a, 0x320b, 0x320e, 0x320f, 0x3222, 0x3652,
                         0x3653, 0x3654, 0x36e9, 0x36f9, 0x4837, 0x4b01):
            try:
                value = bus.readfrom_mem(address, register, 1, addrsize=16)[0]
            except Exception as error:
                raise RuntimeError("I2C%d address=0x%02x register=0x%04x: %s" %
                                   (board.CAMERA_I2C, address, register, error))
            registers[register] = value
            values.append("%04x=%02x" % (register, value))
        print("[DIAG] sensor registers: " + " ".join(values))
        raw_bits = {0x08: 8, 0x0c: 12}.get(registers[0x3031] & 0x0f, 10)
        phy_bits = (8, 10, 12, "reserved")[(registers[0x3037] >> 5) & 0x03]
        print("[DIAG] SC132GS decoded: streaming=%d lanes=%d raw_bits=%d "
              "phy_bits=%s interface=%s external_trigger=%d" %
              (registers[0x0100] & 0x01, ((registers[0x3018] >> 5) & 0x07) + 1,
               raw_bits, phy_bits, "LVDS" if registers[0x4b01] & 0x08 else "MIPI",
               (registers[0x3222] >> 1) & 0x01))
        print("[DIAG] Dart SC132GS MCLK comes from external Y1, not a K230 "
              "MCLK pin; verify its actual frequency")
    except Exception as error:
        print("[DIAG] sensor readback unavailable: %s" % error)

    try:
        from mpp.vicap import kd_mpi_vicap_tpg_enable
    except ImportError as error:
        print("[DIAG] internal TPG unavailable: %s" % error)
        return
    try:
        result = kd_mpi_vicap_tpg_enable(True)
        if result != 0:
            raise RuntimeError("TPG enable returned %s" % result)
        frame = sensor.snapshot(timeout=max(FRAME_TIMEOUT_MS, 1000), auto_reboot=False)
        if frame is None or (frame.width(), frame.height()) != (640, 480):
            raise RuntimeError("Internal TPG did not return a 640x480 frame")
        print("[DIAG] ISP internal TPG produced 640x480: ISP output works with "
              "generated input; investigate sensor / MCLK / MIPI input")
        print("[DIAG] This is NOT a camera capture pass; TPG is not shown in the IDE")
    except Exception as error:
        print("[DIAG] internal TPG check failed: %s" % error)
    finally:
        try:
            result = kd_mpi_vicap_tpg_enable(False)
            if result != 0:
                raise RuntimeError("TPG disable returned %s" % result)
        except Exception as error:
            print("[FAIL] internal TPG disable: %s; restart the board before retesting" % error)


def capture_frames(sensor, display, sampler=None):
    from image import RGB565

    frames = 0
    report_frames = 0
    started = time.ticks_ms()
    reported = started
    elapsed_ms = 0
    while TEST_SECONDS == 0 or elapsed_ms < TEST_SECONDS * 1000:
        os.exitpoint()
        frame = sensor.snapshot(timeout=FRAME_TIMEOUT_MS, auto_reboot=False)
        if frame is None or (frame.width(), frame.height()) != (640, 480):
            raise RuntimeError("Expected a nonempty 640x480 frame")
        if frame.format() != RGB565:
            raise RuntimeError("Expected an RGB565 color frame")
        frames += 1
        if sampler is not None:
            sampler.process(frame, frames)
        if display is not None:
            if sampler is not None:
                sampler.draw(frame)
            display.show_image(frame)
        if frames == 1:
            print("[PASS] first frame: 640x480 RGB565")
        now = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now, started)
        report_ms = time.ticks_diff(now, reported)
        if report_ms >= 1000:
            print("[INFO] frames=%d received_fps=%.1f elapsed_ms=%d" %
                  (frames, (frames - report_frames) * 1000.0 / report_ms, elapsed_ms))
            reported = now
            report_frames = frames
    return frames, elapsed_ms


def release_resources(sensor, display, sampler=None):
    actions = []
    if sampler is not None:
        actions.append(("color touch", sampler.deinit))
    if sensor is not None:
        actions.append(("sensor", lambda: sensor.stop(is_del=True)))
    if display is not None:
        actions.append(("display", display.deinit))
    if actions:
        actions.append(("exitpoint", lambda: os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)))
        actions.append(("settle", lambda: time.sleep_ms(100)))
    first_error = None
    for name, action in actions:
        try:
            action()
        except Exception as error:
            print("[FAIL] cleanup %s: %s" % (name, error))
            if first_error is None:
                first_error = error
    return first_error


def main():
    sensor = None
    display = None
    sampler = None
    completed = False
    stage = "configuration"
    try:
        validate_config()
        stage = "firmware / I2C probe"
        import dart_board

        check_firmware(dart_board)
        probe_deferred = False
        if CHECK_CHIP_ID or PROBE_ONLY:
            try:
                probe_camera(dart_board)
            except OSError as error:
                if PROBE_ONLY:
                    raise
                probe_deferred = True
                print("[INFO] I2C probe unavailable before stream start: %s" % error)
                print("[INFO] Sensor power-off is only one possible cause. "
                      "Rechecking after driver power-on")
        else:
            print("[INFO] Chip ID check skipped in Python; using the configured SC132GS mode")
            print("[INFO] Native sensor / ISP ID access is controlled by firmware")
        if PROBE_ONLY:
            print("[INFO] Probe only: MIPI / ISP capture has NOT been tested")
            return

        stage = "sensor construction / reset"
        from media.sensor import Sensor

        sensor = dart_board.camera_sensor(vga=USE_VGA_MODE)
        sensor.reset()
        stage = "firmware sensor mode"
        report_sensor_mode(sensor, dart_board)
        sensor.set_framesize(width=640, height=480, crop=True)
        # Preserve ISP color processing/white balance; do not convert to grayscale.
        sensor.set_pixformat(Sensor.RGB565)
        sensor.auto_exposure(False)

        if PREVIEW_ENABLED:
            stage = "IDE display initialization"
            from media.display import Display

            display = Display
            display.init(display.VIRT, width=640, height=480, fps=30, to_ide=True)

        stage = "MIPI / ISP stream start"
        sensor.run()
        if probe_deferred:
            stage = "powered I2C probe"
            probe_camera(dart_board)
        stage = "manual gain / exposure"
        if sensor.again(ANALOG_GAIN) != 0:
            raise RuntimeError("Setting analog gain failed")
        if not sensor.exposure(EXPOSURE_US):
            raise RuntimeError("Setting exposure failed")
        print("[INFO] exposure_us=%d analog_gain=%.3f duration_s=%d" %
              (EXPOSURE_US, ANALOG_GAIN, TEST_SECONDS))
        if TEST_SECONDS == 0:
            print("[INFO] Continuous color preview; stop in the IDE to release resources")
        print("[INFO] received_fps includes Python / IDE overhead, not sensor FPS")

        if COLOR_SAMPLING_ENABLED:
            stage = "color sampler initialization"
            sampler = ColorSampler()
            sampler.start()
        stage = "frame capture"
        frames, elapsed_ms = capture_frames(sensor, display, sampler)
        completed = True
    except KeyboardInterrupt:
        print("[STOP] Interrupted during %s; capture test not completed" % stage)
        return
    except Exception as error:
        if str(error) == "IDE interrupt":
            print("[STOP] IDE interrupted during %s; capture test not completed" % stage)
            return
        print("[FAIL] %s: %s" % (stage, error))
        if stage in ("firmware / I2C probe", "powered I2C probe"):
            print("[HINT] Check SC132GS firmware, adapter power, FPC and I2C1; "
                  "do not reuse SC038HGS GPIO control")
        elif stage == "MIPI / ISP stream start":
            print("[HINT] Check sensor availability and native VICAP / ISP "
                  "errors on UART0 before changing MIPI settings")
        elif stage != "configuration":
            print("[HINT] Stop other camera users; check the CSI0 four-lane "
                  "configuration, external MCLK and installed SC132GS ISP files")
        if DIAGNOSTICS_ENABLED:
            try:
                if stage == "MIPI / ISP stream start":
                    diagnose_startup(dart_board, error)
                elif stage == "frame capture":
                    diagnose_capture(sensor, dart_board)
            except Exception as diagnostic_error:
                print("[DIAG] diagnostics unavailable: %s" % diagnostic_error)
        raise
    finally:
        cleanup_error = release_resources(sensor, display, sampler)
        if cleanup_error is not None and completed:
            raise cleanup_error

    print("[PASS] capture: frames=%d elapsed_ms=%d received_fps=%.1f; resources released" %
          (frames, elapsed_ms, frames * 1000.0 / max(elapsed_ms, 1)))


if __name__ == "__main__":
    main()
