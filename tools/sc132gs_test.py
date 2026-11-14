"""SC132GS color capture test for the Dart V1 CanMV MicroPython firmware.

Run from the CanMV VS Code extension, with other camera users stopped.
CHECK_CHIP_ID=False skips Python ID checks, not native driver / ISP ID access.
PROBE_ONLY checks I2C without starting MIPI. TEST_SECONDS=0 previews until stopped.
Full mode runs the sensor at native 1080x1280. NATIVE_FULL_FRAME=True delivers every
1080x1280 pixel (needs the PITCH_FIX firmware); False lets the ISP scale the whole view,
uncropped, to 480x640. USE_VGA_MODE=True selects a 640x480 sensor crop.
The IDE preview uses the virtual display: VO writeback is JPEG-encoded in hardware.
PREVIEW_FPS caps how often frames are copied to it; capture continues between them.
Exposure is in microseconds, analog gain is a linear multiplier.
AUTO_WHITE_BALANCE=True runs ISP AWB. False clears the AWB and WB flags, but the ISP
keeps the profile's fixed gains (R 1.12, B 2.51), which turn this lighting blue-violet.
ISP_3DNR_ENABLED=False disables temporal noise reduction, which smears moving targets.
RAW_CHECK_ENABLED reads sensor RAW10 before every ISP stage and reports the color
filter phase, MIPI byte-lane errors and brightness flicker; aim at an evenly lit,
unsaturated white or grey target that fills the view. It does not use the IDE preview.
Run it once with the lens fully covered to measure RAW_BLACK_LEVEL; WB gain estimates
need that value. OUTPUT_MONITOR_ENABLED prints the ISP output color of every frame,
separating ISP flashes from IDE preview flashes.
DETECTION_ENABLED runs the lamp / armor detector entirely in this Python file, using
CanMV RGB565/Lab connected components; no grayscale recognition image or native
_dart_detect / _dart_raw interpreter module is needed.
The current lamp anchors the armor search above it. Color is checked before selection
and tracking; both armor bars must have the same accepted color. Lab defaults are
starting points: sample the real targets with the current exposure and white balance.
Click two opposite corners in the IDE preview to restrict detection to that ROI.
Use the preview buttons or terminal keys: r selects a detection ROI, c samples color,
f clears the detection ROI. Keys are sent through the IDE terminal, not the image.
RECORDING_ENABLED saves the clean ISP view as H.264 MP4 through a second YUV channel;
preview overlays are not recorded. Color features can also be saved as JSON.
TEST_SECONDS=0 keeps the preview running until stopped in the IDE.
"""

import math
import os
import time


PROBE_ONLY = False
CHECK_CHIP_ID = False
I2C_ADDRESS = 0x30
USE_VGA_MODE = False
NATIVE_FULL_FRAME = False        # Full mode: 480x640 scaled, or 1080x1280 (PITCH_FIX firmware).
PREVIEW_ENABLED = True
PREVIEW_FPS = 60                 # Upper limit for frames copied to the IDE display.
TEST_SECONDS = 1200
# Fixed settings: AE controls both automatic exposure and automatic sensor gain.
EXPOSURE_US = 300
ANALOG_GAIN = 5.0
FRAME_TIMEOUT_MS = 1000
AUTO_WHITE_BALANCE = False        # False: fixed profile WB gains, blue-violet under this light.
ISP_3DNR_ENABLED = False
RAW_CHECK_ENABLED = False
RAW_CHECK_FRAMES = 30            # Analysed RAW frames; Python statistics take a while per frame.
RAW_BLACK_LEVEL = None           # RAW code with the lens fully covered; None until measured.
OUTPUT_MONITOR_ENABLED = False   # Per-frame ISP output color; lowers received_fps.
DIAGNOSTICS_ENABLED = True
COLOR_SAMPLING_ENABLED = True
COLOR_ROI = None                 # Optional (x, y, width, height) in the preview image.
COLOR_SAMPLE_FRAMES = 30         # New live frames after selection, not a frozen IDE frame.
COLOR_PERCENTILES = (0.05, 0.95)
COLOR_THRESHOLD_MARGIN = (3, 5, 5)  # Extra tolerance for the L, a and b channels.
COLOR_RESULT_PATH = "/sdcard/sc132gs_color_sample.json"  # Empty string: console only.
RECORDING_ENABLED = False
RECORDING_DIR = "/sdcard/sc132gs_recordings"  # A new sc132gs_NNNN.mp4 for each run.
RECORDING_FPS = 30              # Hardware encoder target, independent of PREVIEW_FPS.
RECORDING_BITRATE_KBPS = 4000
RECORDING_QUEUE_FRAMES = 6      # Includes the frame currently being written.
RECORDING_FRAME_BYTES = 256 * 1024  # Fixed slot capacity; oversized frames are dropped.
RECORDING_STOP_TIMEOUT_MS = 1000  # Bounded wait; the writer owns any unfinished I/O.


# ---------------- Python lamp / armor detection ----------------
# All coordinates are in output_size(), including the portrait 480x640 mode.
# Lab thresholds operate directly on RGB565 frames; geometry and tracking consume color blobs.
DETECTION_ENABLED = True
DETECTION_ROI = None                    # Optional fixed (x, y, width, height); preview clicks replace it.
# A profile may contain several CanMV Lab (L0,L1,a0,a1,b0,b1) intervals.
# These broad initial profiles must be calibrated for the actual lamps and ISP.
DETECTION_COLOR_THRESHOLDS = {
    "green": ((56, 96, -83, -43, 7, 47),),
    "red": ((15, 100, 20, 127, 0, 127),),
    "blue":  ((50, 80, -20, 10, -70, 10),),
}
DETECTION_LIGHT_COLORS = ("green",)
DETECTION_ARMOR_COLORS = ("green", "red", "blue")  # Both bars must match the same profile.
COLOR_SAMPLE_LABEL = "green"            # Label recorded with the sampled Lab interval.
COLOR_SAMPLE_APPLY_TO_DETECTION = False  # True: apply each completed sample to this label.
DETECTOR_CONFIG = {
    "workspace.max_candidates": 256,     # Overflow is an error; no silent truncation.
    "color.min_chroma": 10.0,            # sqrt(a*a+b*b); white highlights are not color proof.
    "light.min_area": 3,
    "light.far_area_threshold": 30,
    "light.far_size_px": 8,
    "light.search_step": 2,              # Native find_blobs seed stride; ROI scans use 1.
    "light.merge_margin_px": 2,          # Merge nearby green fragments; 0 disables merging.
    "light.body_l_margin": 15,          # Expand color seeds to the illuminated lamp body.
    "light.body_ab_margin": 20,
    "light.body_neutral_chroma": 8,      # Include the desaturated / white lamp core.
    "light.body_min_green_fraction": 0.02,
    "light.confirm_frames": 2,
    "light.far_confirm_frames": 3,
    "light.far_jump_k": 1.5,
    "light.far_jump_min_px": 6,
    "light.deadband_px": 2,
    "light.min_fill": 0.5,
    "light.min_circularity": 0.45,         # All lamp sizes; corrected axis ratio and disk-fill similarity.
    "light.circularity_weight": 5.0,     # Linear shape bonus after the circularity gate.
    "light.aspect_min": 0.7,
    "light.aspect_max": 1.4,
    "light.max_area_fraction": 0.25,
    "armor.length_floor": 2,
    "armor.area_floor": 2,
    "armor.length_min_k": 0.2,
    "armor.length_max_k": 8.0,
    "armor.tiny_px": 8.0,
    "armor.aspect_min": 2.0,
    "armor.fill_min": 0.5,
    "armor.flat_deg": 12.0,
    "armor.parallel_deg": 15.0,
    "armor.thickness_ratio": 0.3,
    "armor.length_ratio": 0.4,
    "armor.overlap": 0.4,
    "armor.gap_max_k": 10.0,
    "armor.diagonal_min_sin": 0.30,
    "armor.border_penalty": 0.0,
    "armor.window_width_k": 4.0,
    "armor.window_height_k": 3.0,
    "armor.window_gap_k": 0.0,
    "armor.span_min_k": 0.2,
    "armor.span_max_k": 12.0,
    "armor.plate_up_lo_pct": 0,
    "armor.plate_up_hi_pct": 1200,
    "armor.plate_offx_pct": 600,
    "armor.association_k": 0.6,
    "armor.association_min_px": 4.0,
    "armor.track_hold": 2,
    "armor.confirm_frames": 2,
    "armor.max_lab_distance": 25.0,      # Maximum Lab distance between the two bars.
    "armor.max_gap_color_fraction": 0.4, # At most 3 of 9 gap samples may match the bar color.
    "track.association_k": 1.5,
    "track.association_min_px": 6.0,
    "track.min_size_ratio": 0.5,
    "track.min_color_ratio": 0.35,       # Current / previous color-quality similarity.
    "track.max_lab_distance": 30.0,
    "track.validation_interval": 8,
    "track.quality_drop_ratio": 0.65,
    "track.velocity_alpha": 0.5,
    "track.prediction_max_ms": 50,
    "roi.min_width": 24,
    "roi.min_height": 64,
    "roi.size_k": 2.6,
    "roi.margin": 12.0,
    "roi.grow_per_miss": 0.5,
    "roi.full_area_fraction": 0.75,
    "roi.reset_after_misses": 3,
    "roi.full_scan_interval": 0,
    "roi.max_gap": 250,                  # Real receive-time interval, in milliseconds.
}
# Native find_blobs owns its extraction scratch. The script retains bounded candidate
# lists; grayscale thresholds, contrast, SNR and grayscale stability are not used.
DRAW_DETECTION = True
DRAW_ROI = True
DRAW_CANDIDATES = True                  # Green-lamp diagnostics and blue armor-bar candidates.
DETECTION_DEBUG_MAX_CANDIDATES = 5
DETECTION_DEBUG_GREEN_MIN_L = 70        # Keep midtone backgrounds out of bright-lamp diagnostics.
DETECTION_DEBUG_INTERVAL_MS = 500        # Refresh supplemental diagnostics at most twice per second.
DETECTION_DEBUG_BUDGET_MS = 20           # Per diagnostic color; native find_blobs cannot be interrupted.
LIGHT_COLOR = (0, 255, 0)
ARMOR_COLOR = (255, 0, 0)
ROI_COLOR = (0, 128, 255)
ARMOR_ROI_COLOR = (255, 0, 255)
DETECTION_LABEL_COLORS = {"green": (0, 255, 0), "red": (255, 64, 64), "blue": (64, 128, 255)}


# The ISP writes 1080-wide rows at a 1088-pixel pitch. The PITCH_FIX firmware sizes the
# buffers for it and packs rows before Python sees them; older firmware rejects 1080.
# On the board, 1080-wide output plus the display preview stalled the ISP after one
# frame (2026-11-13, pre-PITCH_FIX firmware), so the verified 480x640 path is the default.
NATIVE_FULL_SIZE = (1080, 1280)
# Whole view scaled by the ISP without cropping (aspect 0.84 shown as 0.75).
SCALED_FULL_SIZE = (480, 640)


def sensor_size():
    """Sensor mode resolution."""
    return (640, 480) if USE_VGA_MODE else (1080, 1280)


def output_size():
    """RGB565 frames delivered to Python, previewed and sampled."""
    if USE_VGA_MODE:
        return (640, 480)
    return NATIVE_FULL_SIZE if NATIVE_FULL_FRAME else SCALED_FULL_SIZE


def check_native_width_support():
    import mpp.vicap

    if not hasattr(mpp.vicap, "vb_mgmt_pack_vicap_frame"):
        raise RuntimeError("Native 1080x1280 needs the PITCH_FIX firmware (16-pixel row "
                           "pitch); flash it or set NATIVE_FULL_FRAME=False for 480x640")


def white_balance_mode():
    return "ISP AWB" if AUTO_WHITE_BALANCE else "disabled (ISP AWB and WB)"


class DisplayPreview:
    """IDE preview through the virtual display; VO writeback is JPEG-encoded in hardware.

    A software JPEG of 544x640 took about 0.13 s per frame on the board; showing a
    480x640 frame here took 4.5 ms on average (91 fps loop).
    """

    def __init__(self, width, height):
        from media.display import Display

        Display.init(Display.VIRT, width=width, height=height, fps=60, to_ide=True)
        self.display = Display

    def show_image(self, frame):
        self.display.show_image(frame)

    def deinit(self):
        self.display.deinit()


def capture_color_frame(sensor, timeout_ms=FRAME_TIMEOUT_MS):
    return sensor.snapshot(timeout=timeout_ms, auto_reboot=False)


def validate_config():
    for name, value in (("PROBE_ONLY", PROBE_ONLY),
                        ("CHECK_CHIP_ID", CHECK_CHIP_ID),
                        ("USE_VGA_MODE", USE_VGA_MODE),
                        ("PREVIEW_ENABLED", PREVIEW_ENABLED),
                        ("AUTO_WHITE_BALANCE", AUTO_WHITE_BALANCE),
                        ("ISP_3DNR_ENABLED", ISP_3DNR_ENABLED),
                        ("RAW_CHECK_ENABLED", RAW_CHECK_ENABLED),
                        ("OUTPUT_MONITOR_ENABLED", OUTPUT_MONITOR_ENABLED),
                        ("DIAGNOSTICS_ENABLED", DIAGNOSTICS_ENABLED),
                        ("COLOR_SAMPLING_ENABLED", COLOR_SAMPLING_ENABLED),
                        ("RECORDING_ENABLED", RECORDING_ENABLED),
                        ("DETECTION_ENABLED", DETECTION_ENABLED),
                        ("COLOR_SAMPLE_APPLY_TO_DETECTION", COLOR_SAMPLE_APPLY_TO_DETECTION),
                        ("DRAW_DETECTION", DRAW_DETECTION), ("DRAW_ROI", DRAW_ROI),
                        ("DRAW_CANDIDATES", DRAW_CANDIDATES)):
        if type(value) is not bool:
            raise ValueError("%s must be True or False" % name)
    for name, value, minimum in (("TEST_SECONDS", TEST_SECONDS, 0),
                                 ("PREVIEW_FPS", PREVIEW_FPS, 1),
                                 ("EXPOSURE_US", EXPOSURE_US, 1),
                                 ("FRAME_TIMEOUT_MS", FRAME_TIMEOUT_MS, 1),
                                 ("RAW_CHECK_FRAMES", RAW_CHECK_FRAMES, 2),
                                 ("COLOR_SAMPLE_FRAMES", COLOR_SAMPLE_FRAMES, 1)):
        if type(value) is not int or value < minimum:
            raise ValueError("%s must be an integer >= %d" % (name, minimum))
    if type(ANALOG_GAIN) not in (int, float) or not 1.0 <= ANALOG_GAIN <= 28.546875:
        raise ValueError("ANALOG_GAIN must be within 1.0..28.546875")
    if type(I2C_ADDRESS) is not int or I2C_ADDRESS not in range(0x30, 0x34):
        raise ValueError("I2C_ADDRESS must be within 0x30..0x33")
    if RAW_BLACK_LEVEL is not None and (type(RAW_BLACK_LEVEL) is not int
                                        or not 0 <= RAW_BLACK_LEVEL < 1023):
        raise ValueError("RAW_BLACK_LEVEL must be None or an integer within 0..1022")
    for name, roi in (("COLOR_ROI", COLOR_ROI), ("DETECTION_ROI", DETECTION_ROI)):
        if roi is None:
            continue
        if (not isinstance(roi, (tuple, list)) or len(roi) != 4
                or any(type(value) is not int for value in roi)):
            raise ValueError("%s must be None or (x, y, width, height)" % name)
        x, y, width, height = roi
        frame_width, frame_height = output_size()
        if (x < 0 or y < 0 or width < 4 or height < 4
                or x + width > frame_width or y + height > frame_height):
            raise ValueError("%s must be at least 4x4 and fit inside %dx%d" %
                             (name, frame_width, frame_height))
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
    if type(RECORDING_FPS) is not int or not 1 <= RECORDING_FPS <= 60:
        raise ValueError("RECORDING_FPS must be an integer within 1..60")
    if type(RECORDING_BITRATE_KBPS) is not int or not 64 <= RECORDING_BITRATE_KBPS <= 20000:
        raise ValueError("RECORDING_BITRATE_KBPS must be an integer within 64..20000")
    if type(RECORDING_QUEUE_FRAMES) is not int or not 1 <= RECORDING_QUEUE_FRAMES <= 8:
        raise ValueError("RECORDING_QUEUE_FRAMES must be an integer within 1..8")
    if (type(RECORDING_FRAME_BYTES) is not int or not 32768 <= RECORDING_FRAME_BYTES <= 524288
            or RECORDING_QUEUE_FRAMES * RECORDING_FRAME_BYTES > 4 * 1024 * 1024):
        raise ValueError("RECORDING_FRAME_BYTES must be within 32768..524288; queue <= 4 MiB")
    if type(RECORDING_STOP_TIMEOUT_MS) is not int or not 10 <= RECORDING_STOP_TIMEOUT_MS <= 5000:
        raise ValueError("RECORDING_STOP_TIMEOUT_MS must be an integer within 10..5000")
    if (not isinstance(RECORDING_DIR, str) or not RECORDING_DIR.startswith("/")
            or not RECORDING_DIR.strip("/") or "\x00" in RECORDING_DIR
            or len((RECORDING_DIR.rstrip("/") + "/sc132gs_9999.mp4").encode()) >= 128):
        raise ValueError("RECORDING_DIR must be an absolute directory; MP4 paths must fit in 127 bytes")
    if RECORDING_ENABLED and (PROBE_ONLY or RAW_CHECK_ENABLED):
        raise ValueError("RECORDING_ENABLED needs color capture; disable PROBE_ONLY and RAW_CHECK_ENABLED")
    validate_detection_config()


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

    def start(self, interactive=True):
        if PREVIEW_ENABLED and interactive:
            try:
                from machine import TOUCH

                self.touch_down = TOUCH.EVENT_DOWN
                width, height = output_size()
                self.touch = TOUCH(TOUCH.DEV_IDE, range_x=width, range_y=height)
            except (ImportError, AttributeError, OSError, RuntimeError) as error:
                if str(error) == "IDE interrupt":
                    raise
                print("[COLOR] IDE touch unavailable: %s; set COLOR_ROI to sample" % error)
        if self.touch is not None:
            print("[COLOR] Click two opposite corners in the IDE preview; "
                  "the next pair starts a new sample")
        if interactive and self.roi is None and self.touch is None:
            print("[COLOR] Waiting for a configured COLOR_ROI; preview/capture continues")
        print("[COLOR] Sampling %dx%d ISP output with fixed exposure/gain; white balance: %s" %
              (output_size() + (white_balance_mode(),)))

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
        frame_width, frame_height = output_size()
        point = (max(0, min(frame_width - 1, x)), max(0, min(frame_height - 1, y)))
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

    def poll(self):
        if self.touch is not None:
            for point in self.touch.read(5):
                # The IDE sends DOWN/UP pairs for a click. Count each click once.
                if point.event == self.touch_down:
                    self.select_point(point.x, point.y)

    def sampling(self):
        return self.roi is not None and self.samples < COLOR_SAMPLE_FRAMES

    def process(self, frame, frame_index):
        self.poll()
        self.sample(frame, frame_index)

    def sample(self, frame, frame_index):
        if not self.sampling():
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
            "sensor_mode": "vga" if USE_VGA_MODE else "full", "frame_size": output_size(),
            "sensor_size": sensor_size(),
            "pixel_format": "RGB565", "color_space": "CanMV Lab",
            "color_label": COLOR_SAMPLE_LABEL,
            "roi": self.roi, "sample_frames": self.samples,
            "sample_pixels": self.samples * self.roi[2] * self.roi[3],
            "first_frame": self.first_frame, "last_frame": frame_index,
            "elapsed_ms": time.ticks_diff(time.ticks_ms(), self.started),
            "requested_exposure_us": EXPOSURE_US, "requested_analog_gain": ANALOG_GAIN,
            "white_balance": white_balance_mode(),
            "lab_mean": mean, "lab_stdev": stdev,
            "percentiles": COLOR_PERCENTILES,
            "percentile_envelope_low": self.low, "percentile_envelope_high": self.high,
            "threshold_margin": COLOR_THRESHOLD_MARGIN, "lab_threshold": threshold,
        }
        print("[COLOR] complete: roi=%s frames=%d Lab mean=%s stdev=%s" %
              (self.roi, self.samples, mean, stdev))
        print("COLOR_THRESHOLD = %s  # (L_min, L_max, a_min, a_max, b_min, b_max)" %
              (tuple(threshold),))
        print("DETECTION_COLOR_THRESHOLDS[%r] = (%r,)" %
              (COLOR_SAMPLE_LABEL, tuple(threshold)))
        print("[COLOR] result_json=%s" % json.dumps(self.result))
        if COLOR_RESULT_PATH:
            try:
                with open(COLOR_RESULT_PATH, "w") as output:
                    json.dump(self.result, output)
                print("[COLOR] saved: %s" % COLOR_RESULT_PATH)
            except OSError as error:
                print("[COLOR] Save failed: %s; use result_json from the console" % error)

    def draw(self, frame, interactive=True):
        if self.corner is not None:
            frame.draw_cross(self.corner[0], self.corner[1], color=(255, 255, 0))
            text = "Click the opposite corner"
        elif self.roi is not None:
            color = (255, 255, 255) if self.result is not None else (255, 255, 0)
            frame.draw_rectangle(self.roi, color=color, thickness=2)
            frame.draw_string_advanced(self.roi[0] + 2, self.roi[1] + 2, 14, "取色选区", color=color)
            if self.result is not None:
                text = "Lab mean: %.1f %.1f %.1f" % tuple(self.result["lab_mean"])
                if interactive:
                    text += "; click twice to resample"
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


class PreviewControls:
    """One IDE touch owner; detection and color selection never consume the same click."""

    def __init__(self, detector, sampler=None):
        self.detector, self.sampler = detector, sampler
        self.width, self.height = detector.width, detector.height
        self.mode = "roi"
        self.corner = None
        self.touch = None
        self.poller = self.stdin = None

    def start(self):
        try:
            from machine import TOUCH

            self.touch_down = TOUCH.EVENT_DOWN
            self.touch = TOUCH(TOUCH.DEV_IDE, range_x=self.width, range_y=self.height)
        except (ImportError, AttributeError, OSError, RuntimeError) as error:
            if str(error) == "IDE interrupt":
                raise
            print("[INPUT] IDE touch unavailable: %s; set DETECTION_ROI / COLOR_ROI" % error)
        try:
            import select
            import sys

            poller = select.poll()
            poller.register(sys.stdin, select.POLLIN)
            self.poller, self.stdin = poller, sys.stdin
            self.pollin = select.POLLIN
            self.poll_errors = getattr(select, "POLLHUP", 0) | getattr(select, "POLLERR", 0)
        except (ImportError, AttributeError, OSError, RuntimeError, ValueError, TypeError) as error:
            if str(error) == "IDE interrupt":
                raise
            print("[INPUT] Terminal keys unavailable: %s; use the preview buttons" % error)
        print("[INPUT] Default: click two detection ROI corners (leave a margin around the target). "
              "Preview buttons / terminal keys: r=ROI, c=color, f=full frame; x=cancel corners")

    def cancel_corners(self):
        self.corner = None
        if self.sampler is not None:
            self.sampler.corner = None

    def command(self, key):
        key = key.lower()
        if key == "c" and self.sampler is None:
            print("[INPUT] Color sampling disabled; set COLOR_SAMPLING_ENABLED=True")
            return
        if key not in ("r", "c", "f", "x"):
            return
        self.cancel_corners()
        if key == "f":
            self.detector.set_search_roi(None)
            self.mode = "roi"
        elif key != "x":
            self.mode = "color" if key == "c" else "roi"
        print("[INPUT] mode=%s; click two opposite corners%s" %
              (self.mode, "; detection ROI cleared" if key == "f" else ""))

    def buttons(self):
        width = min(120, (self.width - 24) // 3)
        y = self.height - 54
        return tuple((key, (8 + index * (width + 4), y, width, 26), text)
                     for index, (key, text) in enumerate((("r", "R 检测ROI"),
                                                         ("c", "C 取色"), ("f", "F 全图"))))

    def select_point(self, x, y):
        for key, (bx, by, bw, bh), _ in self.buttons():
            if bx <= x < bx + bw and by <= y < by + bh:
                self.command(key)
                return
        if self.mode == "color":
            self.sampler.select_point(x, y)
            return
        point = (max(0, min(self.width - 1, x)), max(0, min(self.height - 1, y)))
        if self.corner is None:
            self.corner = point
            print("[INPUT] Detection ROI first corner=%s; click the opposite corner" % (point,))
            return
        x0, y0 = self.corner
        width, height = abs(point[0] - x0) + 1, abs(point[1] - y0) + 1
        if width < 4 or height < 4:
            print("[INPUT] Detection ROI must be at least 4x4; choose the opposite corner again")
            return
        self.detector.set_search_roi((min(x0, point[0]), min(y0, point[1]), width, height))
        self.corner = None

    def poll(self):
        if self.poller is not None:
            # Bound input work per frame, and never wait for a character or newline.
            for _ in range(16):
                ready = self.poller.poll(0)
                if any(event[1] & self.poll_errors for event in ready):
                    self.poller.unregister(self.stdin)
                    self.poller = self.stdin = None
                    print("[INPUT] Terminal closed; use the preview buttons")
                    break
                if not ready or not any(event[1] & self.pollin for event in ready):
                    break
                key = self.stdin.read(1)
                if not key:
                    break
                self.command(key)
        if self.touch is not None:
            for point in self.touch.read(5):
                if point.event == self.touch_down:
                    self.select_point(point.x, point.y)

    def draw(self, frame):
        if self.touch is None:
            return
        for key, bounds, text in self.buttons():
            selected = (key == "r" and self.mode == "roi") or (key == "c" and self.mode == "color")
            frame.draw_rectangle(bounds, color=(0, 0, 0), fill=True)
            color = (255, 255, 0) if selected else (220, 220, 220)
            frame.draw_rectangle(bounds, color=color, thickness=1)
            frame.draw_string_advanced(bounds[0] + 4, bounds[1] + 4, 16, text, color=color)
        if self.corner is not None:
            frame.draw_cross(self.corner[0], self.corner[1], color=(255, 255, 0), size=10)
            frame.draw_string_advanced(8, 104, 16, "检测ROI: 点击另一对角", color=(255, 255, 0))

    def deinit(self):
        try:
            if self.poller is not None:
                self.poller.unregister(self.stdin)
        finally:
            self.poller = self.stdin = None
            if self.touch is not None:
                self.touch.deinit()
                self.touch = None


def validate_detection_config():
    counts = (
        "workspace.max_candidates", "light.min_area", "light.far_area_threshold", "light.far_size_px",
        "light.search_step", "light.merge_margin_px", "light.body_l_margin", "light.body_ab_margin",
        "light.body_neutral_chroma", "light.confirm_frames", "light.far_confirm_frames",
        "light.far_jump_min_px", "light.deadband_px", "armor.length_floor", "armor.area_floor",
        "armor.plate_up_lo_pct", "armor.plate_up_hi_pct", "armor.plate_offx_pct",
        "armor.track_hold", "armor.confirm_frames", "track.validation_interval",
        "track.prediction_max_ms", "roi.min_width", "roi.min_height", "roi.reset_after_misses",
        "roi.full_scan_interval", "roi.max_gap",
    )
    for name, value in DETECTOR_CONFIG.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError("DETECTOR_CONFIG[%s] must be finite and nonnegative" % name)
        if name in counts and type(value) is not int:
            raise ValueError("DETECTOR_CONFIG[%s] must be an integer" % name)
    positive = (
        "workspace.max_candidates", "light.min_area", "light.far_area_threshold", "light.far_size_px",
        "light.search_step", "light.confirm_frames", "light.far_confirm_frames", "light.aspect_min",
        "light.aspect_max", "light.max_area_fraction", "armor.length_floor", "armor.area_floor",
        "armor.length_min_k", "armor.length_max_k", "armor.tiny_px", "armor.aspect_min",
        "armor.parallel_deg", "armor.gap_max_k", "armor.diagonal_min_sin", "armor.window_width_k",
        "armor.window_height_k", "armor.span_min_k", "armor.span_max_k",
        "armor.plate_offx_pct", "armor.association_k", "armor.association_min_px", "armor.track_hold",
        "armor.confirm_frames", "armor.max_lab_distance", "track.association_k",
        "track.association_min_px", "track.min_size_ratio", "track.max_lab_distance",
        "track.validation_interval", "track.prediction_max_ms", "roi.min_width", "roi.min_height",
        "roi.reset_after_misses", "roi.max_gap", "roi.full_area_fraction",
    )
    for name in positive:
        if DETECTOR_CONFIG[name] <= 0:
            raise ValueError("DETECTOR_CONFIG[%s] must be positive" % name)
    for name in ("light.min_fill", "light.min_circularity", "light.max_area_fraction",
                 "light.body_min_green_fraction", "armor.fill_min",
                 "armor.thickness_ratio", "armor.length_ratio", "armor.overlap", "armor.diagonal_min_sin",
                 "armor.max_gap_color_fraction", "track.min_size_ratio", "track.min_color_ratio",
                 "track.quality_drop_ratio", "track.velocity_alpha", "roi.full_area_fraction"):
        if DETECTOR_CONFIG[name] > 1:
            raise ValueError("DETECTOR_CONFIG[%s] must be within 0..1" % name)
    for lower, upper in (("light.aspect_min", "light.aspect_max"),
                         ("armor.length_min_k", "armor.length_max_k"),
                         ("armor.span_min_k", "armor.span_max_k"),
                         ("armor.plate_up_lo_pct", "armor.plate_up_hi_pct")):
        if DETECTOR_CONFIG[lower] > DETECTOR_CONFIG[upper]:
            raise ValueError("%s must be <= %s" % (lower, upper))
    if (DETECTOR_CONFIG["workspace.max_candidates"] > 256
            or DETECTOR_CONFIG["light.search_step"] > 8
            or DETECTOR_CONFIG["armor.flat_deg"] > 45
            or DETECTOR_CONFIG["armor.parallel_deg"] >= 90
            or DETECTOR_CONFIG["track.prediction_max_ms"] > 1000 or DETECTOR_CONFIG["roi.size_k"] < 1):
        raise ValueError("Detector capacity, angle or ROI configuration is out of range")
    if type(DETECTION_DEBUG_MAX_CANDIDATES) is not int or not 0 <= DETECTION_DEBUG_MAX_CANDIDATES <= 32:
        raise ValueError("DETECTION_DEBUG_MAX_CANDIDATES must be an integer within 0..32")
    if type(DETECTION_DEBUG_GREEN_MIN_L) is not int or not 0 <= DETECTION_DEBUG_GREEN_MIN_L <= 100:
        raise ValueError("DETECTION_DEBUG_GREEN_MIN_L must be an integer within 0..100")
    for name, value in (("DETECTION_DEBUG_INTERVAL_MS", DETECTION_DEBUG_INTERVAL_MS),
                        ("DETECTION_DEBUG_BUDGET_MS", DETECTION_DEBUG_BUDGET_MS)):
        if type(value) is not int or value <= 0:
            raise ValueError("%s must be a positive integer" % name)
    if not isinstance(DETECTION_COLOR_THRESHOLDS, dict) or not 1 <= len(DETECTION_COLOR_THRESHOLDS) <= 8:
        raise ValueError("DETECTION_COLOR_THRESHOLDS must contain 1..8 named Lab profiles")
    for label, intervals in DETECTION_COLOR_THRESHOLDS.items():
        if not isinstance(label, str) or not label or not isinstance(intervals, (tuple, list)):
            raise ValueError("Color profiles require a name and Lab intervals")
        if not 1 <= len(intervals) <= 4:
            raise ValueError("Each color profile must contain 1..4 Lab intervals")
        for interval in intervals:
            if (not isinstance(interval, (tuple, list)) or len(interval) != 6
                    or any(type(value) is not int for value in interval)):
                raise ValueError("Lab intervals must contain six integers")
            for index, lower, upper in ((0, 0, 100), (2, -128, 127), (4, -128, 127)):
                if not lower <= interval[index] <= interval[index + 1] <= upper:
                    raise ValueError("Invalid Lab interval for %s" % label)
    for labels in (DETECTION_LIGHT_COLORS, DETECTION_ARMOR_COLORS):
        if (not isinstance(labels, (tuple, list)) or not labels
                or any(label not in DETECTION_COLOR_THRESHOLDS for label in labels)):
            raise ValueError("Light and armor colors must name configured Lab profiles")
    if COLOR_SAMPLE_LABEL not in DETECTION_COLOR_THRESHOLDS:
        raise ValueError("COLOR_SAMPLE_LABEL must name a configured Lab profile")
    for color in (LIGHT_COLOR, ARMOR_COLOR, ROI_COLOR, ARMOR_ROI_COLOR) + tuple(DETECTION_LABEL_COLORS.values()):
        if (not isinstance(color, (tuple, list)) or len(color) != 3
                or any(type(value) is not int or not 0 <= value <= 255 for value in color)):
            raise ValueError("Detection drawing colors must contain three integers within 0..255")


class DetectionError(RuntimeError):
    """A script detector failure, separate from sensor / ISP diagnostics."""


class DetectionMotion:
    """Own measured history only; predictions are used for association, never publication."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.last = None
        self.received = None
        self.velocity = (0.0, 0.0)
        self.velocity_valid = False

    def observe(self, target, received):
        if self.last is not None:
            elapsed = time.ticks_diff(received, self.received)
            if elapsed > 0:
                x, y = target["center"]
                old_x, old_y = self.last["center"]
                vx, vy = (x - old_x) / elapsed, (y - old_y) / elapsed
                if self.velocity_valid:
                    alpha = self.config["track.velocity_alpha"]
                    vx = alpha * vx + (1 - alpha) * self.velocity[0]
                    vy = alpha * vy + (1 - alpha) * self.velocity[1]
                self.velocity = (vx, vy)
                self.velocity_valid = True
        self.last = {key: value for key, value in target.items() if key != "_blob"}
        self.received = received

    def predict(self, received, width, height):
        if self.last is None:
            return None
        target = dict(self.last)
        x, y, w, h = target["bounds"]
        dt = max(0, min(self.config["track.prediction_max_ms"],
                        time.ticks_diff(received, self.received)))
        limit = 4 * max(self.config["track.association_min_px"],
                        self.config["track.association_k"] * max(w, h))
        dx = max(-limit, min(limit, self.velocity[0] * dt))
        dy = max(-limit, min(limit, self.velocity[1] * dt))
        new_x = max(0, min(width - w, int(round(x + dx))))
        new_y = max(0, min(height - h, int(round(y + dy))))
        dx, dy = new_x - x, new_y - y
        target["bounds"] = (new_x, new_y, w, h)
        target["center"] = (target["center"][0] + dx, target["center"][1] + dy)
        if "bar_centers" in target:
            target["bar_centers"] = tuple((cx + dx, cy + dy) for cx, cy in target["bar_centers"])
        return target


class ColorTargetDetector:
    """RGB565/Lab detection, lamp / armor geometry and measured-target tracking.

    Native find_blobs extracts each configured color directly. All shape, color
    statistics and gap samples refer to that RGB565 frame, before any overlays.
    The detector borrows pixels only during process(); history owns numeric values.
    """

    def __init__(self, width, height):
        import image

        self.image = image
        self.config = dict(DETECTOR_CONFIG)
        self.thresholds = {label: tuple(tuple(interval) for interval in intervals)
                           for label, intervals in DETECTION_COLOR_THRESHOLDS.items()}
        self.width, self.height = width, height
        self.manual_roi = None
        sensor_width, sensor_height = sensor_size()
        self.shape_x_scale = sensor_width * height / (sensor_height * width)
        self.frame = None
        self.sample_result = None
        self.light_motion = DetectionMotion(self.config)
        self.armor_motion = DetectionMotion(self.config)
        self.previous_frame = None
        self.previous_received = None
        self.result = None
        self.debug_candidates = []
        self.debug_bars = []
        self.clear_debug_cache()
        self.reset_tracking()
        print("[DETECT] RGB565 Lab-only detector: %dx%d; lamp=%s armor=%s; shape_x_scale=%.3f" %
              (width, height, DETECTION_LIGHT_COLORS, DETECTION_ARMOR_COLORS, self.shape_x_scale))
        labels = {}
        for label in tuple(DETECTION_LIGHT_COLORS) + tuple(DETECTION_ARMOR_COLORS):
            labels[label] = True
        for label in labels:
            print("[DETECT] Lab %s=%s" % (label, self.thresholds[label]))
        if DETECTION_ROI is not None:
            self.set_search_roi(DETECTION_ROI)

    def set_search_roi(self, roi):
        if roi is not None:
            if (not isinstance(roi, (tuple, list)) or len(roi) != 4
                    or any(type(value) is not int for value in roi)):
                raise ValueError("Detection ROI must be (x, y, width, height)")
            x, y, w, h = roi
            if x < 0 or y < 0 or w < 4 or h < 4 or x + w > self.width or y + h > self.height:
                raise ValueError("Detection ROI must be at least 4x4 and fit inside the frame")
            roi = tuple(roi)
        if roi == self.manual_roi:
            return
        self.manual_roi = roi
        self.reset_tracking()
        self.clear_debug_cache()
        self.result = None
        print("[DETECT] manual_ROI=%s; %s; confirmation restarted" %
              (roi, "full-frame search restored" if roi is None else
               "lamp / armor searches stay inside this ROI, stride=1"))

    def search_bounds(self):
        return self.manual_roi if self.manual_roi is not None else (0, 0, self.width, self.height)

    def search_clip(self, rect):
        x, y, w, h = self.clip(rect)
        bx, by, bw, bh = self.search_bounds()
        x0, y0 = max(bx, min(x, bx + bw)), max(by, min(y, by + bh))
        x1, y1 = min(x + w, bx + bw), min(y + h, by + bh)
        return (x0, y0, max(0, x1 - x0), max(0, y1 - y0))

    def search_step(self, window, tracking=False):
        full = (0, 0, self.width, self.height)
        return 1 if tracking or self.manual_roi is not None or window != full else self.config["light.search_step"]

    def reset_tracking(self):
        self.light_motion.reset()
        self.armor_motion.reset()
        self.pending_light = None
        self.pending_armor = None
        self.light_hits = self.armor_hits = 0
        self.missed_frames = self.armor_misses = 0
        self.frames_since_full = self.tracking_frames = 0
        self.validated_light = self.validated_armor = None
        self.published_light = None

    def apply_sample(self, result):
        if not COLOR_SAMPLE_APPLY_TO_DETECTION or result is None or result is self.sample_result:
            return
        self.sample_result = result
        label = result.get("color_label", COLOR_SAMPLE_LABEL)
        self.thresholds[label] = (tuple(result["lab_threshold"]),)
        self.reset_tracking()
        self.clear_debug_cache()
        print("[DETECT] Applied sampled %s Lab=%s; confirmation restarted" %
              (label, tuple(result["lab_threshold"])))

    def close(self):
        self.reset_tracking()
        self.frame = self.result = self.sample_result = None
        self.debug_candidates = []
        self.debug_bars = []
        self.clear_debug_cache()

    def clear_debug_cache(self):
        self.debug_snapshot = []
        self.debug_checked_ms = None
        self.debug_received_ms = None
        self.debug_limited = False

    def clip(self, rect):
        x, y, w, h = rect
        x0 = max(0, min(self.width, int(math.floor(x))))
        y0 = max(0, min(self.height, int(math.floor(y))))
        x1 = max(x0, min(self.width, int(math.ceil(x + w))))
        y1 = max(y0, min(self.height, int(math.ceil(y + h))))
        return (x0, y0, x1 - x0, y1 - y0)

    @staticmethod
    def unite(a, b):
        if a[2] <= 0 or a[3] <= 0:
            return b
        if b[2] <= 0 or b[3] <= 0:
            return a
        x, y = min(a[0], b[0]), min(a[1], b[1])
        return (x, y, max(a[0] + a[2], b[0] + b[2]) - x,
                max(a[1] + a[3], b[1] + b[3]) - y)

    @staticmethod
    def overlap(a, b):
        return max(0, min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0])) * max(
            0, min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1]))

    @staticmethod
    def cross(a, b):
        return a[0] * b[1] - a[1] * b[0]

    @staticmethod
    def lab_distance(a, b):
        return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))

    def matches_lab(self, lab, label):
        for l0, l1, a0, a1, b0, b1 in self.thresholds[label]:
            if l0 <= lab[0] <= l1 and a0 <= lab[1] <= a1 and b0 <= lab[2] <= b1:
                return True
        return False

    def circularity(self, raw, fill):
        # CanMV roundness is a covariance eigenvalue ratio; the axis ratio is its
        # square root. Undo only the ISP aspect distortion for lamp shape tests.
        ratio = min(1.0, max(0.0, raw.roundness()))
        angle = raw.rotation_rad()
        co, si = math.cos(angle), math.sin(angle)
        xx = (co * co + ratio * si * si) * self.shape_x_scale ** 2
        yy = si * si + ratio * co * co
        xy = (1.0 - ratio) * co * si * self.shape_x_scale
        spread = math.sqrt((xx - yy) ** 2 + 4 * xy * xy)
        axis_ratio = math.sqrt(max(0.0, xx + yy - spread) / max(1e-9, xx + yy + spread))
        # A square also has equal covariance axes. Penalize both sparse regions
        # and regions that fill their rectangular bounds more than a disk does.
        disk_fill = math.pi * 0.25
        fill_score = min(1.0, fill / disk_fill, disk_fill / max(1e-9, fill))
        return axis_ratio * fill_score

    def candidate(self, raw, window, label, debug_thresholds=None):
        bounds = tuple(raw.rect())
        x, y, w, h = bounds
        pixels = raw.pixels()
        if w <= 0 or h <= 0 or pixels <= 0:
            return None
        thresholds = self.thresholds[label] if debug_thresholds is None else debug_thresholds
        stats = self.frame.get_statistics(roi=bounds, thresholds=thresholds)
        lab = (stats.l_mean(), stats.a_mean(), stats.b_mean())
        chroma = math.sqrt(lab[1] ** 2 + lab[2] ** 2)
        if chroma < self.config["color.min_chroma"] and debug_thresholds is None:
            self.counters["rejected_color"] += 1
            return None
        corners = tuple(tuple(point) for point in raw.min_corners())
        edges = [(corners[(index + 1) % 4][0] - corners[index][0],
                  corners[(index + 1) % 4][1] - corners[index][1]) for index in range(4)]
        lengths = [math.sqrt(dx * dx + dy * dy) for dx, dy in edges]
        longest = max(range(4), key=lambda index: lengths[index])
        length, thickness = lengths[longest] + 1.0, lengths[(longest + 1) % 4] + 1.0
        if lengths[longest] > 0:
            axis = (edges[longest][0] / lengths[longest], edges[longest][1] / lengths[longest])
        else:
            axis = (math.cos(raw.rotation_rad()), math.sin(raw.rotation_rad()))
        fill = min(1.0, pixels / (w * h))
        # Profile centrality and chroma form a tuning score, not a probability.
        centrality = 0.0
        for interval in self.thresholds[label]:
            if self.matches_interval(lab, interval):
                distance = max(abs(lab[index] - (interval[2 * index] + interval[2 * index + 1]) * 0.5)
                               / max(1.0, (interval[2 * index + 1] - interval[2 * index]) * 0.5)
                               for index in range(3))
                centrality = max(centrality, 1.0 - min(1.0, distance))
        quality = min(1.0, chroma / max(24.0, 2 * self.config["color.min_chroma"])) * (0.5 + 0.5 * centrality)
        return {"center": (raw.cxf(), raw.cyf()), "bounds": bounds, "area": pixels,
                "color": label, "lab_mean": lab, "lab_stdev": (stats.l_stdev(), stats.a_stdev(), stats.b_stdev()),
                "color_quality": quality, "fill": fill, "circularity": self.circularity(raw, fill),
                "length": length, "thickness": thickness, "axis": axis,
                "box_center": (sum(point[0] for point in corners) * 0.25,
                               sum(point[1] for point in corners) * 0.25),
                "oriented_fill": min(1.0, pixels / (length * thickness)),
                "clipped": x <= window[0] or y <= window[1]
                           or x + w >= window[0] + window[2] or y + h >= window[1] + window[3],
                "reason": "candidate"}

    @staticmethod
    def matches_interval(lab, interval):
        return all(interval[2 * index] <= lab[index] <= interval[2 * index + 1] for index in range(3))

    def scan(self, window, labels, step, kind):
        if window[2] <= 0 or window[3] <= 0:
            return []
        minimum = self.config["light.min_area"] if kind == "light" else self.config["armor.area_floor"]
        # Keep even sub-threshold seeds visible in a manual ROI. Selection still
        # requires the original minimum area and all body / shape / color gates.
        if kind == "light" and self.manual_roi is not None:
            minimum = 1
        candidates, seen = [], set()
        extracted = 0
        visited_labels = set()
        for label in labels:
            if label in visited_labels:
                continue
            visited_labels.add(label)
            merge_margin = self.config["light.merge_margin_px"] if kind == "light" and label == "green" else 0
            raw_blobs = self.frame.find_blobs(self.thresholds[label], roi=window, x_stride=step,
                                              y_stride=step, pixels_threshold=minimum,
                                              area_threshold=minimum, merge=merge_margin > 0,
                                              margin=merge_margin)
            extracted += len(raw_blobs)
            self.counters["raw_color_blobs"] += len(raw_blobs)
            if extracted > self.config["workspace.max_candidates"]:
                raise DetectionError("Lab candidate capacity exceeded; tighten color profiles or the search region")
            for index, raw in enumerate(raw_blobs):
                if index % 16 == 0:
                    os.exitpoint()
                identity = (label, tuple(raw.rect()), raw.pixels())
                if identity in seen:
                    continue
                seen.add(identity)
                blob = self.candidate(raw, window, label)
                if blob is None:
                    continue
                candidates.append(blob)
        self.counters["candidates"] += len(candidates)
        self.counters["%s_candidates" % kind] += len(candidates)
        return candidates

    def refine_green_lamps(self, candidates, window, step):
        # Narrow green components prove color; they do not define the lamp rim.
        # Recover the connected illuminated body before shape tests / tracking.
        seeds = [blob for blob in candidates if blob["color"] == "green"]
        refined = [blob for blob in candidates if blob["color"] != "green"]
        if not seeds:
            return refined
        c = self.config
        neutral = min(127, c["light.body_neutral_chroma"])
        thresholds = tuple((max(0, l0 - c["light.body_l_margin"]), 100,
                            max(-128, min(a0 - c["light.body_ab_margin"], -neutral)),
                            min(127, max(a1 + c["light.body_ab_margin"], neutral)),
                            max(-128, min(b0 - c["light.body_ab_margin"], -neutral)),
                            min(127, max(b1 + c["light.body_ab_margin"], neutral)))
                           for l0, l1, a0, a1, b0, b1 in self.thresholds["green"])
        margin = c["light.merge_margin_px"]
        bodies = self.frame.find_blobs(thresholds, roi=window, x_stride=step, y_stride=step,
                                       pixels_threshold=c["light.min_area"],
                                       area_threshold=c["light.min_area"],
                                       merge=margin > 0, margin=margin)
        if len(bodies) > c["workspace.max_candidates"]:
            raise DetectionError("Lamp body capacity exceeded; tighten the body Lab expansion")
        maximum_area = self.width * self.height * c["light.max_area_fraction"]
        for seed in seeds:
            seed["reason"] = "body_unresolved"
        for index, raw in enumerate(bodies):
            if index % 16 == 0:
                os.exitpoint()
            bounds = tuple(raw.rect())
            x, y, w, h = bounds
            pixels = raw.pixels()
            if w <= 0 or h <= 0 or pixels <= 0:
                continue
            contained = [seed for seed in seeds
                         if self.overlap(seed["bounds"], bounds)
                         == seed["bounds"][2] * seed["bounds"][3]]
            if not contained:
                continue
            for seed in contained:
                seed["reason"] = "body_seed"
            color_seed = max(contained, key=lambda seed: seed["area"])
            fill = min(1.0, pixels / (w * h))
            clipped = (x <= window[0] or y <= window[1]
                       or x + w >= window[0] + window[2] or y + h >= window[1] + window[3])
            # Numeric diagnostics first: never histogram a large background just
            # because its bounds happen to contain a small green component.
            body = {"bounds": bounds, "area": pixels, "color": "green",
                    "lab_mean": color_seed["lab_mean"], "color_quality": color_seed["color_quality"],
                    "fill": fill, "circularity": self.circularity(raw, fill), "clipped": clipped}
            fraction = min(1.0, sum(seed["area"] for seed in contained) / pixels)
            if w * h > maximum_area:
                reason = "area"
            elif clipped:
                reason = "body_clipped"
            elif fraction < c["light.body_min_green_fraction"]:
                reason = "body_color"
            else:
                reason = self.light_rejection(body)
            if reason is not None:
                body["reason"] = reason
                self.counters["rejected_light_body"] += 1
                self.remember_debug_candidate(body)
                continue
            # Geometry comes from the broad body, Lab statistics still use only
            # the original green profile. White pixels never prove lamp color.
            body = self.candidate(raw, window, "green")
            if body is None:
                continue
            body["green_fraction"] = fraction
            refined.append(body)
            self.counters["light_body_candidates"] += 1
        for seed in seeds:
            if seed["reason"] == "body_unresolved":
                if seed["area"] < c["light.min_area"]:
                    seed["reason"] = "small"
                self.remember_debug_candidate(seed)
        # No fallback to an internal color fragment: it would recreate the same
        # incorrect diameter and put the armor search window inside the lamp.
        return refined

    def light_target(self, blob):
        # Geometry follows the lamp's outer extent, not the number / centroid of
        # pixels that happen to pass the color threshold in this frame.
        x, y, w, h = blob["bounds"]
        return {"center": (x + w * 0.5, y + h * 0.5), "bounds": blob["bounds"], "area": blob["area"],
                "radius": 0.5 * math.sqrt(w * h),
                "quality": blob["color_quality"] * blob["circularity"] * (0.25 if blob["clipped"] else 1.0),
                "clipped": blob["clipped"], "color": blob["color"], "lab_mean": blob["lab_mean"],
                "color_quality": blob["color_quality"], "fill": blob["fill"],
                "circularity": blob["circularity"], "_blob": blob}

    def far(self, target):
        return (target["area"] < self.config["light.far_area_threshold"]
                or max(target["bounds"][2:]) < self.config["light.far_size_px"])

    def match_cost(self, target, predicted, reference):
        if target is None or predicted is None or reference is None or target["color"] != reference["color"]:
            return None
        wr, hr = target["bounds"][2] / reference["bounds"][2], target["bounds"][3] / reference["bounds"][3]
        minimum = self.config["track.min_size_ratio"]
        lo = min(target["color_quality"], reference["color_quality"])
        hi = max(target["color_quality"], reference["color_quality"])
        lab_distance = self.lab_distance(target["lab_mean"], reference["lab_mean"])
        if (not minimum <= wr <= 1 / minimum or not minimum <= hr <= 1 / minimum
                or lo < self.config["track.min_color_ratio"] * hi
                or lab_distance > self.config["track.max_lab_distance"]):
            return None
        radius = max(self.config["track.association_min_px"],
                     self.config["track.association_k"] * max(reference["bounds"][2:]))
        dx, dy = target["center"][0] - predicted["center"][0], target["center"][1] - predicted["center"][1]
        position = (dx * dx + dy * dy) / (radius * radius)
        if position > 1:
            return None
        return position + 0.1 * (abs(1 - wr) + abs(1 - hr)) + 0.1 * lab_distance / self.config["track.max_lab_distance"]

    def light_score(self, blob):
        # The shape gate already rejects non-circular lamps. Raising circularity
        # to a high power favored tiny, round fragments over the whole lamp.
        shape_bonus = 1.0 + self.config["light.circularity_weight"] * blob["circularity"]
        return blob["area"] * shape_bonus * max(0.05, blob["color_quality"])

    def light_rejection(self, blob, check_color=False):
        if check_color:
            lab = blob["lab_mean"]
            if (not self.matches_lab(lab, "green")
                    or math.sqrt(lab[1] ** 2 + lab[2] ** 2) < self.config["color.min_chroma"]):
                return "color"
        if blob["area"] < self.config["light.min_area"]:
            return "small"
        if blob["area"] > self.width * self.height * self.config["light.max_area_fraction"]:
            return "area"
        # Small / far lamps must also pass circularity before selection or tracking.
        if blob["circularity"] < self.config["light.min_circularity"]:
            return "circularity"
        if not self.far(blob):
            if blob["fill"] < self.config["light.min_fill"]:
                return "fill"
            w, h = blob["bounds"][2:]
            if not self.config["light.aspect_min"] <= w * self.shape_x_scale / h <= self.config["light.aspect_max"]:
                return "aspect"
        return None

    def bar_score(self, blob):
        slenderness = max(0.05, 1.0 - blob["thickness"] / max(1.0, blob["length"]))
        return blob["area"] * slenderness * max(0.05, blob["color_quality"])

    def remember_debug_candidate(self, blob, candidates=None, role="light"):
        expected_color = "blue" if role == "armor" else "green"
        if blob["color"] != expected_color or not DETECTION_DEBUG_MAX_CANDIDATES:
            return
        if candidates is None:
            candidates = self.debug_bars if role == "armor" else self.debug_candidates
        # Keep the actual detector's reason when a broad diagnostic blob has the same bounds.
        if any(previous["bounds"] == blob["bounds"] for previous in candidates):
            return
        blob["role"] = role
        candidates.append(blob)
        candidates.sort(key=self.bar_score if role == "armor" else self.light_score, reverse=True)
        del candidates[DETECTION_DEBUG_MAX_CANDIDATES:]

    def debug_raw_score(self, raw, role="light"):
        # Cheap shortlist only; corrected circularity and Lab are evaluated later.
        if role == "armor":
            return raw.pixels() * max(0.05, 1.0 - min(1.0, raw.roundness()))
        axis_ratio = math.sqrt(min(1.0, max(0.0, raw.roundness())))
        return raw.pixels() * (1.0 + self.config["light.circularity_weight"] * axis_ratio)

    def debug_shortlist(self, window, thresholds, role):
        step = 1 if self.manual_roi is not None or window != (0, 0, self.width, self.height) else 2
        raw_blobs = self.frame.find_blobs(thresholds, roi=window, x_stride=step, y_stride=step,
                                          pixels_threshold=1, area_threshold=1, merge=False)
        started = time.ticks_ms()  # Budget candidate processing after the native extraction.
        capacity = self.config["workspace.max_candidates"]
        limited = len(raw_blobs) > capacity
        shortlist = []
        shortlist_size = min(capacity, 2 * DETECTION_DEBUG_MAX_CANDIDATES)
        maximum_area = self.width * self.height * self.config["light.max_area_fraction"]
        current = self.debug_bars if role == "armor" else self.debug_candidates
        for index, raw in enumerate(raw_blobs):
            if index >= capacity:
                break
            if index % 16 == 0:
                os.exitpoint()
                if time.ticks_diff(time.ticks_ms(), started) >= DETECTION_DEBUG_BUDGET_MS:
                    limited = True
                    break
            _, _, w, h = raw.rect()
            # Huge background components and sparse, huge bounding boxes caused
            # repeated full-frame histograms. Actual green rejects are kept above.
            if raw.pixels() > maximum_area or w * h > maximum_area:
                continue
            if any(previous["bounds"] == tuple(raw.rect()) for previous in current):
                continue
            score = self.debug_raw_score(raw, role)
            if len(shortlist) >= shortlist_size and score <= shortlist[-1][0]:
                continue
            shortlist.append((score, raw))
            shortlist.sort(key=lambda entry: entry[0], reverse=True)
            del shortlist[shortlist_size:]
        return shortlist, limited, started

    def collect_debug_green(self, window, tracking, predicted):
        # Supplemental components never enter target selection or confirmation.
        thresholds = ((DETECTION_DEBUG_GREEN_MIN_L, 100, -128, 127, -128, 127),)
        if self.manual_roi is not None:
            # Far lamps may be dimmer than the full-frame diagnostic L floor.
            # Widen only diagnostics, within the selected area; retain Lab gates.
            # Retain bright neutral highlights too: an overexposed far lamp
            # should explain its missing green evidence instead of disappearing.
            thresholds += tuple((0, 100, max(-128, a0 - 20), min(127, a1 + 20),
                                 max(-128, b0 - 20), min(127, b1 + 20))
                                for _, _, a0, a1, b0, b1 in self.thresholds["green"])
        shortlist, limited, started = self.debug_shortlist(window, thresholds, "light")
        diagnostics = []
        for _, raw in shortlist:
            os.exitpoint()
            if time.ticks_diff(time.ticks_ms(), started) >= DETECTION_DEBUG_BUDGET_MS:
                limited = True
                break
            blob = self.candidate(raw, window, "green", debug_thresholds=thresholds)
            if blob is None:
                continue
            reason = self.light_rejection(blob, check_color=True)
            if reason is None and tracking:
                if self.match_cost(self.light_target(blob), predicted, self.light_motion.last) is None:
                    reason = "association"
            # A broad component can differ from the native green component; passing
            # its checks is a diagnostic hint, not evidence of a confirmation hit.
            blob["reason"] = reason or "diagnostic"
            self.remember_debug_candidate(blob, diagnostics)
        return diagnostics, limited

    def blue_debug_reason(self, blob, light, armor_roi=None):
        lab = blob["lab_mean"]
        if (not self.matches_lab(lab, "blue")
                or math.sqrt(lab[1] ** 2 + lab[2] ** 2) < self.config["color.min_chroma"]):
            return "color"
        scale = None if light is None else 2.0 * light["radius"]
        reason = self.bar_rejection(blob, scale)
        if reason is not None:
            return reason
        if light is None:
            return "no_lamp"
        _, _, w, h = blob["bounds"]
        if armor_roi is None:
            armor_roi = self.armor_window(light)
        if self.overlap(blob["bounds"], armor_roi) < w * h:
            return "armor_roi"
        return "bar_eligible"

    def collect_debug_blue(self, light, armor_roi=None):
        # Search independently of the green anchor so missing / rejected bars are
        # visible too. Exact configured blue bounds are still checked afterward.
        # Keep this scan near the blue profiles so white lights and other colors
        # cannot occupy the whole bounded shortlist. Do not widen detection itself.
        thresholds = tuple((max(0, l0 - 10), 100, max(-128, a0 - 20), min(127, a1 + 20),
                            max(-128, b0 - 20), min(127, b1 + 20))
                           for l0, l1, a0, a1, b0, b1 in self.thresholds["blue"])
        window = self.search_bounds()
        shortlist, limited, started = self.debug_shortlist(window, thresholds, "armor")
        diagnostics = []
        for _, raw in shortlist:
            os.exitpoint()
            if time.ticks_diff(time.ticks_ms(), started) >= DETECTION_DEBUG_BUDGET_MS:
                limited = True
                break
            blob = self.candidate(raw, window, "blue", debug_thresholds=thresholds)
            if blob is None:
                continue
            blob["reason"] = self.blue_debug_reason(blob, light, armor_roi)
            self.remember_debug_candidate(blob, diagnostics, role="armor")
        return diagnostics, limited

    def update_debug_candidates(self, result, tracking, predicted, received, preview_due):
        result["debug_limited"] = False
        result["debug_age_ms"] = None
        green_needed = (result["target"] is None and result["reason"] != "lamp_confirm"
                        and "green" in DETECTION_LIGHT_COLORS)
        blue_needed = result["armor"] is None and "blue" in DETECTION_ARMOR_COLORS
        if (not PREVIEW_ENABLED or not DRAW_CANDIDATES or not DETECTION_DEBUG_MAX_CANDIDATES
                or not (green_needed or blue_needed)):
            self.clear_debug_cache()
            return
        light = self.pending_light if result["reason"] == "lamp_confirm" else result["light"]
        armor_roi = result["armor_roi"]
        if self.debug_checked_ms is None:
            # Publish the first preview before attempting supplemental extraction.
            self.debug_checked_ms = time.ticks_ms()
        if preview_due and time.ticks_diff(time.ticks_ms(), self.debug_checked_ms) >= DETECTION_DEBUG_INTERVAL_MS:
            self.debug_snapshot = []
            self.debug_limited = False
            if blue_needed:
                bars, limited = self.collect_debug_blue(light, armor_roi)
                self.debug_snapshot.extend(bars)
                self.debug_limited = limited
            if green_needed:
                lamps, limited = self.collect_debug_green(result["roi"], tracking, predicted)
                self.debug_snapshot.extend(lamps)
                self.debug_limited = self.debug_limited or limited
            self.debug_received_ms = received
            self.debug_checked_ms = time.ticks_ms()
        if self.debug_received_ms is not None:
            age = max(0, time.ticks_diff(received, self.debug_received_ms))
            # Cache numeric features only, with a bounded lifetime and a visible age.
            if age <= DETECTION_DEBUG_INTERVAL_MS + DETECTION_DEBUG_BUDGET_MS:
                for blob in self.debug_snapshot:
                    if blob["role"] == "armor" and blue_needed:
                        blob = dict(blob)
                        blob["reason"] = self.blue_debug_reason(blob, light, armor_roi)
                        self.remember_debug_candidate(blob, role="armor")
                    elif blob["role"] == "light" and green_needed:
                        self.remember_debug_candidate(blob)
                result["debug_age_ms"] = age
                result["debug_limited"] = self.debug_limited

    def select_light(self, candidates, tracking, predicted):
        best, best_score = None, None
        for blob in candidates:
            reason = self.light_rejection(blob)
            if reason is not None:
                blob["reason"] = reason
                self.counters["rejected_light_area" if reason in ("small", "area") else "rejected_light_shape"] += 1
                continue
            target = self.light_target(blob)
            if tracking:
                score = self.match_cost(target, predicted, self.light_motion.last)
                if score is None:
                    blob["reason"] = "association"
                    self.counters["rejected_association"] += 1
                    continue
                previous_area = self.light_motion.last["area"]
                score += 1.0 - min(target["area"], previous_area) / max(target["area"], previous_area)
            else:
                score = -self.light_score(blob)
            blob["reason"] = "eligible"
            if best_score is None or score < best_score:
                best, best_score = target, score
        if best is not None:
            best["_blob"]["reason"] = "selected"
        for blob in candidates:
            self.remember_debug_candidate(blob)
        return best

    def armor_window(self, light, clip_to_frame=True):
        c = self.config
        x, y, lamp_w, _ = light["bounds"]
        diameter = 2 * light["radius"]
        w = c["armor.window_width_k"] * diameter
        h = c["armor.window_height_k"] * diameter
        # The lower edge is at the lamp's actual top, even for a sparse mask.
        # This geometry is also used by the visible lamp outline.
        window = (x + lamp_w * 0.5 - w * 0.5,
                  y - c["armor.window_gap_k"] * diameter - h, w, h)
        return self.search_clip(window) if clip_to_frame else window

    def bar_rejection(self, blob, scale=None):
        # Armor bars are filled rectangles. Lamp circularity is never a bar gate.
        c = self.config
        minimum = c["armor.length_floor"] if scale is None else max(
            c["armor.length_floor"], math.floor(c["armor.length_min_k"] * scale))
        maximum = None if scale is None else max(minimum, math.floor(c["armor.length_max_k"] * scale))
        length, thickness = blob["length"], blob["thickness"]
        if blob["area"] < max(c["armor.area_floor"], math.floor(minimum * 0.5)):
            return "bar_area"
        if max(blob["bounds"][2:]) >= c["armor.tiny_px"]:
            if length < minimum or (maximum is not None and length > maximum):
                return "bar_length"
        # Tiny bars may relax scale-dependent length, but must still be rectangular.
        if length < c["armor.aspect_min"] * thickness:
            return "bar_aspect"
        if blob["oriented_fill"] < c["armor.fill_min"]:
            return "bar_fill"
        return None

    def make_bar(self, blob, scale):
        c = self.config
        reason = self.bar_rejection(blob, scale)
        if reason is not None:
            blob["reason"] = reason
            self.counters["rejected_bar_shape"] += 1
            return None
        length, thickness = blob["length"], blob["thickness"]
        axis = blob["axis"]
        vertical = abs(axis[1]) >= abs(axis[0])
        if (vertical and axis[1] < 0) or (not vertical and axis[0] < 0):
            axis = (-axis[0], -axis[1])
        skew = math.atan2(min(abs(axis[0]), abs(axis[1])), max(abs(axis[0]), abs(axis[1]))) * 180 / math.pi
        endpoint_length = max(0.0, length - 1.0)
        if skew <= c["armor.flat_deg"]:
            axis = (0.0, 1.0) if vertical else (1.0, 0.0)
            endpoint_length = (blob["bounds"][3] if vertical else blob["bounds"][2]) - 1
        center = blob["box_center"]
        endpoints = ((center[0] - axis[0] * endpoint_length * 0.5,
                      center[1] - axis[1] * endpoint_length * 0.5),
                     (center[0] + axis[0] * endpoint_length * 0.5,
                      center[1] + axis[1] * endpoint_length * 0.5))
        blob["reason"] = "bar_eligible"
        return {"blob": blob, "center": center, "length": length, "thickness": thickness,
                "axis": axis, "endpoints": endpoints}

    def armor_center(self, a, b):
        a0, a1 = a["endpoints"]
        b0, b1 = b["endpoints"]
        if ((a1[0] - a0[0]) * (b1[0] - b0[0]) + (a1[1] - a0[1]) * (b1[1] - b0[1])) < 0:
            b0, b1 = b1, b0
        u, v = (b1[0] - a0[0], b1[1] - a0[1]), (b0[0] - a1[0], b0[1] - a1[1])
        offset = (a1[0] - a0[0], a1[1] - a0[1])
        determinant = self.cross(u, v)
        lengths = math.sqrt((u[0] ** 2 + u[1] ** 2) * (v[0] ** 2 + v[1] ** 2))
        if lengths <= 0 or abs(determinant) < self.config["armor.diagonal_min_sin"] * lengths:
            return None
        t, s = self.cross(offset, v) / determinant, self.cross(offset, u) / determinant
        if not 0 <= t <= 1 or not 0 <= s <= 1:
            return None
        return (a0[0] + t * u[0], a0[1] + t * u[1])

    def gap_color_fraction(self, a, b, separation):
        dx, dy = b["center"][0] - a["center"][0], b["center"][1] - a["center"][1]
        lo = min(0.45, a["thickness"] * 0.5 / separation)
        hi = 1.0 - min(0.45, b["thickness"] * 0.5 / separation)
        matched = 0
        for row in (-1, 0, 1):
            along = row * min(a["length"], b["length"]) * 0.25
            for col in (1, 2, 3):
                t = lo + (hi - lo) * col * 0.25
                x = max(0, min(self.width - 1, int(round(a["center"][0] + dx * t + a["axis"][0] * along))))
                y = max(0, min(self.height - 1, int(round(a["center"][1] + dy * t + a["axis"][1] * along))))
                rgb = self.frame.get_pixel(x, y, rgbtuple=True)
                lab = self.image.rgb_to_lab(rgb[0], rgb[1], rgb[2])
                if self.matches_lab(lab, a["blob"]["color"]):
                    matched += 1
        return matched / 9.0

    def same_bars(self, target, reference, tolerance):
        if target is None or reference is None:
            return False
        if "bar_centers" not in target or "bar_centers" not in reference:
            return True
        a, b = target["bar_centers"]
        old_a, old_b = reference["bar_centers"]
        def distance_squared(p, q):
            return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2
        direct = max(distance_squared(a, old_a), distance_squared(b, old_b))
        reverse = max(distance_squared(a, old_b), distance_squared(b, old_a))
        return min(direct, reverse) <= tolerance * tolerance

    def select_armor(self, candidates, light, predicted):
        c = self.config
        scale = 2.0 * light["radius"]
        bars = []
        for blob in candidates:
            if predicted is not None:
                margin = max(c["armor.association_min_px"], c["armor.association_k"] * max(predicted["bounds"][2:]))
                x, y, w, h = predicted["bounds"]
                if not self.overlap(blob["bounds"], (x - margin, y - margin, w + 2 * margin, h + 2 * margin)):
                    blob["reason"] = "association"
                    self.counters["rejected_association"] += 1
                    continue
            bar = self.make_bar(blob, scale)
            if bar is not None:
                bars.append(bar)
        parallel = math.cos(c["armor.parallel_deg"] * math.pi / 180)
        best, best_score = None, None
        for index, a in enumerate(bars):
            if index % 8 == 0:
                os.exitpoint()
            for other in range(index + 1, len(bars)):
                b = bars[other]
                ba, bb = a["blob"], b["blob"]
                if ba["color"] != bb["color"]:
                    self.counters["rejected_pair_color"] += 1
                    continue
                dot = a["axis"][0] * b["axis"][0] + a["axis"][1] * b["axis"][1]
                longest, shortest = max(a["length"], b["length"]), min(a["length"], b["length"])
                offset = (b["center"][0] - a["center"][0], b["center"][1] - a["center"][1])
                separation = abs(self.cross(offset, a["axis"]))
                along = abs(offset[0] * a["axis"][0] + offset[1] * a["axis"][1])
                if (abs(dot) < parallel or shortest < c["armor.length_ratio"] * longest
                        or min(a["thickness"], b["thickness"])
                           < c["armor.thickness_ratio"] * max(a["thickness"], b["thickness"])
                        or (a["length"] + b["length"]) * 0.5 - along < c["armor.overlap"] * shortest
                        or separation <= (a["thickness"] + b["thickness"]) * 0.5
                        or separation > c["armor.gap_max_k"] * longest
                        or not c["armor.span_min_k"] * scale <= separation <= c["armor.span_max_k"] * scale):
                    self.counters["rejected_pair_geometry"] += 1
                    continue
                center = self.armor_center(a, b)
                if center is None:
                    self.counters["rejected_pair_geometry"] += 1
                    continue
                anchor_scale = max(light["bounds"][2:])
                up, off_x = light["center"][1] - center[1], abs(center[0] - light["center"][0])
                if (not c["armor.plate_up_lo_pct"] * anchor_scale <= up * 100 <= c["armor.plate_up_hi_pct"] * anchor_scale
                        or off_x * 100 > c["armor.plate_offx_pct"] * anchor_scale):
                    self.counters["rejected_anchor"] += 1
                    continue
                lab_distance = self.lab_distance(ba["lab_mean"], bb["lab_mean"])
                if lab_distance > c["armor.max_lab_distance"]:
                    self.counters["rejected_pair_lab"] += 1
                    continue
                gap_fraction = self.gap_color_fraction(a, b, separation)
                if gap_fraction > c["armor.max_gap_color_fraction"]:
                    self.counters["rejected_gap"] += 1
                    continue
                color_similarity = max(0.0, 1.0 - lab_distance / c["armor.max_lab_distance"])
                bounds = self.unite(ba["bounds"], bb["bounds"])
                area, clipped = ba["area"] + bb["area"], ba["clipped"] or bb["clipped"]
                color_quality = min(ba["color_quality"], bb["color_quality"])
                target = {"center": center, "bounds": bounds, "area": area,
                          "radius": math.sqrt(area / math.pi), "clipped": clipped, "color": ba["color"],
                          "color_quality": color_quality,
                          "quality": color_quality * (0.5 + 0.5 * color_similarity) * (1 - gap_fraction)
                                     * (0.25 if clipped else 1.0),
                          "lab_mean": tuple((ba["lab_mean"][i] + bb["lab_mean"][i]) * 0.5 for i in range(3)),
                          "bar_lab_distance": lab_distance, "gap_color_fraction": gap_fraction,
                          "bar_centers": (a["center"], b["center"]), "bars": (a, b)}
                score = ((longest - shortest) + along) * 100.0 / longest
                score += (1 - color_similarity) * 25 + gap_fraction * 25
                score += c["armor.border_penalty"] * (int(ba["clipped"]) + int(bb["clipped"]))
                if predicted is not None:
                    tolerance = max(c["armor.association_min_px"], c["armor.association_k"] * longest)
                    change = (center[0] - predicted["center"][0]) ** 2 + (center[1] - predicted["center"][1]) ** 2
                    if (change > tolerance * tolerance or not self.same_bars(target, predicted, tolerance)
                            or self.match_cost(target, predicted, self.armor_motion.last) is None):
                        self.counters["rejected_association"] += 1
                        continue
                    score += 100 * change / (tolerance * tolerance)
                if best_score is None or score < best_score or (score == best_score and area > best["area"]):
                    best, best_score = target, score
        if best is not None:
            for bar in best["bars"]:
                bar["blob"]["reason"] = "paired"
        for blob in candidates:
            self.remember_debug_candidate(blob, role="armor")
        return best

    def quality_dropped(self, target, reference):
        return (target is not None and reference is not None
                and target["color_quality"] < reference["color_quality"] * self.config["track.quality_drop_ratio"])

    def measure(self, window, tracking, validate, predicted_light, predicted_armor):
        step = self.search_step(window, tracking)
        candidates = self.scan(window, DETECTION_LIGHT_COLORS, step, "light")
        candidates = self.refine_green_lamps(candidates, window, step)
        light = self.select_light(candidates, tracking, predicted_light)
        validate = not tracking or validate or self.quality_dropped(light, self.validated_light)
        armor = None
        # Preserve the exact pre-overlay window. finish() may apply a center
        # deadband later; drawing and diagnostics must not recompute a shifted ROI.
        armor_requested_roi = self.armor_window(light, False) if light is not None else None
        armor_roi = self.search_clip(armor_requested_roi) if armor_requested_roi is not None else None
        armor_scanned = light is not None and (predicted_armor is not None or validate)
        if armor_scanned:
            armor_candidates = self.scan(armor_roi, DETECTION_ARMOR_COLORS, 1, "armor")
            armor = self.select_armor(armor_candidates, light, predicted_armor)
        return {"light": light, "armor": armor, "full_validation": validate,
                "lamp_diameter": 2 * light["radius"] if light is not None else None,
                "armor_requested_roi": armor_requested_roi,
                "armor_roi": armor_roi, "armor_scanned": armor_scanned}

    def next_window(self, predicted_light):
        c = self.config
        full = self.search_bounds()
        if (predicted_light is None or self.missed_frames >= c["roi.reset_after_misses"]
                or (c["roi.full_scan_interval"] and self.frames_since_full >= c["roi.full_scan_interval"])):
            return full
        bounds = predicted_light["bounds"]
        # Armor is scanned independently on the full frame. Adding its window
        # here only enlarged the lamp scan and made its outer box enclose armor.
        grow = 1 + c["roi.grow_per_miss"] * self.missed_frames
        w = min(self.width, int(math.ceil(max(c["roi.min_width"],
                (bounds[2] * c["roi.size_k"] + 2 * c["roi.margin"]) * grow))))
        h = min(self.height, int(math.ceil(max(c["roi.min_height"],
                (bounds[3] * c["roi.size_k"] + 2 * c["roi.margin"]) * grow))))
        if w * h >= self.width * self.height * c["roi.full_area_fraction"]:
            return full
        x = max(0, min(self.width - w, int(math.floor(bounds[0] + bounds[2] * 0.5 - w * 0.5))))
        y = max(0, min(self.height - h, int(math.floor(bounds[1] + bounds[3] * 0.5 - h * 0.5))))
        return self.search_clip((x, y, w, h))

    def confirm_light(self, target):
        if target is None:
            self.pending_light = self.pending_armor = None
            self.light_hits = self.armor_hits = 0
            return False
        previous = self.pending_light
        matched = self.match_cost(target, previous, previous) is not None
        if previous is not None and self.far(target):
            jump = max(self.config["light.far_jump_min_px"],
                       self.config["light.far_jump_k"] * max(previous["bounds"][2:]))
            matched = matched and abs(target["center"][0] - previous["center"][0]) <= jump
            matched = matched and abs(target["center"][1] - previous["center"][1]) <= jump
        if not matched:
            self.light_hits = self.armor_hits = 0
            self.pending_armor = None
        self.pending_light = target
        needed = self.config["light.far_confirm_frames"] if self.far(target) else self.config["light.confirm_frames"]
        self.light_hits = min(needed, self.light_hits + 1)
        return self.light_hits >= needed

    def confirm_armor(self, target):
        if target is None:
            self.pending_armor = None
            self.armor_hits = 0
            return False
        tolerance = max(self.config["armor.association_min_px"],
                        self.config["armor.association_k"] * max(target["bounds"][2:]))
        if (self.match_cost(target, self.pending_armor, self.pending_armor) is None
                or not self.same_bars(target, self.pending_armor, tolerance)):
            self.armor_hits = 0
        self.pending_armor = target
        self.armor_hits = min(self.config["armor.confirm_frames"], self.armor_hits + 1)
        return self.armor_hits >= self.config["armor.confirm_frames"]

    def finish(self, result, tracking, received):
        light, armor = result["light"], result["armor"]
        measured_light, measured_armor = light, armor
        if not tracking:
            light_confirmed = self.confirm_light(light)
            armor_confirmed = self.confirm_armor(armor)
            if not light_confirmed:
                light = None
            if light is None or not armor_confirmed:
                armor = None
        elif self.armor_motion.last is None and not self.confirm_armor(armor):
            armor = None
        if light is not None:
            self.light_motion.observe(light, received)
            self.missed_frames = 0
            if result["full_validation"]:
                self.validated_light = dict(light)
                self.tracking_frames = 0
            else:
                self.tracking_frames += 1
            if armor is not None:
                self.armor_motion.observe(armor, received)
                self.validated_armor = dict(armor)
                self.armor_misses = self.armor_hits = 0
                self.pending_armor = None
            elif self.armor_motion.last is not None:
                self.armor_misses += 1
                if self.armor_misses >= self.config["armor.track_hold"]:
                    self.armor_motion.reset()
                    self.validated_armor = self.pending_armor = None
                    self.armor_hits = 0
                    self.tracking_frames = self.config["track.validation_interval"] - 1
        else:
            armor = None
            if tracking or measured_light is None:
                self.pending_armor = None
                self.armor_hits = 0
            self.missed_frames += 1
            if tracking and self.missed_frames >= self.config["roi.reset_after_misses"]:
                misses = self.missed_frames
                self.reset_tracking()
                self.missed_frames = misses
        # Motion sees measured centers; only the published lamp gets the deadband.
        if light is not None and self.published_light is not None:
            deadband = self.config["light.deadband_px"]
            if (abs(light["center"][0] - self.published_light["center"][0]) <= deadband
                    and abs(light["center"][1] - self.published_light["center"][1]) <= deadband):
                light["center"] = self.published_light["center"]
        self.published_light = light
        result["light"], result["armor"] = light, armor
        result["target"] = armor if armor is not None else light
        result["kind"] = "armor" if armor is not None else "light" if light is not None else "none"
        result["missed_frames"] = self.missed_frames
        result["light_hits"] = self.light_hits
        result["armor_hits"] = self.config["armor.confirm_frames"] if armor is not None else self.armor_hits
        if measured_light is None:
            if not result["light_candidates"]:
                result["reason"] = "no_lamp_color" if not result["rejected_color"] else "lamp_color_quality"
            elif not result["light_body_candidates"] and "green" in DETECTION_LIGHT_COLORS:
                result["reason"] = "lamp_body_unresolved"
            else:
                result["reason"] = "lamp_shape_or_association"
        elif light is None:
            result["reason"] = "lamp_confirm"
        elif armor is not None:
            result["reason"] = "armor_locked"
        elif measured_armor is not None:
            result["reason"] = "armor_confirm"
        elif not result["armor_scanned"]:
            result["reason"] = "lamp_locked"
        else:
            result["reason"] = "no_armor_color" if not result["armor_candidates"] else "armor_pair_rejected"
        self.frames_since_full = 0 if result["roi"] == self.search_bounds() else self.frames_since_full + 1

    def process(self, frame, frame_index, received, preview_due=True):
        started = time.ticks_us()
        self.counters = {name: 0 for name in (
            "raw_color_blobs", "candidates", "light_candidates", "light_body_candidates",
            "rejected_light_body", "armor_candidates", "rejected_color",
            "rejected_light_area", "rejected_light_shape", "rejected_bar_shape", "rejected_pair_color",
            "rejected_pair_geometry", "rejected_pair_lab", "rejected_gap", "rejected_anchor", "rejected_association")}
        self.debug_candidates = []
        self.debug_bars = []
        gap_reset = False
        try:
            if (frame.width(), frame.height()) != (self.width, self.height):
                raise DetectionError("Frame dimensions changed; restart for the new output mode")
            if self.previous_frame is not None:
                gap = time.ticks_diff(received, self.previous_received)
                if frame_index <= self.previous_frame or gap < 0:
                    raise DetectionError("Frame index / receive time went backwards")
                if gap > self.config["roi.max_gap"]:
                    self.reset_tracking()
                    gap_reset = True
            self.previous_frame, self.previous_received = frame_index, received
            self.frame = frame
            tracking = self.light_motion.last is not None
            predicted_light = self.light_motion.predict(received, self.width, self.height)
            predicted_armor = self.armor_motion.predict(received, self.width, self.height)
            validate = (not tracking or self.tracking_frames + 1 >= self.config["track.validation_interval"]
                        or self.pending_armor is not None or self.armor_misses > 0)
            window = self.next_window(predicted_light)
            result = self.measure(window, tracking, validate, predicted_light, predicted_armor)
            full = (0, 0, self.width, self.height)
            fallback = self.search_bounds()
            rescanned = False
            if tracking and window != fallback and (
                    result["light"] is None or result["light"]["clipped"]
                    or (result["armor"] is not None and result["armor"]["clipped"])):
                window = fallback
                self.debug_candidates = []
                self.debug_bars = []
                result = self.measure(window, True, True, predicted_light, predicted_armor)
                rescanned = True
            result.update(self.counters)
            result.update({"roi": window, "full_frame": window == full, "rescanned": rescanned,
                           "manual_roi": self.manual_roi, "search_step": self.search_step(window, tracking),
                           "phase": "track" if tracking else "detect", "frame": frame_index,
                           "gap_reset": gap_reset})
            # Multiple color scans of a frame still give one confirmation hit.
            self.finish(result, tracking, received)
            self.update_debug_candidates(result, tracking, predicted_light, received, preview_due)
            result["detect_us"] = time.ticks_diff(time.ticks_us(), started)
            self.result = result
            return result
        except Exception as error:
            self.result = None
            self.debug_candidates = []
            self.debug_bars = []
            self.clear_debug_cache()
            self.reset_tracking()
            if isinstance(error, DetectionError) or str(error) == "IDE interrupt":
                raise
            raise DetectionError("RGB565 Lab detection: %s" % error)
        finally:
            self.frame = None

    def report(self):
        result = self.result
        if result is None:
            return
        print("[DETECT] frame=%d phase=%s ROI=%s full=%s rescan=%s Lab_blobs=%d lamp_candidates=%d "
              "armor_candidates=%d misses=%d gap_reset=%s detect_ms=%.2f reason=%s" %
              (result["frame"], result["phase"], result["roi"], result["full_frame"], result["rescanned"],
               result["raw_color_blobs"], result["light_candidates"], result["armor_candidates"],
               result["missed_frames"], result["gap_reset"], result["detect_us"] / 1000.0, result["reason"]))
        if result["manual_roi"] is not None:
            print("[DETECT] manual_ROI=%s lamp_stride=%d; f=clear, c=color sample" %
                  (result["manual_roi"], result["search_step"]))
        print("[DETECT] lamp_diameter=%s armor_requested=%s armor_ROI=%s scanned=%s" %
              (result["lamp_diameter"], result["armor_requested_roi"], result["armor_roi"], result["armor_scanned"]))
        for kind in ("light", "armor"):
            target = result[kind]
            if target is None:
                print("[DETECT] %s=none confirm_hits=%d" %
                      (kind, result["light_hits"] if kind == "light" else result["armor_hits"]))
                continue
            print("[DETECT] %s=(%.2f,%.2f) color=%s Lab=%s pixels=%d color_quality=%.2f "
                  "quality=%.2f clipped=%s" %
                  (kind, target["center"][0], target["center"][1], target["color"], target["lab_mean"],
                   target["area"], target["color_quality"], target["quality"], target["clipped"]))
            if kind == "armor":
                print("[DETECT] bar_Lab_distance=%.2f gap_color_fraction=%.2f" %
                      (target["bar_lab_distance"], target["gap_color_fraction"]))
        target = result["target"]
        if target is None:
            print("[DETECT] target=none")
        else:
            cx, cy = target["center"]
            print("[DETECT] target=%s color=%s offset=(%.2f,%.2f) px" %
                  (result["kind"], target["color"], cx - self.width * 0.5, cy - self.height * 0.5))
        print("[DETECT] rejects color=%d lamp_body=%d lamp_area=%d lamp_shape=%d bar_shape=%d mixed_pairs=%d "
              "pair_geometry=%d pair_Lab=%d gap_color=%d anchor=%d association=%d" %
              tuple(result[name] for name in (
                  "rejected_color", "rejected_light_body", "rejected_light_area", "rejected_light_shape", "rejected_bar_shape",
                  "rejected_pair_color", "rejected_pair_geometry", "rejected_pair_lab", "rejected_gap",
                  "rejected_anchor", "rejected_association")))
        if result["target"] is None or result["armor"] is None:
            if result.get("debug_limited"):
                print("[DETECT] debug: candidate / time limit reached; supplemental scan is partial")
            if result.get("debug_age_ms") is not None:
                print("[DETECT] supplemental features age_ms=%d" % result["debug_age_ms"])
            print("[DETECT] blue_bar_debug=%d armor_scanned=%s" %
                  (len(self.debug_bars), result["armor_scanned"]))
            debug_blobs = list(self.debug_candidates) if result["target"] is None else []
            if result["armor"] is None:
                debug_blobs.extend(self.debug_bars)
            for blob in debug_blobs:
                print("[DETECT] candidate role=%s color=%s bounds=%s pixels=%d fill=%.2f "
                      "circularity=%.2f Lab=%s stage=%s" %
                      (blob["role"], blob["color"], blob["bounds"], blob["area"], blob["fill"],
                       blob["circularity"], blob["lab_mean"], blob["reason"]))

    def debug_label(self, blob):
        reason = blob["reason"]
        if blob.get("role") == "armor":
            if reason == "color":
                return "颜色 Lab(%d,%d,%d)" % tuple(blob["lab_mean"])
            if reason == "paired":
                return "装甲待确认 %d/%d" % (self.result["armor_hits"], self.config["armor.confirm_frames"])
            labels = {"bar_area": "太小", "bar_length": "长度", "bar_aspect": "灯条长宽比",
                      "bar_fill": "灯条填充率", "no_lamp": "无合格绿灯",
                      "armor_roi": "绿灯搜索区外", "bar_eligible": "未配对"}
            if reason in labels:
                return labels[reason]
        if reason == "selected":
            needed = self.config["light.far_confirm_frames"] if self.far(blob) else self.config["light.confirm_frames"]
            return "待确认 %d/%d" % (self.result["light_hits"], needed)
        if reason == "small":
            return "太小 %d<%d" % (blob["area"], self.config["light.min_area"])
        if reason == "circularity":
            return "圆度 %.2f<%.2f" % (blob["circularity"], self.config["light.min_circularity"])
        return {"color": "颜色", "area": "太大", "fill": "填充率", "aspect": "宽高比",
                "body_unresolved": "仅绿色种子/未取得整灯", "body_clipped": "灯体被扫描边界截断",
                "body_color": "灯体绿色占比不足", "association": "关联",
                "eligible": "评分未入选", "diagnostic": "待复核"}.get(reason, reason)

    def draw_light_outline(self, frame, target, color, thickness):
        # Fit the accepted lamp's outer bounds. Use the same center / extent as
        # armor_window(), including when the published point has a deadband.
        x, y, w, h = target["bounds"]
        cx, cy = int(round(x + w * 0.5)), int(round(y + h * 0.5))
        rx, ry = max(1, int(round(w * 0.5))), max(1, int(round(h * 0.5)))
        frame.draw_ellipse(cx, cy, rx, ry, 0, color=color, thickness=thickness)

    def draw_search_rois(self, frame):
        # Draw this layer after candidate labels, targets and the color sampler.
        # An absent / clipped armor window must have a visible explanation too.
        result = self.result
        if not DRAW_ROI or result is None:
            return
        lamp_roi, armor_roi = result["roi"], result["armor_roi"]
        lamp_text = ("手动ROI内扫描 %dx%d 步长1" if self.manual_roi is not None else
                     "绿灯扫描区 %dx%d") % lamp_roi[2:]
        if armor_roi is None:
            armor_text = ("装甲搜索:整灯范围未确定" if result["reason"] == "lamp_body_unresolved"
                          else "装甲搜索:无合格绿灯")
        elif armor_roi[2] <= 0 or armor_roi[3] <= 0:
            armor_text = "装甲搜索:窗口在选区外" if self.manual_roi is not None else "装甲搜索:窗口在画面外"
        else:
            state = "扫描" if result["armor_scanned"] else "间隔"
            armor_text = "装甲搜索 裁后%dx%d %s" % (armor_roi[2], armor_roi[3], state)
        labels = [(lamp_text, ROI_COLOR), (armor_text, ARMOR_ROI_COLOR)]
        if result["lamp_diameter"] is not None:
            requested = result["armor_requested_roi"]
            scale_text = "D=%.1f %.1fDx%.1fD=%.1fx%.1f" % (
                result["lamp_diameter"], self.config["armor.window_width_k"],
                self.config["armor.window_height_k"], requested[2], requested[3])
            labels.append((scale_text, ARMOR_ROI_COLOR))
        for index, (text, color) in enumerate(labels):
            text_width = min(self.width - 16, 4 + sum(16 if ord(char) > 127 else 9 for char in text))
            y = 26 + index * 22
            frame.draw_rectangle(8, y, text_width, 22, color=(0, 0, 0), fill=True)
            frame.draw_string_advanced(10, y + 2, 16, text, color=color)
        for window, color, thickness in ((lamp_roi, ROI_COLOR, 1), (armor_roi, ARMOR_ROI_COLOR, 3)):
            if window is None or window[2] <= 0 or window[3] <= 0:
                continue
            frame.draw_rectangle(window, color=(0, 0, 0), thickness=thickness + 2)
            frame.draw_rectangle(window, color=color, thickness=thickness)
        if self.manual_roi is not None:
            frame.draw_rectangle(self.manual_roi, color=(255, 220, 0), thickness=2)

    def draw(self, frame):
        result = self.result
        if result is None:
            return
        if DRAW_CANDIDATES:
            groups = (("light", "G", self.debug_candidates, (255, 180, 0)),
                      ("armor", "B", self.debug_bars, DETECTION_LABEL_COLORS["blue"]))
            for role, prefix, candidates, color in groups:
                if role == "armor" and result["armor"] is not None:
                    continue
                for index, blob in enumerate(candidates):
                    if role == "light" and blob["reason"] in ("selected", "eligible"):
                        self.draw_light_outline(frame, blob, color, 1)
                    else:
                        # Rejected components retain their true rectangular bounds;
                        # this outline is a search / diagnostic box, not a fitted circle.
                        frame.draw_rectangle(blob["bounds"], color=color, thickness=1)
                    if role == "armor":
                        cx, cy = blob["box_center"]
                        dx, dy = blob["axis"]
                        half = max(0.0, blob["length"] - 1.0) * 0.5
                        frame.draw_line(int(round(cx - dx * half)), int(round(cy - dy * half)),
                                        int(round(cx + dx * half)), int(round(cy + dy * half)),
                                        color=color, thickness=2)
                    if role == "light" and result["target"] is not None:
                        continue
                    text = "%s%d %s" % (prefix, index + 1, self.debug_label(blob))
                    text_width = min(self.width, 4 + sum(16 if ord(char) > 127 else 9 for char in text))
                    x, y, w, h = blob["bounds"]
                    x = max(0, min(self.width - text_width, x))
                    y = max(0, min(self.height - 20, y - 20 if y >= 20 else y + h + 2))
                    frame.draw_rectangle(x, y, text_width, 20, color=(0, 0, 0), fill=True)
                    frame.draw_string_advanced(x + 2, y, 16, text, color=color)
        if not DRAW_DETECTION:
            return
        for kind, label, color in (("light", "L", LIGHT_COLOR), ("armor", "A", ARMOR_COLOR)):
            target = result[kind]
            if target is None:
                continue
            if kind == "light":
                self.draw_light_outline(frame, target, color, 2)
            else:
                frame.draw_rectangle(target["bounds"], color=color, thickness=2)
            if kind == "armor":
                for bar in target["bars"]:
                    bar_color = DETECTION_LABEL_COLORS.get(bar["blob"]["color"], color)
                    first, last = bar["endpoints"]
                    frame.draw_line(int(round(first[0])), int(round(first[1])),
                                    int(round(last[0])), int(round(last[1])), color=bar_color, thickness=2)
            x, y = int(round(target["center"][0])), int(round(target["center"][1]))
            frame.draw_cross(x, y, color=(0, 0, 0), size=10, thickness=4)
            frame.draw_cross(x, y, color=color, size=10, thickness=2)
            text = "%s %s" % (label, target["color"])
            if kind == "light":
                text += " 灯体 D=%.1f C=%.2f" % (2 * target["radius"], target["circularity"])
            text_width = min(self.width, 4 + sum(14 if ord(char) > 127 else 8 for char in text))
            frame.draw_string_advanced(max(0, min(self.width - text_width, x + 5)), max(0, y - 20), 14,
                                       text, color=color)
        target = result["target"]
        if target is not None:
            x, y = int(round(target["center"][0])), int(round(target["center"][1]))
            color = ARMOR_COLOR if result["kind"] == "armor" else LIGHT_COLOR
            frame.draw_cross(x, y, color=(0, 0, 0), size=14, thickness=4)
            frame.draw_cross(x, y, color=color, size=14, thickness=2)
        text = "%s %s L=%d A=%d" % (result["kind"], result["reason"],
                                     result["light_candidates"], result["armor_candidates"])
        if DRAW_CANDIDATES and result["armor"] is None:
            text += " B=%d" % len(self.debug_bars)
        if result.get("debug_limited"):
            text += " 诊断限额"
        if result.get("debug_age_ms") is not None:
            text += " debug=%dms" % result["debug_age_ms"]
        frame.draw_string_advanced(8, max(0, self.height - 22), 14, text, color=(255, 255, 255))


class RawCheck:
    """Bounded RAW10 statistics read from VICAP before demosaic, WB, CCM, LSC and NR.

    Phases are (row parity, column parity) of the full sensor frame. The firmware
    declares BGGR: (0,0)=B, (0,1)=(1,0)=G, (1,1)=R. Each frame is released before
    its statistics are reported; only small per-frame sums are kept.
    """

    WARMUP_FRAMES = 5      # Let the fixed exposure / gain reach the sensor first.
    GRID_STEP = 16         # One 2x2 Bayer cell every 16 pixels in both directions.
    BANDS = 8              # Horizontal bands for a phase change inside a frame.
    LANE_ROWS = 8          # Full-width rows for the MIPI byte-lane comparison.
    MONO_SPREAD = 0.04     # Four phases within 4%: no color filter response.
    DARK_SIGNAL = 0.1      # Signal below 10% of the range: ratios are noise / offset bound.
    PHASE_MARGIN = 0.05    # Weaker diagonal contrast does not decide a phase.
    LANE_RATIO = 1.5       # Worst / median lane roughness flagged as a suspect lane.
    FLICKER_CV = 0.03      # Frame-level variation at fixed exposure and gain.

    def __init__(self, sensor):
        import uctypes
        from mpp.sys import kd_mpi_sys_mmap, kd_mpi_sys_munmap
        from mpp.vicap import (VICAP_DUMP_RAW, VICAP_MIPI_4LANE, kd_mpi_vicap_dump_frame,
                               kd_mpi_vicap_dump_release)
        from mpp.video_struct import (PIXEL_FORMAT_RGB_BAYER_10BPP,
                                      PIXEL_FORMAT_RGB_BAYER_12BPP, k_video_frame_info)

        info = sensor._dev_attr.sensor_info
        self.uctypes = uctypes
        self.device = sensor._dev_id
        self.size = (info.width, info.height)
        self.full_scales = {PIXEL_FORMAT_RGB_BAYER_10BPP: 1023,
                            PIXEL_FORMAT_RGB_BAYER_12BPP: 4095}
        self.mmap = kd_mpi_sys_mmap
        self.munmap = kd_mpi_sys_munmap
        self.dump = kd_mpi_vicap_dump_frame
        self.release = kd_mpi_vicap_dump_release
        self.dump_raw = VICAP_DUMP_RAW
        self.frame_info = k_video_frame_info
        self.setup(info.width, 4 if info.mipi_lanes == VICAP_MIPI_4LANE else 2)

    def setup(self, width, lanes):
        # CSI-2 deals payload bytes round-robin over the lanes. RAW10 sends four MSB
        # bytes and one shared LSB byte per four pixels; pixel x's MSB byte is here.
        self.lanes = lanes
        self.lane_of = [((x >> 2) * 5 + (x & 3)) % lanes for x in range(width)]
        self.lane_samples = [0] * lanes
        for x in range(2, width - 2):
            self.lane_samples[self.lane_of[x]] += 1
        self.results = []
        self.layout = None

    def run(self, frames, timeout_ms):
        print("[RAW] Sensor RAW before every ISP stage; aim at an evenly lit, "
              "unsaturated white or grey target that fills the view")
        for index in range(self.WARMUP_FRAMES + frames):
            os.exitpoint()
            self.capture(timeout_ms, index >= self.WARMUP_FRAMES)
        return self.report()

    def capture(self, timeout_ms, measure):
        info = self.frame_info()
        ret = self.dump(self.device, 0, self.dump_raw, info, timeout_ms)
        if ret != 0:
            raise RuntimeError("RAW dump returned %d" % ret)
        virt = 0
        size = 0
        result = None
        try:
            frame = info.v_frame
            width, height, stride = frame.width, frame.height, frame.stride[0]
            full_scale = self.full_scales.get(frame.pixel_format)
            if (width, height) != self.size or full_scale is None:
                raise RuntimeError("Unexpected RAW frame %dx%d format=%d" %
                                   (width, height, frame.pixel_format))
            # Vendor RAW dumps hold one pixel per 16-bit word, rows width*2 bytes apart.
            row_bytes = max(stride, width * 2)
            size = row_bytes * height
            virt = self.mmap(frame.phys_addr[0], size)
            if not virt:
                raise RuntimeError("RAW frame mmap failed")
            if measure:
                self.layout = (width, height, stride, full_scale)
                result = self.measure(self.rows(virt, row_bytes, width),
                                      width, height, full_scale)
        finally:
            if virt:
                self.munmap(virt, size)
            released = self.release(self.device, 0, info)
        if released != 0:
            raise RuntimeError("RAW frame release returned %d; restart the board" % released)
        if result is not None:
            self.results.append(result)

    def rows(self, virt, row_bytes, width):
        uctypes = self.uctypes
        layout = {"v": (uctypes.ARRAY | 0, width | uctypes.UINT16)}

        def row(y):
            return uctypes.struct(virt + y * row_bytes, layout, uctypes.LITTLE_ENDIAN).v

        return row

    def measure(self, row, width, height, full_scale):
        step = self.GRID_STEP
        band_height = max(2, height // self.BANDS)
        clip = full_scale - 3
        phase = [0, 0, 0, 0]
        bands = [[0, 0, 0] for _ in range(self.BANDS)]  # (anti-diagonal, main diagonal, cells)
        peak = 0
        clipped = 0
        cells = 0
        for y in range(0, height - 1, step):
            top = row(y)
            bottom = row(y + 1)
            band = bands[min(y // band_height, self.BANDS - 1)]
            for x in range(0, width - 1, step):
                p00 = top[x]
                p01 = top[x + 1]
                p10 = bottom[x]
                p11 = bottom[x + 1]
                phase[0] += p00
                phase[1] += p01
                phase[2] += p10
                phase[3] += p11
                band[0] += p01 + p10
                band[1] += p00 + p11
                band[2] += 1
                high = max(p00, p01, p10, p11)
                if high > peak:
                    peak = high
                if high >= clip:
                    clipped += 1
                cells += 1
        # Second difference against same-color neighbours, attributed to pixel x's lane.
        lane_sums = [0] * self.lanes
        lane_of = self.lane_of
        for k in range(self.LANE_ROWS):
            line = row(((2 * k + 1) * height) // (2 * self.LANE_ROWS))
            values = [line[x] for x in range(width)]
            for x in range(2, width - 2):
                lane_sums[lane_of[x]] += abs(2 * values[x] - values[x - 2] - values[x + 2])
        return {"phase": phase, "cells": cells, "bands": bands, "peak": peak,
                "clipped": clipped, "lanes": lane_sums}

    def report(self):
        """Print the RAW findings.

        Returns True when no fault is seen before the ISP, False for a fault and None
        when the frames cannot decide (covered lens, or no usable color signal).
        """
        results = self.results
        count = len(results)
        width, height, stride, full_scale = self.layout
        cells = sum(result["cells"] for result in results)
        phase = [sum(result["phase"][i] for result in results) / cells for i in range(4)]
        level = sum(phase) / 4
        peak = max(result["peak"] for result in results)
        clipped = sum(result["clipped"] for result in results) * 100.0 / cells
        faults = []
        print("[RAW] layout: %dx%d stride=%d full_scale=%d peak=%d frames=%d lanes=%d" %
              (width, height, stride, full_scale, peak, count, self.lanes))
        print("[RAW] mean code (row,col): 00=%.1f 01=%.1f 10=%.1f 11=%.1f" % tuple(phase))
        print("[RAW] level=%.0f%% of full scale, clipped cells=%.1f%% "
              "(codes include the sensor black offset)" %
              (level * 100.0 / full_scale, clipped))
        if clipped > 1.0:
            print("[WARN] RAW clipped; lower EXPOSURE_US or ANALOG_GAIN")
        if level <= 0:
            print("[FAIL] RAW frames are all zero; no color or lane result")
            return False

        black = RAW_BLACK_LEVEL
        if black is None:
            if (level < self.DARK_SIGNAL * full_scale and
                    (max(phase) - min(phase)) / level < self.MONO_SPREAD):
                print("[RAW] dark frame with four equal phases: if the lens is fully "
                      "covered this is the black level; set RAW_BLACK_LEVEL = %d" %
                      int(level + 0.5))
                print("[RAW] RESULT: black reference only; rerun at a lit white target")
                return None
            print("[RAW] RAW_BLACK_LEVEL unknown: ratios below include the black offset; "
                  "cover the lens fully and rerun once to measure it")
            black = 0
        signal = [value - black for value in phase]
        signal_level = sum(signal) / 4
        if min(signal) <= 0:
            print("[RAW] RESULT: a phase is at or below RAW_BLACK_LEVEL=%d; re-measure "
                  "the black level with the lens covered" % black)
            return None
        dark = signal_level < self.DARK_SIGNAL * (full_scale - black)
        if dark:
            print("[WARN] RAW signal is only %.1f%% of the range above black: underexposed. "
                  "The ISP amplifies noise and small black-level errors into color casts "
                  "and smears them in noise reduction; raise EXPOSURE_US (about 8200 max "
                  "at 120 fps) or ANALOG_GAIN" %
                  (signal_level * 100.0 / (full_scale - black)))

        anti = (signal[1] + signal[2]) / 2
        main = (signal[0] + signal[3]) / 2
        contrast = (anti - main) / (anti + main)
        spread = (max(signal) - min(signal)) / signal_level
        green_pair = None
        if spread < self.MONO_SPREAD and dark:
            print("[RAW] CFA: four phases within %.1f%%, but too dark to tell a mono part "
                  "from missing light" % (spread * 100))
        elif spread < self.MONO_SPREAD:
            print("[RAW] CFA: four phases within %.1f%%: monochrome sensor or IR-only light; "
                  "a mono part cannot be demosaiced into color (use GRAYSCALE and "
                  "MPP_SC132GS_MONO)" % (spread * 100))
            faults.append("mono")
        elif abs(contrast) < self.PHASE_MARGIN:
            print("[RAW] CFA: diagonal contrast %.1f%% is too weak to decide the Bayer "
                  "phase; use a white or grey target" % (contrast * 100))
        else:
            anti_closer = abs(signal[1] - signal[2]) < abs(signal[0] - signal[3])
            if anti_closer and contrast > 0:
                green_pair = "anti"
                print("[RAW] CFA: green at (0,1)/(1,0), as declared by the firmware BGGR; "
                      "a static Bayer phase error does not cause purple white")
                print("[RAW] R/B order: under a red target, (1,1) must rise above (0,0)")
            elif not anti_closer and contrast < 0:
                green_pair = "main"
                print("[RAW] CFA: green at (0,0)/(1,1), but the firmware declares BGGR; "
                      "the ISP reads green as R/B, which turns white purple. Firmware "
                      "fix: GBRG/GRBG in the SC132GS mode list")
                faults.append("bayer")
            else:
                print("[RAW] CFA: green-pair similarity and brightness disagree; "
                      "use a neutral white or grey target")

        if green_pair is not None:
            sign = 1 if contrast > 0 else -1
            frame_flips = 0
            band_flips = 0
            for result in results:
                values = result["phase"]
                # The black offset cancels in the differences, not in the totals.
                total = sum(values) - 4 * black * result["cells"]
                if total > 0 and sign * (values[1] + values[2] - values[0] - values[3]) < \
                        -self.PHASE_MARGIN * total:
                    frame_flips += 1
                for band_anti, band_main, band_cells in result["bands"]:
                    band_total = band_anti + band_main - 4 * black * band_cells
                    if band_total > 0 and sign * (band_anti - band_main) < \
                            -self.PHASE_MARGIN * band_total:
                        band_flips += 1
            print("[RAW] phase flips: frames=%d/%d bands=%d/%d" %
                  (frame_flips, count, band_flips, count * self.BANDS))
            if frame_flips or band_flips:
                print("[RAW] Bayer phase changes between frames or inside a frame: "
                      "MIPI line / pixel slips; this flashes or bands the image purple. "
                      "Check the FPC, the lanes and the receiver timing, not the ISP")
                faults.append("phase slip")

        lanes = [sum(result["lanes"][i] for result in results) /
                 max(1, self.lane_samples[i] * count * self.LANE_ROWS)
                 for i in range(self.lanes)]
        ordered = sorted(lanes)
        middle = len(ordered) // 2
        median = (ordered[middle - 1] + ordered[middle]) / 2
        print("[RAW] MIPI byte-lane roughness: " +
              " ".join("L%d=%.1f" % (i, value) for i, value in enumerate(lanes)))
        if median > 0 and ordered[-1] / median > self.LANE_RATIO:
            worst = lanes.index(ordered[-1])
            print("[RAW] byte lane %d is %.1fx the median: suspect its data pair or the "
                  "FPC contact (the lane numbers are receiver byte lanes)" %
                  (worst, ordered[-1] / median))
            faults.append("lane")

        levels = [sum(result["phase"]) / (4.0 * result["cells"]) for result in results]
        mean = sum(levels) / count
        deviation = (sum((value - mean) ** 2 for value in levels) / count) ** 0.5
        variation = deviation / (mean - black) if mean > black else 0.0
        print("[RAW] frame level min=%.1f max=%.1f cv=%.1f%%" %
              (min(levels), max(levels), variation * 100))
        if variation > self.FLICKER_CV:
            print("[RAW] brightness changes at fixed exposure / gain: lamp flicker "
                  "(mains lamps at 100 Hz against 120 fps) or unstable exposure; "
                  "retest under daylight or a DC lamp before blaming the ISP")
            faults.append("flicker")

        if faults:
            print("[RAW] RESULT: findings before the ISP: %s; ISP calibration cannot "
                  "fix these" % ", ".join(faults))
            return False
        if green_pair is None:
            print("[RAW] RESULT: no MIPI fault seen, but no color verdict; rerun at a "
                  "lit white or grey target")
            return None
        if green_pair == "anti":
            if RAW_BLACK_LEVEL is not None and not dark and clipped <= 1.0:
                print("[RAW] white needs WB gains R=%.2f B=%.2f under this light "
                      "(firmware manual profile: R=1.12 B=2.51)" %
                      (anti / signal[3], anti / signal[0]))
            else:
                print("[RAW] no WB gain estimate: it needs RAW_BLACK_LEVEL and an "
                      "unclipped white target at >= %d%% signal" %
                      int(self.DARK_SIGNAL * 100))
        if dark:
            print("[RAW] RESULT: no fault before the ISP, but the frame is underexposed; "
                  "fix exposure before judging ISP color or sharpness")
        else:
            print("[RAW] RESULT: RAW data is clean; remaining color / blur problems come "
                  "from ISP processing (WB, CCM, LSC, noise reduction profile), the "
                  "RGB565 / IDE preview path or lens focus")
        return True


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
    expected_size = sensor_size()
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
    width, height = output_size()
    print("[INFO] firmware sensor_mode=%dx%d %s type=%d RAW10 nominal_fps=%d "
          "lanes_enum=%d output=%dx%d RGB565" %
          (info.width, info.height, "crop" if USE_VGA_MODE else "full sensor",
           actual_type, info.fps, info.mipi_lanes, width, height))
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
        frame = capture_color_frame(sensor, max(FRAME_TIMEOUT_MS, 1000))
        width, height = output_size()
        if frame is None or (frame.width(), frame.height()) != (width, height):
            raise RuntimeError("Internal TPG did not return a %dx%d frame" % (width, height))
        print("[DIAG] ISP internal TPG produced %dx%d: ISP output works with "
              "generated input; investigate sensor / MCLK / MIPI input" % (width, height))
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


class RecordingError(RuntimeError):
    pass


class RecordingSlot:
    def __init__(self, capacity):
        self.data = bytearray(capacity)
        self.view = memoryview(self.data)
        self.size = self.header_size = self.pts = 0
        self.key = False


class RecordingQueue:
    """Single producer / single consumer; publish after copying, retire after I/O.

    Each index has exactly one writer. Python attribute publication uses the
    interpreter GIL. The consumer keeps its slot occupied while file I/O releases
    the GIL, so the producer never overwrites a buffer still used by write().
    No lock is held across I/O and neither side waits for the other.
    """

    def __init__(self, count, capacity):
        self.slots = [RecordingSlot(capacity) for _ in range(count)]
        self.produced = self.consumed = self.peak = 0

    def reserve(self):
        if self.produced - self.consumed >= len(self.slots):
            return None
        return self.slots[self.produced % len(self.slots)]

    def publish(self):
        self.produced += 1
        self.peak = max(self.peak, self.produced - self.consumed)

    def front(self):
        if self.produced == self.consumed:
            return None
        return self.slots[self.consumed % len(self.slots)]

    def retire(self):
        self.consumed += 1


def h264_nals(data, size, offset=0):
    """Iterate Annex B NAL ranges without copying the frame or growing a list."""
    start = data.find(b"\x00\x00\x01", offset, size)
    count = 0
    while start >= 0:
        payload = start + 3
        following = data.find(b"\x00\x00\x01", payload, size)
        end = size if following < 0 else following
        while end > payload and data[end - 1] == 0:
            end -= 1
        if end > payload:
            count += 1
            if count > 64:
                raise RecordingError("H.264 access unit exceeds 64 NALs")
            yield payload, end
        start = following


class FragmentedMp4:
    """Streaming AVC MP4: one bounded fragment per frame, no growing sample table.

    File methods are called only by RecordingWriter. No native MP4 binding is
    used: its current firmware wrappers hold the GIL during disk I/O.
    """

    def __init__(self, file, width, height, fps, limit):
        import struct

        self.pack = struct.pack
        self.file = file
        self.width, self.height = width, height
        self.duration = 1000000 // fps
        self.limit = limit
        self.bytes = self.frames = 0
        self.first_pts = None
        self.limit_reached = False

    def u32(self, value):
        return self.pack(">I", value)

    def box(self, kind, data):
        return self.u32(len(data) + 8) + kind + data

    def movie(self, slot):
        sps = pps = None
        for begin, end in h264_nals(slot.data, slot.header_size):
            kind = slot.data[begin] & 31
            if kind == 7:
                sps = bytes(slot.view[begin:end])
            elif kind == 8:
                pps = bytes(slot.view[begin:end])
        if sps is None or len(sps) < 4 or pps is None:
            raise RecordingError("H.264 keyframe needs Annex B SPS and PPS headers")
        box, u32, pack = self.box, self.u32, self.pack
        matrix = pack(">9I", 0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000)
        avcc = (b"\x01" + sps[1:4] + b"\xff\xe1" + pack(">H", len(sps)) + sps
                + b"\x01" + pack(">H", len(pps)) + pps)
        avc1 = (b"\x00" * 6 + pack(">H", 1) + b"\x00" * 16
                + pack(">HHII", self.width, self.height, 0x480000, 0x480000)
                + u32(0) + pack(">H", 1) + b"\x00" * 32 + pack(">HH", 24, 0xffff)
                + box(b"avcC", avcc))
        stbl = box(b"stbl", box(b"stsd", u32(0) + u32(1) + box(b"avc1", avc1))
                   + box(b"stts", b"\x00" * 8) + box(b"stsc", b"\x00" * 8)
                   + box(b"stsz", b"\x00" * 12) + box(b"stco", b"\x00" * 8))
        dinf = box(b"dinf", box(b"dref", u32(0) + u32(1) + box(b"url ", u32(1))))
        minf = box(b"minf", box(b"vmhd", u32(1) + b"\x00" * 8) + dinf + stbl)
        mdhd = box(b"mdhd", b"\x00" * 12 + u32(1000000) + u32(0) + pack(">HH", 0x55c4, 0))
        hdlr = box(b"hdlr", b"\x00" * 8 + b"vide" + b"\x00" * 12 + b"Video\x00")
        mdia = box(b"mdia", mdhd + hdlr + minf)
        tkhd = box(b"tkhd", u32(7) + b"\x00" * 8 + u32(1) + b"\x00" * 24
                   + matrix + pack(">II", self.width << 16, self.height << 16))
        mvhd = box(b"mvhd", b"\x00" * 12 + u32(1000000) + u32(0) + u32(0x10000)
                   + pack(">H", 0x100) + b"\x00" * 10 + matrix + b"\x00" * 24 + u32(2))
        mvex = box(b"mvex", box(b"trex", pack(">6I", 0, 1, 1, self.duration, 0, 0)))
        return (box(b"ftyp", b"isom" + u32(0x200) + b"isomiso6avc1mp41")
                + box(b"moov", mvhd + box(b"trak", tkhd + mdia) + mvex))

    def fragment(self, pts, size, key, offset):
        box, pack = self.box, self.pack
        tfhd = box(b"tfhd", pack(">II", 0x20000, 1))
        tfdt = box(b"tfdt", pack(">IQ", 0x1000000, pts))
        flags = 0x02000000 if key else 0x01010000
        trun = box(b"trun", pack(">6I", 0x701, 1, offset, self.duration, size, flags))
        return box(b"moof", box(b"mfhd", pack(">II", 0, self.frames + 1))
                   + box(b"traf", tfhd + tfdt + trun))

    def write_all(self, data):
        view = memoryview(data)
        offset = 0
        while offset < len(view):
            count = self.file.write(view[offset:])
            if count is None or count <= 0 or count > len(view) - offset:
                raise RecordingError("Video write made no progress")
            offset += count
            self.bytes += count

    def write(self, slot):
        if self.first_pts is None and not slot.key:
            raise RecordingError("MP4 must start with an H.264 keyframe")
        payload_size = 0
        for begin, end in h264_nals(slot.data, slot.size, slot.header_size):
            payload_size += end - begin + 4
        if payload_size == 0:
            raise RecordingError("H.264 frame contains no Annex B NALs")
        movie = self.movie(slot) if self.first_pts is None else b""
        first_pts = slot.pts if self.first_pts is None else self.first_pts
        pts = slot.pts - first_pts
        moof = self.fragment(pts, payload_size, slot.key, 0)
        moof = self.fragment(pts, payload_size, slot.key, len(moof) + 8)
        # Reserve the entire fragment INCLUDING all MP4 overhead before writing
        # its first byte. No final index/trailer is appended on close.
        required = len(movie) + len(moof) + 8 + payload_size
        if self.bytes + required > self.limit:
            self.limit_reached = True
            return False
        self.first_pts = first_pts
        self.write_all(movie)
        self.write_all(moof)
        self.write_all(self.u32(payload_size + 8) + b"mdat")
        for begin, end in h264_nals(slot.data, slot.size, slot.header_size):
            self.write_all(self.u32(end - begin))
            self.write_all(slot.view[begin:end])
        self.frames += 1
        return True


class RecordingWriter:
    """Own every filesystem operation, including scan, flush and close."""

    STORAGE_LIMIT_BYTES = 2000000000

    def __init__(self, queue, width, height, fps):
        self.queue = queue
        self.width, self.height, self.fps = width, height, fps
        self.directory = RECORDING_DIR.rstrip("/")
        self.path = None
        self.stop = self.done = self.limit_reached = False
        self.error = None
        self.frames = self.bytes = self.existing_bytes = 0

    def storage_path(self):
        import errno

        try:
            os.mkdir(self.directory)
        except OSError as error:
            if error.args[0] != errno.EEXIST:
                raise
            if not os.stat(self.directory)[0] & 0x4000:
                raise RecordingError("Recording directory is not a directory")
        index = 0
        # ilistdir streams entries rather than allocating a list proportional to
        # the recording history. Count ALL files in this dedicated directory.
        for entry in os.ilistdir(self.directory):
            name, kind = entry[0], entry[1]
            if kind & 0xf000 != 0x8000:
                raise RecordingError("Recording directory must contain only regular files")
            size = entry[3] if len(entry) >= 4 else os.stat(self.directory + "/" + name)[6]
            self.existing_bytes += size
            if self.existing_bytes >= self.STORAGE_LIMIT_BYTES:
                self.limit_reached = True
                return None
            if (name.startswith("sc132gs_") and name.endswith(".mp4")
                    and len(name) == 16 and name[8:12].isdigit()):
                index = max(index, int(name[8:12]))
        if index >= 9999:
            raise RecordingError("No unused sc132gs_NNNN.mp4 name")
        return "%s/sc132gs_%04d.mp4" % (self.directory, index + 1)

    def run(self):
        file = None
        try:
            self.path = self.storage_path()
            if self.path is None:
                return
            muxer = None
            while not self.limit_reached:
                slot = self.queue.front()
                if slot is None:
                    if self.stop:
                        break
                    time.sleep_ms(2)
                    continue
                try:
                    if file is None:
                        file = open(self.path, "wb")
                        muxer = FragmentedMp4(file, self.width, self.height, self.fps,
                                              self.STORAGE_LIMIT_BYTES - self.existing_bytes)
                    if not muxer.write(slot):
                        self.limit_reached = True
                    self.frames, self.bytes = muxer.frames, muxer.bytes
                finally:
                    if muxer is not None:
                        self.frames, self.bytes = muxer.frames, muxer.bytes
                    self.queue.retire()
        except BaseException as error:
            self.error = "Video writer: %s" % error
        finally:
            # Never hold queue locks or an encoder frame during these syscalls.
            # CanMV POSIX file write/flush/close release the interpreter GIL.
            if file is not None:
                for action in (file.flush, file.close):
                    try:
                        action()
                    except BaseException as error:
                        if self.error is None:
                            self.error = "Video writer close: %s" % error
            self.done = True


class VideoRecorder:
    """Main loop copies bounded encoded frames; only the writer touches disk."""

    MAX_STREAMS_PER_POLL = 4
    MAX_HEADER_BYTES = 65536

    def __init__(self):
        self.encoder = None
        self.created = self.started = False
        self.link = self.writer = self.queue = None
        self.header_size = 0
        self.header_group = False
        self.last_pts = self.last_stream_ms = None
        self.need_idr = True
        self.idr_requested = False
        self.enqueued = self.dropped = self.oversized = 0
        self.limit_reported = False

    @property
    def frames(self):
        return 0 if self.writer is None else self.writer.frames

    def prepare(self, sensor):
        import _thread
        import uctypes
        from media.sensor import Sensor
        from media.vencoder import Encoder, ChnAttrStr, StreamData
        from media.media import MediaManager, CAM_CHN_ID_1, VIDEO_ENCODE_MOD_ID, VENC_DEV_ID

        self.uctypes = uctypes
        self.queue = RecordingQueue(RECORDING_QUEUE_FRAMES, RECORDING_FRAME_BYTES)
        self.headers = bytearray(self.MAX_HEADER_BYTES)
        self.header_view = memoryview(self.headers)
        width, height = (640, 480) if USE_VGA_MODE else SCALED_FULL_SIZE
        source_fps = sensor._dev_attr.sensor_info.fps
        if source_fps < RECORDING_FPS:
            raise RecordingError("RECORDING_FPS exceeds sensor nominal fps %s" % source_fps)
        sensor.set_framesize(width=width, height=height, chn=CAM_CHN_ID_1,
                             alignment=12, crop=USE_VGA_MODE)
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)
        self.encoder = Encoder()
        self.stream = StreamData()
        self.encoder.SetOutBufs(self.MAX_STREAMS_PER_POLL, width, height)
        attr = ChnAttrStr(Encoder.PAYLOAD_TYPE_H264, Encoder.H264_PROFILE_MAIN,
                          width, height, bit_rate=RECORDING_BITRATE_KBPS,
                          gopLen=RECORDING_FPS, src_frame_rate=source_fps,
                          dst_frame_rate=RECORDING_FPS)
        self.encoder.Create(attr)
        self.created = True
        self.link = MediaManager.link(sensor.bind_info(chn=CAM_CHN_ID_1)["src"],
                                      (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.encoder.chn))
        self.encoder.Start()
        self.started = True
        writer = RecordingWriter(self.queue, width, height, RECORDING_FPS)
        _thread.start_new_thread(writer.run, ())
        self.writer = writer
        print("[RECORD] H.264 fragmented MP4 %dx%d target_fps=%d bitrate_kbps=%d; "
              "queue=%d x %d bytes, directory cap=2000000000 bytes; clean ISP image" %
              (width, height, RECORDING_FPS, RECORDING_BITRATE_KBPS,
               RECORDING_QUEUE_FRAMES, RECORDING_FRAME_BYTES))

    def drop(self, oversized=False):
        self.dropped += 1
        self.oversized += int(oversized)
        self.need_idr = True
        if not self.idr_requested:
            self.encoder.RequestIDR()
            self.idr_requested = True

    def copy_stream(self):
        if not 0 <= self.stream.pack_cnt <= len(self.stream.data):
            raise RecordingError("Invalid H.264 stream pack count")
        for index in range(self.stream.pack_cnt):
            kind = self.stream.stream_type[index]
            address, size = self.stream.data[index], self.stream.data_size[index]
            if kind == self.encoder.STREAM_TYPE_HEADER:
                if not self.header_group:
                    self.header_size = 0
                    self.header_group = True
                if size < 0 or self.header_size + size > self.MAX_HEADER_BYTES:
                    raise RecordingError("H.264 headers exceeded %d bytes" % self.MAX_HEADER_BYTES)
                self.header_view[self.header_size:self.header_size + size] = self.uctypes.bytearray_at(address, size)
                self.header_size += size
                continue
            self.header_group = False
            if kind not in (self.encoder.STREAM_TYPE_I, self.encoder.STREAM_TYPE_P):
                raise RecordingError("Unexpected H.264 stream type %s" % kind)
            key = kind == self.encoder.STREAM_TYPE_I
            if key:
                self.idr_requested = False
            pts = self.stream.pts[index]
            if self.last_pts is not None and pts <= self.last_pts:
                raise RecordingError("H.264 capture timestamps did not increase")
            self.last_pts = pts
            if self.need_idr and not key:
                self.dropped += 1
                continue
            if key and not self.header_size:
                raise RecordingError("First H.264 I-frame has no SPS/PPS headers")
            header_size = self.header_size if key else 0
            if size <= 0 or size + header_size > RECORDING_FRAME_BYTES:
                self.drop(oversized=True)
                continue
            slot = self.queue.reserve()
            if slot is None:
                self.drop()
                continue
            if header_size:
                slot.view[:header_size] = self.header_view[:header_size]
            slot.view[header_size:header_size + size] = self.uctypes.bytearray_at(address, size)
            slot.size, slot.header_size = header_size + size, header_size
            slot.pts, slot.key = pts, key
            self.queue.publish()
            self.enqueued += 1
            self.need_idr = False

    def stop_encoder(self):
        actions = []
        if self.link is not None:
            actions.append(self.link.destroy)
            self.link = None
        if self.started:
            actions.append(self.encoder.Stop)
            self.started = False
        if self.created:
            actions.append(self.encoder.Destroy)
            self.created = False
        error = None
        for action in actions:
            try:
                action()
            except Exception as failure:
                if error is None:
                    error = failure
        if error is not None:
            raise RecordingError("Video encoder cleanup: %s" % error)

    def check_writer(self):
        if self.writer is not None and self.writer.error is not None:
            raise RecordingError(self.writer.error)

    def poll(self, check_timeout=True):
        self.check_writer()
        if self.writer is not None and self.writer.limit_reached:
            self.stop_encoder()
            if not self.limit_reported:
                print("[RECORD] 2 GB directory limit reached; recording stopped, capture continues")
                self.limit_reported = True
            return
        if not self.started:
            return
        try:
            for _ in range(self.MAX_STREAMS_PER_POLL):
                result = self.encoder.GetStream(self.stream, timeout=0)
                now = time.ticks_ms()
                if self.last_stream_ms is None:
                    self.last_stream_ms = now
                if result != 0:
                    if check_timeout and time.ticks_diff(now, self.last_stream_ms) >= FRAME_TIMEOUT_MS:
                        raise RecordingError("No H.264 stream for %d ms (GetStream returned %s)" %
                                             (FRAME_TIMEOUT_MS, result))
                    break
                try:
                    self.copy_stream()
                finally:
                    self.encoder.ReleaseStream(self.stream)
                self.last_stream_ms = now
        except Exception as error:
            if isinstance(error, RecordingError) or str(error) == "IDE interrupt":
                raise
            raise RecordingError("Video recording: %s" % error)

    def check_output(self):
        self.check_writer()
        if self.enqueued == 0 and not (self.writer is not None and self.writer.limit_reached):
            raise RecordingError("Capture ended without any H.264 video frames")

    def report(self):
        print("[RECORD] written=%d queued=%d dropped=%d oversized=%d peak_pending=%d" %
              (self.frames, self.queue.produced - self.queue.consumed, self.dropped,
               self.oversized, self.queue.peak))

    def close(self):
        error = None
        try:
            if self.started:
                self.poll(check_timeout=False)
        except Exception as failure:
            error = failure
        try:
            self.stop_encoder()
        except Exception as failure:
            if error is None:
                error = failure
        if self.writer is not None:
            self.writer.stop = True
            # Bounded cooperative wait, never join/flush/fsync from this thread.
            for _ in range((RECORDING_STOP_TIMEOUT_MS + 9) // 10):
                if self.writer.done:
                    break
                time.sleep_ms(10)
            if not self.writer.done:
                error = RecordingError("Video writer still finishing after %d ms; "
                                       "its buffers remain owned by the background thread" %
                                       RECORDING_STOP_TIMEOUT_MS)
            elif self.writer.error is not None and error is None:
                error = RecordingError(self.writer.error)
            if self.writer.done:
                print("[RECORD] closed %s: video_frames=%d file_bytes=%d directory_bytes=%d "
                      "dropped=%d peak_pending=%d" %
                      (self.writer.path, self.frames, self.writer.bytes,
                       self.writer.existing_bytes + self.writer.bytes, self.dropped, self.queue.peak))
        if error is not None:
            raise error


class OutputMonitor:
    """Per-frame Lab mean of the central quarter of the ISP output, before overlays.

    A purple flash seen in the IDE but never here comes from the preview path.
    Purple is a > 0 with b < 0. Keep the scene still: motion also changes the mean.
    """

    JUMP = 8               # Lab units between consecutive frames counted as a jump.
    JUMP_LINES = 3         # Jump details printed per one-second window.

    def __init__(self):
        self.previous = None
        self.total_jumps = 0
        self.frames = 0
        self.reset_window()

    def reset_window(self):
        self.low = None
        self.high = None
        self.jumps = 0

    def process(self, frame, frame_index):
        width, height = frame.width(), frame.height()
        stats = frame.get_statistics(roi=(width // 4, height // 4, width // 2, height // 2))
        value = (stats.l_mean(), stats.a_mean(), stats.b_mean())
        self.frames += 1
        previous = self.previous
        self.previous = value
        if self.low is None:
            self.low = list(value)
            self.high = list(value)
        else:
            for i in range(3):
                self.low[i] = min(self.low[i], value[i])
                self.high[i] = max(self.high[i], value[i])
        if previous is not None and max(abs(value[i] - previous[i])
                                        for i in range(3)) >= self.JUMP:
            self.jumps += 1
            self.total_jumps += 1
            if self.jumps <= self.JUMP_LINES:
                print("[ISP] jump at frame %d: Lab %s -> %s" % (frame_index, previous, value))

    def report_window(self):
        if self.low is not None:
            print("[ISP] center Lab L=%d..%d a=%d..%d b=%d..%d jumps=%d" %
                  (self.low[0], self.high[0], self.low[1], self.high[1],
                   self.low[2], self.high[2], self.jumps))
        self.reset_window()

    def finish(self):
        self.report_window()
        print("[ISP] output jumps=%d over %d frames; a flash seen in the IDE without "
              "jumps here comes from the preview path" % (self.total_jumps, self.frames))


def checked_color_frame(sensor):
    from image import RGB565

    width, height = output_size()
    frame = capture_color_frame(sensor, FRAME_TIMEOUT_MS)
    if frame is None or (frame.width(), frame.height()) != (width, height):
        raise RuntimeError("Expected a nonempty %dx%d frame" % (width, height))
    if frame.format() != RGB565:
        raise RuntimeError("Expected an RGB565 color frame")
    return frame


def capture_frames(sensor, display, sampler=None, monitor=None, detector=None, recorder=None, controls=None):
    """Count every frame; copies to the preview display are rate-limited by PREVIEW_FPS."""
    width, height = output_size()
    preview_interval_ms = 1000 // PREVIEW_FPS
    frames = 0
    report_frames = 0
    started = time.ticks_ms()
    reported = started
    previewed = None
    elapsed_ms = 0
    while TEST_SECONDS == 0 or elapsed_ms < TEST_SECONDS * 1000:
        os.exitpoint()
        if recorder is not None:
            recorder.poll()
        frame = checked_color_frame(sensor)
        received = time.ticks_ms()  # Stamp the snapshot before sampling or detection.
        frames += 1
        if frames == 1:
            print("[PASS] first frame: %dx%d RGB565" % (width, height))
        if controls is not None:
            controls.poll()
        if sampler is not None:
            sampler.process(frame, frames)
        if monitor is not None:
            monitor.process(frame, frames)
        if detector is not None:
            if sampler is not None:
                detector.apply_sample(sampler.result)
            preview_due = display is not None and (
                previewed is None or time.ticks_diff(time.ticks_ms(), previewed) >= preview_interval_ms)
            detector.process(frame, frames, received, preview_due=preview_due)
        now = time.ticks_ms()
        if display is not None and (
                previewed is None or time.ticks_diff(now, previewed) >= preview_interval_ms):
            if detector is not None:
                detector.draw(frame)
            if sampler is not None:
                sampler.draw(frame, interactive=controls is None or controls.mode == "color")
            if detector is not None:
                detector.draw_search_rois(frame)
            if controls is not None:
                controls.draw(frame)
            display.show_image(frame)
            previewed = now
        if recorder is not None:
            recorder.poll()
        now = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now, started)
        report_ms = time.ticks_diff(now, reported)
        if report_ms >= 1000:
            print("[INFO] frames=%d received_fps=%.1f elapsed_ms=%d" %
                  (frames, (frames - report_frames) * 1000.0 / report_ms, elapsed_ms))
            if monitor is not None:
                monitor.report_window()
            if detector is not None:
                detector.report()
            if recorder is not None:
                recorder.report()
            reported = now
            report_frames = frames
    if monitor is not None:
        monitor.finish()
    return frames, elapsed_ms


def release_resources(sensor, display, sampler=None, detector=None, recorder=None, controls=None):
    actions = []
    if controls is not None:
        actions.append(("preview controls", controls.deinit))
    if detector is not None:
        actions.append(("Python detector", detector.close))
    if sampler is not None:
        actions.append(("color touch", sampler.deinit))
    if sensor is not None:
        # Explicit cleanup must report native failures; is_del=True would
        # suppress them and could falsely report a completed capture.
        actions.append(("sensor", lambda: sensor.stop()))
    if recorder is not None:
        actions.append(("video recording", recorder.close))
    if display is not None:
        actions.append(("display", display.deinit))
    if actions:
        actions.append(("exitpoint", lambda: os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)))
        actions.append(("settle", lambda: time.sleep_ms(100)))
    first_error = None
    for name, action in actions:
        try:
            print("[CLEANUP] %s" % name)
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
    detector = None
    recorder = None
    controls = None
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
        if RAW_CHECK_ENABLED:
            from mpp.video_struct import PIXEL_FORMAT_RGB_BAYER_10BPP

            # A Bayer channel uses the full acquisition window and bypasses the ISP.
            sensor.set_framesize(width=640, height=480)
            sensor.set_pixformat(PIXEL_FORMAT_RGB_BAYER_10BPP)
        else:
            width, height = output_size()
            if USE_VGA_MODE:
                sensor.set_framesize(width=width, height=height, crop=True)
            else:
                if NATIVE_FULL_FRAME:
                    check_native_width_support()
                # Native: every pixel. Scaled: no crop window, the ISP scales the whole view.
                sensor.set_framesize(width=width, height=height)
            sensor.set_pixformat(Sensor.RGB565)
        stage = "firmware ISP configuration"
        # reset() restores the ISP flags; run() submits them to VICAP. These
        # settings must be applied between those calls, never after run().
        sensor.auto_exposure(False)
        # set_auto_whitebal() and set_auto_gain() are unimplemented wrappers
        # in this firmware; the ISP pipe flags are the working controls.
        white_balance = 1 if AUTO_WHITE_BALANCE else 0
        try:
            sensor._dev_attr.pipe_ctrl.bits.awb_enable = white_balance
            sensor._dev_attr.pipe_ctrl.bits.wb_enable = white_balance
            # The full-mode profile enables temporal NR, which blends past frames.
            sensor._dev_attr.pipe_ctrl.bits.dnr3_enable = 1 if ISP_3DNR_ENABLED else 0
        except AttributeError:
            raise RuntimeError("Firmware lacks ISP AWB/WB controls; update the Dart CanMV firmware")
        if RAW_CHECK_ENABLED:
            print("[INFO] RAW check: AE=off; output is sensor RAW10, no ISP processing; "
                  "IDE preview and color sampling are not used")
        else:
            state = "on" if AUTO_WHITE_BALANCE else "off"
            print("[INFO] ISP configuration: AE=off AWB=%s WB=%s 3DNR=%s" %
                  (state, state, "on" if ISP_3DNR_ENABLED else "off"))
            if not AUTO_WHITE_BALANCE:
                print("[INFO] AWB off keeps the profile's fixed WB gains (R 1.12, B 2.51); "
                      "expect a blue-violet cast under this lighting")
            if not USE_VGA_MODE:
                print("[INFO] sensor 1080x1280 full view -> %dx%d RGB565 (%s)" %
                      (output_size() + ("native, rows packed from the 1088-pixel ISP pitch"
                                        if NATIVE_FULL_FRAME else "ISP-scaled, uncropped",)))

        if RECORDING_ENABLED:
            stage = "video recording initialization"
            recorder = VideoRecorder()
            recorder.prepare(sensor)

        if PREVIEW_ENABLED and not RAW_CHECK_ENABLED:
            stage = "IDE preview initialization"
            display = DisplayPreview(*output_size())
            print("[INFO] IDE preview: virtual display %dx%d, hardware JPEG; frames copied "
                  "at most %d fps" % (output_size() + (PREVIEW_FPS,)))

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
        if RAW_CHECK_ENABLED:
            stage = "RAW sensor check"
            raw_clean = RawCheck(sensor).run(RAW_CHECK_FRAMES, FRAME_TIMEOUT_MS)
            completed = True
            print("[%s] RAW check complete; set RAW_CHECK_ENABLED=False for the "
                  "capture test" % ("INFO" if raw_clean is None else
                                    "PASS" if raw_clean else "FAIL"))
            return
        if TEST_SECONDS == 0:
            print("[INFO] Continuous color preview; stop in the IDE to release resources")
        print("[INFO] received_fps includes configured frame processing. Disable "
              "preview, color sampling, output monitoring, detection and recording for capture-only timing")

        if COLOR_SAMPLING_ENABLED:
            stage = "color sampler initialization"
            sampler = ColorSampler()
            sampler.start(interactive=not DETECTION_ENABLED)
        if DETECTION_ENABLED:
            stage = "Python detector initialization"
            detector = ColorTargetDetector(*output_size())
            if PREVIEW_ENABLED:
                stage = "preview controls initialization"
                controls = PreviewControls(detector, sampler)
                controls.start()
        stage = "frame capture"
        monitor = None
        if OUTPUT_MONITOR_ENABLED:
            monitor = OutputMonitor()
            print("[ISP] Monitoring the center Lab mean of every ISP frame; keep the "
                  "scene still. Purple is a > 0 with b < 0")
        frames, elapsed_ms = capture_frames(sensor, display, sampler, monitor, detector,
                                           recorder=recorder, controls=controls)
        if recorder is not None:
            recorder.check_output()
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
        elif stage == "RAW sensor check":
            print("[HINT] The RAW path is a diagnostic; report this error and the "
                  "UART0 VICAP log, then retest with RAW_CHECK_ENABLED=False")
        elif isinstance(error, RecordingError) or stage == "video recording initialization":
            print("[HINT] Check SD access, recorder limits and firmware H.264 / thread support; "
                  "RECORDING_ENABLED=False disables recording")
        elif isinstance(error, DetectionError) or stage == "Python detector initialization":
            print("[HINT] Check detector configuration, Lab profiles and image API support; "
                  "DETECTION_ENABLED=False keeps the capture / color sampler available")
        elif stage not in ("configuration", "firmware ISP configuration"):
            print("[HINT] Stop other camera users; check the CSI0 four-lane "
                  "configuration, external MCLK and installed SC132GS ISP files")
        if DIAGNOSTICS_ENABLED and not isinstance(error, (DetectionError, RecordingError)):
            try:
                if stage == "MIPI / ISP stream start":
                    diagnose_startup(dart_board, error)
                elif stage == "frame capture":
                    diagnose_capture(sensor, dart_board)
            except Exception as diagnostic_error:
                print("[DIAG] diagnostics unavailable: %s" % diagnostic_error)
        raise
    finally:
        cleanup_error = release_resources(sensor, display, sampler, detector,
                                          recorder=recorder, controls=controls)
        if cleanup_error is not None and completed:
            raise cleanup_error

    print("[PASS] capture: frames=%d elapsed_ms=%d received_fps=%.1f; resources released" %
          (frames, elapsed_ms, frames * 1000.0 / max(elapsed_ms, 1)))


if __name__ == "__main__":
    main()
