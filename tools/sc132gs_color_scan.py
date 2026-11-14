"""SC132GS color sweeps for the Dart CanMV firmware; run in the CanMV IDE.

Keep the target and lighting still. Select one homogeneous ROI with two preview
clicks, or set COLOR_ROI. Run both one-variable sweeps with AE and AWB disabled.
CSV contains per-frame measurements; JSONL contains metadata and point summaries.
Colors and clipping are measured AFTER the ISP, not in sensor RAW. Other ISP
processing is preserved, so gain-dependent ISP changes remain observable.
"""

import json
import math
import os
import time


# ---------------- Edit these settings before running ----------------
USE_VGA_MODE = False                  # False: full view scaled to 480x640; True: 640x480 crop.
PREVIEW_ENABLED = True
PREVIEW_FPS = 15
COLOR_ROI = None                      # (x, y, width, height); None: select two corners in the IDE.
ROI_SELECTION_TIMEOUT_MS = 120000

GAIN_SWEEP_ENABLED = True
FIXED_EXPOSURE_US = 800
GAIN_STEPS = (1.0, 2.0, 4.0, 5.0, 8.0, 12.0, 16.0, 24.0, 28.0)  # Linear multipliers, not dB.
EXPOSURE_SWEEP_ENABLED = True
FIXED_ANALOG_GAIN = 1.0
EXPOSURE_MIN_US = 100
EXPOSURE_MAX_US = 300                  # Inclusive upper limit; never exceeded by generated steps.
EXPOSURE_STEP_US = 10
EXPOSURE_STEPS_US = None              # None: use range above; tuple/list: override with custom steps.

SETTLE_FRAMES = 12                    # Discard at least this many frames after EVERY setting.
SETTLE_MS = 300                       # Also wait this long while draining live frames.
SAMPLE_FRAMES = 30
FRAME_TIMEOUT_MS = 1000
RGB_MAX_SAMPLES_PER_FRAME = 1024       # Bounded uniform grid; Lab histogram uses the whole ROI.
RGB_CLIP_MIN = (248, 252, 248)         # Near full scale of RGB565 expanded to 8-bit RGB.
RGB_DARK_MAX = 8                      # A pixel is dark when ALL three channels are <= this.
MIN_LAB_CHROMA = 10.0                 # Below this, hue of the ROI mean is undefined/noisy.
RESULT_PREFIX = "/sdcard/sc132gs_color_scan"  # Unique numbered .csv/.jsonl pair; "": console only.


CSV_FIELDS = (
    "sweep", "point", "sample", "frame", "received_ms", "exposure_us_requested",
    "analog_gain_requested", "exposure_us_readback", "analog_gain_readback",
    "r_mean", "g_mean", "b_mean", "r_stdev", "g_stdev", "b_stdev",
    "l_mean", "a_mean", "b_lab_mean", "l_stdev", "a_stdev", "b_lab_stdev",
    "lab_hue_deg", "lab_chroma", "rgb_r_fraction", "rgb_g_fraction", "rgb_b_fraction",
    "r_clip_fraction", "g_clip_fraction", "b_clip_fraction", "any_clip_fraction",
    "dark_fraction", "rgb_sample_pixels",
)


def output_size():
    return (640, 480) if USE_VGA_MODE else (480, 640)


def validate_roi(roi):
    if not isinstance(roi, (tuple, list)) or len(roi) != 4 or any(type(v) is not int for v in roi):
        raise ValueError("COLOR_ROI must be (x, y, width, height) in output coordinates")
    x, y, width, height = roi
    fw, fh = output_size()
    if x < 0 or y < 0 or width < 4 or height < 4 or x + width > fw or y + height > fh:
        raise ValueError("COLOR_ROI must be at least 4x4 and fit inside %dx%d" % (fw, fh))
    return tuple(roi)


def positive_number(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def exposure_steps():
    if EXPOSURE_STEPS_US is not None:
        return EXPOSURE_STEPS_US
    for name, value in (("EXPOSURE_MIN_US", EXPOSURE_MIN_US),
                        ("EXPOSURE_MAX_US", EXPOSURE_MAX_US),
                        ("EXPOSURE_STEP_US", EXPOSURE_STEP_US)):
        if type(value) is not int or value < 1:
            raise ValueError("%s must be a positive integer in microseconds" % name)
    if EXPOSURE_MIN_US > EXPOSURE_MAX_US:
        raise ValueError("EXPOSURE_MIN_US must not exceed EXPOSURE_MAX_US")
    count = (EXPOSURE_MAX_US - EXPOSURE_MIN_US) // EXPOSURE_STEP_US + 1
    if count > 256:
        raise ValueError("Exposure sweep needs 1..256 steps; increase EXPOSURE_STEP_US")
    return tuple(range(EXPOSURE_MIN_US, EXPOSURE_MAX_US + 1, EXPOSURE_STEP_US))


def validate_config():
    for name, value in (("USE_VGA_MODE", USE_VGA_MODE), ("PREVIEW_ENABLED", PREVIEW_ENABLED),
                        ("GAIN_SWEEP_ENABLED", GAIN_SWEEP_ENABLED),
                        ("EXPOSURE_SWEEP_ENABLED", EXPOSURE_SWEEP_ENABLED)):
        if type(value) is not bool:
            raise ValueError("%s must be bool" % name)
    if not GAIN_SWEEP_ENABLED and not EXPOSURE_SWEEP_ENABLED:
        raise ValueError("Enable at least one sweep")
    for name, value in (("PREVIEW_FPS", PREVIEW_FPS), ("SAMPLE_FRAMES", SAMPLE_FRAMES),
                        ("SETTLE_FRAMES", SETTLE_FRAMES), ("FRAME_TIMEOUT_MS", FRAME_TIMEOUT_MS),
                        ("ROI_SELECTION_TIMEOUT_MS", ROI_SELECTION_TIMEOUT_MS),
                        ("RGB_MAX_SAMPLES_PER_FRAME", RGB_MAX_SAMPLES_PER_FRAME)):
        if type(value) is not int or value < 1:
            raise ValueError("%s must be a positive integer" % name)
    if type(SETTLE_MS) is not int or SETTLE_MS < 0:
        raise ValueError("SETTLE_MS must be a nonnegative integer")
    if type(FIXED_EXPOSURE_US) is not int or FIXED_EXPOSURE_US < 1:
        raise ValueError("FIXED_EXPOSURE_US must be a positive integer")
    if not positive_number(FIXED_ANALOG_GAIN) or not 1 <= FIXED_ANALOG_GAIN <= 28.546875:
        raise ValueError("FIXED_ANALOG_GAIN must be within 1..28.546875")
    for enabled, values, gain in ((GAIN_SWEEP_ENABLED, GAIN_STEPS, True),
                                  (EXPOSURE_SWEEP_ENABLED,
                                   exposure_steps() if EXPOSURE_SWEEP_ENABLED else (), False)):
        if not enabled:
            continue
        if not isinstance(values, (tuple, list)) or not values or len(values) > 256:
            raise ValueError("Each enabled sweep needs 1..256 steps")
        for value in values:
            if gain:
                if not positive_number(value) or not 1 <= value <= 28.546875:
                    raise ValueError("GAIN_STEPS must be finite linear multipliers within 1..28.546875")
            elif type(value) is not int or value < 1:
                raise ValueError("EXPOSURE_STEPS_US must contain positive integer microseconds")
    if (not isinstance(RGB_CLIP_MIN, (tuple, list)) or len(RGB_CLIP_MIN) != 3
            or any(type(v) is not int or not 1 <= v <= 255 for v in RGB_CLIP_MIN)):
        raise ValueError("RGB_CLIP_MIN must contain three levels within 1..255")
    if type(RGB_DARK_MAX) is not int or not 0 <= RGB_DARK_MAX < min(RGB_CLIP_MIN):
        raise ValueError("RGB_DARK_MAX must be below every clipping level")
    if not positive_number(MIN_LAB_CHROMA):
        raise ValueError("MIN_LAB_CHROMA must be finite and positive")
    if not isinstance(RESULT_PREFIX, str):
        raise ValueError("RESULT_PREFIX must be a string")
    if COLOR_ROI is not None:
        validate_roi(COLOR_ROI)
    elif not PREVIEW_ENABLED:
        raise ValueError("Set COLOR_ROI when PREVIEW_ENABLED=False")


def sweep_points():
    if GAIN_SWEEP_ENABLED:
        for index, gain in enumerate(GAIN_STEPS):
            yield "gain", index + 1, FIXED_EXPOSURE_US, gain
    if EXPOSURE_SWEEP_ENABLED:
        for index, exposure in enumerate(exposure_steps()):
            yield "exposure", index + 1, exposure, FIXED_ANALOG_GAIN


class Preview:
    def __init__(self):
        from media.display import Display

        self.display = Display
        self.last_ms = None
        width, height = output_size()
        try:
            Display.init(Display.VIRT, width=width, height=height, fps=60, to_ide=True)
        except BaseException:
            Display.deinit()
            raise

    def show(self, frame, text, roi=None, corner=None):
        now = time.ticks_ms()
        if self.last_ms is not None and time.ticks_diff(now, self.last_ms) < 1000 // PREVIEW_FPS:
            return
        # Call only after statistics: overlays must never enter the measurements.
        if roi is not None:
            frame.draw_rectangle(roi, color=(255, 255, 0), thickness=2)
        if corner is not None:
            frame.draw_cross(*corner, color=(255, 255, 0))
        frame.draw_string_advanced(8, 8, 16, text, color=(255, 255, 255))
        self.display.show_image(frame)
        self.last_ms = now

    def deinit(self):
        self.display.deinit()


class Capture:
    def __init__(self, sensor):
        self.sensor = sensor
        self.sequence = 0

    def next(self):
        import image

        os.exitpoint()
        frame = self.sensor.snapshot(timeout=FRAME_TIMEOUT_MS, auto_reboot=False)
        if frame is None or (frame.width(), frame.height()) != output_size() or frame.format() != image.RGB565:
            raise RuntimeError("Expected a fresh %dx%d RGB565 frame" % output_size())
        self.sequence += 1
        return frame


def select_roi(capture, preview):
    if COLOR_ROI is not None:
        return validate_roi(COLOR_ROI)
    from machine import TOUCH

    width, height = output_size()
    touch = TOUCH(TOUCH.DEV_IDE, range_x=width, range_y=height)
    corner = None
    started = time.ticks_ms()
    print("[SCAN] Click two opposite corners of one uniform target region; keep it still for both sweeps")
    try:
        while time.ticks_diff(time.ticks_ms(), started) < ROI_SELECTION_TIMEOUT_MS:
            frame = capture.next()
            for point in touch.read(5):
                if point.event != TOUCH.EVENT_DOWN:
                    continue
                pos = (max(0, min(width - 1, point.x)), max(0, min(height - 1, point.y)))
                if corner is None:
                    corner = pos
                else:
                    x, y = min(corner[0], pos[0]), min(corner[1], pos[1])
                    w, h = abs(corner[0] - pos[0]) + 1, abs(corner[1] - pos[1]) + 1
                    if w >= 4 and h >= 4:
                        return (x, y, w, h)
                    print("[SCAN] ROI must be at least 4x4; choose the second corner again")
            preview.show(frame, "Select two ROI corners", corner=corner)
        raise RuntimeError("ROI selection timed out; set COLOR_ROI or select it in the IDE")
    finally:
        touch.deinit()


def set_point(sensor, exposure, gain):
    # Reassert BOTH parameters so the fixed variable is restored between sweeps.
    if sensor.again(gain) != 0:
        raise RuntimeError("Setting analog gain %.6g failed" % gain)
    if not sensor.exposure(exposure):
        raise RuntimeError("Setting exposure %d us failed; check the mode's exposure range" % exposure)


def read_settings(sensor):
    exposure, raw_gain = sensor.exposure(), sensor.again()
    # CanMV returns exposure in microseconds, but again() returns a
    # k_sensor_gain structure. SC132GS linear mode uses its first gain slot.
    gain = raw_gain
    if hasattr(raw_gain, "gain"):
        try:
            gain = raw_gain.gain[0]
        except (IndexError, TypeError):
            gain = None
    if not positive_number(exposure) or not positive_number(gain):
        raise RuntimeError("Invalid sensor readback: exposure()=%r, again()=%r, gain[0]=%r; "
                           "expected positive exposure in us and k_sensor_gain.gain[0] or numeric gain" %
                           (exposure, raw_gain, gain))
    return (exposure, gain)


def settle(capture, preview, roi, text):
    started, count = time.ticks_ms(), 0
    while count < SETTLE_FRAMES or time.ticks_diff(time.ticks_ms(), started) < SETTLE_MS:
        frame = capture.next()
        count += 1
        if preview is not None:
            preview.show(frame, "Settling " + text, roi)
    return count


def sample_step(roi):
    width, height = roi[2:]
    step = max(1, int(math.sqrt(width * height / RGB_MAX_SAMPLES_PER_FRAME)))
    while ((width + step - 1) // step) * ((height + step - 1) // step) > RGB_MAX_SAMPLES_PER_FRAME:
        step += 1
    return step


def measure(frame, roi, step):
    histogram = frame.get_histogram(roi=roi)
    stats = histogram.get_statistics()
    lab = (stats.l_mean(), stats.a_mean(), stats.b_mean())
    lab_sd = (stats.l_stdev(), stats.a_stdev(), stats.b_stdev())
    lo, hi = histogram.get_percentile(0.05), histogram.get_percentile(0.95)
    chroma = math.sqrt(lab[1] ** 2 + lab[2] ** 2)
    hue = (math.atan2(lab[2], lab[1]) * 180 / math.pi) % 360 if chroma >= MIN_LAB_CHROMA else None
    sums, squares, clip = [0, 0, 0], [0, 0, 0], [0, 0, 0]
    count, any_clip, dark = 0, 0, 0
    x0, y0, width, height = roi
    for y in range(y0, y0 + height, step):
        os.exitpoint()
        for x in range(x0, x0 + width, step):
            pixel = frame.get_pixel(x, y, rgbtuple=True)
            clipped = False
            for c in range(3):
                value = pixel[c]
                sums[c] += value
                squares[c] += value * value
                if value >= RGB_CLIP_MIN[c]:
                    clip[c] += 1
                    clipped = True
            any_clip += int(clipped)
            dark += int(max(pixel) <= RGB_DARK_MAX)
            count += 1
    rgb = [v / count for v in sums]
    rgb_sd = [math.sqrt(max(0.0, squares[c] / count - rgb[c] ** 2)) for c in range(3)]
    total = sum(rgb)
    return {
        "rgb_mean": rgb, "rgb_stdev": rgb_sd, "lab_mean": lab, "lab_stdev": lab_sd,
        "lab_p05": (lo.l_value(), lo.a_value(), lo.b_value()),
        "lab_p95": (hi.l_value(), hi.a_value(), hi.b_value()),
        "lab_hue_deg": hue, "lab_chroma": chroma,
        "rgb_fraction": [v / total if total > 0 else None for v in rgb],
        "channel_clip_fraction": [v / count for v in clip],
        "any_clip_fraction": any_clip / count, "dark_fraction": dark / count,
        "rgb_sample_pixels": count,
    }


class PointStatistics:
    def __init__(self):
        self.frames = 0
        self.sums, self.second = [0.0] * 6, [0.0] * 6
        self.low, self.high = [100, 127, 127], [0, -128, -128]
        self.clip, self.dark, self.any_clip = [0.0] * 3, 0.0, 0.0
        self.hue_x, self.hue_y, self.hue_frames = 0.0, 0.0, 0

    def add(self, value):
        means = tuple(value["rgb_mean"]) + tuple(value["lab_mean"])
        stdevs = tuple(value["rgb_stdev"]) + tuple(value["lab_stdev"])
        for c in range(6):
            self.sums[c] += means[c]
            self.second[c] += stdevs[c] ** 2 + means[c] ** 2
        for c in range(3):
            self.low[c] = min(self.low[c], value["lab_p05"][c])
            self.high[c] = max(self.high[c], value["lab_p95"][c])
            self.clip[c] += value["channel_clip_fraction"][c]
        self.any_clip += value["any_clip_fraction"]
        self.dark += value["dark_fraction"]
        if value["lab_hue_deg"] is not None:
            angle = value["lab_hue_deg"] * math.pi / 180
            self.hue_x += math.cos(angle)
            self.hue_y += math.sin(angle)
            self.hue_frames += 1
        self.frames += 1

    def result(self):
        means = [v / self.frames for v in self.sums]
        sd = [math.sqrt(max(0.0, self.second[c] / self.frames - means[c] ** 2)) for c in range(6)]
        length = math.sqrt(self.hue_x ** 2 + self.hue_y ** 2)
        hue = (math.atan2(self.hue_y, self.hue_x) * 180 / math.pi) % 360 if length > 1e-6 else None
        return {
            "sample_frames": self.frames, "rgb_mean": means[:3], "rgb_stdev": sd[:3],
            "lab_mean": means[3:], "lab_stdev": sd[3:],
            "lab_p05_envelope": self.low, "lab_p95_envelope": self.high,
            "lab_hue_mean_deg": hue, "lab_hue_valid_frames": self.hue_frames,
            "lab_hue_concentration": length / self.hue_frames if self.hue_frames else None,
            "channel_clip_fraction": [v / self.frames for v in self.clip],
            "any_clip_fraction": self.any_clip / self.frames, "dark_fraction": self.dark / self.frames,
        }


class Results:
    def __init__(self, metadata):
        self.csv, self.jsonl = None, None
        self.completed_points = 0
        try:
            if RESULT_PREFIX:
                for index in range(10000):
                    stem = "%s_%04d" % (RESULT_PREFIX, index)
                    if not self.exists(stem + ".csv") and not self.exists(stem + ".jsonl"):
                        break
                else:
                    raise RuntimeError("No unused result filename; change RESULT_PREFIX")
                self.csv = open(stem + ".csv", "w")
                self.jsonl = open(stem + ".jsonl", "w")
                self.csv.write(",".join(CSV_FIELDS) + "\n")
                print("[SCAN] Output: %s.csv and %s.jsonl" % (stem, stem))
            else:
                print("[SCAN_CSV] " + ",".join(CSV_FIELDS))
            self.event(metadata)
        except BaseException:
            self.close()
            raise

    @staticmethod
    def exists(path):
        try:
            os.stat(path)
            return True
        except OSError as error:
            if error.args and error.args[0] == 2:
                return False
            raise

    def event(self, value):
        encoded = json.dumps(value)
        print("[SCAN_JSON] " + encoded)
        if self.jsonl is not None:
            self.jsonl.write(encoded + "\n")
            self.jsonl.flush()
        if self.csv is not None:
            self.csv.flush()

    def frame(self, context, sample, sequence, received, value):
        row = [context["sweep"], context["point"], sample, sequence, received,
               context["exposure_us_requested"], context["analog_gain_requested"],
               context["exposure_us_readback"], context["analog_gain_readback"]]
        for key in ("rgb_mean", "rgb_stdev", "lab_mean", "lab_stdev"):
            row.extend(value[key])
        row.extend((value["lab_hue_deg"], value["lab_chroma"]))
        row.extend(value["rgb_fraction"])
        row.extend(value["channel_clip_fraction"])
        row.extend((value["any_clip_fraction"], value["dark_fraction"], value["rgb_sample_pixels"]))
        encoded = ",".join("" if v is None else str(v) for v in row)
        if self.csv is not None:
            self.csv.write(encoded + "\n")
        else:
            print("[SCAN_CSV] " + encoded)

    def close(self):
        error = None
        for stream in (self.csv, self.jsonl):
            if stream is not None:
                try:
                    stream.close()
                except Exception as caught:
                    error = error or caught
        self.csv, self.jsonl = None, None
        if error is not None:
            raise error


def scan(capture, preview, roi, results):
    step = sample_step(roi)
    fixed_readbacks = {}
    for sweep, index, exposure, gain in sweep_points():
        text = "%s #%d exp=%dus gain=%.3fx" % (sweep, index, exposure, gain)
        print("[SCAN] " + text)
        set_point(capture.sensor, exposure, gain)
        discarded = settle(capture, preview, roi, text)
        actual_exp, actual_gain = read_settings(capture.sensor)
        context = {"sweep": sweep, "point": index, "exposure_us_requested": exposure,
                   "analog_gain_requested": gain, "exposure_us_readback": actual_exp,
                   "analog_gain_readback": actual_gain}
        results.event(dict(context, event="point_start", discarded_frames=discarded))
        fixed_value = actual_exp if sweep == "gain" else actual_gain
        tolerance = 0.01 if sweep == "gain" else 0.00001
        if sweep not in fixed_readbacks:
            fixed_readbacks[sweep] = fixed_value
        elif abs(fixed_value - fixed_readbacks[sweep]) > tolerance:
            raise RuntimeError("The fixed parameter changed between %s sweep points" % sweep)
        stats = PointStatistics()
        started = time.ticks_ms()
        first_frame = capture.sequence + 1
        for sample in range(1, SAMPLE_FRAMES + 1):
            frame = capture.next()
            received = time.ticks_ms()
            value = measure(frame, roi, step)
            stats.add(value)
            results.frame(context, sample, capture.sequence, received, value)
            if preview is not None:
                preview.show(frame, "%s %d/%d" % (text, sample, SAMPLE_FRAMES), roi)
        end_exp, end_gain = read_settings(capture.sensor)
        stable = abs(end_exp - actual_exp) <= 0.01 and abs(end_gain - actual_gain) <= 0.00001
        summary = dict(context, event="point_end", settings_stable=stable,
                       exposure_us_readback_end=end_exp, analog_gain_readback_end=end_gain,
                       first_frame=first_frame, last_frame=capture.sequence,
                       elapsed_ms=time.ticks_diff(time.ticks_ms(), started), rgb_sample_step=step)
        summary.update(stats.result())
        results.event(summary)
        if not stable:
            raise RuntimeError("Exposure/gain changed during sampling; point is invalid")
        results.completed_points += 1
    return results.completed_points


def main():
    sensor, preview, results = None, None, None
    status, failure, completed = "failed", None, 0
    try:
        validate_config()
        import dart_board
        from media.sensor import Sensor

        sensor = dart_board.camera_sensor(vga=USE_VGA_MODE)
        sensor.reset()
        width, height = output_size()
        if USE_VGA_MODE:
            sensor.set_framesize(width=width, height=height, crop=True)
        else:
            sensor.set_framesize(width=width, height=height)
        sensor.set_pixformat(Sensor.RGB565)
        sensor.auto_exposure(False)
        # reset() restores these flags; run() submits them to the ISP. Do not use
        # set_auto_whitebal()/set_auto_gain(): they are placeholders in this firmware.
        bits = sensor._dev_attr.pipe_ctrl.bits
        bits.awb_enable = 0
        bits.wb_enable = 0
        bits.dnr3_enable = 0
        if bits.ae_enable != 0:
            raise RuntimeError("Disabling automatic exposure/gain failed")
        pipe = {name: getattr(bits, name) for name in
                ("ae_enable", "awb_enable", "wb_enable", "dnr3_enable", "ccm_enable", "gc_enable", "cproc_enable")}
        print("[SCAN] Configured AE=off AWB=off WB=off 3DNR=off; other ISP stages preserved")
        if PREVIEW_ENABLED:
            preview = Preview()
        sensor.run()
        # A valid sensor handle exists only after run(). Start ROI selection at
        # the common reference point; each scan point will then set both values.
        set_point(sensor, FIXED_EXPOSURE_US, FIXED_ANALOG_GAIN)
        capture = Capture(sensor)
        settle(capture, preview, COLOR_ROI, "ROI reference")
        roi = select_roi(capture, preview)
        results = Results({
            "event": "run_start", "version": 1, "sensor": "SC132GS",
            "started_ms": time.ticks_ms(),
            "sensor_mode": "vga" if USE_VGA_MODE else "full", "output_size": output_size(),
            "roi": roi, "pixel_format": "RGB565", "isp_pipe_ctrl_configured": pipe,
            "fixed_exposure_us": FIXED_EXPOSURE_US, "fixed_analog_gain": FIXED_ANALOG_GAIN,
            "gain_steps": GAIN_STEPS if GAIN_SWEEP_ENABLED else (),
            "exposure_steps_us": exposure_steps() if EXPOSURE_SWEEP_ENABLED else (),
            "settle_frames_min": SETTLE_FRAMES, "settle_ms_min": SETTLE_MS,
            "sample_frames_per_point": SAMPLE_FRAMES, "rgb_sample_step": sample_step(roi),
            "rgb_max_samples_per_frame": RGB_MAX_SAMPLES_PER_FRAME,
            "rgb_clip_min": RGB_CLIP_MIN, "rgb_dark_max": RGB_DARK_MAX,
            "min_lab_chroma": MIN_LAB_CHROMA,
            "measurement_notes": "ISP output; Lab uses full ROI, RGB/clipping use a uniform grid; "
                                 "hue is of each frame's ROI mean; readback is not frame-synchronized; "
                                 "RGB fractions use nonlinear ISP codes, not calibrated linear RGB",
        })
        completed = scan(capture, preview, roi, results)
        status = "complete"
    except KeyboardInterrupt:
        status = "interrupted"
    except Exception as error:
        if str(error) == "IDE interrupt":
            status = "interrupted"
        else:
            failure = error
    finally:
        # Always attempt every cleanup, retaining the original scan failure.
        actions = []
        if sensor is not None:
            actions.append(("sensor", sensor.stop))
        if preview is not None:
            actions.append(("preview", preview.deinit))
        actions.append(("IDE exit", lambda: os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)))
        actions.append(("release delay", lambda: time.sleep_ms(100)))
        for name, action in actions:
            try:
                action()
            except Exception as error:
                print("[FAIL] cleanup %s: %s" % (name, error))
                failure = failure or error
                status = "failed"
        if results is not None:
            completed = results.completed_points
            try:
                results.event({"event": "run_end", "status": status, "completed_points": completed,
                               "error": str(failure) if failure is not None else None})
            except Exception as error:
                failure = failure or error
            finally:
                try:
                    results.close()
                except Exception as error:
                    failure = failure or error
    if failure is not None:
        print("[FAIL] color scan: %s; completed points remain in the result files" % failure)
        raise failure
    if status == "interrupted":
        print("[STOP] Color scan interrupted; partial results retained")
    else:
        print("[PASS] Color scan: %d points; resources released" % completed)


if __name__ == "__main__":
    main()
