"""SC132GS RAW10 white-balance calibration for the Dart CanMV firmware.

The script measures the four Bayer phases before ISP processing.  Run it in the
CanMV IDE with the lens and target kept still:

* ``CALIBRATION_STAGE = "both"``: cover the lens, press Enter, then expose a
  matte white/18% grey target and press Enter again.  This is the normal mode.
* ``CALIBRATION_STAGE = "black"``: save only the covered-lens black reference.
* ``CALIBRATION_STAGE = "white"``: load the saved black reference and measure
  the target.  This is useful when changing the target requires restarting the
  script.

The result is written to ``RESULT_PATH``.  ``manual_gain`` is ordered for the
SC132GS ISP CManualWb block as ``[R, Gr, Gb, B]``.  The raw phase measurements
in the result are ordered ``[B, G1, G2, R]`` (BGGR).

This produces a gain estimate; it does not modify the ISP scene JSON in /bin.
Copy the reported ``manual_gain`` values into the matching SC132GS manual scene,
then validate with a grey card and a colour chart.  The RAW path is intentional:
RGB565 has already passed demosaic, CCM, gamma and other ISP stages.
"""

import json
import math
import os
import time


# ---------------- Edit these settings before running ----------------
USE_VGA_MODE = False
CALIBRATION_STAGE = "both"       # "black", "white" or "both"
EXPOSURE_US = 5000
ANALOG_GAIN = 1.0
WARMUP_FRAMES = 8
SAMPLE_FRAMES = 30
FRAME_TIMEOUT_MS = 1000
GRID_STEP = 16                    # One 2x2 Bayer cell per sampled grid point.
ROI_FRACTION = 0.60               # Use the central area; avoid lens/shading edges.
CLIP_HEADROOM = 8                 # Reject codes at/above full_scale - this value.
MAX_CLIP_FRACTION = 0.01          # A clipped target cannot produce a WB estimate.
MIN_SIGNAL_FRACTION = 0.10        # Require at least 10% of the range above black.
MAX_GREEN_MISMATCH = 0.10         # Warn if Gr and Gb differ by more than 10%.
MAX_FRAME_CV = 0.03               # Warn about flicker or unstable exposure.
MAX_WB_GAIN = 8.0                 # Warn when a result is probably not trustworthy.

BLACK_RESULT_PATH = "/sdcard/sc132gs_wb_black.json"
RESULT_PATH = "/sdcard/sc132gs_wb_calibration.json"

# The current SC132GS manual profile contains these fixed values.  The script
# reports both replacement values and a correction relative to this profile.
CURRENT_PROFILE_GAIN = (1.1226616, 1.0, 1.0, 2.511838)

RAW_PHASE_NAMES = ("B", "G1", "G2", "R")
MANUAL_GAIN_NAMES = ("R", "Gr", "Gb", "B")
RAW10_FULL_SCALE = 1023
RAW12_FULL_SCALE = 4095


def positive_number(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def validate_config():
    if CALIBRATION_STAGE not in ("black", "white", "both"):
        raise ValueError("CALIBRATION_STAGE must be 'black', 'white' or 'both'")
    for name, value in (("EXPOSURE_US", EXPOSURE_US), ("WARMUP_FRAMES", WARMUP_FRAMES),
                        ("SAMPLE_FRAMES", SAMPLE_FRAMES), ("FRAME_TIMEOUT_MS", FRAME_TIMEOUT_MS),
                        ("GRID_STEP", GRID_STEP)):
        if type(value) is not int or value < 1:
            raise ValueError("%s must be a positive integer" % name)
    if not positive_number(ANALOG_GAIN):
        raise ValueError("ANALOG_GAIN must be a finite positive number")
    if not 0.01 <= ROI_FRACTION <= 1.0:
        raise ValueError("ROI_FRACTION must be within 0.01..1.0")
    if GRID_STEP < 2:
        raise ValueError("GRID_STEP must be at least 2")
    if CLIP_HEADROOM < 1 or MAX_CLIP_FRACTION < 0 or MAX_CLIP_FRACTION >= 1:
        raise ValueError("invalid clipping limits")
    if MIN_SIGNAL_FRACTION <= 0 or MIN_SIGNAL_FRACTION >= 1:
        raise ValueError("MIN_SIGNAL_FRACTION must be within 0..1")
    if not isinstance(BLACK_RESULT_PATH, str) or not BLACK_RESULT_PATH.startswith("/"):
        raise ValueError("BLACK_RESULT_PATH must be an absolute path")
    if not isinstance(RESULT_PATH, str) or not RESULT_PATH.startswith("/"):
        raise ValueError("RESULT_PATH must be an absolute path")


def calculate_wb_gains(target_mean, black_mean, full_scale=RAW10_FULL_SCALE):
    """Return raw signals and ISP gains for a BGGR neutral target.

    ``target_mean`` and ``black_mean`` are ordered [B, G1, G2, R].  The
    returned manual gains are ordered [R, Gr, Gb, B], matching the SC132GS
    ISP scene files.
    """
    if len(target_mean) != 4 or len(black_mean) != 4:
        raise ValueError("target_mean and black_mean must contain four phases")
    signal = [float(target_mean[i]) - float(black_mean[i]) for i in range(4)]
    if any(not math.isfinite(value) or value <= 0 for value in signal):
        raise ValueError("target signal must be positive after black-level subtraction")
    green = (signal[1] + signal[2]) / 2.0
    if not math.isfinite(green) or green <= 0:
        raise ValueError("green signal must be positive")
    gains = [green / signal[3], 1.0, 1.0, green / signal[0]]
    if any(not math.isfinite(value) or value <= 0 for value in gains):
        raise ValueError("computed WB gain is invalid")
    signal_level = sum(signal) / 4.0
    black_level = sum(float(value) for value in black_mean) / 4.0
    signal_fraction = signal_level / max(float(full_scale) - black_level, 1.0)
    green_mismatch = abs(signal[1] - signal[2]) / green
    return {
        "black_mean": [float(value) for value in black_mean],
        "target_mean": [float(value) for value in target_mean],
        "signal": signal,
        "green_signal": green,
        "signal_fraction": signal_fraction,
        "green_mismatch": green_mismatch,
        "manual_gain": gains,
        "current_profile_gain": list(CURRENT_PROFILE_GAIN),
        "correction_relative_to_current_profile": [
            gains[i] / CURRENT_PROFILE_GAIN[i] for i in range(4)
        ],
    }


def _exitpoint():
    """Yield to the CanMV scheduler without making host-side imports required."""
    try:
        os.exitpoint()
    except AttributeError:
        pass


def _write_json(path, value):
    stream = open(path, "w")
    try:
        stream.write(json.dumps(value))
        stream.write("\n")
        stream.flush()
    finally:
        stream.close()


def _exists(path):
    try:
        os.stat(path)
        return True
    except OSError as error:
        if error.args and error.args[0] == 2:
            return False
        raise


def _read_json(path):
    stream = open(path, "r")
    try:
        return json.loads(stream.read())
    finally:
        stream.close()


class Statistics:
    """Bounded multi-frame statistics for four RAW Bayer phases."""

    def __init__(self):
        self.frames = 0
        self.cells = 0
        self.sums = [0.0] * 4
        self.squares = [0.0] * 4
        self.clipped_cells = 0
        self.frame_levels = []
        self.width = 0
        self.height = 0
        self.stride = 0
        self.full_scale = RAW10_FULL_SCALE
        self.roi = None

    def add(self, sample):
        self.frames += 1
        self.cells += sample["cells"]
        for index in range(4):
            value = sample["mean"][index]
            self.sums[index] += value * sample["cells"]
            # The sample contains per-cell second moments, so pooled variance
            # remains meaningful when frame brightness changes slightly.
            self.squares[index] += sample["second_moment"][index] * sample["cells"]
        self.clipped_cells += sample["clipped_cells"]
        self.frame_levels.append(sample["level"])
        self.width, self.height, self.stride = sample["width"], sample["height"], sample["stride"]
        self.full_scale = sample["full_scale"]
        self.roi = sample["roi"]

    def result(self):
        if self.frames == 0 or self.cells == 0:
            raise ValueError("no RAW samples collected")
        mean = [value / self.cells for value in self.sums]
        stdev = [math.sqrt(max(0.0, self.squares[i] / self.cells - mean[i] ** 2))
                 for i in range(4)]
        level = sum(self.frame_levels) / len(self.frame_levels)
        frame_cv = 0.0
        if level > 0 and len(self.frame_levels) > 1:
            frame_variance = sum((value - level) ** 2 for value in self.frame_levels) / len(self.frame_levels)
            frame_cv = math.sqrt(max(0.0, frame_variance)) / level
        return {
            "frames": self.frames,
            "cells_per_frame": self.cells // self.frames,
            "mean_code": mean,
            "stdev_code": stdev,
            "clip_fraction": self.clipped_cells / float(self.cells),
            "frame_level_mean": level,
            "frame_level_cv": frame_cv,
            "width": self.width,
            "height": self.height,
            "stride": self.stride,
            "full_scale": self.full_scale,
            "roi": self.roi,
            "phase_order": list(RAW_PHASE_NAMES),
        }


class Raw10Collector:
    """Read SC132GS RAW10 frames before demosaic and ISP processing."""

    def __init__(self, sensor):
        import uctypes
        from mpp.sys import kd_mpi_sys_mmap, kd_mpi_sys_munmap
        from mpp.vicap import (VICAP_DUMP_RAW, kd_mpi_vicap_dump_frame,
                               kd_mpi_vicap_dump_release)
        from mpp.video_struct import (PIXEL_FORMAT_RGB_BAYER_10BPP,
                                      PIXEL_FORMAT_RGB_BAYER_12BPP, k_video_frame_info)

        self.uctypes = uctypes
        self.device = sensor._dev_id
        self.mmap = kd_mpi_sys_mmap
        self.munmap = kd_mpi_sys_munmap
        self.dump = kd_mpi_vicap_dump_frame
        self.release = kd_mpi_vicap_dump_release
        self.dump_raw = VICAP_DUMP_RAW
        self.frame_info = k_video_frame_info
        self.full_scales = {
            PIXEL_FORMAT_RGB_BAYER_10BPP: RAW10_FULL_SCALE,
            PIXEL_FORMAT_RGB_BAYER_12BPP: RAW12_FULL_SCALE,
        }

    def capture(self, timeout_ms):
        info = self.frame_info()
        result = None
        virt = 0
        size = 0
        ret = self.dump(self.device, 0, self.dump_raw, info, timeout_ms)
        if ret != 0:
            raise RuntimeError("RAW dump returned %d" % ret)
        try:
            frame = info.v_frame
            width = int(frame.width)
            height = int(frame.height)
            stride = int(frame.stride[0])
            full_scale = self.full_scales.get(frame.pixel_format)
            if width < 4 or height < 4 or width % 2 or height % 2 or full_scale is None:
                raise RuntimeError("unexpected RAW frame %dx%d format=%d" %
                                   (width, height, frame.pixel_format))
            row_bytes = max(stride, width * 2)
            size = row_bytes * height
            virt = self.mmap(frame.phys_addr[0], size)
            if not virt:
                raise RuntimeError("RAW frame mmap failed")
            row = self._rows(virt, row_bytes, width)
            result = self._measure(row, width, height, row_bytes, full_scale)
        finally:
            if virt:
                self.munmap(virt, size)
            released = self.release(self.device, 0, info)
            if released != 0:
                raise RuntimeError("RAW frame release returned %d; restart the board" % released)
        return result

    def _rows(self, virt, row_bytes, width):
        uctypes = self.uctypes
        layout = {"v": (uctypes.ARRAY | 0, width | uctypes.UINT16)}

        def row(y):
            return uctypes.struct(virt + y * row_bytes, layout, uctypes.LITTLE_ENDIAN).v

        return row

    def _measure(self, row, width, height, stride, full_scale):
        # Align the sampled area to the Bayer 2x2 phase grid.
        margin_x = int(width * (1.0 - ROI_FRACTION) / 2.0) & ~1
        margin_y = int(height * (1.0 - ROI_FRACTION) / 2.0) & ~1
        x0, y0 = margin_x, margin_y
        x1 = (width - margin_x) & ~1
        y1 = (height - margin_y) & ~1
        step = max(2, GRID_STEP)
        if step & 1:
            step += 1
        if x1 - x0 < 4 or y1 - y0 < 4:
            raise RuntimeError("RAW ROI is too small: %dx%d" % (x1 - x0, y1 - y0))

        sums = [0.0] * 4
        squares = [0.0] * 4
        cells = 0
        clipped_cells = 0
        clip_code = full_scale - CLIP_HEADROOM
        for y in range(y0, y1 - 1, step):
            _exitpoint()
            top = row(y)
            bottom = row(y + 1)
            for x in range(x0, x1 - 1, step):
                # SC132GS is declared BGGR: B, G, G, R in a 2x2 cell.
                values = (float(top[x]), float(top[x + 1]),
                          float(bottom[x]), float(bottom[x + 1]))
                if max(values) >= clip_code:
                    clipped_cells += 1
                for index, value in enumerate(values):
                    sums[index] += value
                    squares[index] += value * value
                cells += 1
        if cells == 0:
            raise RuntimeError("RAW ROI produced no samples")
        means = [value / cells for value in sums]
        return {
            "mean": means,
            "second_moment": [value / cells for value in squares],
            "cells": cells,
            "clipped_cells": clipped_cells,
            "level": sum(means) / 4.0,
            "width": width,
            "height": height,
            "stride": stride,
            "full_scale": full_scale,
            "roi": [x0, y0, x1 - x0, y1 - y0],
        }

    def collect(self, label):
        print("[WB] %s: warming up %d frames, then measuring %d frames" %
              (label, WARMUP_FRAMES, SAMPLE_FRAMES))
        stats = Statistics()
        for index in range(WARMUP_FRAMES + SAMPLE_FRAMES):
            _exitpoint()
            sample = self.capture(FRAME_TIMEOUT_MS)
            if index < WARMUP_FRAMES:
                continue
            stats.add(sample)
            if (index - WARMUP_FRAMES + 1) % 5 == 0 or index + 1 == WARMUP_FRAMES + SAMPLE_FRAMES:
                print("[WB] %s: %d/%d frames" %
                      (label, index - WARMUP_FRAMES + 1, SAMPLE_FRAMES))
        result = stats.result()
        if result["full_scale"] != RAW10_FULL_SCALE:
            raise RuntimeError("SC132GS 标定要求 RAW10，当前帧量程=%d" % result["full_scale"])
        print("[WB] %s mean [B,G1,G2,R]=%s clip=%.3f%% frame_cv=%.2f%%" %
              (label, [round(value, 2) for value in result["mean_code"]],
               result["clip_fraction"] * 100.0, result["frame_level_cv"] * 100.0))
        return result


def _prompt(text):
    print("[WB] " + text)
    try:
        input("[WB] 按 Enter 继续: ")
    except (EOFError, KeyboardInterrupt):
        raise RuntimeError("等待用户确认时被中断")


def _load_black_reference():
    if not _exists(BLACK_RESULT_PATH):
        raise RuntimeError("找不到黑场文件 %s；先运行 CALIBRATION_STAGE='black'" % BLACK_RESULT_PATH)
    value = _read_json(BLACK_RESULT_PATH)
    if value.get("phase_order") != list(RAW_PHASE_NAMES):
        raise RuntimeError("黑场文件的 Bayer phase 顺序不匹配")
    if value.get("grid_step") != GRID_STEP or abs(float(value.get("roi_fraction")) - ROI_FRACTION) > 1e-9:
        raise RuntimeError("黑场文件的采样区域或网格设置不一致")
    if value.get("raw_bits") != 10:
        raise RuntimeError("黑场文件不是 SC132GS RAW10 标定结果")
    return value


def _configure_sensor():
    import dart_board
    from media.sensor import Sensor
    from mpp.video_struct import PIXEL_FORMAT_RGB_BAYER_10BPP

    sensor = dart_board.camera_sensor(vga=USE_VGA_MODE)
    sensor.reset()
    # Match the verified SC132GS RAW diagnostic path.  Raw WB only needs a
    # stable Bayer stream; the ISP output size is irrelevant.
    sensor.set_framesize(width=640, height=480)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_BAYER_10BPP)
    sensor.auto_exposure(False)
    try:
        bits = sensor._dev_attr.pipe_ctrl.bits
        bits.awb_enable = 0
        bits.wb_enable = 0
        bits.dnr3_enable = 0
        if getattr(bits, "ae_enable", 0) != 0:
            raise RuntimeError("关闭 AE 失败")
    except AttributeError:
        raise RuntimeError("固件缺少 ISP 控制位；请更新 Dart CanMV 固件")
    sensor.run()
    if sensor.again(ANALOG_GAIN) != 0:
        raise RuntimeError("设置模拟增益失败")
    if not sensor.exposure(EXPOSURE_US):
        raise RuntimeError("设置曝光失败")
    print("[WB] SC132GS RAW10 started: exposure=%dus analog_gain=%.3f" %
          (EXPOSURE_US, ANALOG_GAIN))
    return sensor, Sensor


def _cleanup(sensor, sensor_class):
    if sensor is not None:
        try:
            sensor.stop()
        except Exception as error:
            print("[WB] sensor.stop failed: %s" % error)
    if sensor_class is not None:
        try:
            sensor_class.deinit()
        except Exception as error:
            print("[WB] Sensor.deinit failed: %s" % error)
    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    except (AttributeError, TypeError):
        pass


def _print_gain_result(calibration, target):
    gains = calibration["manual_gain"]
    correction = calibration["correction_relative_to_current_profile"]
    print("[WB] target signal [B,G1,G2,R]=%s (%.1f%% of range)" %
          ([round(value, 2) for value in calibration["signal"]],
           calibration["signal_fraction"] * 100.0))
    print("[WB] green mismatch=%.2f%%; target clip=%.2f%%; frame_cv=%.2f%%" %
          (calibration["green_mismatch"] * 100.0,
           target["clip_fraction"] * 100.0,
           target["frame_level_cv"] * 100.0))
    print("[WB] CManualWb.gain [R,Gr,Gb,B] = [%0.6f, %0.6f, %0.6f, %0.6f]" %
          tuple(gains))
    print("[WB] correction relative to current [1.1226616,1,1,2.511838] = "
          "[%0.6f, %0.6f, %0.6f, %0.6f]" % tuple(correction))
    if calibration["green_mismatch"] > MAX_GREEN_MISMATCH:
        print("[WARN] G1/G2 mismatch is high; check Bayer phase, shading or illumination")
    if target["frame_level_cv"] > MAX_FRAME_CV:
        print("[WARN] frame brightness is unstable; use DC light or a shorter exposure")
    if max(gains) > MAX_WB_GAIN:
        print("[WARN] a WB gain is unusually high; verify black level and target exposure")
    if calibration["signal_fraction"] < MIN_SIGNAL_FRACTION:
        raise RuntimeError("目标信号低于 %.1f%%，请提高曝光或模拟增益" %
                           (MIN_SIGNAL_FRACTION * 100.0))
    if target["clip_fraction"] > MAX_CLIP_FRACTION:
        raise RuntimeError("目标存在 %.2f%% 饱和采样，请降低曝光或模拟增益" %
                           (target["clip_fraction"] * 100.0))


def main():
    validate_config()
    sensor = None
    sensor_class = None
    try:
        sensor, sensor_class = _configure_sensor()
        collector = Raw10Collector(sensor)

        black = None
        if CALIBRATION_STAGE in ("black", "both"):
            _prompt("请完全遮住镜头；保持曝光、增益和光路不变")
            black = collector.collect("black")
            black_record = {
                "version": 1,
                "sensor": "SC132GS",
                "stage": "black",
                "captured_ms": time.ticks_ms(),
                "exposure_us": EXPOSURE_US,
                "analog_gain": ANALOG_GAIN,
                "grid_step": GRID_STEP,
                "roi_fraction": ROI_FRACTION,
                "raw_bits": 10,
                "phase_order": list(RAW_PHASE_NAMES),
                "statistics": black,
            }
            _write_json(BLACK_RESULT_PATH, black_record)
            print("[WB] black reference saved: %s" % BLACK_RESULT_PATH)

        if CALIBRATION_STAGE == "black":
            print("[PASS] black reference complete; rerun with CALIBRATION_STAGE='white'")
            return

        if black is None:
            black_record = _load_black_reference()
            if black_record.get("exposure_us") != EXPOSURE_US or \
                    abs(float(black_record.get("analog_gain")) - ANALOG_GAIN) > 1e-6:
                raise RuntimeError("黑场文件的曝光/增益与当前设置不一致")
            black = black_record["statistics"]

        _prompt("请移除镜头盖，放置均匀的白卡或 18% 灰卡，填充中央视野并避免反光")
        target = collector.collect("target")
        if target["clip_fraction"] > MAX_CLIP_FRACTION:
            print("[WARN] target is clipped; the final result will be rejected")
        if black["full_scale"] != target["full_scale"]:
            raise RuntimeError("黑场与目标帧的 RAW 位深不一致")
        calibration = calculate_wb_gains(target["mean_code"], black["mean_code"],
                                          target["full_scale"])
        _print_gain_result(calibration, target)
        result = {
            "version": 1,
            "sensor": "SC132GS",
            "stage": "white",
            "captured_ms": time.ticks_ms(),
            "exposure_us": EXPOSURE_US,
            "analog_gain": ANALOG_GAIN,
            "grid_step": GRID_STEP,
            "roi_fraction": ROI_FRACTION,
            "raw_bits": 10,
            "raw_phase_order": list(RAW_PHASE_NAMES),
            "manual_gain_order": list(MANUAL_GAIN_NAMES),
            "manual_gain": calibration["manual_gain"],
            "correction_relative_to_current_profile": calibration["correction_relative_to_current_profile"],
            "black": black,
            "target": target,
            "calibration": calibration,
        }
        _write_json(RESULT_PATH, result)
        print("[PASS] white-balance calibration saved: %s" % RESULT_PATH)
    finally:
        _cleanup(sensor, sensor_class)


if __name__ == "__main__":
    main()
