"""Host checks for scan isolation, statistics and partial results; no hardware claims."""

import contextlib
import csv
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SCRIPT = Path(__file__).resolve().parents[1] / "tools/sc132gs_color_scan.py"


class Frame:
    def __init__(self, lab=(50, -40, 30), pixel=None):
        self.lab = lab
        self.pixel = pixel or (lambda x, y: (16, 80, 24))
        self.annotated = False

    def width(self):
        return 8

    def height(self):
        return 8

    def format(self):
        return 5

    def get_histogram(self, **kwargs):
        if self.annotated:
            raise AssertionError("Measured preview overlays")
        lab = self.lab
        stats = SimpleNamespace(l_mean=lambda: lab[0], a_mean=lambda: lab[1], b_mean=lambda: lab[2],
                                l_stdev=lambda: 1, a_stdev=lambda: 2, b_stdev=lambda: 3)

        def percentile(p):
            offset = -2 if p < 0.5 else 2
            return SimpleNamespace(l_value=lambda: lab[0] + offset,
                                   a_value=lambda: lab[1] + offset, b_value=lambda: lab[2] + offset)

        return SimpleNamespace(get_statistics=lambda: stats, get_percentile=percentile)

    def get_pixel(self, x, y, **kwargs):
        if self.annotated:
            raise AssertionError("Measured preview overlays")
        return self.pixel(x, y)

    def draw_rectangle(self, *args, **kwargs):
        self.annotated = True


class Sensor:
    def __init__(self, fixture):
        self.fixture = fixture
        self.exp, self.gain, self.old_frames = 1000.0, 1.0, 0
        self.requested = []
        self.fail_gain = None
        bits = SimpleNamespace(ae_enable=1, awb_enable=1, wb_enable=1, dnr3_enable=1,
                               ccm_enable=1, gc_enable=1, cproc_enable=1)
        self._dev_attr = SimpleNamespace(pipe_ctrl=SimpleNamespace(bits=bits))
        self.reset, self.set_framesize, self.set_pixformat, self.stop = Mock(), Mock(), Mock(), Mock()

    def auto_exposure(self, enable):
        self._dev_attr.pipe_ctrl.bits.ae_enable = int(enable)

    def run(self):
        bits = self._dev_attr.pipe_ctrl.bits
        self.fixture.assertEqual((bits.ae_enable, bits.awb_enable, bits.wb_enable, bits.dnr3_enable),
                                 (0, 0, 0, 0))

    def again(self, value=None):
        if value is None:
            # Match CanMV's k_sensor_gain, not the scalar accepted by its setter.
            return SimpleNamespace(gain=[self.gain, 0.0, 0.0])
        if value == self.fail_gain:
            raise RuntimeError("gain write failed")
        self.gain = math.floor(value * 2) / 2
        self.old_frames = 2
        self.requested.append(("gain", value))
        return 0

    def exposure(self, value=None):
        if value is None:
            return self.exp
        self.exp = float(value // 100 * 100)
        self.old_frames = 2
        self.requested.append(("exposure", value))
        return True

    def snapshot(self, **kwargs):
        self.fixture.assertEqual(kwargs, {"timeout": 1000, "auto_reboot": False})
        self.fixture.clock += 100
        if self.old_frames:
            self.old_frames -= 1
            return Frame((1, 0, 0))
        return Frame((int(self.exp / 100), -int(self.gain * 10), 30))


class ColorScanTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location("color_scan_under_test", SCRIPT)
        self.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.script)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.script.RESULT_PREFIX = str(Path(self.directory.name) / "scan")
        self.script.output_size = lambda: (8, 8)
        self.script.COLOR_ROI = (0, 0, 8, 8)
        self.script.GAIN_STEPS = (1.1, 2.7)
        self.script.EXPOSURE_STEPS_US = (500, 1400)
        self.script.FIXED_EXPOSURE_US = 1000
        self.script.FIXED_ANALOG_GAIN = 1.25
        self.script.SETTLE_FRAMES = 3
        self.script.SETTLE_MS = 0
        self.script.SAMPLE_FRAMES = 3
        self.script.RGB_MAX_SAMPLES_PER_FRAME = 16
        self.clock = 0
        self.script.time = SimpleNamespace(ticks_ms=lambda: self.clock, ticks_diff=lambda a, b: a - b,
                                           sleep_ms=Mock())
        self.script.os = SimpleNamespace(stat=os.stat, exitpoint=Mock(), EXITPOINT_ENABLE_SLEEP=1)
        self.sensor = Sensor(self)
        self.board = SimpleNamespace(camera_sensor=Mock(return_value=self.sensor))
        self.preview = SimpleNamespace(show=Mock(side_effect=lambda frame, *a, **k: frame.draw_rectangle()),
                                       deinit=Mock())
        self.script.Preview = lambda: self.preview
        sensor_module = ModuleType("media.sensor")
        sensor_module.Sensor = SimpleNamespace(RGB565=2)
        image_module = ModuleType("image")
        image_module.RGB565 = 5
        self.modules = {"dart_board": self.board, "media": ModuleType("media"),
                        "media.sensor": sensor_module, "image": image_module}
        self.output = ""

    def run_main(self):
        output = io.StringIO()
        try:
            with patch.dict(sys.modules, self.modules), contextlib.redirect_stdout(output):
                self.script.main()
        finally:
            self.output = output.getvalue()

    def events(self, number=0):
        path = Path(self.directory.name) / ("scan_%04d.jsonl" % number)
        return [json.loads(line) for line in path.read_text().splitlines()]

    def rows(self, number=0):
        with (Path(self.directory.name) / ("scan_%04d.csv" % number)).open() as stream:
            return list(csv.DictReader(stream))

    def test_two_independent_sweeps_record_quantized_readback_and_clean_frames(self):
        self.run_main()
        rows, events = self.rows(), self.events()
        self.assertEqual(len(rows), 12)
        points = [event for event in events if event["event"] == "point_end"]
        self.assertEqual([(p["sweep"], p["exposure_us_requested"], p["analog_gain_requested"]) for p in points],
                         [("gain", 1000, 1.1), ("gain", 1000, 2.7),
                          ("exposure", 500, 1.25), ("exposure", 1400, 1.25)])
        self.assertEqual([p["analog_gain_readback"] for p in points], [1.0, 2.5, 1.0, 1.0])
        self.assertEqual(self.sensor.requested, [
            ("gain", 1.25), ("exposure", 1000),
            ("gain", 1.1), ("exposure", 1000), ("gain", 2.7), ("exposure", 1000),
            ("gain", 1.25), ("exposure", 500), ("gain", 1.25), ("exposure", 1400)])
        for row in rows:
            self.assertEqual(float(row["l_mean"]), float(row["exposure_us_readback"]) / 100)
            self.assertEqual(float(row["a_mean"]), -10 * float(row["analog_gain_readback"]))
            self.assertNotIn(None, row)  # No extra CSV values missing a header.
        self.assertEqual(events[-1], {"event": "run_end", "status": "complete",
                                      "completed_points": 4, "error": None})
        self.sensor.stop.assert_called_once_with()
        self.preview.deinit.assert_called_once_with()
        self.assertIn("[PASS] Color scan: 4 points", self.output)

    def test_disabled_sweep_never_runs_and_headless_capture_uses_configured_roi(self):
        self.script.GAIN_SWEEP_ENABLED = False
        self.script.PREVIEW_ENABLED = False
        self.run_main()
        self.assertEqual({row["sweep"] for row in self.rows()}, {"exposure"})
        self.preview.show.assert_not_called()
        self.preview.deinit.assert_not_called()

    def test_exposure_range_drives_sensor_requests_and_result_metadata(self):
        self.script.GAIN_SWEEP_ENABLED = False
        self.script.EXPOSURE_STEPS_US = None
        self.run_main()
        expected = list(range(100, 301, 10))
        events = self.events()
        points = [event for event in events if event["event"] == "point_end"]
        self.assertEqual(events[0]["exposure_steps_us"], expected)
        self.assertEqual([point["exposure_us_requested"] for point in points], expected)
        self.assertEqual([value for name, value in self.sensor.requested if name == "exposure"],
                         [self.script.FIXED_EXPOSURE_US] + expected)
        self.assertTrue(all(point["analog_gain_requested"] == self.script.FIXED_ANALOG_GAIN
                            for point in points))
        self.assertEqual(len(self.rows()), 21 * self.script.SAMPLE_FRAMES)
        self.assertEqual(events[-1]["completed_points"], 21)

    def test_exposure_range_respects_upper_bound_single_point_and_step_limit(self):
        self.script.EXPOSURE_STEPS_US = None
        for lower, upper, step, expected in (
                (100, 300, 60, (100, 160, 220, 280)),
                (200, 200, 10, (200,)),
                (100, 300, 500, (100,)),
                (1, 256, 1, tuple(range(1, 257)))):
            with self.subTest(lower=lower, upper=upper, step=step):
                self.script.EXPOSURE_MIN_US = lower
                self.script.EXPOSURE_MAX_US = upper
                self.script.EXPOSURE_STEP_US = step
                self.script.validate_config()
                self.assertEqual(self.script.exposure_steps(), expected)

    def test_invalid_exposure_range_is_rejected_before_camera_construction(self):
        self.script.EXPOSURE_STEPS_US = None
        defaults = {"EXPOSURE_MIN_US": 100, "EXPOSURE_MAX_US": 300, "EXPOSURE_STEP_US": 10}
        invalid = [(name, value) for name in defaults
                   for value in (0, -1, True, 1.5, None, float("nan"), float("inf"))]
        invalid += [("EXPOSURE_MIN_US", 301), ("EXPOSURE_MAX_US", 99),
                    ("EXPOSURE_MAX_US", 1000000000)]
        for name, value in invalid:
            with self.subTest(name=name, value=value):
                for key, default in defaults.items():
                    setattr(self.script, key, default)
                setattr(self.script, name, value)
                with self.assertRaisesRegex(ValueError, "EXPOSURE_|Exposure sweep"):
                    self.run_main()
        self.board.camera_sensor.assert_not_called()

    def test_custom_exposure_steps_override_range_and_disabled_range_is_ignored(self):
        self.script.EXPOSURE_STEP_US = 0
        self.script.GAIN_SWEEP_ENABLED = False
        self.run_main()
        self.assertEqual(self.events()[0]["exposure_steps_us"], [500, 1400])
        self.script.GAIN_SWEEP_ENABLED = True
        self.script.EXPOSURE_SWEEP_ENABLED = False
        self.script.EXPOSURE_STEPS_US = None
        self.run_main()
        self.assertEqual({row["sweep"] for row in self.rows(1)}, {"gain"})
        self.assertEqual(self.events(1)[0]["exposure_steps_us"], [])

    def test_struct_readback_uses_first_linear_gain_without_changing_exposure_units(self):
        sensor = SimpleNamespace(exposure=lambda: 999.5,
                                 again=lambda: SimpleNamespace(gain=[4.75, 0.0, 0.0]))
        self.assertEqual(self.script.read_settings(sensor), (999.5, 4.75))

    def test_numeric_gain_readback_is_still_accepted(self):
        sensor = SimpleNamespace(exposure=lambda: 1000.0, again=lambda: 5.0)
        self.assertEqual(self.script.read_settings(sensor), (1000.0, 5.0))

    def test_invalid_gain_readback_reports_returned_value_instead_of_using_request(self):
        values = (None, True, False, 0, -1, float("nan"), float("inf"),
                  SimpleNamespace(gain=[]), SimpleNamespace(gain=[None]),
                  SimpleNamespace(gain=[True]), SimpleNamespace(gain=[float("nan")]))
        for value in values:
            with self.subTest(value=value):
                sensor = SimpleNamespace(exposure=lambda: 1000.0, again=lambda: value)
                with self.assertRaisesRegex(RuntimeError, r"Invalid sensor readback: exposure\(\)=1000.0, again\(\)="):
                    self.script.read_settings(sensor)

    def test_invalid_exposure_readback_remains_an_error(self):
        for value in (None, True, False, 0, -1, float("nan"), float("inf")):
            with self.subTest(value=value):
                sensor = SimpleNamespace(exposure=lambda: value,
                                         again=lambda: SimpleNamespace(gain=[5.0, 0.0, 0.0]))
                with self.assertRaisesRegex(RuntimeError, "Invalid sensor readback"):
                    self.script.read_settings(sensor)

    def test_each_setting_drains_both_minimum_frames_and_time(self):
        self.script.SETTLE_MS = 550
        self.run_main()
        starts = [event for event in self.events() if event["event"] == "point_start"]
        self.assertTrue(all(event["discarded_frames"] == 6 for event in starts))

    def test_failure_preserves_completed_points_and_attempts_all_cleanup(self):
        self.sensor.fail_gain = 2.7
        self.sensor.stop.side_effect = RuntimeError("stop failed")
        with self.assertRaisesRegex(RuntimeError, "gain write failed"):
            self.run_main()
        self.assertEqual(len(self.rows()), 3)
        end = self.events()[-1]
        self.assertEqual((end["status"], end["completed_points"]), ("failed", 1))
        self.assertEqual(end["error"], "gain write failed")
        self.preview.deinit.assert_called_once()
        self.assertNotIn("[PASS]", self.output)

    def test_interrupt_keeps_partial_results_without_claiming_completion(self):
        original = self.script.measure
        count = 0

        def measure(*args):
            nonlocal count
            count += 1
            if count == 5:
                raise KeyboardInterrupt()
            return original(*args)

        self.script.measure = measure
        self.run_main()
        self.assertEqual(len(self.rows()), 4)
        self.assertEqual(self.events()[-1]["status"], "interrupted")
        self.assertEqual(self.events()[-1]["completed_points"], 1)
        self.assertNotIn("[PASS]", self.output)
        self.sensor.stop.assert_called_once()

    def test_fixed_parameter_drift_between_points_aborts_scan(self):
        self.script.read_settings = lambda sensor: (sensor.exp + (1 if sensor.gain == 2.5 else 0), sensor.gain)
        with self.assertRaisesRegex(RuntimeError, "fixed parameter changed"):
            self.run_main()
        self.assertEqual(len(self.rows()), 3)
        self.assertEqual(self.events()[-1]["completed_points"], 1)

    def test_settings_change_during_sampling_marks_point_invalid(self):
        readbacks = iter(((1000.0, 1.0), (1000.0, 2.0)))
        self.script.read_settings = lambda sensor: next(readbacks)
        with self.assertRaisesRegex(RuntimeError, "point is invalid"):
            self.run_main()
        end = [event for event in self.events() if event["event"] == "point_end"][0]
        self.assertIs(end["settings_stable"], False)
        self.assertEqual(self.events()[-1]["completed_points"], 0)

    def test_invalid_configuration_is_rejected_before_camera_construction(self):
        for value in (float("nan"), float("inf"), True, 0, 29):
            self.script.GAIN_STEPS = (value,)
            with self.assertRaises(ValueError):
                self.run_main()
        self.board.camera_sensor.assert_not_called()

    def test_result_files_are_not_overwritten(self):
        self.run_main()
        path = Path(self.directory.name) / "scan_0000.csv"
        original = path.read_bytes()
        self.run_main()
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(len(self.rows(1)), 12)

    def test_two_preview_clicks_select_one_roi_and_release_touch(self):
        self.script.COLOR_ROI = None
        touch = SimpleNamespace(read=Mock(side_effect=[
            [SimpleNamespace(event=1, x=0, y=0), SimpleNamespace(event=2, x=0, y=0)],
            [SimpleNamespace(event=1, x=7, y=7)]]), deinit=Mock())
        touch_type = Mock(return_value=touch)
        touch_type.DEV_IDE, touch_type.EVENT_DOWN = 7, 1
        modules = dict(self.modules, machine=SimpleNamespace(TOUCH=touch_type))
        with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()):
            roi = self.script.select_roi(self.script.Capture(self.sensor), self.preview)
        self.assertEqual(roi, (0, 0, 8, 8))
        touch_type.assert_called_once_with(7, range_x=8, range_y=8)
        touch.deinit.assert_called_once_with()

    def test_roi_selection_timeout_releases_touch(self):
        self.script.COLOR_ROI = None
        self.script.ROI_SELECTION_TIMEOUT_MS = 150
        touch = SimpleNamespace(read=Mock(return_value=[]), deinit=Mock())
        touch_type = Mock(return_value=touch)
        touch_type.DEV_IDE, touch_type.EVENT_DOWN = 7, 1
        modules = dict(self.modules, machine=SimpleNamespace(TOUCH=touch_type))
        with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "selection timed out"):
                self.script.select_roi(self.script.Capture(self.sensor), self.preview)
        touch.deinit.assert_called_once_with()

    def test_rgb_statistics_clipping_and_dark_pixels_use_bounded_grid(self):
        self.script.RGB_MAX_SAMPLES_PER_FRAME = 4
        pixels = {(0, 0): (255, 0, 0), (4, 0): (0, 255, 0),
                  (0, 4): (0, 0, 255), (4, 4): (0, 0, 0)}
        frame = Frame((50, 0, 0), lambda x, y: pixels[(x, y)])
        step = self.script.sample_step((0, 0, 8, 8))
        value = self.script.measure(frame, (0, 0, 8, 8), step)
        self.assertEqual(value["rgb_sample_pixels"], 4)
        self.assertEqual(value["rgb_mean"], [63.75] * 3)
        self.assertEqual(value["channel_clip_fraction"], [0.25] * 3)
        self.assertEqual(value["any_clip_fraction"], 0.75)
        self.assertEqual(value["dark_fraction"], 0.25)
        self.assertIsNone(value["lab_hue_deg"])
        self.assertEqual(value["rgb_fraction"], [1 / 3] * 3)

    def test_pooled_variance_and_circular_hue_handle_brightness_drift_and_wrap(self):
        stats = self.script.PointStatistics()
        value = self.script.measure(Frame(), (0, 0, 8, 8), 2)
        value.update(rgb_mean=[0, 0, 0], rgb_stdev=[0, 0, 0], lab_hue_deg=359)
        stats.add(value)
        value.update(rgb_mean=[20, 20, 20], lab_hue_deg=1)
        stats.add(value)
        result = stats.result()
        self.assertEqual(result["rgb_mean"], [10.0] * 3)
        self.assertEqual(result["rgb_stdev"], [10.0] * 3)
        self.assertLess(abs((result["lab_hue_mean_deg"] + 180) % 360 - 180), 0.001)
        self.assertAlmostEqual(result["lab_hue_concentration"], math.cos(math.pi / 180))


if __name__ == "__main__":
    unittest.main()
