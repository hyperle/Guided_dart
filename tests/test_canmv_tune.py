"""Host checks for calibration output, stop handling, and failed startup cleanup."""

import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "tools/canmv_tune.py"


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location("canmv_calibration_test", SCRIPT)
        self.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.script)
        # 测试使用固定配置，不依赖正在编辑的相机开关和舵机标定参数。
        self.script.CAMERA_ENABLED = True
        self.script.PWM_ENABLED = True
        self.script.SERVO_INITIAL_US = (1523.0, 1520.0, 1520.0, 1520.0)
        self.script.SERVO_MIN_US = (1420.0,) * 4
        self.script.SERVO_MAX_US = (1620.0,) * 4
        self.script.SERVO_MAX_DELTA_US = 500.0
        self.script.SERVO_ENABLED = (True,) * 4
        self.script.DETECTION_ENABLED = False
        self.events = []
        self.pwms = []
        self.results = iter([0, 100, KeyboardInterrupt()])
        self.frame_pixels = bytes(640 * 480)
        self.fail_channel = None
        self.pad_values = [0x200] * 1024  # 电压位必须在配置、接通、关闭过程中保留。
        self.routes = {}
        self.clock = 0
        self.script.os = SimpleNamespace(exitpoint=lambda: None)
        self.script.time = SimpleNamespace(
            ticks_ms=self.tick, ticks_us=self.tick_us, ticks_diff=lambda a, b: a - b,
            sleep_ms=lambda ms: self.events.append(("wait", ms))
        )
        self.script.MAX_CONSECUTIVE_TIMEOUTS = 2
        owner = self

        class PWM:
            def __init__(self, channel):
                if channel == owner.fail_channel:
                    raise RuntimeError("PWM startup failed")
                self.pin = dict(zip((1, 2, 3, 5), (61, 7, 8, 59)))[channel]
                owner.assertEqual(owner.routes[self.pin], "pwm%d" % channel,
                                  "PWM(channel) requires the route during construction")
                self.channel = channel
                self.hz = 0
                self.ns = 0
                self.stopped = False
                owner.pwms.append(self)

            def freq(self, value=None):
                if value is not None:
                    for i, pin in enumerate(owner.script.PWM_PINS):
                        if owner.script.SERVO_ENABLED[i]:
                            owner.assertEqual(owner.routes[pin], "gpio%d" % pin)
                            owner.assertFalse(owner.pad_values[pin] & 0x80)
                    self.hz = value
                return self.hz

            def duty_ns(self, value=None):
                if value is not None:
                    self.ns = value
                    owner.events.append(("pulse", self.channel, value))
                return self.ns

            def deinit(self):
                self.stopped = True
                owner.events.append(("pwm_close", self.channel))

        class FPIOA:
            def get_pin_func(self, pin):
                return owner.routes.get(pin, "gpio%d" % pin)

            def set_function(self, pin, function, **kwargs):
                # SDK first applies the function defaults, then the keyword flags.
                if function.startswith("pwm"):
                    mux = {61: 1, 7: 1, 8: 1, 59: 3}[pin]
                    value = (mux << 11) | 0x8E
                    if any(p.hz for p in owner.pwms):
                        owner.assertTrue(all(p.ns for p in owner.pwms),
                                         "all pulses must be ready before routing an active PWM")
                else:
                    owner.assertEqual(function, "gpio%d" % pin)
                    value = 0x100
                value |= owner.pad_values[pin] & 0x200
                owner.routes[pin] = function
                owner.events.append(("pad_default", pin, value))
                for name, bit in (("pd", 5), ("pu", 6), ("oe", 7), ("ie", 8)):
                    if name in kwargs:
                        value = (value & ~(1 << bit)) | (kwargs[name] << bit)
                owner.pad_values[pin] = value
                owner.events.append(("pad_write", pin, value))

        for channel in (1, 2, 3, 5):
            setattr(FPIOA, "PWM%d" % channel, "pwm%d" % channel)
        for pin in (61, 7, 8, 59):
            setattr(FPIOA, "GPIO%d" % pin, "gpio%d" % pin)
        class Camera:
            def __init__(self, *args):
                owner.events.append(("camera_open", args))
                self.sequence = 0

            def snapshot_into(self, buffer, timeout):
                owner.assertEqual(len(buffer), 640 * 480)
                owner.assertEqual(timeout, owner.script.ACQUIRE_TIMEOUT_MS)
                result = next(owner.results)
                if isinstance(result, BaseException):
                    raise result
                if result is not None:
                    self.sequence += 1
                    buffer[:] = owner.frame_pixels
                return result

            def frame_info(self):
                return (self.sequence, self.sequence * 10000, 0, 1)

            def close(self):
                owner.events.append(("camera_close",))

        class Display:
            VIRT = "virtual"

            @staticmethod
            def init(*_, **__):
                owner.events.append(("display_open",))

            @staticmethod
            def show_image(_):
                owner.events.append(("display_frame",))

            @staticmethod
            def deinit():
                owner.events.append(("display_close",))

        def image(width, height, format, **__):
            pixels = bytearray(width * height * (2 if format == 2 else 1))
            def draw(kind, *args, **kwargs):
                owner.events.append((kind, args, kwargs))
                pixels[0] = 128  # Make an accidental detect-after-overlay visible.
            def draw_image(other, *args):
                owner.assertIsNot(pixels, other.bytearray())
                pixels[0] = other.bytearray()[0]
                owner.events.append(("preview_copy",))
            return SimpleNamespace(
                bytearray=lambda: pixels,
                draw_image=draw_image,
                draw_rectangle=lambda *a, **kw: draw("rectangle", *a, **kw),
                draw_cross=lambda *a, **kw: draw("cross", *a, **kw),
                draw_string=lambda *a, **kw: draw("text", *a, **kw),
            )

        class Detector:
            def __init__(self, config):
                owner.events.append(("detector_open",))
                owner.assertEqual(config["threshold"], owner.script.GRAY_THRESHOLD)
                owner.assertEqual(config["workspace.min_contrast"], 12.0)

            def detect(self, pixels, metadata):
                owner.assertEqual(bytes(pixels), owner.frame_pixels,
                                  "grayscale pixels must reach detection unchanged")
                owner.events.append(("detect", metadata[0]))
                return {"kind": "none", "target": None, "light": None, "armor": None,
                        "roi": (0, 0, 640, 480), "full_frame": True, "rescanned": False,
                        "candidates": 0, "missed_frames": 1, "run_high_water": 0,
                        "rejected_contrast": 0, "rejected_snr": 0, "rejected_stability": 0,
                        "phase": "detect", "full_validation": True, "stability_checks": 0}

            def close(self):
                owner.events.append(("detector_close",))

        image_module = SimpleNamespace(Image=image, GRAYSCALE=1, RGB565=2, ALLOC_HEAP=0)
        self.modules = {
            "machine": SimpleNamespace(PWM=PWM, FPIOA=FPIOA),
            "image": image_module,
            "_dart_raw": SimpleNamespace(Camera=Camera, PIXEL_FORMAT="gray8"),
            "_dart_detect": SimpleNamespace(Detector=Detector, GRAY_FEATURE_VERSION=1, TRACK_FEATURE_VERSION=1),
            "media": SimpleNamespace(),
            "media.display": SimpleNamespace(Display=Display),
        }

    def tick(self):
        self.clock += 100
        return self.clock

    def tick_us(self):
        self.clock += 100
        return self.clock

    def run_script(self):
        with patch.dict(sys.modules, self.modules), contextlib.redirect_stdout(io.StringIO()):
            self.script.main()

    def assert_cleaned(self, count):
        self.assertEqual(len(self.pwms), count)
        self.assertTrue(all(pwm.stopped for pwm in self.pwms))
        self.assertIn(("camera_close",), self.events)
        self.assertIn(("display_close",), self.events)
        last_pwm_close = max(i for i, item in enumerate(self.events) if item[0] == "pwm_close")
        self.assertLess(last_pwm_close, self.events.index(("camera_close",)))

    def test_old_binary_interpreter_requires_update(self):
        del self.modules["_dart_raw"].PIXEL_FORMAT
        with self.assertRaisesRegex(RuntimeError, "updated tuning interpreter"):
            self.run_script()
        self.assertFalse(self.pwms)
        self.assertFalse(any(item[0] == "camera_open" for item in self.events))

    def test_export_uses_one_increment_limit(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.script._print_parameters()
        tokens = output.getvalue().split()
        self.assertIn("--servo-max-delta", tokens)
        self.assertEqual(tokens[tokens.index("--servo-max-delta") + 1], "500.000")
        self.assertNotIn("--servo-min", tokens)
        self.assertNotIn("--servo-max", tokens)
        for invalid in (-1, float("nan"), float("inf"), 1600):
            with self.subTest(limit=invalid):
                self.script.SERVO_MAX_DELTA_US = invalid
                with self.assertRaises(ValueError):
                    self.script._validate()

    def test_black_frame_is_valid_and_interrupt_closes_all_outputs(self):
        self.run_script()
        self.assertEqual(self.events.count(("display_frame",)), 2)
        self.assert_cleaned(4)

    def test_fractional_microseconds_and_routes_only_after_all_pulses_written(self):
        self.script.SERVO_INITIAL_US = (1512.125, 1530.5, 1499.9996, 1520.0)
        self.run_script()
        self.assertEqual([pwm.ns for pwm in self.pwms], [1512125, 1530500, 1500000, 1520000])
        startup_writes = [i for i, item in enumerate(self.events) if item[0] == "pulse"][:4]
        last_write = max(startup_writes)
        for pin in (61, 7, 8, 59):
            connections = [i for i, item in enumerate(self.events)
                           if item[0] == "pad_write" and item[1] == pin and item[2] & 0x80]
            self.assertEqual(len(connections), 1)
            self.assertGreater(connections[0], last_write)
        self.assertTrue(all(item[2] & 0x200 for item in self.events if item[0] == "pad_write"))

    def test_camera_timeouts_close_pwm_and_camera(self):
        self.results = iter([None, None])
        with self.assertRaisesRegex(RuntimeError, "consecutive frame timeouts"):
            self.run_script()
        self.assert_cleaned(4)

    def test_pwm_runs_without_register_mapping_or_extra_native_module(self):
        self.modules["_dart_pwm"] = None
        self.modules["mpp"] = None
        self.modules["mpp.sys"] = None
        self.modules["uctypes"] = None
        self.run_script()
        self.assert_cleaned(4)

    def test_route_read_failure_preserves_pin_configuration_and_closes_camera(self):
        def fail(*_):
            raise RuntimeError("get pin func failed")
        self.modules["machine"].FPIOA.get_pin_func = fail
        with self.assertRaisesRegex(RuntimeError, "get pin func failed"):
            self.run_script()
        self.assertFalse(self.pwms)
        self.assertEqual(self.routes[61], "pwm1")
        self.assertFalse(self.pad_values[61] & 0x80)
        self.assertIn(("camera_close",), self.events)
        self.assertIn(("display_close",), self.events)

    def test_connect_failure_closes_pwm_without_rerouting_pins(self):
        original = self.modules["machine"].FPIOA.set_function
        def fail(fpioa, pin, function, **kwargs):
            original(fpioa, pin, function, **kwargs)
            if pin == 7 and kwargs.get("oe") == 1:
                raise RuntimeError("connect failed")
        self.modules["machine"].FPIOA.set_function = fail
        with self.assertRaisesRegex(RuntimeError, "connect failed"):
            self.run_script()
        self.assertEqual(self.routes, {61: "pwm1", 7: "pwm2", 8: "gpio8", 59: "gpio59"})
        self.assert_cleaned(4)

    def test_route_mismatch_is_reported_before_pwm_construction(self):
        self.modules["machine"].FPIOA.get_pin_func = lambda *_: "wrong_route"
        with self.assertRaisesRegex(RuntimeError, "pin route mismatch"):
            self.run_script()
        self.assertFalse(self.pwms)
        self.assertFalse(self.pad_values[61] & 0x80)

    def test_bad_pwm_readback_leaves_outputs_disconnected(self):
        original = self.modules["machine"].PWM.duty_ns
        def fail_readback(pwm, value=None):
            result = original(pwm, value)
            return result if value is not None else 0
        self.modules["machine"].PWM.duty_ns = fail_readback
        with self.assertRaisesRegex(RuntimeError, "configuration readback mismatch"):
            self.run_script()
        self.assertFalse(any(item[0] == "pad_write" and item[2] & 0x80 for item in self.events))
        self.assert_cleaned(4)

    def test_partial_pwm_startup_closes_preceding_channels_and_camera(self):
        self.fail_channel = 3
        with self.assertRaisesRegex(RuntimeError, "PWM startup failed"):
            self.run_script()
        self.assert_cleaned(2)
        self.assertFalse(any(item[0] in ("pulse", "wait") for item in self.events))

    def interrupt_with_displaced_servos(self):
        for pwm in self.pwms:
            pwm.duty_ns(1600000)
        raise KeyboardInterrupt()

    def test_stop_returns_all_servos_before_wait_and_close_without_rerouting(self):
        self.script.os.exitpoint = self.interrupt_with_displaced_servos
        self.run_script()
        self.assertEqual([pwm.ns for pwm in self.pwms], [1523000, 1520000, 1520000, 1520000])
        wait = self.events.index(("wait", self.script.SERVO_RETURN_MS))
        return_writes = []
        for pwm in self.pwms:
            writes = [i for i, item in enumerate(self.events)
                      if item[:2] == ("pulse", pwm.channel)]
            self.assertEqual(len(writes), 3)  # 启动零点、模拟偏移、停止回位。
            self.assertLess(writes[-1], wait)
            return_writes.append(writes[-1])
        first_close = next(i for i, item in enumerate(self.events) if item[0] == "pwm_close")
        self.assertLess(wait, first_close)
        self.assertFalse(any(item[0] in ("pad_default", "pad_write")
                             for item in self.events[min(return_writes):]))
        for pin, channel in zip(self.script.PWM_PINS, self.script.PWM_CHANNELS):
            self.assertEqual(self.routes[pin], "pwm%d" % channel)
            self.assertTrue(self.pad_values[pin] & 0x80)
            self.assertFalse(self.pad_values[pin] & 0x20)
        self.assert_cleaned(4)

    def test_stop_uses_correct_zeros_when_only_nonadjacent_channels_enabled(self):
        self.script.SERVO_ENABLED = (False, True, False, True)
        self.script.SERVO_INITIAL_US = (1500.0, 1512.125, 1540.0, 1530.5)
        self.script.os.exitpoint = self.interrupt_with_displaced_servos
        self.run_script()
        self.assertEqual([(pwm.channel, pwm.ns) for pwm in self.pwms],
                         [(2, 1512125), (5, 1530500)])
        self.assertEqual({item[1] for item in self.events if item[0] == "pad_write"}, {7, 59})
        self.assert_cleaned(2)

    def test_failed_return_still_homes_other_channels_and_closes_everything(self):
        def interrupt_and_fail_one_return():
            def fail(_):
                raise RuntimeError("PWM zero write failed")
            self.pwms[0].duty_ns = fail
            raise KeyboardInterrupt()
        self.script.os.exitpoint = interrupt_and_fail_one_return
        self.run_script()
        self.assertEqual([pwm.ns for pwm in self.pwms[1:]], [1520000] * 3)
        self.assertIn(("wait", self.script.SERVO_RETURN_MS), self.events)
        self.assert_cleaned(4)

    def test_interrupted_return_wait_still_closes_pwm_and_camera(self):
        def interrupt_wait(_):
            raise KeyboardInterrupt()
        self.script.time.sleep_ms = interrupt_wait
        self.run_script()
        self.assert_cleaned(4)

    def test_failed_readback_still_allows_time_for_successful_zero_writes(self):
        def interrupt_with_bad_reads():
            for pwm in self.pwms:
                original = pwm.duty_ns
                def read_fails(value=None, original=original):
                    if value is None:
                        raise RuntimeError("PWM readback failed")
                    return original(value)
                pwm.duty_ns = read_fails
            raise KeyboardInterrupt()
        self.script.os.exitpoint = interrupt_with_bad_reads
        self.run_script()
        self.assertIn(("wait", self.script.SERVO_RETURN_MS), self.events)
        self.assert_cleaned(4)

    def test_failed_deinit_still_closes_other_channels_and_camera(self):
        def interrupt_with_failed_close():
            def fail():
                raise RuntimeError("PWM deinit failed")
            self.pwms[0].deinit = fail
            raise KeyboardInterrupt()
        self.script.os.exitpoint = interrupt_with_failed_close
        self.run_script()
        self.assertTrue(all(pwm.stopped for pwm in self.pwms[1:]))
        self.assertIn(("camera_close",), self.events)
        self.assertIn(("display_close",), self.events)

    def test_return_delay_requires_several_pwm_periods(self):
        self.script.SERVO_RETURN_MS = 0
        with self.assertRaisesRegex(ValueError, "SERVO_RETURN_MS"):
            self.run_script()
        self.assertFalse(self.pwms)

    def test_disabled_pwm_does_not_wait_for_return(self):
        self.script.PWM_ENABLED = False
        self.modules["_dart_pwm"] = None
        self.run_script()
        self.assertFalse(self.pwms)
        self.assertFalse(any(item[0] == "wait" for item in self.events))

    def test_detection_uses_raw_pixels_and_never_advances_on_timeout(self):
        self.script.DETECTION_ENABLED = True
        self.frame_pixels = bytes((0, 20, 59, 60, 61, 180, 254, 255)) * (640 * 480 // 8)
        self.results = iter([None, 0, None, 100, KeyboardInterrupt()])
        self.run_script()
        self.assertEqual([item for item in self.events if item[0] == "detect"],
                         [("detect", 1), ("detect", 2)])
        self.assertEqual(self.events.count(("display_frame",)), 2)
        self.assertEqual(self.events.count(("preview_copy",)), 2)
        self.assertIn(("detector_close",), self.events)
        self.assert_cleaned(4)

    def test_light_and_armor_crosses_keep_separate_colors(self):
        events = []
        preview = SimpleNamespace(
            draw_rectangle=lambda *args, **kwargs: None,
            draw_string=lambda *args, **kwargs: None,
            draw_cross=lambda *args, **kwargs: events.append((args, kwargs)),
        )
        light = {"center": (100.5, 120.5), "bounds": (96, 116, 10, 10)}
        armor = {"center": (102.5, 80.5), "bounds": (92, 70, 20, 20)}
        self.script._draw_detection(preview, {
            "kind": "armor", "light": light, "armor": armor, "target": armor,
            "roi": (80, 60, 50, 90), "candidates": 3,
        })
        self.assertIn(((101, 121), {"color": (0, 255, 0), "size": 10, "thickness": 2}), events)
        self.assertIn(((103, 81), {"color": (255, 0, 0), "size": 10, "thickness": 2}), events)
        self.assertEqual(events[-1][1]["color"], (255, 0, 0))

    def test_detector_failure_closes_camera_and_pwm(self):
        self.script.DETECTION_ENABLED = True
        def fail(*_):
            raise RuntimeError("Detector detect failed: 6")
        with patch.object(self.modules["_dart_detect"].Detector, "detect", fail):
            with self.assertRaisesRegex(RuntimeError, "Detector detect failed: 6"):
                self.run_script()
        self.assertIn(("detector_close",), self.events)
        self.assert_cleaned(4)

    def test_old_grayscale_interpreter_fails_before_camera_or_pwm_start(self):
        self.script.DETECTION_ENABLED = True
        self.modules["_dart_detect"].GRAY_FEATURE_VERSION = 0
        with self.assertRaisesRegex(RuntimeError, "Grayscale feature checks require"):
            self.run_script()
        self.assertFalse(self.pwms)
        self.assertNotIn(("detector_open",), self.events)

    def test_old_tracking_interpreter_fails_before_outputs_start(self):
        self.script.DETECTION_ENABLED = True
        self.modules["_dart_detect"].TRACK_FEATURE_VERSION = 0
        with self.assertRaisesRegex(RuntimeError, "Detect/track requires"):
            self.run_script()
        self.assertFalse(self.pwms)
        self.assertNotIn(("detector_open",), self.events)


if __name__ == "__main__":
    unittest.main()
