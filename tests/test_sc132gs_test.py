"""Host checks for the standalone CanMV SC132GS smoke test, not hardware tests."""

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, call, mock_open, patch


SCRIPT = Path(__file__).resolve().parents[1] / "tools/sc132gs_test.py"


class Sc132gsTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location("sc132gs_under_test", SCRIPT)
        self.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.script)
        self.default_check_chip_id = self.script.CHECK_CHIP_ID
        self.default_use_vga_mode = self.script.USE_VGA_MODE
        self.default_color_sampling_enabled = self.script.COLOR_SAMPLING_ENABLED
        self.default_output_monitor_enabled = self.script.OUTPUT_MONITOR_ENABLED
        self.default_native_full_frame = self.script.NATIVE_FULL_FRAME
        self.default_preview_fps = self.script.PREVIEW_FPS
        self.script.CHECK_CHIP_ID = True
        self.script.PROBE_ONLY = False
        self.script.USE_VGA_MODE = True
        self.script.PREVIEW_ENABLED = True
        self.script.TEST_SECONDS = 1
        self.script.EXPOSURE_US = 5000
        self.script.FRAME_TIMEOUT_MS = 1000
        self.script.DIAGNOSTICS_ENABLED = False
        self.script.COLOR_SAMPLING_ENABLED = False
        self.script.DETECTION_ENABLED = False
        self.script.RECORDING_ENABLED = False
        self.script.RAW_CHECK_ENABLED = False
        self.script.RAW_BLACK_LEVEL = None
        self.script.OUTPUT_MONITOR_ENABLED = False
        self.script.AUTO_WHITE_BALANCE = True
        self.script.PREVIEW_FPS = 10
        self.script.NATIVE_FULL_FRAME = False
        self.clock = 0
        self.output = ""
        self.frame = SimpleNamespace(width=lambda: 640, height=lambda: 480,
                                     format=lambda: 5, save=Mock())
        self.sensor = Mock()
        self.sensor.get_type.return_value = 11
        self.sensor._dev_attr = SimpleNamespace(sensor_info=SimpleNamespace(
            type=11, width=640, height=480, fps=240, mipi_lanes=3, data_type=0x2b,
        ), pipe_ctrl=SimpleNamespace(bits=SimpleNamespace(
            ae_enable=1, awb_enable=1, wb_enable=1, demosaic_enable=1, ccm_enable=1,
        )))
        self.sensor._chn_attr = [SimpleNamespace(out_win=SimpleNamespace(width=0))]
        self.sensor._framesize = [None]
        self.sensor.auto_exposure.side_effect = lambda enable: setattr(
            self.sensor._dev_attr.pipe_ctrl.bits, "ae_enable", int(enable)) or True
        self.sensor.again.return_value = 0
        self.sensor.exposure.return_value = True
        self.sensor.snapshot.side_effect = self.snapshot
        self.board = SimpleNamespace(
            CAMERA_NAME="SC132GS", CAMERA_I2C=1, CAMERA_CSI=0,
            CAMERA_MODE=7, CAMERA_VGA_MODE=11,
            camera_probe=Mock(return_value=(0x30, 0x0132)),
            camera_sensor=Mock(return_value=self.sensor),
        )
        self.display = SimpleNamespace(VIRT=0, init=Mock(), show_image=Mock(), deinit=Mock())
        self.preview_type = self.script.DisplayPreview
        self.preview_factory = Mock(return_value=self.display)
        self.script.DisplayPreview = self.preview_factory
        self.script.os = SimpleNamespace(exitpoint=Mock(), EXITPOINT_ENABLE_SLEEP=1)
        self.script.time = SimpleNamespace(
            ticks_ms=lambda: self.clock % (1 << 30),
            ticks_diff=lambda end, start: (end - start + (1 << 29)) % (1 << 30) - (1 << 29),
            sleep_ms=Mock(),
        )
        sensor_module = ModuleType("media.sensor")
        sensor_module.Sensor = SimpleNamespace(RGB565=2, RGB888=3)
        display_module = ModuleType("media.display")
        display_module.Display = self.display
        image_module = ModuleType("image")
        image_module.RGB565 = 5
        self.modules = {"dart_board": self.board, "media": ModuleType("media"),
                        "media.sensor": sensor_module, "media.display": display_module,
                        "image": image_module}

    def snapshot(self, **kwargs):
        self.clock += 250
        return self.frame

    def run_main(self):
        output = io.StringIO()
        try:
            with patch.dict(sys.modules, self.modules), contextlib.redirect_stdout(output):
                self.script.main()
        finally:
            self.output = output.getvalue()

    def assert_released(self):
        self.sensor.stop.assert_called_once_with()
        self.display.deinit.assert_called_once_with()
        self.script.time.sleep_ms.assert_called_once_with(100)

    def test_preview_capture_and_cleanup(self):
        self.run_main()
        self.board.camera_probe.assert_called_once_with()
        self.board.camera_sensor.assert_called_once_with(vga=True)
        self.sensor.reset.assert_called_once_with()
        self.sensor.set_framesize.assert_called_once_with(width=640, height=480, crop=True)
        self.sensor.set_pixformat.assert_called_once_with(2)
        self.sensor.auto_exposure.assert_called_once_with(False)
        self.sensor.again.assert_called_once_with(self.script.ANALOG_GAIN)
        self.sensor.exposure.assert_called_once_with(5000)
        methods = [call[0] for call in self.sensor.mock_calls]
        self.assertLess(methods.index("auto_exposure"), methods.index("run"))
        self.assertLess(methods.index("run"), methods.index("again"))
        self.assertLess(methods.index("run"), methods.index("exposure"))
        self.sensor.snapshot.assert_called_with(timeout=1000, auto_reboot=False)
        self.assertEqual(self.sensor.snapshot.call_count, 4)
        self.preview_factory.assert_called_once_with(640, 480)
        self.assertEqual(self.display.show_image.call_count, 4)
        self.assert_released()
        self.assertIn("[PASS] probe:", self.output)
        self.assertIn("[PASS] first frame:", self.output)
        self.assertIn("[PASS] capture: frames=4 elapsed_ms=1000", self.output)

    def test_finite_capture_never_saves_images(self):
        self.script.TEST_SECONDS = 20
        self.frame.save.side_effect = AssertionError("Preview must not save images")
        self.run_main()
        self.assertEqual(self.sensor.snapshot.call_count, 80)
        self.frame.save.assert_not_called()
        self.assertIn("[PASS] capture: frames=80 elapsed_ms=20000", self.output)
        self.assert_released()

    def test_continuous_preview_keeps_running_until_ide_stop_without_saving(self):
        self.script.TEST_SECONDS = 0
        self.script.DIAGNOSTICS_ENABLED = True
        self.frame.save.side_effect = AssertionError("Preview must not save images")
        def snapshot(**kwargs):
            if self.sensor.snapshot.call_count > 80:
                raise Exception("IDE interrupt")
            return self.snapshot(**kwargs)
        self.sensor.snapshot.side_effect = snapshot
        with patch.object(self.script, "diagnose_capture") as diagnose:
            self.run_main()
        self.assertEqual(self.sensor.snapshot.call_count, 81)
        self.assertEqual(self.display.show_image.call_count, 80)
        self.frame.save.assert_not_called()
        diagnose.assert_not_called()
        self.assertIn("Continuous color preview", self.output)
        self.assertIn("elapsed_ms=20000", self.output)
        self.assertIn("[STOP] IDE interrupted", self.output)
        self.assertNotIn("[FAIL]", self.output)
        self.assertNotIn("[PASS] capture:", self.output)
        self.assert_released()

    def test_auto_white_balance_is_the_default_and_applied_before_stream_start(self):
        bits = self.sensor._dev_attr.pipe_ctrl.bits

        def reset():
            bits.ae_enable = bits.awb_enable = bits.wb_enable = 0

        def run():
            self.assertEqual((bits.ae_enable, bits.awb_enable, bits.wb_enable), (0, 1, 1))

        self.sensor.reset.side_effect = reset
        self.sensor.run.side_effect = run
        spec = importlib.util.spec_from_file_location("sc132gs_defaults", SCRIPT)
        defaults = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(defaults)
        self.assertIs(defaults.AUTO_WHITE_BALANCE, True)
        self.run_main()
        self.sensor.run.assert_called_once_with()
        self.assertIn("AE=off AWB=on WB=on", self.output)
        self.assertNotIn("blue-violet", self.output)

    def test_isp_controls_are_disabled_after_reset_before_stream_start(self):
        self.script.AUTO_WHITE_BALANCE = False
        bits = self.sensor._dev_attr.pipe_ctrl.bits

        def reset():
            bits.ae_enable = bits.awb_enable = bits.wb_enable = 1

        def run():
            self.assertEqual((bits.ae_enable, bits.awb_enable, bits.wb_enable), (0, 0, 0))
            self.assertEqual((bits.demosaic_enable, bits.ccm_enable), (1, 1))

        self.sensor.reset.side_effect = reset
        self.sensor.run.side_effect = run
        self.script.EXPOSURE_US = 2000
        self.script.ANALOG_GAIN = 2.0
        self.run_main()
        self.sensor.set_auto_whitebal.assert_not_called()
        self.sensor.set_auto_gain.assert_not_called()
        self.sensor.again.assert_called_once_with(2.0)
        self.sensor.exposure.assert_called_once_with(2000)
        methods = [entry[0] for entry in self.sensor.mock_calls]
        self.assertLess(methods.index("run"), methods.index("again"))
        self.assertLess(methods.index("run"), methods.index("exposure"))
        self.assertLess(methods.index("exposure"), methods.index("snapshot"))
        self.assertIn("AE=off AWB=off WB=off", self.output)
        self.assertIn("fixed WB gains (R 1.12, B 2.51)", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_missing_isp_controls_fail_before_stream_start(self):
        class LegacyBits:
            __slots__ = ("ae_enable",)

        self.sensor._dev_attr.pipe_ctrl.bits = LegacyBits()
        with self.assertRaisesRegex(RuntimeError, "Firmware lacks ISP AWB/WB controls"):
            self.run_main()
        self.sensor.run.assert_not_called()
        self.display.init.assert_not_called()
        self.sensor.stop.assert_called_once_with()
        self.assertIn("[FAIL] firmware ISP configuration", self.output)
        self.assertNotIn("[PASS] capture:", self.output)

    def test_temporal_noise_reduction_is_disabled_before_stream_start(self):
        bits = self.sensor._dev_attr.pipe_ctrl.bits
        bits.dnr3_enable = 1
        self.sensor.run.side_effect = lambda: self.assertEqual(bits.dnr3_enable, 0)
        self.run_main()
        self.sensor.run.assert_called_once_with()
        self.assertIn("3DNR=off", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_raw_check_uses_bayer_channel_without_preview_or_capture(self):
        self.script.RAW_CHECK_ENABLED = True
        self.script.COLOR_SAMPLING_ENABLED = True
        video_struct = ModuleType("mpp.video_struct")
        video_struct.PIXEL_FORMAT_RGB_BAYER_10BPP = 20
        self.modules.update({"mpp": ModuleType("mpp"), "mpp.video_struct": video_struct})
        checker = Mock()
        checker.return_value.run.return_value = True
        self.script.RawCheck = checker
        self.run_main()
        self.sensor.set_framesize.assert_called_once_with(width=640, height=480)
        self.sensor.set_pixformat.assert_called_once_with(20)
        checker.assert_called_once_with(self.sensor)
        checker.return_value.run.assert_called_once_with(
            self.script.RAW_CHECK_FRAMES, self.script.FRAME_TIMEOUT_MS)
        methods = [entry[0] for entry in self.sensor.mock_calls]
        self.assertLess(methods.index("run"), methods.index("exposure"))
        self.sensor.snapshot.assert_not_called()
        self.display.init.assert_not_called()
        self.sensor.stop.assert_called_once_with()
        self.assertIn("[PASS] RAW check complete", self.output)
        self.assertNotIn("[COLOR]", self.output)
        self.assertNotIn("[PASS] capture:", self.output)

    RAW_BGGR = {(0, 0): 300, (0, 1): 600, (1, 0): 590, (1, 1): 360}

    def raw_rows(self, levels, width=1080, bad_lane=None, row_shift=0):
        lane_of = [((x >> 2) * 5 + (x & 3)) % 4 for x in range(width)]

        def row(y):
            values = []
            for x in range(width):
                value = levels[((y + row_shift) & 1, x & 1)] + (x * 7 + y * 13) % 5 - 2
                if bad_lane is not None and lane_of[x] == bad_lane and (x * 3 + y) % 5 == 0:
                    value ^= 0x200
                values.append(value)
            return values

        return row

    def raw_report(self, frames):
        checker = self.script.RawCheck.__new__(self.script.RawCheck)
        checker.setup(1080, 4)
        checker.layout = (1080, 1280, 2160, 1023)
        for row in frames:
            checker.results.append(checker.measure(row, 1080, 1280, 1023))
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            clean = checker.report()
        return clean, output.getvalue()

    def test_raw_report_accepts_bggr_without_black_level_estimate(self):
        clean, output = self.raw_report([self.raw_rows(self.RAW_BGGR)] * 3)
        self.assertTrue(clean)
        self.assertIn("green at (0,1)/(1,0)", output)
        self.assertIn("phase flips: frames=0/3 bands=0/24", output)
        self.assertIn("RAW_BLACK_LEVEL unknown", output)
        self.assertIn("no WB gain estimate", output)
        self.assertNotIn("WB gains", output)
        self.assertIn("RAW data is clean", output)

    def test_raw_report_estimates_white_balance_above_measured_black(self):
        self.script.RAW_BLACK_LEVEL = 32
        clean, output = self.raw_report([self.raw_rows(self.RAW_BGGR)] * 3)
        self.assertTrue(clean)
        self.assertIn("WB gains R=1.72 B=2.10", output)
        self.assertNotIn("RAW_BLACK_LEVEL unknown", output)

    def test_raw_report_covered_lens_measures_black_level(self):
        covered = {(0, 0): 40, (0, 1): 40, (1, 0): 40, (1, 1): 40}
        clean, output = self.raw_report([self.raw_rows(covered)] * 3)
        self.assertIsNone(clean)
        self.assertIn("set RAW_BLACK_LEVEL = 40", output)
        self.assertNotIn("monochrome", output)
        self.assertNotIn("WB gains", output)

    def test_raw_report_underexposed_board_frame_has_no_color_estimate(self):
        # Codes measured on the board on 2026-11-12 at 5000 us and 1x gain.
        board = {(0, 0): 47, (0, 1): 55, (1, 0): 55, (1, 1): 48}
        for black in (None, 32):
            with self.subTest(black=black):
                self.script.RAW_BLACK_LEVEL = black
                clean, output = self.raw_report([self.raw_rows(board)] * 3)
                self.assertTrue(clean)
                self.assertIn("underexposed", output)
                self.assertIn("green at (0,1)/(1,0)", output)
                self.assertIn("no WB gain estimate", output)
                self.assertNotIn("WB gains", output)
                self.assertNotIn("RAW data is clean", output)

    def test_raw_report_rejects_black_level_above_a_phase(self):
        self.script.RAW_BLACK_LEVEL = 320
        clean, output = self.raw_report([self.raw_rows(self.RAW_BGGR)] * 3)
        self.assertIsNone(clean)
        self.assertIn("at or below RAW_BLACK_LEVEL=320", output)

    def test_raw_report_detects_phase_slip_with_measured_black(self):
        self.script.RAW_BLACK_LEVEL = 32
        clean, output = self.raw_report([self.raw_rows(self.RAW_BGGR, row_shift=shift)
                                         for shift in (0, 1, 0, 0)])
        self.assertFalse(clean)
        self.assertIn("frames=1/4", output)
        self.assertIn("findings before the ISP: phase slip", output)

    def test_output_monitor_reports_isp_color_jumps(self):
        self.script.OUTPUT_MONITOR_ENABLED = True
        self.script.PREVIEW_ENABLED = False
        values = [(50, 0, 0), (51, 1, -1), (50, 20, -20), (50, 0, 0)]
        self.frame.get_statistics = Mock(side_effect=[
            SimpleNamespace(l_mean=lambda v=v: v[0], a_mean=lambda v=v: v[1],
                            b_mean=lambda v=v: v[2]) for v in values])
        self.run_main()
        self.frame.get_statistics.assert_called_with(roi=(160, 120, 320, 240))
        self.assertEqual(self.frame.get_statistics.call_count, 4)
        self.assertIn("[ISP] jump at frame 3: Lab (51, 1, -1) -> (50, 20, -20)", self.output)
        self.assertIn("[ISP] jump at frame 4:", self.output)
        self.assertIn("[ISP] center Lab L=50..51 a=0..20 b=-20..0 jumps=2", self.output)
        self.assertIn("[ISP] output jumps=2 over 4 frames", self.output)
        self.assertIn("[PASS] capture: frames=4", self.output)
        self.sensor.stop.assert_called_once_with()

    def test_output_monitor_is_off_by_default(self):
        self.assertIs(self.default_output_monitor_enabled, False)
        self.frame.get_statistics = Mock(side_effect=AssertionError("monitor must be off"))
        self.run_main()
        self.assertNotIn("[ISP]", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_raw_report_detects_sensor_side_faults(self):
        wrong = {(0, 0): 600, (0, 1): 360, (1, 0): 300, (1, 1): 590}
        mono = {(0, 0): 500, (0, 1): 502, (1, 0): 499, (1, 1): 501}
        cases = (
            ("bayer", [self.raw_rows(wrong)] * 3, "firmware declares BGGR"),
            ("mono", [self.raw_rows(mono)] * 3, "monochrome sensor"),
            ("phase slip", [self.raw_rows(self.RAW_BGGR, row_shift=shift)
                            for shift in (0, 1, 0, 0)], "frames=1/4"),
            ("lane", [self.raw_rows(self.RAW_BGGR, bad_lane=2)] * 3, "byte lane 2"),
        )
        for fault, frames, message in cases:
            with self.subTest(fault=fault):
                clean, output = self.raw_report(frames)
                self.assertFalse(clean)
                self.assertIn(message, output)
                self.assertIn("findings before the ISP: %s" % fault, output)
                self.assertNotIn("WB gains", output)

    def raw_native(self, width=1080, height=1280, stride=2160, pixel_format=20,
                   dump_result=0):
        calls = []
        rows = self.raw_rows(self.RAW_BGGR, width)
        base = 0x1000000

        def dump(device, channel, kind, info, timeout):
            calls.append(("dump", device, channel, kind, timeout))
            info.v_frame = SimpleNamespace(
                width=width, height=height, pixel_format=pixel_format,
                stride=[stride, 0, 0], phys_addr=[0x8000, 0, 0])
            return dump_result

        uctypes = ModuleType("uctypes")
        uctypes.ARRAY = uctypes.UINT16 = uctypes.LITTLE_ENDIAN = 0
        uctypes.struct = lambda address, layout, endian: SimpleNamespace(
            v=rows((address - base) // max(stride, width * 2)))
        sys_module = ModuleType("mpp.sys")
        sys_module.kd_mpi_sys_mmap = lambda phys, size: calls.append(("mmap", phys, size)) or base
        sys_module.kd_mpi_sys_munmap = lambda virt, size: calls.append(("munmap", virt, size)) or 0
        vicap = ModuleType("mpp.vicap")
        vicap.VICAP_DUMP_RAW = 2
        vicap.VICAP_MIPI_4LANE = 3
        vicap.kd_mpi_vicap_dump_frame = dump
        vicap.kd_mpi_vicap_dump_release = lambda device, channel, info: calls.append(
            ("release", device, channel)) or 0
        video_struct = ModuleType("mpp.video_struct")
        video_struct.PIXEL_FORMAT_RGB_BAYER_10BPP = 20
        video_struct.PIXEL_FORMAT_RGB_BAYER_12BPP = 21
        video_struct.k_video_frame_info = lambda: SimpleNamespace(v_frame=None)
        self.modules.update({"uctypes": uctypes, "mpp": ModuleType("mpp"), "mpp.sys": sys_module,
                             "mpp.vicap": vicap, "mpp.video_struct": video_struct})
        self.sensor._dev_id = 0
        self.sensor._dev_attr.sensor_info = SimpleNamespace(
            width=1080, height=1280, mipi_lanes=3)
        return calls

    def test_raw_check_releases_every_dumped_frame(self):
        calls = self.raw_native()
        output = io.StringIO()
        with patch.dict(sys.modules, self.modules), contextlib.redirect_stdout(output):
            clean = self.script.RawCheck(self.sensor).run(2, 1000)
        self.assertTrue(clean)
        frames = self.script.RawCheck.WARMUP_FRAMES + 2
        self.assertEqual([entry[0] for entry in calls], ["dump", "mmap", "munmap", "release"] * frames)
        self.assertEqual(calls[0], ("dump", 0, 0, 2, 1000))
        self.assertEqual(calls[1], ("mmap", 0x8000, 2160 * 1280))
        self.assertEqual(self.script.os.exitpoint.call_count, frames)
        self.assertIn("RAW data is clean", output.getvalue())

    def test_raw_check_releases_unexpected_frame_before_failing(self):
        calls = self.raw_native(pixel_format=99)
        with patch.dict(sys.modules, self.modules):
            with self.assertRaisesRegex(RuntimeError, "Unexpected RAW frame 1080x1280 format=99"):
                self.script.RawCheck(self.sensor).run(2, 1000)
        self.assertEqual([entry[0] for entry in calls], ["dump", "release"])

    def test_raw_check_does_not_release_failed_dump(self):
        calls = self.raw_native(dump_result=-1)
        with patch.dict(sys.modules, self.modules):
            with self.assertRaisesRegex(RuntimeError, "RAW dump returned -1"):
                self.script.RawCheck(self.sensor).run(2, 1000)
        self.assertEqual([entry[0] for entry in calls], ["dump"])

    def test_grayscale_frame_is_not_accepted_as_color(self):
        self.frame.format = lambda: 1
        with self.assertRaisesRegex(RuntimeError, "RGB565 color frame"):
            self.run_main()
        self.display.show_image.assert_not_called()
        self.frame.save.assert_not_called()
        self.assertNotIn("[PASS] first frame:", self.output)
        self.assertNotIn("[PASS] capture:", self.output)
        self.assert_released()

    def test_chip_id_is_skipped_by_default(self):
        self.assertFalse(self.default_check_chip_id)
        self.script.CHECK_CHIP_ID = self.default_check_chip_id
        self.board.camera_probe.side_effect = AssertionError("must not probe chip ID")
        self.run_main()
        self.board.camera_probe.assert_not_called()
        self.sensor.run.assert_called_once_with()
        self.assertIn("Chip ID check skipped", self.output)
        self.assertIn("Native sensor / ISP ID access is controlled by firmware", self.output)
        self.assertNotIn("[PASS] probe:", self.output)
        self.assertIn("[PASS] capture:", self.output)
        self.assert_released()

    def test_chip_id_bypass_does_not_require_probe_helper(self):
        self.script.CHECK_CHIP_ID = False
        del self.board.camera_probe
        self.run_main()
        self.sensor.run.assert_called_once_with()
        self.assertIn("[PASS] capture:", self.output)
        self.assert_released()

    def test_probe_only_overrides_chip_id_bypass(self):
        self.script.PROBE_ONLY = True
        self.script.CHECK_CHIP_ID = False
        self.run_main()
        self.board.camera_probe.assert_called_once_with()
        self.board.camera_sensor.assert_not_called()
        self.assertIn("[PASS] probe:", self.output)

    def test_probe_only_does_not_import_media_or_start_camera(self):
        self.script.PROBE_ONLY = True
        self.modules["media.sensor"] = None
        self.modules["media.display"] = None
        self.modules["image"] = None
        self.run_main()
        self.board.camera_sensor.assert_not_called()
        self.sensor.stop.assert_not_called()
        self.display.init.assert_not_called()
        self.assertIn("capture has NOT been tested", self.output)
        self.assertNotIn("[PASS] capture:", self.output)

    def test_headless_capture_does_not_import_display(self):
        self.script.PREVIEW_ENABLED = False
        self.modules["media.display"] = None
        self.run_main()
        self.sensor.stop.assert_called_once_with()
        self.display.init.assert_not_called()
        self.display.show_image.assert_not_called()
        self.display.deinit.assert_not_called()
        self.frame.save.assert_not_called()
        self.assertIn("[PASS] capture:", self.output)

    def configure_full_mode(self, frame_ms=250, native=False):
        self.script.USE_VGA_MODE = False
        self.script.NATIVE_FULL_FRAME = native
        self.sensor.get_type.return_value = self.board.CAMERA_MODE
        info = self.sensor._dev_attr.sensor_info
        info.type = self.board.CAMERA_MODE
        info.width, info.height, info.fps = 1080, 1280, 120
        size = (1080, 1280) if native else (480, 640)
        self.frame.width = lambda: size[0]
        self.frame.height = lambda: size[1]
        if native:
            # The PITCH_FIX firmware exports the row packer used by Sensor.snapshot().
            vicap = ModuleType("mpp.vicap")
            vicap.vb_mgmt_pack_vicap_frame = Mock()
            mpp = ModuleType("mpp")
            mpp.vicap = vicap
            self.modules.update({"mpp": mpp, "mpp.vicap": vicap})

        def snapshot(**kwargs):
            self.clock += frame_ms
            return self.frame

        self.sensor.snapshot.side_effect = snapshot

    def test_full_sensor_mode_scales_the_whole_view_to_480x640(self):
        self.configure_full_mode()
        self.run_main()
        self.board.camera_sensor.assert_called_once_with(vga=False)
        self.sensor.set_framesize.assert_called_once_with(width=480, height=640)
        self.sensor.set_pixformat.assert_called_once_with(2)
        self.sensor.snapshot.assert_called_with(timeout=1000, auto_reboot=False)
        self.assertEqual(self.sensor.snapshot.call_count, 4)
        self.assertEqual(self.display.show_image.call_count, 4)
        self.preview_factory.assert_called_once_with(480, 640)
        self.assertIn("1080x1280 full sensor", self.output)
        self.assertIn("type=7 RAW10 nominal_fps=120", self.output)
        self.assertIn("output=480x640 RGB565", self.output)
        self.assertIn("1080x1280 full view -> 480x640 RGB565 (ISP-scaled, uncropped)", self.output)
        self.assertIn("virtual display 480x640, hardware JPEG", self.output)
        self.assertIn("[PASS] first frame: 480x640 RGB565", self.output)
        self.assertIn("[PASS] capture: frames=4", self.output)
        self.assert_released()

    def test_native_full_frame_delivers_1080x1280(self):
        self.configure_full_mode(native=True)
        self.run_main()
        self.sensor.set_framesize.assert_called_once_with(width=1080, height=1280)
        self.sensor.set_pixformat.assert_called_once_with(2)
        self.preview_factory.assert_called_once_with(1080, 1280)
        self.assertIn("1080x1280 full view -> 1080x1280 RGB565 (native, rows packed", self.output)
        self.assertIn("[PASS] first frame: 1080x1280 RGB565", self.output)
        self.assertIn("[PASS] capture: frames=4", self.output)
        self.assert_released()

    def test_native_full_frame_requires_the_pitch_fix_firmware(self):
        self.configure_full_mode(native=True)
        del self.modules["mpp.vicap"].vb_mgmt_pack_vicap_frame
        with self.assertRaisesRegex(RuntimeError, "PITCH_FIX firmware"):
            self.run_main()
        self.sensor.set_framesize.assert_not_called()
        self.sensor.run.assert_not_called()
        self.sensor.stop.assert_called_once_with()
        self.assertIn("NATIVE_FULL_FRAME=False for 480x640", self.output)

    def test_scaled_full_view_is_the_default(self):
        self.assertIs(self.default_native_full_frame, False)
        self.assertEqual(self.default_preview_fps, 60)

    def test_preview_is_rate_limited_while_counting_every_frame(self):
        self.configure_full_mode(frame_ms=10)
        self.run_main()
        self.assertEqual(self.sensor.snapshot.call_count, 100)
        self.assertEqual(self.display.show_image.call_count, 10)
        self.assertIn("[PASS] capture: frames=100", self.output)

    def test_full_mode_headless_counts_every_frame(self):
        self.configure_full_mode(frame_ms=10)
        self.script.PREVIEW_ENABLED = False
        self.run_main()
        self.assertEqual(self.sensor.snapshot.call_count, 100)
        self.display.show_image.assert_not_called()
        self.assertIn("[PASS] capture: frames=100", self.output)

    def test_full_preview_rejects_wrong_frames(self):
        self.configure_full_mode()
        self.frame.width = lambda: 1080
        self.frame.height = lambda: 1280
        with patch.dict(sys.modules, self.modules), self.assertRaisesRegex(
                RuntimeError, "nonempty 480x640"):
            self.script.capture_frames(self.sensor, self.display)
        self.display.show_image.assert_not_called()

    def test_default_mode_is_datasheet_full_resolution(self):
        self.assertFalse(self.default_use_vga_mode)

    def test_mode_log_uses_firmware_metadata(self):
        self.sensor._dev_attr.sensor_info.fps = 237
        self.run_main()
        self.assertIn("firmware sensor_mode=640x480 crop type=11 RAW10 nominal_fps=237",
                      self.output)
        self.assertIn("not proof of sensor initialization", self.output)

    def test_inconsistent_firmware_modes_fail_before_stream_start(self):
        for field, value in (("selected", 7), ("type", 7), ("width", 1080),
                             ("height", 1280), ("data_type", 0x2a)):
            with self.subTest(field=field):
                if field == "selected":
                    self.sensor.get_type.return_value = value
                else:
                    setattr(self.sensor._dev_attr.sensor_info, field, value)
                with self.assertRaisesRegex(RuntimeError, "firmware mode mismatch|RAW10 required"):
                    self.run_main()
                self.sensor.run.assert_not_called()
                self.display.init.assert_not_called()
                self.assertNotIn("[PASS] capture:", self.output)
                self.sensor.get_type.return_value = 11
                info = self.sensor._dev_attr.sensor_info
                info.type, info.width, info.height, info.data_type = 11, 640, 480, 0x2b

    def test_old_camera_firmware_is_rejected_before_probe(self):
        self.board.CAMERA_NAME = "SC038HGS"
        with self.assertRaisesRegex(RuntimeError, "SC132GS firmware required"):
            self.run_main()
        self.board.camera_probe.assert_not_called()
        self.board.camera_sensor.assert_not_called()

    def test_missing_board_module_reports_firmware_stage(self):
        self.modules["dart_board"] = None
        with self.assertRaises(ImportError):
            self.run_main()
        self.assertIn("[FAIL] firmware / I2C probe", self.output)
        self.board.camera_sensor.assert_not_called()

    def test_missing_camera_helpers_are_rejected(self):
        del self.board.camera_sensor
        with self.assertRaisesRegex(RuntimeError, "camera helpers"):
            self.run_main()
        self.board.camera_probe.assert_not_called()

    def test_unexpected_chip_or_address_is_rejected_before_reset(self):
        for result in ((0x30, 0x1A79), (0x34, 0x0132)):
            with self.subTest(result=result):
                self.board.camera_probe.return_value = result
                with self.assertRaisesRegex(RuntimeError, "Unexpected sensor"):
                    self.run_main()
        self.board.camera_sensor.assert_not_called()
        self.sensor.reset.assert_not_called()

    def test_unpowered_sensor_is_rechecked_after_driver_start(self):
        self.board.camera_probe.side_effect = [OSError("powered off"), (0x30, 0x0132)]
        self.run_main()
        self.assertEqual(self.board.camera_probe.call_count, 2)
        self.sensor.run.assert_called_once_with()
        self.assert_released()
        self.assertIn("Rechecking after driver power-on", self.output)
        self.assertIn("I2C probe unavailable before stream start: powered off", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_i2c_error_after_power_on_does_not_start_capture(self):
        self.board.camera_probe.side_effect = OSError("no I2C response")
        with self.assertRaisesRegex(OSError, "no I2C response"):
            self.run_main()
        self.sensor.run.assert_called_once_with()
        self.sensor.snapshot.assert_not_called()
        self.assert_released()
        self.assertIn("[FAIL] powered I2C probe", self.output)
        self.assertNotIn("[PASS] probe:", self.output)

    def test_probe_only_does_not_try_to_power_on_an_unavailable_sensor(self):
        self.script.PROBE_ONLY = True
        self.board.camera_probe.side_effect = OSError("powered off")
        with self.assertRaisesRegex(OSError, "powered off"):
            self.run_main()
        self.board.camera_sensor.assert_not_called()

    def test_partial_sensor_reset_failure_releases_sensor(self):
        self.sensor.reset.side_effect = RuntimeError("reset failed")
        with self.assertRaisesRegex(RuntimeError, "reset failed"):
            self.run_main()
        self.sensor.stop.assert_called_once_with()
        self.display.deinit.assert_not_called()

    def test_partial_display_init_failure_releases_both_resources(self):
        self.preview_factory.side_effect = RuntimeError("display failed")
        with self.assertRaisesRegex(RuntimeError, "display failed"):
            self.run_main()
        self.sensor.stop.assert_called_once_with()
        self.display.deinit.assert_not_called()
        self.script.time.sleep_ms.assert_called_once_with(100)
        self.sensor.run.assert_not_called()

    def test_display_preview_uses_the_virtual_display_with_hardware_jpeg(self):
        with patch.dict(sys.modules, self.modules):
            preview = self.preview_type(480, 640)
        self.display.init.assert_called_once_with(0, width=480, height=640, fps=60, to_ide=True)
        preview.show_image(self.frame)
        self.display.show_image.assert_called_once_with(self.frame)
        preview.deinit()
        self.display.deinit.assert_called_once_with()

    def test_main_previews_every_due_frame_through_the_display(self):
        self.script.DisplayPreview = self.preview_type
        self.run_main()
        self.display.init.assert_called_once_with(0, width=640, height=480, fps=60, to_ide=True)
        self.assertEqual(self.display.show_image.call_args_list, [call(self.frame)] * 4)
        self.assert_released()
        self.assertIn("virtual display 640x480, hardware JPEG", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_display_failure_still_stops_camera_and_preserves_error(self):
        self.script.DisplayPreview = self.preview_type
        self.display.show_image.side_effect = OSError("display busy")
        with self.assertRaisesRegex(OSError, "display busy"):
            self.run_main()
        self.sensor.stop.assert_called_once_with()
        self.display.deinit.assert_called_once_with()
        self.assertNotIn("[PASS] capture:", self.output)

    def test_stream_start_failure_releases_both_resources(self):
        self.sensor.run.side_effect = RuntimeError("MIPI failed")
        with self.assertRaisesRegex(RuntimeError, "MIPI failed"):
            self.run_main()
        self.assert_released()
        self.assertIn("[FAIL] MIPI / ISP stream start", self.output)
        self.assertNotIn("[PASS] capture:", self.output)

    def configure_startup_diagnostics(self):
        self.configure_diagnostics()
        self.script.open = mock_open()
        self.script.os.stat = Mock(return_value=(0, 0, 0, 0, 0, 0, 409656))
        self.startup_error = RuntimeError("sensor(0) run error, vicap init failed(-1609203696)")
        self.sensor.run.side_effect = self.startup_error

    def test_failed_start_does_not_reprobe_after_native_poweroff(self):
        self.configure_startup_diagnostics()
        self.board.camera_probe.side_effect = [(0x30, 0x0132), OSError(1, "EPERM")]
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.board.camera_probe.assert_called_once_with()
        self.bus.readfrom_mem.assert_not_called()
        self.assertIn("Startup I2C recheck skipped", self.output)
        self.assertNotIn("I2C remains unavailable", self.output)
        self.assert_released()

    def test_failed_start_does_not_misdiagnose_poweroff_as_eperm(self):
        self.configure_startup_diagnostics()
        self.script.CHECK_CHIP_ID = False
        self.bus.readfrom_mem.side_effect = OSError(1, "EPERM")
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.board.camera_probe.assert_not_called()
        self.modules["machine"].I2C.assert_not_called()
        self.bus.readfrom_mem.assert_not_called()
        self.assertIn("native failure cleanup may already have powered off", self.output)
        self.assertNotIn("EPERM", self.output)
        self.assert_released()

    def test_notready_startup_skips_unpowered_i2c_and_tpg(self):
        self.configure_startup_diagnostics()
        self.board.camera_probe.side_effect = OSError("no I2C response")
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.assertIn("0xa0158010 = K_ERR_VICAP_NOTREADY", self.output)
        self.assertIn("registration alone does not prove current I2C", self.output)
        self.assertIn("Startup I2C recheck skipped", self.output)
        self.assertIn("Preserve the first native sensor / ISP error on UART0", self.output)
        self.script.open.assert_called_once_with("/dev/sensor_sc132gs_csi0", "r+b")
        self.board.camera_probe.assert_called_once_with()
        self.modules["machine"].I2C.assert_not_called()
        self.sensor.snapshot.assert_not_called()
        self.tpg.assert_not_called()
        self.assertNotIn("[PASS] capture:", self.output)
        self.assert_released()

    def test_startup_diagnostics_respect_chip_id_bypass(self):
        self.configure_startup_diagnostics()
        self.script.CHECK_CHIP_ID = False
        self.board.camera_probe.side_effect = AssertionError("must not probe chip ID")
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.board.camera_probe.assert_not_called()
        self.assertNotIn("I2C remains unavailable", self.output)
        self.modules["machine"].I2C.assert_not_called()
        self.bus.readfrom_mem.assert_not_called()
        self.assertIn("Startup I2C recheck skipped", self.output)
        self.assertEqual(self.script.os.stat.call_count, 4)
        self.assert_released()

    def test_startup_diagnostics_do_not_touch_the_configured_i2c_address(self):
        self.configure_startup_diagnostics()
        self.script.CHECK_CHIP_ID = False
        self.script.I2C_ADDRESS = 0x31
        self.bus.readfrom_mem.side_effect = OSError("no ACK")
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.board.camera_probe.assert_not_called()
        self.bus.readfrom_mem.assert_not_called()
        self.assertNotIn("no ACK", self.output)
        self.assertEqual(self.script.os.stat.call_count, 4)
        self.tpg.assert_not_called()
        self.assert_released()

    def test_startup_diagnostics_check_selected_isp_files_without_reprobing(self):
        self.configure_startup_diagnostics()
        self.script.USE_VGA_MODE = False
        self.sensor.get_type.return_value = self.board.CAMERA_MODE
        info = self.sensor._dev_attr.sensor_info
        info.type = self.board.CAMERA_MODE
        info.width, info.height, info.fps = 1080, 1280, 120
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.board.camera_probe.assert_called_once_with()
        self.assertIn("Startup I2C recheck skipped", self.output)
        self.assertEqual(self.script.os.stat.call_args_list, [
            call("/bin/sc132gs-1080x1280" + suffix)
            for suffix in (".bin", ".xml", "_auto.json", "_manual.json")
        ])
        self.sensor.snapshot.assert_not_called()
        self.tpg.assert_not_called()
        self.assert_released()

    def test_missing_startup_device_and_isp_files_do_not_mask_error(self):
        self.configure_startup_diagnostics()
        self.script.open.side_effect = OSError("missing device")
        self.script.os.stat.side_effect = OSError("missing file")
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.assertIn("sensor device unavailable:", self.output)
        self.assertIn("A configured Sensor type does not register a kernel device", self.output)
        self.assertIn("soft reboot does not restart RT-Smart", self.output)
        self.assertEqual(self.output.count("ISP file unavailable:"), 4)
        self.assert_released()

    def test_unexpected_startup_diagnostic_error_does_not_mask_error(self):
        self.configure_startup_diagnostics()
        self.script.diagnose_startup = Mock(side_effect=RuntimeError("diagnostic failed"))
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.assertIn("diagnostics unavailable: diagnostic failed", self.output)
        self.assert_released()

    def test_empty_startup_isp_files_are_not_reported_as_available(self):
        self.configure_startup_diagnostics()
        self.script.os.stat.return_value = (0, 0, 0, 0, 0, 0, 0)
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.assertEqual(self.output.count("empty file"), 4)
        self.assertNotIn("[DIAG] ISP file:", self.output)
        self.assert_released()

    def test_disabled_startup_diagnostics_do_not_access_devices_or_isp_files(self):
        self.configure_startup_diagnostics()
        self.script.DIAGNOSTICS_ENABLED = False
        with self.assertRaises(RuntimeError) as raised:
            self.run_main()
        self.assertIs(raised.exception, self.startup_error)
        self.script.open.assert_not_called()
        self.script.os.stat.assert_not_called()
        self.board.camera_probe.assert_called_once_with()
        self.assertIn("native VICAP / ISP errors on UART0", self.output)
        self.assertNotIn("[DIAG]", self.output)
        self.assert_released()

    def test_nonzero_gain_status_is_an_error(self):
        self.sensor.again.return_value = -1
        with self.assertRaisesRegex(RuntimeError, "analog gain failed"):
            self.run_main()
        self.assert_released()
        self.sensor.snapshot.assert_not_called()

    def test_false_exposure_status_is_an_error(self):
        self.sensor.exposure.return_value = False
        with self.assertRaisesRegex(RuntimeError, "exposure failed"):
            self.run_main()
        self.assert_released()
        self.sensor.snapshot.assert_not_called()

    def test_capture_timeout_is_not_retried_or_reported_as_success(self):
        self.sensor.snapshot.side_effect = RuntimeError("snapshot timeout")
        with self.assertRaisesRegex(RuntimeError, "snapshot timeout"):
            self.run_main()
        self.sensor.snapshot.assert_called_once_with(timeout=1000, auto_reboot=False)
        self.assert_released()
        self.assertNotIn("[PASS] capture:", self.output)

    def test_empty_frame_is_rejected(self):
        self.sensor.snapshot.side_effect = None
        self.sensor.snapshot.return_value = None
        with self.assertRaisesRegex(RuntimeError, "nonempty 640x480"):
            self.run_main()
        self.assert_released()

    def test_wrong_frame_size_is_rejected(self):
        self.frame.width = lambda: 320
        with self.assertRaisesRegex(RuntimeError, "nonempty 640x480"):
            self.run_main()
        self.assert_released()

    def test_interrupt_cleans_up_without_claiming_completed_capture(self):
        self.script.TEST_SECONDS = 0
        self.sensor.snapshot.side_effect = [self.frame, KeyboardInterrupt()]
        self.run_main()
        self.assert_released()
        self.assertIn("[STOP]", self.output)
        self.assertNotIn("[PASS] capture:", self.output)

    def test_tick_wrap_does_not_end_capture_early(self):
        self.clock = (1 << 30) - 500
        self.run_main()
        self.assertEqual(self.sensor.snapshot.call_count, 4)
        self.assertIn("elapsed_ms=1000", self.output)

    def test_ide_interrupt_does_not_run_diagnostics(self):
        self.script.DIAGNOSTICS_ENABLED = True
        self.sensor.snapshot.side_effect = Exception("IDE interrupt")
        with patch.object(self.script, "diagnose_capture") as diagnose:
            self.run_main()
        diagnose.assert_not_called()
        self.assert_released()
        self.assertIn("[STOP] IDE interrupted", self.output)

    def configure_diagnostics(self):
        self.script.DIAGNOSTICS_ENABLED = True
        self.sensor._dev_attr.sensor_info = SimpleNamespace(
            type=11, width=640, height=480, fps=240, csi_num=1, mipi_lanes=3,
            phy_freq=1, data_type=0x2B,
        )
        self.bus = SimpleNamespace(readfrom_mem=Mock(return_value=b"\x01"))
        self.modules["machine"] = SimpleNamespace(I2C=Mock(return_value=self.bus))
        self.tpg = Mock(return_value=0)
        self.modules["mpp"] = ModuleType("mpp")
        self.modules["mpp.vicap"] = SimpleNamespace(
            VICAP_MIPI_4LANE=3, kd_mpi_vicap_tpg_enable=self.tpg,
        )

    def test_tpg_success_does_not_hide_camera_failure(self):
        self.configure_diagnostics()
        self.sensor.snapshot.side_effect = [RuntimeError("snapshot timeout"), self.frame]
        with self.assertRaisesRegex(RuntimeError, "snapshot timeout"):
            self.run_main()
        self.assertEqual(self.tpg.call_args_list, [call(True), call(False)])
        self.assertIn("lanes_enum=3 expected_4lane_enum=3", self.output)
        self.assertIn("sensor registers:", self.output)
        self.assertIn("ISP internal TPG produced 640x480", self.output)
        self.assertIn("NOT a camera capture pass", self.output)
        self.display.show_image.assert_not_called()
        self.assertNotIn("[PASS] capture:", self.output)
        self.assert_released()
        self.assertTrue(all(record.kwargs == {"addrsize": 16}
                            for record in self.bus.readfrom_mem.call_args_list))

    def test_full_mode_tpg_check_expects_the_scaled_output(self):
        self.configure_diagnostics()
        self.configure_full_mode()
        self.sensor.snapshot.side_effect = [RuntimeError("snapshot timeout"), self.frame]
        with patch.object(self.script, "report_sensor_mode"):
            with self.assertRaisesRegex(RuntimeError, "snapshot timeout"):
                self.run_main()
        self.assertEqual(self.tpg.call_args_list, [call(True), call(False)])
        self.assertIn("ISP internal TPG produced 480x640", self.output)

    def test_diagnostics_decode_sc132gs_output_format(self):
        cases = (
            ({0x0100: 0x01, 0x3018: 0x72, 0x3031: 0x0a, 0x3037: 0x20,
              0x4b01: 0x11, 0x3222: 0x00},
             "streaming=1 lanes=4 raw_bits=10 phy_bits=10 interface=MIPI external_trigger=0"),
            ({0x0100: 0x00, 0x3018: 0x32, 0x3031: 0x0c, 0x3037: 0x40,
              0x4b01: 0x19, 0x3222: 0x02},
             "streaming=0 lanes=2 raw_bits=12 phy_bits=12 interface=LVDS external_trigger=1"),
            ({0x0100: 0x01, 0x3018: 0x12, 0x3031: 0x08, 0x3037: 0x00,
              0x4b01: 0x11, 0x3222: 0x00},
             "streaming=1 lanes=1 raw_bits=8 phy_bits=8 interface=MIPI external_trigger=0"),
        )
        for registers, expected in cases:
            with self.subTest(expected=expected):
                self.configure_diagnostics()
                self.bus.readfrom_mem.side_effect = (
                    lambda address, register, length, **kwargs: bytes((registers.get(register, 0),))
                )
                self.sensor.snapshot.side_effect = [RuntimeError("snapshot timeout"), self.frame]
                with self.assertRaisesRegex(RuntimeError, "snapshot timeout"):
                    self.run_main()
                self.assertIn("SC132GS decoded: " + expected, self.output)
                self.assertIn("MCLK comes from external Y1", self.output)
                read_registers = {record.args[1] for record in self.bus.readfrom_mem.call_args_list}
                self.assertTrue({0x303f, 0x3652, 0x3653, 0x3654, 0x4b01} <= read_registers)
                self.assertNotIn("[PASS] capture:", self.output)

    def test_capture_diagnostics_read_configured_address_without_chip_id(self):
        self.configure_diagnostics()
        self.script.CHECK_CHIP_ID = False
        self.script.I2C_ADDRESS = 0x31
        self.board.camera_probe.side_effect = AssertionError("must not probe chip ID")
        self.sensor.snapshot.side_effect = [RuntimeError("camera timeout"), self.frame]
        with self.assertRaisesRegex(RuntimeError, "camera timeout"):
            self.run_main()
        self.board.camera_probe.assert_not_called()
        read_registers = {record.args[1] for record in self.bus.readfrom_mem.call_args_list}
        self.assertTrue(read_registers)
        self.assertTrue({0x3107, 0x3108}.isdisjoint(read_registers))
        self.assertTrue(all(record.args[0] == 0x31 for record in self.bus.readfrom_mem.call_args_list))
        self.assertEqual(self.tpg.call_args_list, [call(True), call(False)])
        self.assert_released()

    def test_failed_tpg_enable_is_still_disabled(self):
        self.configure_diagnostics()
        self.tpg.side_effect = [-1, 0]
        self.sensor.snapshot.side_effect = RuntimeError("snapshot timeout")
        with self.assertRaisesRegex(RuntimeError, "snapshot timeout"):
            self.run_main()
        self.tpg.assert_called_with(False)
        self.assertEqual(self.tpg.call_count, 2)
        self.assertIn("TPG enable returned -1", self.output)
        self.assert_released()

    def test_failed_tpg_capture_is_disabled_without_replacing_original_error(self):
        self.configure_diagnostics()
        self.sensor.snapshot.side_effect = [RuntimeError("camera timeout"), RuntimeError("TPG timeout")]
        with self.assertRaisesRegex(RuntimeError, "camera timeout"):
            self.run_main()
        self.tpg.assert_called_with(False)
        self.assertIn("internal TPG check failed: TPG timeout", self.output)
        self.assert_released()

    def test_failed_tpg_disable_is_reported_without_replacing_original_error(self):
        self.configure_diagnostics()
        self.tpg.side_effect = [0, -1]
        self.sensor.snapshot.side_effect = [RuntimeError("camera timeout"), self.frame]
        with self.assertRaisesRegex(RuntimeError, "camera timeout"):
            self.run_main()
        self.assertIn("restart the board before retesting", self.output)
        self.assert_released()

    def test_missing_tpg_api_does_not_replace_camera_error(self):
        self.configure_diagnostics()
        self.modules["mpp.vicap"] = ModuleType("mpp.vicap")
        self.sensor.snapshot.side_effect = RuntimeError("camera timeout")
        with self.assertRaisesRegex(RuntimeError, "camera timeout"):
            self.run_main()
        self.assertIn("internal TPG unavailable", self.output)
        self.assert_released()

    def test_register_readback_failure_does_not_skip_tpg_or_hide_camera_error(self):
        self.configure_diagnostics()
        self.bus.readfrom_mem.side_effect = OSError("read failed")
        self.sensor.snapshot.side_effect = [RuntimeError("camera timeout"), self.frame]
        with self.assertRaisesRegex(RuntimeError, "camera timeout"):
            self.run_main()
        self.assertIn("sensor readback unavailable:", self.output)
        self.assertIn("I2C1 address=0x30 register=0x0100: read failed", self.output)
        self.assertIn("ISP internal TPG produced 640x480", self.output)
        self.assert_released()

    def test_cleanup_error_still_releases_display_and_prevents_success(self):
        self.sensor.stop.side_effect = RuntimeError("stop failed")
        with self.assertRaisesRegex(RuntimeError, "stop failed"):
            self.run_main()
        self.assert_released()
        self.assertNotIn("[PASS] capture:", self.output)

    def test_cleanup_error_does_not_replace_capture_error(self):
        self.sensor.snapshot.side_effect = RuntimeError("snapshot timeout")
        self.sensor.stop.side_effect = RuntimeError("stop failed")
        with self.assertRaisesRegex(RuntimeError, "snapshot timeout"):
            self.run_main()
        self.assert_released()
        self.assertIn("[FAIL] cleanup sensor: stop failed", self.output)

    @staticmethod
    def color_histogram(mean=(50, 10, -20), stdev=(2, 3, 4),
                        low=(40, 0, -30), high=(60, 20, -10)):
        stats = SimpleNamespace()
        lower = SimpleNamespace()
        upper = SimpleNamespace()
        for i, channel in enumerate(("l", "a", "b")):
            setattr(stats, channel + "_mean", Mock(return_value=mean[i]))
            setattr(stats, channel + "_stdev", Mock(return_value=stdev[i]))
            setattr(lower, channel + "_value", Mock(return_value=low[i]))
            setattr(upper, channel + "_value", Mock(return_value=high[i]))
        return SimpleNamespace(get_statistics=Mock(return_value=stats),
                               get_percentile=Mock(side_effect=lambda p: lower if p < 0.5 else upper))

    def configure_color_sampling(self):
        self.script.COLOR_SAMPLING_ENABLED = True
        self.script.COLOR_SAMPLE_FRAMES = 3
        self.script.COLOR_RESULT_PATH = ""
        self.frame.get_histogram = Mock(return_value=self.color_histogram())
        self.frame.draw_rectangle = Mock()
        self.frame.draw_cross = Mock()
        self.frame.draw_string_advanced = Mock()
        self.touch = SimpleNamespace(read=Mock(return_value=()), deinit=Mock())
        factory = Mock(return_value=self.touch)
        factory.DEV_IDE = 3
        factory.EVENT_DOWN, factory.EVENT_UP = 1, 2
        self.modules["machine"] = SimpleNamespace(TOUCH=factory)
        return factory

    @staticmethod
    def click(x, y):
        return (SimpleNamespace(event=1, x=x, y=y), SimpleNamespace(event=2, x=x, y=y))

    def test_color_sampling_enabled_by_default(self):
        self.assertTrue(self.default_color_sampling_enabled)

    def test_preview_clicks_collect_unannotated_color_and_save_result(self):
        factory = self.configure_color_sampling()
        self.script.COLOR_RESULT_PATH = "/sdcard/sample.json"
        self.touch.read.side_effect = [self.click(10, 20), self.click(19, 29), (), ()]
        operations = []
        self.frame.get_histogram.side_effect = lambda **kw: (
            operations.append("sample") or self.color_histogram())
        self.frame.draw_rectangle.side_effect = lambda *a, **kw: operations.append("draw")
        result_file = mock_open()
        with patch.object(self.script, "open", result_file, create=True):
            self.run_main()
        factory.assert_called_once_with(3, range_x=640, range_y=480)
        self.assertEqual(self.frame.get_histogram.call_args_list,
                         [call(roi=(10, 20, 10, 10))] * 3)
        self.assertEqual(operations, ["sample", "draw"] * 3)
        result_file.assert_called_once_with("/sdcard/sample.json", "w")
        saved = json.loads("".join(c.args[0] for c in result_file().write.call_args_list))
        self.assertEqual(saved["roi"], [10, 20, 10, 10])
        self.assertEqual(saved["sample_frames"], 3)
        self.assertEqual(saved["sample_pixels"], 300)
        self.assertEqual((saved["first_frame"], saved["last_frame"]), (2, 4))
        self.assertEqual(saved["lab_mean"], [50, 10, -20])
        self.assertEqual(saved["lab_threshold"], [37, 63, -5, 25, -35, -5])
        self.assertEqual(saved["requested_analog_gain"], self.script.ANALOG_GAIN)
        self.assertEqual(saved["white_balance"], "ISP AWB")
        self.assertEqual(saved["frame_size"], [640, 480])
        self.assertEqual(saved["sensor_size"], [640, 480])
        self.assertIn("COLOR_THRESHOLD = (37, 63, -5, 25, -35, -5)", self.output)
        self.assertIn("[COLOR] saved:", self.output)
        self.assertIn("[PASS] capture:", self.output)
        self.frame.save.assert_not_called()
        self.touch.deinit.assert_called_once_with()
        self.assert_released()

    def test_multiframe_color_statistics_include_temporal_variation(self):
        self.configure_color_sampling()
        self.script.COLOR_SAMPLE_FRAMES = 2
        sampler = self.script.ColorSampler()
        sampler.select((10, 20, 8, 6))
        self.frame.get_histogram.side_effect = [
            self.color_histogram((20, -10, 5), (2, 3, 4), (10, -20, -10), (30, 0, 12)),
            self.color_histogram((40, 10, -5), (4, 3, 2), (35, 5, -8), (45, 20, 8)),
        ]
        with contextlib.redirect_stdout(io.StringIO()):
            sampler.process(self.frame, 11)
            self.assertIsNone(sampler.result)
            sampler.process(self.frame, 12)
        self.assertEqual(sampler.result["lab_mean"], [30, 0, 0])
        for actual, variance in zip(sampler.result["lab_stdev"], (110, 109, 35)):
            self.assertAlmostEqual(actual, math.sqrt(variance))
        self.assertEqual(sampler.result["lab_threshold"], [7, 48, -25, 25, -15, 17])
        self.assertEqual(sampler.result["sample_pixels"], 96)
        self.assertEqual(sampler.result["percentile_envelope_low"], [10, -20, -10])

    def test_color_thresholds_are_clamped_to_lab_ranges(self):
        self.configure_color_sampling()
        self.script.COLOR_SAMPLE_FRAMES = 1
        self.script.COLOR_ROI = (0, 0, 640, 480)
        self.frame.get_histogram.return_value = self.color_histogram(
            low=(0, -128, -128), high=(100, 127, 127))
        self.run_main()
        self.assertIn("COLOR_THRESHOLD = (0, 100, -128, 127, -128, 127)", self.output)
        self.frame.get_histogram.assert_called_once_with(roi=(0, 0, 640, 480))

    def test_color_reselection_discards_old_samples_and_handles_reversed_corners(self):
        self.configure_color_sampling()
        sampler = self.script.ColorSampler()
        with contextlib.redirect_stdout(io.StringIO()):
            sampler.select((0, 0, 10, 10))
            sampler.process(self.frame, 1)
            sampler.select_point(30, 40)
            sampler.process(self.frame, 2)
            self.assertIsNone(sampler.roi)
            self.assertEqual(sampler.samples, 0)
            sampler.select_point(21, 31)
            sampler.process(self.frame, 3)
        self.assertEqual(sampler.roi, (21, 31, 10, 10))
        self.assertEqual(sampler.samples, 1)
        self.assertEqual(sampler.sums, [50, 10, -20])
        self.assertEqual(self.frame.get_histogram.call_count, 2)

    def test_color_selection_clamps_coordinates_and_rejects_tiny_regions(self):
        sampler = self.script.ColorSampler()
        with contextlib.redirect_stdout(io.StringIO()):
            sampler.select_point(-5, -10)
            sampler.select_point(1, 1)
            self.assertIsNone(sampler.roi)
            self.assertEqual(sampler.corner, (0, 0))
            sampler.select_point(700, 500)
        self.assertEqual(sampler.roi, (0, 0, 640, 480))

    def test_full_preview_color_coordinates_and_metadata(self):
        factory = self.configure_color_sampling()
        self.script.USE_VGA_MODE = False
        self.script.COLOR_SAMPLE_FRAMES = 1
        self.script.COLOR_ROI = (476, 636, 4, 4)
        self.script.validate_config()
        sampler = self.script.ColorSampler()
        with patch.dict(sys.modules, self.modules), contextlib.redirect_stdout(io.StringIO()):
            sampler.start()
            sampler.process(self.frame, 1)
            self.assertEqual(sampler.result["frame_size"], (480, 640))
            self.assertEqual(sampler.result["sensor_size"], (1080, 1280))
            self.assertEqual(sampler.result["roi"], (476, 636, 4, 4))
            sampler.select_point(-1, -1)
            sampler.select_point(2000, 2000)
        factory.assert_called_once_with(3, range_x=480, range_y=640)
        self.assertEqual(sampler.roi, (0, 0, 480, 640))
        for roi in ((477, 0, 4, 4), (0, 637, 4, 4)):
            with self.subTest(roi=roi), patch.object(self.script, "COLOR_ROI", roi):
                with self.assertRaisesRegex(ValueError, "480x640"):
                    self.script.validate_config()

    def test_output_monitor_uses_center_of_full_frame(self):
        self.frame.width = lambda: 1080
        self.frame.height = lambda: 1280
        self.frame.get_statistics = Mock(return_value=SimpleNamespace(
            l_mean=lambda: 50, a_mean=lambda: 0, b_mean=lambda: 0))
        self.script.OutputMonitor().process(self.frame, 1)
        self.frame.get_statistics.assert_called_once_with(roi=(270, 320, 540, 640))

    def test_headless_fixed_roi_collects_without_touch_or_drawing(self):
        self.configure_color_sampling()
        self.script.PREVIEW_ENABLED = False
        self.script.COLOR_ROI = (100, 100, 20, 30)
        self.modules["machine"] = None
        self.run_main()
        self.assertEqual(self.frame.get_histogram.call_count, 3)
        self.frame.draw_rectangle.assert_not_called()
        self.assertIn("[COLOR] complete:", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_color_sampling_without_selection_never_reads_or_saves_pixels(self):
        self.configure_color_sampling()
        self.script.COLOR_RESULT_PATH = "/sdcard/sample.json"
        with patch.object(self.script, "open", mock_open(), create=True) as result_file:
            self.run_main()
        self.frame.get_histogram.assert_not_called()
        result_file.assert_not_called()
        self.assertNotIn("COLOR_THRESHOLD =", self.output)
        self.touch.deinit.assert_called_once_with()

    def test_color_partial_sample_is_reported_without_saving(self):
        self.configure_color_sampling()
        self.script.COLOR_SAMPLE_FRAMES = 5
        self.script.COLOR_ROI = (0, 0, 10, 10)
        self.script.COLOR_RESULT_PATH = "/sdcard/sample.json"
        with patch.object(self.script, "open", mock_open(), create=True) as result_file:
            self.run_main()
        result_file.assert_not_called()
        self.assertIn("Incomplete sample: 4/5 frames", self.output)
        self.assertNotIn("COLOR_THRESHOLD =", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_color_save_failure_retains_console_result_and_preview(self):
        self.configure_color_sampling()
        self.script.COLOR_ROI = (0, 0, 10, 10)
        self.script.COLOR_RESULT_PATH = "/sdcard/sample.json"
        with patch.object(self.script, "open", side_effect=OSError("no SD card"), create=True):
            self.run_main()
        self.assertIn("Save failed: no SD card", self.output)
        self.assertIn('"lab_threshold":', self.output)
        self.assertNotIn("[COLOR] saved:", self.output)
        self.assertEqual(self.display.show_image.call_count, 4)
        self.assertIn("[PASS] capture:", self.output)

    def test_missing_ide_touch_keeps_preview_and_fixed_roi_available(self):
        self.configure_color_sampling()
        self.modules["machine"] = None
        self.script.COLOR_ROI = (0, 0, 10, 10)
        self.run_main()
        self.assertIn("IDE touch unavailable", self.output)
        self.assertIn("[COLOR] complete:", self.output)
        self.assertIn("[PASS] capture:", self.output)
        self.assert_released()

    def test_touch_constructor_failure_does_not_break_preview(self):
        factory = self.configure_color_sampling()
        factory.side_effect = OSError("touch unavailable")
        self.run_main()
        self.assertIn("IDE touch unavailable: touch unavailable", self.output)
        self.assertIn("[PASS] capture:", self.output)
        self.frame.get_histogram.assert_not_called()
        self.assert_released()

    def test_color_touch_read_failure_releases_every_resource(self):
        self.configure_color_sampling()
        self.touch.read.side_effect = RuntimeError("touch read failed")
        with self.assertRaisesRegex(RuntimeError, "touch read failed"):
            self.run_main()
        self.touch.deinit.assert_called_once_with()
        self.assert_released()
        self.assertNotIn("[PASS] capture:", self.output)

    def test_color_touch_cleanup_failure_still_releases_camera_and_display(self):
        self.configure_color_sampling()
        self.touch.deinit.side_effect = RuntimeError("touch stop failed")
        with self.assertRaisesRegex(RuntimeError, "touch stop failed"):
            self.run_main()
        self.assert_released()
        self.assertNotIn("[PASS] capture:", self.output)

    def test_invalid_configuration_is_rejected_without_hardware_access(self):
        for name, value in (("PROBE_ONLY", 1), ("CHECK_CHIP_ID", "yes"),
                            ("I2C_ADDRESS", True), ("I2C_ADDRESS", 0x34),
                            ("USE_VGA_MODE", "yes"),
                            ("PREVIEW_ENABLED", None), ("TEST_SECONDS", -1),
                            ("DIAGNOSTICS_ENABLED", "yes"),
                            ("FRAME_TIMEOUT_MS", 0), ("EXPOSURE_US", True),
                            ("ANALOG_GAIN", float("nan")), ("ANALOG_GAIN", 29),
                            ("COLOR_SAMPLING_ENABLED", 1), ("COLOR_SAMPLE_FRAMES", 0),
                            ("COLOR_SAMPLE_FRAMES", True), ("COLOR_ROI", (0, 0, 3, 10)),
                            ("COLOR_ROI", (-1, 0, 10, 10)), ("COLOR_ROI", (639, 0, 4, 4)),
                            ("COLOR_ROI", (0, 479, 4, 4)), ("COLOR_ROI", "bad"),
                            ("COLOR_ROI", (True, 0, 10, 10)), ("COLOR_ROI", (0, 0, 10)),
                            ("COLOR_PERCENTILES", (0.8, 0.2)),
                            ("COLOR_PERCENTILES", (0.1, float("nan"))),
                            ("COLOR_PERCENTILES", (0.1, 1.1)),
                            ("COLOR_THRESHOLD_MARGIN", (1, -1, 1)),
                            ("COLOR_THRESHOLD_MARGIN", (True, 1, 1)),
                            ("COLOR_THRESHOLD_MARGIN", (1, 1)), ("COLOR_RESULT_PATH", None)):
            with self.subTest(name=name, value=value), patch.object(self.script, name, value):
                with self.assertRaises(ValueError):
                    self.run_main()
        self.board.camera_probe.assert_not_called()
        self.board.camera_sensor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
