"""Host regressions for manual SC132GS search bounds and preview input routing."""

import contextlib
import io
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_sc132gs_test as capture_checks


class Blob:
    def __init__(self, bounds, pixels=12, lab=(76, -63, 27), roundness=1.0):
        self.bounds, self.count, self.lab, self.ratio = bounds, pixels, lab, roundness

    def rect(self):
        return self.bounds

    def pixels(self):
        return self.count

    def cxf(self):
        return self.bounds[0] + (self.bounds[2] - 1) * 0.5

    def cyf(self):
        return self.bounds[1] + (self.bounds[3] - 1) * 0.5

    def min_corners(self):
        x, y, w, h = self.bounds
        return ((x, y), (x + w - 1, y), (x + w - 1, y + h - 1), (x, y + h - 1))

    def rotation_rad(self):
        return 0.0

    def roundness(self):
        return self.ratio


class BlobFrame:
    """Controlled native-extraction results; stride is checked at the API boundary."""

    def __init__(self, blobs=()):
        self.blobs = blobs
        self.scans = []

    def width(self):
        return 640

    def height(self):
        return 480

    def format(self):
        return 5

    def find_blobs(self, thresholds, **kwargs):
        self.scans.append((thresholds, kwargs))
        x, y, w, h = kwargs["roi"]
        found = []
        for blob in self.blobs:
            bx, by, bw, bh = blob.bounds
            matches = any(all(t[2 * i] <= blob.lab[i] <= t[2 * i + 1] for i in range(3))
                          for t in thresholds)
            if (x <= bx and y <= by and bx + bw <= x + w and by + bh <= y + h
                    and matches and blob.count >= kwargs["pixels_threshold"]
                    and bw * bh >= kwargs["area_threshold"]):
                found.append(blob)
        return found

    def get_statistics(self, roi, thresholds):
        blob = next(blob for blob in self.blobs if blob.bounds == roi)
        return capture_checks.Sc132gsTest.color_histogram(mean=blob.lab).get_statistics()


class Sc132gsFixture(unittest.TestCase):
    snapshot = capture_checks.Sc132gsTest.snapshot
    run_main = capture_checks.Sc132gsTest.run_main
    assert_released = capture_checks.Sc132gsTest.assert_released
    configure_color_sampling = capture_checks.Sc132gsTest.configure_color_sampling
    color_histogram = staticmethod(capture_checks.Sc132gsTest.color_histogram)
    click = staticmethod(capture_checks.Sc132gsTest.click)

    def setUp(self):
        capture_checks.Sc132gsTest.setUp(self)
        self.script.time.ticks_us = lambda: self.clock * 1000
        modules = patch.dict(sys.modules, self.modules)
        modules.start()
        self.addCleanup(modules.stop)
        quiet = contextlib.redirect_stdout(io.StringIO())
        quiet.__enter__()
        self.addCleanup(quiet.__exit__, None, None, None)
        self.detector = self.script.ColorTargetDetector(640, 480)

    def process(self, frame, index):
        self.clock = index * 30
        return self.detector.process(frame, index, self.clock)

    def lock_lamp(self, frame):
        for index in range(1, 4):
            result = self.process(frame, index)
        self.assertEqual(result["kind"], "light")
        return result

    def assert_scans_inside(self, frame, bounds):
        x, y, w, h = bounds
        self.assertTrue(frame.scans)
        for _, scan in frame.scans:
            bx, by, bw, bh = scan["roi"]
            self.assertTrue(x <= bx <= bx + bw <= x + w, scan)
            self.assertTrue(y <= by <= by + bh <= y + h, scan)
            self.assertEqual((scan["x_stride"], scan["y_stride"]), (1, 1))


class RoiChecks(Sc132gsFixture):
    def test_manual_roi_selects_local_lamp_and_preserves_confirmation(self):
        roi = (40, 40, 40, 80)
        frame = BlobFrame((Blob((55, 90, 4, 4)), Blob((300, 200, 20, 20), pixels=300)))
        self.detector.set_search_roi(roi)
        first = self.process(frame, 1)
        self.assertIsNone(first["target"])
        self.assertEqual(first["reason"], "lamp_confirm")
        self.assertIsNone(self.process(frame, 2)["target"])
        result = self.process(frame, 3)
        self.assertEqual(result["light"]["bounds"], (55, 90, 4, 4))
        self.assertEqual(result["manual_roi"], roi)
        self.assertEqual(result["search_step"], 1)
        self.assert_scans_inside(frame, roi)

    def test_tracking_loss_retries_selected_roi_and_never_escapes_it(self):
        roi = (30, 40, 80, 120)
        self.detector.set_search_roi(roi)
        self.lock_lamp(BlobFrame((Blob((60, 100, 4, 4)),)))
        frame = BlobFrame((Blob((300, 200, 20, 20), pixels=300),))
        result = self.process(frame, 4)
        self.assertTrue(result["rescanned"])
        self.assertEqual(result["roi"], roi)
        self.assertIsNone(result["target"])
        self.assert_scans_inside(frame, roi)
        for index in range(5, 9):
            result = self.process(frame, index)
        self.assertEqual(result["manual_roi"], roi)
        self.assertEqual(self.detector.next_window(None), roi)
        self.assertIsNone(self.detector.light_motion.last)

    def test_select_and_clear_reset_old_target_and_debug_cache(self):
        self.lock_lamp(BlobFrame((Blob((60, 100, 4, 4)),)))
        self.detector.debug_snapshot = [{"bounds": (60, 100, 4, 4)}]
        self.detector.set_search_roi((200, 200, 40, 60))
        self.assertIsNone(self.detector.light_motion.last)
        self.assertIsNone(self.detector.pending_light)
        self.assertIsNone(self.detector.result)
        self.assertEqual(self.detector.debug_snapshot, [])
        self.detector.set_search_roi(None)
        self.assertEqual(self.detector.next_window(None), (0, 0, 640, 480))
        self.assertEqual(self.detector.search_step((0, 0, 640, 480)), 2)

    def test_full_frame_manual_selection_still_scans_every_pixel(self):
        self.detector.set_search_roi((0, 0, 640, 480))
        result = self.process(BlobFrame(), 1)
        self.assertTrue(result["full_frame"])
        self.assertEqual(result["search_step"], 1)

    def test_empty_roi_never_produces_a_target(self):
        self.detector.set_search_roi((10, 20, 30, 40))
        for index in range(1, 5):
            result = self.process(BlobFrame(), index)
            self.assertIsNone(result["target"])
            self.assertEqual(result["reason"], "no_lamp_color")

    def test_one_pixel_seed_is_visible_but_cannot_become_a_target(self):
        self.detector.set_search_roi((10, 20, 30, 40))
        frame = BlobFrame((Blob((15, 25, 1, 1), pixels=1, roundness=0.0),))
        result = self.process(frame, 1)
        self.assertIsNone(result["target"])
        self.assertEqual(len(self.detector.debug_candidates), 1)
        self.assertEqual(self.detector.debug_candidates[0]["reason"], "small")
        self.assertIn("太小", self.detector.debug_label(self.detector.debug_candidates[0]))
        self.assertEqual(frame.scans[0][1]["pixels_threshold"], 1)
        self.assertEqual(frame.scans[1][1]["pixels_threshold"], 3)

    def test_dim_roi_diagnostics_do_not_widen_real_color_thresholds(self):
        self.detector.set_search_roi((10, 20, 40, 80))
        self.script.PREVIEW_ENABLED = True
        frame = BlobFrame((Blob((20, 40, 4, 4), lab=(30, -63, 27)),))
        original = dict(self.detector.thresholds)
        self.process(frame, 1)
        self.clock = 600
        result = self.detector.process(frame, 2, self.clock)
        self.assertIsNone(result["target"])
        self.assertEqual(self.detector.thresholds, original)
        self.assertEqual(len(self.detector.debug_candidates), 1)
        self.assertEqual(self.detector.debug_candidates[0]["reason"], "color")
        self.assertEqual(self.detector.debug_candidates[0]["lab_mean"][0], 30)
        self.assert_scans_inside(frame, self.detector.manual_roi)

    def test_armor_window_is_intersected_with_manual_roi(self):
        self.detector.set_search_roi((50, 40, 40, 40))
        light = {"bounds": (50, 52, 8, 8), "radius": 4.0}
        self.assertEqual(self.detector.armor_window(light), (50, 40, 20, 12))
        self.assertEqual(self.detector.armor_window(light, False), (38.0, 28.0, 32.0, 24.0))

    def test_white_highlight_still_explains_missing_green_inside_roi(self):
        self.detector.set_search_roi((10, 20, 40, 80))
        frame = BlobFrame((Blob((20, 40, 4, 4), lab=(96, -3, 0)),))
        self.process(frame, 1)
        self.clock = 600
        result = self.detector.process(frame, 2, self.clock)
        self.assertIsNone(result["target"])
        self.assertEqual(len(self.detector.debug_candidates), 1)
        self.assertEqual(self.detector.debug_candidates[0]["reason"], "color")

    def test_nonoverlapping_windows_clip_to_the_manual_roi_edge(self):
        self.detector.set_search_roi((10, 20, 40, 80))
        self.assertEqual(self.detector.search_clip((200, 20, 10, 10)), (50, 20, 0, 10))
        self.assertEqual(self.detector.search_clip((10, 200, 10, 10)), (10, 100, 10, 0))

    def test_gap_reset_and_new_color_sample_preserve_manual_bounds(self):
        roi = (40, 40, 60, 120)
        self.detector.set_search_roi(roi)
        frame = BlobFrame((Blob((60, 100, 4, 4)),))
        self.lock_lamp(frame)
        self.clock = 1000
        result = self.detector.process(frame, 4, self.clock)
        self.assertTrue(result["gap_reset"])
        self.assertIsNone(result["target"])
        self.assertEqual(result["manual_roi"], roi)
        self.script.COLOR_SAMPLE_APPLY_TO_DETECTION = True
        self.detector.apply_sample({"color_label": "green", "lab_threshold": (50, 100, -90, -40, 0, 60)})
        self.assertEqual(self.detector.search_bounds(), roi)
        self.assertIsNone(self.detector.pending_light)

    def test_candidate_overflow_remains_an_error_inside_roi(self):
        self.detector.set_search_roi((10, 20, 100, 100))
        self.detector.config["workspace.max_candidates"] = 2
        frame = BlobFrame(tuple(Blob((20 + 10 * i, 40, 4, 4)) for i in range(3)))
        with self.assertRaisesRegex(self.script.DetectionError, "capacity exceeded"):
            self.process(frame, 1)
        self.assertIsNone(self.detector.frame)
        self.assertEqual(self.detector.search_bounds(), (10, 20, 100, 100))

    def test_invalid_fixed_rois_fail_before_camera_access(self):
        for roi in ((0, 0, 3, 4), (-1, 0, 10, 10), (639, 0, 4, 4),
                    (0, 479, 4, 4), (True, 0, 4, 4), (0, 0, 4), "bad"):
            with self.subTest(roi=roi), patch.object(self.script, "DETECTION_ROI", roi):
                with self.assertRaises(ValueError):
                    self.run_main()
        self.board.camera_sensor.assert_not_called()

    def test_fixed_roi_can_run_headless(self):
        self.script.DETECTION_ENABLED = True
        self.script.PREVIEW_ENABLED = False
        self.script.DETECTION_ROI = (40, 40, 40, 100)
        self.frame = BlobFrame((Blob((60, 100, 4, 4)),))
        # Keep frame gaps below the existing tracking reset threshold.
        self.sensor.snapshot.side_effect = lambda **kw: setattr(self, "clock", self.clock + 100) or self.frame
        self.run_main()
        self.assertIn("manual_ROI=(40, 40, 40, 100)", self.output)
        self.assertIn("target=light", self.output)
        self.assertIn("[PASS] capture:", self.output)
        self.preview_factory.assert_not_called()


class PreviewInputChecks(Sc132gsFixture):
    def setUp(self):
        super().setUp()
        self.factory = self.configure_color_sampling()
        sys.modules.update(self.modules)
        self.poller = Mock()
        self.poller.poll.return_value = ()
        self.select = SimpleNamespace(POLLIN=1, poll=Mock(return_value=self.poller))
        sys.modules["select"] = self.select
        self.sampler = self.script.ColorSampler()
        self.controls = self.script.PreviewControls(self.detector, self.sampler)

    def test_default_clicks_select_detection_without_sampling_color(self):
        self.controls.start()
        self.touch.read.side_effect = (self.click(70, 90), self.click(40, 40))
        self.controls.poll()
        self.controls.poll()
        self.assertEqual(self.detector.manual_roi, (40, 40, 31, 51))
        self.assertIsNone(self.sampler.roi)
        self.frame.get_histogram.assert_not_called()
        self.factory.assert_called_once_with(3, range_x=640, range_y=480)

    def test_terminal_c_routes_clicks_only_to_color_sampler(self):
        self.controls.start()
        self.controls.stdin = io.StringIO("C\n")
        self.poller.poll.side_effect = ([(self.controls.stdin, 1)], [(self.controls.stdin, 1)], ())
        self.touch.read.return_value = self.click(10, 20) + self.click(19, 29)
        self.controls.poll()
        self.assertEqual(self.controls.mode, "color")
        self.assertEqual(self.sampler.roi, (10, 20, 10, 10))
        self.assertIsNone(self.detector.manual_roi)
        self.assertTrue(all(c.args == (0,) for c in self.poller.poll.call_args_list))

    def test_switching_modes_cancels_half_selection(self):
        self.controls.select_point(50, 60)
        self.controls.command("c")
        self.assertIsNone(self.controls.corner)
        self.controls.select_point(10, 20)
        self.controls.command("r")
        self.assertIsNone(self.sampler.corner)
        self.controls.select_point(80, 90)
        self.assertEqual(self.controls.corner, (80, 90))
        self.assertIsNone(self.detector.manual_roi)

    def test_preview_buttons_change_mode_and_clear_roi(self):
        self.detector.set_search_roi((10, 20, 40, 60))
        _, (x, y, _, _), _ = self.controls.buttons()[1]
        self.controls.select_point(x + 1, y + 1)
        self.assertEqual(self.controls.mode, "color")
        self.assertIsNone(self.sampler.corner)
        _, (x, y, _, _), _ = self.controls.buttons()[2]
        self.controls.select_point(x + 1, y + 1)
        self.assertIsNone(self.detector.manual_roi)
        self.assertEqual(self.controls.mode, "roi")

    def test_cancel_preserves_existing_roi_and_clamps_image_corners(self):
        roi = (10, 20, 40, 60)
        self.detector.set_search_roi(roi)
        self.controls.select_point(50, 60)
        self.controls.command("x")
        self.assertIsNone(self.controls.corner)
        self.assertEqual(self.detector.manual_roi, roi)
        self.controls.select_point(-10, -20)
        self.controls.select_point(1, 1)
        self.assertEqual(self.detector.manual_roi, roi)
        self.assertEqual(self.controls.corner, (0, 0))
        self.controls.select_point(2000, 2000)
        self.assertEqual(self.detector.manual_roi, (0, 0, 640, 480))

    def test_keyboard_work_is_bounded_per_frame(self):
        self.controls.start()
        self.controls.stdin = io.StringIO("r" * 100)
        self.poller.poll.return_value = [(self.controls.stdin, 1)]
        self.controls.poll()
        self.assertEqual(self.controls.stdin.tell(), 16)
        self.assertEqual(self.poller.poll.call_count, 16)

    def test_terminal_hangup_keeps_touch_buttons_working(self):
        self.select.POLLHUP = 16
        self.controls.start()
        self.poller.poll.return_value = [(self.controls.stdin, 16)]
        self.touch.read.return_value = self.click(40, 40) + self.click(70, 90)
        self.controls.poll()
        self.assertIsNone(self.controls.poller)
        self.assertEqual(self.detector.manual_roi, (40, 40, 31, 51))

    def test_ide_interrupt_during_input_setup_is_not_ignored(self):
        self.factory.side_effect = RuntimeError("IDE interrupt")
        with self.assertRaisesRegex(RuntimeError, "IDE interrupt"):
            self.controls.start()

    def test_color_disabled_retains_roi_mode(self):
        controls = self.script.PreviewControls(self.detector)
        controls.command("c")
        controls.select_point(10, 20)
        controls.select_point(40, 60)
        self.assertEqual(self.detector.manual_roi, (10, 20, 31, 41))

    def test_cleanup_releases_touch_even_if_terminal_unregister_fails(self):
        self.controls.start()
        self.poller.unregister.side_effect = OSError("terminal closed")
        with self.assertRaisesRegex(OSError, "terminal closed"):
            self.controls.deinit()
        self.touch.deinit.assert_called_once()

    def test_main_uses_one_touch_owner_and_samples_before_annotations(self):
        self.script.DETECTION_ENABLED = True
        detector = Mock(width=640, height=480)
        _, (x, y, _, _), _ = self.controls.buttons()[1]
        self.touch.read.side_effect = (self.click(x + 1, y + 1), self.click(10, 20),
                                       self.click(19, 29), ())
        order = []
        self.frame.get_histogram.side_effect = lambda **kw: order.append("sample") or self.color_histogram()
        self.frame.draw_rectangle.side_effect = lambda *a, **kw: order.append("draw")
        detector.process.side_effect = lambda *a, **kw: order.append("detect")
        with patch.object(self.script, "ColorTargetDetector", return_value=detector):
            self.run_main()
        self.factory.assert_called_once()
        detector.set_search_roi.assert_not_called()
        self.assertEqual(self.frame.get_histogram.call_count, 2)
        for index, action in enumerate(order):
            if action == "sample":
                self.assertEqual(order[index + 1:index + 3], ["detect", "draw"])
        self.touch.deinit.assert_called_once()
        self.assert_released()

    def test_touch_failure_cleans_up_detector_camera_and_display(self):
        self.script.DETECTION_ENABLED = True
        self.touch.read.side_effect = RuntimeError("touch read failed")
        detector = Mock(width=640, height=480)
        with patch.object(self.script, "ColorTargetDetector", return_value=detector):
            with self.assertRaisesRegex(RuntimeError, "touch read failed"):
                self.run_main()
        self.touch.deinit.assert_called_once()
        detector.close.assert_called_once()
        self.assert_released()


if __name__ == "__main__":
    unittest.main()
