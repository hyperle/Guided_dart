"""Bounded memory, H.264 recovery, storage and worker isolation checks; no board claims."""

import builtins
import contextlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

import test_sc132gs_test as capture_checks


SPS = b"\x00\x00\x00\x01\x67\x42\x00\x1e\x80"
PPS = b"\x00\x00\x01\x68\xce\x3c\x80"
IDR = b"\x00\x00\x01\x65\x88\x84"
PFRAME = b"\x00\x00\x01\x41\x9a\x80"


class VideoRecorderTest(unittest.TestCase):
    run_main = capture_checks.Sc132gsTest.run_main

    def setUp(self):
        capture_checks.Sc132gsTest.setUp(self)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.script.RECORDING_ENABLED = True
        self.script.RECORDING_DIR = str(Path(self.directory.name) / "videos")
        self.script.os.mkdir, self.script.os.stat = os.mkdir, os.stat
        self.script.os.ilistdir = self.ilistdir
        self.script.time.sleep_ms = Mock(side_effect=lambda ms: time.sleep(min(ms, 10) / 1000))
        self.modules["media.sensor"].Sensor.YUV420SP = 6
        self.queue = []
        self.addresses = {}
        self.live_addresses = []
        self.threads = []
        self.file_calls = []
        self.block_write = self.block_flush = False
        self.io_started, self.io_release = threading.Event(), threading.Event()
        self.io_release.set()
        self.script.open = self.open_file
        self.encoder = Mock(chn=2, STREAM_TYPE_HEADER=0, STREAM_TYPE_I=1, STREAM_TYPE_P=2)
        self.encoder.GetStream.side_effect = self.get_stream
        self.encoder.ReleaseStream.side_effect = self.release_stream
        self.encoder_factory = Mock(return_value=self.encoder, PAYLOAD_TYPE_H264=10,
                                    H264_PROFILE_MAIN=20)
        self.attr_factory = Mock()
        self.link = Mock()
        self.sensor.bind_info.return_value = {"src": (7, 0, 1)}
        media = ModuleType("media.media")
        media.MediaManager = SimpleNamespace(link=Mock(return_value=self.link))
        media.CAM_CHN_ID_1, media.VIDEO_ENCODE_MOD_ID, media.VENC_DEV_ID = 1, 8, 0
        encoder = ModuleType("media.vencoder")
        encoder.Encoder, encoder.ChnAttrStr = self.encoder_factory, self.attr_factory
        encoder.StreamData = SimpleNamespace
        uctypes = ModuleType("uctypes")
        uctypes.bytearray_at = lambda pointer, size: memoryview(self.addresses[pointer])[:size]
        self.modules.update({"media.media": media, "media.vencoder": encoder, "uctypes": uctypes,
                             "_thread": SimpleNamespace(start_new_thread=self.start_thread)})
        self.order = Mock()
        for name, resource in (("sensor", self.sensor), ("encoder", self.encoder),
                               ("link", self.link), ("display_close", self.display.deinit)):
            self.order.attach_mock(resource, name)
        self.addCleanup(self.finish_threads)

    def start_thread(self, function, args):
        thread = threading.Thread(target=function, args=args, daemon=True)
        self.threads.append(thread)
        thread.start()
        return thread.ident

    def finish_threads(self):
        self.io_release.set()
        for thread in self.threads:
            thread.join(2)
            self.assertFalse(thread.is_alive(), "Recording thread leaked from test")

    def ilistdir(self, directory):
        self.file_calls.append(("scan", threading.get_ident()))
        with os.scandir(directory) as entries:
            for entry in entries:
                yield (entry.name, 0x8000 if entry.is_file() else 0x4000, 0, entry.stat().st_size)

    def open_file(self, path, mode):
        self.file_calls.append(("open", threading.get_ident()))
        file = builtins.open(path, mode)
        fixture = self

        class File:
            def write(self, data):
                fixture.file_calls.append(("write", threading.get_ident()))
                if fixture.block_write:
                    fixture.io_started.set()
                    fixture.io_release.wait(2)
                return file.write(data)

            def flush(self):
                fixture.file_calls.append(("flush", threading.get_ident()))
                if fixture.block_flush:
                    fixture.io_started.set()
                    fixture.io_release.wait(2)
                file.flush()

            def close(self):
                fixture.file_calls.append(("close", threading.get_ident()))
                file.close()

        return File()

    def snapshot(self, **kwargs):
        time.sleep(0.005)  # Let the real test writer run between fake camera frames.
        frame = capture_checks.Sc132gsTest.snapshot(self, **kwargs)
        pts = 10000000 + self.clock * 1000
        if self.sensor.snapshot.call_count == 1:
            self.queue.append([(0, SPS, pts), (0, PPS, pts), (1, IDR, pts)])
        else:
            self.queue.append([(2, PFRAME, pts)])
        return frame

    def get_stream(self, stream, timeout):
        self.assertEqual(timeout, 0)
        if not self.queue:
            return 1
        packs = self.queue.pop(0)
        stream.pack_cnt = len(packs)
        stream.stream_type = [pack[0] for pack in packs]
        stream.data = []
        for _, data, _ in packs:
            buffer = bytearray(data)
            address = id(buffer)
            self.addresses[address] = buffer
            stream.data.append(address)
        stream.data_size = [len(pack[1]) for pack in packs]
        stream.pts = [pack[2] for pack in packs]
        self.live_addresses = list(stream.data)
        return 0

    def release_stream(self, stream):
        # Any queued data must remain valid after these native buffers disappear.
        for address in self.live_addresses:
            data = self.addresses.pop(address)
            data[:] = b"\xee" * len(data)
        self.live_addresses = []

    def assert_released(self):
        self.sensor.stop.assert_called_once_with()
        self.display.deinit.assert_called_once_with()
        self.encoder.Stop.assert_called_once()
        self.encoder.Destroy.assert_called_once()
        self.link.destroy.assert_called_once()
        self.assertFalse(self.addresses)

    def videos(self):
        return sorted(Path(self.script.RECORDING_DIR).glob("sc132gs_*.mp4"))

    def recorder(self):
        recorder = self.script.VideoRecorder()
        recorder.prepare(self.sensor)
        return recorder

    @staticmethod
    def load_slot(slot, pts=1000000, key=True, frame=None):
        header = SPS + PPS if key else b""
        frame = frame if frame is not None else IDR if key else PFRAME
        data = header + frame
        slot.view[:len(data)] = data
        slot.size, slot.header_size, slot.pts, slot.key = len(data), len(header), pts, key
        return slot

    def test_records_with_preview_and_preserves_channel_zero(self):
        self.run_main()
        self.assertEqual(self.sensor.set_framesize.call_args_list, [
            call(width=640, height=480, crop=True),
            call(width=640, height=480, chn=1, alignment=12, crop=True)])
        self.assertEqual(self.sensor.set_pixformat.call_args_list, [call(2), call(6, chn=1)])
        self.attr_factory.assert_called_once_with(10, 20, 640, 480, bit_rate=4000,
                                                  gopLen=30, src_frame_rate=240, dst_frame_rate=30)
        video = self.videos()[0].read_bytes()
        self.assertEqual(video.count(b"moof"), 4)
        self.assertIn(b"avcC", video)
        self.assertIn(b"\x65\x88\x84", video)
        self.assertNotIn(b"\xee\xee\xee", video)
        self.assertIn("video_frames=4", self.output)
        self.assert_released()
        events = [entry[0] for entry in self.order.mock_calls]
        self.assertLess(events.index("encoder.Start"), events.index("sensor.run"))
        self.assertLess(events.index("sensor.stop"), events.index("link.destroy"))
        self.assertLess(events.index("encoder.Destroy"), events.index("display_close"))

    def test_all_filesystem_operations_and_flush_run_on_writer_thread(self):
        self.run_main()
        self.assertEqual({name for name, _ in self.file_calls}, {"scan", "open", "write", "flush", "close"})
        self.assertTrue(all(ident != threading.get_ident() for _, ident in self.file_calls))
        self.assertEqual(len({ident for _, ident in self.file_calls}), 1)

    def test_recording_coexists_with_detection_and_sampling(self):
        self.script.DETECTION_ENABLED = self.script.COLOR_SAMPLING_ENABLED = True
        detector, sampler = Mock(), Mock()
        with patch.object(self.script, "ColorTargetDetector", return_value=detector), \
                patch.object(self.script, "ColorSampler", return_value=sampler), \
                patch.object(self.script, "PreviewControls", return_value=Mock()):
            self.run_main()
        self.assertEqual(detector.process.call_count, 4)
        self.assertEqual(sampler.process.call_count, 4)
        self.assertIn("video_frames=4", self.output)

    def test_records_without_preview(self):
        self.script.PREVIEW_ENABLED = False
        self.run_main()
        self.preview_factory.assert_not_called()
        self.assertEqual(self.videos()[0].read_bytes().count(b"moof"), 4)

    def test_native_preview_keeps_recording_at_aligned_480x640(self):
        capture_checks.Sc132gsTest.configure_full_mode(self, native=True)
        self.sensor.snapshot.side_effect = self.snapshot
        self.run_main()
        self.sensor.set_framesize.assert_has_calls([
            call(width=1080, height=1280), call(width=480, height=640, chn=1, alignment=12, crop=False)])
        self.attr_factory.assert_called_once_with(10, 20, 480, 640, bit_rate=4000,
                                                  gopLen=30, src_frame_rate=120, dst_frame_rate=30)

    def test_disabled_recording_creates_no_encoder_or_directory(self):
        self.script.RECORDING_ENABLED = False
        self.run_main()
        self.encoder_factory.assert_not_called()
        self.assertFalse(Path(self.script.RECORDING_DIR).exists())

    def test_existing_recording_is_not_overwritten(self):
        directory = Path(self.script.RECORDING_DIR)
        directory.mkdir()
        old = directory / "sc132gs_0001.mp4"
        old.write_bytes(b"previous")
        self.run_main()
        self.assertEqual(old.read_bytes(), b"previous")
        self.assertEqual([path.name for path in self.videos()], ["sc132gs_0001.mp4", "sc132gs_0002.mp4"])

    def test_ring_retains_the_inflight_slot_and_never_grows(self):
        queue = self.script.RecordingQueue(2, 1024)
        identities = [id(slot.data) for slot in queue.slots]
        first = queue.reserve()
        queue.publish()
        self.assertIs(queue.front(), first)
        queue.publish()
        for _ in range(1000):
            self.assertIsNone(queue.reserve())
        self.assertEqual([id(slot.data) for slot in queue.slots], identities)
        queue.retire()
        self.assertIs(queue.reserve(), first)
        self.assertEqual(queue.peak, 2)

    def test_full_queue_drops_new_frames_and_waits_for_new_idr(self):
        self.script.RECORDING_QUEUE_FRAMES = 1
        # Keep the consumer inside its first disk write while the producer fills.
        self.block_write = True
        self.io_release.clear()
        with patch.dict("sys.modules", self.modules):
            recorder = self.recorder()
            self.queue = [[(0, SPS + PPS, 100), (1, IDR, 100)]]
            recorder.poll()
            self.assertTrue(self.io_started.wait(1))
            identities = [id(slot.data) for slot in recorder.queue.slots]
            for index in range(100):
                self.queue.append([(2, PFRAME, 200 + index)])
                recorder.poll()
            self.assertEqual(recorder.enqueued, 1)
            self.assertEqual(recorder.dropped, 100)
            self.assertEqual(recorder.queue.peak, 1)
            self.encoder.RequestIDR.assert_called_once()
            self.assertFalse(self.addresses)
            self.io_release.set()
            deadline = time.monotonic() + 1
            while recorder.queue.consumed == 0 and time.monotonic() < deadline:
                time.sleep(0.001)
            self.queue = [[(2, PFRAME, 400)], [(1, IDR, 500)]]
            recorder.poll()
            self.assertEqual(recorder.enqueued, 2)
            self.assertEqual(recorder.dropped, 101)
            self.assertEqual([id(slot.data) for slot in recorder.queue.slots], identities)
            recorder.close()
            self.assertEqual(recorder.frames, 2)

    def test_oversized_frame_is_dropped_before_dereferencing_or_copying_it(self):
        with patch.dict("sys.modules", self.modules):
            recorder = self.recorder()
            recorder.headers[:len(SPS + PPS)] = SPS + PPS
            recorder.header_size = len(SPS + PPS)
            recorder.stream = SimpleNamespace(pack_cnt=1, stream_type=[1], data=[999],
                                               data_size=[self.script.RECORDING_FRAME_BYTES + 1], pts=[100])
            recorder.copy_stream()
            self.assertEqual((recorder.oversized, recorder.dropped, recorder.enqueued), (1, 1, 0))
            self.encoder.RequestIDR.assert_called_once()
            recorder.close()

    def test_poll_drains_a_bounded_number_of_native_streams(self):
        with patch.dict("sys.modules", self.modules):
            recorder = self.recorder()
            self.queue = [[(0, SPS + PPS, 100), (1, IDR, 100)]]
            self.queue.extend([[(2, PFRAME, 100 + i)] for i in range(1, 40)])
            recorder.poll()
            self.assertEqual(len(self.queue), 40 - recorder.MAX_STREAMS_PER_POLL)
            self.assertFalse(self.addresses)
            recorder.close()

    def test_storage_default_is_exactly_two_decimal_gigabytes(self):
        self.assertEqual(self.script.RecordingWriter.STORAGE_LIMIT_BYTES, 2000000000)

    def test_full_directory_stops_recording_and_keeps_preview_running(self):
        directory = Path(self.script.RECORDING_DIR)
        directory.mkdir()
        old = directory / "older.mp4"
        with old.open("wb") as file:
            file.truncate(2000000000)  # Sparse file: no 2 GB host allocation.
        self.run_main()
        self.assertEqual(self.display.show_image.call_count, 4)
        self.assertEqual(old.stat().st_size, 2000000000)
        self.assertEqual(list(directory.iterdir()), [old])
        self.assertIn("2 GB directory limit reached", self.output)
        self.assertIn("[PASS] capture:", self.output)

    def test_limit_includes_mp4_overhead_and_all_existing_files(self):
        reference = io.BytesIO()
        queue = self.script.RecordingQueue(3, 1024)
        first = self.load_slot(queue.reserve())
        muxer = self.script.FragmentedMp4(reference, 640, 480, 30, 100000)
        muxer.write(first)
        self.load_slot(first, pts=1033333, key=False)
        muxer.write(first)
        two_frame_size = muxer.bytes
        directory = Path(self.script.RECORDING_DIR)
        directory.mkdir()
        old = directory / "unrelated.data"
        old.write_bytes(b"a" * 73)
        for index in range(3):
            self.load_slot(queue.reserve(), pts=1000000 + index * 33333, key=index == 0)
            queue.publish()
        writer = self.script.RecordingWriter(queue, 640, 480, 30)
        writer.STORAGE_LIMIT_BYTES = two_frame_size + 73
        writer.stop = True
        writer.run()
        self.assertTrue(writer.limit_reached)
        self.assertEqual(writer.frames, 2)
        self.assertEqual(writer.bytes, two_frame_size)
        self.assertEqual(sum(path.stat().st_size for path in directory.iterdir()), two_frame_size + 73)
        self.assertEqual(old.read_bytes(), b"a" * 73)

    def test_budget_smaller_than_first_fragment_writes_zero_bytes(self):
        queue = self.script.RecordingQueue(1, 1024)
        self.load_slot(queue.reserve())
        queue.publish()
        writer = self.script.RecordingWriter(queue, 640, 480, 30)
        writer.STORAGE_LIMIT_BYTES = 128
        writer.stop = True
        writer.run()
        self.assertTrue(writer.limit_reached)
        self.assertEqual(Path(writer.path).stat().st_size, 0)

    def test_short_writes_do_not_break_fragment_or_capacity_accounting(self):
        output = bytearray()
        file = SimpleNamespace(write=lambda view: output.extend(view[:3]) or min(3, len(view)))
        muxer = self.script.FragmentedMp4(file, 640, 480, 30, 10000)
        slot = self.load_slot(self.script.RecordingSlot(1024))
        muxer.write(slot)
        self.assertEqual(muxer.bytes, len(output))
        self.assertEqual(output.count(b"moof"), 1)

    def test_disk_failure_releases_native_stream_and_cleans_resources(self):
        self.script.open = Mock(side_effect=OSError("SD full"))
        with self.assertRaisesRegex(self.script.RecordingError, "SD full"):
            self.run_main()
        self.assert_released()
        self.assertNotIn("[PASS] capture:", self.output)

    def test_encoder_start_failure_does_not_start_a_writer(self):
        self.encoder.Start.side_effect = RuntimeError("start failed")
        with self.assertRaisesRegex(RuntimeError, "start failed"):
            self.run_main()
        self.encoder.Destroy.assert_called_once()
        self.link.destroy.assert_called_once()
        self.assertFalse(self.threads)

    def test_start_thread_failure_still_releases_native_resources(self):
        self.modules["_thread"].start_new_thread = Mock(side_effect=RuntimeError("thread failed"))
        with self.assertRaisesRegex(RuntimeError, "thread failed"):
            self.run_main()
        self.sensor.stop.assert_called_once_with()
        self.encoder.Stop.assert_called_once()
        self.encoder.Destroy.assert_called_once()
        self.link.destroy.assert_called_once()
        self.preview_factory.assert_not_called()
        self.assertFalse(self.file_calls)

    def test_flush_stall_does_not_hold_main_or_reuse_writer_buffers(self):
        self.script.RECORDING_STOP_TIMEOUT_MS = 30
        self.block_flush = True
        self.io_release.clear()
        with patch.dict("sys.modules", self.modules):
            recorder = self.recorder()
            self.queue = [[(0, SPS + PPS, 100), (1, IDR, 100)]]
            recorder.poll()
            started = time.monotonic()
            with self.assertRaisesRegex(self.script.RecordingError, "still finishing"):
                recorder.close()
            self.assertLess(time.monotonic() - started, 0.3)
            self.assertTrue(self.io_started.is_set())
            self.assertFalse(recorder.writer.done)
            self.assertTrue(all(ident != threading.get_ident() for _, ident in self.file_calls))
            # Main remains able to do independent work while the worker owns I/O.
            self.assertEqual(sum(range(1000)), 499500)
            self.assertEqual(len(recorder.queue.slots[0].data), self.script.RECORDING_FRAME_BYTES)
            self.io_release.set()
        self.finish_threads()
        self.assertTrue(recorder.writer.done)

    def test_flush_timeout_still_releases_display_and_prevents_success(self):
        self.script.RECORDING_STOP_TIMEOUT_MS = 30
        self.block_flush = True
        self.io_release.clear()
        with self.assertRaisesRegex(self.script.RecordingError, "still finishing"):
            self.run_main()
        self.assert_released()
        self.assertNotIn("[PASS] capture:", self.output)
        self.io_release.set()

    def test_interrupt_closes_file_in_worker(self):
        original = self.sensor.snapshot.side_effect

        def interrupt(**kwargs):
            if self.sensor.snapshot.call_count == 3:
                raise Exception("IDE interrupt")
            return original(**kwargs)

        self.sensor.snapshot.side_effect = interrupt
        self.run_main()
        self.assertIn("[STOP] IDE interrupted", self.output)
        self.assertIn("video_frames=2", self.output)
        self.assert_released()

    def test_invalid_recording_config_fails_before_camera_use(self):
        for name, value in (("RECORDING_ENABLED", 1), ("RECORDING_FPS", 0),
                            ("RECORDING_FPS", True), ("RECORDING_FPS", 61),
                            ("RECORDING_BITRATE_KBPS", 0), ("RECORDING_BITRATE_KBPS", True),
                            ("RECORDING_BITRATE_KBPS", 20001), ("RECORDING_DIR", "relative"),
                            ("RECORDING_DIR", None), ("RECORDING_DIR", "/"),
                            ("RECORDING_DIR", "/sdcard/\x00bad"),
                            ("RECORDING_DIR", "/sdcard/" + "测" * 40),
                            ("RECORDING_QUEUE_FRAMES", 0), ("RECORDING_QUEUE_FRAMES", 9),
                            ("RECORDING_QUEUE_FRAMES", True), ("RECORDING_FRAME_BYTES", 0),
                            ("RECORDING_FRAME_BYTES", 524289), ("RECORDING_FRAME_BYTES", True),
                            ("RECORDING_STOP_TIMEOUT_MS", 0), ("RECORDING_STOP_TIMEOUT_MS", 5001),
                            ("RAW_CHECK_ENABLED", True), ("PROBE_ONLY", True)):
            with self.subTest(name=name, value=value), patch.object(self.script, name, value):
                with self.assertRaises(ValueError):
                    self.run_main()
        self.board.camera_sensor.assert_not_called()
        self.encoder_factory.assert_not_called()

    @unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"), "FFmpeg tools required")
    def test_real_h264_fragments_decode_and_preserve_capture_timestamps(self):
        raw = Path(self.directory.name) / "source.h264"
        subprocess.run(["ffmpeg", "-v", "error", "-f", "lavfi", "-i", "testsrc2=size=320x240:rate=30",
                        "-frames:v", "5", "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
                        "-x264-params", "aud=1:keyint=2:scenecut=0", "-f", "h264", str(raw)], check=True)
        data = bytearray(raw.read_bytes())
        nals = list(self.script.h264_nals(data, len(data)))
        units, current = [], []
        for begin, end in nals:
            nal = bytes(data[begin:end])
            if nal[0] & 31 == 9 and current:
                units.append(current)
                current = []
            current.append(nal)
        if current:
            units.append(current)
        output = Path(self.directory.name) / "playback.mp4"
        slot = self.script.RecordingSlot(256 * 1024)
        timestamps = [0, 33333, 100000, 133333, 233333]  # Includes dropped-frame gaps.
        with output.open("wb") as file:
            muxer = self.script.FragmentedMp4(file, 320, 240, 30, 1000000)
            headers = b""
            for pts, unit in zip(timestamps, units):
                key = any(nal[0] & 31 == 5 for nal in unit)
                new_headers = b"".join(b"\x00\x00\x01" + nal for nal in unit if nal[0] & 31 in (7, 8))
                if new_headers:
                    headers = new_headers
                payload = b"".join(b"\x00\x00\x01" + nal for nal in unit if nal[0] & 31 not in (7, 8))
                header = headers if key else b""
                slot.view[:len(header + payload)] = header + payload
                slot.size, slot.header_size, slot.pts, slot.key = len(header + payload), len(header), pts, key
                self.assertTrue(muxer.write(slot))
        result = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v", "-show_streams",
                                 "-show_packets", "-of", "json", str(output)], capture_output=True, check=True)
        parsed = json.loads(result.stdout)
        self.assertEqual((parsed["streams"][0]["width"], parsed["streams"][0]["height"]), (320, 240))
        self.assertEqual([round(float(packet["pts_time"]) * 1000000) for packet in parsed["packets"]], timestamps)
        decoded = subprocess.run(["ffmpeg", "-v", "error", "-i", str(output), "-f", "null", "-"],
                                 capture_output=True, check=True)
        self.assertFalse(decoded.stderr)


if __name__ == "__main__":
    unittest.main()
