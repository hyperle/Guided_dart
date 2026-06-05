import gc
import mjpeg
import os
import time


def _join_path(base, name):
    if base == "/":
        return "/" + name
    return base + "/" + name


def _path_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


class RollingMjpegRecorder:
    def __init__(self,
                 directory_name="recordings",
                 segment_duration_ms=15000,
                 max_segments=12,
                 sync_interval_ms=1000,
                 jpeg_quality=70,
                 min_free_bytes=4 * 1024 * 1024):
        self._directory_name = directory_name
        self._segment_duration_ms = max(1000, int(segment_duration_ms))
        self._max_segments = max(1, int(max_segments))
        self._sync_interval_ms = max(250, int(sync_interval_ms))
        self._jpeg_quality = min(100, max(1, int(jpeg_quality)))
        self._min_free_bytes = max(0, int(min_free_bytes))

        self._storage_root = self._detect_storage_root()
        self._recordings_dir = _join_path(self._storage_root, self._directory_name)
        self._enabled = False
        self._writer = None
        self._writer_path = ""
        self._segment_index = 0
        self._segment_start_ms = 0
        self._last_sync_ms = 0

        self._ensure_recordings_dir()
        self._segment_index = self._next_segment_index()
        self._enabled = True
        print("recording dir:", self._recordings_dir)

    def is_enabled(self):
        return self._enabled

    def recordings_dir(self):
        return self._recordings_dir

    def add_frame(self, img):
        if (not self._enabled) or (img is None):
            return

        now_ms = time.ticks_ms()

        if self._writer is None:
            if not self._open_new_segment(img.width(), img.height(), now_ms):
                return

        if time.ticks_diff(now_ms, self._segment_start_ms) >= self._segment_duration_ms:
            self._close_writer()
            if not self._open_new_segment(img.width(), img.height(), now_ms):
                return

        try:
            self._writer.add_frame(img, quality=self._jpeg_quality)
            if time.ticks_diff(now_ms, self._last_sync_ms) >= self._sync_interval_ms:
                self._writer.sync()
                os.sync()
                self._last_sync_ms = now_ms
        except Exception as exc:
            print("recording write failed:", exc)
            self._disable()

    def _detect_storage_root(self):
        for candidate in ("/sdcard", "/sd", "/flash"):
            if _path_exists(candidate):
                return candidate
        return os.getcwd()

    def _ensure_recordings_dir(self):
        try:
            os.mkdir(self._recordings_dir)
        except OSError:
            pass

    def _disable(self):
        self._close_writer()
        self._enabled = False

    def _close_writer(self):
        if self._writer is None:
            return

        try:
            self._writer.sync()
        except Exception:
            pass

        try:
            self._writer.close()
            os.sync()
            print("recording closed:", self._writer_path)
        except Exception as exc:
            print("recording close failed:", exc)

        self._writer = None
        self._writer_path = ""
        gc.collect()

    def _open_new_segment(self, width, height, now_ms):
        try:
            self._prune_segments()
            path = self._segment_path(self._segment_index)
            self._writer = mjpeg.Mjpeg(path, width, height)
            self._writer_path = path
            self._segment_start_ms = now_ms
            self._last_sync_ms = now_ms
            self._segment_index += 1
            print("recording started:", path)
            return True
        except Exception as exc:
            print("recording open failed:", exc)
            self._disable()
            return False

    def _segment_path(self, index):
        return _join_path(self._recordings_dir, "rec_%05d.mjpeg" % index)

    def _list_segment_entries(self):
        entries = []
        try:
            iterator = os.ilistdir(self._recordings_dir)
        except OSError:
            return entries

        for item in iterator:
            name = item[0]
            if not (name.startswith("rec_") and name.endswith(".mjpeg")):
                continue
            entries.append(name)

        entries.sort()
        return entries

    def _next_segment_index(self):
        entries = self._list_segment_entries()
        highest = -1

        for name in entries:
            try:
                value = int(name[4:9])
            except ValueError:
                continue
            if value > highest:
                highest = value

        return highest + 1

    def _free_bytes(self):
        try:
            stats = os.statvfs(self._recordings_dir)
        except OSError:
            return 0

        block_size = stats[0]
        free_blocks = stats[4]
        return int(block_size * free_blocks)

    def _remove_oldest_segment(self, entries):
        if not entries:
            return False

        oldest = entries.pop(0)
        path = _join_path(self._recordings_dir, oldest)
        try:
            os.remove(path)
            os.sync()
            print("recording removed:", path)
            return True
        except OSError as exc:
            print("recording remove failed:", exc)
            return False

    def _prune_segments(self):
        entries = self._list_segment_entries()

        while len(entries) >= self._max_segments:
            if not self._remove_oldest_segment(entries):
                break

        while entries and (self._free_bytes() < self._min_free_bytes):
            if not self._remove_oldest_segment(entries):
                break
