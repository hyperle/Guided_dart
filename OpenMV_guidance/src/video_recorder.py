import gc
import mjpeg
import os
import time


_SD_ROOTS = ("/sdcard", "/sd")
_SEGMENT_PREFIX = "rec_"
_SEGMENT_SUFFIX = ".mjpeg"


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


def _entry_name(item):
    name = item[0]
    if isinstance(name, bytes):
        try:
            return name.decode()
        except Exception:
            return ""
    return name


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
        self._recordings_dir = ""
        self._enabled = False
        self._writer = None
        self._writer_path = ""
        self._segment_index = 0
        self._segment_start_ms = 0
        self._last_sync_ms = 0

        if not self._storage_root:
            print("recording disabled: no SD card")
            return

        self._recordings_dir = _join_path(self._storage_root, self._directory_name)
        if not self._ensure_recordings_dir():
            return
        self._segment_index = self._next_segment_index()
        self._prune_segments()
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
            self._write_current_frame(img, now_ms)
        except Exception as exc:
            print("recording write failed:", exc)
            self._recover_after_write_failure(img)

    def _recover_after_write_failure(self, img):
        self._close_writer()
        if not self._remove_oldest_available_segment():
            self._disable()
            return

        now_ms = time.ticks_ms()
        if not self._open_new_segment(img.width(), img.height(), now_ms):
            return

        try:
            self._write_current_frame(img, now_ms)
        except Exception as exc:
            print("recording retry failed:", exc)
            self._disable()

    def _write_current_frame(self, img, now_ms):
        self._writer.add_frame(img, quality=self._jpeg_quality)
        if time.ticks_diff(now_ms, self._last_sync_ms) >= self._sync_interval_ms:
            self._writer.sync()
            self._last_sync_ms = now_ms

    def _detect_storage_root(self):
        for candidate in _SD_ROOTS:
            if _path_exists(candidate):
                return candidate
        return ""

    def _ensure_recordings_dir(self):
        if _path_exists(self._recordings_dir):
            return True

        try:
            os.mkdir(self._recordings_dir)
            return True
        except OSError as exc:
            print("recording disabled: mkdir failed:", exc)
            return False

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
        self._prune_segments()
        if self._try_open_new_segment(width, height, now_ms):
            return True

        if self._remove_oldest_available_segment():
            gc.collect()
            if self._try_open_new_segment(width, height, time.ticks_ms()):
                return True

        self._disable()
        return False

    def _try_open_new_segment(self, width, height, now_ms):
        try:
            path = self._segment_path(self._segment_index)
            self._writer = mjpeg.Mjpeg(path, width, height)
            self._writer_path = path
            self._segment_start_ms = now_ms
            self._last_sync_ms = now_ms
            self._segment_index += 1
            print("recording started:", path)
            return True
        except Exception as exc:
            self._writer = None
            self._writer_path = ""
            print("recording open failed:", exc)
            return False

    def _segment_path(self, index):
        return _join_path(self._recordings_dir, "%s%05d%s" % (_SEGMENT_PREFIX, index, _SEGMENT_SUFFIX))

    def _list_segment_entries(self):
        entries = []
        try:
            iterator = os.ilistdir(self._recordings_dir)
        except OSError:
            return entries

        for item in iterator:
            name = _entry_name(item)
            if not (name.startswith(_SEGMENT_PREFIX) and name.endswith(_SEGMENT_SUFFIX)):
                continue
            entries.append(name)

        entries.sort()
        return entries

    def _next_segment_index(self):
        entries = self._list_segment_entries()
        highest = -1

        for name in entries:
            try:
                value = int(name[len(_SEGMENT_PREFIX):-len(_SEGMENT_SUFFIX)])
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

    def _remove_oldest_available_segment(self):
        entries = self._list_segment_entries()
        return self._remove_oldest_segment(entries)

    def _prune_segments(self):
        entries = self._list_segment_entries()

        while len(entries) >= self._max_segments:
            if not self._remove_oldest_segment(entries):
                break

        while entries and (self._free_bytes() < self._min_free_bytes):
            if not self._remove_oldest_segment(entries):
                break

        free_bytes = self._free_bytes()
        if free_bytes < self._min_free_bytes:
            print("recording low space:", free_bytes)
