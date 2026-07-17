import gc
import mjpeg
import os
import time


_SD_ROOTS = ("/sdcard", "/sd")
_SEGMENT_PREFIX = "rec_"
_SEGMENT_SUFFIX = ".mjpeg"
_METADATA_SUFFIX = ".jsonl"


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


def _sdcard_block_device():
    try:
        import pyb
        sdcard = pyb.SDCard()
        del pyb
        return sdcard
    except Exception:
        pass

    try:
        import machine
        sdcard = machine.SDCard(1)
        del machine
        return sdcard
    except Exception:
        return None


def _mount_sdcard_if_needed():
    for candidate in _SD_ROOTS:
        if _path_exists(candidate):
            return candidate

    sdcard = _sdcard_block_device()
    if sdcard is None:
        return ""

    try:
        import vfs
        vfs.mount(vfs.VfsFat(sdcard), "/sdcard")
        del vfs
        if _path_exists("/sdcard"):
            return "/sdcard"
    except Exception:
        pass

    return ""


def _json_escape(value):
    text = str(value)
    text = text.replace(chr(92), chr(92) + chr(92))
    return text.replace(chr(34), chr(92) + chr(34))


def _json_value(value):
    if value is None:
        return "null"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return str(value)
    if isinstance(value, tuple) or isinstance(value, list):
        parts = []
        for item in value:
            parts.append(_json_value(item))
        return "[" + ",".join(parts) + "]"
    if isinstance(value, dict):
        parts = []
        for key, item in value.items():
            parts.append("\"%s\":%s" % (_json_escape(key), _json_value(item)))
        return "{" + ",".join(parts) + "}"
    return "\"%s\"" % _json_escape(value)


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
        self._metadata_file = None
        self._metadata_path = ""
        self._segment_frame_index = 0
        self._segment_index = 0
        self._segment_start_ms = 0
        self._last_sync_ms = 0

        if not self._storage_root:
            return

        self._recordings_dir = _join_path(self._storage_root, self._directory_name)
        if not self._ensure_recordings_dir():
            return
        self._segment_index = self._next_segment_index()
        self._prune_segments()
        self._enabled = True

    def is_enabled(self):
        return self._enabled

    def recordings_dir(self):
        return self._recordings_dir

    def close(self, quiet=True):
        self._close_writer(quiet)

    def add_frame(self, img, metadata=None):
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
            self._write_current_frame(img, now_ms, metadata)
        except Exception:
            self._recover_after_write_failure(img, metadata)

    def _recover_after_write_failure(self, img, metadata):
        self._close_writer()
        if not self._remove_oldest_available_segment():
            self._disable()
            return

        now_ms = time.ticks_ms()
        if not self._open_new_segment(img.width(), img.height(), now_ms):
            return

        try:
            self._write_current_frame(img, now_ms, metadata)
        except Exception:
            self._disable()

    def _write_current_frame(self, img, now_ms, metadata):
        self._writer.add_frame(img, quality=self._jpeg_quality)
        self._write_metadata_line(img, now_ms, metadata)
        self._segment_frame_index += 1
        if time.ticks_diff(now_ms, self._last_sync_ms) >= self._sync_interval_ms:
            self._writer.sync()
            if self._metadata_file is not None:
                try:
                    self._metadata_file.flush()
                except Exception:
                    pass
            self._last_sync_ms = now_ms

    def _write_metadata_line(self, img, now_ms, metadata):
        if self._metadata_file is None:
            return
        record = {
            "frame_index": self._segment_frame_index,
            "target_search": "metadata_missing",
        }
        if metadata is not None and metadata.get("target_search", ""):
            record["target_search"] = metadata.get("target_search")
        self._metadata_file.write(_json_value(record))
        self._metadata_file.write("\n")

    def _detect_storage_root(self):
        return _mount_sdcard_if_needed()

    def _ensure_recordings_dir(self):
        if _path_exists(self._recordings_dir):
            return True

        try:
            os.mkdir(self._recordings_dir)
            return True
        except OSError:
            return False

    def _disable(self):
        self._close_writer()
        self._enabled = False

    def _close_writer(self, quiet=False):
        if self._writer is None:
            return

        try:
            self._writer.sync()
        except Exception:
            pass

        try:
            self._writer.close()
            os.sync()
        except Exception:
            pass

        self._writer = None
        self._writer_path = ""
        if self._metadata_file is not None:
            try:
                self._metadata_file.flush()
            except Exception:
                pass
            try:
                self._metadata_file.close()
            except Exception:
                pass
        self._metadata_file = None
        self._metadata_path = ""
        self._segment_frame_index = 0
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
            metadata_path = self._metadata_path_for_segment(self._segment_index)
            self._writer = mjpeg.Mjpeg(path, width, height)
            self._metadata_file = open(metadata_path, "w")
            self._writer_path = path
            self._metadata_path = metadata_path
            self._segment_frame_index = 0
            self._segment_start_ms = now_ms
            self._last_sync_ms = now_ms
            self._segment_index += 1
            return True
        except Exception:
            try:
                if self._writer is not None:
                    self._writer.close()
            except Exception:
                pass
            try:
                if self._metadata_file is not None:
                    self._metadata_file.close()
            except Exception:
                pass
            self._writer = None
            self._writer_path = ""
            self._metadata_file = None
            self._metadata_path = ""
            return False

    def _segment_path(self, index):
        return _join_path(self._recordings_dir, "%s%05d%s" % (_SEGMENT_PREFIX, index, _SEGMENT_SUFFIX))

    def _metadata_path_for_segment(self, index):
        return _join_path(self._recordings_dir, "%s%05d%s" % (_SEGMENT_PREFIX, index, _METADATA_SUFFIX))

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
        metadata_name = oldest[:-len(_SEGMENT_SUFFIX)] + _METADATA_SUFFIX
        metadata_path = _join_path(self._recordings_dir, metadata_name)
        try:
            os.remove(path)
            if _path_exists(metadata_path):
                os.remove(metadata_path)
            os.sync()
            return True
        except OSError:
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

