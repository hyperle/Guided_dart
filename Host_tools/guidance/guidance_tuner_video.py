from __future__ import annotations

import json
import os
from typing import Iterator

import cv2
import numpy as np


class VideoSource:
    def __init__(self, path: str, frame_count: int, width: int, height: int):
        self.path = path
        self.frame_count = frame_count
        self.width = width
        self.height = height
        self.metadata_path = self._sidecar_path(path)
        self._metadata = self._load_sidecar_metadata(self.metadata_path)

    @staticmethod
    def _sidecar_path(path: str) -> str:
        base, _suffix = os.path.splitext(path)
        return base + ".jsonl"

    @staticmethod
    def _load_sidecar_metadata(path: str) -> list[dict]:
        if not os.path.isfile(path):
            return []
        records: list[dict] = []
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    records.append({})
        return records

    def has_metadata(self) -> bool:
        return bool(self._metadata)

    def get_metadata(self, index: int) -> dict | None:
        if index < 0 or index >= len(self._metadata):
            return None
        return self._metadata[index]

    def get_frame(self, index: int) -> np.ndarray:
        raise NotImplementedError

    def iter_frames(self, start: int = 0, stop: int | None = None) -> Iterator[np.ndarray]:
        raise NotImplementedError

    def close(self) -> None:
        return None


class IndexedMjpegVideoSource(VideoSource):
    _SCAN_CHUNK_SIZE = 1024 * 1024

    def __init__(self, path: str):
        self._offsets = self._scan_offsets(path)
        if not self._offsets:
            raise RuntimeError(f"failed to decode any frames from video: {path}")

        self._cache_index: int | None = None
        self._cache_frame: np.ndarray | None = None
        first_frame = self._decode_frame(path, 0, self._offsets[0])
        super().__init__(path, len(self._offsets), int(first_frame.shape[1]), int(first_frame.shape[0]))
        self._cache_index = 0
        self._cache_frame = first_frame.copy()

    @classmethod
    def _scan_offsets(cls, path: str) -> list[tuple[int, int]]:
        offsets: list[tuple[int, int]] = []
        carry = b""
        buffer_base = 0
        start_abs: int | None = None

        with open(path, "rb") as handle:
            while True:
                chunk = handle.read(cls._SCAN_CHUNK_SIZE)
                if not chunk:
                    break

                buffer = carry + chunk
                search_pos = 0
                while True:
                    if start_abs is None:
                        start_index = buffer.find(b"\xff\xd8", search_pos)
                        if start_index < 0:
                            break
                        start_abs = buffer_base + start_index
                        search_pos = start_index + 2

                    end_index = buffer.find(b"\xff\xd9", search_pos)
                    if end_index < 0:
                        break

                    offsets.append((start_abs, buffer_base + end_index + 2))
                    start_abs = None
                    search_pos = end_index + 2

                carry = buffer[-1:] if buffer else b""
                buffer_base += len(buffer) - len(carry)

        return offsets

    @staticmethod
    def _decode_frame(path: str, index: int, offset_pair: tuple[int, int]) -> np.ndarray:
        start, end = offset_pair
        with open(path, "rb") as handle:
            handle.seek(start)
            payload = handle.read(end - start)

        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"failed to decode frame {index} from video: {path}")
        return frame

    def get_frame(self, index: int) -> np.ndarray:
        if index < 0 or index >= self.frame_count:
            raise IndexError(index)
        if self._cache_index == index and self._cache_frame is not None:
            return self._cache_frame.copy()

        frame = self._decode_frame(self.path, index, self._offsets[index])
        self._cache_index = index
        self._cache_frame = frame.copy()
        return frame

    def iter_frames(self, start: int = 0, stop: int | None = None) -> Iterator[np.ndarray]:
        if stop is None:
            stop = self.frame_count
        start = max(0, start)
        stop = min(stop, self.frame_count)
        with open(self.path, "rb") as handle:
            for index in range(start, stop):
                frame_start, frame_end = self._offsets[index]
                handle.seek(frame_start)
                payload = handle.read(frame_end - frame_start)
                frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError(f"failed to decode frame {index} from video: {self.path}")
                self._cache_index = index
                self._cache_frame = frame.copy()
                yield frame


class OpenCvVideoSource(VideoSource):
    def __init__(self, path: str):
        frame_count, width, height = self._inspect_stream(path)
        self._capture = cv2.VideoCapture(path)
        if not self._capture.isOpened():
            raise RuntimeError(f"failed to open video: {path}")
        self._cache_index: int | None = None
        self._cache_frame: np.ndarray | None = None
        self._next_capture_index: int | None = None
        super().__init__(path, frame_count, width, height)

    @staticmethod
    def _inspect_stream(path: str) -> tuple[int, int, int]:
        capture = cv2.VideoCapture(path)
        if not capture.isOpened():
            raise RuntimeError(f"failed to open video: {path}")

        frame_count = 0
        width = 0
        height = 0
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                if frame_count == 0:
                    height, width = frame.shape[:2]
                frame_count += 1
        finally:
            capture.release()

        if frame_count == 0:
            raise RuntimeError(f"failed to decode any frames from video: {path}")
        return frame_count, width, height

    def close(self) -> None:
        self._capture.release()

    def get_frame(self, index: int) -> np.ndarray:
        if index < 0 or index >= self.frame_count:
            raise IndexError(index)
        if self._cache_index == index and self._cache_frame is not None:
            return self._cache_frame.copy()

        if self._next_capture_index != index:
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, float(index))
        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to decode frame {index} from video: {self.path}")

        self._next_capture_index = index + 1
        self._cache_index = index
        self._cache_frame = frame.copy()
        return frame

    def iter_frames(self, start: int = 0, stop: int | None = None) -> Iterator[np.ndarray]:
        if stop is None:
            stop = self.frame_count
        start = max(0, start)
        stop = min(stop, self.frame_count)

        capture = cv2.VideoCapture(self.path)
        if not capture.isOpened():
            raise RuntimeError(f"failed to open video: {self.path}")

        try:
            if start > 0:
                capture.set(cv2.CAP_PROP_POS_FRAMES, float(start))
            for index in range(start, stop):
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError(f"failed to decode frame {index} from video: {self.path}")
                self._cache_index = index
                self._cache_frame = frame.copy()
                yield frame
        finally:
            capture.release()


def load_video_source(path: str) -> VideoSource:
    suffix = os.path.splitext(path)[1].lower()
    if suffix in {".mjpeg", ".mjpg"}:
        return IndexedMjpegVideoSource(path)

    try:
        return OpenCvVideoSource(path)
    except Exception:
        return IndexedMjpegVideoSource(path)
