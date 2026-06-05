#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from guidance_host_common import GuidanceHostPaths, YamlLiteParser


DISPLAY_MAX_WIDTH = 480
DISPLAY_MAX_HEIGHT = 360
CV_FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class SimulationParams:
    exposure_scale: float = 1.0
    r_gain: float = 1.0
    g_gain: float = 1.0
    b_gain: float = 1.0
    gamma: float = 1.0


@dataclass
class DetectorParams:
    threshold_l_min: int = 20
    threshold_l_max: int = 70
    threshold_a_min: int = -77
    threshold_a_max: int = -19
    threshold_b_min: int = -19
    threshold_b_max: int = 38
    min_area: int = 20
    max_area: int = 2000
    roundness_min_x1000: int = 700
    merge_margin: int = 5
    track_window_radius_px: int = 80
    center_filter_gain_x100: int = 70
    max_missed_frames: int = 2


@dataclass
class SweepParams:
    exposure_min: float = 0.8
    exposure_max: float = 1.4
    exposure_step: float = 0.05
    gain_min: float = 0.8
    gain_max: float = 1.4
    gain_step: float = 0.05

    @staticmethod
    def value_for_frame(min_value: float, max_value: float, step: float, frame_index: int) -> float:
        minimum = min(min_value, max_value)
        maximum = max(min_value, max_value)
        effective_step = abs(step)
        if maximum <= minimum or effective_step < 1e-6:
            return minimum

        count = int(round((maximum - minimum) / effective_step)) + 1
        count = max(count, 1)
        return minimum + effective_step * (frame_index % count)

    def values_for_frame(self, frame_index: int) -> tuple[float, float]:
        exposure = self.value_for_frame(self.exposure_min, self.exposure_max, self.exposure_step, frame_index)
        gain = self.value_for_frame(self.gain_min, self.gain_max, self.gain_step, frame_index)
        return exposure, gain


@dataclass
class BlobCandidate:
    contour: np.ndarray
    area: int
    roundness_x1000: int
    center_x: int
    center_y: int
    radius_px: int
    bbox: tuple[int, int, int, int]
    passes_area: bool
    passes_roundness: bool
    overlaps_expected_roi: bool


@dataclass
class DetectionDebug:
    candidates: list[BlobCandidate]
    best_candidate: BlobCandidate | None
    mask: np.ndarray
    search_roi: tuple[int, int, int, int] | None
    fallback_used: bool
    reason: str
    detected: bool
    locked: bool
    background_misdetect: bool
    raw_center: tuple[int, int] | None
    filtered_center: tuple[int, int] | None
    area: int
    radius_px: int


@dataclass
class ValidationFrameResult:
    frame_index: int
    detected: bool
    locked: bool
    background_misdetect: bool
    reason: str
    raw_center: tuple[int, int] | None
    filtered_center: tuple[int, int] | None
    area: int
    radius_px: int
    exposure_scale: float
    gain_scale: float


@dataclass
class ValidationSummary:
    total_frames: int
    hit_frames: int
    hit_rate: float
    longest_miss_streak: int
    first_miss_frame: int | None
    jitter_mean: float
    jitter_std: float
    jitter_max: float
    misdetected_frames: list[int] = field(default_factory=list)
    recognition_log: list[str] = field(default_factory=list)
    frames: list[ValidationFrameResult] = field(default_factory=list)


def clamp(value: int | float, minimum: int | float, maximum: int | float) -> int | float:
    return max(minimum, min(maximum, value))


def normalize_roi(roi: tuple[int, int, int, int] | None, width: int, height: int) -> tuple[int, int, int, int] | None:
    if roi is None:
        return None

    x, y, w, h = roi
    x0 = int(clamp(x, 0, max(width - 1, 0)))
    y0 = int(clamp(y, 0, max(height - 1, 0)))
    x1 = int(clamp(x + w, 0, width))
    y1 = int(clamp(y + h, 0, height))
    if x1 <= x0 or y1 <= y0:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def point_in_roi(point: tuple[int, int] | None, roi: tuple[int, int, int, int] | None) -> bool:
    if point is None or roi is None:
        return False
    x, y = point
    rx, ry, rw, rh = roi
    return rx <= x < (rx + rw) and ry <= y < (ry + rh)


def rect_intersects(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> bool:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return not ((ax + aw) <= bx or (bx + bw) <= ax or (ay + ah) <= by or (by + bh) <= ay)


def roi_from_drag(start: tuple[int, int] | None, end: tuple[int, int] | None, width: int, height: int) -> tuple[int, int, int, int] | None:
    if start is None or end is None:
        return None
    x0 = int(clamp(min(start[0], end[0]), 0, max(width - 1, 0)))
    y0 = int(clamp(min(start[1], end[1]), 0, max(height - 1, 0)))
    x1 = int(clamp(max(start[0], end[0]), 0, width))
    y1 = int(clamp(max(start[1], end[1]), 0, height))
    if (x1 - x0) < 2 or (y1 - y0) < 2:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def scale_for_display(width: int, height: int) -> float:
    if width <= 0 or height <= 0:
        return 1.0
    return max(1.0, min(DISPLAY_MAX_WIDTH / float(width), DISPLAY_MAX_HEIGHT / float(height)))


def resize_for_display(image: np.ndarray, scale: float) -> np.ndarray:
    if abs(scale - 1.0) < 1e-6:
        return image.copy()
    interpolation = cv2.INTER_NEAREST if image.ndim == 2 else cv2.INTER_LINEAR
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=interpolation)


def bgr_to_photo(image_bgr: np.ndarray) -> ImageTk.PhotoImage:
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return ImageTk.PhotoImage(Image.fromarray(image_rgb))


def gray_to_bgr(mask: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def apply_simulation(
    frame_bgr: np.ndarray,
    sim_params: SimulationParams,
    dynamic_exposure_scale: float = 1.0,
    dynamic_gain_scale: float = 1.0,
) -> np.ndarray:
    image = frame_bgr.astype(np.float32) / 255.0
    channel_gain = np.array(
        [sim_params.b_gain, sim_params.g_gain, sim_params.r_gain],
        dtype=np.float32,
    ).reshape((1, 1, 3))
    image *= channel_gain
    image *= float(sim_params.exposure_scale) * float(dynamic_exposure_scale) * float(dynamic_gain_scale)
    image = np.clip(image, 0.0, 1.0)

    gamma = max(0.1, float(sim_params.gamma))
    image = np.power(image, 1.0 / gamma)
    return np.clip(image * 255.0 + 0.5, 0.0, 255.0).astype(np.uint8)


def threshold_lab_mask(frame_bgr: np.ndarray, params: DetectorParams) -> np.ndarray:
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l_plane = lab[:, :, 0].astype(np.float32) * (100.0 / 255.0)
    a_plane = lab[:, :, 1].astype(np.int16) - 128
    b_plane = lab[:, :, 2].astype(np.int16) - 128

    mask = (
        (l_plane >= params.threshold_l_min)
        & (l_plane <= params.threshold_l_max)
        & (a_plane >= params.threshold_a_min)
        & (a_plane <= params.threshold_a_max)
        & (b_plane >= params.threshold_b_min)
        & (b_plane <= params.threshold_b_max)
    )
    return (mask.astype(np.uint8) * 255)


def contour_to_candidate(
    contour: np.ndarray,
    params: DetectorParams,
    expected_roi: tuple[int, int, int, int] | None,
) -> BlobCandidate:
    area = int(round(cv2.contourArea(contour)))
    perimeter = float(cv2.arcLength(contour, True))
    if perimeter <= 0.0:
        roundness_x1000 = 0
    else:
        roundness_x1000 = int(round((4.0 * math.pi * float(area) / (perimeter * perimeter)) * 1000.0))

    x, y, w, h = cv2.boundingRect(contour)
    moments = cv2.moments(contour)
    if abs(moments["m00"]) > 1e-6:
        center_x = int(round(moments["m10"] / moments["m00"]))
        center_y = int(round(moments["m01"] / moments["m00"]))
    else:
        center_x = x + (w // 2)
        center_y = y + (h // 2)

    passes_area = area >= params.min_area
    if params.max_area > 0:
        passes_area = passes_area and area <= params.max_area
    passes_roundness = roundness_x1000 >= params.roundness_min_x1000

    bbox = (x, y, w, h)
    overlaps_expected_roi = True if expected_roi is None else rect_intersects(bbox, expected_roi)
    radius_px = max(1, int(w / 2))
    return BlobCandidate(
        contour=contour,
        area=area,
        roundness_x1000=roundness_x1000,
        center_x=center_x,
        center_y=center_y,
        radius_px=radius_px,
        bbox=bbox,
        passes_area=passes_area,
        passes_roundness=passes_roundness,
        overlaps_expected_roi=overlaps_expected_roi,
    )


def merge_contours_by_margin(mask: np.ndarray, margin: int, roi: tuple[int, int, int, int] | None = None) -> list[np.ndarray]:
    if roi is not None:
        x, y, w, h = roi
        if w <= 0 or h <= 0:
            return []
        work_mask = mask[y : y + h, x : x + w]
        offset = np.array([[[x, y]]], dtype=np.int32)
    else:
        work_mask = mask
        offset = np.array([[[0, 0]]], dtype=np.int32)

    contours, _ = cv2.findContours(work_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    contours = [contour + offset for contour in contours if cv2.contourArea(contour) > 0.0]
    if not contours:
        return []

    if margin <= 0:
        return contours

    bboxes = [cv2.boundingRect(contour) for contour in contours]
    parents = list(range(len(contours)))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(a_index: int, b_index: int) -> None:
        root_a = find(a_index)
        root_b = find(b_index)
        if root_a != root_b:
            parents[root_b] = root_a

    for left in range(len(contours)):
        lx, ly, lw, lh = bboxes[left]
        expanded_left = (lx - margin, ly - margin, lw + margin * 2, lh + margin * 2)
        for right in range(left + 1, len(contours)):
            if rect_intersects(expanded_left, bboxes[right]):
                union(left, right)

    groups: dict[int, list[int]] = {}
    for index in range(len(contours)):
        groups.setdefault(find(index), []).append(index)

    merged_contours: list[np.ndarray] = []
    for indices in groups.values():
        min_x = min(bboxes[index][0] for index in indices)
        min_y = min(bboxes[index][1] for index in indices)
        max_x = max(bboxes[index][0] + bboxes[index][2] for index in indices)
        max_y = max(bboxes[index][1] + bboxes[index][3] for index in indices)

        local_mask = np.zeros((max_y - min_y + 3, max_x - min_x + 3), dtype=np.uint8)
        shift = np.array([[[1 - min_x, 1 - min_y]]], dtype=np.int32)
        for index in indices:
            local_contour = contours[index] + shift
            cv2.drawContours(local_mask, [local_contour], -1, 255, thickness=cv2.FILLED)

        local_contours, _ = cv2.findContours(local_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in local_contours:
            if cv2.contourArea(contour) <= 0.0:
                continue
            merged_contours.append(contour + np.array([[[min_x - 1, min_y - 1]]], dtype=np.int32))

    return merged_contours


def choose_failure_reason(
    candidates: list[BlobCandidate],
    params: DetectorParams,
    expected_roi: tuple[int, int, int, int] | None,
    best_candidate: BlobCandidate | None,
) -> tuple[str, bool]:
    if best_candidate is not None:
        if expected_roi is not None and not point_in_roi((best_candidate.center_x, best_candidate.center_y), expected_roi):
            return ("误检背景", True)
        return ("锁定成功", False)

    if not candidates:
        return ("颜色不过阈值", False)

    inside_candidates = [
        candidate
        for candidate in candidates
        if expected_roi is None or point_in_roi((candidate.center_x, candidate.center_y), expected_roi)
    ]
    relevant = inside_candidates if inside_candidates else candidates

    if expected_roi is not None and not inside_candidates:
        valid_outside = [candidate for candidate in candidates if candidate.passes_area and candidate.passes_roundness]
        if valid_outside:
            return ("误检背景", True)

    if all(candidate.area < params.min_area for candidate in relevant):
        return ("面积太小", False)
    if params.max_area > 0 and all(candidate.area > params.max_area for candidate in relevant):
        return ("面积太大", False)

    area_passed = [candidate for candidate in relevant if candidate.passes_area]
    if area_passed and all(not candidate.passes_roundness for candidate in area_passed):
        return ("圆度不够", False)

    return ("未找到有效 blob", False)


class HostGreenLightDetector:
    def __init__(self, params: DetectorParams):
        self.params = params
        self._last_center: tuple[int, int] | None = None
        self._missed_frames = 0

    def reset(self) -> None:
        self._last_center = None
        self._missed_frames = 0

    def _resolve_roi(self, width: int, height: int) -> tuple[int, int, int, int] | None:
        if self._last_center is None:
            return None

        radius = int(self.params.track_window_radius_px)
        center_x, center_y = self._last_center
        start_x = int(clamp(center_x - radius, 0, width - 1))
        start_y = int(clamp(center_y - radius, 0, height - 1))
        end_x = int(clamp(center_x + radius, 0, width - 1))
        end_y = int(clamp(center_y + radius, 0, height - 1))
        return (start_x, start_y, end_x - start_x + 1, end_y - start_y + 1)

    def _update_track(self, center: tuple[int, int] | None) -> tuple[int, int] | None:
        if center is None:
            if self._missed_frames < self.params.max_missed_frames:
                self._missed_frames += 1
            else:
                self._last_center = None
            return None

        if self._last_center is None:
            self._last_center = center
            self._missed_frames = 0
            return center

        gain = int(clamp(self.params.center_filter_gain_x100, 0, 100))
        filtered_x = ((self._last_center[0] * (100 - gain)) + (center[0] * gain) + 50) // 100
        filtered_y = ((self._last_center[1] * (100 - gain)) + (center[1] * gain) + 50) // 100
        self._last_center = (filtered_x, filtered_y)
        self._missed_frames = 0
        return self._last_center

    def _analyze_candidates(
        self,
        frame_bgr: np.ndarray,
        expected_roi: tuple[int, int, int, int] | None,
        search_roi: tuple[int, int, int, int] | None,
    ) -> tuple[np.ndarray, list[BlobCandidate], BlobCandidate | None]:
        mask = threshold_lab_mask(frame_bgr, self.params)
        contours = merge_contours_by_margin(mask, self.params.merge_margin, search_roi)
        candidates = [contour_to_candidate(contour, self.params, expected_roi) for contour in contours]

        best_candidate: BlobCandidate | None = None
        best_roundness = -1
        for candidate in candidates:
            if not candidate.passes_area or not candidate.passes_roundness:
                continue
            if candidate.roundness_x1000 > best_roundness:
                best_roundness = candidate.roundness_x1000
                best_candidate = candidate

        return mask, candidates, best_candidate

    def detect_single_frame(
        self,
        frame_bgr: np.ndarray,
        expected_roi: tuple[int, int, int, int] | None,
    ) -> DetectionDebug:
        mask, candidates, best_candidate = self._analyze_candidates(frame_bgr, expected_roi, None)
        reason, background_misdetect = choose_failure_reason(candidates, self.params, expected_roi, best_candidate)

        detected = best_candidate is not None
        locked = detected and not background_misdetect
        raw_center = None if best_candidate is None else (best_candidate.center_x, best_candidate.center_y)
        return DetectionDebug(
            candidates=candidates,
            best_candidate=best_candidate,
            mask=mask,
            search_roi=None,
            fallback_used=False,
            reason=reason,
            detected=detected,
            locked=locked,
            background_misdetect=background_misdetect,
            raw_center=raw_center,
            filtered_center=raw_center,
            area=0 if best_candidate is None else best_candidate.area,
            radius_px=0 if best_candidate is None else best_candidate.radius_px,
        )

    def process_video_frame(
        self,
        frame_bgr: np.ndarray,
        expected_roi: tuple[int, int, int, int] | None,
    ) -> DetectionDebug:
        search_roi = self._resolve_roi(frame_bgr.shape[1], frame_bgr.shape[0])
        mask, candidates, best_candidate = self._analyze_candidates(frame_bgr, expected_roi, search_roi)
        fallback_used = False

        if best_candidate is None and search_roi is not None:
            _, candidates, best_candidate = self._analyze_candidates(frame_bgr, expected_roi, None)
            fallback_used = True

        reason, background_misdetect = choose_failure_reason(candidates, self.params, expected_roi, best_candidate)
        if best_candidate is None:
            self._update_track(None)
            return DetectionDebug(
                candidates=candidates,
                best_candidate=None,
                mask=mask,
                search_roi=search_roi,
                fallback_used=fallback_used,
                reason=reason,
                detected=False,
                locked=False,
                background_misdetect=background_misdetect,
                raw_center=None,
                filtered_center=None,
                area=0,
                radius_px=0,
            )

        raw_center = (best_candidate.center_x, best_candidate.center_y)
        filtered_center = self._update_track(raw_center)
        return DetectionDebug(
            candidates=candidates,
            best_candidate=best_candidate,
            mask=mask,
            search_roi=search_roi,
            fallback_used=fallback_used,
            reason=reason,
            detected=True,
            locked=not background_misdetect,
            background_misdetect=background_misdetect,
            raw_center=raw_center,
            filtered_center=filtered_center,
            area=best_candidate.area,
            radius_px=best_candidate.radius_px,
        )


class LoadedVideo:
    def __init__(self, path: str, frames: list[np.ndarray]):
        self.path = path
        self.frames = frames

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def width(self) -> int:
        return 0 if not self.frames else int(self.frames[0].shape[1])

    @property
    def height(self) -> int:
        return 0 if not self.frames else int(self.frames[0].shape[0])


def load_video_frames(path: str) -> LoadedVideo:
    frames: list[np.ndarray] = []

    capture = cv2.VideoCapture(path)
    if capture.isOpened():
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
    capture.release()

    if not frames:
        with open(path, "rb") as handle:
            data = handle.read()
        cursor = 0
        while True:
            start = data.find(b"\xff\xd8", cursor)
            if start < 0:
                break
            end = data.find(b"\xff\xd9", start + 2)
            if end < 0:
                break
            payload = np.frombuffer(data[start : end + 2], dtype=np.uint8)
            frame = cv2.imdecode(payload, cv2.IMREAD_COLOR)
            if frame is not None:
                frames.append(frame)
            cursor = end + 2

    if not frames:
        raise RuntimeError(f"failed to decode any frames from video: {path}")

    return LoadedVideo(path, frames)


def load_detector_defaults(config_path: str) -> DetectorParams:
    parser = YamlLiteParser()
    config = parser.parse_file(config_path)
    green_light = config.get("green_light", {})
    return DetectorParams(
        threshold_l_min=int(green_light.get("openmv_threshold_l_min", 20)),
        threshold_l_max=int(green_light.get("openmv_threshold_l_max", 70)),
        threshold_a_min=int(green_light.get("openmv_threshold_a_min", -77)),
        threshold_a_max=int(green_light.get("openmv_threshold_a_max", -19)),
        threshold_b_min=int(green_light.get("openmv_threshold_b_min", -19)),
        threshold_b_max=int(green_light.get("openmv_threshold_b_max", 38)),
        min_area=int(green_light.get("openmv_min_area", 20)),
        max_area=int(green_light.get("openmv_max_area", 2000)),
        roundness_min_x1000=int(green_light.get("openmv_roundness_min_x1000", 700)),
        merge_margin=int(green_light.get("openmv_merge_margin", 5)),
        track_window_radius_px=int(green_light.get("openmv_track_window_radius_px", 80)),
        center_filter_gain_x100=int(green_light.get("openmv_center_filter_gain_x100", 70)),
        max_missed_frames=int(green_light.get("openmv_max_missed_frames", 2)),
    )


class VideoTunerApp:
    def __init__(self, root: tk.Tk, video_path: str, config_path: str):
        self.root = root
        self.root.title("Guidance Video Tuner")
        self.root.geometry("1850x980")

        self.config_path = config_path
        self.video: LoadedVideo | None = None
        self.expected_roi: tuple[int, int, int, int] | None = None
        self.drag_start: tuple[int, int] | None = None
        self.drag_current: tuple[int, int] | None = None
        self.single_display_scale = 1.0
        self.single_frame_result: DetectionDebug | None = None
        self.validation_summary: ValidationSummary | None = None
        self.validation_preview_index = 0

        self._adjusted_photo: ImageTk.PhotoImage | None = None
        self._mask_photo: ImageTk.PhotoImage | None = None
        self._overlay_photo: ImageTk.PhotoImage | None = None
        self._validation_photo: ImageTk.PhotoImage | None = None

        self._refresh_single_job: str | None = None

        self.status_var = tk.StringVar(value="Open a video to start.")
        self.video_info_var = tk.StringVar(value="No video loaded")
        self.validation_progress_var = tk.StringVar(value="")

        defaults = load_detector_defaults(config_path)

        self.frame_index_var = tk.IntVar(value=0)
        self.validation_frame_var = tk.IntVar(value=0)

        self.exposure_scale_var = tk.DoubleVar(value=1.0)
        self.r_gain_var = tk.DoubleVar(value=1.0)
        self.g_gain_var = tk.DoubleVar(value=1.0)
        self.b_gain_var = tk.DoubleVar(value=1.0)
        self.gamma_var = tk.DoubleVar(value=1.0)

        self.threshold_l_min_var = tk.IntVar(value=defaults.threshold_l_min)
        self.threshold_l_max_var = tk.IntVar(value=defaults.threshold_l_max)
        self.threshold_a_min_var = tk.IntVar(value=defaults.threshold_a_min)
        self.threshold_a_max_var = tk.IntVar(value=defaults.threshold_a_max)
        self.threshold_b_min_var = tk.IntVar(value=defaults.threshold_b_min)
        self.threshold_b_max_var = tk.IntVar(value=defaults.threshold_b_max)
        self.min_area_var = tk.IntVar(value=defaults.min_area)
        self.max_area_var = tk.IntVar(value=defaults.max_area)
        self.roundness_min_var = tk.IntVar(value=defaults.roundness_min_x1000)
        self.merge_margin_var = tk.IntVar(value=defaults.merge_margin)
        self.track_window_radius_var = tk.IntVar(value=defaults.track_window_radius_px)
        self.center_filter_gain_var = tk.IntVar(value=defaults.center_filter_gain_x100)
        self.max_missed_frames_var = tk.IntVar(value=defaults.max_missed_frames)

        self.sweep_exposure_min_var = tk.DoubleVar(value=0.8)
        self.sweep_exposure_max_var = tk.DoubleVar(value=1.4)
        self.sweep_exposure_step_var = tk.DoubleVar(value=0.05)
        self.sweep_gain_min_var = tk.DoubleVar(value=0.8)
        self.sweep_gain_max_var = tk.DoubleVar(value=1.4)
        self.sweep_gain_step_var = tk.DoubleVar(value=0.05)

        self._build_ui()

        if video_path:
            self.load_video(video_path)

    def _build_ui(self) -> None:
        root_frame = ttk.Frame(self.root)
        root_frame.pack(fill=tk.BOTH, expand=True)

        root_frame.columnconfigure(0, weight=0)
        root_frame.columnconfigure(1, weight=1)
        root_frame.rowconfigure(0, weight=1)

        control_container = ttk.Frame(root_frame)
        control_container.grid(row=0, column=0, sticky="nsw")

        control_canvas = tk.Canvas(control_container, width=380, highlightthickness=0)
        control_scroll = ttk.Scrollbar(control_container, orient=tk.VERTICAL, command=control_canvas.yview)
        self.control_frame = ttk.Frame(control_canvas)
        self.control_frame.bind(
            "<Configure>",
            lambda event: control_canvas.configure(scrollregion=control_canvas.bbox("all")),
        )
        control_canvas.create_window((0, 0), window=self.control_frame, anchor="nw")
        control_canvas.configure(yscrollcommand=control_scroll.set)
        control_canvas.pack(side=tk.LEFT, fill=tk.Y, expand=False)
        control_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        main_panel = ttk.Frame(root_frame)
        main_panel.grid(row=0, column=1, sticky="nsew")
        main_panel.columnconfigure(0, weight=1)
        main_panel.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(main_panel)
        toolbar.grid(row=0, column=0, sticky="ew", padx=8, pady=6)
        toolbar.columnconfigure(2, weight=1)

        ttk.Button(toolbar, text="Open Video", command=self.choose_video).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(toolbar, text="Copy YAML Snippet", command=self.copy_yaml_snippet).grid(row=0, column=1, padx=(0, 6))
        ttk.Label(toolbar, textvariable=self.video_info_var).grid(row=0, column=2, sticky="w")

        self.notebook = ttk.Notebook(main_panel)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))

        self.single_tab = ttk.Frame(self.notebook)
        self.validation_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.single_tab, text="Single Frame")
        self.notebook.add(self.validation_tab, text="Validation")

        self._build_control_panel()
        self._build_single_tab()
        self._build_validation_tab()

        status_bar = ttk.Label(self.root, textvariable=self.status_var, anchor="w")
        status_bar.pack(fill=tk.X, padx=8, pady=(0, 6))

    def _build_control_panel(self) -> None:
        file_box = ttk.LabelFrame(self.control_frame, text="Video / ROI")
        file_box.pack(fill=tk.X, padx=8, pady=8)
        ttk.Button(file_box, text="Open Video...", command=self.choose_video).pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(file_box, text="Clear ROI", command=self.clear_roi).pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(file_box, text=f"Config: {self.config_path}", wraplength=340).pack(fill=tk.X, padx=6, pady=(4, 6))

        sim_box = ttk.LabelFrame(self.control_frame, text="Camera Simulation")
        sim_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(sim_box, "Exposure Scale", self.exposure_scale_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "R Gain", self.r_gain_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "G Gain", self.g_gain_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "B Gain", self.b_gain_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "Gamma", self.gamma_var, 0.2, 3.0, 0.01)

        detector_box = ttk.LabelFrame(self.control_frame, text="Detector Thresholds")
        detector_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(detector_box, "L Min", self.threshold_l_min_var, 0, 100, 1)
        self._add_scale(detector_box, "L Max", self.threshold_l_max_var, 0, 100, 1)
        self._add_scale(detector_box, "A Min", self.threshold_a_min_var, -128, 127, 1)
        self._add_scale(detector_box, "A Max", self.threshold_a_max_var, -128, 127, 1)
        self._add_scale(detector_box, "B Min", self.threshold_b_min_var, -128, 127, 1)
        self._add_scale(detector_box, "B Max", self.threshold_b_max_var, -128, 127, 1)
        self._add_scale(detector_box, "Min Area", self.min_area_var, 1, 20000, 1)
        self._add_scale(detector_box, "Max Area", self.max_area_var, 0, 50000, 1)
        self._add_scale(detector_box, "Roundness x1000", self.roundness_min_var, 0, 1000, 1)
        self._add_scale(detector_box, "Merge Margin", self.merge_margin_var, 0, 50, 1)

        tracking_box = ttk.LabelFrame(self.control_frame, text="Tracking / Validation")
        tracking_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(tracking_box, "Track Radius", self.track_window_radius_var, 1, 200, 1)
        self._add_scale(tracking_box, "Center Filter x100", self.center_filter_gain_var, 0, 100, 1)
        self._add_scale(tracking_box, "Max Missed Frames", self.max_missed_frames_var, 1, 20, 1)

        sweep_box = ttk.LabelFrame(self.control_frame, text="Sweep Debug Mode")
        sweep_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(sweep_box, "Sweep Exposure Min", self.sweep_exposure_min_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Exposure Max", self.sweep_exposure_max_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Exposure Step", self.sweep_exposure_step_var, 0.01, 1.0, 0.01)
        self._add_scale(sweep_box, "Sweep Gain Min", self.sweep_gain_min_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Gain Max", self.sweep_gain_max_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Gain Step", self.sweep_gain_step_var, 0.01, 1.0, 0.01)

    def _add_scale(
        self,
        parent: ttk.LabelFrame,
        label: str,
        variable: tk.Variable,
        minimum: float,
        maximum: float,
        resolution: float,
    ) -> None:
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=6, pady=3)

        ttk.Label(frame, text=label, width=18).pack(side=tk.LEFT)
        scale = tk.Scale(
            frame,
            from_=minimum,
            to=maximum,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=variable,
            command=lambda _value: self.schedule_single_refresh(),
            length=180,
        )
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True)

        entry = ttk.Entry(frame, textvariable=variable, width=8)
        entry.pack(side=tk.LEFT, padx=(6, 0))
        entry.bind("<Return>", lambda _event: self.schedule_single_refresh())
        entry.bind("<FocusOut>", lambda _event: self.schedule_single_refresh())

    def _build_single_tab(self) -> None:
        self.single_tab.columnconfigure(0, weight=1)
        self.single_tab.rowconfigure(1, weight=1)

        top_bar = ttk.Frame(self.single_tab)
        top_bar.grid(row=0, column=0, sticky="ew", padx=8, pady=8)
        top_bar.columnconfigure(1, weight=1)

        ttk.Label(top_bar, text="Frame").grid(row=0, column=0, sticky="w")
        self.frame_slider = tk.Scale(
            top_bar,
            from_=0,
            to=0,
            resolution=1,
            orient=tk.HORIZONTAL,
            variable=self.frame_index_var,
            command=lambda _value: self.schedule_single_refresh(),
        )
        self.frame_slider.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(top_bar, textvariable=self.frame_index_var, width=8).grid(row=0, column=2, sticky="e")

        body = ttk.Frame(self.single_tab)
        body.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.columnconfigure(2, weight=1)
        body.rowconfigure(0, weight=1)
        body.rowconfigure(1, weight=0)

        adjusted_box = ttk.LabelFrame(body, text="Transformed Image / ROI")
        adjusted_box.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        adjusted_box.rowconfigure(0, weight=1)
        adjusted_box.columnconfigure(0, weight=1)

        self.adjusted_canvas = tk.Canvas(adjusted_box, bg="#202020", highlightthickness=0)
        self.adjusted_canvas.grid(row=0, column=0, sticky="nsew")
        self.adjusted_canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.adjusted_canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.adjusted_canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

        mask_box = ttk.LabelFrame(body, text="Threshold Mask")
        mask_box.grid(row=0, column=1, sticky="nsew", padx=6)
        mask_box.rowconfigure(0, weight=1)
        mask_box.columnconfigure(0, weight=1)
        self.mask_label = ttk.Label(mask_box)
        self.mask_label.grid(row=0, column=0, sticky="nsew")

        overlay_box = ttk.LabelFrame(body, text="Blob Overlay")
        overlay_box.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        overlay_box.rowconfigure(0, weight=1)
        overlay_box.columnconfigure(0, weight=1)
        self.overlay_label = ttk.Label(overlay_box)
        self.overlay_label.grid(row=0, column=0, sticky="nsew")

        info_box = ttk.LabelFrame(body, text="Detection Status")
        info_box.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self.single_info_text = tk.Text(info_box, height=9, wrap=tk.WORD)
        self.single_info_text.grid(row=0, column=0, sticky="ew")
        self.single_info_text.configure(state=tk.DISABLED)

    def _build_validation_tab(self) -> None:
        self.validation_tab.columnconfigure(0, weight=1)
        self.validation_tab.rowconfigure(2, weight=1)

        button_bar = ttk.Frame(self.validation_tab)
        button_bar.grid(row=0, column=0, sticky="ew", padx=8, pady=8)
        button_bar.columnconfigure(3, weight=1)

        ttk.Button(button_bar, text="Run Frozen Validation", command=lambda: self.run_validation(False)).grid(
            row=0, column=0, padx=(0, 6)
        )
        ttk.Button(button_bar, text="Run Sweep Validation", command=lambda: self.run_validation(True)).grid(
            row=0, column=1, padx=(0, 6)
        )
        ttk.Button(button_bar, text="Show Failed Frames", command=self.focus_first_failed_frame).grid(
            row=0, column=2, padx=(0, 6)
        )
        ttk.Label(button_bar, textvariable=self.validation_progress_var).grid(row=0, column=3, sticky="w")

        preview_bar = ttk.Frame(self.validation_tab)
        preview_bar.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        preview_bar.columnconfigure(1, weight=1)

        ttk.Label(preview_bar, text="Preview Frame").grid(row=0, column=0, sticky="w")
        self.validation_slider = tk.Scale(
            preview_bar,
            from_=0,
            to=0,
            resolution=1,
            orient=tk.HORIZONTAL,
            variable=self.validation_frame_var,
            command=lambda _value: self.update_validation_preview(),
        )
        self.validation_slider.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(preview_bar, textvariable=self.validation_frame_var, width=8).grid(row=0, column=2, sticky="e")

        content = ttk.Frame(self.validation_tab)
        content.grid(row=2, column=0, sticky="nsew", padx=8, pady=(0, 8))
        content.columnconfigure(0, weight=0)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(0, weight=1)

        summary_box = ttk.LabelFrame(content, text="Summary")
        summary_box.grid(row=0, column=0, sticky="nsw", padx=(0, 8))
        summary_box.rowconfigure(0, weight=1)
        summary_box.columnconfigure(0, weight=1)
        self.validation_text = tk.Text(summary_box, width=54, wrap=tk.WORD)
        self.validation_text.grid(row=0, column=0, sticky="nsew")
        self.validation_text.configure(state=tk.DISABLED)

        preview_box = ttk.LabelFrame(content, text="Annotated Preview")
        preview_box.grid(row=0, column=1, sticky="nsew")
        preview_box.rowconfigure(0, weight=1)
        preview_box.columnconfigure(0, weight=1)
        self.validation_preview_label = ttk.Label(preview_box)
        self.validation_preview_label.grid(row=0, column=0, sticky="nsew")

    def choose_video(self) -> None:
        path = filedialog.askopenfilename(
            title="Open MJPEG / Video",
            initialdir=os.path.join(REPO_ROOT, "record"),
            filetypes=[
                ("Video files", "*.mjpeg *.avi *.mp4 *.mov *.mjpg"),
                ("All files", "*.*"),
            ],
        )
        if path:
            self.load_video(path)

    def clear_roi(self) -> None:
        self.expected_roi = None
        self.drag_start = None
        self.drag_current = None
        self.schedule_single_refresh()

    def load_video(self, path: str) -> None:
        try:
            self.status_var.set(f"Loading video: {path}")
            self.root.update_idletasks()
            self.video = load_video_frames(path)
        except Exception as exc:
            messagebox.showerror("Load video failed", str(exc))
            self.status_var.set("Failed to load video.")
            return

        self.frame_index_var.set(0)
        self.validation_frame_var.set(0)
        self.frame_slider.configure(to=max(self.video.frame_count - 1, 0))
        self.validation_slider.configure(to=max(self.video.frame_count - 1, 0))
        self.expected_roi = None
        self.drag_start = None
        self.drag_current = None
        self.validation_summary = None
        self.video_info_var.set(
            f"{os.path.basename(path)} | {self.video.frame_count} frames | {self.video.width}x{self.video.height}"
        )
        self.status_var.set("Video loaded. Drag on the transformed image to mark the expected target region.")
        self.schedule_single_refresh()
        self.update_validation_preview()
        self._set_text(self.validation_text, "Run frozen validation or sweep validation to see whole-video results.\n")

    def schedule_single_refresh(self) -> None:
        if self._refresh_single_job is not None:
            self.root.after_cancel(self._refresh_single_job)
        self._refresh_single_job = self.root.after(20, self.refresh_single_frame)

    def snapshot_simulation_params(self) -> SimulationParams:
        return SimulationParams(
            exposure_scale=float(self.exposure_scale_var.get()),
            r_gain=float(self.r_gain_var.get()),
            g_gain=float(self.g_gain_var.get()),
            b_gain=float(self.b_gain_var.get()),
            gamma=float(self.gamma_var.get()),
        )

    def snapshot_detector_params(self) -> DetectorParams:
        l_min = int(self.threshold_l_min_var.get())
        l_max = int(self.threshold_l_max_var.get())
        a_min = int(self.threshold_a_min_var.get())
        a_max = int(self.threshold_a_max_var.get())
        b_min = int(self.threshold_b_min_var.get())
        b_max = int(self.threshold_b_max_var.get())
        return DetectorParams(
            threshold_l_min=min(l_min, l_max),
            threshold_l_max=max(l_min, l_max),
            threshold_a_min=min(a_min, a_max),
            threshold_a_max=max(a_min, a_max),
            threshold_b_min=min(b_min, b_max),
            threshold_b_max=max(b_min, b_max),
            min_area=max(1, int(self.min_area_var.get())),
            max_area=max(0, int(self.max_area_var.get())),
            roundness_min_x1000=int(clamp(int(self.roundness_min_var.get()), 0, 1000)),
            merge_margin=max(0, int(self.merge_margin_var.get())),
            track_window_radius_px=max(1, int(self.track_window_radius_var.get())),
            center_filter_gain_x100=int(clamp(int(self.center_filter_gain_var.get()), 0, 100)),
            max_missed_frames=max(1, int(self.max_missed_frames_var.get())),
        )

    def snapshot_sweep_params(self) -> SweepParams:
        return SweepParams(
            exposure_min=float(self.sweep_exposure_min_var.get()),
            exposure_max=float(self.sweep_exposure_max_var.get()),
            exposure_step=float(self.sweep_exposure_step_var.get()),
            gain_min=float(self.sweep_gain_min_var.get()),
            gain_max=float(self.sweep_gain_max_var.get()),
            gain_step=float(self.sweep_gain_step_var.get()),
        )

    def current_frame(self) -> np.ndarray | None:
        if self.video is None or not self.video.frames:
            return None
        index = int(clamp(self.frame_index_var.get(), 0, self.video.frame_count - 1))
        self.frame_index_var.set(index)
        return self.video.frames[index]

    def refresh_single_frame(self) -> None:
        self._refresh_single_job = None
        frame = self.current_frame()
        if frame is None:
            self.adjusted_canvas.delete("all")
            self.mask_label.configure(image="")
            self.overlay_label.configure(image="")
            self._set_text(self.single_info_text, "Open a video to start tuning.\n")
            return

        sim_params = self.snapshot_simulation_params()
        detector_params = self.snapshot_detector_params()

        adjusted = apply_simulation(frame, sim_params)
        detector = HostGreenLightDetector(detector_params)
        self.single_frame_result = detector.detect_single_frame(adjusted, self.expected_roi)

        self.single_display_scale = scale_for_display(adjusted.shape[1], adjusted.shape[0])
        adjusted_display = resize_for_display(self.draw_adjusted_view(adjusted), self.single_display_scale)
        mask_display = resize_for_display(self.draw_mask_view(self.single_frame_result.mask), self.single_display_scale)
        overlay_display = resize_for_display(self.draw_overlay_view(adjusted, self.single_frame_result), self.single_display_scale)

        self._adjusted_photo = bgr_to_photo(adjusted_display)
        self._mask_photo = bgr_to_photo(mask_display)
        self._overlay_photo = bgr_to_photo(overlay_display)

        self.adjusted_canvas.configure(width=adjusted_display.shape[1], height=adjusted_display.shape[0])
        self.adjusted_canvas.delete("all")
        self.adjusted_canvas.create_image(0, 0, image=self._adjusted_photo, anchor="nw")

        roi_to_draw = self.expected_roi
        if self.drag_start is not None and self.drag_current is not None and self.video is not None:
            roi_to_draw = roi_from_drag(self.drag_start, self.drag_current, self.video.width, self.video.height)
        if roi_to_draw is not None:
            x, y, w, h = roi_to_draw
            scale = self.single_display_scale
            self.adjusted_canvas.create_rectangle(
                x * scale,
                y * scale,
                (x + w) * scale,
                (y + h) * scale,
                outline="#00ffff",
                width=2,
            )

        self.mask_label.configure(image=self._mask_photo)
        self.overlay_label.configure(image=self._overlay_photo)
        self._set_text(self.single_info_text, self.format_single_frame_info(self.single_frame_result))

    def draw_adjusted_view(self, adjusted_bgr: np.ndarray) -> np.ndarray:
        image = adjusted_bgr.copy()
        cv2.putText(
            image,
            f"frame={self.frame_index_var.get()}",
            (10, 24),
            CV_FONT,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image

    def draw_mask_view(self, mask: np.ndarray) -> np.ndarray:
        image = gray_to_bgr(mask)
        if self.expected_roi is not None:
            x, y, w, h = self.expected_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
        return image

    def draw_overlay_view(self, adjusted_bgr: np.ndarray, result: DetectionDebug) -> np.ndarray:
        image = adjusted_bgr.copy()
        if self.expected_roi is not None:
            x, y, w, h = self.expected_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)

        for candidate in result.candidates:
            color = (0, 0, 255)
            if candidate.passes_area and candidate.passes_roundness:
                color = (0, 200, 255)
            if result.best_candidate is candidate:
                color = (0, 255, 0) if result.locked else (0, 128, 255)
            cv2.drawContours(image, [candidate.contour], -1, color, 2)
            cv2.circle(image, (candidate.center_x, candidate.center_y), 3, color, -1)

        if result.search_roi is not None:
            x, y, w, h = result.search_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (180, 180, 180), 1)

        if result.raw_center is not None:
            center_x, center_y = result.raw_center
            cv2.circle(image, (center_x, center_y), max(result.radius_px, 4), (0, 255, 0), 2)
            cv2.putText(
                image,
                "LOCK" if result.locked else "MISS",
                (10, 24),
                CV_FONT,
                0.75,
                (0, 255, 0) if result.locked else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(image, "MISS", (10, 24), CV_FONT, 0.75, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(
            image,
            result.reason,
            (10, 48),
            CV_FONT,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image

    def format_single_frame_info(self, result: DetectionDebug) -> str:
        lines = [
            f"Frame: {self.frame_index_var.get()}",
            f"Locked: {'YES' if result.locked else 'NO'}",
            f"Detected candidate: {'YES' if result.detected else 'NO'}",
            f"Failure reason: {result.reason}",
            f"Background misdetect: {'YES' if result.background_misdetect else 'NO'}",
        ]
        if result.raw_center is not None:
            lines.append(f"Center: ({result.raw_center[0]}, {result.raw_center[1]})")
            lines.append(f"Area: {result.area}")
            lines.append(f"Radius: {result.radius_px}")
        else:
            lines.append("Center: -")
            lines.append("Area: -")
            lines.append("Radius: -")

        lines.append(f"Candidates after threshold: {len(result.candidates)}")
        if self.expected_roi is not None:
            rx, ry, rw, rh = self.expected_roi
            lines.append(f"Expected ROI: x={rx}, y={ry}, w={rw}, h={rh}")

        if result.candidates:
            lines.append("")
            lines.append("Top candidates:")
            ranked = sorted(result.candidates, key=lambda candidate: candidate.roundness_x1000, reverse=True)[:6]
            for index, candidate in enumerate(ranked, start=1):
                inside = point_in_roi((candidate.center_x, candidate.center_y), self.expected_roi)
                lines.append(
                    f"{index}. center=({candidate.center_x},{candidate.center_y}) "
                    f"area={candidate.area} roundness={candidate.roundness_x1000} "
                    f"inside_roi={'yes' if inside else 'no'}"
                )

        return "\n".join(lines) + "\n"

    def on_canvas_press(self, event: tk.Event) -> None:
        if self.video is None:
            return
        self.drag_start = self.canvas_to_image_coords(event.x, event.y)
        self.drag_current = self.drag_start
        self.schedule_single_refresh()

    def on_canvas_drag(self, event: tk.Event) -> None:
        if self.video is None or self.drag_start is None:
            return
        self.drag_current = self.canvas_to_image_coords(event.x, event.y)
        self.schedule_single_refresh()

    def on_canvas_release(self, event: tk.Event) -> None:
        if self.video is None or self.drag_start is None:
            return
        self.drag_current = self.canvas_to_image_coords(event.x, event.y)
        self.expected_roi = roi_from_drag(self.drag_start, self.drag_current, self.video.width, self.video.height)
        self.drag_start = None
        self.drag_current = None
        self.schedule_single_refresh()

    def canvas_to_image_coords(self, canvas_x: int, canvas_y: int) -> tuple[int, int]:
        if self.video is None:
            return (0, 0)
        scale = max(self.single_display_scale, 1e-6)
        image_x = int(clamp(round(canvas_x / scale), 0, max(self.video.width - 1, 0)))
        image_y = int(clamp(round(canvas_y / scale), 0, max(self.video.height - 1, 0)))
        return (image_x, image_y)

    def run_validation(self, use_sweep: bool) -> None:
        if self.video is None:
            messagebox.showwarning("No video", "Open a video before running validation.")
            return

        sim_params = self.snapshot_simulation_params()
        detector_params = self.snapshot_detector_params()
        sweep_params = self.snapshot_sweep_params()
        detector = HostGreenLightDetector(detector_params)

        frame_results: list[ValidationFrameResult] = []
        hit_frames = 0
        longest_miss_streak = 0
        current_miss_streak = 0
        first_miss_frame: int | None = None
        centers: list[tuple[int, int]] = []
        misdetected_frames: list[int] = []
        recognition_log: list[str] = []

        self.validation_progress_var.set("Running validation...")
        self.root.update_idletasks()

        for index, frame in enumerate(self.video.frames):
            if use_sweep:
                exposure_scale, gain_scale = sweep_params.values_for_frame(index)
            else:
                exposure_scale, gain_scale = (1.0, 1.0)

            adjusted = apply_simulation(
                frame,
                sim_params,
                dynamic_exposure_scale=exposure_scale,
                dynamic_gain_scale=gain_scale,
            )
            result = detector.process_video_frame(adjusted, self.expected_roi)

            if result.locked:
                hit_frames += 1
                current_miss_streak = 0
                if result.filtered_center is not None:
                    centers.append(result.filtered_center)
                recognition_log.append(
                    f"frame {index}: hit exp={exposure_scale:.2f} gain={gain_scale:.2f} "
                    f"center={result.filtered_center} area={result.area} radius={result.radius_px}"
                )
            else:
                current_miss_streak += 1
                if first_miss_frame is None:
                    first_miss_frame = index
                longest_miss_streak = max(longest_miss_streak, current_miss_streak)

            if result.background_misdetect:
                misdetected_frames.append(index)

            frame_results.append(
                ValidationFrameResult(
                    frame_index=index,
                    detected=result.detected,
                    locked=result.locked,
                    background_misdetect=result.background_misdetect,
                    reason=result.reason,
                    raw_center=result.raw_center,
                    filtered_center=result.filtered_center,
                    area=result.area,
                    radius_px=result.radius_px,
                    exposure_scale=exposure_scale,
                    gain_scale=gain_scale,
                )
            )

            if (index + 1) % 20 == 0 or index == (self.video.frame_count - 1):
                self.validation_progress_var.set(f"Running validation... {index + 1}/{self.video.frame_count}")
                self.root.update_idletasks()

        step_lengths = []
        for previous, current in zip(centers, centers[1:]):
            step_lengths.append(math.hypot(current[0] - previous[0], current[1] - previous[1]))

        jitter_mean = float(statistics.mean(step_lengths)) if step_lengths else 0.0
        jitter_std = float(statistics.pstdev(step_lengths)) if len(step_lengths) > 1 else 0.0
        jitter_max = float(max(step_lengths)) if step_lengths else 0.0

        total_frames = self.video.frame_count
        summary = ValidationSummary(
            total_frames=total_frames,
            hit_frames=hit_frames,
            hit_rate=(0.0 if total_frames == 0 else hit_frames / float(total_frames)),
            longest_miss_streak=max(longest_miss_streak, current_miss_streak),
            first_miss_frame=first_miss_frame,
            jitter_mean=jitter_mean,
            jitter_std=jitter_std,
            jitter_max=jitter_max,
            misdetected_frames=misdetected_frames,
            recognition_log=recognition_log,
            frames=frame_results,
        )
        self.validation_summary = summary

        self.validation_slider.configure(to=max(total_frames - 1, 0))
        self.validation_frame_var.set(0)
        self.update_validation_preview()
        self._set_text(self.validation_text, self.format_validation_summary(summary, use_sweep))
        mode_text = "Sweep validation completed." if use_sweep else "Frozen validation completed."
        self.validation_progress_var.set(mode_text)
        self.status_var.set(mode_text)

    def format_validation_summary(self, summary: ValidationSummary, use_sweep: bool) -> str:
        lines = [
            f"Mode: {'Sweep Validation' if use_sweep else 'Frozen Validation'}",
            f"Total frames: {summary.total_frames}",
            f"Hit frames: {summary.hit_frames}",
            f"Hit rate: {summary.hit_rate * 100.0:.2f}%",
            f"Longest miss streak: {summary.longest_miss_streak}",
            f"First miss frame: {summary.first_miss_frame if summary.first_miss_frame is not None else '-'}",
            f"Center jitter mean: {summary.jitter_mean:.2f}px",
            f"Center jitter std: {summary.jitter_std:.2f}px",
            f"Center jitter max: {summary.jitter_max:.2f}px",
        ]

        if self.expected_roi is not None:
            lines.append(f"Background misdetect frames: {len(summary.misdetected_frames)}")
            if summary.misdetected_frames:
                preview = ", ".join(str(index) for index in summary.misdetected_frames[:50])
                if len(summary.misdetected_frames) > 50:
                    preview += ", ..."
                lines.append(f"Misdetected frame indices: {preview}")
        else:
            lines.append("Background misdetect frames: N/A (set an ROI to enable this check)")

        lines.append("")
        lines.append("Recognition log:")
        if summary.recognition_log:
            lines.extend(summary.recognition_log[:200])
            if len(summary.recognition_log) > 200:
                lines.append("... truncated ...")
        else:
            lines.append("No successful lock frames.")
        return "\n".join(lines) + "\n"

    def update_validation_preview(self) -> None:
        if self.video is None or self.validation_summary is None or not self.validation_summary.frames:
            self.validation_preview_label.configure(image="")
            return

        index = int(clamp(self.validation_frame_var.get(), 0, len(self.validation_summary.frames) - 1))
        self.validation_frame_var.set(index)
        frame_result = self.validation_summary.frames[index]
        frame = self.video.frames[index]
        preview = apply_simulation(
            frame,
            self.snapshot_simulation_params(),
            dynamic_exposure_scale=frame_result.exposure_scale,
            dynamic_gain_scale=frame_result.gain_scale,
        )
        preview = self.draw_validation_preview(preview, frame_result)
        display_scale = scale_for_display(preview.shape[1], preview.shape[0])
        preview_display = resize_for_display(preview, display_scale)
        self._validation_photo = bgr_to_photo(preview_display)
        self.validation_preview_label.configure(image=self._validation_photo)

    def draw_validation_preview(self, frame_bgr: np.ndarray, frame_result: ValidationFrameResult) -> np.ndarray:
        image = frame_bgr.copy()
        if self.expected_roi is not None:
            x, y, w, h = self.expected_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)

        if frame_result.raw_center is not None:
            radius = max(frame_result.radius_px, 4)
            color = (0, 255, 0) if frame_result.locked else (0, 128, 255)
            cv2.circle(image, frame_result.raw_center, radius, color, 2)
            cv2.circle(image, frame_result.raw_center, 3, color, -1)

        status = "LOCK" if frame_result.locked else "MISS"
        cv2.putText(image, f"frame={frame_result.frame_index} {status}", (10, 24), CV_FONT, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(
            image,
            f"exp={frame_result.exposure_scale:.2f} gain={frame_result.gain_scale:.2f}",
            (10, 48),
            CV_FONT,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(image, frame_result.reason, (10, 72), CV_FONT, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        if frame_result.filtered_center is not None:
            cv2.putText(
                image,
                f"center={frame_result.filtered_center} area={frame_result.area} radius={frame_result.radius_px}",
                (10, 96),
                CV_FONT,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return image

    def focus_first_failed_frame(self) -> None:
        if self.validation_summary is None or not self.validation_summary.frames:
            return
        for frame_result in self.validation_summary.frames:
            if not frame_result.locked:
                self.validation_frame_var.set(frame_result.frame_index)
                self.frame_index_var.set(frame_result.frame_index)
                self.update_validation_preview()
                self.schedule_single_refresh()
                self.notebook.select(self.single_tab)
                return
        messagebox.showinfo("Validation", "No failed frames in the current validation result.")

    def copy_yaml_snippet(self) -> None:
        params = self.snapshot_detector_params()
        snippet = "\n".join(
            [
                "green_light:",
                f"  openmv_threshold_l_min: {params.threshold_l_min}",
                f"  openmv_threshold_l_max: {params.threshold_l_max}",
                f"  openmv_threshold_a_min: {params.threshold_a_min}",
                f"  openmv_threshold_a_max: {params.threshold_a_max}",
                f"  openmv_threshold_b_min: {params.threshold_b_min}",
                f"  openmv_threshold_b_max: {params.threshold_b_max}",
                f"  openmv_min_area: {params.min_area}",
                f"  openmv_max_area: {params.max_area}",
                f"  openmv_roundness_min_x1000: {params.roundness_min_x1000}",
                f"  openmv_merge_margin: {params.merge_margin}",
                f"  openmv_track_window_radius_px: {params.track_window_radius_px}",
                f"  openmv_center_filter_gain_x100: {params.center_filter_gain_x100}",
                f"  openmv_max_missed_frames: {params.max_missed_frames}",
                "",
            ]
        )
        self.root.clipboard_clear()
        self.root.clipboard_append(snippet)
        self.status_var.set("Current detector parameters copied to clipboard as a YAML snippet.")

    @staticmethod
    def _set_text(widget: tk.Text, text: str) -> None:
        widget.configure(state=tk.NORMAL)
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, text)
        widget.configure(state=tk.DISABLED)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tune OpenMV green-light detection on recorded video.")
    parser.add_argument(
        "--video",
        default="",
        help="path to the MJPEG/video file; if omitted, choose one in the UI",
    )
    parser.add_argument(
        "--config",
        default=GuidanceHostPaths.default_config_path(),
        help="path to the guidance parameter file for detector defaults",
    )
    return parser


def main(argv: list[str]) -> int:
    args = build_arg_parser().parse_args(argv)
    root = tk.Tk()
    app = VideoTunerApp(root, args.video, args.config)
    del app
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
