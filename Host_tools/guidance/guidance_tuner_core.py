from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from guidance_host_common import YamlLiteParser


DISPLAY_MAX_WIDTH = 480
DISPLAY_MAX_HEIGHT = 360
CV_FONT = cv2.FONT_HERSHEY_SIMPLEX
LOCAL_PREVIEW_NOTE = "Mask/preview is host-side; lock verdict comes from OpenMV."


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
    contour: np.ndarray | None
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
class ValidationRunConfig:
    sim_params: SimulationParams
    detector_params: DetectorParams
    sweep_params: SweepParams
    expected_roi: tuple[int, int, int, int] | None
    use_sweep: bool


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
    run_config: ValidationRunConfig | None = None


def clamp(value: int | float, minimum: int | float, maximum: int | float) -> int | float:
    return max(minimum, min(maximum, value))


def point_in_roi(point: tuple[int, int] | None, roi: tuple[int, int, int, int] | None) -> bool:
    if point is None or roi is None:
        return False
    x, y = point
    rx, ry, rw, rh = roi
    return rx <= x < (rx + rw) and ry <= y < (ry + rh)


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
    return min(1.0, DISPLAY_MAX_WIDTH / float(width), DISPLAY_MAX_HEIGHT / float(height))


def resize_for_display(image: np.ndarray, scale: float) -> np.ndarray:
    if abs(scale - 1.0) < 1e-6:
        return image.copy()
    interpolation = cv2.INTER_NEAREST if image.ndim == 2 else cv2.INTER_LINEAR
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=interpolation)


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
    return mask.astype(np.uint8) * 255


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
