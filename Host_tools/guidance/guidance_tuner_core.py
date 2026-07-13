from __future__ import annotations

from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from openmv_detector_params import load_openmv_detector_param_dict


DISPLAY_MAX_WIDTH = 480
DISPLAY_MAX_HEIGHT = 360
CV_FONT = cv2.FONT_HERSHEY_SIMPLEX
LOCAL_PREVIEW_NOTE = "Host mask/preview update locally. Single-frame OpenMV probes are stateless; use sequence replay or validation for real tracking ROI feedback."


@dataclass
class SimulationParams:
    exposure_scale: float = 1.0
    r_gain: float = 1.0
    g_gain: float = 1.0
    b_gain: float = 1.0
    gamma: float = 1.0


@dataclass
class DetectorParams:
    threshold_l_min: int
    threshold_l_max: int
    threshold_a_min: int
    threshold_a_max: int
    threshold_b_min: int
    threshold_b_max: int
    min_area: int
    max_area: int
    roundness_min_x1000: int
    merge_margin: int
    track_window_radius_px: int
    center_filter_gain_x100: int
    max_missed_frames: int
    ring_detection_enabled: int
    ring_min_roundness_x1000: int
    ring_min_aspect_x100: int
    ring_min_fill_x100: int
    ring_max_fill_x100: int
    ring_min_center_white_x100: int
    ring_center_sample_ratio_x100: int
    ring_center_min_brightness: int
    ring_center_max_channel_delta: int
    ring_min_outer_diameter_px: int


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
    source: str = ""
    passes_ring: bool = False
    green_fill_x100: int = 0
    center_white_x100: int = 0


@dataclass
class ColorSample:
    center_x: int
    center_y: int
    radius_px: int
    pixel_count: int
    rgb_mean: tuple[float, float, float]
    rgb_min: tuple[int, int, int]
    rgb_max: tuple[int, int, int]
    lab_mean: tuple[float, float, float]
    lab_min: tuple[float, int, int]
    lab_max: tuple[float, int, int]


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
    source: str = ""
    fallback_reason: str = ""
    roi_active: bool = False
    roi_target_found: bool = False
    target_in_search_roi: bool = False
    selected_scan: str = ""
    full_target_found: bool = False
    full_target_selected: bool = False
    tracking_lost: bool = False
    missed_frames_before: int = 0
    missed_frames_after: int = 0


@dataclass
class ValidationFrameResult:
    frame_index: int
    detected: bool
    locked: bool
    background_misdetect: bool
    fallback_used: bool
    reason: str
    raw_center: tuple[int, int] | None
    filtered_center: tuple[int, int] | None
    area: int
    radius_px: int
    exposure_scale: float
    gain_scale: float
    search_roi: tuple[int, int, int, int] | None = None
    fallback_reason: str = ""
    roi_active: bool = False
    roi_target_found: bool = False
    target_in_search_roi: bool = False
    selected_scan: str = ""
    full_target_found: bool = False
    full_target_selected: bool = False
    tracking_lost: bool = False
    missed_frames_before: int = 0
    missed_frames_after: int = 0


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


def _roundness_x1000(contour: np.ndarray) -> int:
    area = float(cv2.contourArea(contour))
    perimeter = float(cv2.arcLength(contour, True))
    if area <= 0.0 or perimeter <= 1e-6:
        return 0
    roundness = 4.0 * math.pi * area / (perimeter * perimeter)
    return int(clamp(round(roundness * 1000.0), 0, 1000))


def _merge_mask_for_margin(mask: np.ndarray, merge_margin: int) -> np.ndarray:
    margin = max(0, int(merge_margin))
    if margin <= 0:
        return mask
    kernel_size = margin * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    return cv2.dilate(mask, kernel, iterations=1)


def _host_blob_candidates(
    mask: np.ndarray,
    params: DetectorParams,
    expected_roi: tuple[int, int, int, int] | None,
) -> list[BlobCandidate]:
    if mask.size == 0:
        return []

    merged_mask = _merge_mask_for_margin(mask, params.merge_margin)
    contours, _hierarchy = cv2.findContours(merged_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[BlobCandidate] = []

    for merged_contour in contours:
        x, y, w, h = cv2.boundingRect(merged_contour)
        if w <= 0 or h <= 0:
            continue

        original_crop = mask[y : y + h, x : x + w]
        ys, xs = np.nonzero(original_crop)
        if len(xs) == 0:
            continue

        area = int(len(xs))
        original_x = int(x + xs.min())
        original_y = int(y + ys.min())
        original_w = int(xs.max() - xs.min() + 1)
        original_h = int(ys.max() - ys.min() + 1)

        moments = cv2.moments(original_crop, binaryImage=True)
        if abs(moments["m00"]) > 1e-6:
            center_x = int(round(x + moments["m10"] / moments["m00"]))
            center_y = int(round(y + moments["m01"] / moments["m00"]))
        else:
            center_x = original_x + original_w // 2
            center_y = original_y + original_h // 2

        passes_area = area >= params.min_area
        if params.max_area > 0 and area > params.max_area:
            passes_area = False

        roundness_x1000 = _roundness_x1000(merged_contour)
        passes_roundness = roundness_x1000 >= params.roundness_min_x1000
        radius_px = max(1, int(round(max(original_w, original_h) / 2.0)))
        center = (center_x, center_y)
        candidates.append(
            BlobCandidate(
                contour=merged_contour,
                area=area,
                roundness_x1000=roundness_x1000,
                center_x=center_x,
                center_y=center_y,
                radius_px=radius_px,
                bbox=(original_x, original_y, original_w, original_h),
                passes_area=passes_area,
                passes_roundness=passes_roundness,
                overlaps_expected_roi=point_in_roi(center, expected_roi),
                source="solid" if passes_area and passes_roundness else "",
                green_fill_x100=100,
            )
        )

    candidates.sort(key=lambda candidate: (candidate.roundness_x1000, candidate.area), reverse=True)
    return candidates


def _choose_best_candidate(candidates: list[BlobCandidate]) -> BlobCandidate | None:
    for candidate in candidates:
        if candidate.passes_area and candidate.passes_roundness:
            return candidate
    return None


def _choose_host_failure_reason(
    candidates: list[BlobCandidate],
    params: DetectorParams,
    expected_roi: tuple[int, int, int, int] | None,
    best_candidate: BlobCandidate | None,
) -> tuple[str, bool]:
    if best_candidate is not None:
        if expected_roi is not None and not point_in_roi((best_candidate.center_x, best_candidate.center_y), expected_roi):
            return "误检背景", True
        return "锁定成功", False

    if not candidates:
        return "颜色不过阈值", False

    inside_candidates = [
        candidate
        for candidate in candidates
        if expected_roi is None or point_in_roi((candidate.center_x, candidate.center_y), expected_roi)
    ]
    relevant = inside_candidates if inside_candidates else candidates

    if expected_roi is not None and not inside_candidates:
        for candidate in candidates:
            if candidate.passes_area and candidate.passes_roundness:
                return "误检背景", True

    all_small = all(candidate.area < params.min_area for candidate in relevant)
    all_large = bool(params.max_area > 0) and all(candidate.area > params.max_area for candidate in relevant)
    if all_small:
        return "面积太小", False
    if all_large:
        return "面积太大", False

    any_area_pass = any(candidate.passes_area for candidate in relevant)
    any_roundness_pass = any(candidate.passes_area and candidate.passes_roundness for candidate in relevant)
    if any_area_pass and not any_roundness_pass:
        return "圆度不够", False

    return "未找到有效 blob", False


def evaluate_host_frame(
    frame_bgr: np.ndarray,
    params: DetectorParams,
    expected_roi: tuple[int, int, int, int] | None = None,
    mask: np.ndarray | None = None,
) -> DetectionDebug:
    preview_mask = threshold_lab_mask(frame_bgr, params) if mask is None else mask
    candidates = _host_blob_candidates(preview_mask, params, expected_roi)
    best_candidate = _choose_best_candidate(candidates)
    reason, background_misdetect = _choose_host_failure_reason(candidates, params, expected_roi, best_candidate)

    raw_center = None
    area = 0
    radius_px = 0
    if best_candidate is not None:
        raw_center = (best_candidate.center_x, best_candidate.center_y)
        area = best_candidate.area
        radius_px = best_candidate.radius_px

    locked = best_candidate is not None and not background_misdetect
    return DetectionDebug(
        candidates=candidates,
        best_candidate=best_candidate,
        mask=preview_mask,
        search_roi=None,
        fallback_used=False,
        reason=reason,
        detected=best_candidate is not None,
        locked=locked,
        background_misdetect=background_misdetect,
        raw_center=raw_center,
        filtered_center=raw_center,
        area=area,
        radius_px=radius_px,
        source=best_candidate.source if best_candidate is not None else "",
        selected_scan="host",
    )


def sample_color_circle(frame_bgr: np.ndarray, center: tuple[int, int] | None, radius_px: int) -> ColorSample | None:
    if center is None or frame_bgr.size == 0:
        return None

    height, width = frame_bgr.shape[:2]
    center_x = int(clamp(center[0], 0, max(width - 1, 0)))
    center_y = int(clamp(center[1], 0, max(height - 1, 0)))
    radius = max(0, int(radius_px))

    circle_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(circle_mask, (center_x, center_y), radius, 255, -1)
    selected = frame_bgr[circle_mask > 0]
    if selected.size == 0:
        return None

    rgb_pixels = selected[:, ::-1]
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    lab_pixels = lab[circle_mask > 0]

    l_values = lab_pixels[:, 0].astype(np.float32) * (100.0 / 255.0)
    a_values = lab_pixels[:, 1].astype(np.int16) - 128
    b_values = lab_pixels[:, 2].astype(np.int16) - 128

    return ColorSample(
        center_x=center_x,
        center_y=center_y,
        radius_px=radius,
        pixel_count=int(selected.shape[0]),
        rgb_mean=tuple(float(value) for value in rgb_pixels.mean(axis=0)),
        rgb_min=tuple(int(value) for value in rgb_pixels.min(axis=0)),
        rgb_max=tuple(int(value) for value in rgb_pixels.max(axis=0)),
        lab_mean=(float(l_values.mean()), float(a_values.mean()), float(b_values.mean())),
        lab_min=(float(l_values.min()), int(a_values.min()), int(b_values.min())),
        lab_max=(float(l_values.max()), int(a_values.max()), int(b_values.max())),
    )


def load_detector_defaults(config_path: str) -> DetectorParams:
    params = load_openmv_detector_param_dict(config_path)
    detector_fields = DetectorParams.__dataclass_fields__
    return DetectorParams(**{key: params[key] for key in detector_fields})
