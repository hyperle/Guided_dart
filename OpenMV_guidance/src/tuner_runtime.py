import os

import image

from green_light_detector import GreenLightDetector


_SEQUENCE_DETECTOR = None


def _point_in_roi(point, roi):
    if point is None or roi is None:
        return False
    x, y = point
    rx, ry, rw, rh = roi
    return (x >= rx) and (x < (rx + rw)) and (y >= ry) and (y < (ry + rh))


def _choose_failure_reason(candidates, params, expected_roi, best_target):
    if best_target is not None:
        if expected_roi is not None and not _point_in_roi((best_target["center_x"], best_target["center_y"]), expected_roi):
            return "误检背景", True
        if best_target["source"] == "ring":
            return "绿环锁定", False
        return "锁定成功", False

    if not candidates:
        return "颜色不过阈值", False

    inside_candidates = []
    for candidate in candidates:
        if expected_roi is None or _point_in_roi((candidate["center_x"], candidate["center_y"]), expected_roi):
            inside_candidates.append(candidate)

    relevant = inside_candidates if inside_candidates else candidates

    if expected_roi is not None and not inside_candidates:
        for candidate in candidates:
            if candidate["passes_area"] and candidate["passes_roundness"]:
                return "误检背景", True

    if relevant:
        if True:
            all_small = True
            all_large = True
            for candidate in relevant:
                if candidate["area"] >= params["min_area"]:
                    all_small = False
                if not (params["max_area"] > 0 and candidate["area"] > params["max_area"]):
                    all_large = False
            if all_small:
                return "面积太小", False
            if params["max_area"] > 0 and all_large:
                return "面积太大", False

            any_area_pass = False
            any_round_pass = False
            for candidate in relevant:
                if candidate["passes_area"]:
                    any_area_pass = True
                    if candidate["passes_roundness"]:
                        any_round_pass = True
            if any_area_pass and not any_round_pass:
                return "圆度不够", False

    return "未找到有效 blob", False


def _annotate_candidates(detector, img, blobs):
    candidates = []
    for blob in blobs:
        metrics = detector._blob_metrics(blob)
        outer_roundness_x1000 = int(blob.roundness() * 1000)
        passes_area = detector._blob_passes_area(blob)
        solid_target = detector._solid_target_from_blob(blob, img, metrics)
        ring_target = detector._ring_target_from_blob(img, blob, metrics)
        target = solid_target
        if target is None or (ring_target is not None and detector._target_score(ring_target) > detector._target_score(target)):
            target = ring_target
        if target is not None and target["source"] == "ring":
            roundness_x1000 = target["roundness_x1000"]
            passes_roundness = roundness_x1000 >= detector.params["ring_min_roundness_x1000"]
        else:
            roundness_x1000 = outer_roundness_x1000
            passes_roundness = roundness_x1000 >= detector.params["roundness_min_x1000"]
        candidates.append(
            {
                "rect": metrics["rect"],
                "center_x": target["center_x"] if target is not None else blob.cx(),
                "center_y": target["center_y"] if target is not None else blob.cy(),
                "area": metrics["area"],
                "roundness_x1000": roundness_x1000,
                "radius_px": target["radius"] if target is not None else metrics["radius"],
                "passes_area": passes_area,
                "passes_roundness": passes_roundness,
                "passes_ring": ring_target is not None,
                "source": target["source"] if target is not None else "",
                "green_fill_x100": metrics["green_fill_x100"],
                "center_white_x100": ring_target["center_white_x100"] if ring_target is not None else 0,
            }
        )
    return candidates


def _load_image(path):
    img = image.Image(path)
    return img


def _apply_params(detector, params):
    for key in detector.params:
        if key in params:
            detector.params[key] = int(params[key])


def _evaluate_with_detector(detector, img, expected_roi, reset_tracking):
    if reset_tracking:
        detector.reset_tracking()

    threshold = detector._active_threshold()
    roi = detector._resolve_roi(img.width(), img.height())
    blobs = detector._scan_blobs(img, threshold, roi)
    candidates = _annotate_candidates(detector, img, blobs)
    best_target = detector._find_best_target(img, blobs)
    fallback_used = False

    if detector._full_scan_due(roi, best_target):
        fallback_used = True
        full_blobs = detector._scan_blobs(img, threshold, None)
        full_candidates = _annotate_candidates(detector, img, full_blobs)
        full_target = detector._find_best_target(img, full_blobs)
        if full_target is not None:
            if best_target is None or detector._target_score(full_target) > (detector._target_score(best_target) + 250):
                blobs = full_blobs
                candidates = full_candidates
                best_target = full_target

    reason, background_misdetect = _choose_failure_reason(candidates, detector.params, expected_roi, best_target)

    if best_target is None:
        detector._update_track(None)
        return {
            "candidates": candidates,
            "search_roi": roi,
            "fallback_used": fallback_used,
            "reason": reason,
            "detected": False,
            "locked": False,
            "background_misdetect": background_misdetect,
            "raw_center": None,
            "filtered_center": None,
            "area": 0,
            "radius_px": 0,
            "source": "",
        }

    raw_center = (best_target["center_x"], best_target["center_y"])
    filtered_center = detector._update_track(raw_center)
    detector._last_radius = best_target["radius"]
    detector._last_area = best_target["area"]
    return {
        "candidates": candidates,
        "search_roi": roi,
        "fallback_used": fallback_used,
        "reason": reason,
        "detected": True,
        "locked": not background_misdetect,
        "background_misdetect": background_misdetect,
        "raw_center": raw_center,
        "filtered_center": filtered_center,
        "area": best_target["area"],
        "radius_px": best_target["radius"],
        "source": best_target["source"],
    }


def evaluate_single_frame(path, params, expected_roi=None):
    detector = GreenLightDetector()
    _apply_params(detector, params)
    img = _load_image(path)
    return _evaluate_with_detector(detector, img, expected_roi, True)


def reset_sequence_state(params):
    global _SEQUENCE_DETECTOR
    detector = GreenLightDetector()
    _apply_params(detector, params)
    detector.reset_tracking()
    _SEQUENCE_DETECTOR = detector


def clear_sequence_state():
    global _SEQUENCE_DETECTOR
    _SEQUENCE_DETECTOR = None


def _sequence_detector(params):
    global _SEQUENCE_DETECTOR
    if _SEQUENCE_DETECTOR is None:
        reset_sequence_state(params)
    elif params:
        _apply_params(_SEQUENCE_DETECTOR, params)
    return _SEQUENCE_DETECTOR


def _frame_file_names(directory):
    names = []
    try:
        entries = os.listdir(directory)
    except OSError:
        return names

    for name in entries:
        lower_name = name.lower()
        if lower_name.endswith(".jpg") or lower_name.endswith(".jpeg") or lower_name.endswith(".png"):
            names.append(name)
    names.sort()
    return names


def print_sequence_results(directory, params, expected_roi=None):
    detector = GreenLightDetector()
    _apply_params(detector, params)
    names = _frame_file_names(directory)
    print("total|%d" % len(names))
    for index in range(len(names)):
        name = names[index]
        path = directory + "/" + name
        result = _evaluate_with_detector(detector, _load_image(path), expected_roi, False)
        raw_center = result["raw_center"] if result["raw_center"] is not None else (-1, -1)
        filtered_center = result["filtered_center"] if result["filtered_center"] is not None else (-1, -1)
        print(
            "frame|%d|%s|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s" % (
                index,
                name,
                1 if result["detected"] else 0,
                1 if result["locked"] else 0,
                1 if result["background_misdetect"] else 0,
                raw_center[0],
                raw_center[1],
                filtered_center[0],
                filtered_center[1],
                result["area"],
                result["radius_px"],
                1 if result["fallback_used"] else 0,
                result["reason"],
            )
        )


def print_sequence_chunk_results(directory, names, params, expected_roi=None, start_index=0):
    detector = _sequence_detector(params)
    print("total|%d" % len(names))
    for offset in range(len(names)):
        name = names[offset]
        path = directory + "/" + name
        result = _evaluate_with_detector(detector, _load_image(path), expected_roi, False)
        raw_center = result["raw_center"] if result["raw_center"] is not None else (-1, -1)
        filtered_center = result["filtered_center"] if result["filtered_center"] is not None else (-1, -1)
        print(
            "frame|%d|%s|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s" % (
                start_index + offset,
                name,
                1 if result["detected"] else 0,
                1 if result["locked"] else 0,
                1 if result["background_misdetect"] else 0,
                raw_center[0],
                raw_center[1],
                filtered_center[0],
                filtered_center[1],
                result["area"],
                result["radius_px"],
                1 if result["fallback_used"] else 0,
                result["reason"],
            )
        )


def detect_storage_root():
    for candidate in ("/sdcard", "/sd", "/flash"):
        try:
            os.stat(candidate)
            return candidate
        except OSError:
            pass
    return ""


def ensure_clean_dir(path):
    try:
        entries = os.listdir(path)
        for name in entries:
            try:
                os.remove(path + "/" + name)
            except OSError:
                pass
        return
    except OSError:
        pass

    try:
        os.mkdir(path)
    except OSError:
        pass

    try:
        entries = os.listdir(path)
        for name in entries:
            try:
                os.remove(path + "/" + name)
            except OSError:
                pass
    except OSError:
        pass
