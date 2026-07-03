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


def _blob_to_dict(blob):
    return {
        "rect": blob.rect(),
        "center_x": blob.cx(),
        "center_y": blob.cy(),
        "area": blob.area(),
        "roundness_x1000": int(blob.roundness() * 1000),
        "radius_px": int(blob.w() / 2),
    }


def _choose_failure_reason(candidates, params, expected_roi, best_blob):
    if best_blob is not None:
        if expected_roi is not None and not _point_in_roi((best_blob.cx(), best_blob.cy()), expected_roi):
            return "误检背景", True
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


def _annotate_candidates(blobs, params):
    candidates = []
    for blob in blobs:
        area = blob.area()
        roundness_x1000 = int(blob.roundness() * 1000)
        passes_area = area >= params["min_area"]
        if params["max_area"] > 0 and area > params["max_area"]:
            passes_area = False
        candidates.append(
            {
                "rect": blob.rect(),
                "center_x": blob.cx(),
                "center_y": blob.cy(),
                "area": area,
                "roundness_x1000": roundness_x1000,
                "radius_px": int(blob.w() / 2),
                "passes_area": passes_area,
                "passes_roundness": roundness_x1000 >= params["roundness_min_x1000"],
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
        detector._last_center = None
        detector._missed_frames = 0

    roi = detector._resolve_roi(img.width(), img.height())
    if roi is None:
        blobs = img.find_blobs(
            [detector.threshold_tuple()],
            pixels_threshold=max(detector.params["min_area"], 1),
            merge=True,
            margin=detector.params["merge_margin"],
        )
    else:
        blobs = img.find_blobs(
            [detector.threshold_tuple()],
            roi=roi,
            pixels_threshold=max(detector.params["min_area"], 1),
            merge=True,
            margin=detector.params["merge_margin"],
        )

    candidates = _annotate_candidates(blobs, detector.params)
    best_blob = detector._find_best_blob(blobs)
    fallback_used = False

    if best_blob is None and roi is not None:
        fallback_used = True
        blobs = img.find_blobs(
            [detector.threshold_tuple()],
            pixels_threshold=max(detector.params["min_area"], 1),
            merge=True,
            margin=detector.params["merge_margin"],
        )
        candidates = _annotate_candidates(blobs, detector.params)
        best_blob = detector._find_best_blob(blobs)

    reason, background_misdetect = _choose_failure_reason(candidates, detector.params, expected_roi, best_blob)

    if best_blob is None:
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
        }

    raw_center = (best_blob.cx(), best_blob.cy())
    filtered_center = detector._update_track(raw_center)
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
        "area": best_blob.area(),
        "radius_px": int(best_blob.w() / 2),
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
    detector._last_center = None
    detector._missed_frames = 0
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
            "frame|%d|%s|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s" % (
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
            "frame|%d|%s|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s" % (
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
