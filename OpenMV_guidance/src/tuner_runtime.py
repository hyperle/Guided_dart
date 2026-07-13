import os
import ubinascii

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


def _load_image_bytes(payload):
    try:
        return image.Image(payload)
    except Exception:
        pass
    try:
        return image.Image(bytearray(payload))
    except Exception:
        raise RuntimeError("RAM JPEG decode failed")


def _roi_or_empty(roi):
    if roi is None:
        return (-1, -1, -1, -1)
    return roi


def _apply_params(detector, params):
    for key in detector.params:
        if key in params:
            detector.params[key] = int(params[key])


def _evaluate_with_detector(detector, img, expected_roi, reset_tracking):
    if reset_tracking:
        detector.reset_tracking()

    result = detector.process_frame(img, collect_debug_blobs=True)
    debug = detector.last_debug or {}
    candidates = _annotate_candidates(detector, img, debug.get("debug_blobs", []))
    raw_center = debug.get("raw_center")
    filtered_center = debug.get("filtered_center")
    detected = result is not None

    best_target = None
    if detected and raw_center is not None:
        best_target = {
            "center_x": raw_center[0],
            "center_y": raw_center[1],
            "source": debug.get("source", ""),
        }

    reason, background_misdetect = _choose_failure_reason(candidates, detector.params, expected_roi, best_target)
    locked = detected and not background_misdetect

    return {
        "candidates": candidates,
        "search_roi": debug.get("search_roi"),
        "roi_active": bool(debug.get("roi_active", False)),
        "roi_target_found": bool(debug.get("roi_target_found", False)),
        "fallback_used": bool(debug.get("fallback_used", False)),
        "fallback_reason": debug.get("fallback_reason", ""),
        "full_target_found": bool(debug.get("full_target_found", False)),
        "full_target_selected": bool(debug.get("full_target_selected", False)),
        "selected_scan": debug.get("selected_scan", ""),
        "target_in_search_roi": bool(debug.get("target_in_search_roi", False)),
        "tracking_lost": bool(debug.get("tracking_lost", False)),
        "missed_frames_before": int(debug.get("missed_frames_before", 0)),
        "missed_frames_after": int(debug.get("missed_frames_after", 0)),
        "reason": reason,
        "detected": detected,
        "locked": locked,
        "background_misdetect": background_misdetect,
        "raw_center": raw_center,
        "filtered_center": filtered_center,
        "area": int(debug.get("area", 0)),
        "radius_px": int(debug.get("radius", 0)),
        "source": debug.get("source", ""),
    }


def evaluate_single_frame(path, params, expected_roi=None):
    detector = GreenLightDetector()
    _apply_params(detector, params)
    img = _load_image(path)
    return _evaluate_with_detector(detector, img, expected_roi, True)


def evaluate_single_frame_bytes(payload, params, expected_roi=None):
    detector = GreenLightDetector()
    _apply_params(detector, params)
    img = _load_image_bytes(payload)
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


def _print_frame_result(frame_index, name, result):
    raw_center = result["raw_center"] if result["raw_center"] is not None else (-1, -1)
    filtered_center = result["filtered_center"] if result["filtered_center"] is not None else (-1, -1)
    roi = _roi_or_empty(result["search_roi"])
    print(
        "frame|%d|%s|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%s|%d|%d|%d|%d|%d|%s|%s" % (
            frame_index,
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
            roi[0],
            roi[1],
            roi[2],
            roi[3],
            1 if result["roi_active"] else 0,
            1 if result["roi_target_found"] else 0,
            1 if result["target_in_search_roi"] else 0,
            result["selected_scan"],
            1 if result["full_target_found"] else 0,
            1 if result["full_target_selected"] else 0,
            1 if result["tracking_lost"] else 0,
            result["missed_frames_before"],
            result["missed_frames_after"],
            result["fallback_reason"],
            result["reason"],
        )
    )


def print_sequence_results(directory, params, expected_roi=None):
    detector = GreenLightDetector()
    _apply_params(detector, params)
    names = _frame_file_names(directory)
    print("total|%d" % len(names))
    for index in range(len(names)):
        name = names[index]
        path = directory + "/" + name
        result = _evaluate_with_detector(detector, _load_image(path), expected_roi, False)
        _print_frame_result(index, name, result)


def print_sequence_chunk_results(directory, names, params, expected_roi=None, start_index=0):
    detector = _sequence_detector(params)
    print("total|%d" % len(names))
    for offset in range(len(names)):
        name = names[offset]
        path = directory + "/" + name
        result = _evaluate_with_detector(detector, _load_image(path), expected_roi, False)
        _print_frame_result(start_index + offset, name, result)


def print_sequence_frame_bytes(frame_index, name, payload, params, expected_roi=None):
    detector = _sequence_detector(params)
    print("total|1")
    result = _evaluate_with_detector(detector, _load_image_bytes(payload), expected_roi, False)
    _print_frame_result(frame_index, name, result)


def print_sequence_frame_hex(frame_index, name, payload_hex, params, expected_roi=None):
    print_sequence_frame_bytes(frame_index, name, ubinascii.unhexlify(payload_hex), params, expected_roi)


def detect_storage_root():
    try:
        os.stat("/flash")
        return "/flash"
    except OSError:
        return "."


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
