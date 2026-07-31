import time
import ustruct


USB_FRAME_MAGIC = b"OMVJ"
USB_FRAME_HEADER_FORMAT = ">IHHI"
USB_METADATA_MAGIC = b"OMVM"
USB_METADATA_HEADER_FORMAT = ">I"
USB_PREVIEW_START_REQUEST = b"OMVP"
USB_PREVIEW_REQUEST_READ_BYTES = 64

OVERLAY_X = 2
OVERLAY_SCALE = 2
OVERLAY_CHAR_W = 8
OVERLAY_CHAR_H = 10
OVERLAY_LINE_H = (OVERLAY_CHAR_H * OVERLAY_SCALE) + 4
OVERLAY_STATUS_Y = 2
OVERLAY_TARGET_Y = OVERLAY_STATUS_Y + OVERLAY_LINE_H
OVERLAY_ROI_Y = OVERLAY_TARGET_Y + OVERLAY_LINE_H
COLOR_OK = (0, 255, 0)
COLOR_ERROR = (255, 0, 0)
COLOR_TEXT = (255, 255, 255)
COLOR_BOX = (0, 255, 0)
COLOR_ROI = (255, 255, 0)
COLOR_BG = (0, 0, 0)


def target_search_status(result, debug, debug_detector=True, detector_available=True):
    if not debug_detector:
        return "detector_disabled"
    if not detector_available:
        return "detector_unavailable"

    debug = debug or {}
    status = debug.get("target_search", "")
    if status:
        return status
    if result is None:
        return "target_not_found"
    selected_scan = debug.get("selected_scan", "")
    if selected_scan == "roi" or debug.get("target_in_search_roi", False):
        return "search_roi"
    return "full_frame_search"


def make_frame_metadata(result,
                        debug,
                        image_width,
                        image_height,
                        now_ms,
                        frame_index,
                        fps=None,
                        debug_detector=True,
                        detector_available=True):
    metadata = {
        "frame_index": int(frame_index),
        "target_search": target_search_status(result, debug, debug_detector, detector_available),
        "detected": result is not None,
        "image_width": int(image_width),
        "image_height": int(image_height),
        "ticks_ms": int(now_ms),
        "debug_detector": bool(debug_detector),
        "detector_available": bool(detector_available),
    }
    if fps is not None:
        metadata["fps_x100"] = int(float(fps) * 100.0)

    debug = debug or {}
    for key in (
        "selected_scan",
        "roi_active",
        "target_in_search_roi",
        "search_roi",
        "fallback_used",
        "fallback_reason",
        "full_target_selected",
    ):
        if key in debug:
            metadata[key] = debug[key]

    if result is not None:
        metadata["center_x"] = int(result["center_x"])
        metadata["center_y"] = int(result["center_y"])
        metadata["area"] = int(result["area"])
        metadata["radius"] = int(result.get("radius", 0))
    return metadata


def payload_length(payload):
    try:
        return len(payload)
    except TypeError:
        return payload.size()


def json_escape(value):
    text = str(value)
    text = text.replace(chr(92), chr(92) + chr(92))
    return text.replace(chr(34), chr(92) + chr(34))


def json_value(value):
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
            parts.append(json_value(item))
        return "[" + ",".join(parts) + "]"
    if isinstance(value, dict):
        parts = []
        for key, item in value.items():
            parts.append("\"%s\":%s" % (json_escape(key), json_value(item)))
        return "{" + ",".join(parts) + "}"
    return "\"%s\"" % json_escape(value)


def try_draw(callable_obj, *args, **kwargs):
    try:
        callable_obj(*args, **kwargs)
        return True
    except Exception:
        return False


def draw_rectangle(img, x, y, w, h, color, thickness=1, fill=False):
    if w < 1 or h < 1:
        return False
    rect = (int(x), int(y), int(w), int(h))
    if try_draw(img.draw_rectangle, rect, color=color, thickness=thickness, fill=fill):
        return True
    if try_draw(img.draw_rectangle, rect[0], rect[1], rect[2], rect[3], color=color, thickness=thickness, fill=fill):
        return True
    return try_draw(img.draw_rectangle, rect[0], rect[1], rect[2], rect[3], color, thickness, fill)


def draw_circle(img, x, y, radius, color, thickness=2):
    r = int(radius)
    if r < 1:
        r = 1
    cx = int(x)
    cy = int(y)
    # OpenMV 5.0: draw_circle((cx, cy, r), ...)
    if try_draw(img.draw_circle, (cx, cy, r), color=color, thickness=thickness):
        return True
    if try_draw(img.draw_circle, [cx, cy, r], color=color, thickness=thickness):
        return True
    if try_draw(img.draw_circle, (cx, cy, r), color=color):
        return True
    # OpenMV 4.x fallback
    if try_draw(img.draw_circle, cx, cy, r, color=color, thickness=thickness):
        return True
    return try_draw(img.draw_circle, cx, cy, r, color=color)


def draw_cross(img, x, y, color, size=6, thickness=1):
    cx = int(x)
    cy = int(y)
    s = int(size)
    line = (cx - s, cy, cx + s, cy)
    if not try_draw(img.draw_line, line, color=color, thickness=thickness):
        try_draw(img.draw_line, cx - s, cy, cx + s, cy, color=color, thickness=thickness)
    line = (cx, cy - s, cx, cy + s)
    if not try_draw(img.draw_line, line, color=color, thickness=thickness):
        try_draw(img.draw_line, cx, cy - s, cx, cy + s, color=color, thickness=thickness)


def draw_string(img, x, y, text, color, scale=1):
    text = str(text)
    if scale < 1:
        scale = 1
    # OpenMV 5.0: draw_string((x, y), text, ...)
    if try_draw(img.draw_string, (int(x), int(y)), text, color=color, scale=scale):
        return True
    if try_draw(img.draw_string, (int(x), int(y)), text, color=color):
        return True
    # OpenMV 4.x: draw_string(x, y, text, ...)
    if try_draw(img.draw_string, int(x), int(y), text, color=color, scale=scale):
        return True
    return try_draw(img.draw_string, int(x), int(y), text, color=color)


def draw_text(img, y, text, color=COLOR_TEXT):
    scale = OVERLAY_SCALE
    if scale < 1:
        scale = 1
    draw_string(img, OVERLAY_X, y, str(text), color, scale=scale)


def draw_stream_status(img, fps, color):
    draw_text(img, OVERLAY_STATUS_Y, "work_fps=%0.1f" % float(fps), color)


def search_roi_text(debug):
    debug = debug or {}
    selected = debug.get("selected_scan", "")
    roi = debug.get("search_roi", None)
    roi_active = bool(debug.get("roi_active", False))

    if selected == "roi":
        if roi is not None:
            return "search_roi=on %s" % (roi,)
        return "search_roi=on"
    if roi_active:
        if debug.get("fallback_used", False):
            reason = debug.get("fallback_reason", "unknown")
            if roi is not None:
                return "search_roi=fallback:%s %s" % (reason, roi)
            return "search_roi=fallback:%s" % reason
        if roi is not None:
            return "search_roi=active %s" % (roi,)
        return "search_roi=active"
    return "search_roi=off"


def draw_target_circle(img, x, y, radius, color):
    r = int(radius)
    if r < 4:
        r = 4
    draw_circle(img, x, y, r, color, thickness=2)
    draw_circle(img, x, y, r + 1, color, thickness=1)
    draw_cross(img, x, y, COLOR_TEXT, size=max(4, r // 3), thickness=1)


def draw_preview_overlay(img, result, fps, debug):
    draw_stream_status(img, fps, COLOR_TEXT)

    if result is None:
        draw_text(img, OVERLAY_TARGET_Y, "target x=-1 y=-1", COLOR_ERROR)
        draw_text(img, OVERLAY_ROI_Y, search_roi_text(debug), COLOR_TEXT)
        return

    x = int(result["center_x"])
    y = int(result["center_y"])
    radius = int(result.get("radius", 1))
    area = int(result["area"])
    draw_target_circle(img, x, y, radius, COLOR_BOX)
    draw_text(img, OVERLAY_TARGET_Y, "target x=%d y=%d a=%d" % (x, y, area), COLOR_OK)
    draw_text(img, OVERLAY_ROI_Y, search_roi_text(debug), COLOR_TEXT)


def encode_jpeg(img, quality):
    try:
        jpeg = img.compress(quality=quality)
        if jpeg is not None:
            return jpeg
    except Exception:
        pass
    try:
        return img.compressed(quality=quality)
    except Exception:
        return None


def jpeg_payload(jpeg):
    if isinstance(jpeg, (bytes, bytearray)):
        return jpeg
    for name in ("bytearray", "to_bytes"):
        method = getattr(jpeg, name, None)
        if callable(method):
            try:
                data = method()
                if data is not None:
                    return data
            except Exception:
                pass
    try:
        return bytes(jpeg)
    except Exception:
        return jpeg


def write_usb_preview_frame(vcp, img, quality, metadata=None):
    jpeg = encode_jpeg(img, quality)
    if jpeg is None:
        return False
    payload = jpeg_payload(jpeg)
    if payload is None:
        return False
    header = ustruct.pack(
        USB_FRAME_HEADER_FORMAT,
        payload_length(payload),
        img.width(),
        img.height(),
        time.ticks_ms() & 0xFFFFFFFF,
    )
    try:
        vcp.write(USB_FRAME_MAGIC)
        vcp.write(header)
        vcp.write(payload)
        if metadata is not None:
            meta_payload = json_value(metadata).encode("utf-8")
            vcp.write(USB_METADATA_MAGIC)
            vcp.write(ustruct.pack(USB_METADATA_HEADER_FORMAT, payload_length(meta_payload)))
            vcp.write(meta_payload)
    except Exception:
        return False
    return True


def usb_cable_connected(vcp):
    try:
        return bool(vcp.isconnected())
    except Exception:
        return False


def drain_usb_input(vcp):
    try:
        pending = vcp.any()
    except Exception:
        return
    if not pending:
        return
    try:
        vcp.read(min(int(pending), USB_PREVIEW_REQUEST_READ_BYTES))
    except Exception:
        pass


def poll_usb_preview_request(vcp, current_active):
    try:
        pending = vcp.any()
    except Exception:
        return current_active
    if not pending:
        return current_active
    try:
        data = vcp.read(min(int(pending), USB_PREVIEW_REQUEST_READ_BYTES))
    except Exception:
        return current_active
    if data and USB_PREVIEW_START_REQUEST in data:
        return True
    return current_active
