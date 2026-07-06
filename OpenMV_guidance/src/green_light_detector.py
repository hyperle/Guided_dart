FRAME_HEADER = (0xA5, 0x5A)
MEASUREMENT_HEADER = 0x5A
MEASUREMENT_LENGTH = 0x0A
PARAM_TYPE_SET = 0x10
PARAM_TYPE_ACK = 0x11

STATUS_APPLIED = 0x00
STATUS_UNKNOWN_KEY = 0x01
STATUS_INVALID_VALUE = 0x02
STATUS_INVALID_LENGTH = 0x03

from generated_guidance_params import OPENMV_DETECTOR_PARAMS

KEY_DETECTOR_MIN_GREEN_CHANNEL = 0x10
KEY_DETECTOR_MIN_GREEN_DOMINANCE = 0x11
KEY_DETECTOR_MIN_BLOB_PIXELS = 0x12
KEY_DETECTOR_MAX_BLOB_PIXELS = 0x13
KEY_DETECTOR_SMALL_BLOB_PIXELS = 0x14
KEY_DETECTOR_MIN_FILL_RATIO_SMALL_X100 = 0x15
KEY_DETECTOR_MIN_FILL_RATIO_LARGE_X100 = 0x16
KEY_DETECTOR_TRACK_WINDOW_RADIUS_PX = 0x17
KEY_DETECTOR_CENTER_FILTER_GAIN_X100 = 0x18
KEY_DETECTOR_MAX_MISSED_FRAMES = 0x19
KEY_OPENMV_THRESHOLD_L_MIN = 0x30
KEY_OPENMV_THRESHOLD_L_MAX = 0x31
KEY_OPENMV_THRESHOLD_A_MIN = 0x32
KEY_OPENMV_THRESHOLD_A_MAX = 0x33
KEY_OPENMV_THRESHOLD_B_MIN = 0x34
KEY_OPENMV_THRESHOLD_B_MAX = 0x35
KEY_OPENMV_MIN_AREA = 0x36
KEY_OPENMV_MAX_AREA = 0x37
KEY_OPENMV_ROUNDNESS_MIN_X1000 = 0x38
KEY_OPENMV_MERGE_MARGIN = 0x39
KEY_OPENMV_TRACK_WINDOW_RADIUS_PX = 0x3A
KEY_OPENMV_CENTER_FILTER_GAIN_X100 = 0x3B
KEY_OPENMV_MAX_MISSED_FRAMES = 0x3C
KEY_OPENMV_RING_DETECTION_ENABLED = 0x3D
KEY_OPENMV_RING_MIN_ROUNDNESS_X1000 = 0x3E
KEY_OPENMV_RING_MIN_ASPECT_X100 = 0x3F
KEY_OPENMV_RING_MIN_FILL_X100 = 0x40
KEY_OPENMV_RING_MAX_FILL_X100 = 0x41
KEY_OPENMV_RING_MIN_CENTER_WHITE_X100 = 0x42
KEY_OPENMV_RING_CENTER_SAMPLE_RATIO_X100 = 0x43
KEY_OPENMV_RING_CENTER_MIN_BRIGHTNESS = 0x44
KEY_OPENMV_RING_CENTER_MAX_CHANNEL_DELTA = 0x45
KEY_OPENMV_RING_MIN_OUTER_DIAMETER_PX = 0x46
KEY_OPENMV_THRESHOLD_SMALL_L_MIN = 0x47
KEY_OPENMV_THRESHOLD_SMALL_L_MAX = 0x48
KEY_OPENMV_THRESHOLD_SMALL_A_MIN = 0x49
KEY_OPENMV_THRESHOLD_SMALL_A_MAX = 0x4A
KEY_OPENMV_THRESHOLD_SMALL_B_MIN = 0x4B
KEY_OPENMV_THRESHOLD_SMALL_B_MAX = 0x4C
KEY_OPENMV_THRESHOLD_SMALL_AREA_MAX = 0x4D

_EDGE_SAMPLE_OFFSETS_X100 = (
    (100, 0),
    (71, 71),
    (0, 100),
    (-71, 71),
    (-100, 0),
    (-71, -71),
    (0, -100),
    (71, -71),
)

_RING_SAMPLE_OFFSETS_X100 = (
    (100, 0),
    (92, 38),
    (71, 71),
    (38, 92),
    (0, 100),
    (-38, 92),
    (-71, 71),
    (-92, 38),
    (-100, 0),
    (-92, -38),
    (-71, -71),
    (-38, -92),
    (0, -100),
    (38, -92),
    (71, -71),
    (92, -38),
)

_SOLID_SAMPLE_OFFSETS_X100 = (
    (0, 0),
    (40, 0),
    (-40, 0),
    (0, 40),
    (0, -40),
    (80, 0),
    (-80, 0),
    (0, 80),
    (0, -80),
    (40, 40),
    (-40, 40),
    (40, -40),
    (-40, -40),
)

_ROI_MIN_HALF_SIZE_PX = 28
_ROI_RADIUS_SCALE_X100 = 400
_ROI_RADIUS_PAD_PX = 12
_BLOB_X_STRIDE = 2
_BLOB_Y_STRIDE = 1
_RING_INNER_MIN_HITS = 6
_RING_INNER_MIN_RADIUS_X100 = 20
_RING_INNER_MAX_RADIUS_X100 = 85
_RING_TARGET_RINGS_X100 = (70, 90)
_BACKGROUND_INNER_PAD_MIN_PX = 4
_BACKGROUND_OUTER_PAD_MIN_PX = 8
_MIN_BACKGROUND_SAMPLES = 6
_EMISSION_MODE_SOLID = 0
_EMISSION_MODE_RING = 1
_MIN_SOLID_GREEN_SAMPLES_X100 = 35
_MIN_RING_GREEN_SAMPLES_X100 = 35
_HARD_MIN_GREEN_SAMPLES_X100 = 20
_RING_SOLID_LIKE_FILL_X100 = 70
_RGB_GREEN_MIN = 24
_RGB_GREEN_DOMINANCE = 8
_EMIT_HARD_MIN_PEAK_GREEN = 35
_EMIT_MIN_PEAK_GREEN = 45
_EMIT_MIN_GREEN_DELTA = 8
_EMIT_MIN_LUMA_DELTA = 6
_EMIT_HARD_MIN_EXCESS_AVG = 4
_EMIT_MIN_EXCESS_AVG = 6
_EMIT_MIN_EXCESS_DELTA = 5


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def checksum_bytes(values):
    total = 0
    for value in values:
        total = (total + value) & 0xFF
    return total


def u32_to_i32(value):
    if value & 0x80000000:
        return -((~value + 1) & 0xFFFFFFFF)
    return value


def pixel_channels(pixel):
    if isinstance(pixel, tuple) or isinstance(pixel, list):
        if len(pixel) >= 3:
            return int(pixel[0]), int(pixel[1]), int(pixel[2])
        if len(pixel) >= 1:
            value = int(pixel[0])
            return value, value, value
    value = int(pixel)
    if value > 255:
        red = ((value >> 11) & 0x1F) * 255 // 31
        green = ((value >> 5) & 0x3F) * 255 // 63
        blue = (value & 0x1F) * 255 // 31
        return red, green, blue
    return value, value, value


class GreenLightDetector:
    def __init__(self):
        self.params = dict(OPENMV_DETECTOR_PARAMS)
        self._param_rx_state = 0
        self._param_rx_buffer = bytearray()
        self._last_center = None
        self._last_radius = None
        self._missed_frames = 0
        self._last_area = None

    def threshold_tuple(self):
        return (
            self.params["threshold_l_min"],
            self.params["threshold_l_max"],
            self.params["threshold_a_min"],
            self.params["threshold_a_max"],
            self.params["threshold_b_min"],
            self.params["threshold_b_max"],
        )

    def threshold_tuple_small(self):
        return (
            self.params["threshold_small_l_min"],
            self.params["threshold_small_l_max"],
            self.params["threshold_small_a_min"],
            self.params["threshold_small_a_max"],
            self.params["threshold_small_b_min"],
            self.params["threshold_small_b_max"],
        )

    def _active_threshold(self):
        if self._last_area is not None and self._last_area < self.params["threshold_small_area_max"]:
            return self.threshold_tuple_small()
        return self.threshold_tuple()

    def _resolve_roi(self, image_width, image_height):
        if self._last_center is None:
            return None

        if self._last_radius is None:
            half = self.params["track_window_radius_px"]
        else:
            half = ((self._last_radius * _ROI_RADIUS_SCALE_X100) // 100) + _ROI_RADIUS_PAD_PX
            half = clamp(half, _ROI_MIN_HALF_SIZE_PX, self.params["track_window_radius_px"])
        if self._missed_frames > 0:
            half = min(self.params["track_window_radius_px"], half + (self._missed_frames * 16))

        center_x, center_y = self._last_center
        start_x = clamp(center_x - half, 0, image_width - 1)
        start_y = clamp(center_y - half, 0, image_height - 1)
        end_x = clamp(center_x + half, 0, image_width - 1)
        end_y = clamp(center_y + half, 0, image_height - 1)
        return (start_x, start_y, end_x - start_x + 1, end_y - start_y + 1)

    def reset_tracking(self):
        self._last_center = None
        self._last_radius = None
        self._last_area = None
        self._missed_frames = 0

    def send_measurement(self, uart, x, y, area, image_width=0, image_height=0):
        packet = bytearray()
        packet.append(MEASUREMENT_HEADER)
        packet.append(MEASUREMENT_LENGTH)
        packet.append((x >> 8) & 0xFF)
        packet.append(x & 0xFF)
        packet.append((y >> 8) & 0xFF)
        packet.append(y & 0xFF)
        packet.append((area >> 8) & 0xFF)
        packet.append(area & 0xFF)
        packet.append((image_width >> 8) & 0xFF)
        packet.append(image_width & 0xFF)
        packet.append((image_height >> 8) & 0xFF)
        packet.append(image_height & 0xFF)
        packet.append(checksum_bytes(packet))
        uart.write(packet)

    def send_param_ack(self, uart, key, status, value):
        frame = bytearray()
        frame.append(FRAME_HEADER[0])
        frame.append(FRAME_HEADER[1])
        frame.append(PARAM_TYPE_ACK)
        frame.append(key)
        frame.append(status)
        frame.append(value & 0xFF)
        frame.append((value >> 8) & 0xFF)
        frame.append((value >> 16) & 0xFF)
        frame.append((value >> 24) & 0xFF)
        frame.append(checksum_bytes(frame))
        uart.write(frame)

    def apply_param(self, key, value_u32):
        signed_value = u32_to_i32(value_u32)

        if key == KEY_OPENMV_THRESHOLD_L_MIN or key == KEY_DETECTOR_MIN_GREEN_CHANNEL:
            self.params["threshold_l_min"] = clamp(signed_value, 0, 100)
            return STATUS_APPLIED, self.params["threshold_l_min"]
        if key == KEY_OPENMV_THRESHOLD_L_MAX:
            self.params["threshold_l_max"] = clamp(signed_value, 0, 100)
            return STATUS_APPLIED, self.params["threshold_l_max"]
        if key == KEY_OPENMV_THRESHOLD_A_MIN or key == KEY_DETECTOR_MIN_GREEN_DOMINANCE:
            self.params["threshold_a_min"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_a_min"] & 0xFFFFFFFF
        if key == KEY_OPENMV_THRESHOLD_A_MAX:
            self.params["threshold_a_max"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_a_max"] & 0xFFFFFFFF
        if key == KEY_OPENMV_THRESHOLD_B_MIN:
            self.params["threshold_b_min"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_b_min"] & 0xFFFFFFFF
        if key == KEY_OPENMV_THRESHOLD_B_MAX:
            self.params["threshold_b_max"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_b_max"] & 0xFFFFFFFF
        if key == KEY_OPENMV_MIN_AREA or key == KEY_DETECTOR_MIN_BLOB_PIXELS:
            self.params["min_area"] = clamp(value_u32, 1, 20000)
            return STATUS_APPLIED, self.params["min_area"]
        if key == KEY_OPENMV_MAX_AREA or key == KEY_DETECTOR_MAX_BLOB_PIXELS:
            self.params["max_area"] = clamp(value_u32, 0, 50000)
            return STATUS_APPLIED, self.params["max_area"]
        if key == KEY_OPENMV_ROUNDNESS_MIN_X1000:
            self.params["roundness_min_x1000"] = clamp(value_u32, 0, 1000)
            return STATUS_APPLIED, self.params["roundness_min_x1000"]
        if key == KEY_OPENMV_MERGE_MARGIN:
            self.params["merge_margin"] = clamp(value_u32, 0, 50)
            return STATUS_APPLIED, self.params["merge_margin"]
        if key == KEY_OPENMV_TRACK_WINDOW_RADIUS_PX or key == KEY_DETECTOR_TRACK_WINDOW_RADIUS_PX:
            self.params["track_window_radius_px"] = clamp(value_u32, 1, 200)
            return STATUS_APPLIED, self.params["track_window_radius_px"]
        if key == KEY_OPENMV_CENTER_FILTER_GAIN_X100 or key == KEY_DETECTOR_CENTER_FILTER_GAIN_X100:
            self.params["center_filter_gain_x100"] = clamp(value_u32, 0, 100)
            return STATUS_APPLIED, self.params["center_filter_gain_x100"]
        if key == KEY_OPENMV_MAX_MISSED_FRAMES or key == KEY_DETECTOR_MAX_MISSED_FRAMES:
            self.params["max_missed_frames"] = clamp(value_u32, 1, 255)
            return STATUS_APPLIED, self.params["max_missed_frames"]
        if key == KEY_OPENMV_RING_DETECTION_ENABLED:
            self.params["ring_detection_enabled"] = 1 if value_u32 != 0 else 0
            return STATUS_APPLIED, self.params["ring_detection_enabled"]
        if key == KEY_OPENMV_RING_MIN_ROUNDNESS_X1000:
            self.params["ring_min_roundness_x1000"] = clamp(value_u32, 0, 1000)
            return STATUS_APPLIED, self.params["ring_min_roundness_x1000"]
        if key == KEY_OPENMV_RING_MIN_ASPECT_X100:
            self.params["ring_min_aspect_x100"] = clamp(value_u32, 1, 100)
            return STATUS_APPLIED, self.params["ring_min_aspect_x100"]
        if key == KEY_OPENMV_RING_MIN_FILL_X100:
            self.params["ring_min_fill_x100"] = clamp(value_u32, 0, 100)
            return STATUS_APPLIED, self.params["ring_min_fill_x100"]
        if key == KEY_OPENMV_RING_MAX_FILL_X100:
            self.params["ring_max_fill_x100"] = clamp(value_u32, 0, 100)
            return STATUS_APPLIED, self.params["ring_max_fill_x100"]
        if key == KEY_OPENMV_RING_MIN_CENTER_WHITE_X100:
            self.params["ring_min_center_white_x100"] = clamp(value_u32, 0, 100)
            return STATUS_APPLIED, self.params["ring_min_center_white_x100"]
        if key == KEY_OPENMV_RING_CENTER_SAMPLE_RATIO_X100:
            self.params["ring_center_sample_ratio_x100"] = clamp(value_u32, 5, 100)
            return STATUS_APPLIED, self.params["ring_center_sample_ratio_x100"]
        if key == KEY_OPENMV_RING_CENTER_MIN_BRIGHTNESS:
            self.params["ring_center_min_brightness"] = clamp(value_u32, 0, 255)
            return STATUS_APPLIED, self.params["ring_center_min_brightness"]
        if key == KEY_OPENMV_RING_CENTER_MAX_CHANNEL_DELTA:
            self.params["ring_center_max_channel_delta"] = clamp(value_u32, 0, 255)
            return STATUS_APPLIED, self.params["ring_center_max_channel_delta"]
        if key == KEY_OPENMV_RING_MIN_OUTER_DIAMETER_PX:
            self.params["ring_min_outer_diameter_px"] = clamp(value_u32, 1, 240)
            return STATUS_APPLIED, self.params["ring_min_outer_diameter_px"]

        if key == KEY_OPENMV_THRESHOLD_SMALL_L_MIN:
            self.params["threshold_small_l_min"] = clamp(signed_value, 0, 100)
            return STATUS_APPLIED, self.params["threshold_small_l_min"]
        if key == KEY_OPENMV_THRESHOLD_SMALL_L_MAX:
            self.params["threshold_small_l_max"] = clamp(signed_value, 0, 100)
            return STATUS_APPLIED, self.params["threshold_small_l_max"]
        if key == KEY_OPENMV_THRESHOLD_SMALL_A_MIN:
            self.params["threshold_small_a_min"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_small_a_min"] & 0xFFFFFFFF
        if key == KEY_OPENMV_THRESHOLD_SMALL_A_MAX:
            self.params["threshold_small_a_max"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_small_a_max"] & 0xFFFFFFFF
        if key == KEY_OPENMV_THRESHOLD_SMALL_B_MIN:
            self.params["threshold_small_b_min"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_small_b_min"] & 0xFFFFFFFF
        if key == KEY_OPENMV_THRESHOLD_SMALL_B_MAX:
            self.params["threshold_small_b_max"] = clamp(signed_value, -128, 127)
            return STATUS_APPLIED, self.params["threshold_small_b_max"] & 0xFFFFFFFF
        if key == KEY_OPENMV_THRESHOLD_SMALL_AREA_MAX:
            self.params["threshold_small_area_max"] = clamp(value_u32, 1, 50000)
            return STATUS_APPLIED, self.params["threshold_small_area_max"]

        return STATUS_UNKNOWN_KEY, 0

    def _process_param_byte(self, uart, byte_value):
        if self._param_rx_state == 0:
            if byte_value == FRAME_HEADER[0]:
                self._param_rx_buffer = bytearray([byte_value])
                self._param_rx_state = 1
            return

        if self._param_rx_state == 1:
            if byte_value == FRAME_HEADER[1]:
                self._param_rx_buffer.append(byte_value)
                self._param_rx_state = 2
            elif byte_value == FRAME_HEADER[0]:
                self._param_rx_buffer = bytearray([byte_value])
            else:
                self._param_rx_state = 0
                self._param_rx_buffer = bytearray()
            return

        if self._param_rx_state == 2:
            if byte_value != PARAM_TYPE_SET:
                self._param_rx_state = 0
                self._param_rx_buffer = bytearray()
                return
            self._param_rx_buffer.append(byte_value)
            self._param_rx_state = 3
            return

        self._param_rx_buffer.append(byte_value)
        if len(self._param_rx_buffer) < 9:
            return

        key = self._param_rx_buffer[3]
        checksum = self._param_rx_buffer[8]
        if checksum_bytes(self._param_rx_buffer[:8]) != checksum:
            self.send_param_ack(uart, key, STATUS_INVALID_LENGTH, 0)
        else:
            value_u32 = (self._param_rx_buffer[4] |
                         (self._param_rx_buffer[5] << 8) |
                         (self._param_rx_buffer[6] << 16) |
                         (self._param_rx_buffer[7] << 24))
            status, ack_value = self.apply_param(key, value_u32)
            self.send_param_ack(uart, key, status, ack_value & 0xFFFFFFFF)

        self._param_rx_state = 0
        self._param_rx_buffer = bytearray()

    def poll_params(self, uart):
        while uart.any() > 0:
            self._process_param_byte(uart, uart.readchar())

    def _area_passes(self, area):
        if area < self.params["min_area"]:
            return False
        if self.params["max_area"] > 0 and area > self.params["max_area"]:
            return False
        return True

    def _blob_passes_area(self, blob):
        return self._area_passes(blob.area())

    def _blob_pixels(self, blob):
        try:
            return blob.pixels()
        except AttributeError:
            return blob.area()

    def _blob_metrics(self, blob):
        rect = blob.rect()
        x, y, width, height = rect
        area = blob.area()
        pixels = self._blob_pixels(blob)
        outer_diameter = min(width, height)
        max_side = max(width, height)
        aspect_x100 = 0
        if max_side > 0:
            aspect_x100 = (outer_diameter * 100) // max_side
        return {
            "blob": blob,
            "rect": rect,
            "x": x,
            "y": y,
            "width": width,
            "height": height,
            "area": area,
            "pixels": pixels,
            "outer_diameter": outer_diameter,
            "max_side": max_side,
            "aspect_x100": aspect_x100,
            "green_fill_x100": (pixels * 100) // max(area, 1),
            "bbox_center_x": x + (width // 2),
            "bbox_center_y": y + (height // 2),
            "radius": max(1, int(outer_diameter / 2)),
            "roundness_x1000": -1,
        }

    def _metrics_pass_fast_filter(self, metrics):
        if not self._area_passes(metrics["area"]):
            return False
        if metrics["outer_diameter"] <= 0 or metrics["max_side"] <= 0:
            return False
        if metrics["green_fill_x100"] < self.params["ring_min_fill_x100"]:
            return False
        if metrics["outer_diameter"] >= self.params["ring_min_outer_diameter_px"]:
            if metrics["aspect_x100"] < self.params["ring_min_aspect_x100"]:
                return False
        return True

    def _roundness_x1000(self, blob, metrics=None):
        if metrics is not None and metrics["roundness_x1000"] >= 0:
            return metrics["roundness_x1000"]
        roundness_x1000 = int(blob.roundness() * 1000)
        if metrics is not None:
            metrics["roundness_x1000"] = roundness_x1000
        return roundness_x1000

    def _scaled_offset(self, value_x100, radius):
        if value_x100 >= 0:
            return (value_x100 * radius + 50) // 100
        return -(((-value_x100) * radius + 50) // 100)

    def _green_sample_values(self, pixel):
        red, green, blue = pixel_channels(pixel)
        max_rb = red if red > blue else blue
        green_excess = green - max_rb
        if green_excess < 0:
            green_excess = 0
        luma = (red + (green * 2) + blue) // 4
        is_green = green >= _RGB_GREEN_MIN and green_excess >= _RGB_GREEN_DOMINANCE
        max_channel = max(red, green, blue)
        min_channel = min(red, green, blue)
        min_brightness = clamp(self.params["ring_center_min_brightness"], 0, 255)
        max_delta = clamp(self.params["ring_center_max_channel_delta"], 0, 255)
        white_like = max_channel >= min_brightness and (max_channel - min_channel) <= max_delta
        return is_green, green, luma, green_excess, max_channel, white_like

    def _new_sample_stats(self):
        return {
            "total": 0,
            "green_count": 0,
            "white_count": 0,
            "green_sum": 0,
            "luma_sum": 0,
            "excess_sum": 0,
            "green_peak": 0,
            "luma_peak": 0,
            "excess_peak": 0,
        }

    def _add_pixel_to_stats(self, stats, pixel):
        is_green, green, luma, green_excess, _, white_like = self._green_sample_values(pixel)
        if is_green:
            stats["green_count"] += 1
        if white_like:
            stats["white_count"] += 1
        stats["green_sum"] += green
        stats["luma_sum"] += luma
        stats["excess_sum"] += green_excess
        if green > stats["green_peak"]:
            stats["green_peak"] = green
        if luma > stats["luma_peak"]:
            stats["luma_peak"] = luma
        if green_excess > stats["excess_peak"]:
            stats["excess_peak"] = green_excess
        stats["total"] += 1

    def _finish_sample_stats(self, stats):
        total = stats["total"]
        if total <= 0:
            return None
        return {
            "total": total,
            "green_ratio_x100": (stats["green_count"] * 100) // total,
            "white_ratio_x100": (stats["white_count"] * 100) // total,
            "green_avg": stats["green_sum"] // total,
            "luma_avg": stats["luma_sum"] // total,
            "excess_avg": stats["excess_sum"] // total,
            "green_peak": stats["green_peak"],
            "luma_peak": stats["luma_peak"],
            "excess_peak": stats["excess_peak"],
        }

    def _sample_offset_stats_into(self, img, stats, center_x, center_y, sample_radius, offsets_x100, clamp_edges):
        image_width = img.width()
        image_height = img.height()
        for dx100, dy100 in offsets_x100:
            x = center_x + self._scaled_offset(dx100, sample_radius)
            y = center_y + self._scaled_offset(dy100, sample_radius)
            if clamp_edges:
                x = clamp(x, 0, image_width - 1)
                y = clamp(y, 0, image_height - 1)
            elif x < 0 or x >= image_width or y < 0 or y >= image_height:
                continue
            self._add_pixel_to_stats(stats, img.get_pixel(x, y))

    def _sample_center_patch_stats(self, img, center_x, center_y, patch_radius):
        image_width = img.width()
        image_height = img.height()
        patch_radius = clamp(patch_radius, 1, 3)
        stats = self._new_sample_stats()
        dy = -patch_radius
        while dy <= patch_radius:
            y = center_y + dy
            if y >= 0 and y < image_height:
                dx = -patch_radius
                while dx <= patch_radius:
                    x = center_x + dx
                    if x >= 0 and x < image_width:
                        self._add_pixel_to_stats(stats, img.get_pixel(x, y))
                    dx += 1
            dy += 1
        return self._finish_sample_stats(stats)

    def _sample_solid_target_stats(self, img, center_x, center_y, radius):
        if radius <= 3:
            return self._sample_center_patch_stats(img, center_x, center_y, 1)
        if radius <= 6:
            return self._sample_center_patch_stats(img, center_x, center_y, 2)
        stats = self._new_sample_stats()
        self._sample_offset_stats_into(img,
                                       stats,
                                       center_x,
                                       center_y,
                                       radius,
                                       _SOLID_SAMPLE_OFFSETS_X100,
                                       True)
        return self._finish_sample_stats(stats)

    def _sample_ring_target_stats(self, img, center_x, center_y, radius):
        stats = self._new_sample_stats()
        for ring_x100 in _RING_TARGET_RINGS_X100:
            sample_radius = max(1, (radius * ring_x100 + 50) // 100)
            self._sample_offset_stats_into(img,
                                           stats,
                                           center_x,
                                           center_y,
                                           sample_radius,
                                           _RING_SAMPLE_OFFSETS_X100,
                                           True)
        return self._finish_sample_stats(stats)

    def _sample_background_stats(self, img, center_x, center_y, radius):
        stats = self._new_sample_stats()
        inner_radius = radius + max(_BACKGROUND_INNER_PAD_MIN_PX, radius // 2)
        outer_radius = radius + max(_BACKGROUND_OUTER_PAD_MIN_PX, radius)
        self._sample_offset_stats_into(img,
                                       stats,
                                       center_x,
                                       center_y,
                                       inner_radius,
                                       _RING_SAMPLE_OFFSETS_X100,
                                       False)
        self._sample_offset_stats_into(img,
                                       stats,
                                       center_x,
                                       center_y,
                                       outer_radius,
                                       _RING_SAMPLE_OFFSETS_X100,
                                       False)
        if stats["total"] < _MIN_BACKGROUND_SAMPLES:
            return None
        return self._finish_sample_stats(stats)

    def _target_sample_stats(self, img, center_x, center_y, radius, target_mode):
        if target_mode == _EMISSION_MODE_RING:
            return self._sample_ring_target_stats(img, center_x, center_y, radius)
        return self._sample_solid_target_stats(img, center_x, center_y, radius)

    def _emission_profile(self, img, center_x, center_y, radius, target_mode, min_green_ratio_x100):
        if img is None:
            return {
                "green_ratio_x100": 100,
                "green_delta": 0,
                "luma_delta": 0,
                "excess_delta": 0,
                "green_peak": 255,
                "score": 0,
            }

        target = self._target_sample_stats(img, center_x, center_y, radius, target_mode)
        if target is None:
            return None

        background = self._sample_background_stats(img, center_x, center_y, radius)
        if background is None:
            green_delta = 0
            luma_delta = 0
            excess_delta = 0
            min_green_delta = _EMIT_MIN_GREEN_DELTA
            min_luma_delta = _EMIT_MIN_LUMA_DELTA
            min_excess_delta = _EMIT_MIN_EXCESS_DELTA
        else:
            green_delta = target["green_avg"] - background["green_avg"]
            luma_delta = target["luma_avg"] - background["luma_avg"]
            excess_delta = target["excess_avg"] - background["excess_avg"]
            min_green_delta = _EMIT_MIN_GREEN_DELTA + (background["green_avg"] // 16)
            min_luma_delta = _EMIT_MIN_LUMA_DELTA + (background["luma_avg"] // 24)
            min_excess_delta = _EMIT_MIN_EXCESS_DELTA + (background["excess_avg"] // 5)

        if target["green_ratio_x100"] < _HARD_MIN_GREEN_SAMPLES_X100:
            return None
        if target["green_peak"] < _EMIT_HARD_MIN_PEAK_GREEN:
            return None
        if (target["excess_avg"] < _EMIT_HARD_MIN_EXCESS_AVG and
                target["excess_peak"] < (_RGB_GREEN_DOMINANCE + _EMIT_MIN_EXCESS_DELTA)):
            return None

        positive_green_delta = green_delta if green_delta > 0 else 0
        positive_luma_delta = luma_delta if luma_delta > 0 else 0
        positive_excess_delta = excess_delta if excess_delta > 0 else 0
        score = target["green_ratio_x100"] * 8
        score += target["excess_avg"] * 20
        score += target["green_peak"] * 2
        score += target["excess_peak"] * 4
        score += (positive_green_delta * 5) + (positive_luma_delta * 4) + (positive_excess_delta * 16)

        if target["green_ratio_x100"] < min_green_ratio_x100:
            score -= (min_green_ratio_x100 - target["green_ratio_x100"]) * 14
        if target["green_peak"] < _EMIT_MIN_PEAK_GREEN:
            score -= (_EMIT_MIN_PEAK_GREEN - target["green_peak"]) * 8
        if target["excess_avg"] < _EMIT_MIN_EXCESS_AVG:
            score -= (_EMIT_MIN_EXCESS_AVG - target["excess_avg"]) * 18

        if background is None:
            score -= 60
        else:
            if green_delta < min_green_delta and luma_delta < min_luma_delta:
                score -= (min_green_delta - green_delta) * 4
                score -= (min_luma_delta - luma_delta) * 4
            if excess_delta < min_excess_delta:
                score -= (min_excess_delta - excess_delta) * 14

        return {
            "green_ratio_x100": target["green_ratio_x100"],
            "green_delta": green_delta,
            "luma_delta": luma_delta,
            "excess_delta": excess_delta,
            "green_peak": target["green_peak"],
            "score": score,
        }

    def _solid_target_from_blob(self, blob, img=None, metrics=None):
        if metrics is None:
            metrics = self._blob_metrics(blob)
        if not self._metrics_pass_fast_filter(metrics):
            return None

        roundness_x1000 = self._roundness_x1000(blob, metrics)
        if roundness_x1000 < self.params["roundness_min_x1000"]:
            return None

        center_x = blob.cx()
        center_y = blob.cy()
        emission = self._emission_profile(img,
                                          center_x,
                                          center_y,
                                          metrics["radius"],
                                          _EMISSION_MODE_SOLID,
                                          _MIN_SOLID_GREEN_SAMPLES_X100)
        if emission is None:
            return None

        return {
            "blob": blob,
            "source": "solid",
            "center_x": center_x,
            "center_y": center_y,
            "area": metrics["area"],
            "radius": metrics["radius"],
            "roundness_x1000": roundness_x1000,
            "green_fill_x100": metrics["green_fill_x100"],
            "center_white_x100": 0,
            "green_ratio_x100": emission["green_ratio_x100"],
            "green_delta": emission["green_delta"],
            "luma_delta": emission["luma_delta"],
            "excess_delta": emission["excess_delta"],
            "green_peak": emission["green_peak"],
            "emission_score": emission["score"],
        }

    def _center_white_ratio_x100(self, img, center_x, center_y, outer_diameter):
        sample_ratio = self.params["ring_center_sample_ratio_x100"]
        sample_radius = (outer_diameter * sample_ratio + 199) // 200
        sample_radius = clamp(sample_radius, 1, max(1, outer_diameter // 2))
        stats = self._sample_center_patch_stats(img, center_x, center_y, sample_radius)
        if stats is None:
            return 0
        return stats["white_ratio_x100"]

    def _inner_ring_roundness_x1000(self, img, center_x, center_y, outer_radius):
        if img is None or outer_radius <= 1:
            return 0

        min_inner_radius = max(1, (outer_radius * _RING_INNER_MIN_RADIUS_X100 + 50) // 100)
        max_inner_radius = max(min_inner_radius + 1,
                               (outer_radius * _RING_INNER_MAX_RADIUS_X100 + 50) // 100)
        image_width = img.width()
        image_height = img.height()
        inner_radii = []

        for dx100, dy100 in _EDGE_SAMPLE_OFFSETS_X100:
            found_radius = 0
            radius = 1
            while radius <= max_inner_radius:
                x = center_x + self._scaled_offset(dx100, radius)
                y = center_y + self._scaled_offset(dy100, radius)
                if x < 0 or x >= image_width or y < 0 or y >= image_height:
                    break
                is_green, _, _, _, _, _ = self._green_sample_values(img.get_pixel(x, y))
                if is_green:
                    found_radius = radius
                    break
                radius += 1
            if found_radius < min_inner_radius or found_radius > max_inner_radius:
                continue
            inner_radii.append(found_radius)

        if len(inner_radii) < _RING_INNER_MIN_HITS:
            return 0
        min_radius = min(inner_radii)
        max_radius = max(inner_radii)
        if max_radius <= 0:
            return 0
        return (min_radius * 1000) // max_radius

    def _ring_target_from_blob(self, img, blob, metrics=None):
        if img is None or self.params["ring_detection_enabled"] == 0:
            return None
        if metrics is None:
            metrics = self._blob_metrics(blob)
        if not self._metrics_pass_fast_filter(metrics):
            return None
        if metrics["outer_diameter"] < self.params["ring_min_outer_diameter_px"]:
            return None
        if metrics["aspect_x100"] < self.params["ring_min_aspect_x100"]:
            return None

        green_fill_x100 = metrics["green_fill_x100"]
        if green_fill_x100 < self.params["ring_min_fill_x100"]:
            return None
        if green_fill_x100 > self.params["ring_max_fill_x100"]:
            return None

        center_x = metrics["bbox_center_x"]
        center_y = metrics["bbox_center_y"]
        radius = metrics["radius"]
        roundness_x1000 = self._inner_ring_roundness_x1000(img, center_x, center_y, radius)
        if roundness_x1000 < self.params["ring_min_roundness_x1000"]:
            return None
        emission = self._emission_profile(img,
                                          center_x,
                                          center_y,
                                          radius,
                                          _EMISSION_MODE_RING,
                                          _MIN_RING_GREEN_SAMPLES_X100)
        if emission is None:
            return None

        center_white_x100 = 0
        if self.params["ring_min_center_white_x100"] > 0:
            center_white_x100 = self._center_white_ratio_x100(img, center_x, center_y, metrics["outer_diameter"])
            if center_white_x100 < self.params["ring_min_center_white_x100"]:
                return None

        return {
            "blob": blob,
            "source": "ring",
            "center_x": center_x,
            "center_y": center_y,
            "area": metrics["area"],
            "radius": radius,
            "roundness_x1000": roundness_x1000,
            "green_fill_x100": green_fill_x100,
            "center_white_x100": center_white_x100,
            "green_ratio_x100": emission["green_ratio_x100"],
            "green_delta": emission["green_delta"],
            "luma_delta": emission["luma_delta"],
            "excess_delta": emission["excess_delta"],
            "green_peak": emission["green_peak"],
            "emission_score": emission["score"],
        }

    def _target_score(self, target):
        score = target["roundness_x1000"] * 8
        score += target["green_ratio_x100"] * 10
        score += target["green_fill_x100"] * 2
        score += target["emission_score"]
        if target["source"] == "ring":
            score += 600 + (target["center_white_x100"] * 12)
            if target["center_white_x100"] <= 0 and target["green_fill_x100"] > _RING_SOLID_LIKE_FILL_X100:
                score -= 900
        if self._last_center is not None:
            distance = abs(target["center_x"] - self._last_center[0]) + abs(target["center_y"] - self._last_center[1])
            score -= distance * 4
        return score

    def _should_skip_ring_check(self, solid, metrics):
        return (solid is not None and
                self.params["ring_min_center_white_x100"] <= 0 and
                metrics["green_fill_x100"] > _RING_SOLID_LIKE_FILL_X100)

    def _target_from_blob(self, img, blob):
        metrics = self._blob_metrics(blob)
        solid = self._solid_target_from_blob(blob, img, metrics)
        if self._should_skip_ring_check(solid, metrics):
            return solid
        ring = self._ring_target_from_blob(img, blob, metrics)
        if solid is None:
            return ring
        if ring is None:
            return solid
        if self._target_score(ring) > self._target_score(solid):
            return ring
        return solid

    def _find_best_target(self, img, blobs):
        best_target = None
        best_score = -1000000

        for blob in blobs:
            metrics = self._blob_metrics(blob)
            if not self._metrics_pass_fast_filter(metrics):
                continue

            solid = self._solid_target_from_blob(blob, img, metrics)
            if self._should_skip_ring_check(solid, metrics):
                ring = None
            else:
                ring = self._ring_target_from_blob(img, blob, metrics)
            for target in (solid, ring):
                if target is None:
                    continue
                score = self._target_score(target)
                if score > best_score:
                    best_target = target
                    best_score = score

        return best_target

    def _find_best_blob(self, blobs):
        best_target = self._find_best_target(None, blobs)
        if best_target is None:
            return None
        return best_target["blob"]

    def _scan_blobs(self, img, threshold, roi):
        pixels_threshold = max(self.params["min_area"], 1)
        merge_margin = self.params["merge_margin"]
        merge_blobs = merge_margin > 0
        if roi is None:
            return img.find_blobs([threshold],
                                  x_stride=_BLOB_X_STRIDE,
                                  y_stride=_BLOB_Y_STRIDE,
                                  pixels_threshold=pixels_threshold,
                                  area_threshold=pixels_threshold,
                                  merge=merge_blobs,
                                  margin=merge_margin)
        return img.find_blobs([threshold],
                              roi=roi,
                              x_stride=_BLOB_X_STRIDE,
                              y_stride=_BLOB_Y_STRIDE,
                              pixels_threshold=pixels_threshold,
                              area_threshold=pixels_threshold,
                              merge=merge_blobs,
                              margin=merge_margin)

    def _full_scan_due(self, roi, best_target):
        if roi is None:
            return False
        return best_target is None or self._missed_frames > 0

    def _update_track(self, center):
        if center is None:
            if self._missed_frames < self.params["max_missed_frames"]:
                self._missed_frames += 1
            else:
                self._last_center = None
                self._last_radius = None
                self._last_area = None
            return None

        if self._last_center is None:
            self._last_center = center
            self._missed_frames = 0
            return center

        gain = self.params["center_filter_gain_x100"]
        filtered_x = ((self._last_center[0] * (100 - gain)) + (center[0] * gain) + 50) // 100
        filtered_y = ((self._last_center[1] * (100 - gain)) + (center[1] * gain) + 50) // 100
        self._last_center = (filtered_x, filtered_y)
        self._missed_frames = 0
        return self._last_center

    def process_frame(self, img):
        threshold = self._active_threshold()
        roi = self._resolve_roi(img.width(), img.height())

        blobs = self._scan_blobs(img, threshold, roi)
        best_target = self._find_best_target(img, blobs)

        if self._full_scan_due(roi, best_target):
            full_blobs = self._scan_blobs(img, threshold, None)
            full_target = self._find_best_target(img, full_blobs)
            if full_target is not None:
                if best_target is None or self._target_score(full_target) > (self._target_score(best_target) + 250):
                    best_target = full_target

        if best_target is None:
            self._update_track(None)
            return None

        filtered_center = self._update_track((best_target["center_x"], best_target["center_y"]))
        self._last_radius = best_target["radius"]
        self._last_area = best_target["area"]
        return {
            "center_x": filtered_center[0],
            "center_y": filtered_center[1],
            "area": best_target["area"],
            "radius": best_target["radius"],
            "source": best_target["source"],
        }
