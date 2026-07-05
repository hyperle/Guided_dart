FRAME_HEADER = (0xA5, 0x5A)
MEASUREMENT_HEADER = 0x5A
MEASUREMENT_LENGTH = 0x0A
PARAM_TYPE_SET = 0x10
PARAM_TYPE_ACK = 0x11

STATUS_APPLIED = 0x00
STATUS_UNKNOWN_KEY = 0x01
STATUS_INVALID_VALUE = 0x02
STATUS_INVALID_LENGTH = 0x03

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
    return value, value, value


class GreenLightDetector:
    def __init__(self):
        self.params = {
            "threshold_l_min": 20,
            "threshold_l_max": 70,
            "threshold_a_min": -77,
            "threshold_a_max": -19,
            "threshold_b_min": -19,
            "threshold_b_max": 38,
            "min_area": 1,
            "max_area": 36000,
            "roundness_min_x1000": 700,
            "merge_margin": 5,
            "track_window_radius_px": 80,
            "center_filter_gain_x100": 70,
            "max_missed_frames": 8,
            "ring_detection_enabled": 1,
            "ring_min_roundness_x1000": 350,
            "ring_min_aspect_x100": 65,
            "ring_min_fill_x100": 8,
            "ring_max_fill_x100": 76,
            "ring_min_center_white_x100": 20,
            "ring_center_sample_ratio_x100": 35,
            "ring_center_min_brightness": 220,
            "ring_center_max_channel_delta": 80,
            "ring_min_outer_diameter_px": 12,
        }
        self._param_rx_state = 0
        self._param_rx_buffer = bytearray()
        self._last_center = None
        self._missed_frames = 0

    def threshold_tuple(self):
        return (
            self.params["threshold_l_min"],
            self.params["threshold_l_max"],
            self.params["threshold_a_min"],
            self.params["threshold_a_max"],
            self.params["threshold_b_min"],
            self.params["threshold_b_max"],
        )

    def _resolve_roi(self, image_width, image_height):
        if self._last_center is None:
            return None
        radius = self.params["track_window_radius_px"]
        center_x, center_y = self._last_center
        start_x = clamp(center_x - radius, 0, image_width - 1)
        start_y = clamp(center_y - radius, 0, image_height - 1)
        end_x = clamp(center_x + radius, 0, image_width - 1)
        end_y = clamp(center_y + radius, 0, image_height - 1)
        return (start_x, start_y, end_x - start_x + 1, end_y - start_y + 1)

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

    def _blob_passes_area(self, blob):
        area = blob.area()
        if area < self.params["min_area"]:
            return False
        if self.params["max_area"] > 0 and area > self.params["max_area"]:
            return False
        return True

    def _blob_pixels(self, blob):
        try:
            return blob.pixels()
        except AttributeError:
            return blob.area()

    def _solid_target_from_blob(self, blob):
        if not self._blob_passes_area(blob):
            return None

        roundness_x1000 = int(blob.roundness() * 1000)
        if roundness_x1000 < self.params["roundness_min_x1000"]:
            return None

        return {
            "blob": blob,
            "source": "solid",
            "center_x": blob.cx(),
            "center_y": blob.cy(),
            "area": blob.area(),
            "radius": int(blob.w() / 2),
            "roundness_x1000": roundness_x1000,
            "green_fill_x100": (self._blob_pixels(blob) * 100) // max(blob.area(), 1),
            "center_white_x100": 0,
        }

    def _center_white_ratio_x100(self, img, center_x, center_y, outer_diameter):
        sample_ratio = self.params["ring_center_sample_ratio_x100"]
        sample_radius = (outer_diameter * sample_ratio + 199) // 200
        sample_radius = clamp(sample_radius, 1, max(1, outer_diameter // 2))
        step = 1
        if sample_radius > 8:
            step = 2

        x0 = clamp(center_x - sample_radius, 0, img.width() - 1)
        y0 = clamp(center_y - sample_radius, 0, img.height() - 1)
        x1 = clamp(center_x + sample_radius, 0, img.width() - 1)
        y1 = clamp(center_y + sample_radius, 0, img.height() - 1)
        min_brightness = self.params["ring_center_min_brightness"]
        max_delta = self.params["ring_center_max_channel_delta"]
        total = 0
        white = 0

        y = y0
        while y <= y1:
            x = x0
            while x <= x1:
                red, green, blue = pixel_channels(img.get_pixel(x, y))
                max_channel = max(red, green, blue)
                min_channel = min(red, green, blue)
                if max_channel >= min_brightness and (max_channel - min_channel) <= max_delta:
                    white += 1
                total += 1
                x += step
            y += step

        if total <= 0:
            return 0
        return (white * 100) // total

    def _ring_target_from_blob(self, img, blob):
        if img is None or self.params["ring_detection_enabled"] == 0:
            return None
        if not self._blob_passes_area(blob):
            return None

        rect = blob.rect()
        x, y, width, height = rect
        outer_diameter = min(width, height)
        if outer_diameter < self.params["ring_min_outer_diameter_px"]:
            return None

        max_side = max(width, height)
        if max_side <= 0:
            return None
        aspect_x100 = (outer_diameter * 100) // max_side
        if aspect_x100 < self.params["ring_min_aspect_x100"]:
            return None

        roundness_x1000 = int(blob.roundness() * 1000)
        if roundness_x1000 < self.params["ring_min_roundness_x1000"]:
            return None

        area = blob.area()
        green_fill_x100 = (self._blob_pixels(blob) * 100) // max(area, 1)
        if green_fill_x100 < self.params["ring_min_fill_x100"]:
            return None
        if green_fill_x100 > self.params["ring_max_fill_x100"]:
            return None

        center_x = x + (width // 2)
        center_y = y + (height // 2)
        center_white_x100 = self._center_white_ratio_x100(img, center_x, center_y, outer_diameter)
        if center_white_x100 < self.params["ring_min_center_white_x100"]:
            return None

        return {
            "blob": blob,
            "source": "ring",
            "center_x": center_x,
            "center_y": center_y,
            "area": area,
            "radius": int(outer_diameter / 2),
            "roundness_x1000": roundness_x1000,
            "green_fill_x100": green_fill_x100,
            "center_white_x100": center_white_x100,
        }

    def _target_from_blob(self, img, blob):
        target = self._solid_target_from_blob(blob)
        if target is not None:
            return target
        return self._ring_target_from_blob(img, blob)

    def _find_best_target(self, img, blobs):
        best_solid = None
        best_solid_roundness = -1
        best_ring = None
        best_ring_score = -1

        for blob in blobs:
            solid = self._solid_target_from_blob(blob)
            if solid is not None:
                if solid["roundness_x1000"] > best_solid_roundness:
                    best_solid = solid
                    best_solid_roundness = solid["roundness_x1000"]
                continue

            ring = self._ring_target_from_blob(img, blob)
            if ring is None:
                continue
            ring_score = (ring["roundness_x1000"] * 1000) + ring["center_white_x100"]
            if ring_score > best_ring_score:
                best_ring = ring
                best_ring_score = ring_score

        if best_solid is not None:
            return best_solid
        return best_ring

    def _find_best_blob(self, blobs):
        best_target = self._find_best_target(None, blobs)
        if best_target is None:
            return None
        return best_target["blob"]

    def _update_track(self, center):
        if center is None:
            if self._missed_frames < self.params["max_missed_frames"]:
                self._missed_frames += 1
            else:
                self._last_center = None
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
        roi = self._resolve_roi(img.width(), img.height())
        if roi is None:
            blobs = img.find_blobs([self.threshold_tuple()],
                                   pixels_threshold=max(self.params["min_area"], 1),
                                   merge=True,
                                   margin=self.params["merge_margin"])
        else:
            blobs = img.find_blobs([self.threshold_tuple()],
                                   roi=roi,
                                   pixels_threshold=max(self.params["min_area"], 1),
                                   merge=True,
                                   margin=self.params["merge_margin"])

        best_target = self._find_best_target(img, blobs)
        if best_target is None and roi is not None:
            blobs = img.find_blobs([self.threshold_tuple()],
                                   pixels_threshold=max(self.params["min_area"], 1),
                                   merge=True,
                                   margin=self.params["merge_margin"])
            best_target = self._find_best_target(img, blobs)

        if best_target is None:
            self._update_track(None)
            return None

        filtered_center = self._update_track((best_target["center_x"], best_target["center_y"]))
        return {
            "center_x": filtered_center[0],
            "center_y": filtered_center[1],
            "area": best_target["area"],
            "radius": best_target["radius"],
            "source": best_target["source"],
        }
