FRAME_HEADER = (0xA5, 0x5A)
MEASUREMENT_HEADER = 0x5A
MEASUREMENT_LENGTH = 0x06
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


class GreenLightDetector:
    def __init__(self):
        self.params = {
            "threshold_l_min": 20,
            "threshold_l_max": 70,
            "threshold_a_min": -77,
            "threshold_a_max": -19,
            "threshold_b_min": -19,
            "threshold_b_max": 38,
            "min_area": 20,
            "max_area": 2000,
            "roundness_min_x1000": 700,
            "merge_margin": 5,
            "track_window_radius_px": 80,
            "center_filter_gain_x100": 70,
            "max_missed_frames": 2,
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

    def send_measurement(self, uart, x, y, area):
        packet = bytearray()
        packet.append(MEASUREMENT_HEADER)
        packet.append(MEASUREMENT_LENGTH)
        packet.append((x >> 8) & 0xFF)
        packet.append(x & 0xFF)
        packet.append((y >> 8) & 0xFF)
        packet.append(y & 0xFF)
        packet.append((area >> 8) & 0xFF)
        packet.append(area & 0xFF)
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

    def _find_best_blob(self, blobs):
        best_blob = None
        best_roundness = 0

        for blob in blobs:
            area = blob.area()
            if area < self.params["min_area"]:
                continue
            if self.params["max_area"] > 0 and area > self.params["max_area"]:
                continue

            roundness_x1000 = int(blob.roundness() * 1000)
            if roundness_x1000 < self.params["roundness_min_x1000"]:
                continue

            if roundness_x1000 > best_roundness:
                best_blob = blob
                best_roundness = roundness_x1000

        return best_blob

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

        best_blob = self._find_best_blob(blobs)
        if best_blob is None and roi is not None:
            blobs = img.find_blobs([self.threshold_tuple()],
                                   pixels_threshold=max(self.params["min_area"], 1),
                                   merge=True,
                                   margin=self.params["merge_margin"])
            best_blob = self._find_best_blob(blobs)

        if best_blob is None:
            self._update_track(None)
            return None

        filtered_center = self._update_track((best_blob.cx(), best_blob.cy()))
        return {
            "center_x": filtered_center[0],
            "center_y": filtered_center[1],
            "area": best_blob.area(),
            "radius": int(best_blob.w() / 2),
        }
