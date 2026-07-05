import sensor


DEFAULT_EXPOSURE_US = 2500
MIN_EXPOSURE_US = 80
MAX_EXPOSURE_US = 8000
SATURATION_THRESHOLD = 245
TRACKED_SAMPLE_STEP = 4
FULL_FRAME_SAMPLE_STEP = 8
MIN_ROI_RADIUS_PX = 18
MAX_ROI_RADIUS_PX = 64
TRACK_HOLD_FRAMES = 8
MIN_EXPOSURE_DELTA_US = 8


def _clamp(value, min_value, max_value):
    value = int(value)
    if value < min_value:
        return int(min_value)
    if value > max_value:
        return int(max_value)
    return value


def _pixel_channels(pixel):
    if isinstance(pixel, tuple) or isinstance(pixel, list):
        if len(pixel) >= 3:
            return int(pixel[0]), int(pixel[1]), int(pixel[2])
        if len(pixel) >= 1:
            value = int(pixel[0])
            return value, value, value
    value = int(pixel)
    return value, value, value


class TargetExposureController:
    def __init__(self,
                 initial_exposure_us=DEFAULT_EXPOSURE_US,
                 min_exposure_us=MIN_EXPOSURE_US,
                 max_exposure_us=MAX_EXPOSURE_US):
        self._exposure_us = _clamp(initial_exposure_us, min_exposure_us, max_exposure_us)
        self._min_exposure_us = int(min_exposure_us)
        self._max_exposure_us = int(max_exposure_us)
        self._last_center = None
        self._last_radius = MIN_ROI_RADIUS_PX
        self._missed_frames = TRACK_HOLD_FRAMES + 1
        self.last_saturation_x1000 = 0
        self.last_max_channel = 0
        self.last_avg_brightness = 0

    def apply(self):
        self._apply_exposure(self._exposure_us, True)

    def exposure_us(self):
        return self._exposure_us

    def _apply_exposure(self, exposure_us, force=False):
        exposure_us = _clamp(exposure_us, self._min_exposure_us, self._max_exposure_us)
        if (not force) and abs(exposure_us - self._exposure_us) < MIN_EXPOSURE_DELTA_US:
            return
        self._exposure_us = exposure_us
        sensor.set_auto_exposure(False, exposure_us=self._exposure_us)

    def _remember_result(self, result):
        if result is None:
            if self._missed_frames < TRACK_HOLD_FRAMES + 1:
                self._missed_frames += 1
            return
        self._last_center = (int(result["center_x"]), int(result["center_y"]))
        self._last_radius = _clamp(int(result.get("radius", MIN_ROI_RADIUS_PX)) * 2,
                                  MIN_ROI_RADIUS_PX,
                                  MAX_ROI_RADIUS_PX)
        self._missed_frames = 0

    def _roi(self, img):
        if self._last_center is None or self._missed_frames > TRACK_HOLD_FRAMES:
            return (0, 0, img.width(), img.height(), FULL_FRAME_SAMPLE_STEP, False)

        center_x, center_y = self._last_center
        radius = self._last_radius
        x0 = _clamp(center_x - radius, 0, img.width() - 1)
        y0 = _clamp(center_y - radius, 0, img.height() - 1)
        x1 = _clamp(center_x + radius, 0, img.width() - 1)
        y1 = _clamp(center_y + radius, 0, img.height() - 1)
        return (x0, y0, x1 - x0 + 1, y1 - y0 + 1, TRACKED_SAMPLE_STEP, True)

    def _sample_roi(self, img, roi):
        x0, y0, width, height, step, tracked = roi
        x1 = x0 + width
        y1 = y0 + height
        count = 0
        saturated = 0
        max_channel_seen = 0
        total_brightness = 0

        y = y0
        while y < y1:
            x = x0
            while x < x1:
                pixel = img.get_pixel(x, y)
                red, green, blue = _pixel_channels(pixel)
                max_channel = max(red, green, blue)
                total_brightness += max_channel
                if max_channel > max_channel_seen:
                    max_channel_seen = max_channel
                if max_channel >= SATURATION_THRESHOLD:
                    saturated += 1
                count += 1
                x += step
            y += step

        if count <= 0:
            return 0, 0, 0, 0, tracked
        return (saturated * 1000) // count, max_channel_seen, count, total_brightness // count, tracked

    def _next_exposure(self, saturation_x1000, max_channel, avg_brightness, tracked):
        exposure = self._exposure_us

        if saturation_x1000 >= 120:
            return (exposure * 65) // 100
        if saturation_x1000 >= 50:
            return (exposure * 78) // 100
        if saturation_x1000 >= 15:
            return (exposure * 90) // 100

        if tracked:
            if avg_brightness >= 200 or max_channel >= 235:
                return (exposure * 82) // 100
            if avg_brightness >= 175 or max_channel >= 220:
                return (exposure * 90) // 100
            if saturation_x1000 == 0 and avg_brightness < 160:
                return (exposure * 112) // 100
            if saturation_x1000 <= 3 and avg_brightness < 190:
                return (exposure * 105) // 100
            return exposure

        if max_channel >= 220:
            return (exposure * 78) // 100
        if max_channel >= 200:
            return (exposure * 90) // 100
        return exposure

    def update(self, img, result):
        self._remember_result(result)
        saturation_x1000, max_channel, count, avg_brightness, tracked = self._sample_roi(img, self._roi(img))
        if count <= 0:
            return
        self.last_saturation_x1000 = saturation_x1000
        self.last_max_channel = max_channel
        self.last_avg_brightness = avg_brightness
        self._apply_exposure(self._next_exposure(saturation_x1000, max_channel, avg_brightness, tracked))
