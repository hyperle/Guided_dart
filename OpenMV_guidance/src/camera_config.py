try:
    from micropython import const
except ImportError:
    def const(value):
        return value

# Set to 1 only when SD video capture is needed; MJPEG writes consume frame time.
SD_RECORD_FLAG = const(0)

SD_RECORD_SEGMENT_DURATION_MS = const(15000)
SD_RECORD_MAX_SEGMENTS = const(12)
SD_RECORD_SYNC_INTERVAL_MS = const(1000)
SD_RECORD_JPEG_QUALITY = const(30)
SD_RECORD_MIN_FREE_BYTES = const(4 * 1024 * 1024)
