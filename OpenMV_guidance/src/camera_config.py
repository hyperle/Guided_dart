try:
    from micropython import const
except ImportError:
    def const(value):
        return value

# Set to 0 to disable SD video capture; MJPEG writes consume frame time.
SD_RECORD_FLAG = const(0)

# Set to 1 to enable target-area-triggered fixed exposure switching.
TARGET_EXPOSURE_SWITCH_FLAG = const(1)

# Set to 1 to enable blob ring target checks; 0 disables them silently.
BLOB_RING_DETECTION_FLAG = const(0)

SD_RECORD_SEGMENT_DURATION_MS = const(15000)
SD_RECORD_MAX_SEGMENTS = const(12)
SD_RECORD_SYNC_INTERVAL_MS = const(1000)
SD_RECORD_JPEG_QUALITY = const(30)
SD_RECORD_MIN_FREE_BYTES = const(4 * 1024 * 1024)
