#ifndef DART_CAMERA_ERROR_CODES_H
#define DART_CAMERA_ERROR_CODES_H

/* One int carries the result across the C ABI:
 * 0: success; 1..999: dart site code; -1..-4095: -errno;
 * other native values are interpreted by the selected backend.
 * C and C++ share site codes; backend diagnostics retain the native value.
 */
#define DART_CAMERA_CODE_ERRNO_MAX 4095

enum {
    DART_CAMERA_OK = 0,

    /* 1..19: arguments, state and normal no-frame result. */
    DART_CAMERA_E_INVALID_ARGUMENT = 1,
    DART_CAMERA_E_INVALID_STATE = 2,
    DART_CAMERA_E_BUSY = 3,
    DART_CAMERA_E_NOT_SUPPORTED = 4,
    DART_CAMERA_E_OUT_OF_RANGE = 5,
    DART_CAMERA_E_STALE_LEASE = 6,
    DART_CAMERA_E_NO_MEMORY = 7,
    DART_CAMERA_E_NO_FRAME = 8, /* Normal absence; C++ returns kNoFrame. */

    /* 20..39: configuration. */
    DART_CAMERA_E_CONFIG_VERSION = 20,
    DART_CAMERA_E_CONFIG_INVALID = 21,

    /* 40..59: sensor selection. */
    DART_CAMERA_E_SENSOR_PROBE_FAILED = 40,
    DART_CAMERA_E_SENSOR_MODE_MISMATCH = 41,

    /* 60..79: pool, block and mapping ownership. */
    DART_CAMERA_E_POOL_CREATE_FAILED = 60,
    DART_CAMERA_E_BLOCK_EXHAUSTED = 61,
    DART_CAMERA_E_MAP_FAILED = 62,
    DART_CAMERA_E_BLOCK_RELEASE_FAILED = 63,

    /* 80..99: frame geometry and ownership. */
    DART_CAMERA_E_FRAME_GEOMETRY_MISMATCH = 80,
    DART_CAMERA_E_FRAME_POOL_MISMATCH = 81,
    DART_CAMERA_E_FRAME_NOT_MAPPED = 82,

    /* 100..119: cache synchronization and clock. */
    DART_CAMERA_E_CACHE_INVALIDATE_FAILED = 100,
    DART_CAMERA_E_CLOCK_FAILED = 101,

    /* 120..139: exposure. */
    DART_CAMERA_E_EXPOSURE_RANGE_FAILED = 120,
    DART_CAMERA_E_EXPOSURE_WRITE_FAILED = 121,

    /* 140..159: lease ABI. */
    DART_CAMERA_E_LEASE_ABI_MISMATCH = 140,
    /* Every configured lease slot is held. Not a fault: the caller must finish
     * (release) a lease before taking another frame. The frame stays in the
     * driver pool and is recycled, i.e. it is dropped by policy.
     */
    DART_CAMERA_E_LEASE_EXHAUSTED = 141,

    DART_CAMERA_E_SITE_CODE_MAX = 999,
};

#endif
