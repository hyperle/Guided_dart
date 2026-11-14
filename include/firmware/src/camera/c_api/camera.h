#ifndef DART_CAMERA_H
#define DART_CAMERA_H

#include <stdint.h>

#include "camera/error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DART_CAMERA_ABI_VERSION 7u
#define DART_CAMERA_MAX_BUFFERS 10u
/* Ceiling for config.max_outstanding_frames. A held lease keeps one grayscale
 * CPU slot busy. RAW DMA dumps are released before a lease is published.
 */
#define DART_CAMERA_MAX_OUTSTANDING_FRAMES 10u

/* Handles are created once at startup. All calls must be serialized. */
typedef struct dart_media dart_media_t;
typedef struct dart_camera dart_camera_t;

/* ABI values owned by the C state machine; mirrored by C++ with static_assert. */
enum {
    DART_CAMERA_STATE_EMPTY = 0,
    DART_CAMERA_STATE_INITIALIZED = 1,
    DART_CAMERA_STATE_STARTED = 2,
    DART_CAMERA_STATE_FAULTED = 3,
};

enum {
    DART_CAMERA_EXPOSURE_FIXED = 0,
    DART_CAMERA_EXPOSURE_AUTO = 1,
};

enum {
    DART_CAMERA_RAW10_LE16_LSB = 0,
    DART_CAMERA_RAW10_LE16_MSB = 1,
    DART_CAMERA_RAW10_MIPI = 2,
    DART_CAMERA_RAW10_PACKED_LSB = 3, /* Four samples in a little-endian 40-bit word. */
};

enum {
    DART_CAMERA_FRAME_SOURCE_PTS_UNVALIDATED = 1u,
};

typedef struct dart_camera_config {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t csi_port;                /* CSI0 / CSI1 / CSI2: 0 / 1 / 2. */
    /* Zero geometry selects a supported mode; explicit geometry must match. */
    uint32_t sensor_width;
    uint32_t sensor_height;
    uint32_t sensor_fps;      /* Zero: no rate request; readback may be unknown. */
    uint32_t output_width;    /* Both zero: sensor backend default (RAW: native size). */
    uint32_t output_height;
    uint32_t capture_buffers; /* Reserved capacity hint, 3..10; K230 MCM ring is fixed at 4. */
    uint32_t channel_buffers; /* Grayscale lease capacity bound, 3..10. */
    uint32_t max_outstanding_frames; /* 1..channel_buffers and <= MAX_OUTSTANDING.
                                      * Capacity is injected by the caller: it may
                                      * hold this many frames at once. 1 keeps the
                                      * single-frame behaviour. */
    uint32_t exposure_mode;
    uint32_t initial_exposure_lines;
    uint32_t mirror;                 /* 0: none, 1: horizontal, 2: vertical, 3: both. */
    uint32_t raw10_layout;           /* Explicit DDR packing; verify with sensor test pattern. */
    uint32_t analog_gain_milli;      /* Fixed mode: sensor analog gain x1000; 0 keeps init gain. */
} dart_camera_config_t;

typedef struct dart_camera_caps {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t sensor_width;
    uint32_t sensor_height;
    uint32_t sensor_fps;
    uint32_t output_width;
    uint32_t output_height;
    uint32_t exposure_mode;
} dart_camera_caps_t;

typedef struct dart_camera_frame {
    uint32_t abi_version;
    uint32_t struct_size;
    /* Read-only linear grayscale Y8 (RAW10 >> 2, or the ISP luma plane), valid
     * until successful release.
     * Each lease owns a separate preallocated CPU buffer. RAW DMA memory is
     * never modified. Every row is stride bytes; width bytes are pixels.
     */
    const uint8_t* data;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t flags;
    uint64_t source_pts;  /* Raw SDK value; timebase is unvalidated. */
    uint64_t received_us; /* CLOCK_MONOTONIC receipt time, not exposure time. */
    uint64_t token;
    const dart_camera_t* owner;
} dart_camera_frame_t;

/* Operations return only a code from camera/error_codes.h; outputs carry data.
 * C never logs or constructs errors. Rollback retains the primary return code;
 * failed rollback is observable as DART_CAMERA_STATE_FAULTED via get_state.
 */
void dart_camera_config_init(dart_camera_config_t* config);
int dart_camera_config_validate(const dart_camera_config_t* config);

/* One media owner per process. All MPP consumers must finish before destroy.
 * This slice reserves DEV0/CHN0 and permits one attached camera. The owner
 * must not coexist with independent raw SDK VB initialization/teardown.
 * Creation/destruction allocate/free only at startup/shutdown.
 */
int dart_media_create(dart_media_t** media);
int dart_media_destroy(dart_media_t* media);

int dart_camera_create(dart_camera_t** camera);
int dart_camera_init(
    dart_camera_t* camera, dart_media_t* media, const dart_camera_config_t* config);
/* Powers/initializes sensor, starts receiver and sensor, prepares DMA mappings,
 * then applies the configured analog gain and initial fixed exposure.
 * Zero initial_exposure_lines preserves the sensor's initialized exposure.
 */
int dart_camera_start(dart_camera_t* camera);
/* Finite timeout, 0=poll, <=1000 ms. Up to max_outstanding_frames frames may be
 * outstanding at once, each carrying a distinct token, and release may happen in
 * any order. Two results are normal negatives rather than errors:
 * DART_CAMERA_E_NO_FRAME (no frame arrived in time) and
 * DART_CAMERA_E_LEASE_EXHAUSTED (every slot is held, so this frame is not taken
 * and is recycled by the driver, i.e. dropped by policy).
 */
int dart_camera_acquire_frame(
    dart_camera_t* camera, uint32_t timeout_ms, dart_camera_frame_t* frame);
/* Releases the grayscale CPU slot. A stale, doubled or foreign token is rejected
 * and changes nothing. RAW dumps were released before acquisition returned.
 */
int dart_camera_release_frame(dart_camera_t* camera, const dart_camera_frame_t* frame);
/* Stop/deinit return BUSY while any lease is outstanding. */
int dart_camera_stop(dart_camera_t* camera);
/* Also stops an active stream. Retry after cleanup failure; handle is retained. */
int dart_camera_deinit(dart_camera_t* camera);
/* Only succeeds after deinit. Never silently drops resources. */
int dart_camera_destroy(dart_camera_t* camera);
int dart_camera_get_state(const dart_camera_t* camera, uint32_t* state);
/* Lease accounting for callers applying their own bounded-drop policy:
 * outstanding = leases still held, capacity = config.max_outstanding_frames.
 * Observable diagnostics, not an error channel.
 */
int dart_camera_get_lease_usage(
    const dart_camera_t* camera, uint32_t* outstanding, uint32_t* capacity);
int dart_camera_get_caps(const dart_camera_t* camera, dart_camera_caps_t* caps);

/* Fixed mode only, after sensor initialization. Units are whole sensor lines.
 * The backend extends/reduces frame length to accommodate the exposure.
 * C++ clamps requests to the reported line range; C rejects invalid values.
 */
int dart_camera_get_exposure_range(
    const dart_camera_t* camera, uint32_t* min_lines, uint32_t* max_lines);
int dart_camera_set_exposure_lines(dart_camera_t* camera, uint32_t lines);

#ifdef __cplusplus
}
#endif
#endif
