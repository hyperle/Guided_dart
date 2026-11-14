/* VICAP ISP pipeline for color Bayer sensors. The channel DMA writes YUV420SP
 * into a VICAP-created VB pool; frame.c adopts that pool from the first frame,
 * maps each block's luma plane, and the sensor ops copy it out as Y8. Settings
 * mirror the board-verified CanMV setup.
 */
#include "camera/k230/internal.h"

#include <stdint.h>

#include "k_vb_comm.h"
#include "mpi_vicap_api.h"

enum {
    /* VICAP-owned RAW input ring for the offline ISP (CanMV default: four). */
    DART_ISP_INPUT_BUFFERS = 4u,
};

static uint64_t luma_plane_bytes(const dart_camera_t* camera) {
    return (uint64_t)camera->raw_stride * camera->config.output_height;
}

uint32_t dart_camera_isp_luma_bytes(const dart_camera_t* camera) {
    return (uint32_t)VB_ALIGN_UP(luma_plane_bytes(camera), 4096u);
}

int dart_camera_initialize_isp(dart_camera_t* camera, const k_vicap_sensor_info* sensor) {
    const dart_camera_config_t* config = &camera->config;
    if (camera->raw_pool_id >= 0 || camera->output_pool.block_bytes < luma_plane_bytes(camera))
        return DART_CAMERA_E_INVALID_STATE;

    k_vicap_dev_attr dev = {0};
    dev.acq_win.width = sensor->width;
    dev.acq_win.height = sensor->height;
    /* Offline: the sensor writes RAW to DDR and the ISP reads it back; this
     * decouples the 1080x1280 120 fps MIPI stream from ISP back-pressure. */
    dev.mode = VICAP_WORK_OFFLINE_MODE;
    dev.input_type = VICAP_INPUT_TYPE_SENSOR;
    dev.dev_enable = K_TRUE;
    dev.buffer_num = DART_ISP_INPUT_BUFFERS;
    dev.buffer_size = (k_u32)VB_ALIGN_UP((uint64_t)sensor->width * sensor->height * 2u, 4096u);
    dev.buffer_pool_id = VB_INVALID_POOLID;
    /* Every ISP block except those that move the picture between frames:
     * AE would override the fixed exposure, AWB/WB tint follows the scene, and
     * 3DNR blends past frames into moving targets. AF/HDR are absent. */
    dev.pipe_ctrl.data = 0xffffffffu;
    dev.pipe_ctrl.bits.ae_enable = 0;
    dev.pipe_ctrl.bits.af_enable = 0;
    dev.pipe_ctrl.bits.ahdr_enable = 0;
    dev.pipe_ctrl.bits.awb_enable = 0;
    dev.pipe_ctrl.bits.wb_enable = 0;
    dev.pipe_ctrl.bits.dnr3_enable = 0;
    dev.dw_enable = K_FALSE;
    dev.cpature_frame = 0;
    dev.mirror = VICAP_MIRROR_NONE;
    dev.sensor_info = *sensor;
    int rc = kd_mpi_vicap_set_dev_attr(VICAP_DEV_ID_0, dev);
    if (rc != 0)
        return rc;

    /* No crop window: the channel scales the whole acquisition window. */
    k_vicap_chn_attr channel = {0};
    channel.out_win.width = config->output_width;
    channel.out_win.height = config->output_height;
    channel.crop_enable = K_FALSE;
    channel.scale_enable = K_FALSE;
    channel.chn_enable = K_TRUE;
    channel.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    channel.buffer_num = dart_camera_buffer_count(camera);
    channel.buffer_size = (k_u32)VB_ALIGN_UP(luma_plane_bytes(camera) * 3u / 2u, 4096u);
    channel.buffer_pool_id = VB_INVALID_POOLID;
    channel.alignment = 12;
    rc = kd_mpi_vicap_set_chn_attr(VICAP_DEV_ID_0, VICAP_CHN_ID_0, channel);
    if (rc != 0)
        return rc;

    /* Even a failed SDK init may require deinit. */
    camera->vicap_initialized = 1;
    return kd_mpi_vicap_init(VICAP_DEV_ID_0);
}
