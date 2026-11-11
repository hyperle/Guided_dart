#include "recorder/k230/h264.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <span>

#include "mpi_sys_api.h"
#include "mpi_vb_api.h"
#include "mpi_venc_api.h"

namespace dart::recorder::k230 {

class H264Encoder final : public IVideoEncoder {
public:
    ~H264Encoder() override { (void)close(); }

    bool prepare(std::uint32_t width, std::uint32_t height, std::uint32_t fps) noexcept override {
        if (width == 0 || height == 0 || width > 1920 || height > 1080
            || width % 2 != 0 || height % 2 != 0 || fps == 0 || fps > 120)
            return fail("geometry", EINVAL);
        width_ = width;
        height_ = height;
        fps_ = fps;
        stride_ = (width + 15U) & ~15U;
        input_bytes_ = align_page(static_cast<std::size_t>(stride_) * height * 3U / 2U);
        capacity_ = static_cast<std::size_t>(width) * height * 2U + 65536U;
        output_.reset(new (std::nothrow) std::byte[capacity_]);
        return output_ != nullptr;
    }

    std::size_t max_output_bytes() const noexcept override { return capacity_; }

    bool reset() noexcept override {
        if (!close() || !output_) return false;
        k_vb_pool_config input_config{};
        input_config.blk_cnt = 1;
        input_config.blk_size = input_bytes_;
        input_config.mode = VB_REMAP_MODE_NOCACHE;
        const auto input_pool = kd_mpi_vb_create_pool(&input_config);
        if (input_pool < 0) return fail("input_pool", input_pool);
        input_pool_ = static_cast<k_u32>(input_pool);
        input_block_ = kd_mpi_vb_get_block(input_pool_, input_bytes_, nullptr);
        if (input_block_ == VB_INVALID_HANDLE) return fail("input_block", ENOMEM);
        input_physical_ = kd_mpi_vb_handle_to_phyaddr(input_block_);
        if (input_physical_ == 0) return fail("input_physical", EFAULT);
        input_ = kd_mpi_sys_mmap(input_physical_, input_bytes_);
        if (input_ == nullptr || input_ == reinterpret_cast<void*>(-1)) {
            input_ = nullptr;
            return fail("input_map", errno);
        }
        // Neutral NV12 chroma preserves the Y8 camera image as grayscale.
        std::memset(input_, 128, input_bytes_);
        k_vb_pool_config stream_config{};
        stream_config.blk_cnt = 4;
        stream_config.blk_size = align_page(static_cast<std::size_t>(width_) * height_);
        stream_config.mode = VB_REMAP_MODE_NOCACHE;
        const auto stream_pool = kd_mpi_vb_create_pool(&stream_config);
        if (stream_pool < 0) return fail("stream_pool", stream_pool);
        stream_pool_ = static_cast<k_u32>(stream_pool);
        int rc = kd_mpi_venc_request_chn(&channel_);
        if (rc != 0) return fail("request_channel", rc);
        requested_ = true;
        rc = kd_mpi_venc_attach_vb_pool(channel_, stream_pool_);
        if (rc != 0) return fail("attach_pool", rc);
        attached_ = true;
        k_venc_chn_attr attr{};
        attr.venc_attr.type = K_PT_H264;
        attr.venc_attr.profile = VENC_PROFILE_H264_BASELINE;
        attr.venc_attr.pic_width = width_;
        attr.venc_attr.pic_height = height_;
        attr.rc_attr.rc_mode = K_VENC_RC_MODE_CBR;
        attr.rc_attr.cbr.gop = fps_;
        attr.rc_attr.cbr.stats_time = 1;
        attr.rc_attr.cbr.src_frame_rate = fps_;
        attr.rc_attr.cbr.dst_frame_rate = fps_;
        attr.rc_attr.cbr.bit_rate = 2000;
        rc = kd_mpi_venc_create_chn(channel_, &attr);
        if (rc != 0) return fail("create_channel", rc);
        created_ = true;
        rc = kd_mpi_venc_start_chn(channel_);
        if (rc != 0) return fail("start_channel", rc);
        started_ = true;
        sequence_ = 0;
        std::fprintf(stdout, "level=INFO module=recorder op=h264_start width=%u height=%u fps=%u kbps=2000 channel=%u\n",
                     width_, height_, fps_, channel_);
        return true;
    }

    std::span<const std::byte> encode(const camera::FrameView& view) noexcept override {
        if (!started_ || view.width != width_ || view.height != height_ || view.stride < width_
            || view.luma.size() < static_cast<std::size_t>(height_ - 1U) * view.stride + width_) {
            (void)fail("input_frame", EINVAL);
            return {};
        }
        auto* y = static_cast<std::byte*>(input_);
        for (std::uint32_t row = 0; row < height_; ++row)
            std::memcpy(y + static_cast<std::size_t>(row) * stride_,
                        view.luma.data() + static_cast<std::size_t>(row) * view.stride, width_);
        k_video_frame_info frame{};
        frame.pool_id = input_pool_;
        frame.mod_id = K_ID_VENC;
        frame.v_frame.width = width_;
        frame.v_frame.height = height_;
        frame.v_frame.pixel_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        frame.v_frame.stride[0] = frame.v_frame.stride[1] = stride_;
        frame.v_frame.phys_addr[0] = input_physical_;
        frame.v_frame.phys_addr[1] = input_physical_ + static_cast<k_u64>(stride_) * height_;
        frame.v_frame.virt_addr[0] = reinterpret_cast<k_u64>(input_);
        frame.v_frame.virt_addr[1] = frame.v_frame.virt_addr[0] + static_cast<k_u64>(stride_) * height_;
        frame.v_frame.pts = static_cast<k_u64>(view.received.count());
        frame.v_frame.time_ref = ++sequence_;
        int rc = kd_mpi_venc_send_frame(channel_, &frame, 200);
        if (rc != 0) { (void)fail("send_frame", rc); return {}; }
        std::size_t bytes = 0;
        // VENC may deliver SPS/PPS separately. Count a complete picture only
        // after its non-header packet; never count codec headers as frames.
        for (unsigned batch = 0; batch < 8; ++batch) {
            k_venc_chn_status status{};
            rc = kd_mpi_venc_query_status(channel_, &status);
            if (rc != 0) { (void)fail("query_stream", rc); return {}; }
            std::array<k_venc_pack, 32> packs{};
            const k_u32 count = status.cur_packs != 0 ? status.cur_packs : 1U;
            if (count > packs.size()) { (void)fail("packet_capacity", EOVERFLOW); return {}; }
            k_venc_stream stream{packs.data(), count};
            rc = kd_mpi_venc_get_stream(channel_, &stream, 200);
            if (rc != 0) { (void)fail("get_stream", rc); return {}; }
            bool picture = false, ok = stream.pack_cnt <= packs.size();
            for (k_u32 i = 0; ok && i < stream.pack_cnt; ++i) {
                const auto& pack = packs[i];
                if (pack.len == 0 || pack.len > capacity_ - bytes) { ok = false; break; }
                void* data = kd_mpi_sys_mmap(pack.phys_addr, pack.len);
                if (data == nullptr || data == reinterpret_cast<void*>(-1)) { ok = false; break; }
                std::memcpy(output_.get() + bytes, data, pack.len);
                bytes += pack.len;
                if (kd_mpi_sys_munmap(data, pack.len) != 0) ok = false;
                picture = picture || pack.type != K_VENC_HEADER;
            }
            rc = kd_mpi_venc_release_stream(channel_, &stream);
            if (!ok || rc != 0) { (void)fail("copy_release_stream", rc != 0 ? rc : EIO); return {}; }
            if (picture) return {output_.get(), bytes};
        }
        (void)fail("picture_missing", ETIMEDOUT);
        return {};
    }

private:
    static std::size_t align_page(std::size_t bytes) noexcept { return (bytes + 4095U) & ~std::size_t{4095U}; }
    static bool fail(const char* operation, int code) noexcept {
        std::fprintf(stderr, "level=ERROR module=recorder codec=h264 op=%s native=%d hex=0x%x\n",
                     operation, code, static_cast<unsigned>(code));
        errno = EIO;
        return false;
    }
    bool close() noexcept {
        if (started_) {
            const int rc = kd_mpi_venc_stop_chn(channel_);
            if (rc != 0) return fail("stop_channel", rc);
            started_ = false;
        }
        if (created_) {
            const int rc = kd_mpi_venc_destroy_chn(channel_);
            if (rc != 0) return fail("destroy_channel", rc);
            created_ = false;
        }
        if (attached_) {
            const int rc = kd_mpi_venc_detach_vb_pool(channel_);
            if (rc != 0) return fail("detach_pool", rc);
            attached_ = false;
        }
        if (requested_) {
            const int rc = kd_mpi_venc_release_chn(channel_);
            if (rc != 0) return fail("release_channel", rc);
            requested_ = false;
        }
        if (input_) {
            const int rc = kd_mpi_sys_munmap(input_, input_bytes_);
            if (rc != 0) return fail("input_unmap", rc);
            input_ = nullptr;
        }
        if (input_block_ != VB_INVALID_HANDLE) {
            const int rc = kd_mpi_vb_release_block(input_block_);
            if (rc != 0) return fail("input_release", rc);
            input_block_ = VB_INVALID_HANDLE;
        }
        if (input_pool_ != VB_INVALID_POOLID) {
            const int rc = kd_mpi_vb_destory_pool(input_pool_);
            if (rc != 0) return fail("input_pool_destroy", rc);
            input_pool_ = VB_INVALID_POOLID;
        }
        if (stream_pool_ != VB_INVALID_POOLID) {
            const int rc = kd_mpi_vb_destory_pool(stream_pool_);
            if (rc != 0) return fail("stream_pool_destroy", rc);
            stream_pool_ = VB_INVALID_POOLID;
        }
        return true;
    }

    std::uint32_t width_ = 0, height_ = 0, stride_ = 0, fps_ = 0, sequence_ = 0;
    std::size_t input_bytes_ = 0, capacity_ = 0;
    std::unique_ptr<std::byte[]> output_{};
    void* input_ = nullptr;
    k_u64 input_physical_ = 0;
    k_vb_blk_handle input_block_ = VB_INVALID_HANDLE;
    k_u32 input_pool_ = VB_INVALID_POOLID, stream_pool_ = VB_INVALID_POOLID, channel_ = 0;
    bool requested_ = false, attached_ = false, created_ = false, started_ = false;
};

std::unique_ptr<IVideoEncoder> create_h264_encoder() noexcept {
    return std::unique_ptr<IVideoEncoder>{new (std::nothrow) H264Encoder};
}

} // namespace dart::recorder::k230
