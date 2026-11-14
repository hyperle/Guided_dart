#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include <span>
#include <zlib.h>

#include "camera/frame_view.hpp"
#include "recorder/config.hpp"
#include "recorder/encoding/encoders.hpp"

namespace dart::recorder {

// Views refer either to the held camera frame or to this encoder's workspace.
// They remain valid until the next encode/reset; borrowed_input retains the lease.
class Encoder final {
public:
    using Image = EncodedImage;

    explicit Encoder(const Config& config) noexcept : config_(config) {}
    bool prepare() noexcept {
        if (config_.image_format == ImageFormat::kH264) {
            video_ = config_.video_encoder_factory != nullptr ? config_.video_encoder_factory() : nullptr;
            return video_ && video_->prepare(config_.width, config_.height, config_.fps);
        }
        if (config_.image_format == ImageFormat::kPng
            || config_.image_format == ImageFormat::kPngStream)
            return png_.prepare(config_.width, config_.height);
        if (config_.image_format == ImageFormat::kDbv)
            return binary_.prepare(config_.width, config_.height, config_.pbm_threshold);
        const bool pbm = config_.image_format == ImageFormat::kPbm;
        const int length = pbm
            ? std::snprintf(header_, sizeof(header_), "P4\n%u %u\n", config_.width, config_.height)
            : std::snprintf(header_, sizeof(header_), "P5\n%u %u\n255\n", config_.width, config_.height);
        if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(header_)) return false;
        header_bytes_ = static_cast<std::size_t>(length);
        if (pbm) {
            packed_bytes_ = ((static_cast<std::size_t>(config_.width) + 7U) / 8U) * config_.height;
            packed_.reset(new (std::nothrow) std::byte[packed_bytes_]);
            return packed_ != nullptr;
        }
        return true;
    }
    bool reset() noexcept { binary_.reset(); return !video_ || video_->reset(); }
    bool valid_view(const camera::FrameView& view) const noexcept {
        // Every image encoder here consumes grayscale rows.
        return config_.width != 0 && config_.height != 0
            && view.format == camera::PixelFormat::kY8 && view.valid()
            && view.width == config_.width && view.height == config_.height;
    }
    std::size_t record_bytes() const noexcept {
        if (config_.variable_record_size()) return 0;
        return header_bytes_ + (config_.image_format == ImageFormat::kPbm
            ? packed_bytes_ : static_cast<std::size_t>(config_.width) * config_.height);
    }
    std::size_t max_output_bytes() const noexcept {
        if (video_) return video_->max_output_bytes();
        if (config_.image_format == ImageFormat::kPng || config_.image_format == ImageFormat::kPngStream)
            return png_.max_output_bytes();
        if (config_.image_format == ImageFormat::kDbv) return binary_.max_output_bytes();
        return record_bytes();
    }
    Image encode(const camera::FrameView& view) noexcept {
        if (!valid_view(view)) return {};
        if (video_) return {{}, video_->encode(view)};
        if (config_.image_format == ImageFormat::kPng || config_.image_format == ImageFormat::kPngStream)
            return {{}, png_.encode(view.pixels, view.stride)};
        if (config_.image_format == ImageFormat::kDbv)
            return {{}, binary_.encode(view.pixels, view.stride)};
        const auto header = std::span<const std::byte>{reinterpret_cast<const std::byte*>(header_), header_bytes_};
        if (config_.image_format == ImageFormat::kPgm)
            return {header, view.pixels, view.width, view.height, view.stride, true};
        const auto row_bytes = (static_cast<std::size_t>(view.width) + 7U) / 8U;
        for (std::uint32_t y = 0; y < view.height; ++y) {
            const auto* row = view.row(y);
            for (std::uint32_t x = 0; x < view.width; x += 8U) {
                unsigned value = 0;
                for (unsigned bit = 0; bit < std::min(8U, view.width - x); ++bit)
                    value |= (std::to_integer<unsigned>(row[x + bit]) < config_.pbm_threshold) << (7U - bit);
                packed_[static_cast<std::size_t>(y) * row_bytes + x / 8U] = std::byte(value);
            }
        }
        return {header, {packed_.get(), packed_bytes_}};
    }

private:
    const Config& config_;
    std::unique_ptr<IVideoEncoder> video_{};
    PngEncoder png_{};
    BinaryEncoder binary_{};
    std::unique_ptr<std::byte[]> packed_{};
    std::size_t packed_bytes_ = 0, header_bytes_ = 0;
    char header_[64]{};
};

} // namespace dart::recorder
