#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

#include "camera/frame_view.hpp"

namespace dart::recorder {

// One encode call produces one complete access unit, including any codec headers.
// Returned bytes belong to the encoder until the next encode/reset. The input is
// only borrowed during encode; no camera lease is retained by the device adapter.
class IVideoEncoder {
public:
    virtual ~IVideoEncoder() = default;
    virtual bool prepare(std::uint32_t width, std::uint32_t height, std::uint32_t fps) noexcept = 0;
    // Bound for a complete access unit after prepare(), including codec headers.
    virtual std::size_t max_output_bytes() const noexcept = 0;
    virtual bool reset() noexcept = 0;
    virtual std::span<const std::byte> encode(const camera::FrameView& frame) noexcept = 0;
};

using VideoEncoderFactory = std::unique_ptr<IVideoEncoder> (*)() noexcept;

} // namespace dart::recorder
