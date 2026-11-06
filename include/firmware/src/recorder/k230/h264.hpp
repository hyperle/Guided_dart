#pragma once

#include <memory>

#include "recorder/encoding/video_encoder.hpp"

namespace dart::recorder::k230 {
std::unique_ptr<IVideoEncoder> create_h264_encoder() noexcept;
} // namespace dart::recorder::k230
