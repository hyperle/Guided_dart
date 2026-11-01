#pragma once

#include <QString>

#include "tools/video_player/binary_stream.hpp"
#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {
QImage read_frame_image(const Frame& frame, QString& error);

} // namespace dart::tools::video_player
