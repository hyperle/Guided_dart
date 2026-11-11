#pragma once

#include <QFileInfo>
#include <QImage>
#include <QString>
#include <cstddef>
#include <memory>
#include <vector>

#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {

class VideoStream final {
public:
    ~VideoStream();
    static bool supports(const QString& suffix);
    static bool append(
        const QFileInfo& file, const QString& relative_name, std::vector<Frame>& frames,
        QString& error);

private:
    friend class VideoImageReader;
    VideoStream();
    struct Index;
    std::unique_ptr<Index> index_;
};

class VideoImageReader final {
public:
    VideoImageReader();
    ~VideoImageReader();
    QImage read(const Frame& frame, QString& error);

private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace dart::tools::video_player
