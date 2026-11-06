#pragma once

#include <QFileInfo>
#include <QString>
#include <QStringList>
#include <cstddef>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "recorder/encoding/binary_format.hpp"
#include "tools/video_player/binary_stream.hpp"
#include "tools/video_player/csv.hpp"
#include "tools/video_player/recording_paths.hpp"
#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {
class Recording final {
public:
    bool load_directory(
        const QString& directory, const std::optional<QString>& frames_csv,
        const std::optional<QString>& detection_csv, const QString& extension, QString& error);
    bool load(
        const QString& image_directory, const QString& frames_csv, const QString& detection_csv,
        const QString& extension, QString& error);
    const std::vector<Frame>& frames() const { return frames_; }
    const QStringList& notes() const { return notes_; }
    QString summary() const;
    bool complete() const;

private:
    bool append_stream(const QFileInfo& file_info, QString& error);
    bool append_binary(const QFileInfo& file_info, QString& error);
    bool append_png_stream(const QFileInfo& file_info, QString& error);
    std::vector<Frame> frames_;
    QStringList notes_;
    std::size_t event_matches_ = 0;
    std::size_t detection_matches_ = 0;
    std::size_t missing_images_ = 0;
    std::size_t required_detection_matches_ = 0;
};

} // namespace dart::tools::video_player
