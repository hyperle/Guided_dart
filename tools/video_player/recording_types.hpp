#pragma once

#include <QMap>
#include <QString>
#include <QStringList>
#include <cstddef>
#include <memory>
#include <optional>

namespace dart::tools::video_player {
class VideoStream;

struct CsvRow {
    QMap<QString, QString> values;
    QString path;
    QString raw;
    int line = 0;

    QString value(const QString& name) const { return values.value(name); }
    QString location() const { return QStringLiteral("%1:%2").arg(path).arg(line); }
    QString text() const;
};

// image_index is an output counter, never a CSV line or source sequence.
struct Frame {
    quint64 image_index = 0;
    QString path;
    QString relative_name;
    qint64 file_offset = 0;
    qint64 file_bytes = 0;            // Nonzero for one complete record within an image stream.
    qint64 binary_key_offset = -1;
    qint64 binary_decode_offset = -1; // Repeat frames reuse the last changed frame.
    quint32 binary_width = 0, binary_height = 0;
    std::shared_ptr<VideoStream> video;
    std::size_t video_frame_index = 0;
    std::optional<CsvRow> event;
    std::optional<CsvRow> detection;
    QStringList notes;

    QString key() const;
};

} // namespace dart::tools::video_player
