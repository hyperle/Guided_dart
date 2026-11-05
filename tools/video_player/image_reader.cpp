#include "tools/video_player/image_reader.hpp"

#include <QBuffer>
#include <QFile>
#include <QImageReader>
#include <QString>

#include "tools/video_player/binary_stream.hpp"
#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {

QImage read_frame_image(const Frame& frame, QString& error) {
    error.clear();
    if (frame.binary_key_offset >= 0) {
        static thread_local BinaryImageReader binary;
        return binary.read(frame, error);
    }
    if (frame.file_bytes == 0) {
        QImageReader reader(frame.path);
        const auto image = reader.read();
        error = image.isNull() ? reader.errorString() : QString{};
        return image;
    }
    QFile file(frame.path);
    if (!file.open(QIODevice::ReadOnly) || !file.seek(frame.file_offset)) {
        error = file.errorString();
        return {};
    }
    // Only the requested frame enters memory, never the whole recording.
    QByteArray data = file.read(frame.file_bytes);
    if (data.size() != frame.file_bytes) {
        error = QStringLiteral("Incomplete image record");
        return {};
    }
    QBuffer buffer(&data);
    buffer.open(QIODevice::ReadOnly);
    QImageReader reader(&buffer);
    const auto image = reader.read();
    error = image.isNull() ? reader.errorString() : QString{};
    return image;
}

} // namespace dart::tools::video_player
