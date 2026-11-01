#pragma once

#include <QFile>
#include <QImage>
#include <QString>
#include <cstddef>
#include <cstdint>
#include <span>

#include "recorder/encoding/binary_decoder.hpp"
#include "recorder/encoding/binary_format.hpp"
#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {
struct BinaryStream final {
    static std::span<const std::byte> bytes(const QByteArray& data);
    // Leaves the file positioned at the payload. Zero means a truncated tail.
    static int prefix(QFile& file, quint32& code);
};

// One decoded frame is cached. Forward browsing continues from that frame;
// backwards/random seeks replay from the nearest independent full record.
class BinaryImageReader final {
public:
    QImage read(const Frame& frame, QString& error);

private:
    QImage failed(QString& error);
    recorder::BinaryDecoder decoder_{};
    QString path_;
    qint64 size_ = -1, modified_ = -1, key_ = -1, decoded_ = -1, next_ = -1;
    quint32 width_ = 0, height_ = 0;
};

} // namespace dart::tools::video_player
