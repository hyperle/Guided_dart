module;
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QString>
#include <cstddef>
#include <cstdint>
#include <span>

export module dart.tools.video_player.binary_stream;

import dart.recorder.encoding.binary_decoder;
import dart.recorder.encoding.binary_format;
import dart.tools.video_player.recording_types;

export namespace dart::tools::video_player {

struct BinaryStream final {
    static std::span<const std::byte> bytes(const QByteArray& data) {
        return {reinterpret_cast<const std::byte*>(data.constData()),
                static_cast<std::size_t>(data.size())};
    }
    // Leaves the file positioned at the payload. Zero means a truncated tail.
    static int prefix(QFile& file, quint32& code) {
        const auto data = file.peek(5);
        const auto size = recorder::BinaryFormat::parse(bytes(data), code);
        if (size <= 0)
            return size;
        if (!file.seek(file.pos() + size))
            return -1;
        return size;
    }
};

// One decoded frame is cached. Forward browsing continues from that frame;
// backwards/random seeks replay from the nearest independent full record.
class BinaryImageReader final {
public:
    QImage read(const Frame& frame, QString& error) {
        QFile file(frame.path);
        if (!file.open(QIODevice::ReadOnly)) {
            error = file.errorString();
            return {};
        }
        const QFileInfo info(file);
        const auto modified = info.lastModified().toMSecsSinceEpoch();
        const bool reusable = path_ == frame.path && size_ == file.size() && modified_ == modified
            && width_ == frame.binary_width && height_ == frame.binary_height
            && key_ == frame.binary_key_offset && decoded_ <= frame.binary_decode_offset;
        if (!reusable) {
            path_.clear();
            if (!decoder_.prepare(frame.binary_width, frame.binary_height)) {
                error = QStringLiteral("Invalid binary geometry or insufficient memory");
                return {};
            }
            next_ = frame.binary_key_offset;
            decoded_ = -1;
        }
        if (!file.seek(next_)) {
            error = file.errorString();
            return {};
        }
        while (decoded_ != frame.binary_decode_offset) {
            const auto offset = file.pos();
            quint32 code = 0;
            if (offset > frame.binary_decode_offset || BinaryStream::prefix(file, code) <= 0
                || (code >> 2U) > recorder::BinaryFormat::packed_bytes(
                       frame.binary_width, frame.binary_height))
                return failed(error);
            const auto payload = file.read(code >> 2U);
            if (!decoder_.decode(code, BinaryStream::bytes(payload)))
                return failed(error);
            decoded_ = offset;
            next_ = file.pos();
        }
        path_ = frame.path;
        size_ = file.size();
        modified_ = modified;
        key_ = frame.binary_key_offset;
        width_ = frame.binary_width;
        height_ = frame.binary_height;
        QImage image(static_cast<int>(width_), static_cast<int>(height_), QImage::Format_Grayscale8);
        if (image.isNull()) {
            error = QStringLiteral("Insufficient memory for binary image");
            return {};
        }
        const auto bits = decoder_.bits();
        const auto stride = (static_cast<std::size_t>(width_) + 7U) / 8U;
        for (quint32 y = 0; y < height_; ++y) {
            auto* row = image.scanLine(static_cast<int>(y));
            for (quint32 x = 0; x < width_; ++x)
                row[x] = (std::to_integer<unsigned>(bits[y * stride + x / 8U])
                          & (128U >> (x % 8U))) ? 255 : 0;
        }
        return image;
    }

private:
    QImage failed(QString& error) {
        path_.clear();
        error = QStringLiteral("Invalid, incomplete or corrupt binary image record");
        return {};
    }
    recorder::BinaryDecoder decoder_{};
    QString path_;
    qint64 size_ = -1, modified_ = -1, key_ = -1, decoded_ = -1, next_ = -1;
    quint32 width_ = 0, height_ = 0;
};

} // namespace dart::tools::video_player
