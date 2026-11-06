#include "tools/video_player/recording.hpp"

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QString>
#include <QStringList>
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "recorder/encoding/binary_format.hpp"
#include "tools/video_player/binary_stream.hpp"
#include "tools/video_player/csv.hpp"
#include "tools/video_player/recording_paths.hpp"
#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {

QString Recording::summary() const {
    return QStringLiteral(
               "images=%1 recorder_matches=%2 detection_matches=%3 "
               "missing_images=%4")
        .arg(frames_.size())
        .arg(event_matches_)
        .arg(detection_matches_)
        .arg(missing_images_);
}

bool Recording::complete() const {
    return event_matches_ == frames_.size() && missing_images_ == 0
        && detection_matches_ == required_detection_matches_;
}

bool Recording::load_directory(
    const QString& directory, const std::optional<QString>& frames_csv,
    const std::optional<QString>& detection_csv, const QString& extension, QString& error) {
    *this = Recording{};
    error.clear();
    const auto directories = RecordingPaths::directories(directory);
    if (directories.size() > 1 && frames_csv && !frames_csv->isEmpty()) {
        error = QStringLiteral(
            "Multiple sessions use separate frames.csv files; "
            "select one rec_NNNN directory to override its recorder CSV.");
        return false;
    }
    for (const auto& session : directories) {
        const auto paths = RecordingPaths::resolve(session);
        Recording recording;
        if (!recording.load(
                paths.images, frames_csv.value_or(paths.frames_csv),
                detection_csv.value_or(paths.detection_csv), extension, error))
            return false;
        if (directories.size() > 1)
            notes_ << session + ": " + recording.summary();
        notes_ << recording.notes_;
        event_matches_ += recording.event_matches_;
        detection_matches_ += recording.detection_matches_;
        missing_images_ += recording.missing_images_;
        required_detection_matches_ += recording.required_detection_matches_;
        frames_.insert(
            frames_.end(), std::make_move_iterator(recording.frames_.begin()),
            std::make_move_iterator(recording.frames_.end()));
    }
    return true;
}

bool Recording::append_binary(const QFileInfo& file_info, QString& error) {
    QFile file(file_info.absoluteFilePath());
    if (!file.open(QIODevice::ReadOnly)) {
        error = file_info.absoluteFilePath() + ": " + file.errorString();
        return false;
    }
    const auto header = file.read(recorder::BinaryFormat::kHeaderBytes);
    if (header.size() < static_cast<int>(recorder::BinaryFormat::kHeaderBytes)) {
        notes_ << file_info.absoluteFilePath()
                      + QStringLiteral(": incomplete binary header ignored");
        return true;
    }
    quint32 width = 0, height = 0;
    if (!recorder::BinaryFormat::geometry(BinaryStream::bytes(header), width, height)) {
        error = file_info.absoluteFilePath() + QStringLiteral(": invalid binary recording header");
        return false;
    }
    const auto packed = recorder::BinaryFormat::packed_bytes(width, height);
    quint64 index = 1;
    qint64 key = -1, changed = -1;
    while (file.pos() < file.size()) {
        const auto offset = file.pos();
        quint32 code = 0;
        const auto prefix = BinaryStream::prefix(file, code);
        if (prefix == 0 || (prefix > 0 && (code >> 2U) > file.size() - file.pos())) {
            notes_ << file_info.absoluteFilePath()
                          + QStringLiteral(": incomplete final frame ignored");
            break;
        }
        if (prefix < 0 || !recorder::BinaryFormat::valid(code, packed, key >= 0)) {
            error = QStringLiteral("%1:%2: invalid binary frame header")
                        .arg(file_info.absoluteFilePath())
                        .arg(offset);
            return false;
        }
        const auto kind = code & 3U;
        if (kind == recorder::BinaryFormat::kRaw || kind == recorder::BinaryFormat::kFull)
            key = offset;
        if (code != 0)
            changed = offset;
        Frame frame;
        frame.image_index = index++;
        frame.path = file_info.absoluteFilePath();
        frame.file_offset =
            offset == static_cast<qint64>(recorder::BinaryFormat::kHeaderBytes) ? 0 : offset;
        frame.file_bytes = offset + prefix + (code >> 2U) - frame.file_offset;
        frame.binary_key_offset = key;
        frame.binary_decode_offset = changed;
        frame.binary_width = width;
        frame.binary_height = height;
        frames_.push_back(std::move(frame));
        if (!file.seek(file.pos() + (code >> 2U))) {
            error = file.errorString();
            return false;
        }
    }
    return true;
}

bool Recording::append_stream(const QFileInfo& file_info, QString& error) {
    QFile file(file_info.absoluteFilePath());
    if (!file.open(QIODevice::ReadOnly)) {
        error = file_info.absoluteFilePath() + ": " + file.errorString();
        return false;
    }
    if (file.size() == 0)
        return true;
    const QByteArray magic = file.readLine(64);
    const QByteArray dimensions = file.readLine(64);
    const bool pbm = file_info.suffix() == "pbm";
    const QByteArray maximum = pbm ? QByteArray{} : file.readLine(64);
    const auto match = QRegularExpression(QStringLiteral("^([0-9]+) ([0-9]+)\\n$"))
                           .match(QString::fromLatin1(dimensions));
    const auto width = unsigned_value(match.captured(1));
    const auto height = unsigned_value(match.captured(2));
    if (magic != (pbm ? "P4\n" : "P5\n") || !match.hasMatch() || !width || !height || *width == 0
        || *height == 0 || *width > 16384 || *height > 16384 || (!pbm && maximum != "255\n")) {
        error = file_info.absoluteFilePath() + QStringLiteral(": invalid recording header");
        return false;
    }
    const QByteArray header = magic + dimensions + maximum;
    const auto payload = (pbm ? (*width + 7) / 8 : *width) * *height;
    const auto record_bytes = static_cast<qint64>(payload + header.size());
    quint64 index = 1;
    qint64 offset = 0;
    for (; record_bytes <= file.size() - offset; offset += record_bytes, ++index) {
        if (!file.seek(offset) || file.read(header.size()) != header) {
            error = QStringLiteral("%1:%2: inconsistent image header")
                        .arg(file_info.absoluteFilePath())
                        .arg(offset);
            return false;
        }
        Frame frame;
        frame.image_index = index;
        frame.path = file_info.absoluteFilePath();
        frame.file_offset = offset;
        frame.file_bytes = record_bytes;
        frames_.push_back(std::move(frame));
    }
    if (offset != file.size())
        notes_ << file_info.absoluteFilePath()
                      + QStringLiteral(
                          ": incomplete final frame ignored (possible reset during write)");
    return true;
}

bool Recording::append_png_stream(const QFileInfo& file_info, QString& error) {
    QFile file(file_info.absoluteFilePath());
    if (!file.open(QIODevice::ReadOnly)) {
        error = file.errorString();
        return false;
    }
    const auto header = file.read(16);
    const auto le32 = [](const char* data) {
        quint32 value = 0;
        for (unsigned i = 0; i < 4; ++i)
            value |= static_cast<quint32>(static_cast<unsigned char>(data[i])) << (8U * i);
        return value;
    };
    const auto be32 = [](const char* data) {
        quint32 value = 0;
        for (unsigned i = 0; i < 4; ++i)
            value = (value << 8U) | static_cast<unsigned char>(data[i]);
        return value;
    };
    if (header.size() != 16 || header.left(8) != QByteArray("DPNG\1\0\20\0", 8)) {
        error = file_info.absoluteFilePath() + ": invalid DPNG header";
        return false;
    }
    const auto width = le32(header.constData() + 8), height = le32(header.constData() + 12);
    if (width == 0 || height == 0 || width > 16384 || height > 16384) {
        error = file_info.absoluteFilePath() + ": invalid DPNG geometry";
        return false;
    }
    quint64 index = 1;
    while (file.pos() < file.size()) {
        const auto offset = file.pos();
        const auto signature = file.read(8);
        bool complete = false, first = true;
        if (signature.size() == 8 && signature != QByteArray("\x89PNG\r\n\x1a\n", 8)) {
            error = file_info.absoluteFilePath() + ": invalid PNG signature";
            return false;
        }
        while (signature.size() == 8 && file.pos() < file.size()) {
            const auto chunk = file.read(8);
            if (chunk.size() != 8)
                break;
            const auto size = be32(chunk.constData());
            if (static_cast<qint64>(size) + 4 > file.size() - file.pos())
                break;
            const auto kind = chunk.mid(4, 4);
            if (first) {
                if (kind != "IHDR" || size != 13) {
                    error = file_info.absoluteFilePath() + ": invalid PNG IHDR";
                    return false;
                }
                const auto geometry = file.read(13);
                if (geometry.size() != 13 || be32(geometry.constData()) != width
                    || be32(geometry.constData() + 4) != height) {
                    error = file_info.absoluteFilePath() + ": PNG geometry differs from DPNG";
                    return false;
                }
                if (!file.seek(file.pos() + 4))
                    break;
                first = false;
            } else if (!file.seek(file.pos() + static_cast<qint64>(size) + 4)) {
                break;
            }
            if (kind == "IEND" && size == 0) {
                complete = true;
                break;
            }
        }
        if (!complete) {
            notes_ << file_info.absoluteFilePath() + ": incomplete final PNG ignored";
            break;
        }
        Frame frame;
        frame.image_index = index++;
        frame.path = file_info.absoluteFilePath();
        frame.file_offset = offset;
        frame.file_bytes = file.pos() - offset;
        frames_.push_back(std::move(frame));
    }
    return true;
}

bool Recording::load(
    const QString& image_directory, const QString& frames_csv, const QString& detection_csv,
    const QString& extension, QString& error) {
    *this = Recording{};
    error.clear();
    const QDir image_dir(image_directory);
    if (!image_dir.exists()) {
        error = image_directory + QStringLiteral(": image directory does not exist");
        return false;
    }
    const QRegularExpression filename(QStringLiteral(
        "^(?:(?:part_[0-9]{6}|b[0-9]{7}/p[0-9]{7})/)?(?:frame_)?([0-9]+)\\.(pgm|pbm|png)$"));
    QDirIterator files(
        image_dir.absolutePath(), QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
    while (files.hasNext()) {
        files.next();
        const QFileInfo file = files.fileInfo();
        const QString relative = image_dir.relativeFilePath(file.absoluteFilePath());
        if (!extension.isEmpty() && file.suffix() != extension)
            continue;
        if (relative == "recording.dbv") {
            if (!append_binary(file, error))
                return false;
            continue;
        }
        if (relative == "recording.pngs") {
            if (!append_png_stream(file, error))
                return false;
            continue;
        }
        if (relative == "recording.pgm" || relative == "recording.pbm") {
            if (!append_stream(file, error))
                return false;
            continue;
        }
        const auto match = filename.match(relative);
        const auto index = unsigned_value(match.captured(1));
        if (!match.hasMatch() || !index || *index == 0
            || (!extension.isEmpty() && file.suffix() != extension))
            continue;
        Frame frame;
        frame.image_index = *index;
        frame.path = file.absoluteFilePath();
        frame.relative_name = QStringLiteral("img/") + relative;
        frames_.push_back(std::move(frame));
    }
    std::sort(frames_.begin(), frames_.end(), [](const Frame& a, const Frame& b) {
        return a.image_index != b.image_index ? a.image_index < b.image_index : a.path < b.path;
    });
    if (frames_.empty()) {
        error = image_directory + QStringLiteral(": no DBV/PNG/PGM/PBM frames found");
        return false;
    }

    std::vector<CsvRow> events;
    if (frames_csv.isEmpty()) {
        notes_ << QStringLiteral("No recorder CSV; images have no recorder metadata.");
    } else if (!QFileInfo::exists(frames_csv)) {
        notes_ << frames_csv + QStringLiteral(": missing; images have no recorder metadata");
    } else if (!read_csv(
                   frames_csv,
                   {"input_index", "source_sequence", "source_pts", "received_us", "decision",
                    "image_index", "image_file"},
                   events, error)) {
        return false;
    } else if (events.empty()) {
        notes_ << frames_csv + QStringLiteral(": no data rows; images have no recorder metadata");
    }
    std::map<QString, std::vector<const CsvRow*>> by_image;
    const QRegularExpression image_name(QStringLiteral(
        "^img/((?:(?:part_[0-9]{6}|b[0-9]{7}/p[0-9]{7})/"
        ")?(?:frame_)?[0-9]+\\.(pgm|pbm|png)|recording\\.(pgm|pbm|dbv|pngs))$"));
    for (const CsvRow& row : events) {
        if (!frame_identity(row, "source_sequence")) {
            error = row.location() + QStringLiteral(": invalid frame identity");
            return false;
        }
        const QString name = row.value("image_file");
        if (name.isEmpty())
            continue; // Downsampled/busy inputs are not pictures.
        if (!image_name.match(name).hasMatch()) {
            error = row.location() + QStringLiteral(": invalid image_file: ") + name;
            return false;
        }
        QString key = name;
        if (name == "img/recording.pgm" || name == "img/recording.pbm"
            || name == "img/recording.dbv" || name == "img/recording.pngs") {
            const auto offset = unsigned_value(row.value("file_offset"));
            const auto bytes = unsigned_value(row.value("file_bytes"));
            if (!offset || !bytes || *bytes == 0) {
                error = row.location() + QStringLiteral(": invalid stream offset/size");
                return false;
            }
            key += '@' + QString::number(*offset);
        }
        if (extension.isEmpty() || name.endsWith('.' + extension))
            by_image[key].push_back(&row);
    }
    std::set<QString> disk_images;
    for (const Frame& frame : frames_)
        disk_images.insert(frame.key());
    for (const auto& entry : by_image) {
        if (!disk_images.contains(entry.first))
            ++missing_images_;
    }

    std::vector<CsvRow> detections;
    bool has_detection = !detection_csv.isEmpty() && QFileInfo::exists(detection_csv);
    if (!has_detection) {
        notes_ << QStringLiteral("No detection CSV; target/scan overlays unavailable.");
    } else if (!read_csv(
                   detection_csv,
                   {"sequence", "received_us", "source_pts", "kind", "valid", "cx", "cy", "win_x0",
                    "win_y0", "win_x1", "win_y1", "admission"},
                   detections, error)) {
        return false;
    } else if (detections.empty()) {
        has_detection = false;
        notes_ << detection_csv
                      + QStringLiteral(": no data rows; target/scan overlays unavailable.");
    }
    std::map<QString, std::vector<const CsvRow*>> by_identity;
    for (const CsvRow& row : detections) {
        const auto identity = frame_identity(row, "sequence");
        if (!identity) {
            error = row.location() + QStringLiteral(": invalid frame identity");
            return false;
        }
        by_identity[*identity].push_back(&row);
    }
    for (Frame& frame : frames_) {
        const auto event_it = by_image.find(frame.key());
        if (event_it == by_image.end() || event_it->second.size() != 1) {
            const QString reason =
                event_it == by_image.end() ? "no matching image_file" : "ambiguous image_file";
            frame.notes << QStringLiteral("Recorder: %1; no detection association.").arg(reason);
            continue;
        }
        const CsvRow& event = *event_it->second.front();
        const auto index = unsigned_value(event.value("image_index"));
        if (event.value("decision") != "accepted" || !index || *index != frame.image_index) {
            frame.notes << event.location() + QStringLiteral(": decision/image_index conflict");
            continue;
        }
        if (frame.file_bytes != 0
            && unsigned_value(event.value("file_bytes"))
                   != static_cast<quint64>(frame.file_bytes)) {
            frame.notes << event.location() + QStringLiteral(": stream record size conflict");
            continue;
        }
        if (frame.file_bytes == 0 && frame.path.endsWith(".png")
            && (unsigned_value(event.value("file_offset")) != quint64{0}
                || unsigned_value(event.value("file_bytes"))
                       != static_cast<quint64>(QFileInfo(frame.path).size()))) {
            frame.notes << event.location() + QStringLiteral(": PNG offset/size conflict");
            continue;
        }
        frame.event = event;
        ++event_matches_;
        if (!has_detection)
            continue;
        ++required_detection_matches_;
        const auto it = by_identity.find(*frame_identity(event, "source_sequence"));
        if (it == by_identity.end() || it->second.size() != 1) {
            frame.notes << QStringLiteral("Detection: %1 for sequence + received_us + source_pts.")
                               .arg(it == by_identity.end() ? "no exact match" : "ambiguous rows");
        } else if (it->second.front()->value("admission") != "accepted") {
            frame.notes << it->second.front()->location()
                               + QStringLiteral(": detection admission conflicts with recorder");
        } else {
            frame.detection = *it->second.front();
            ++detection_matches_;
        }
    }
    if (missing_images_ != 0)
        notes_ << QStringLiteral("%1 CSV image_file entries have no image on disk.")
                      .arg(missing_images_);
    return true;
}

} // namespace dart::tools::video_player
