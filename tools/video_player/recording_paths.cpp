#include "tools/video_player/recording_paths.hpp"

#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>
#include <QString>
#include <QStringList>
#include <algorithm>

#include "tools/video_player/video_stream.hpp"

namespace dart::tools::video_player {

QStringList RecordingPaths::directories(const QString& directory) {
    if (QFileInfo(directory).isFile())
        return {QFileInfo(directory).absoluteFilePath()};
    const QDir input(QFileInfo(directory).absoluteFilePath());
    // A session or flat image directory takes precedence over child sessions.
    if (input.exists("img") || input.exists("frames.csv") || input.exists("logs/frames.csv"))
        return {input.absolutePath()};
    const QRegularExpression image_entry(QStringLiteral(
        "^(?:b[0-9]{7}|part_[0-9]{6}|recording\\.(pgm|pbm|dbv|pngs)|(?:frame_)?[0-9]+\\.(png|pgm|pbm))"
        "$"));
    const QRegularExpression session_name(QStringLiteral("^rec_([0-9]+)$"));
    QStringList sessions;
    for (const auto& entry :
         input.entryInfoList(QDir::AllEntries | QDir::NoDotAndDotDot | QDir::NoSymLinks)) {
        if (image_entry.match(entry.fileName()).hasMatch()
            || (entry.isFile() && VideoStream::supports(entry.suffix())))
            return {input.absolutePath()};
        if (entry.isDir() && session_name.match(entry.fileName()).hasMatch())
            sessions.push_back(entry.absoluteFilePath());
    }
    std::sort(sessions.begin(), sessions.end(), [](const QString& a, const QString& b) {
        const QString first = QFileInfo(a).fileName().mid(4);
        const QString second = QFileInfo(b).fileName().mid(4);
        const auto number = [](QString value) {
            while (value.size() > 1 && value.startsWith('0'))
                value.remove(0, 1);
            return value;
        };
        const QString x = number(first), y = number(second);
        if (x.size() != y.size())
            return x.size() < y.size();
        return x != y ? x < y : a < b;
    });
    return sessions.isEmpty() ? QStringList{input.absolutePath()} : sessions;
}

RecordingPaths RecordingPaths::resolve(const QString& directory) {
    const QFileInfo source(directory);
    const bool is_file = source.isFile();
    const QDir input(is_file ? source.absolutePath() : source.absoluteFilePath());
    const bool has_img = !is_file && QFileInfo(input.filePath("img")).isDir();
    const QDir images(has_img ? input.filePath("img") : input.absolutePath());
    QDir session(input);
    const bool is_img = !has_img && input.dirName() == "img" && !input.exists("frames.csv")
                     && !input.exists("logs/frames.csv");
    if (is_img)
        session.cdUp();
    const bool nested = has_img || is_img;
    const auto first_existing = [](const QStringList& candidates) {
        for (const auto& path : candidates) {
            if (QFileInfo::exists(path))
                return path;
        }
        return candidates.front(); // Keep the expected path in missing-metadata diagnostics.
    };
    const QString local_frames = session.filePath("frames.csv");
    const QString log_frames = session.filePath("logs/frames.csv");
    const QString frames_csv = first_existing(
        nested ? QStringList{log_frames, local_frames} : QStringList{local_frames, log_frames});
    QStringList detection_candidates{session.filePath("detection.csv")};
    if (nested) {
        QDir records(session);
        records.cdUp();
        detection_candidates.push_back(records.filePath("detection.csv"));
    }
    return {
        is_file ? source.absoluteFilePath() : images.absolutePath(), frames_csv,
        first_existing(detection_candidates)};
}

} // namespace dart::tools::video_player
