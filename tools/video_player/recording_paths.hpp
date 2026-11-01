#pragma once

#include <QString>
#include <QStringList>

namespace dart::tools::video_player {
// Paths on the host may differ from the session path recorded on the board.
struct RecordingPaths {
    QString images;
    QString frames_csv;
    QString detection_csv;

    static RecordingPaths resolve(const QString& directory);
    static QStringList directories(const QString& directory);
};

} // namespace dart::tools::video_player
