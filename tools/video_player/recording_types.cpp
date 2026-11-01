#include "tools/video_player/recording_types.hpp"

#include <QFileInfo>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QtGlobal>
#include <optional>

namespace dart::tools::video_player {

QString CsvRow::text() const {
    QString result = location() + '\n' + raw + '\n';
    for (auto it = values.cbegin(); it != values.cend(); ++it)
        result += QStringLiteral("%1: %2\n").arg(it.key(), it.value());
    return result;
}

QString Frame::key() const {
    const QString name = relative_name.isEmpty()
                           ? QStringLiteral("img/") + QFileInfo(path).fileName()
                           : relative_name;
    return file_bytes == 0 ? name : name + '@' + QString::number(file_offset);
}

} // namespace dart::tools::video_player
