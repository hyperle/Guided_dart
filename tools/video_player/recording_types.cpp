module;
#include <QFileInfo>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QtGlobal>
#include <optional>

export module dart.tools.video_player.recording_types;



export namespace dart::tools::video_player {

struct CsvRow {
    QMap<QString, QString> values;
    QString path;
    QString raw;
    int line = 0;

    QString value(const QString& name) const { return values.value(name); }
    QString location() const { return QStringLiteral("%1:%2").arg(path).arg(line); }
    QString text() const {
        QString result = location() + '\n' + raw + '\n';
        for (auto it = values.cbegin(); it != values.cend(); ++it)
            result += QStringLiteral("%1: %2\n").arg(it.key(), it.value());
        return result;
    }
};

// image_index is an output counter, never a CSV line or source sequence.
struct Frame {
    quint64 image_index = 0;
    QString path;
    QString relative_name;
    qint64 file_offset = 0;
    qint64 file_bytes = 0; // Nonzero for one complete record within an image stream.
    qint64 binary_key_offset = -1;
    qint64 binary_decode_offset = -1; // Repeat frames reuse the last changed frame.
    quint32 binary_width = 0, binary_height = 0;
    std::optional<CsvRow> event;
    std::optional<CsvRow> detection;
    QStringList notes;

    QString key() const {
        const QString name = relative_name.isEmpty()
                               ? QStringLiteral("img/") + QFileInfo(path).fileName()
                               : relative_name;
        return file_bytes == 0 ? name : name + '@' + QString::number(file_offset);
    }
};

} // namespace dart::tools::video_player
