module;
#include <QFile>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <algorithm>
#include <optional>
#include <utility>
#include <vector>

export module dart.tools.video_player.csv;

import dart.tools.video_player.recording_types;

export namespace dart::tools::video_player {

bool split_csv(const QString& line, QStringList& fields) {
    QString field;
    bool quoted = false;
    bool closed = false;
    for (int i = 0; i < line.size(); ++i) {
        const QChar c = line.at(i);
        if (quoted) {
            if (c != '"') {
                field += c;
            } else if (i + 1 < line.size() && line.at(i + 1) == '"') {
                field += '"';
                ++i;
            } else {
                quoted = false;
                closed = true;
            }
        } else if (c == ',') {
            fields.push_back(field);
            field.clear();
            closed = false;
        } else if (c == '"') {
            if (!field.isEmpty() || closed)
                return false;
            quoted = true;
        } else {
            if (closed)
                return false;
            field += c;
        }
    }
    fields.push_back(field);
    return !quoted;
}

bool read_csv(
    const QString& path, const QStringList& required, std::vector<CsvRow>& rows, QString& error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error = QStringLiteral("%1: %2").arg(path, file.errorString());
        return false;
    }
    QTextStream input(&file);
    QStringList headers;
    int line_number = 0;
    while (!input.atEnd()) {
        const QString line = input.readLine();
        ++line_number;
        const QString trimmed = line.trimmed();
        if (trimmed.isEmpty() || trimmed.startsWith('#'))
            continue;
        QStringList fields;
        if (!split_csv(line, fields)) {
            error = QStringLiteral("%1:%2: malformed CSV quoting").arg(path).arg(line_number);
            return false;
        }
        if (headers.isEmpty()) {
            for (const QString& name : fields) {
                if (name.isEmpty() || headers.contains(name)) {
                    error =
                        QStringLiteral("%1:%2: empty/duplicate column").arg(path).arg(line_number);
                    return false;
                }
                headers.push_back(name);
            }
            for (const QString& name : required) {
                if (!headers.contains(name)) {
                    error = QStringLiteral("%1:%2: missing column %3 (v2 CSV required)")
                                .arg(path)
                                .arg(line_number)
                                .arg(name);
                    return false;
                }
            }
            continue;
        }
        // DetectionCsv::row(nullptr) emits a short empty payload. Its first five
        // fields and final admission are still unambiguous. Preserve the raw line.
        if (fields.size() < headers.size() && fields.size() > 5 && headers.value(3) == "kind"
            && headers.value(4) == "valid" && headers.last() == "admission"
            && fields.at(3) == "none" && fields.at(4) == "0"
            && std::all_of(fields.cbegin() + 5, fields.cend() - 1, [](const QString& field) {
                   return field.isEmpty();
               })) {
            const QString admission = fields.takeLast();
            while (fields.size() < headers.size() - 1)
                fields.push_back({});
            fields.push_back(admission);
        }
        if (fields.size() != headers.size()) {
            error = QStringLiteral("%1:%2: expected %3 fields, got %4")
                        .arg(path)
                        .arg(line_number)
                        .arg(headers.size())
                        .arg(fields.size());
            return false;
        }
        CsvRow row;
        row.path = path;
        row.line = line_number;
        row.raw = line;
        for (int i = 0; i < headers.size(); ++i)
            row.values.insert(headers.at(i), fields.at(i));
        rows.push_back(std::move(row));
    }
    if (input.status() != QTextStream::Ok) {
        error = path + QStringLiteral(": CSV read error");
        return false;
    }
    return true;
}

std::optional<quint64> unsigned_value(const QString& text) {
    if (text.isEmpty()
        || std::any_of(text.cbegin(), text.cend(), [](QChar c) { return c < '0' || c > '9'; }))
        return std::nullopt;
    bool ok = false;
    const quint64 number = text.toULongLong(&ok);
    return ok ? std::optional<quint64>{number} : std::nullopt;
}

std::optional<QString> frame_identity(const CsvRow& row, const QString& sequence) {
    const auto seq = unsigned_value(row.value(sequence));
    const auto received = unsigned_value(row.value("received_us"));
    const auto pts = unsigned_value(row.value("source_pts"));
    if (!seq || !received || !pts)
        return std::nullopt;
    return QStringLiteral("%1/%2/%3").arg(*seq).arg(*received).arg(*pts);
}

} // namespace dart::tools::video_player
