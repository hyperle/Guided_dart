#pragma once

#include <QString>
#include <QStringList>
#include <optional>
#include <vector>

#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {
bool split_csv(const QString& line, QStringList& fields);

bool read_csv(
    const QString& path, const QStringList& required, std::vector<CsvRow>& rows, QString& error);

std::optional<quint64> unsigned_value(const QString& text);

std::optional<QString> frame_identity(const CsvRow& row, const QString& sequence);

} // namespace dart::tools::video_player
