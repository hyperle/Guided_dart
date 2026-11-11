#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QSlider>
#include <QString>
#include <QStringList>
#include <QTemporaryDir>
#include <algorithm>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <vector>

#include "tools/video_player/image_reader.hpp"
#include "tools/video_player/player.hpp"
#include "tools/video_player/recording.hpp"
#include "tools/video_player/recording_paths.hpp"

namespace dart::tools::video_player::tests {

class VideoStreamTest final {
public:
    static void run() {
        QTemporaryDir directory(QDir::tempPath() + "/dart video streams-XXXXXX");
        require(directory.isValid(), "video fixture directory");
        const auto root = directory.path();
        exercise(root + "/rec_2", "h264", false);
        exercise(root + "/rec_10", "h265", true);
        Recording collection;
        QString error;
        require(
            collection.load_directory(root, {}, {}, {}, error) && collection.complete()
                && collection.frames().size() == 2 * kFrames,
            "video session collection with separate metadata");
        require(
            collection.frames().front().path.contains("/rec_2/")
                && collection.frames()[kFrames].path.contains("/rec_10/"),
            "video sessions sort numerically");
        exercise(root + "/bframes", "h264", true);
        exercise(root + "/main10", "h265", true, "yuv420p10le");
        std::puts("video streams: H.264/H.265, B frames, seeks, CSV, export and GUI passed");
    }

private:
    static constexpr std::size_t kFrames = 16;
    static constexpr int kWidth = 64, kHeight = 48;

    static void require(bool condition, const char* message) {
        if (!condition)
            throw std::runtime_error(message);
    }

    static QByteArray
        process(const QString& executable, const QStringList& arguments, int expected = 0) {
        QProcess command;
        auto environment = QProcessEnvironment::systemEnvironment();
        environment.insert("QT_QPA_PLATFORM", "intentionally_unavailable");
        command.setProcessEnvironment(environment);
        command.start(executable, arguments);
        require(
            command.waitForFinished(30000),
            "video test process finished (requires FFmpeg with libx264/libx265)");
        const auto output = command.readAllStandardOutput();
        const auto errors = command.readAllStandardError();
        if (command.exitStatus() != QProcess::NormalExit || command.exitCode() != expected)
            throw std::runtime_error((executable.toUtf8() + ": " + output + errors).toStdString());
        return output;
    }

    static QByteArray cli(const QStringList& arguments, int expected = 0) {
        return process(
            QCoreApplication::applicationDirPath() + "/video_player", arguments, expected);
    }

    static void write(const QString& path, const QByteArray& data) {
        QFile file(path);
        require(
            file.open(QIODevice::WriteOnly) && file.write(data) == data.size(),
            "write video fixture");
    }

    static void load(Recording& recording, const QString& path, const QString& extension = {}) {
        QString error;
        if (!recording.load_directory(path, {}, {}, extension, error))
            throw std::runtime_error(error.toStdString());
    }

    static QImage read(const Frame& frame) {
        QString error;
        const auto image = read_frame_image(frame, error);
        if (image.isNull() || !error.isEmpty())
            throw std::runtime_error(error.toStdString());
        return image;
    }

    static void exercise(
        const QString& root, const QString& suffix, bool bframes,
        const QString& pixel_format = "yuv420p") {
        require(QDir(root).mkpath("img") && QDir(root).mkpath("logs"), "video session layout");
        QByteArray pixels;
        for (std::size_t frame = 0; frame < kFrames; ++frame)
            for (int row = 0; row < kHeight; ++row)
                for (int column = 0; column < kWidth; ++column) {
                    pixels.append(static_cast<char>((frame * 11 + row * 2 + column * 3) % 256));
                    pixels.append(static_cast<char>((frame * 7 + row * 5 + column) % 256));
                    pixels.append(static_cast<char>((frame * 17 + row + column * 4) % 256));
                }
        const auto raw = root + "/input.rgb";
        write(raw, pixels);
        const auto path = root + "/img/recording." + suffix;
        const bool hevc = suffix == "h265";
        QStringList arguments{
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            "64x48",
            "-framerate",
            "6",
            "-i",
            raw,
            "-frames:v",
            QString::number(kFrames),
            "-c:v",
            hevc ? "libx265" : "libx264",
            "-pix_fmt",
            pixel_format,
            "-colorspace",
            "bt709",
            "-color_primaries",
            "bt709",
            "-color_trc",
            "bt709",
            "-color_range",
            "tv",
            "-preset",
            "medium",
            "-bf",
            bframes ? "2" : "0"};
        arguments << (hevc ? "-x265-params" : "-x264-params")
                  << (hevc ? "keyint=6:min-keyint=6:scenecut=0:b-adapt=0:open-gop=1:pools=none:"
                             "frame-threads=1:log-level=error"
                           : "keyint=6:min-keyint=6:scenecut=0:b-adapt=0:open-gop=0:threads=1");
        arguments << path;
        process("ffmpeg", arguments);
        const auto reference = process(
            "ffmpeg", {"-hide_banner", "-loglevel", "error", "-nostdin", "-i", path, "-sws_flags",
                       "bilinear", "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1"});
        constexpr int picture_bytes = kWidth * kHeight * 3;
        require(
            reference.size() == static_cast<qint64>(kFrames * picture_bytes),
            "reference frame count");

        Recording recording;
        load(recording, path);
        require(
            recording.frames().size() == kFrames && !recording.complete(),
            "video without CSV is viewable");
        const auto check = [&](std::size_t position) {
            const QImage expected(
                reinterpret_cast<const unsigned char*>(reference.constData())
                    + position * picture_bytes,
                kWidth, kHeight, kWidth * 3, QImage::Format_RGB888);
            require(
                read(recording.frames()[position]) == expected,
                "video pixels and presentation order match FFmpeg");
        };
        for (std::size_t position = 0; position < kFrames; ++position)
            check(position);
        for (std::size_t position = kFrames; position > 0; --position)
            check(position - 1);
        for (const auto position : {0U, 15U, 5U, 8U, 3U, 12U, 11U, 1U, 9U})
            check(position);
        check(9);
        bool reordered = false;
        for (std::size_t position = 1; position < kFrames; ++position)
            reordered |= recording.frames()[position].file_offset
                       < recording.frames()[position - 1].file_offset;
        require(reordered == bframes, "fixture covers delayed and reordered frames");

        const QByteArray header =
            "input_index,source_sequence,source_pts,source_pts_timebase_validated,received_us,"
            "decision,image_index,image_file,file_offset,file_bytes\n";
        QByteArray events, detections(
                               "sequence,received_us,source_pts,kind,valid,cx,cy,win_x0,win_y0,win_"
                               "x1,win_y1,admission\n");
        for (std::size_t position = 0; position < kFrames; ++position) {
            const auto& frame = recording.frames()[position];
            events.prepend(QStringLiteral("%1,%1,%1,0,%1,accepted,%2,img/recording.%3,%4,%5\n")
                               .arg(100 + position * 2)
                               .arg(position + 1)
                               .arg(suffix)
                               .arg(frame.file_offset)
                               .arg(frame.file_bytes)
                               .toUtf8());
            detections += QStringLiteral("%1,%1,%1,light,1,2.5,0.5,0,0,64,48,accepted\n")
                              .arg(100 + position * 2)
                              .toUtf8();
        }
        const auto csv = root + "/logs/frames.csv";
        write(csv, header + events);
        write(root + "/detection.csv", detections);
        load(recording, root, suffix);
        require(
            recording.complete() && recording.frames().front().event
                && recording.frames().back().detection,
            "video byte offsets associate shuffled CSV and detections");
        require(
            cli({"--check", "--" + suffix, root})
                .contains("recorder_matches=16 detection_matches=16"),
            "video CLI metadata check");
        cli({"--check", path});
        cli({"--check", root + "/img"});
        cli({"--check", "--h264", "--h265", root}, 2);
        cli({"--check", hevc ? "--h264" : "--h265", path}, 1);
        const auto exported = root + "/exported";
        cli({"--export-png", exported, path});
        for (std::size_t position = 0; position < kFrames; ++position)
            require(
                QImage(exported + QStringLiteral("/%1.png").arg(position + 1, 8, 10, QChar('0')))
                        .convertToFormat(QImage::Format_RGB888)
                    == read(recording.frames()[position]),
                "video PNG export preserves decoded pixels");
        cli({"--export-png", exported, path}, 1);

        Player player(recording);
        auto* slider = player.findChild<QSlider*>("frame_slider");
        auto* info = player.findChild<QPlainTextEdit*>("frame_info");
        require(
            slider && info && slider->maximum() == static_cast<int>(kFrames - 1),
            "video GUI controls");
        for (int position : {15, 3, 8, 0}) {
            slider->setValue(position);
            require(
                !info->toPlainText().contains("IMAGE ERROR")
                    && info->toPlainText().contains(
                        QStringLiteral("source_sequence: %1").arg(100 + position * 2)),
                "video GUI seeks keep image and metadata aligned");
        }
        auto conflict = events;
        const auto& first = recording.frames().front();
        conflict.replace(
            QByteArray::number(first.file_offset) + ',' + QByteArray::number(first.file_bytes)
                + '\n',
            QByteArray::number(first.file_offset) + ',' + QByteArray::number(first.file_bytes + 1)
                + '\n');
        write(csv, header + conflict);
        load(recording, path);
        require(
            !recording.complete() && !recording.frames().front().event
                && !recording.frames().front().detection,
            "video byte-size conflict disables metadata association");
        write(csv, header + events);

        const auto flat = root + "/flat";
        require(QDir().mkpath(flat + "/rec_99"), "flat video with child session");
        const auto alias = flat + (hevc ? "/clip with spaces.HEVC" : "/clip with spaces.H264");
        require(QFile::copy(path, alias), "copy video alias");
        require(
            RecordingPaths::directories(flat) == QStringList{flat},
            "flat video precedes child sessions");
        load(recording, flat, suffix);
        require(recording.frames().size() == kFrames, "uppercase video extensions and HEVC alias");
        cli({"--export-png", root + "/alias-export", alias, "", ""});
        const auto cached = recording.frames().front();
        read(cached);
        write(alias, "not a video");
        QString error;
        require(
            read_frame_image(cached, error).isNull() && error.contains("changed"),
            "modified video invalidates cached images");
        require(
            !recording.load_directory(alias, {}, {}, {}, error) && !error.isEmpty(),
            "corrupt video reports an error");
        write(alias, {});
        require(
            !recording.load_directory(alias, {}, {}, {}, error), "empty video reports an error");
    }
};

} // namespace dart::tools::video_player::tests

int main(int argc, char** argv) {
    QApplication application(argc, argv);
    try {
        dart::tools::video_player::tests::VideoStreamTest::run();
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
