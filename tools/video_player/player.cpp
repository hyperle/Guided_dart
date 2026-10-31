module;
#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QFileDialog>
#include <QFileInfo>
#include <QImageReader>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QTextStream>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

export module dart.tools.video_player.player;

import dart.tools.video_player.image_reader;
export import dart.tools.video_player.image_view;
export import dart.tools.video_player.recording;
import dart.tools.video_player.recording_paths;

export namespace dart::tools::video_player {

class Player final : public QMainWindow {
public:
    explicit Player(Recording recording)
        : recording_(std::move(recording)) {
        setWindowTitle(QStringLiteral("Dart recording player"));
        resize(1200, 760);
        auto* central = new QWidget(this);
        auto* layout = new QVBoxLayout(central);
        auto* summary = new QLabel(recording_.summary(), central);
        summary->setTextFormat(Qt::PlainText);
        summary->setTextInteractionFlags(Qt::TextSelectableByMouse);
        layout->addWidget(summary);
        auto* splitter = new QSplitter(Qt::Horizontal, central);
        view_ = new ImageView(splitter);
        view_->setObjectName(QStringLiteral("image_view"));
        info_ = new QPlainTextEdit(splitter);
        info_->setObjectName(QStringLiteral("frame_info"));
        info_->setReadOnly(true);
        info_->setMinimumWidth(310);
        splitter->addWidget(view_);
        splitter->addWidget(info_);
        splitter->setStretchFactor(0, 3);
        splitter->setStretchFactor(1, 2);
        layout->addWidget(splitter, 1);
        slider_ = new QSlider(Qt::Horizontal, central);
        slider_->setObjectName(QStringLiteral("frame_slider"));
        slider_->setRange(0, static_cast<int>(recording_.frames().size()) - 1);
        layout->addWidget(slider_);
        auto* export_button = new QPushButton(QStringLiteral("Export current frame as PNG"), central);
        layout->addWidget(export_button);
        connect(export_button, &QPushButton::clicked, this, [this] {
            const auto& frame = recording_.frames().at(static_cast<std::size_t>(slider_->value()));
            const auto destination = QFileDialog::getSaveFileName(
                this, QStringLiteral("Export frame"),
                QStringLiteral("%1.png").arg(frame.image_index, 8, 10, QChar('0')),
                QStringLiteral("PNG images (*.png)"));
            if (destination.isEmpty())
                return;
            QString error;
            const auto image = read_frame_image(frame, error);
            const bool saved = !image.isNull() && image.save(destination, "PNG");
            statusBar()->showMessage(saved ? QStringLiteral("Saved ") + destination
                                          : QStringLiteral("Export failed: ") + error);
        });
        setCentralWidget(central);
        connect(slider_, &QSlider::valueChanged, this, [this](int value) { show_frame(value); });
        view_->clicked = [this](int x, int y) {
            const QString pixel = QStringLiteral("pixel (%1, %2)").arg(x).arg(y);
            info_->setPlainText(details_ + QStringLiteral("\nClicked ") + pixel);
            statusBar()->showMessage(pixel);
        };
        for (const auto key : {Qt::Key_Left, Qt::Key_Right, Qt::Key_Home, Qt::Key_End}) {
            auto* shortcut = new QShortcut(QKeySequence(key), this);
            connect(shortcut, &QShortcut::activated, this, [this, key] {
                int value = slider_->value();
                if (key == Qt::Key_Home)
                    value = slider_->minimum();
                else if (key == Qt::Key_End)
                    value = slider_->maximum();
                else
                    value += key == Qt::Key_Left ? -1 : 1;
                slider_->setValue(std::clamp(value, slider_->minimum(), slider_->maximum()));
            });
        }
        show_frame(0);
    }

private:
    void show_frame(int index) {
        if (index < 0 || index >= static_cast<int>(recording_.frames().size()))
            return;
        const auto& frame = recording_.frames().at(static_cast<std::size_t>(index));
        details_ = QStringLiteral("Frame %1/%2\nimage_index: %3\nfile: %4\n")
                       .arg(index + 1)
                       .arg(recording_.frames().size())
                       .arg(frame.image_index)
                       .arg(frame.path);
        if (!recording_.notes().isEmpty())
            details_ += '\n' + recording_.notes().join('\n') + '\n';
        if (!frame.notes.isEmpty())
            details_ += '\n' + frame.notes.join('\n') + '\n';
        if (frame.event)
            details_ += QStringLiteral("\nRECORDER\n") + frame.event->text();
        if (frame.detection)
            details_ +=
                QStringLiteral("\nDETECTION (exact frame identity)\n") + frame.detection->text();

        QString image_error;
        const QImage image = read_frame_image(frame, image_error);
        if (image.isNull()) {
            view_->set_frame({}); // Never retain the previous image under the next frame's log.
            details_ += QStringLiteral("\nIMAGE ERROR: ") + image_error;
        } else {
            view_->set_frame(image, ImageView::overlay_for(frame, image.size()));
            details_ += QStringLiteral("\nsize: %1 x %2\nGreen: valid target; cyan: scan window.")
                            .arg(image.width())
                            .arg(image.height());
        }
        info_->setPlainText(details_);
        statusBar()->showMessage(QStringLiteral("Frame %1/%2 — Left/Right, Home/End; click pixel")
                                     .arg(index + 1)
                                     .arg(recording_.frames().size()));
    }

    Recording recording_;
    ImageView* view_ = nullptr;
    QPlainTextEdit* info_ = nullptr;
    QSlider* slider_ = nullptr;
    QString details_;
};

int run_application(int argc, char** argv) {
    // Parse/help/check need no window system. Construct QApplication only for the viewer.
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral(
        "Host frame viewer for compact DBV and PNG/PGM/PBM recordings and recorder CSV.\n"
        "Accepts a directory of rec_NNNN sessions, a single session, or an image directory.\n"
        "Port of k230_v1 tools/video_player; manual stepping, no timed playback."));
    parser.addHelpOption();
    parser.addOption(QCommandLineOption({"f", "pbm"}, "Show only frame_*.pbm."));
    parser.addOption(QCommandLineOption({"r", "pgm"}, "Show only frame_*.pgm."));
    parser.addOption(QCommandLineOption("png", "Show only independent PNG frames."));
    parser.addOption(QCommandLineOption("dbv", "Show only compact binary recordings."));
    parser.addOption(QCommandLineOption(
        "export-png", "Export all frames as PNG without opening GUI.", "directory"));
    parser.addOption(
        QCommandLineOption("check", "Check metadata associations without opening GUI."));
    parser.addPositionalArgument(
        "recording", "A directory of sessions, one session, or an image directory with frames.csv.");
    parser.addPositionalArgument(
        "frames.csv",
        "Recorder CSV (auto: local frames.csv or session/logs/frames.csv; empty skips).",
        "[frames.csv]");
    parser.addPositionalArgument(
        "detection.csv",
        "Detection CSV (auto: local detection.csv or session parent; empty skips).",
        "[detection.csv]");
    QStringList args;
    for (int i = 0; i < argc; ++i)
        args.push_back(QString::fromLocal8Bit(argv[i]));
    const bool parsed = parser.parse(args);
    const auto positional = parser.positionalArguments();
    const bool valid =
        !positional.isEmpty() && positional.size() <= 3
        && (int(parser.isSet("f")) + int(parser.isSet("r")) + int(parser.isSet("png"))
            + int(parser.isSet("dbv")) <= 1)
        && !(parser.isSet("check") && parser.isSet("export-png"));
    const bool wants_help = parser.isSet("help") || parser.isSet("help-all");
    if (!parsed || wants_help || !valid) {
        QCoreApplication app(argc, argv); // helpText needs an application, but no GUI plugin.
        if (!parsed)
            QTextStream(stderr) << parser.errorText() << '\n';
        const bool help = parsed && wants_help;
        QTextStream(help ? stdout : stderr) << parser.helpText();
        return help ? 0 : 2;
    }
    std::unique_ptr<QCoreApplication> application;
    if (parser.isSet("check") || parser.isSet("export-png"))
        application = std::make_unique<QCoreApplication>(argc, argv);
    const std::optional<QString> frames_csv =
        positional.size() > 1 ? std::optional<QString>{positional.at(1)} : std::nullopt;
    const std::optional<QString> detection_csv =
        positional.size() > 2 ? std::optional<QString>{positional.at(2)} : std::nullopt;
    for (int i = 1; i < positional.size(); ++i) {
        if (!positional.at(i).isEmpty() && !QFileInfo::exists(positional.at(i))) {
            QTextStream(stderr) << positional.at(i) << ": explicit CSV does not exist\n";
            return 2;
        }
    }
    Recording recording;
    QString error;
    const QString extension = parser.isSet("f")   ? "pbm"
                            : parser.isSet("r")   ? "pgm"
                            : parser.isSet("png") ? "png"
                            : parser.isSet("dbv") ? "dbv"
                                                  : "";
    if (!recording.load_directory(positional.at(0), frames_csv, detection_csv, extension, error)) {
        QTextStream(stderr) << error << '\n';
        return 1;
    }
    if (recording.frames().size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        QTextStream(stderr) << "Too many images for the frame slider\n";
        return 1;
    }
    QTextStream(stdout) << recording.summary() << '\n';
    for (const auto& note : recording.notes())
        QTextStream(stderr) << note << '\n';
    if (parser.isSet("export-png")) {
        const QDir destination(parser.value("export-png"));
        if (!QDir().mkpath(destination.absolutePath())) {
            QTextStream(stderr) << "Cannot create export directory\n";
            return 1;
        }
        std::size_t index = 0;
        for (const auto& frame : recording.frames()) {
            const auto path = destination.filePath(
                QStringLiteral("%1.png").arg(++index, 8, 10, QChar('0')));
            const auto image = read_frame_image(frame, error);
            if (QFileInfo::exists(path) || image.isNull() || !image.save(path, "PNG")) {
                QTextStream(stderr) << "Cannot export " << path << ": " << error << '\n';
                return 1;
            }
        }
        return 0;
    }
    if (parser.isSet("check")) {
        for (const auto& frame : recording.frames()) {
            for (const auto& note : frame.notes)
                QTextStream(stderr) << frame.path << ": " << note << '\n';
        }
        return recording.complete() ? 0 : 1;
    }
    application = std::make_unique<QApplication>(argc, argv);
    Player player(std::move(recording));
    player.show();
    return QCoreApplication::exec();
}

} // namespace dart::tools::video_player
