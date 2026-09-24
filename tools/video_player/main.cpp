#include <QApplication>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPainter>
#include <QSlider>
#include <QSplitter>
#include <QStatusBar>
#include <QTextStream>
#include <QVBoxLayout>
#include <QWidget>
#include <QKeyEvent>
#include <QMouseEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <filesystem>
#include <limits>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

struct LogRow {
    std::unordered_map<std::string, std::string> values;
    int line = 0;
};

struct Rect {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

static std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                field += '"';
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (c == ',' && !quoted) {
            fields.push_back(field);
            field.clear();
        } else {
            field += c;
        }
    }
    fields.push_back(field);
    return fields;
}

static std::optional<int> to_int(const std::string& value) {
    try {
        std::size_t used = 0;
        const int result = std::stoi(value, &used);
        return used == value.size() ? std::optional<int>(result) : std::nullopt;
    } catch (...) {
        return std::nullopt;
    }
}

static std::optional<Rect> parse_roi(const std::string& value) {
    std::string normalized = value;
    for (char& c : normalized) {
        if (c == ':' || c == ';' || c == 'x' || c == 'X') c = ',';
    }
    std::stringstream stream(normalized);
    std::vector<int> numbers;
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (const auto number = to_int(token)) numbers.push_back(*number);
    }
    if (numbers.size() != 4) return std::nullopt;
    if (numbers[2] <= 0 || numbers[3] <= 0) return std::nullopt;
    return Rect{numbers[0], numbers[1], numbers[2], numbers[3]};
}

static std::optional<QImage> read_pbm(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return std::nullopt;
    std::string magic;
    file >> magic;
    if (magic != "P4") return std::nullopt;
    auto read_token = [&file]() {
        std::string token;
        while (file.peek() == '#') {
            std::string comment;
            std::getline(file, comment);
        }
        file >> token;
        return token;
    };
    const auto width = to_int(read_token());
    const auto height = to_int(read_token());
    if (!width || !height || *width <= 0 || *height <= 0) return std::nullopt;
    file.get();
    const int row_bytes = (*width + 7) / 8;
    std::vector<std::uint8_t> packed(static_cast<std::size_t>(row_bytes * *height));
    file.read(reinterpret_cast<char*>(packed.data()), static_cast<std::streamsize>(packed.size()));
    if (file.gcount() != static_cast<std::streamsize>(packed.size())) return std::nullopt;
    QImage image(*width, *height, QImage::Format_Grayscale8);
    for (int y = 0; y < *height; ++y) {
        auto* scan = image.scanLine(y);
        for (int x = 0; x < *width; ++x) {
            const bool dark = (packed[static_cast<std::size_t>(y * row_bytes + x / 8)] & (0x80u >> (x % 8))) != 0;
            scan[x] = dark ? 0 : 255;
        }
    }
    return image;
}

static std::optional<QImage> read_pgm(const fs::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return std::nullopt;
    auto read_token = [&file]() {
        std::string token;
        char c = 0;
        while (file.get(c)) {
            if (std::isspace(static_cast<unsigned char>(c))) continue;
            if (c == '#') {
                file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }
            token.push_back(c);
            break;
        }
        while (file.get(c) && !std::isspace(static_cast<unsigned char>(c))) token.push_back(c);
        return token;
    };
    if (read_token() != "P5") return std::nullopt;
    const auto width = to_int(read_token());
    const auto height = to_int(read_token());
    const auto max_value = to_int(read_token());
    if (!width || !height || !max_value || *width <= 0 || *height <= 0 || *max_value <= 0 || *max_value > 255) return std::nullopt;
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(*width) * static_cast<std::size_t>(*height));
    file.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (file.gcount() != static_cast<std::streamsize>(pixels.size())) return std::nullopt;
    QImage image(*width, *height, QImage::Format_Grayscale8);
    for (int y = 0; y < *height; ++y) {
        auto* scan = image.scanLine(y);
        for (int x = 0; x < *width; ++x) {
            const auto value = pixels[static_cast<std::size_t>(y * *width + x)];
            scan[x] = static_cast<std::uint8_t>((static_cast<unsigned>(value) * 255u) / static_cast<unsigned>(*max_value));
        }
    }
    return image;
}

enum class ImageMode { Frames, Raw };

class ImageView final : public QWidget {
    Q_OBJECT
public:
    explicit ImageView(QWidget* parent = nullptr) : QWidget(parent) {
        setFocusPolicy(Qt::StrongFocus);
        setMinimumSize(480, 360);
    }
    void set_frame(QImage image, std::optional<QPoint> center, std::optional<Rect> roi) {
        image_ = std::move(image);
        center_ = center;
        roi_ = roi;
        update();
    }
    void clear_frame() { image_ = {}; center_.reset(); roi_.reset(); update(); }
signals:
    void image_clicked(int x, int y);
protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);
        if (image_.isNull()) return;
        const QRect target = QRect(QPoint(0, 0), image_.size().scaled(size(), Qt::KeepAspectRatio));
        const QPoint origin((width() - target.width()) / 2, (height() - target.height()) / 2);
        painter.drawImage(QRect(origin, target.size()), image_);
        const double sx = target.width() / static_cast<double>(image_.width());
        const double sy = target.height() / static_cast<double>(image_.height());
        auto map = [&](int x, int y) { return QPoint(origin.x() + qRound(x * sx), origin.y() + qRound(y * sy)); };
        if (center_) {
            const QPoint p = map(center_->x(), center_->y());
            painter.setPen(QPen(Qt::green, 2));
            painter.drawLine(p.x() - 10, p.y(), p.x() + 10, p.y());
            painter.drawLine(p.x(), p.y() - 10, p.x(), p.y() + 10);
        }
        if (roi_) {
            painter.setPen(QPen(Qt::blue, 2));
            painter.drawRect(QRect(map(roi_->x, roi_->y), QSize(qRound(roi_->width * sx), qRound(roi_->height * sy))));
        }
    }
    void mousePressEvent(QMouseEvent* event) override {
        if (image_.isNull()) return;
        const QRect target = QRect(QPoint(0, 0), image_.size().scaled(size(), Qt::KeepAspectRatio));
        const QPoint origin((width() - target.width()) / 2, (height() - target.height()) / 2);
        if (!target.translated(origin).contains(event->pos())) return;
        const int x = std::clamp(static_cast<int>((event->pos().x() - origin.x()) * image_.width() / target.width()), 0, image_.width() - 1);
        const int y = std::clamp(static_cast<int>((event->pos().y() - origin.y()) * image_.height() / target.height()), 0, image_.height() - 1);
        emit image_clicked(x, y);
        setFocus();
    }
private:
    QImage image_;
    std::optional<QPoint> center_;
    std::optional<Rect> roi_;
};

class Player final : public QMainWindow {
public:
    Player(fs::path image_dir, fs::path csv_path, ImageMode mode) : image_dir_(std::move(image_dir)), mode_(mode) {
        setWindowTitle(mode_ == ImageMode::Raw ? QStringLiteral("SelfGuidingDart Raw PGM Player") : QStringLiteral("SelfGuidingDart PBM Frame Player"));
        resize(1100, 700);
        load_logs(csv_path);
        load_frames();
        auto* central = new QWidget(this);
        auto* layout = new QVBoxLayout(central);
        auto* splitter = new QSplitter(Qt::Horizontal, central);
        view_ = new ImageView(splitter);
        info_ = new QLabel(splitter);
        info_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        info_->setMinimumWidth(270);
        info_->setTextInteractionFlags(Qt::TextSelectableByMouse);
        splitter->addWidget(view_);
        splitter->addWidget(info_);
        splitter->setStretchFactor(0, 1);
        slider_ = new QSlider(Qt::Horizontal, central);
        slider_->setFocusPolicy(Qt::StrongFocus);
        layout->addWidget(splitter, 1);
        layout->addWidget(slider_);
        setCentralWidget(central);
        connect(slider_, &QSlider::valueChanged, this, [this](int value) { show_frame(value); });
        connect(view_, &ImageView::image_clicked, this, [this](int x, int y) {
            statusBar()->showMessage(QStringLiteral("pixel (%1, %2)").arg(x).arg(y));
        });
        if (!frames_.empty()) {
            slider_->setRange(0, static_cast<int>(frames_.size()) - 1);
            show_frame(0);
        } else {
            info_->setText(QStringLiteral("No %1 files found in %2").arg(mode_ == ImageMode::Raw ? QStringLiteral("raw*.pgm") : QStringLiteral("f*.pbm")).arg(QString::fromStdString(image_dir_.string())));
        }
    }
protected:
    void keyPressEvent(QKeyEvent* event) override {
        if (event->key() == Qt::Key_Left || event->key() == Qt::Key_Right) {
            const int delta = event->key() == Qt::Key_Left ? -1 : 1;
            slider_->setValue(std::clamp(slider_->value() + delta, slider_->minimum(), slider_->maximum()));
            event->accept();
            return;
        }
        QMainWindow::keyPressEvent(event);
    }
private:
    void load_logs(const fs::path& path) {
        std::ifstream file(path);
        if (!file) return;
        std::string line;
        if (!std::getline(file, line)) return;
        const auto headers = split_csv(line);
        int log_line = 0;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            const auto fields = split_csv(line);
            if (fields.empty()) continue;
            LogRow row;
            for (std::size_t i = 0; i < headers.size() && i < fields.size(); ++i) row.values[headers[i]] = fields[i];
            row.line = ++log_line;
            logs_[row.line] = std::move(row);
        }
    }
    void load_frames() {
        if (!fs::exists(image_dir_)) return;
        for (const auto& entry : fs::directory_iterator(image_dir_)) {
            if (!entry.is_regular_file()) continue;
            const std::string stem = entry.path().stem().string();
            const std::string prefix = mode_ == ImageMode::Raw ? "raw" : "f";
            const std::string extension = mode_ == ImageMode::Raw ? ".pgm" : ".pbm";
            if (entry.path().extension() != extension || stem.size() < prefix.size() + 6 || stem.compare(0, prefix.size(), prefix) != 0) continue;
            const std::string suffix = stem.substr(stem.size() - 6);
            if (const auto line = to_int(suffix)) frames_.push_back({*line, entry.path()});
        }
        std::sort(frames_.begin(), frames_.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    }
    void show_frame(int index) {
        if (index < 0 || index >= static_cast<int>(frames_.size())) return;
        const auto& frame = frames_[static_cast<std::size_t>(index)];
        const auto image = mode_ == ImageMode::Raw ? read_pgm(frame.second) : read_pbm(frame.second);
        if (!image) { info_->setText(QStringLiteral("Failed to read %1").arg(QString::fromStdString(frame.second.string()))); return; }
        std::optional<QPoint> center;
        std::optional<Rect> roi;
        QString details = QStringLiteral("image line %1\nfile: %2\nsize: %3 x %4\n")
            .arg(frame.first).arg(QString::fromStdString(frame.second.filename().string())).arg(image->width()).arg(image->height());
        if (const auto it = logs_.find(frame.first); it != logs_.end()) {
            const auto& values = it->second.values;
            const auto cx = to_int(values.count("cx") ? values.at("cx") : "");
            const auto cy = to_int(values.count("cy") ? values.at("cy") : "");
            if (cx && cy && *cx >= 0 && *cy >= 0) center = QPoint(*cx, *cy);
            details += QStringLiteral("\nlog line: %1").arg(frame.first);
            for (const char* key : {"mono_ms", "src_pts", "d_pts_us", "exp_us", "cx", "cy", "roi", "cost_us", "captured_fps", "pbm"}) {
                if (values.count(key)) details += QStringLiteral("\n%1: %2").arg(key).arg(QString::fromStdString(values.at(key)));
            }
            if (values.count("roi")) roi = parse_roi(values.at("roi"));
        } else {
            details += QStringLiteral("\nno matching CSV row");
        }
        view_->set_frame(*image, center, roi);
        info_->setText(details);
        statusBar()->showMessage(QStringLiteral("frame %1/%2").arg(index + 1).arg(frames_.size()));
    }
    fs::path image_dir_;
    ImageMode mode_ = ImageMode::Frames;
    std::vector<std::pair<int, fs::path>> frames_;
    std::unordered_map<int, LogRow> logs_;
    ImageView* view_ = nullptr;
    QLabel* info_ = nullptr;
    QSlider* slider_ = nullptr;
};

int main(int argc, char** argv) {
    ImageMode mode = ImageMode::Frames;
    int positional = 1;
    if (argc > 1 && (std::string(argv[1]) == "-f" || std::string(argv[1]) == "-r")) {
        mode = std::string(argv[1]) == "-r" ? ImageMode::Raw : ImageMode::Frames;
        positional = 2;
    }
    if (argc > positional + 2 || (argc > 1 && std::string(argv[1]) == "--help")) {
        std::cerr << "Usage: " << argv[0] << " [-f|-r] [image-dir] [csv]\n"
                  << "  -f  play f*.pbm frames (default)\n"
                  << "  -r  play raw*.pgm images\n";
        return argc > 1 && std::string(argv[1]) == "--help" ? 0 : 2;
    }
    QApplication app(argc, argv);
    const fs::path image_dir = argc > positional ? argv[positional] : "records/img";
    const fs::path csv_path = argc > positional + 1 ? argv[positional + 1] : "records/logs/frames.csv";
    Player player(image_dir, csv_path, mode);
    player.show();
    return app.exec();
}

#include "main.moc"
