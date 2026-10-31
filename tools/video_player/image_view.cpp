module;
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPointF>
#include <QRectF>
#include <QWidget>
#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>

export module dart.tools.video_player.image_view;

export import dart.tools.video_player.recording_types;

export namespace dart::tools::video_player {

class ImageView final : public QWidget {
public:
    struct Overlay {
        std::optional<QPointF> center;
        std::optional<QRectF> window;
    };

    explicit ImageView(QWidget* parent = nullptr)
        : QWidget(parent) {
        setMinimumSize(320, 240);
        setFocusPolicy(Qt::StrongFocus);
    }
    void set_frame(QImage image, Overlay overlay = {}) {
        image_ = std::move(image);
        overlay_ = std::move(overlay);
        update();
    }
    std::function<void(int, int)> clicked;
    static Overlay overlay_for(const Frame& frame, const QSize& size);

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);
        if (image_.isNull())
            return;
        const QRect target = image_rect();
        painter.drawImage(target, image_);
        const double sx = target.width() / static_cast<double>(image_.width());
        const double sy = target.height() / static_cast<double>(image_.height());
        painter.save();
        painter.setClipRect(target);
        painter.translate(target.topLeft());
        if (overlay_.window) {
            const auto& box = *overlay_.window;
            painter.setPen(QPen(Qt::cyan, 2));
            // win_x1/win_y1 are exclusive pixel boundaries.
            painter.drawRect(
                QRectF(box.x() * sx, box.y() * sy, box.width() * sx, box.height() * sy));
        }
        if (overlay_.center) {
            const auto& center = *overlay_.center;
            const QPointF p((center.x() + 0.5) * sx, (center.y() + 0.5) * sy);
            painter.setPen(QPen(Qt::green, 2));
            painter.drawLine(p + QPointF(-10, 0), p + QPointF(10, 0));
            painter.drawLine(p + QPointF(0, -10), p + QPointF(0, 10));
        }
        painter.restore();
    }
    void mousePressEvent(QMouseEvent* event) override {
        const QRect target = image_rect();
        if (image_.isNull() || !target.contains(event->pos()))
            return;
        const int x = std::clamp(
            (event->pos().x() - target.x()) * image_.width() / target.width(), 0,
            image_.width() - 1);
        const int y = std::clamp(
            (event->pos().y() - target.y()) * image_.height() / target.height(), 0,
            image_.height() - 1);
        if (clicked)
            clicked(x, y);
        setFocus();
    }

private:
    QRect image_rect() const {
        if (image_.isNull())
            return {};
        const QSize scaled = image_.size().scaled(size(), Qt::KeepAspectRatio);
        return QRect(
            QPoint((width() - scaled.width()) / 2, (height() - scaled.height()) / 2), scaled);
    }
    QImage image_;
    Overlay overlay_;
};

ImageView::Overlay ImageView::overlay_for(const Frame& frame, const QSize& size) {
    Overlay overlay;
    if (!frame.detection)
        return overlay;
    const auto number = [&](const char* name) -> std::optional<double> {
        bool ok = false;
        const double value = frame.detection->value(name).toDouble(&ok);
        return ok && std::isfinite(value) ? std::optional<double>{value} : std::nullopt;
    };
    const auto cx = number("cx");
    const auto cy = number("cy");
    if (frame.detection->value("valid") == "1" && cx && cy && *cx >= 0 && *cy >= 0
        && *cx < size.width() && *cy < size.height())
        overlay.center = QPointF(*cx, *cy);
    const auto x0 = number("win_x0");
    const auto y0 = number("win_y0");
    const auto x1 = number("win_x1");
    const auto y1 = number("win_y1");
    if (x0 && y0 && x1 && y1 && *x0 >= 0 && *y0 >= 0 && *x1 > *x0 && *y1 > *y0
        && *x1 <= size.width() && *y1 <= size.height())
        overlay.window = QRectF(*x0, *y0, *x1 - *x0, *y1 - *y0);
    return overlay;
}

} // namespace dart::tools::video_player
