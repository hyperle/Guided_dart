#pragma once

#include <QImage>
#include <QMouseEvent>
#include <QPointF>
#include <QRectF>
#include <QWidget>
#include <functional>
#include <optional>

#include "tools/video_player/recording_types.hpp"

namespace dart::tools::video_player {
class ImageView final : public QWidget {
public:
    struct Overlay {
        std::optional<QPointF> center;
        std::optional<QRectF> window;
    };

    explicit ImageView(QWidget* parent = nullptr);
    void set_frame(QImage image, Overlay overlay = {});
    std::function<void(int, int)> clicked;
    static Overlay overlay_for(const Frame& frame, const QSize& size);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent* event) override;

private:
    QRect image_rect() const;
    QImage image_;
    Overlay overlay_;
};

} // namespace dart::tools::video_player
