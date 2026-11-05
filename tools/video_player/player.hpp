#pragma once

#include <QMainWindow>
#include <QPlainTextEdit>
#include <QSlider>
#include <cstddef>

#include "tools/video_player/image_reader.hpp"
#include "tools/video_player/image_view.hpp"
#include "tools/video_player/recording.hpp"
#include "tools/video_player/recording_paths.hpp"

namespace dart::tools::video_player {
class Player final : public QMainWindow {
public:
    explicit Player(Recording recording);

private:
    void show_frame(int index);

    Recording recording_;
    ImageView* view_ = nullptr;
    QPlainTextEdit* info_ = nullptr;
    QSlider* slider_ = nullptr;
    QString details_;
};

int run_application(int argc, char** argv);

} // namespace dart::tools::video_player
