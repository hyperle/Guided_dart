#pragma once

#include "camera/frame_view.hpp"
#include "logging/csv_writer.hpp"
#include "src/detector/target.hpp"
#include "system/write_control.hpp"

namespace dart::app {
// Detection schema and business values; buffering and file I/O live below L4.
class DetectionCsv final {
public:
    bool open(const char* path, platform::WriteControl* writes = nullptr) noexcept;

    void
        row(const camera::FrameView& view, const detection::DetectResult* result,
            const char* admission) noexcept;

    void flush() noexcept { writer_.flush(); }
    void close() noexcept { writer_.close(); }

private:
    logging::CsvWriter writer_;
    static const char* kind_text(detection::TargetKind kind) noexcept;
};

} // namespace dart::app
