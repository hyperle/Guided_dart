#include "tests/diagnostics/detection_csv.hpp"

#include "camera/frame_view.hpp"
#include "logging/csv_writer.hpp"
#include "src/detector/target.hpp"
#include "system/write_control.hpp"

namespace dart::app {

bool DetectionCsv::open(const char* path, platform::WriteControl* writes) noexcept {
    return writer_.open(
        path,
        "sequence,received_us,source_pts,kind,valid,cx,cy,area,radius,"
        "quality,clipped,missed_frames,candidates,full_frame,rescanned,"
        "win_x0,win_y0,win_x1,win_y1,admission\n",
        writes);
}

void DetectionCsv::row(
    const camera::FrameView& view, const detection::DetectResult* result,
    const char* admission) noexcept {
    if (result == nullptr) {
        /* 识别关闭时仍然记录帧身份与放行决定，保证 CSV 覆盖每一帧。 */
        writer_.row(
            "%llu,%lld,%llu,none,0,,,,,,,,,,,%s\n", static_cast<unsigned long long>(view.sequence),
            static_cast<long long>(view.received.count()),
            static_cast<unsigned long long>(view.source_pts), admission);
        return;
    }
    const auto& target = result->target;
    const auto& scan = result->scan;
    writer_.row(
        "%llu,%lld,%llu,%s,%d,%.1f,%.1f,%u,%.2f,%.3f,%d,%u,%u,%d,%d,"
        "%u,%u,%u,%u,%s\n",
        static_cast<unsigned long long>(result->source.frame_id),
        static_cast<long long>(result->source.received.count()),
        static_cast<unsigned long long>(result->source.source_pts), kind_text(result->kind),
        target.valid ? 1 : 0, static_cast<double>(target.center.x),
        static_cast<double>(target.center.y), target.area, static_cast<double>(target.radius),
        static_cast<double>(target.quality), target.clipped ? 1 : 0, result->missed_frames,
        scan.candidates, scan.full_frame ? 1 : 0, scan.rescanned ? 1 : 0, scan.search_window.x,
        scan.search_window.y, scan.search_window.x + scan.search_window.width,
        scan.search_window.y + scan.search_window.height, admission);
}

const char* DetectionCsv::kind_text(detection::TargetKind kind) noexcept {
    switch (kind) {
    case detection::TargetKind::kNone: return "none";
    case detection::TargetKind::kLight: return "light";
    case detection::TargetKind::kArmor: return "armor";
    }
    return "none";
}

} // namespace dart::app
