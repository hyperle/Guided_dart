module;
#include <chrono>
#include <cstdint>
#include <string_view>

export module dart.app.recording.frame_recording;

import dart.app.startup.argument_parser;
import dart.logging.logger;
import dart.recorder.recorder;
import dart.system.monotonic_clock;

export namespace dart::app {

const char* admission_text(recorder::Admission admission) noexcept {
    switch (admission) {
    case recorder::Admission::kAccepted: return "accepted";
    case recorder::Admission::kDownsampled: return "downsampled";
    case recorder::Admission::kBusy: return "busy";
    case recorder::Admission::kStopped: return "stopped";
    case recorder::Admission::kInvalidFrame: return "invalid_frame";
    }
    return "unknown";
}

void drain_recording_releases(recorder::Recorder& rec, logging::Logger& log) noexcept {
    for (int attempt = 0; attempt < 500; ++attempt) {
        if (rec.pending_releases() == 0) {
            log.line(
                "level=INFO module=app op=shutdown step=leases_drained "
                "pending=0\n");
            return;
        }
        (void)rec.drain_releases();
        platform::sleep_ms(10);
    }
    log.line(
        "level=WARN module=app op=shutdown step=leases_drained pending=%u "
        "action=continue note=drain_incomplete\n",
        rec.pending_releases());
}

recorder::Config
    recording_config(const Arguments& args, std::uint32_t width, std::uint32_t height) noexcept {
    recorder::Config config{};
    config.width = width;
    config.height = height;
    config.fps = args.record_fps;
    config.frame_timeout = std::chrono::milliseconds{args.record_timeout_ms};
    config.keep_sessions = args.record_keep;
    // Bounded copied pixels never hold camera leases while waiting for disk.
    // The writer discards expired entries instead of replaying a stalled queue.
    config.queue_slots = 16;
    config.image_format = args.record_format;
    config.pbm_threshold = static_cast<std::uint8_t>(args.threshold);
    config.output_directory = std::string_view{args.rec_dir};
    if (config.output_directory == "/data" || config.output_directory.starts_with("/data/"))
        config.max_storage_bytes = recorder::Config::kMaxStorageBytes;
    return config;
}

bool start_recording(
    recorder::Recorder& rec, const recorder::Config& config, logging::Logger& log) noexcept {
    if (!rec.start()) {
        log.line(
            "level=ERROR module=app op=recorder_start result=failed "
            "code=%s system_errno=%d path=%s action=disable_recording\n",
            recorder::Recorder::error_text(rec.last_error()).data(), rec.last_system_error(),
            rec.last_error_path());
        return false;
    }
    const auto stats = rec.stats();
    log.line(
        "level=INFO module=app op=recorder_start result=ok dir=%s file=%s fps=%u "
        "frame_timeout_ms=%u format=%s "
        "max_file_bytes=%llu file_limit_bytes=%llu max_storage_bytes=%llu queued=%u\n",
        rec.directory(), rec.recording_path(), config.fps,
        static_cast<unsigned>(config.frame_timeout.count()),
        config.image_format == recorder::ImageFormat::kDbv   ? "dbv"
        : config.image_format == recorder::ImageFormat::kPng ? "png"
        : config.image_format == recorder::ImageFormat::kPbm ? "pbm"
                                                             : "pgm",
        static_cast<unsigned long long>(config.max_file_bytes),
        static_cast<unsigned long long>(rec.file_limit_bytes()),
        static_cast<unsigned long long>(config.max_storage_bytes), stats.queued);
    return true;
}

bool stop_recording_if_finished(
    recorder::Recorder& rec, const recorder::Config& config, logging::Logger& log) noexcept {
    if (!rec.limit_reached() && rec.last_error() == recorder::Error::kNone)
        return false;
    rec.stop();
    drain_recording_releases(rec, log);
    const auto stats = rec.stats();
    log.line(
        "level=INFO module=app op=recorder_stop reason=%s file=%s "
        "bytes=%llu max_file_bytes=%llu file_limit_bytes=%llu written=%llu code=%s "
        "system_errno=%d action=disable_recording\n",
        rec.last_error() != recorder::Error::kNone       ? "write_error"
        : rec.file_limit_bytes() < config.max_file_bytes ? "storage_limit"
                                                         : "size_limit",
        rec.recording_path(), static_cast<unsigned long long>(stats.bytes),
        static_cast<unsigned long long>(config.max_file_bytes),
        static_cast<unsigned long long>(rec.file_limit_bytes()),
        static_cast<unsigned long long>(stats.written),
        recorder::Recorder::error_text(rec.last_error()).data(), rec.last_system_error());
    return true;
}

} // namespace dart::app
