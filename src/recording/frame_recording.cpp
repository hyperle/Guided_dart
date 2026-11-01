#ifndef DART_RECORDING_FRAME_RECORDING_CPP
#define DART_RECORDING_FRAME_RECORDING_CPP

#include <cstdint>
#include <string_view>

#include "logging/logger.cpp"
#include "recorder/recorder.cpp"
#include "src/startup/argument_parser.cpp"
#include "system/monotonic_clock.cpp"

namespace dart::app {

inline const char* admission_text(recorder::Admission admission) noexcept {
    switch (admission) {
    case recorder::Admission::kAccepted: return "accepted";
    case recorder::Admission::kDownsampled: return "downsampled";
    case recorder::Admission::kBusy: return "busy";
    case recorder::Admission::kStopped: return "stopped";
    case recorder::Admission::kInvalidFrame: return "invalid_frame";
    }
    return "unknown";
}

inline const char* image_format_text(recorder::ImageFormat format) noexcept {
    switch (format) {
    case recorder::ImageFormat::kDbv: return "dbv";
    case recorder::ImageFormat::kPng: return "png";
    case recorder::ImageFormat::kPbm: return "pbm";
    case recorder::ImageFormat::kPgm: return "pgm";
    }
    return "pgm";
}

/* The recorder starts asynchronously through WriteControl, so the owning
 * caller reports the transition once it can observe a started session. */
inline void log_recorder_started(
    recorder::Recorder& rec, const recorder::Config& config, logging::Logger& log) noexcept {
    const auto stats = rec.stats();
    log.line(
        logging::Tag::kRecord,
        "level=INFO op=recorder_start result=ok dir=%s file=%s fps=%u format=%s "
        "max_file_bytes=%llu file_limit_bytes=%llu max_storage_bytes=%llu queued=%u\n",
        rec.directory(), rec.recording_path(), config.fps, image_format_text(config.image_format),
        static_cast<unsigned long long>(config.max_file_bytes),
        static_cast<unsigned long long>(rec.file_limit_bytes()),
        static_cast<unsigned long long>(config.max_storage_bytes), stats.queued);
}

inline void log_recorder_failed(recorder::Recorder& rec, logging::Logger& log) noexcept {
    log.line(
        logging::Tag::kRecord,
        "level=ERROR op=recorder_start result=failed code=%s system_errno=%d "
        "path=%s action=disable_recording\n",
        recorder::Recorder::error_text(rec.last_error()).data(), rec.last_system_error(),
        rec.last_error_path());
}

inline void drain_recording_releases(
    recorder::Recorder& rec, logging::Logger& log, bool report) noexcept {
    for (int attempt = 0; attempt < 500; ++attempt) {
        if (rec.pending_releases() == 0) {
            if (report)
                log.line(
                    logging::Tag::kRecord,
                    "level=INFO op=shutdown step=leases_drained pending=0\n");
            return;
        }
        (void)rec.drain_releases();
        platform::sleep_ms(10);
    }
    if (report)
        log.line(
            logging::Tag::kRecord,
            "level=WARN op=shutdown step=leases_drained pending=%u "
            "action=continue note=drain_incomplete\n",
            rec.pending_releases());
}

inline recorder::Config
    recording_config(const Arguments& args, std::uint32_t width, std::uint32_t height) noexcept {
    recorder::Config config{};
    config.width = width;
    config.height = height;
    config.fps = args.record_fps;
    config.keep_sessions = args.record_keep;
    // About 267 ms at 60 FPS, covering observed 134 ms FAT write stalls.
    // Pixels are copied before enqueue; this does not hold 16 camera leases.
    config.queue_slots = 16;
    config.image_format = args.record_format;
    config.pbm_threshold = static_cast<std::uint8_t>(args.threshold);
    config.output_directory = std::string_view{args.rec_dir};
    if (config.output_directory == "/data" || config.output_directory.starts_with("/data/"))
        config.max_storage_bytes = recorder::Config::kMaxStorageBytes;
    return config;
}

inline bool start_recording(
    recorder::Recorder& rec, const recorder::Config& config, logging::Logger& log) noexcept {
    if (!rec.start()) {
        log_recorder_failed(rec, log);
        return false;
    }
    log_recorder_started(rec, config, log);
    return true;
}

inline bool stop_recording_if_finished(
    recorder::Recorder& rec, const recorder::Config& config, logging::Logger& log) noexcept {
    if (!rec.limit_reached() && rec.last_error() == recorder::Error::kNone)
        return false;
    rec.stop();
    drain_recording_releases(rec, log, true);
    const auto stats = rec.stats();
    log.line(
        logging::Tag::kRecord,
        "level=INFO op=recorder_stop reason=%s file=%s "
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

#endif // DART_RECORDING_FRAME_RECORDING_CPP
