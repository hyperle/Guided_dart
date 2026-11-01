#include "recorder/storage/event_writer.hpp"

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <new>
#include <string_view>
#include <unistd.h>

#include "recorder/config.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/session.hpp"

namespace dart::recorder {

bool EventWriter::prepare() noexcept {
    event_buffer_.reset(new (std::nothrow) char[kEventBufferBytes]);
    return event_buffer_ != nullptr;
}

bool EventWriter::open(const Session& session, RecorderError& error) noexcept {
    event_file_ = std::fopen(session.event_path(), "w");
    if (event_file_ == nullptr) {
        error.set_file_error(Error::kEventFile, session.event_path());
        return false;
    }
    if (event_buffer_)
        (void)std::setvbuf(event_file_, event_buffer_.get(), _IOFBF, kEventBufferBytes);
    std::fprintf(
        event_file_,
        "# recorder format=2 width=%u height=%u fps=%u frame_timeout_ms=%lld max_file_bytes=%llu "
        "file_limit_bytes=%llu session=%s\n"
        "input_index,source_sequence,source_pts,source_pts_timebase_validated,received_us,"
        "decision,image_index,image_file,file_offset,file_bytes\n",
        config_.width, config_.height, config_.fps,
        static_cast<long long>(config_.frame_timeout.count()),
        static_cast<unsigned long long>(config_.max_file_bytes),
        static_cast<unsigned long long>(session.file_limit_bytes()), session.directory());
    if (std::fflush(event_file_) != 0 || ::fsync(::fileno(event_file_)) != 0) {
        error.set_file_error(Error::kEventFile, session.event_path());
        return false;
    }
    last_event_flush_ = std::chrono::steady_clock::now();
    return true;
}

void EventWriter::close() noexcept {
    if (event_file_ != nullptr) {
        maybe_flush_events(true);
        std::fclose(event_file_);
        event_file_ = nullptr;
    }
}

void EventWriter::write(const FrameEvent& event) noexcept {
    if (event_file_ != nullptr) {
        const auto decision = decision_text(event.decision);
        std::fprintf(
            event_file_, "%llu,%llu,%llu,%u,%lld,%.*s,%llu,%s,%llu,%llu\n",
            static_cast<unsigned long long>(event.input_index),
            static_cast<unsigned long long>(event.source_sequence),
            static_cast<unsigned long long>(event.source_pts),
            event.source_pts_timebase_validated ? 1U : 0U,
            static_cast<long long>(event.received_us), static_cast<int>(decision.size()),
            decision.data(), static_cast<unsigned long long>(event.image_index),
            event.image_name.data(), static_cast<unsigned long long>(event.file_offset),
            static_cast<unsigned long long>(event.file_bytes));
    }
    if (config_.event_sink != nullptr)
        config_.event_sink(event, config_.event_sink_context);
    maybe_flush_events(false);
}

void EventWriter::maybe_flush_events(bool force) noexcept {
    if (event_file_ == nullptr)
        return;
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - last_event_flush_ < config_.flush_period)
        return;
    (void)std::fflush(event_file_);
    (void)::fsync(::fileno(event_file_));
    last_event_flush_ = now;
}

std::string_view EventWriter::decision_text(EventDecision decision) noexcept {
    switch (decision) {
    case EventDecision::kAccepted: return "accepted";
    case EventDecision::kDownsampled: return "downsampled";
    case EventDecision::kBusy: return "busy";
    case EventDecision::kStopped: return "stopped";
    case EventDecision::kInvalidFrame: return "invalid_frame";
    case EventDecision::kTimedOut: return "timed_out";
    }
    return "unknown";
}

} // namespace dart::recorder
