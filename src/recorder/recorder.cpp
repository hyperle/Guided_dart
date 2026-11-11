#include "src/recorder/frame_channel.hpp"
#include "src/recorder/buffered_storage.hpp"
#include "src/recorder/encode.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <memory>
#include <new>
#include <utility>

namespace dart::recorder {

class Recorder final : public executor::Component {
public:
    struct Settings { Config record{}; bool enabled = true; };

    Recorder(const Spec& spec, Settings settings, FrameChannel& frames,
             RecorderStatus& status, platform::WriteControl* writes) noexcept
        : Component(spec), settings_(settings), frames_(frames), status_(status), writes_(writes),
          record_(settings_.record), encoder_(settings_.record), storage_(settings_.record, frames, status, writes) {}
    ~Recorder() override { (void)shutingdowm(); }

    static std::expected<Settings, executor::Error> parameters(const Spec& spec, Config config) noexcept {
        if (const auto valid = spec.validate(); !valid) return std::unexpected(valid.error());
        Settings settings{config};
        for (const auto& [key, value] : spec.parameter) {
            if (std::floor(value) != value || value < 0 || value > 1'000'000'000'000.0)
                return std::unexpected(executor::Error::invalid_spec);
            const auto number = static_cast<std::uint64_t>(value);
            if (key == "enabled" && number <= 1) settings.enabled = number != 0;
            else if (key == "fps" && number <= 1'000'000) settings.record.fps = static_cast<std::uint32_t>(number);
            else if (key == "frame_timeout_ms" && number <= 60'000) settings.record.frame_timeout = std::chrono::milliseconds{number};
            else if (key == "threshold" && number <= 255) settings.record.pbm_threshold = static_cast<std::uint8_t>(number);
            else if (key == "event_slots" && number <= 4096) settings.record.event_slots = static_cast<std::uint32_t>(number);
            else if (key == "queue_slots" && number <= 64) settings.record.queue_slots = static_cast<std::uint32_t>(number);
            else if (key == "keep_sessions" && number <= 9999) settings.record.keep_sessions = static_cast<std::uint32_t>(number);
            else if (key == "max_file_bytes") settings.record.max_file_bytes = number;
            else if (key == "max_storage_bytes") settings.record.max_storage_bytes = number;
            else return std::unexpected(executor::Error::invalid_spec);
        }
        if (Record::validate(settings.record) != Error::kNone)
            return std::unexpected(executor::Error::invalid_spec);
        return settings;
    }

private:
    std::expected<void, executor::Error> initializing() noexcept override {
        if (!frames_.quiescent()) {
            status_.set_error(Error::kAlreadyStarted);
            return std::unexpected(executor::Error::invalid_state);
        }
        if (!frames_.prepare(settings_.record, status_, writes_) || !encoder_.prepare() || !storage_.prepare(
                static_cast<std::size_t>(settings_.record.width) * settings_.record.height,
                encoder_.max_output_bytes())) {
            status_.set_error(Error::kNoMemory);
            return std::unexpected(executor::Error::creation_fail);
        }
        prepared_ = true;
        status_.set_error(Error::kNone);
        if (!settings_.record.variable_record_size()
            && settings_.record.max_file_bytes < encoder_.record_bytes()) {
            status_.set_error(Error::kInvalidConfig);
            return std::unexpected(executor::Error::invalid_spec);
        }
        return {};
    }

    std::expected<void, executor::Error> capture_inputs() noexcept override {
        output_ = {};
        wanted_ = settings_.enabled && (writes_ == nullptr || writes_->recording_enabled());
        if (!wanted_) {
            frames_.close();
            attempted_ = false;
        } else if (!session_open_ && !attempted_ && frames_.quiescent()) {
            attempted_ = true;
            (void)open_session();
        }
        frame_ = frames_.take();
        return {};
    }

    std::expected<void, executor::Error> update() noexcept override {
        if (!frame_.lease.valid()) return {};
        if (!wanted_ || !session_open_ || status_.limit_reached() || status_.last_error() != Error::kNone) {
            frame_.event.decision = EventDecision::kStopped;
        } else if (record_.expired(frame_.queued_at)) {
            frame_.event.decision = EventDecision::kTimedOut;
            ++status_.timed_out_;
        } else if (!storage_.available()) {
            // Never advance a predictive encoder for an input we cannot retain.
            frame_.event.decision = EventDecision::kBufferFull;
            ++status_.buffer_dropped_;
        } else {
            const auto begin = std::chrono::steady_clock::now();
            ++status_.encode_attempts_;
            output_ = encoder_.encode(frame_.lease.view());
            const auto elapsed = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - begin).count());
            status_.pack_total_us_.fetch_add(elapsed, std::memory_order_relaxed);
            status_.pack_max_us_ = std::max(status_.pack_max_us_.load(), elapsed);
            if (!output_.valid()) {
                errno = EIO;
                write_error();
                frame_.event.decision = EventDecision::kWriteError;
            } else {
                ++status_.encoded_;
            }
        }
        // Encoded formats no longer read the camera pixels during storage I/O.
        if (!output_.borrowed_input) frames_.return_frame(std::move(frame_.lease));
        return {};
    }

    std::expected<void, executor::Error> publish_outputs() noexcept override {
        bool buffered = false;
        if (output_.valid()) {
            buffered = storage_.submit(output_, frame_.event);
            if (!buffered) {
                // Capacity was checked before encode by this sole producer.
                // An unexpected failure must stop predictive encoding rather
                // than continue with a missing reference frame.
                if (errno == EAGAIN) errno = EIO;
                write_error();
                frame_.event.decision = EventDecision::kWriteError;
            }
        }
        frames_.return_frame(std::move(frame_.lease));
        if (!buffered && frame_.event.input_index != 0 && session_open_) {
            if (frame_.event.decision == EventDecision::kStopped) ++status_.stopped_;
            frames_.record_event(frame_.event);
        }
        frame_ = {};
        output_ = {};
        if (session_open_ && (!wanted_ || status_.limit_reached() || status_.last_error() != Error::kNone))
            close_session(wanted_ ? RecorderStatus::SessionEvent::Reason::kShutdown
                                  : RecorderStatus::SessionEvent::Reason::kPaused);
        frames_.complete();
        return {};
    }

    std::expected<void, executor::Error> shutingdowm() noexcept override {
        if (!prepared_) return {};
        frames_.close();
        if (!frame_.lease.valid()) frame_ = frames_.take();
        if (frame_.event.input_index != 0) {
            wanted_ = session_open_ && status_.last_error() == Error::kNone && !status_.limit_reached()
                && (writes_ == nullptr || writes_->recording_enabled());
            (void)update();
            (void)publish_outputs();
        }
        close_session();
        frames_.complete();
        prepared_ = false;
        return {};
    }

    bool open_session() noexcept {
        status_.reset_session();
        if (!encoder_.reset()) {
            errno = EIO;
            write_error();
            status_.publish_session(RecorderStatus::SessionEvent::Kind::kStartFailed,
                                    RecorderStatus::SessionEvent::Reason::kError);
            return false;
        }
        if (!storage_.open()) {
            status_.publish_session(RecorderStatus::SessionEvent::Kind::kStartFailed,
                                    RecorderStatus::SessionEvent::Reason::kError);
            return false;
        }
        session_open_ = true;
        ++status_.generation_;
        status_.publish_session(RecorderStatus::SessionEvent::Kind::kStarted,
                                RecorderStatus::SessionEvent::Reason::kRequested);
        frames_.open();
        return true;
    }
    void close_session(RecorderStatus::SessionEvent::Reason reason = RecorderStatus::SessionEvent::Reason::kShutdown) noexcept {
        frames_.close();
        if (!session_open_) return;
        const auto begin = std::chrono::steady_clock::now();
        storage_.close();
        status_.drain_us_ = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - begin).count());
        session_open_ = false;
        if (status_.last_error() != Error::kNone) reason = RecorderStatus::SessionEvent::Reason::kError;
        else if (status_.limit_reached()) reason = RecorderStatus::SessionEvent::Reason::kLimit;
        status_.publish_session(RecorderStatus::SessionEvent::Kind::kStopped, reason);
    }
    void write_error() noexcept { storage_.fail(errno); }

    Settings settings_;
    FrameChannel& frames_;
    RecorderStatus& status_;
    platform::WriteControl* writes_;
    Record record_;
    Encoder encoder_;
    BufferedStorage storage_;
    FrameChannel::Frame frame_{};
    Encoder::Image output_{};
    bool prepared_ = false, session_open_ = false, attempted_ = false, wanted_ = false;
};

std::expected<std::unique_ptr<executor::Component>, executor::Error> RecorderFactory::create(
    const executor::Component::Spec& spec, Config config, FrameChannel& frames,
    RecorderStatus& status, platform::WriteControl* writes) {
    const auto settings = Recorder::parameters(spec, config);
    if (!settings) {
        const auto error = Record::validate(config);
        status.set_error(error == Error::kNone ? Error::kInvalidConfig : error);
        return std::unexpected(settings.error());
    }
    auto component = std::unique_ptr<executor::Component>{new (std::nothrow) Recorder(spec, *settings, frames, status, writes)};
    if (!component) return std::unexpected(executor::Error::creation_fail);
    return component;
}

} // namespace dart::recorder
