#include "src/recorder/frame_channel.hpp"
#include "src/recorder/storage.hpp"
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
          record_(settings_.record), encoder_(settings_.record), storage_(settings_.record) {}
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
            else if (key == "flush_ms") settings.record.flush_period = std::chrono::milliseconds{number};
            else if (key == "threshold" && number <= 255) settings.record.pbm_threshold = static_cast<std::uint8_t>(number);
            else if (key == "event_slots" && number <= 4096) settings.record.event_slots = static_cast<std::uint32_t>(number);
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
                static_cast<std::size_t>(settings_.record.width) * settings_.record.height)) {
            status_.set_error(Error::kNoMemory);
            return std::unexpected(executor::Error::creation_fail);
        }
        prepared_ = true;
        status_.set_error(Error::kNone);
        if (!settings_.record.variable_record_size()
            && settings_.record.max_file_bytes < storage_.images().record_bytes()) {
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
        if (!wanted_ || !session_open_) {
            frame_.event.decision = EventDecision::kStopped;
        } else if (record_.expired(frame_.queued_at)) {
            frame_.event.decision = EventDecision::kTimedOut;
            ++status_.timed_out_;
        } else {
            const auto begin = std::chrono::steady_clock::now();
            output_ = encoder_.encode(frame_.lease.view());
            status_.pack_total_us_.fetch_add(static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count()),
                std::memory_order_relaxed);
            if (!output_.valid()) {
                errno = EIO;
                write_error();
                frame_.event.decision = EventDecision::kStopped;
            }
        }
        // Encoded formats no longer read the camera pixels during storage I/O.
        if (!output_.borrowed_input) frames_.return_frame(std::move(frame_.lease));
        return {};
    }

    std::expected<void, executor::Error> publish_outputs() noexcept override {
        if (output_.valid()) {
            frame_.event.image_index = status_.written_.load(std::memory_order_relaxed) + 1U;
            if (storage_.images().write_encoded(output_, frame_.event.image_index)) {
                ++status_.written_;
                status_.bytes_ = storage_.images().bytes_written();
                status_.io_total_us_.fetch_add(storage_.images().io_us(), std::memory_order_relaxed);
                status_.space_total_us_.fetch_add(storage_.images().space_us(), std::memory_order_relaxed);
                status_.open_total_us_.fetch_add(storage_.images().open_us(), std::memory_order_relaxed);
                status_.write_total_us_.fetch_add(storage_.images().write_us(), std::memory_order_relaxed);
                status_.close_total_us_.fetch_add(storage_.images().close_us(), std::memory_order_relaxed);
                status_.io_max_us_ = std::max(status_.io_max_us_.load(), storage_.images().io_us());
                frame_.event.decision = EventDecision::kAccepted;
                frame_.event.file_offset = storage_.images().file_offset();
                frame_.event.file_bytes = storage_.images().record_bytes();
                (void)std::snprintf(frame_.event.image_name.data(), frame_.event.image_name.size(), "%s", storage_.images().image_name());
                const auto remaining = storage_.session().file_limit_bytes() - storage_.images().bytes_written();
                if (remaining == 0 || (!settings_.record.variable_record_size() && remaining < storage_.images().record_bytes()))
                    status_.limit_ = true;
            } else {
                if (errno == EFBIG) status_.limit_ = true;
                else write_error();
                frame_.event.decision = EventDecision::kStopped;
                frame_.event.image_index = 0;
            }
        }
        frames_.return_frame(std::move(frame_.lease));
        if (frame_.event.input_index != 0 && session_open_) storage_.events().write(frame_.event);
        frame_ = {};
        output_ = {};
        drain_events();
        if (session_open_ && (!wanted_ || status_.limit_reached() || status_.last_error() != Error::kNone))
            close_session();
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
        storage_.error().reset();
        if (!encoder_.reset()) {
            errno = EIO;
            write_error();
            return false;
        }
        if (!storage_.open()) {
            (void)storage_.images().close();
            storage_.events().close();
            publish_details();
            return false;
        }
        session_open_ = true;
        status_.bytes_ = storage_.images().bytes_written();
        publish_details();
        frames_.open();
        return true;
    }
    void close_session() noexcept {
        frames_.close();
        if (!session_open_) return;
        drain_events();
        if (!storage_.images().close()) {
            storage_.error().set_file_error(Error::kImageFile, storage_.session().image_path());
            ++status_.write_fail_;
        }
        storage_.events().close();
        session_open_ = false;
        publish_details();
    }
    void drain_events() noexcept {
        FrameEvent event{};
        for (std::uint32_t i = 0; i < settings_.record.event_slots && frames_.pop_event(event); ++i)
            if (session_open_) storage_.events().write(event);
    }
    void write_error() noexcept {
        storage_.error().set_file_error(errno == ENOSPC ? Error::kStorage : Error::kImageFile, storage_.session().image_path());
        ++status_.write_fail_;
        publish_details();
    }
    void publish_details() noexcept {
        {
            std::lock_guard lock(status_.details_mutex_);
            auto& details = status_.details_;
            details.system_error = storage_.error().last_system_error();
            (void)std::snprintf(details.error_path.data(), details.error_path.size(), "%s", storage_.error().last_error_path());
            (void)std::snprintf(details.directory.data(), details.directory.size(), "%s", storage_.session().directory());
            (void)std::snprintf(details.recording_path.data(), details.recording_path.size(), "%s", storage_.session().image_path());
            details.file_limit_bytes = storage_.session().file_limit_bytes();
        }
        status_.set_error(storage_.error().last_error());
    }

    Settings settings_;
    FrameChannel& frames_;
    RecorderStatus& status_;
    platform::WriteControl* writes_;
    Record record_;
    Encoder encoder_;
    Storage storage_;
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
