#include "src/detector/detect.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <expected>
#include <memory>
#include <new>
#include <utility>

namespace dart::detection {

class Detector final : public executor::Component {
public:
    struct Settings {
        Detect::Config detection{};
        std::chrono::milliseconds acquire_timeout{200};
        std::uint32_t max_acquire_errors = 8;
        bool enabled = true;
    };

    Detector(
        const Spec& spec, Settings settings, camera::ICamera& camera,
        executor::Output<Point> target, DetectorStatus& status,
        DetectorFactory::FrameConsumer consume_frame)
        : Component(spec)
        , settings_(settings)
        , camera_(camera)
        , target_(target)
        , status_(status)
        , consume_frame_(std::move(consume_frame))
        , detect_(settings.detection)
        , track_(
              settings.detection.light, settings.detection.armor, settings.detection.track,
              settings.detection.roi) {}

    ~Detector() override { (void)shutingdowm(); }

    static std::expected<Settings, executor::Error> parameters(const Spec& spec) noexcept {
        if (const auto valid = spec.validate(); !valid)
            return std::unexpected(valid.error());
        Settings settings{};
        for (const auto& [key, value] : spec.parameter) {
            if (key == "enabled") {
                if (value != 0.0 && value != 1.0)
                    return std::unexpected(executor::Error::invalid_spec);
                settings.enabled = value != 0.0;
            } else if (key == "acquire_ms" || key == "max_acquire_errors") {
                if (value < 1 || value > 10000 || std::floor(value) != value)
                    return std::unexpected(executor::Error::invalid_spec);
                if (key == "acquire_ms")
                    settings.acquire_timeout =
                        std::chrono::milliseconds{static_cast<std::uint32_t>(value)};
                else
                    settings.max_acquire_errors = static_cast<std::uint32_t>(value);
            } else if (!Detect::set_parameter(settings.detection, key, value)) {
                return std::unexpected(executor::Error::invalid_spec);
            }
        }
        return settings;
    }

private:
    std::expected<void, executor::Error> initializing() noexcept override {
        clear_target();
        status_ = {};
        status_.vision_enabled = settings_.enabled;
        if (settings_.enabled) {
            // Format, size and pixel aspect come from the camera; frames must match.
            const auto prepared = detect_.Prepare(camera_.frame_info());
            if (!prepared) {
                status_.detection_error = prepared.error();
                return std::unexpected(executor::Error::invalid_spec);
            }
        }
        track_.Reset();
        if (const auto started = camera_.start(); !started) {
            status_.camera_error = started.error();
            return std::unexpected(executor::Error::input_failed);
        }
        camera_started_ = true;
        return {};
    }

    std::expected<void, executor::Error> capture_inputs() noexcept override {
        status_ = {};
        status_.vision_enabled = settings_.enabled;
        status_.span_high_water = detect_.span_high_water();
        output_ = Point::missing();
        result_.reset();
        if (const auto released = release_frame(); !released) {
            clear_target();
            return released;
        }
        const auto begin = std::chrono::steady_clock::now();
        auto acquired = camera_.acquire(settings_.acquire_timeout);
        status_.acquire_time = elapsed(begin);
        if (!acquired) {
            status_.camera_error = acquired.error();
            status_.consecutive_acquire_errors = ++consecutive_acquire_errors_;
            clear_target();
            if (consecutive_acquire_errors_ >= settings_.max_acquire_errors)
                return std::unexpected(executor::Error::input_failed);
            return {};
        }
        consecutive_acquire_errors_ = 0;
        status_.acquisition = acquired->status;
        if (acquired->status == camera::AcquireStatus::kFrame) {
            lease_ = std::move(acquired->lease);
            status_.acquired_frame = lease_.valid();
            if (!lease_.valid()) {
                clear_target();
                return std::unexpected(executor::Error::input_failed);
            }
        }
        return {};
    }

    std::expected<void, executor::Error> update() noexcept override {
        if (!lease_.valid() || !settings_.enabled)
            return {};
        const auto begin = std::chrono::steady_clock::now();
        auto detected = detect_.update(lease_.view(), track_);
        status_.detect_time = elapsed(begin);
        status_.span_high_water = detect_.span_high_water();
        if (!detected) {
            status_.detection_error = detected.error();
            return {};
        }
        result_ = *detected;
        output_ = result_->coordinate();
        return {};
    }

    std::expected<void, executor::Error> publish_outputs() noexcept override {
        target_.publish(output_);
        if (lease_.valid() && consume_frame_)
            consume_frame_(lease_, result_ ? &*result_ : nullptr);
        return release_frame();
    }

    std::expected<void, executor::Error> shutingdowm() noexcept override {
        clear_target();
        result_.reset();
        track_.Reset();
        const auto released = release_frame();
        if (!released)
            return released;
        if (camera_started_) {
            camera_started_ = false;
            if (const auto stopped = camera_.stop(); !stopped) {
                status_.camera_error = stopped.error();
                return std::unexpected(executor::Error::input_failed);
            }
        }
        return released;
    }

    std::expected<void, executor::Error> release_frame() noexcept {
        if (!lease_.valid())
            return {};
        if (const auto released = lease_.release(); !released) {
            status_.release_error = released.error();
            clear_target();
            return std::unexpected(executor::Error::input_failed);
        }
        status_.released_frame = true;
        return {};
    }

    void clear_target() noexcept {
        output_ = Point::missing();
        target_.publish(output_);
    }

    static std::chrono::microseconds elapsed(std::chrono::steady_clock::time_point begin) noexcept {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - begin);
    }

    Settings settings_;
    camera::ICamera& camera_;
    executor::Output<Point> target_;
    DetectorStatus& status_;
    DetectorFactory::FrameConsumer consume_frame_;
    Detect detect_;
    Track track_;
    camera::FrameLease lease_{};
    std::optional<DetectResult> result_{};
    Point output_ = Point::missing();
    std::uint32_t consecutive_acquire_errors_ = 0;
    bool camera_started_ = false;
};

std::expected<std::unique_ptr<executor::Component>, executor::Error> DetectorFactory::create(
    const executor::Component::Spec& spec, camera::ICamera& camera, executor::Output<Point> target,
    DetectorStatus& status, FrameConsumer consume_frame) {
    const auto settings = Detector::parameters(spec);
    if (!settings)
        return std::unexpected(settings.error());
    auto component = std::unique_ptr<executor::Component>{new (std::nothrow) Detector(
        spec, *settings, camera, target, status, std::move(consume_frame))};
    if (!component)
        return std::unexpected(executor::Error::creation_fail);
    return component;
}

} // namespace dart::detection
