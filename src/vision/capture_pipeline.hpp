#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>

#include "camera/interface.hpp"
#include "logging/logger.hpp"
#include "runtime/imu/session.hpp"
#include "src/startup/argument_parser.hpp"
#include "streaming/interface.hpp"
#include "system/device_services.hpp"
#include "system/run_control.hpp"
#include "system/write_control.hpp"

namespace dart::app {
class CapturePipeline final {
private:
    struct StageTiming {
        std::uint64_t total_us = 0;
        std::uint64_t max_us = 0;
        std::uint64_t samples = 0;

        void observe(std::chrono::steady_clock::time_point begin) noexcept {
            const auto us =
                static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                               std::chrono::steady_clock::now() - begin)
                                               .count());
            total_us += us;
            max_us = std::max(max_us, us);
            ++samples;
        }

        double average_us() const noexcept {
            return samples > 0 ? static_cast<double>(total_us) / static_cast<double>(samples) : 0.0;
        }
    };

    struct Counters {
        std::uint64_t frames = 0;
        std::uint64_t no_frame = 0;
        std::uint64_t lease_exhausted = 0;
        std::uint64_t acquire_errors = 0;
        std::uint64_t detect_calls = 0;
        std::uint64_t detect_frames = 0, track_frames = 0, validation_frames = 0,
                      stability_checks = 0;
        std::uint64_t detect_errors = 0;
        std::uint64_t target_hits = 0;
        std::uint64_t release_ok = 0;
        std::uint64_t release_errors = 0;
        std::uint64_t rec_accepted = 0;
        std::uint64_t rec_downsampled = 0;
        std::uint64_t rec_busy = 0;
        std::uint64_t rec_invalid = 0;
    };

public:
    static int execute(
        const Arguments& args, std::unique_ptr<camera::ICamera> cam, platform::RunControl& control,
        logging::Logger& log, imu::Session* imu_session = nullptr,
        platform::WriteControl* writes = nullptr, platform::PwmFactory pwm_factory = nullptr,
        streaming::TransportFactory wifi_factory = nullptr) noexcept;
};

} // namespace dart::app
