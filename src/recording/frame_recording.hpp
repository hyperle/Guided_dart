#pragma once

#include <chrono>
#include <cstdint>

#include "logging/logger.hpp"
#include "recorder/recorder.hpp"
#include "src/startup/argument_parser.hpp"
#include "system/monotonic_clock.hpp"

namespace dart::app {
const char* admission_text(recorder::Admission admission) noexcept;

void drain_recording_releases(recorder::Recorder& rec, logging::Logger& log) noexcept;

recorder::Config
    recording_config(const Arguments& args, std::uint32_t width, std::uint32_t height) noexcept;

bool start_recording(
    recorder::Recorder& rec, const recorder::Config& config, logging::Logger& log) noexcept;

bool stop_recording_if_finished(
    recorder::Recorder& rec, const recorder::Config& config, logging::Logger& log) noexcept;

} // namespace dart::app
