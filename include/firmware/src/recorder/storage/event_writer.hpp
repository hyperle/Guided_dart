#pragma once

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string_view>

#include "recorder/config.hpp"
#include "recorder/error.hpp"
#include "recorder/storage/session.hpp"

namespace dart::recorder {
class EventWriter final {
public:
    explicit EventWriter(const Config& config) noexcept
        : config_(config) {}
    ~EventWriter() { close(); }
    bool prepare() noexcept;
    bool open(const Session& session, RecorderError& error) noexcept;
    void close() noexcept;
    void write(const FrameEvent& event) noexcept;
    void maybe_flush_events(bool force) noexcept;
    static std::string_view decision_text(EventDecision decision) noexcept;

private:
    static constexpr std::size_t kEventBufferBytes = 64U * 1024U;
    const Config& config_;
    std::unique_ptr<char[]> event_buffer_{};
    std::FILE* event_file_ = nullptr;
    std::chrono::steady_clock::time_point last_event_flush_{};
};

} // namespace dart::recorder
