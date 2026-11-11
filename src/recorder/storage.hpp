#pragma once

#include "recorder/storage.hpp"

namespace dart::recorder {

// Own one recording session and its file adapters. BufferedStorage serializes
// their lifecycle and gives its file worker exclusive access while recording.
class Storage final {
public:
    explicit Storage(const Config& config) noexcept
        : session_(config, error_), images_(config, session_), events_(config) {}
    bool prepare(std::size_t bytes) noexcept { return images_.prepare(bytes) && events_.prepare(); }
    bool open() noexcept { return session_.open() && images_.open(error_) && events_.open(session_, error_); }
    RecorderError& error() noexcept { return error_; }
    Session& session() noexcept { return session_; }
    ImageWriter& images() noexcept { return images_; }
    EventWriter& events() noexcept { return events_; }
private:
    RecorderError error_{};
    Session session_;
    ImageWriter images_;
    EventWriter events_;
};

} // namespace dart::recorder
