#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <pthread.h>

#include "src/recorder/frame_channel.hpp"
#include "src/recorder/storage.hpp"

namespace dart::recorder {

// One encoder producer, one file worker. Slots own their bytes until all file
// work finishes; no camera lease crosses this boundary. File I/O never holds a
// producer lock. Lifecycle calls are serialized by the Recorder component.
class BufferedStorage final {
public:
    BufferedStorage(const Config& config, FrameChannel& frames, RecorderStatus& status,
                    platform::WriteControl* writes) noexcept;
    ~BufferedStorage();
    bool prepare(std::size_t input_bytes, std::size_t output_bytes) noexcept;
    bool open() noexcept;
    void close() noexcept;
    bool available() const noexcept;
    bool submit(const EncodedImage& image, const FrameEvent& event) noexcept;
    void fail(int system_error, Error error = Error::kImageFile, const char* path = nullptr) noexcept;

private:
    struct Slot {
        std::unique_ptr<std::byte[]> bytes{};
        EncodedImage image{};
        FrameEvent event{};
    };
    static void* entry(void* context) noexcept;
    void run() noexcept;
    void write(Slot& slot) noexcept;
    void write_event(const FrameEvent& event) noexcept;
    void drain_events() noexcept;
    void close_files() noexcept;
    void publish_details() noexcept;

    const Config& config_;
    FrameChannel& frames_;
    RecorderStatus& status_;
    platform::WriteControl* writes_;
    Storage storage_;
    std::unique_ptr<Slot[]> slots_{};
    std::size_t slot_bytes_ = 0;
    std::atomic<std::uint64_t> produced_{0}, consumed_{0};
    std::atomic<bool> running_{false};
    std::mutex wake_mutex_;
    std::condition_variable ready_;
    pthread_t thread_{};
    bool started_ = false;
};

} // namespace dart::recorder
