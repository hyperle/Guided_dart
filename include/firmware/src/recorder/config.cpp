module;
#include <array>
#include <chrono>
#include <cstdint>
#include <string_view>

export module dart.recorder.config;



export namespace dart::recorder {

enum class EventDecision : std::uint8_t {
    kAccepted,
    kDownsampled,
    kBusy,
    kStopped,
    kInvalidFrame,
};

class FrameEvent {
public:
    std::uint64_t input_index = 0;
    std::uint64_t source_sequence = 0;
    std::uint64_t source_pts = 0;
    std::int64_t received_us = 0;
    bool source_pts_timebase_validated = false;
    std::uint64_t image_index = 0;
    std::uint64_t file_offset = 0;
    std::uint64_t file_bytes = 0;
    EventDecision decision = EventDecision::kStopped;
    std::array<char, 48> image_name{};
};

using FrameEventSink = void (*)(const FrameEvent&, void*) noexcept;

enum class ImageFormat : std::uint8_t { kPgm, kPbm, kPng, kDbv };

class Config {
public:
    static constexpr std::uint64_t kMaxFileBytes = 500'000'000;
    static constexpr std::uint64_t kMaxStorageBytes = 2'000'000'000;
    static constexpr std::uint64_t kStorageReserveBytes = 64U * 1024U * 1024U;
    static constexpr std::uint32_t kImagesPerGroup = 32;
    static constexpr std::uint32_t kGroupsPerBucket = 32;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    /* Target images per second, sampled using the monotonic receipt timestamp.
     * Checked before queue availability; busy slots still drop without waiting. */
    std::uint32_t fps = 60;
    /* Total image bytes per session, including headers. Smaller caps are useful for
     * short runs; the 500 MB ceiling cannot be disabled or raised. */
    std::uint64_t max_file_bytes = kMaxFileBytes;
    /* Total used bytes on the output volume, including earlier sessions and
     * other files. Zero leaves only the physical free-space guard enabled.
     * dart_app applies the 2 GB ceiling to recordings under /data. */
    std::uint64_t max_storage_bytes = 0;
    /* Hard copied-image slot cap, independent of camera lease capacity. */
    std::uint32_t queue_slots = 2;
    /* Separate bounded metadata queue for one CSV row per enqueued submission; drops
     * are counted instead of blocking capture. */
    std::uint32_t event_slots = 512;
    /* Recheck free space at most this many images apart. DBV charges actual
     * record bytes; other formats retain the conservative 2 MiB charge. */
    std::uint32_t storage_check_frames = 32;
    ImageFormat image_format = ImageFormat::kPgm;
    std::uint8_t pbm_threshold = 128;
    std::chrono::milliseconds flush_period{1000};
    std::string_view output_directory = "records";
    std::uint32_t keep_sessions = 0; /* 0 = never remove earlier sessions */
    FrameEventSink event_sink = nullptr;
    void* event_sink_context = nullptr;

    bool variable_record_size() const noexcept {
        return image_format == ImageFormat::kPng || image_format == ImageFormat::kDbv;
    }
};

} // namespace dart::recorder
