#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <pthread.h>

#include "imu/estimator.hpp"
#include "imu/interface.hpp"
#include "system/write_control.hpp"

namespace dart::imu {
/* Owns the estimator shared by consumers and the log. Device lifetime encloses
 * Session lifetime. start()/stop() belong to the application lifecycle thread;
 * current()/seed()/reset() may be called by consumers while sampling runs. */
class Session final {
public:
    struct Stats {
        std::uint64_t samples = 0;
        std::uint64_t read_errors = 0;
        std::uint64_t written = 0;
        std::uint64_t dropped = 0;
        std::uint64_t io_errors = 0;
        std::uint64_t log_bytes = 0;
        bool log_limit_reached = false;
    };

    using SampleObserver = void (*)(const Sample&, void*) noexcept;

    static constexpr std::uint64_t kDefaultLogLimitBytes = 32ULL * 1024 * 1024;

    explicit Session(IImu& source, const Estimator::Config& config = {}) noexcept
        : source_(source)
        , config_(config)
        , estimator_(config) {}
    ~Session() { stop(); }
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    /* One run per instance. Each start creates a numbered file from the path base.
     * Parents must exist. The application may request an initial stationary seed;
     * reset() never silently seeds again. */
    bool start(
        const char* path, std::uint32_t period_us = 1000, bool initial_seed = false,
        std::uint64_t log_limit_bytes = kDefaultLogLimitBytes,
        platform::WriteControl* writes = nullptr, SampleObserver observer = nullptr,
        void* observer_context = nullptr) noexcept;
    void stop() noexcept;
    Estimator::Estimate current() const noexcept;
    Estimator::SeedResult seed(const Quaternionf& sensor_to_reference) noexcept;
    void reset() noexcept;
    Stats stats() const noexcept;
    const char* log_path() const noexcept { return log_path_; }

private:
    enum class EventKind { kSample, kSeed, kReset, kReadError };

    struct Event {
        EventKind kind = EventKind::kSample;
        Estimator::Estimate estimate{};
        Quaternionf requested_reference{};
        std::uint64_t sequence = 0;
        std::uint64_t recorded_ns = 0;
        std::uint64_t dropped = 0;
        std::uint32_t command_quality_events = 0;
        int read_error = -1;
        bool command_accepted = false;
    };

    static constexpr std::size_t kQueueCapacity = 512;
    static std::uint64_t now_ns() noexcept;

    void enqueue(Event event) noexcept; // Called with state_mutex_; never waits for disk.
    Estimator::SeedResult seed_locked(const Quaternionf& reference) noexcept;
    void sample_loop() noexcept;
    void writer_loop() noexcept;
    bool write_event(const Event& event) noexcept;
    bool write_log(const char* data, int length, std::size_t capacity) noexcept;
    bool open_log(const char* base_path) noexcept;
    bool begin_log(bool initial_seed) noexcept;
    void end_log() noexcept;
    bool logging_enabled() const noexcept;
    static void* sample_entry(void* context) noexcept;
    static void* writer_entry(void* context) noexcept;

    IImu& source_;
    Estimator::Config config_;
    Estimator estimator_;
    mutable std::mutex state_mutex_;
    Estimator::Estimate latest_{estimator_.current()};
    std::uint64_t sequence_ = 0;
    bool initial_seed_pending_ = false;
    std::mutex queue_mutex_;
    std::condition_variable queue_ready_;
    std::unique_ptr<Event[]> queue_{};
    std::size_t head_ = 0, tail_ = 0, count_ = 0;
    bool writer_stopping_ = false;
    bool accepting_ = false;            // Protected by state_mutex_.
    std::mutex sleep_mutex_;
    std::condition_variable sample_wake_;
    std::atomic<bool> sampling_{false};
    std::uint32_t period_us_ = 1000;
    pthread_t sample_thread_{}, writer_thread_{};
    bool sample_started_ = false, writer_started_ = false, used_ = false;
    std::FILE* file_ = nullptr;
    char log_path_[512]{};
    char base_path_[512]{};
    platform::WriteControl* writes_ = nullptr;
    SampleObserver observer_ = nullptr;
    void* observer_context_ = nullptr;
    std::atomic<bool> log_active_{false};
    std::atomic<std::uint64_t> samples_{0}, read_errors_{0}, written_{0}, dropped_{0},
        io_errors_{0};
    std::atomic<std::uint64_t> log_bytes_{0};
    std::atomic<bool> log_limit_reached_{false};
    std::uint64_t log_limit_bytes_ = kDefaultLogLimitBytes;
};

} // namespace dart::imu
