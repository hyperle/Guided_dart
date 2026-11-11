#include "runtime/imu/session.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <new>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>

#include "imu/estimator.hpp"
#include "imu/interface.hpp"
#include "storage/directories.hpp"
#include "system/write_control.hpp"

namespace dart::imu {

std::uint64_t Session::now_ns() noexcept {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

bool Session::logging_enabled() const noexcept {
    return writes_ == nullptr || writes_->logging_enabled();
}

void* Session::sample_entry(void* context) noexcept {
    static_cast<Session*>(context)->sample_loop();
    return nullptr;
}

void* Session::writer_entry(void* context) noexcept {
    static_cast<Session*>(context)->writer_loop();
    return nullptr;
}

bool Session::open_log(const char* base_path) noexcept {
    const auto length = std::strlen(base_path);
    if (length >= sizeof(log_path_)) {
        errno = ENAMETOOLONG;
        return false;
    }
    struct stat existing{};
    if (::stat(base_path, &existing) == 0) {
        if (!S_ISREG(existing.st_mode)) {
            errno = EINVAL;
            return false;
        }
    } else if (errno != ENOENT) {
        return false;
    }
    const char* slash = std::strrchr(base_path, '/');
    const char* name = slash == nullptr ? base_path : slash + 1;
    if (name[0] == '\0' || std::strcmp(name, ".") == 0 || std::strcmp(name, "..") == 0) {
        errno = EINVAL;
        return false;
    }
    const char* extension = std::strrchr(name, '.');
    if (extension == name)
        extension = nullptr;
    const char* suffix = extension == nullptr ? ".log" : extension;
    const auto stem_length =
        extension == nullptr ? length : static_cast<std::size_t>(extension - base_path);
    const auto name_stem_length = stem_length - static_cast<std::size_t>(name - base_path);
    if (stem_length + 7 + std::strlen(suffix) >= sizeof(log_path_)) {
        errno = ENAMETOOLONG;
        return false;
    }
    char directory[sizeof(log_path_)] = ".";
    if (slash != nullptr) {
        const auto size = slash == base_path ? 1U : static_cast<std::size_t>(slash - base_path);
        std::memcpy(directory, base_path, size);
        directory[size] = '\0';
    }
    DIR* entries = ::opendir(directory);
    if (entries == nullptr)
        return false;
    unsigned newest = 0;
    for (;;) {
        errno = 0;
        const auto* entry = ::readdir(entries);
        if (entry == nullptr) {
            const int saved = errno;
            if (::closedir(entries) != 0)
                return false;
            if (saved != 0) {
                errno = saved;
                return false;
            }
            break;
        }
        const char* candidate = entry->d_name;
        if (std::strlen(candidate) != name_stem_length + 7 + std::strlen(suffix)
            || std::memcmp(candidate, name, name_stem_length) != 0
            || candidate[name_stem_length] != '_'
            || std::strcmp(candidate + name_stem_length + 7, suffix) != 0)
            continue;
        unsigned number = 0;
        for (std::size_t i = name_stem_length + 1; i < name_stem_length + 7; ++i) {
            if (candidate[i] < '0' || candidate[i] > '9') {
                number = 0;
                break;
            }
            number = number * 10U + static_cast<unsigned>(candidate[i] - '0');
        }
        if (number > newest)
            newest = number;
    }
    // Exclusive creation also protects against another process choosing the same number.
    unsigned number = newest + 1;
    for (; number <= 999999; ++number) {
        const int size = std::snprintf(
            log_path_, sizeof(log_path_), "%.*s_%06u%s", static_cast<int>(stem_length), base_path,
            number, suffix);
        if (size < 0 || static_cast<std::size_t>(size) >= sizeof(log_path_)) {
            errno = ENAMETOOLONG;
            break;
        }
        const int fd = ::open(log_path_, O_WRONLY | O_CREAT | O_EXCL, 0666);
        if (fd < 0) {
            if (errno == EEXIST)
                continue;
            break;
        }
        file_ = ::fdopen(fd, "w");
        if (file_ != nullptr)
            return true;
        const int saved = errno;
        ::close(fd);
        ::unlink(log_path_);
        errno = saved;
        break;
    }
    if (number > 999999)
        errno = ENOSPC;
    log_path_[0] = '\0';
    return false;
}

bool Session::begin_log(bool initial_seed) noexcept {
    if (writes_ != nullptr)
        storage::ensure_parent(base_path_);
    if (!open_log(base_path_)) {
        ++io_errors_;
        return false;
    }
    log_bytes_.store(0);
    log_limit_reached_.store(false);
    char header[2048];
    const int header_length = std::snprintf(
        header, sizeof(header),
        "# imu_log version=1 start_ns=%llu period_us=%u initial_seed=%d log_limit_bytes=%llu "
        "queue_capacity=%zu queue_bytes=%zu max_interval_ns=%llu "
        "max_acceleration_m_s2=%.9g max_rate_rad_s=%.9g "
        "max_acceleration_step_m_s2=%.9g max_rate_step_rad_s=%.9g "
        "seed_gravity_tolerance_m_s2=%.9g seed_max_rate_rad_s=%.9g\n"
        "event,sequence,recorded_ns,timestamp_ns,generation,quality_events,"
        "sample_accepted,integrated,has_reference_frame,command_accepted,command_quality_events,"
        "read_error,dropped,raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz,"
        "ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s,temperature_c,sample_flags,"
        "qw,qx,qy,qz,roll_rad,pitch_rad,yaw_rad,wx_rad_s,wy_rad_s,wz_rad_s,"
        "linear_ax_m_s2,linear_ay_m_s2,linear_az_m_s2,vx_m_s,vy_m_s,vz_m_s,"
        "rotation_x_rad,rotation_y_rad,rotation_z_rad,seed_qw,seed_qx,seed_qy,seed_qz\n",
        static_cast<unsigned long long>(now_ns()), period_us_, initial_seed ? 1 : 0,
        static_cast<unsigned long long>(log_limit_bytes_), kQueueCapacity,
        sizeof(Event) * kQueueCapacity,
        static_cast<unsigned long long>(config_.max_integration_interval_ns),
        config_.max_acceleration_m_s2, config_.max_angular_velocity_rad_s,
        config_.max_acceleration_step_m_s2, config_.max_angular_velocity_step_rad_s,
        config_.seed_gravity_tolerance_m_s2, config_.seed_max_angular_velocity_rad_s);
    const bool header_ok = write_log(header, header_length, sizeof(header));
    if (!header_ok && !log_limit_reached_.load()) {
        ++io_errors_;
        end_log();
        return false;
    }
    log_active_.store(true);
    return true;
}

bool Session::start(
    const char* path, std::uint32_t period_us, bool initial_seed, std::uint64_t log_limit_bytes,
    platform::WriteControl* writes, SampleObserver observer, void* observer_context) noexcept {
    if (used_ || path == nullptr || path[0] == '\0' || std::strlen(path) >= sizeof(base_path_)
        || period_us == 0 || log_limit_bytes == 0)
        return false;
    queue_.reset(new (std::nothrow) Event[kQueueCapacity]);
    if (!queue_)
        return false;
    std::strcpy(base_path_, path);
    writes_ = writes;
    observer_ = observer;
    observer_context_ = observer_context;
    // Rejected path probes before the first successful start do not belong to
    // the subsequent sampling session's I/O counters.
    io_errors_.store(0);
    log_limit_bytes_ = log_limit_bytes;
    period_us_ = period_us;
    initial_seed_pending_ = initial_seed;
    writer_stopping_ = false;
    // Logging failures must not disable acquisition in the controlled service.
    if (logging_enabled() && !begin_log(initial_seed) && writes_ == nullptr)
        return false;
    used_ = true;
    if (::pthread_create(&writer_thread_, nullptr, &Session::writer_entry, this) != 0) {
        stop();
        return false;
    }
    writer_started_ = true;
    {
        std::lock_guard lock(state_mutex_);
        accepting_ = true;
    }
    sampling_.store(true);
    if (::pthread_create(&sample_thread_, nullptr, &Session::sample_entry, this) != 0) {
        stop();
        return false;
    }
    sample_started_ = true;
    return true;
}

void Session::stop() noexcept {
    {
        std::lock_guard lock(sleep_mutex_);
        sampling_.store(false);
    }
    {
        std::lock_guard lock(state_mutex_);
        imu_working_.store(false);
    }
    sample_wake_.notify_all();
    if (sample_started_) {
        (void)::pthread_join(sample_thread_, nullptr);
        sample_started_ = false;
    }
    {
        std::lock_guard lock(state_mutex_);
        accepting_ = false;
    }
    {
        std::lock_guard lock(queue_mutex_);
        writer_stopping_ = true;
    }
    queue_ready_.notify_all();
    if (writer_started_) {
        (void)::pthread_join(writer_thread_, nullptr);
        writer_started_ = false;
    }
    end_log();
}

void Session::end_log() noexcept {
    log_active_.store(false);
    if (file_ != nullptr) {
        char footer[256];
        const int footer_length = std::snprintf(
            footer, sizeof(footer),
            "# session_end samples=%llu read_errors=%llu written=%llu dropped=%llu "
            "io_errors=%llu\n",
            static_cast<unsigned long long>(samples_.load()),
            static_cast<unsigned long long>(read_errors_.load()),
            static_cast<unsigned long long>(written_.load()),
            static_cast<unsigned long long>(dropped_.load()),
            static_cast<unsigned long long>(io_errors_.load()));
        if (!write_log(footer, footer_length, sizeof(footer)) && !log_limit_reached_.load())
            ++io_errors_;
        if (std::fclose(file_) != 0)
            ++io_errors_;
        file_ = nullptr;
    }
}

Estimator::Estimate Session::current() const noexcept {
    std::lock_guard lock(state_mutex_);
    return latest_;
}

Estimator::SeedResult Session::seed(const Quaternionf& reference) noexcept {
    std::lock_guard lock(state_mutex_);
    initial_seed_pending_ = false;
    return seed_locked(reference);
}

Estimator::SeedResult Session::seed_locked(const Quaternionf& reference) noexcept {
    const auto result = estimator_.seed(latest_.sample, reference);
    if (result.accepted)
        latest_ = estimator_.current();
    Event event{};
    event.kind = EventKind::kSeed;
    event.estimate = latest_;
    event.requested_reference = reference;
    event.command_accepted = result.accepted;
    event.command_quality_events = result.quality_events;
    enqueue(event);
    return result;
}

void Session::reset() noexcept {
    std::lock_guard lock(state_mutex_);
    initial_seed_pending_ = false;
    estimator_.reset();
    latest_ = estimator_.current();
    Event event{};
    event.kind = EventKind::kReset;
    event.estimate = latest_;
    event.command_accepted = true;
    enqueue(event);
}

Session::Stats Session::stats() const noexcept {
    return {samples_.load(),   read_errors_.load(), written_.load(),          dropped_.load(),
            io_errors_.load(), log_bytes_.load(),   log_limit_reached_.load()};
}

void Session::enqueue(Event event) noexcept {
    if (!accepting_ || !logging_enabled() || !log_active_.load())
        return;
    event.sequence = ++sequence_;
    event.recorded_ns = now_ns();
    if (log_limit_reached_.load()) {
        ++dropped_;
        return;
    }
    {
        // The writer only holds this lock to copy a single event, never for I/O.
        std::lock_guard lock(queue_mutex_);
        if (count_ == kQueueCapacity) {
            ++dropped_;
            return;
        }
        event.dropped = dropped_.load();
        queue_[tail_] = event;
        tail_ = (tail_ + 1) % kQueueCapacity;
        ++count_;
    }
    queue_ready_.notify_one();
}

void Session::sample_loop() noexcept {
    auto next = std::chrono::steady_clock::now();
    while (sampling_.load()) {
        const auto sample = source_.read_sample();
        if (sample && observer_ != nullptr)
            observer_(*sample, observer_context_);
        {
            std::lock_guard lock(state_mutex_);
            imu_working_.store(sampling_.load() && sample.has_value());
            Event event{};
            if (sample) {
                ++samples_;
                latest_ = estimator_.update(*sample);
                event.estimate = latest_;
                enqueue(event);
                if (initial_seed_pending_) {
                    const auto seeded = seed_locked({});
                    initial_seed_pending_ = !seeded.accepted;
                }
            } else {
                ++read_errors_;
                event.kind = EventKind::kReadError;
                event.estimate = latest_;
                event.read_error = static_cast<int>(sample.error());
                enqueue(event);
            }
        }
        next += std::chrono::microseconds{period_us_};
        const auto now = std::chrono::steady_clock::now();
        if (next < now)
            next = now + std::chrono::microseconds{period_us_};
        std::unique_lock lock(sleep_mutex_);
        sample_wake_.wait_until(lock, next, [this] { return !sampling_.load(); });
    }
}

void Session::writer_loop() noexcept {
    bool attempted = file_ != nullptr || logging_enabled();
    for (;;) {
        Event event{};
        bool stopping = false;
        {
            std::unique_lock lock(queue_mutex_);
            queue_ready_.wait_for(lock, std::chrono::milliseconds{20}, [this] {
                return count_ > 0 || writer_stopping_;
            });
            stopping = writer_stopping_;
            if (count_ > 0) {
                event = queue_[head_];
                head_ = (head_ + 1) % kQueueCapacity;
                --count_;
            }
        }
        if (event.sequence != 0) {
            if (file_ == nullptr)
                ++dropped_;
            else if (write_event(event))
                ++written_;
            else if (log_limit_reached_.load())
                ++dropped_;
            else {
                ++io_errors_;
                end_log();
            }
        }
        bool empty = false;
        {
            std::lock_guard lock(queue_mutex_);
            empty = count_ == 0;
        }
        const bool wanted = logging_enabled();
        if ((!wanted || stopping) && empty) {
            end_log();
            attempted = false;
            if (stopping)
                break;
        } else if (wanted && !stopping && file_ == nullptr && !attempted) {
            attempted = true;
            (void)begin_log(false);
        }
    }
}

bool Session::write_event(const Event& event) noexcept {
    if (log_limit_reached_.load())
        return false;
    const auto& e = event.estimate;
    const auto& s = e.sample;
    const auto& a = s.raw_counts.accelerometer;
    const auto& g = s.raw_counts.gyroscope;
    const auto& q = e.attitude;
    const auto& r = event.requested_reference;
    const char* kind = "sample";
    switch (event.kind) {
    case EventKind::kSample: break;
    case EventKind::kSeed: kind = "seed"; break;
    case EventKind::kReset: kind = "reset"; break;
    case EventKind::kReadError: kind = "read_error"; break;
    }
    char row[2048];
    const int length = std::snprintf(
        row, sizeof(row),
        "%s,%llu,%llu,%llu,%llu,%u,%d,%d,%d,%d,%u,%d,%llu,"
        "%d,%d,%d,%d,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,"
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        kind, static_cast<unsigned long long>(event.sequence),
        static_cast<unsigned long long>(event.recorded_ns),
        static_cast<unsigned long long>(s.timestamp_ns),
        static_cast<unsigned long long>(e.generation), e.quality_events, e.sample_accepted,
        e.integrated, e.has_reference_frame, event.command_accepted, event.command_quality_events,
        event.read_error, static_cast<unsigned long long>(event.dropped), a.x, a.y, a.z, g.x, g.y,
        g.z, s.acceleration_m_s2.x, s.acceleration_m_s2.y, s.acceleration_m_s2.z,
        s.angular_velocity_rad_s.x, s.angular_velocity_rad_s.y, s.angular_velocity_rad_s.z,
        s.temperature_c, s.flags, q.w, q.x, q.y, q.z, e.attitude_rpy_rad.x, e.attitude_rpy_rad.y,
        e.attitude_rpy_rad.z, e.angular_velocity_rad_s.x, e.angular_velocity_rad_s.y,
        e.angular_velocity_rad_s.z, e.linear_acceleration_m_s2.x, e.linear_acceleration_m_s2.y,
        e.linear_acceleration_m_s2.z, e.velocity_m_s.x, e.velocity_m_s.y, e.velocity_m_s.z,
        e.cumulative_rotation_rad.x, e.cumulative_rotation_rad.y, e.cumulative_rotation_rad.z, r.w,
        r.x, r.y, r.z);
    return write_log(row, length, sizeof(row));
}

bool Session::write_log(const char* data, int length, std::size_t capacity) noexcept {
    if (log_limit_reached_.load())
        return false;
    if (length < 0 || static_cast<std::size_t>(length) >= capacity)
        return false;
    const auto bytes = static_cast<std::uint64_t>(length);
    const auto used = log_bytes_.load();
    // Each session owns a new file and an independent budget.
    if (used >= log_limit_bytes_ || bytes > log_limit_bytes_ - used) {
        log_limit_reached_.store(true);
        return false;
    }
    const auto written = std::fwrite(data, 1, static_cast<std::size_t>(length), file_);
    log_bytes_.fetch_add(written);
    return written == static_cast<std::size_t>(length);
}

} // namespace dart::imu
