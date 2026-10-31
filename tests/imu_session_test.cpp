#include <cstdint>
#include <cstddef>
#include <expected>
#include <cstring>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

import dart.runtime.imu.session;
import dart.system.write_control;

namespace dart::tests {

static std::atomic<bool> fail_log_writes{false};

extern "C" std::size_t __real_fwrite(const void*, std::size_t, std::size_t, std::FILE*);
extern "C" std::size_t
__wrap_fwrite(const void* data, std::size_t size, std::size_t count, std::FILE* file) {
    if (fail_log_writes.load()) {
        errno = ENOSPC;
        return 0;
    }
    return __real_fwrite(data, size, count, file);
}

class ImuSessionTest final {
    class Source final : public imu::IImu {
    public:
        std::expected<imu::Sample, imu::ErrorCode> read_sample() noexcept override {
            std::lock_guard lock(mutex_);
            auto result = pending_;
            pending_ = std::unexpected(imu::ErrorCode::kBusTransfer);
            return result;
        }
        void push(const imu::Sample& sample) noexcept {
            std::lock_guard lock(mutex_);
            pending_ = sample;
        }

    private:
        std::mutex mutex_;
        std::expected<imu::Sample, imu::ErrorCode> pending_{
            std::unexpected(imu::ErrorCode::kBusTransfer)};
    };

    struct Expected {
        std::string kind;
        imu::Estimator::Estimate estimate;
        bool command_accepted = false;
        std::uint32_t command_quality = 0;
        imu::Quaternionf reference{};
    };

    static imu::Estimator::Estimate
        feed(imu::Session& session, Source& source, const imu::Sample& sample) {
        const auto count = session.stats().samples;
        source.push(sample);
        for (int i = 0; i < 500 && session.stats().samples == count; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(session.stats().samples == count + 1);
        return session.current();
    }

    static std::vector<std::string> split(const std::string& line) {
        std::vector<std::string> fields;
        std::size_t begin = 0;
        for (;;) {
            const auto end = line.find(',', begin);
            fields.push_back(line.substr(begin, end - begin));
            if (end == std::string::npos)
                return fields;
            begin = end + 1;
        }
    }

    static std::string read(const std::filesystem::path& path) {
        std::ifstream input{path};
        assert(input);
        return {std::istreambuf_iterator<char>{input}, {}};
    }

    static void verify(const std::map<std::string, std::string>& row, const Expected& expected) {
        const auto& e = expected.estimate;
        const auto& s = e.sample;
        const auto integer = [&](const char* name, std::uint64_t value) {
            assert(std::stoull(row.at(name)) == value);
        };
        const auto scalar = [&](const std::string& name, float value) {
            const float logged = std::strtof(row.at(name).c_str(), nullptr);
            assert(logged == value || (std::isnan(logged) && std::isnan(value)));
        };
        const auto vector = [&](const char* x, const char* y, const char* z, imu::Vector3f value) {
            scalar(x, value.x);
            scalar(y, value.y);
            scalar(z, value.z);
        };
        assert(row.at("event") == expected.kind);
        integer("timestamp_ns", s.timestamp_ns);
        integer("generation", e.generation);
        integer("quality_events", e.quality_events);
        integer("sample_accepted", e.sample_accepted);
        integer("integrated", e.integrated);
        integer("has_reference_frame", e.has_reference_frame);
        integer("command_accepted", expected.command_accepted);
        integer("command_quality_events", expected.command_quality);
        integer("sample_flags", s.flags);
        assert(std::stoi(row.at("raw_ax")) == s.raw_counts.accelerometer.x);
        assert(std::stoi(row.at("raw_ay")) == s.raw_counts.accelerometer.y);
        assert(std::stoi(row.at("raw_az")) == s.raw_counts.accelerometer.z);
        assert(std::stoi(row.at("raw_gx")) == s.raw_counts.gyroscope.x);
        assert(std::stoi(row.at("raw_gy")) == s.raw_counts.gyroscope.y);
        assert(std::stoi(row.at("raw_gz")) == s.raw_counts.gyroscope.z);
        vector("ax_m_s2", "ay_m_s2", "az_m_s2", s.acceleration_m_s2);
        vector("gx_rad_s", "gy_rad_s", "gz_rad_s", s.angular_velocity_rad_s);
        scalar("temperature_c", s.temperature_c);
        scalar("qw", e.attitude.w);
        scalar("qx", e.attitude.x);
        scalar("qy", e.attitude.y);
        scalar("qz", e.attitude.z);
        vector("roll_rad", "pitch_rad", "yaw_rad", e.attitude_rpy_rad);
        vector("wx_rad_s", "wy_rad_s", "wz_rad_s", e.angular_velocity_rad_s);
        vector("linear_ax_m_s2", "linear_ay_m_s2", "linear_az_m_s2", e.linear_acceleration_m_s2);
        vector("vx_m_s", "vy_m_s", "vz_m_s", e.velocity_m_s);
        vector("rotation_x_rad", "rotation_y_rad", "rotation_z_rad", e.cumulative_rotation_rad);
        scalar("seed_qw", expected.reference.w);
        scalar("seed_qx", expected.reference.x);
        scalar("seed_qy", expected.reference.y);
        scalar("seed_qz", expected.reference.z);
    }

public:
    static void run() {
        char directory[] = "/tmp/dart-imu-session-XXXXXX";
        assert(::mkdtemp(directory) != nullptr);
        const auto path = std::filesystem::path{directory} / "imu.log";
        const std::string legacy = "previous boot log; do not append or overwrite\n";
        std::ofstream{path} << legacy;
        Source source;
        imu::Session session{source};
        assert(!session.current().has_reference_frame);
        assert(!session.start(nullptr));
        assert(!session.start(path.c_str(), 0));
        assert(!session.start(directory));
        assert(session.start(path.c_str()));
        assert(std::filesystem::path{session.log_path()} == path.parent_path() / "imu_000001.log");
        for (int i = 0; i < 500 && session.stats().read_errors == 0; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(session.stats().read_errors > 0);
        std::vector<Expected> expected;
        imu::Sample sample{};
        sample.timestamp_ns = 1'000'000'000;
        sample.acceleration_m_s2.z = 9.80665F;
        sample.raw_counts.accelerometer = {-123, 456, 2048};
        sample.raw_counts.gyroscope = {321, -654, 987};
        sample.temperature_c = 26.125F;
        auto state = feed(session, source, sample);
        assert(state.sample_accepted && !state.integrated && !state.has_reference_frame);
        expected.push_back({"sample", state});
        auto seeded = session.seed({});
        assert(seeded.accepted && seeded.generation == 1);
        expected.push_back({"seed", session.current(), true});

        sample.timestamp_ns += 10'000'000;
        sample.acceleration_m_s2.x = 2;
        sample.angular_velocity_rad_s.z = 1;
        state = feed(session, source, sample);
        assert(state.integrated && state.velocity_m_s.x > 0 && state.cumulative_rotation_rad.z > 0);
        expected.push_back({"sample", state});
        seeded = session.seed({});             // Moving samples cannot start a new integral.
        assert(!seeded.accepted && seeded.generation == 1);
        expected.push_back({"seed", session.current(), false, seeded.quality_events});

        state = feed(session, source, sample); // Duplicate timestamp, preserve rejection flags.
        assert(!state.sample_accepted);
        expected.push_back({"sample", state});
        sample.timestamp_ns += 10'000'000;
        sample.flags = static_cast<std::uint32_t>(imu::SampleFlag::kAccelerometerSaturated);
        state = feed(session, source, sample);
        assert(!state.sample_accepted);
        expected.push_back({"sample", state});
        sample.flags = 0;
        sample.timestamp_ns += 10'000'000;
        sample.acceleration_m_s2.x = std::numeric_limits<float>::quiet_NaN();
        state = feed(session, source, sample);
        assert(!state.sample_accepted);
        expected.push_back({"sample", state});
        sample.acceleration_m_s2.x = 0;
        sample.angular_velocity_rad_s.z = 0;
        sample.timestamp_ns += 500'000'000;
        state = feed(session, source, sample);
        assert(state.sample_accepted && !state.integrated && state.quality_events != 0);
        expected.push_back({"sample", state});

        const imu::Quaternionf reference{0.70710677F, 0, 0, 0.70710677F};
        seeded = session.seed(reference);
        state = session.current();
        assert(seeded.accepted && state.generation == 2);
        assert(state.velocity_m_s.x == 0 && state.cumulative_rotation_rad.z == 0);
        expected.push_back({"seed", state, true, 0, reference});
        const imu::Quaternionf invalid{0, 0, 0, 0};
        seeded = session.seed(invalid);
        assert(!seeded.accepted && seeded.generation == 2);
        expected.push_back({"seed", session.current(), false, seeded.quality_events, invalid});
        sample.timestamp_ns += 10'000'000;
        sample.acceleration_m_s2.x = 2;
        state = feed(session, source, sample);
        assert(state.integrated && state.velocity_m_s.y > 0);
        expected.push_back({"sample", state});
        session.reset();
        state = session.current();
        assert(state.generation == 3 && !state.has_reference_frame);
        expected.push_back({"reset", state, true});
        sample.timestamp_ns += 10'000'000;
        state = feed(session, source, sample);
        assert(!state.integrated && !state.has_reference_frame);
        expected.push_back({"sample", state});
        session.stop();
        session.stop();
        assert(!session.start(path.c_str()));
        const auto stats = session.stats();
        assert(stats.dropped == 0 && stats.io_errors == 0);
        assert(stats.read_errors > 0);
        assert(stats.written == expected.size() + stats.read_errors);

        std::ifstream input{session.log_path()};
        std::string line;
        assert(std::getline(input, line) && line.find("# imu_log version=1") == 0);
        assert(std::getline(input, line));
        const auto header = split(line);
        std::size_t next = 0;
        std::uint64_t previous_sequence = 0;
        bool footer = false;
        while (std::getline(input, line)) {
            if (line.starts_with("# session_end")) {
                footer = true;
                continue;
            }
            const auto fields = split(line);
            assert(fields.size() == header.size());
            std::map<std::string, std::string> row;
            for (std::size_t i = 0; i < fields.size(); ++i)
                row.emplace(header[i], fields[i]);
            assert(std::stoull(row.at("sequence")) == ++previous_sequence);
            assert(std::stoull(row.at("recorded_ns")) > 0);
            if (row.at("event") == "read_error") {
                assert(
                    std::stoi(row.at("read_error"))
                    == static_cast<int>(imu::ErrorCode::kBusTransfer));
                continue;
            }
            assert(next < expected.size());
            verify(row, expected[next++]);
        }
        assert(footer && next == expected.size());
        assert(read(path) == legacy);
        const auto first_log = read(session.log_path());

        // Startup seed is a one-time request. Reset must wait for a new command.
        Source initial_source;
        sample = {};
        sample.acceleration_m_s2.z = 9.80665F;
        sample.timestamp_ns = 2'000'000'000;
        initial_source.push(sample);
        imu::Session initial{initial_source};
        assert(initial.start(path.c_str(), 1000, true));
        assert(std::filesystem::path{initial.log_path()} == path.parent_path() / "imu_000002.log");
        for (int i = 0; i < 500 && !initial.current().has_reference_frame; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(initial.current().generation == 1);
        initial.reset();
        sample.timestamp_ns += 10'000'000;
        assert(!feed(initial, initial_source, sample).has_reference_frame);
        initial.stop();
        assert(read(session.log_path()) == first_log);
        assert(read(initial.log_path()).starts_with("# imu_log version=1"));

        // Reaching the disk budget must leave sampling and estimation running.
        const auto limited_path = std::filesystem::path{directory} / "limited.log";
        Source limited_source;
        imu::Session limited{limited_source};
        constexpr std::uint64_t limit = 4096;
        assert(limited.start(limited_path.c_str(), 1000, false, limit));
        for (int i = 0; i < 500 && !limited.stats().log_limit_reached; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(limited.stats().log_limit_reached);
        sample = {};
        sample.acceleration_m_s2.z = 9.80665F;
        sample.timestamp_ns = 3'000'000'000;
        assert(feed(limited, limited_source, sample).sample_accepted);
        assert(limited.seed({}).accepted);
        limited.stop();
        assert(limited.stats().io_errors == 0 && limited.stats().dropped > 0);
        const auto capped_size = std::filesystem::file_size(limited.log_path());
        assert(capped_size <= limit && capped_size == limited.stats().log_bytes);

        // A new boot gets its own budget; a capped old file is never reopened.
        const auto preserved = read(limited.log_path());
        imu::Session resumed{limited_source};
        assert(resumed.start(limited_path.c_str(), 10'000, false, limit));
        assert(std::filesystem::path{resumed.log_path()} == path.parent_path() / "limited_000002.log");
        assert(!resumed.stats().log_limit_reached);
        sample.timestamp_ns += 10'000'000;
        assert(feed(resumed, limited_source, sample).sample_accepted);
        resumed.stop();
        assert(read(limited.log_path()) == preserved);
        assert(resumed.stats().io_errors == 0 && resumed.stats().written > 0);
        assert(resumed.stats().log_bytes <= limit);

        // Holes, unrelated names, and interrupted empty files cannot reuse an old number.
        const auto occupied = path.parent_path() / "imu_000010.log";
        std::ofstream{occupied};
        std::ofstream{path.parent_path() / "imu_000999.log.bak"};
        imu::Session later{source};
        assert(later.start(path.c_str()));
        assert(std::filesystem::path{later.log_path()} == path.parent_path() / "imu_000011.log");
        later.stop();
        assert(std::filesystem::file_size(occupied) == 0);
        assert(read(session.log_path()) == first_log && read(path) == legacy);

        // Concurrent creators must reserve different files without truncating either one.
        Source other_source;
        imu::Session concurrent_a{source}, concurrent_b{other_source};
        std::atomic<bool> go{false};
        const auto create = [&](imu::Session& current) {
            while (!go.load())
                std::this_thread::yield();
            assert(current.start(path.c_str()));
        };
        std::thread a{[&] { create(concurrent_a); }};
        std::thread b{[&] { create(concurrent_b); }};
        go.store(true);
        a.join();
        b.join();
        assert(std::strcmp(concurrent_a.log_path(), concurrent_b.log_path()) != 0);
        concurrent_a.stop();
        concurrent_b.stop();
        assert(read(concurrent_a.log_path()).starts_with("# imu_log version=1"));
        assert(read(concurrent_b.log_path()).starts_with("# imu_log version=1"));

        imu::Session no_extension{source}, custom_extension{source};
        assert(no_extension.start((path.parent_path() / "raw").c_str()));
        assert(std::filesystem::path{no_extension.log_path()} == path.parent_path() / "raw_000001.log");
        no_extension.stop();
        assert(custom_extension.start((path.parent_path() / "raw.csv").c_str()));
        assert(std::filesystem::path{custom_extension.log_path()} == path.parent_path() / "raw_000001.csv");
        custom_extension.stop();

        imu::Session invalid_path{source};
        assert(!invalid_path.start((std::string(directory) + '/').c_str()));
        assert(!invalid_path.start((path.parent_path() / "missing/imu.log").c_str()));
        assert(!invalid_path.start(std::string(512, 'x').c_str()));
        const auto exhausted_path = path.parent_path() / "full.log";
        std::ofstream{path.parent_path() / "full_999999.log"} << legacy;
        assert(!invalid_path.start(exhausted_path.c_str()));
        assert(read(path.parent_path() / "full_999999.log") == legacy);

        imu::Session full{source};
        fail_log_writes.store(true);
        assert(!full.start((path.parent_path() / "disk-error.log").c_str()));
        fail_log_writes.store(false);
        assert(full.stats().io_errors > 0);

        // An active logging fault closes only the log. Acquisition survives and
        // a later off/on request creates another numbered file on this session.
        platform::WriteControl writes;
        assert(writes.start((path.parent_path() / "write.pause").c_str()));
        Source fault_source;
        imu::Session fault{fault_source};
        const auto fault_path = path.parent_path() / "fault.log";
        assert(fault.start(fault_path.c_str(), 1000, false,
                           imu::Session::kDefaultLogLimitBytes, &writes));
        fail_log_writes.store(true);
        for (int i = 0; i < 500 && fault.stats().io_errors == 0; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(fault.stats().io_errors > 0);
        fail_log_writes.store(false);
        sample = {};
        sample.acceleration_m_s2.z = 9.80665F;
        sample.timestamp_ns = 4'000'000'000;
        assert(feed(fault, fault_source, sample).sample_accepted);
        writes.request_logging(false);
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
        writes.request_logging(true);
        const auto resumed_fault_path = path.parent_path() / "fault_000002.log";
        for (int i = 0; i < 500 && !std::filesystem::exists(resumed_fault_path); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(std::filesystem::exists(resumed_fault_path));
        sample.timestamp_ns += 10'000'000;
        assert(feed(fault, fault_source, sample).sample_accepted);
        fault.stop();
        assert(read(resumed_fault_path).starts_with("# imu_log version=1"));
        std::filesystem::remove_all(directory);
    }
};

} // namespace dart::tests

int main() { dart::tests::ImuSessionTest::run(); }
