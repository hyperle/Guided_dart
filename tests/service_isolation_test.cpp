#include <csignal>
#include <cstdint>
#include <expected>
#include <memory>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <unistd.h>
#include "camera/error_codes.h"

#include "src/startup/services.hpp"

#include "camera/error.hpp"
#include "camera/interface.hpp"
#include "imu/sample.hpp"

#include "imu/interface.hpp"
#include "runtime/imu/session.hpp"
#include "src/startup/argument_parser.hpp"
#include "logging/logger.hpp"
#include "system/run_control.hpp"
#include "system/write_control.hpp"
#include "system/device_services.hpp"

#include "src/diagnostics/camera_error.hpp"

namespace dart::tests {

class ServiceIsolationTest final {
    enum class Failure { kCreate, kStart, kAcquire, kRecorder, kNone };

    class Source final : public imu::IImu {
    public:
        std::atomic<unsigned> reads{0};
        std::expected<imu::Sample, imu::ErrorCode> read_sample() noexcept override {
            ++reads;
            imu::Sample value{};
            value.timestamp_ns =
                static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                               std::chrono::steady_clock::now().time_since_epoch())
                                               .count());
            value.acceleration_m_s2.z = 9.80665F;
            return value;
        }
    };

    static camera::CameraError failure(camera::Operation operation) noexcept {
        failure_reads_ = source_->reads.load();
        camera::CallContext context{};
        context.operation = operation;
        return camera::CameraError{DART_CAMERA_E_SENSOR_PROBE_FAILED, context};
    }

    class Camera final : public camera::ICamera {
    public:
        ~Camera() override { ++destroyed_; }
        std::expected<void, camera::CameraError> start() noexcept override {
            if (mode_ == Failure::kStart)
                return std::unexpected(failure(camera::Operation::kStart));
            state_ = camera::State::kStarted;
            return {};
        }
        std::expected<camera::AcquireResult, camera::CameraError>
            acquire(std::chrono::milliseconds) noexcept override {
            ++acquires_;
            if (mode_ == Failure::kAcquire)
                return std::unexpected(failure(camera::Operation::kAcquire));
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
            return camera::AcquireResult{};
        }
        std::expected<void, camera::CameraError> stop() noexcept override {
            ++stops_;
            state_ = camera::State::kInitialized;
            return {};
        }
        camera::State state() const noexcept override { return state_; }
        const camera::FrameInfo& frame_info() const noexcept override { return info_; }
        std::expected<void, camera::CameraError>
            set_fixed_exposure_lines(std::uint32_t) noexcept override {
            return {};
        }
        std::uint32_t capacity() const noexcept override { return 4; }
        std::uint32_t outstanding() const noexcept override { return 0; }

    private:
        camera::State state_ = camera::State::kInitialized;
        camera::FrameInfo info_{320, 240};
    };

    static std::expected<std::unique_ptr<camera::ICamera>, camera::CameraError>
        create(const camera::Controls&) noexcept {
        ++creates_;
        if (mode_ == Failure::kCreate)
            return std::unexpected(failure(camera::Operation::kInitialize));
        return std::unique_ptr<camera::ICamera>{new Camera};
    }

    static std::string read(const std::filesystem::path& path) {
        std::ifstream input{path};
        return {std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
    }

    static void run_case(Failure mode, bool signal_stop = false, bool imu_on = true) {
        char directory[] = "/tmp/dart-service-isolation-XXXXXX";
        assert(::mkdtemp(directory));
        const auto root = std::filesystem::path{directory};
        mode_ = mode;
        creates_ = acquires_ = stops_ = destroyed_ = failure_reads_ = 0;
        app::Arguments args{};
        args.seconds = signal_stop ? 0 : 1;
        args.status_ms = 50;
        args.max_acquire_errors = 2;
        args.vision_on = false;
        args.record_on = mode == Failure::kRecorder;
        assert(app::copy_path(args.log_path, sizeof(args.log_path), (root / "app.log").string()));
        // A file cannot be a parent directory: recording fails without affecting capture.
        std::ofstream{root / "file"}.put('x');
        assert(
            app::copy_path(args.rec_dir, sizeof(args.rec_dir), (root / "file/records").string()));
        Source source;
        source_ = &source;
        imu::Session session{source};
        if (imu_on)
            assert(session.start((root / "imu.log").c_str()));
        const auto begin = std::chrono::steady_clock::now();
        {
            platform::RunControl control{args.seconds};
            std::thread stop;
            if (signal_stop) {
                stop = std::thread{[] {
                    std::this_thread::sleep_for(std::chrono::milliseconds{120});
                    ::kill(::getpid(), SIGTERM);
                }};
            }
            logging::Logger log;
            assert(log.open(args.log_path));
            assert(app::run_services(args, create, control, log, imu_on ? &session : nullptr) == 0);
            if (stop.joinable())
                stop.join();
        }
        const auto elapsed = std::chrono::steady_clock::now() - begin;
        assert(elapsed >= std::chrono::milliseconds{signal_stop ? 100 : 900});
        assert(elapsed < std::chrono::seconds{5});
        assert(creates_ == 1);                    // No process restart or tight probe retry loop.
        if (imu_on) {
            assert(source.reads.load() > failure_reads_ + 30);
            const auto before = source.reads.load();
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
            assert(source.reads.load() > before); // The runner never stops the independent IMU.
            session.stop();
            assert(session.stats().samples > 30 && session.stats().io_errors == 0);
            assert(read(session.log_path()).find("# session_end samples=") != std::string::npos);
        }
        const auto log = read(root / "app.log");
        assert(log.find(signal_stop ? "reason=signal" : "reason=deadline") != std::string::npos);
        if (mode == Failure::kCreate || mode == Failure::kStart || mode == Failure::kAcquire) {
            assert(
                log.find("action=disable_capture") != std::string::npos
                || log.find("result=disabled") != std::string::npos);
            assert(log.find("op=degraded camera=disabled") != std::string::npos);
            if (imu_on)
                assert(log.find("module=imu op=status samples=") != std::string::npos);
        }
        if (mode == Failure::kCreate) {
            assert(destroyed_ == 0 && acquires_ == 0);
        } else {
            assert(destroyed_ == 1);
        }
        if (mode == Failure::kAcquire)
            assert(acquires_ == 2 && stops_ == 1);
        if (mode == Failure::kRecorder) {
            assert(log.find("action=disable_recording") != std::string::npos);
            assert(acquires_ > 20 && stops_ == 1);
        }
        std::filesystem::remove_all(root);
    }

    static void silent_handler() {
        auto* output = std::tmpfile();
        assert(output);
        const int saved_out = ::dup(STDOUT_FILENO);
        const int saved_err = ::dup(STDERR_FILENO);
        assert(saved_out >= 0 && saved_err >= 0);
        assert(::dup2(::fileno(output), STDOUT_FILENO) >= 0);
        assert(::dup2(::fileno(output), STDERR_FILENO) >= 0);
        logging::Logger log;
        assert(!log.open("/dev/null/app.log"));
        camera::CallContext context{};
        context.operation = camera::Operation::kInitialize;
        app::log_camera_error(
            log, "create", "disable_capture",
            camera::CameraError{DART_CAMERA_E_SENSOR_PROBE_FAILED, context});
        assert(::lseek(::fileno(output), 0, SEEK_END) == 0);
        assert(::dup2(saved_out, STDOUT_FILENO) >= 0);
        assert(::dup2(saved_err, STDERR_FILENO) >= 0);
        ::close(saved_out);
        ::close(saved_err);
        std::fclose(output);
    }

    inline static Source* source_ = nullptr;
    inline static Failure mode_ = Failure::kNone;
    inline static unsigned creates_ = 0, acquires_ = 0, stops_ = 0, destroyed_ = 0,
                           failure_reads_ = 0;

public:
    static void run() {
        silent_handler();
        run_case(Failure::kCreate);
        run_case(Failure::kStart);
        run_case(Failure::kAcquire);
        run_case(Failure::kRecorder);
        run_case(Failure::kCreate, true);
        run_case(Failure::kCreate, true, false);
    }
};

} // namespace dart::tests

int main() { dart::tests::ServiceIsolationTest::run(); }
