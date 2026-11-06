#include <expected>
#include <span>
#include <cmath>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "camera/c_api/raw10.h"

#include "runtime/application.hpp"
#include "recorder/encoding/binary_decoder.hpp"

#include "runtime/argument_parser.hpp"
#include "camera/interface.hpp"
#include "logging/logger.hpp"
#include "pwm/interface.hpp"
#include "recorder/encoding/binary_format.hpp"

#include "system/run_control.hpp"
#include "system/write_control.hpp"
#include "system/device_services.hpp"

namespace dart::tests {

class BinaryPipelineTest final {
    static constexpr std::uint32_t kWidth = 320, kHeight = 240;
    inline static bool low_contrast_;
    inline static camera::Raw10Layout layout_;
    inline static unsigned pwm_starts_, pwm_stops_;
    inline static std::atomic<unsigned> pwm_writes_{0};
    inline static pwm::Pulses last_pwm_;
    inline static bool pwm_failure_;

    class Pwm final : public pwm::IPwm {
    public:
        std::expected<void, pwm::Error> start(const pwm::Pulses& initial) noexcept override {
            ++pwm_starts_;
            assert((initial == pwm::Pulses{1510, 1520, 1530, 1540}));
            last_pwm_ = initial;
            return {};
        }
        std::expected<void, pwm::Error> write(const pwm::Pulses& pulses) noexcept override {
            ++pwm_writes_;
            if (pwm_failure_)
                return std::unexpected(pwm::Error{"fake_write", 2, -7});
            last_pwm_ = pulses;
            return {};
        }
        std::expected<void, pwm::Error> stop() noexcept override {
            ++pwm_stops_;
            return {};
        }
    };

    static std::expected<std::unique_ptr<pwm::IPwm>, pwm::Error> create_pwm() noexcept {
        return std::unique_ptr<pwm::IPwm>{new Pwm};
    }

    class Camera final : public camera::ICamera {
    public:
        explicit Camera(const camera::Controls& controls) {
            assert(controls.raw10_layout == layout_);
            std::vector<std::uint8_t> raw(kWidth * kHeight * 2);
            for (std::uint32_t y = 0; y < kHeight; ++y) {
                for (std::uint32_t x = 0; x < kWidth; ++x) {
                    std::uint32_t sample = raw_sample(x, y);
                    if (layout_ == camera::Raw10Layout::kLe16Msb)
                        sample <<= 6;
                    const auto offset = (y * kWidth + x) * 2;
                    raw[offset] = static_cast<std::uint8_t>(sample);
                    raw[offset + 1] = static_cast<std::uint8_t>(sample >> 8);
                }
            }
            assert(
                dart_camera_raw10_gray(
                    raw.data(), raw.size(), kWidth, kHeight, kWidth * 2,
                    static_cast<std::uint32_t>(layout_), 0,
                    reinterpret_cast<std::uint8_t*>(pixels_.data()), pixels_.size())
                == 0);
            for (auto& slot : slots_) {
                slot.view.luma = pixels_;
                slot.view.width = slot.view.stride = kWidth;
                slot.view.height = kHeight;
                slot.release = release;
                slot.owner = this;
            }
        }
        ~Camera() override { assert(std::this_thread::get_id() == owner_); }
        std::expected<void, camera::CameraError> start() noexcept override {
            assert(std::this_thread::get_id() == owner_);
            state_ = camera::State::kStarted;
            return {};
        }
        std::expected<camera::AcquireResult, camera::CameraError>
            acquire(std::chrono::milliseconds) noexcept override {
            assert(std::this_thread::get_id() == owner_);
            if (sequence_ == 5 && !pwm_failure_) {
                const auto before = pwm_writes_.load();
                std::this_thread::sleep_for(std::chrono::milliseconds{60});
                assert(pwm_writes_.load() > before);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds{20});
            }
            camera::AcquireResult result{};
            for (auto& slot : slots_) {
                if (slot.state != camera::FrameLeaseAccess::SlotState::kFree)
                    continue;
                slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
                slot.view.sequence = ++sequence_;
                slot.view.received = std::chrono::microseconds{sequence_ * 20'000};
                result.status = camera::AcquireStatus::kFrame;
                result.lease = camera::FrameLeaseAccess::make(&slot);
                return result;
            }
            result.status = camera::AcquireStatus::kLeaseExhausted;
            return result;
        }
        std::expected<void, camera::CameraError> stop() noexcept override {
            assert(std::this_thread::get_id() == owner_);
            assert(outstanding() == 0);
            state_ = camera::State::kInitialized;
            return {};
        }
        camera::State state() const noexcept override { return state_; }
        const camera::FrameInfo& frame_info() const noexcept override { return info_; }
        std::expected<void, camera::CameraError>
            set_fixed_exposure_lines(std::uint32_t) noexcept override {
            return {};
        }
        std::uint32_t capacity() const noexcept override { return slots_.size(); }
        std::uint32_t outstanding() const noexcept override {
            std::uint32_t count = 0;
            for (const auto& slot : slots_)
                count += slot.state != camera::FrameLeaseAccess::SlotState::kFree;
            return count;
        }

    private:
        static std::expected<void, camera::CameraError>
            release(camera::FrameLeaseAccess::Slot* slot) noexcept {
            assert(std::this_thread::get_id() == static_cast<Camera*>(slot->owner)->owner_);
            slot->state = camera::FrameLeaseAccess::SlotState::kFree;
            return {};
        }
        const std::thread::id owner_ = std::this_thread::get_id();
        camera::State state_ = camera::State::kInitialized;
        camera::FrameInfo info_{kWidth, kHeight};
        std::uint64_t sequence_ = 0;
        std::array<std::byte, kWidth * kHeight> pixels_{};
        std::array<camera::FrameLeaseAccess::Slot, 4> slots_{};
    };

    static bool foreground(std::uint32_t x, std::uint32_t y) {
        return x >= 100 && x < 108 && y >= 100 && y < 108;
    }
    static std::uint32_t raw_sample(std::uint32_t x, std::uint32_t y) {
        return foreground(x, y) ? (low_contrast_ ? 200 : 800) : (low_contrast_ ? 199 : 160);
    }
    static std::expected<std::unique_ptr<camera::ICamera>, camera::CameraError>
        create(const camera::Controls& controls) noexcept {
        return std::unique_ptr<camera::ICamera>{new Camera{controls}};
    }
    static std::string read(const std::filesystem::path& path) {
        std::ifstream input{path, std::ios::binary};
        assert(input);
        return {std::istreambuf_iterator<char>{input}, {}};
    }

    static void check_binary(const std::string& image) {
        const auto data = std::span{
            reinterpret_cast<const std::byte*>(image.data()), image.size()};
        std::uint32_t width = 0, height = 0;
        assert(data.size() >= recorder::BinaryFormat::kHeaderBytes);
        assert(recorder::BinaryFormat::geometry(data.first(12), width, height));
        assert(width == kWidth && height == kHeight);
        recorder::BinaryDecoder decoder;
        assert(decoder.prepare(width, height));
        std::size_t offset = recorder::BinaryFormat::kHeaderBytes;
        unsigned frames = 0;
        while (offset < data.size()) {
            std::uint32_t code = 0;
            const auto prefix = recorder::BinaryFormat::parse(data.subspan(offset), code);
            assert(prefix > 0);
            offset += prefix;
            assert((code >> 2U) <= data.size() - offset);
            assert(decoder.decode(code, data.subspan(offset, code >> 2U)));
            offset += code >> 2U;
            const auto bits = decoder.bits();
            for (std::size_t i = 0; i < kWidth * kHeight; ++i) {
                const bool white = (std::to_integer<unsigned>(bits[i / 8]) & (128U >> (i % 8))) != 0;
                assert(white == foreground(i % kWidth, i / kWidth));
            }
            ++frames;
        }
        assert(frames > 0 && offset == data.size());
    }

public:
    static void run(bool pbm, bool pwm_failure = false, bool compact = false,
                    bool low_contrast = false, std::uint32_t threshold = 50) {
        low_contrast_ = low_contrast;
        char directory[] = "/tmp/dart-binary-pipeline-XXXXXX";
        assert(::mkdtemp(directory));
        const auto root = std::filesystem::path{directory};
        app::Arguments args{};
        args.seconds = 1;
        // Exercise camera -> RAW conversion -> detector -> mapping -> PWM task,
        // with recording active and no IMU session.
        pwm_starts_ = pwm_stops_ = 0;
        pwm_writes_ = 0;
        pwm_failure_ = pwm_failure;
        for (std::size_t i = 0; i < args.servo.channels.size(); ++i) {
            auto& channel = args.servo.channels[i];
            channel.initial_pulse_us = 1510.0F + i * 10.0F;
            channel.horizontal_us_per_pixel = i % 2 == 0 ? 1.0F : -1.0F;
            channel.vertical_us_per_pixel = i < 2 ? 2.0F : -2.0F;
        }
        args.servo.max_delta_us = 100;
        if (compact)
            args.record_format = recorder::ImageFormat::kDbv;
        else
            args.record_format = pbm ? recorder::ImageFormat::kPbm : recorder::ImageFormat::kPgm;
        args.threshold = pbm ? 0 : threshold;
        args.raw10_layout = layout_ =
            pbm ? camera::Raw10Layout::kLe16Lsb : camera::Raw10Layout::kLe16Msb;
        assert(app::copy_path(args.log_path, sizeof(args.log_path), (root / "app.log").string()));
        assert(app::copy_path(args.rec_dir, sizeof(args.rec_dir), (root / "records").string()));
        assert(app::copy_path(args.det_csv, sizeof(args.det_csv), (root / "detect.csv").string()));
        platform::RunControl control{args.seconds};
        logging::Logger log_sink;
        assert(log_sink.open(args.log_path));
        assert((app::Application{args, create, control, log_sink, nullptr, nullptr, create_pwm}.run() == 0));
        assert(log_sink.flush(std::chrono::milliseconds{5000}));
        assert(pwm_starts_ == 1 && pwm_stops_ == 1);
        if (pbm || pwm_failure || low_contrast || threshold > 200) // No accepted target, or PWM has failed.
            assert((last_pwm_ == pwm::Pulses{1510, 1520, 1530, 1540}));
        else {
            assert(pwm_writes_ >= 1);
            // Tracker retains the sampled acquisition centroid (103,103)
            // inside its deadband. Automatic aim is (160,120).
            const pwm::Pulses expected{1419, 1543, 1507, 1631};
            for (std::size_t i = 0; i < expected.size(); ++i)
                assert(std::abs(last_pwm_[i] - expected[i]) < 0.01F);
        }
        const auto observations = read(root / "detect.csv");
        assert(observations.starts_with("sequence,received_us,source_pts,kind,valid,"));
        if (!(pbm || low_contrast || threshold > 200))
            assert(observations.find(",light,1,") != std::string::npos);
        const auto log = read(root / "app.log");
        assert(log.find("op=detector_prepare result=ok") != std::string::npos);
        if (pwm_failure)
            assert(log.find("module=servo op=stop operation=fake_write") != std::string::npos);
        else
            assert(log.find("level=ERROR") == std::string::npos);
        const auto image =
            read(root / "records/rec_0001/img" / (compact ? "recording.dbv"
                                                       : pbm ? "recording.pbm" : "recording.pgm"));
        if (compact) {
            check_binary(image);
            std::filesystem::remove_all(root);
            return;
        }
        const std::string header = pbm ? "P4\n320 240\n" : "P5\n320 240\n255\n";
        const std::size_t payload = pbm ? kWidth * kHeight / 8 : kWidth * kHeight;
        const auto record = header.size() + payload;
        assert(image.size() >= record && image.size() % record == 0);
        for (std::size_t offset = 0; offset < image.size(); offset += record) {
            assert(image.substr(offset, header.size()) == header);
            for (std::size_t i = 0; i < payload; ++i) {
                const auto expected = pbm ? 0 : raw_sample(i % kWidth, i / kWidth) >> 2;
                assert(static_cast<unsigned char>(image[offset + header.size() + i]) == expected);
            }
        }
        std::filesystem::remove_all(root);
    }
};

} // namespace dart::tests

int main() {
    dart::tests::BinaryPipelineTest::run(true);
    dart::tests::BinaryPipelineTest::run(false);
    dart::tests::BinaryPipelineTest::run(false, true);
    dart::tests::BinaryPipelineTest::run(false, false, true);
    dart::tests::BinaryPipelineTest::run(false, false, false, true);
    dart::tests::BinaryPipelineTest::run(false, false, false, false, 201);
}
