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
#include "camera/c_api/camera.h"

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

enum class PipelineScene : std::uint8_t {
    kGrayLamp,   // RAW10 -> Y8 conversion, as the SC038HGS backend delivers.
    kColorLamp,  // RGB565 green lamp, as a color backend delivers.
    kColorWhite, // RGB565 white square: bright, but not a lamp color.
};

struct PipelineCase {
    PipelineScene scene = PipelineScene::kGrayLamp;
    bool vision = false;
    bool pwm = false;
    bool pwm_failure = false;
    bool record = true;
    recorder::ImageFormat format = recorder::ImageFormat::kPgm;
    std::uint32_t layout = DART_CAMERA_RAW10_LE16_MSB;
};

class BinaryPipelineTest final {
    static constexpr std::uint32_t kWidth = 320, kHeight = 240;

public:
    using Scene = PipelineScene;
    using Case = PipelineCase;

private:
    inline static Case case_;
    inline static unsigned pwm_starts_, pwm_stops_;
    inline static std::atomic<unsigned> pwm_writes_{0};
    inline static pwm::Pulses last_pwm_;

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
            if (case_.pwm_failure)
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
        explicit Camera(const Case& c) {
            if (c.scene == Scene::kGrayLamp) {
                std::vector<std::uint8_t> raw(kWidth * kHeight * 2);
                for (std::uint32_t y = 0; y < kHeight; ++y) {
                    for (std::uint32_t x = 0; x < kWidth; ++x) {
                        std::uint32_t sample = raw_sample(x, y);
                        if (c.layout == DART_CAMERA_RAW10_LE16_MSB)
                            sample <<= 6;
                        const auto offset = (y * kWidth + x) * 2;
                        raw[offset] = static_cast<std::uint8_t>(sample);
                        raw[offset + 1] = static_cast<std::uint8_t>(sample >> 8);
                    }
                }
                assert(
                    dart_camera_raw10_gray(
                        raw.data(), raw.size(), kWidth, kHeight, kWidth * 2, c.layout, 0,
                        reinterpret_cast<std::uint8_t*>(pixels_.data()), kWidth * kHeight)
                    == 0);
            } else {
                // Dark background, 8x8 foreground square; little-endian RGB565.
                const std::uint16_t lamp = c.scene == Scene::kColorLamp ? 0x2EED : 0xFFFF;
                for (std::uint32_t y = 0; y < kHeight; ++y)
                    for (std::uint32_t x = 0; x < kWidth; ++x) {
                        const std::uint16_t value = foreground(x, y) ? lamp : 0x10A3;
                        pixels_[(y * kWidth + x) * 2] = std::byte(value & 0xFFU);
                        pixels_[(y * kWidth + x) * 2 + 1] = std::byte(value >> 8U);
                    }
                info_.format = camera::PixelFormat::kRgb565;
            }
            const auto bytes = kWidth * camera::bytes_per_pixel(info_.format);
            for (auto& slot : slots_) {
                slot.view.pixels = std::span<const std::byte>{pixels_}.first(bytes * kHeight);
                slot.view.format = info_.format;
                slot.view.width = kWidth;
                slot.view.stride = bytes;
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
            if (sequence_ == 5 && case_.pwm && case_.vision && case_.scene != Scene::kGrayLamp
                && !case_.pwm_failure) {
                // The servo task runs independently of the capture loop.
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
        std::array<std::byte, kWidth * kHeight * 2> pixels_{};
        std::array<camera::FrameLeaseAccess::Slot, 4> slots_{};
    };

    static bool foreground(std::uint32_t x, std::uint32_t y) {
        return x >= 100 && x < 108 && y >= 100 && y < 108;
    }
    static std::uint32_t raw_sample(std::uint32_t x, std::uint32_t y) {
        return foreground(x, y) ? 800 : 160;
    }
    static std::expected<std::unique_ptr<camera::ICamera>, camera::CameraError>
        create(const void* context) noexcept {
        return std::unique_ptr<camera::ICamera>{new Camera{*static_cast<const Case*>(context)}};
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
    static void run(const Case& c) {
        case_ = c;
        char directory[] = "/tmp/dart-binary-pipeline-XXXXXX";
        assert(::mkdtemp(directory));
        const auto root = std::filesystem::path{directory};
        app::Arguments args{};
        args.seconds = 1;
        args.vision_on = c.vision;
        args.pwm_on = c.pwm;
        args.record_on = c.record;
        pwm_starts_ = pwm_stops_ = 0;
        pwm_writes_ = 0;
        for (std::size_t i = 0; i < args.servo.channels.size(); ++i) {
            auto& channel = args.servo.channels[i];
            channel.initial_pulse_us = 1510.0F + i * 10.0F;
            channel.horizontal_us_per_pixel = i % 2 == 0 ? 1.0F : -1.0F;
            channel.vertical_us_per_pixel = i < 2 ? 2.0F : -2.0F;
        }
        args.servo.max_delta_us = 100;
        args.record_format = c.format;
        args.threshold = c.format == recorder::ImageFormat::kPbm ? 0 : 50;
        assert(app::copy_path(args.log_path, sizeof(args.log_path), (root / "app.log").string()));
        assert(app::copy_path(args.rec_dir, sizeof(args.rec_dir), (root / "records").string()));
        assert(app::copy_path(args.det_csv, sizeof(args.det_csv), (root / "detect.csv").string()));
        platform::RunControl control{args.seconds};
        logging::Logger log_sink;
        assert(log_sink.open(args.log_path));
        assert((app::Application{args, platform::CameraFactory{create, &case_}, control, log_sink,
                                 nullptr, nullptr, create_pwm}.run() == 0));
        assert(log_sink.flush(std::chrono::milliseconds{5000}));

        const bool color = c.scene != Scene::kGrayLamp;
        const bool vision = c.vision && color;
        const bool target = vision && c.scene == Scene::kColorLamp;
        const auto log = read(root / "app.log");
        const auto observations = read(root / "detect.csv");
        assert(observations.starts_with("sequence,received_us,source_pts,kind,valid,"));
        assert(log.find("op=detector_prepare result=ok") != std::string::npos);
        assert((log.find("module=vision op=prepare result=unsupported_format") != std::string::npos)
               == (c.vision && !color));
        assert((log.find("module=recorder op=start result=unsupported_format") != std::string::npos)
               == (c.record && color));
        if (c.pwm_failure)
            assert(log.find("module=servo op=stop operation=fake_write") != std::string::npos);
        if (!c.pwm_failure && !(c.vision && !color) && !(c.record && color))
            assert(log.find("level=ERROR") == std::string::npos);

        // Servo output requires accepted vision; it maps the lamp centroid
        // (103.5, 103.5) relative to the automatic aim (160, 120).
        assert(pwm_starts_ == (c.pwm && vision ? 1U : 0U) && pwm_stops_ == pwm_starts_);
        if (target && !c.pwm_failure) {
            assert(pwm_writes_ >= 1 && observations.find(",light,1,") != std::string::npos);
            const pwm::Pulses expected{1420.5F, 1543.5F, 1506.5F, 1629.5F};
            for (std::size_t i = 0; i < expected.size(); ++i)
                assert(std::abs(last_pwm_[i] - expected[i]) < 0.01F);
        } else if (pwm_starts_ != 0) {
            assert((last_pwm_ == pwm::Pulses{1510, 1520, 1530, 1540}));
        }
        if (!target)
            assert(observations.find(",light,1,") == std::string::npos);
        if (!vision)
            assert(observations.find(",none,0,") != std::string::npos); // Frames without detection.

        if (!c.record || color) {
            assert(!std::filesystem::exists(root / "records/rec_0001"));
            std::filesystem::remove_all(root);
            return;
        }
        const bool pbm = c.format == recorder::ImageFormat::kPbm;
        const bool compact = c.format == recorder::ImageFormat::kDbv;
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
    using Test = dart::tests::BinaryPipelineTest;
    using Format = dart::recorder::ImageFormat;
    // Grayscale recording, byte-exact, with both RAW10 layouts.
    Test::run({Test::Scene::kGrayLamp, false, false, false, true, Format::kPbm, DART_CAMERA_RAW10_LE16_LSB});
    Test::run({Test::Scene::kGrayLamp, false, false, false, true, Format::kPgm});
    Test::run({Test::Scene::kGrayLamp, false, false, false, true, Format::kDbv});
    // Vision requested on a Y8 camera: reported and disabled, recording continues.
    Test::run({Test::Scene::kGrayLamp, true, true, false, true, Format::kPgm});
    // Color vision -> servo mapping -> PWM task.
    Test::run({Test::Scene::kColorLamp, true, true, false, false});
    Test::run({Test::Scene::kColorLamp, true, true, true, false});
    Test::run({Test::Scene::kColorWhite, true, true, false, false});
    // Recording requested on a color camera: reported and disabled, vision continues.
    Test::run({Test::Scene::kColorLamp, true, true, false, true});
}
