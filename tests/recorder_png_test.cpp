#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <png.h>
#include <thread>
#include <vector>

#include "tests/recorder_fixture.hpp"
#include "src/recorder/storage.hpp"

namespace dart::tests {

extern "C" int __wrap_fsync(int) {
    assert(false && "PNG recording must not force device synchronization");
    return -1;
}

class RecorderPngTest final {
    static void decode(
        std::span<const std::byte> png, std::span<const std::byte> pixels, unsigned width,
        unsigned height) {
        // Independent libpng decoder checks the container, DEFLATE, CRCs,
        // 8-bit grayscale values, row padding and every pixel.
        png_image image{};
        image.version = PNG_IMAGE_VERSION;
        assert(png_image_begin_read_from_memory(&image, png.data(), png.size()));
        assert(image.width == width && image.height == height);
        image.format = PNG_FORMAT_GRAY;
        std::vector<unsigned char> decoded(PNG_IMAGE_SIZE(image));
        assert(png_image_finish_read(&image, nullptr, decoded.data(), 0, nullptr));
        for (std::size_t i = 0; i < pixels.size(); ++i)
            assert(decoded[i] == std::to_integer<unsigned>(pixels[i]));
        png_image_free(&image);
    }
    static std::vector<std::byte> read(const std::filesystem::path& path) {
        std::ifstream in(path, std::ios::binary);
        assert(in);
        const std::vector<char> data{std::istreambuf_iterator<char>{in}, {}};
        std::vector<std::byte> bytes(data.size());
        std::memcpy(bytes.data(), data.data(), data.size());
        return bytes;
    }
    static void encoder() {
        for (const unsigned width : {1U, 7U, 8U, 9U, 17U, 640U}) {
            recorder::PngEncoder png;
            assert(png.prepare(width, 7));
            std::vector<std::byte> pixels(width * 7);
            for (unsigned frame = 0; frame < 3; ++frame) {
                for (std::size_t i = 0; i < pixels.size(); ++i)
                    pixels[i] = std::byte((i * 73 + frame * 127) & 255);
                decode(png.encode(pixels), pixels, width, 7);
            }
        }
        recorder::PngEncoder png;
        assert(png.prepare(640, 480));
        std::vector<std::byte> pixels(640 * 480);
        for (unsigned pattern = 0; pattern < 3; ++pattern) {
            std::uint32_t random = 0x12345678;
            for (unsigned y = 0; y < 480; ++y) {
                for (unsigned x = 0; x < 640; ++x) {
                    random ^= random << 13;
                    random ^= random >> 17;
                    random ^= random << 5;
                    const bool white = pattern == 1 ? (x > 100 && x < 160 && y > 100 && y < 200)
                                                    : pattern == 2 && (random & 1);
                    pixels[y * 640 + x] = white ? std::byte{255} : std::byte{0};
                }
            }
            const auto begin = std::chrono::steady_clock::now();
            std::span<const std::byte> encoded;
            for (unsigned i = 0; i < 100; ++i)
                encoded = png.encode(pixels);
            const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - begin)
                                .count()
                          / 100;
            assert(!encoded.empty() && encoded.size() < pixels.size());
            decode(encoded, pixels, 640, 480);
            std::printf(
                "PNG pattern=%u bytes=%zu encode_us=%lld\n", pattern, encoded.size(),
                static_cast<long long>(us));
        }
    }
    static std::expected<void, camera::CameraError>
        release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }
    static void event_sink(const recorder::FrameEvent& event, void* context) noexcept {
        static_cast<std::vector<recorder::FrameEvent>*>(context)->push_back(event);
    }
    static void limit_and_queue() {
        char directory[] = "/tmp/dart-png-limit-XXXXXX";
        assert(::mkdtemp(directory));
        const std::filesystem::path root{directory};
        std::vector<std::byte> black(640 * 480), patterned(black.size());
        for (std::size_t i = 0; i < patterned.size(); ++i)
            patterned[i] = ((i * 71U + i / 640) % 251) < 128 ? std::byte{255} : std::byte{0};
        recorder::PngEncoder encoder;
        assert(encoder.prepare(640, 480));
        const auto first_size = encoder.encode(black).size();
        const auto second_size = encoder.encode(patterned).size();
        assert(first_size != second_size);
        recorder::Config config;
        config.width = 640;
        config.height = 480;

        config.frame_timeout = std::chrono::milliseconds{5000};
        config.image_format = recorder::ImageFormat::kPng;
        config.output_directory = directory;
        config.max_file_bytes = first_size * 2 + second_size - 1;
        std::vector<recorder::FrameEvent> events;
        config.event_sink = event_sink;
        config.event_sink_context = &events;
        RecorderFixture rec{config};
        assert(rec.start());
        camera::FrameLeaseAccess::Slot slot;
        slot.release = release;
        slot.view.width = slot.view.stride = 640;
        slot.view.height = 480;
        unsigned submitted = 0;
        unsigned tick = 0;
        for (unsigned i = 0; i < 3; ++i) {
            slot.view.pixels =
                i == 1 ? std::span<const std::byte>{patterned} : std::span<const std::byte>{black};
            for (unsigned attempt = 0;; ++attempt) {
                assert(attempt < 1000);
                slot.view.sequence = ++tick;
                slot.view.received = std::chrono::microseconds{20'000U * tick};
                slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
                auto lease = camera::FrameLeaseAccess::make(&slot);
                const auto admission = rec.try_submit(std::move(lease));
                (void)lease.release();
                while (rec.pending_releases())
                    (void)rec.drain_releases();
                if (admission == recorder::Admission::kAccepted) {
                    ++submitted;
                    rec.wait_released();
                    break;
                }
                assert(admission == recorder::Admission::kBusy);
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
        }
        assert(submitted == 3);
        for (unsigned i = 0; i < 1000 && !rec.limit_reached(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(submitted >= 2 && rec.limit_reached());
        rec.stop();
        assert(rec.last_error() == recorder::Error::kNone);
        const auto stats = rec.stats();
        assert(stats.written == 2 && stats.write_fail == 0 && stats.queued == 0);
        assert(stats.bytes == first_size + second_size && stats.bytes <= config.max_file_bytes);
        unsigned accepted = 0, files = 0;
        for (const auto& event : events) {
            if (event.decision != recorder::EventDecision::kAccepted) {
                assert(event.image_name[0] == 0 && event.file_bytes == 0);
                continue;
            }
            ++accepted;
            assert(event.file_offset == 0);
            const auto path = std::filesystem::path{rec.directory()} / event.image_name.data();
            assert(std::filesystem::file_size(path) == event.file_bytes);
            decode(read(path), event.image_index == 1 ? black : patterned, 640, 480);
        }
        for (const auto& file : std::filesystem::recursive_directory_iterator(rec.directory())) {
            assert(file.path().extension() != ".tmp");
            if (file.path().extension() == ".png")
                ++files;
        }
        assert(files == 2 && accepted == 2);
        // Reset creates a new session and leaves previous frames intact.
        RecorderFixture next{config};
        assert(next.start());
        next.stop();
        assert(std::filesystem::path{next.directory()}.filename() == "rec_0002");
        std::filesystem::remove_all(root);
    }
    static void grouping_and_cleanup() {
        char directory[] = "/tmp/dart-png-groups-XXXXXX";
        assert(::mkdtemp(directory));
        recorder::Config config;
        config.width = 9;
        config.height = 1;
        config.image_format = recorder::ImageFormat::kPng;
        config.output_directory = directory;
        config.keep_sessions = 1;
        recorder::RecorderError error;
        recorder::Session session{config, error};
        assert(session.open());
        ImageWriterFixture writer{config, session};
        assert(writer.prepare(9) && writer.open(error));
        const std::array<std::byte, 9> pixels{};
        assert(writer.write_image(pixels, 32));
        assert(std::string_view{writer.image_name()} == "img/b0000001/p0000001/00000032.png");
        assert(writer.write_image(pixels, 33));
        assert(std::string_view{writer.image_name()} == "img/b0000001/p0000002/00000033.png");
        assert(writer.write_image(pixels, 1024));
        assert(std::string_view{writer.image_name()} == "img/b0000001/p0000032/00001024.png");
        assert(writer.write_image(pixels, 1025));
        assert(std::string_view{writer.image_name()} == "img/b0000002/p0000001/00001025.png");
        assert(writer.close());
        const auto unknown =
            std::filesystem::path{session.directory()} / "img/b0000001/p0000002/notes.txt";
        std::ofstream{unknown} << "must preserve";
        recorder::Session refused{config, error};
        assert(!refused.open());
        assert(
            std::filesystem::exists(
                std::filesystem::path{session.directory()} / "img/b0000001/p0000001/00000032.png"));
        std::filesystem::remove(unknown);
        recorder::Session next{config, error};
        assert(next.open());
        assert(!std::filesystem::exists(std::filesystem::path{next.directory()} / "img/b0000001"));
        std::filesystem::remove_all(directory);
    }

    static void png_stream() {
        char directory[] = "/tmp/dart-png-stream-XXXXXX";
        assert(::mkdtemp(directory));
        const std::filesystem::path root{directory};
        std::vector<std::byte> first(640U * 480U, std::byte{0});
        std::vector<std::byte> second(first.size());
        for (std::size_t i = 0; i < second.size(); ++i)
            second[i] = std::byte{static_cast<unsigned char>((i * 37U + i / 640U) & 255U)};

        recorder::Config config;
        config.width = 640;
        config.height = 480;
        config.fps = 30;

        config.frame_timeout = std::chrono::milliseconds{5000};
        config.image_format = recorder::ImageFormat::kPngStream;
        config.output_directory = directory;
        std::vector<recorder::FrameEvent> events;
        config.event_sink = event_sink;
        config.event_sink_context = &events;
        RecorderFixture rec{config};
        assert(rec.start());

        camera::FrameLeaseAccess::Slot slot;
        slot.release = release;
        slot.view.width = slot.view.stride = 640;
        slot.view.height = 480;
        unsigned tick = 0;
        for (unsigned i = 0; i < 2; ++i) {
            const auto& pixels = i == 0 ? first : second;
            slot.view.pixels = std::span<const std::byte>{pixels};
            for (unsigned attempt = 0;; ++attempt) {
                assert(attempt < 1000);
                slot.view.sequence = ++tick;
                slot.view.received = std::chrono::microseconds{40'000U * tick};
                slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
                auto lease = camera::FrameLeaseAccess::make(&slot);
                const auto admission = rec.try_submit(std::move(lease));
                (void)lease.release();
                while (rec.pending_releases())
                    (void)rec.drain_releases();
                if (admission == recorder::Admission::kAccepted) {
                    rec.wait_released();
                    break;
                }
                assert(admission == recorder::Admission::kBusy);
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
        }
        for (unsigned i = 0; i < 1000 && rec.stats().written < 2; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(rec.stats().written == 2);
        rec.stop();
        assert(rec.last_error() == recorder::Error::kNone);
        assert(rec.stats().write_fail == 0 && rec.stats().queued == 0);

        const auto session_root = std::filesystem::path{rec.directory()};
        const auto container = session_root / "img/recording.pngs";
        const auto bytes = read(container);
        assert(bytes.size() >= recorder::ImageWriter::kPngStreamHeaderBytes);
        assert(std::memcmp(bytes.data(), "DPNG", 4) == 0);
        const auto u16 = [&](std::size_t offset) {
            return static_cast<unsigned>(std::to_integer<unsigned>(bytes[offset]))
                | (static_cast<unsigned>(std::to_integer<unsigned>(bytes[offset + 1])) << 8U);
        };
        const auto u32 = [&](std::size_t offset) {
            return (static_cast<std::uint32_t>(u16(offset)))
                | (static_cast<std::uint32_t>(u16(offset + 2)) << 16U);
        };
        assert(u16(4) == 1 && u16(6) == 16);
        assert(u32(8) == 640 && u32(12) == 480);

        std::uint64_t expected_offset = recorder::ImageWriter::kPngStreamHeaderBytes;
        unsigned accepted = 0, written = 0;
        for (const auto& event : events) {
            if (event.decision != recorder::EventDecision::kAccepted)
                continue;
            ++accepted;
            assert(std::string_view{event.image_name.data()} == "img/recording.pngs");
            assert(event.file_offset == expected_offset);
            assert(event.file_offset + event.file_bytes <= bytes.size());
            const auto* data = bytes.data() + static_cast<std::size_t>(event.file_offset);
            decode(
                std::span<const std::byte>{data, static_cast<std::size_t>(event.file_bytes)},
                event.image_index == 1 ? first : second, 640, 480);
            expected_offset += event.file_bytes;
            ++written;
        }
        assert(accepted == 2 && written == 2);
        assert(expected_offset == bytes.size());
        assert(rec.stats().bytes == bytes.size());

        // A later session with a two-session retention window must recognize and
        // remove the older container file, not fail on an unknown image name.
        RecorderFixture next_session{config};
        assert(next_session.start());
        next_session.stop();
        recorder::Config cleanup = config;
        cleanup.keep_sessions = 2;
        RecorderFixture next{cleanup};
        assert(next.start());
        next.stop();
        assert(!std::filesystem::exists(root / "rec_0001"));
        assert(std::filesystem::exists(root / "rec_0002"));
        assert(std::filesystem::exists(root / "rec_0003"));
        std::filesystem::remove_all(root);
    }

    static void png_stream_budget() {
        char directory[] = "/tmp/dart-png-stream-budget-XXXXXX";
        assert(::mkdtemp(directory));
        const std::array<std::byte, 6> pixels{
            std::byte{0}, std::byte{1}, std::byte{127},
            std::byte{128}, std::byte{254}, std::byte{255}};
        recorder::PngEncoder encoder;
        assert(encoder.prepare(3, 2));
        const auto png_bytes = encoder.encode(pixels).size();
        recorder::Config config;
        config.width = 3;
        config.height = 2;
        config.image_format = recorder::ImageFormat::kPngStream;
        config.output_directory = directory;
        for (const bool exact : {false, true}) {
            config.max_file_bytes = 16 + png_bytes - (exact ? 0 : 1);
            recorder::RecorderError error;
            recorder::Session session{config, error};
            assert(session.open());
            ImageWriterFixture writer{config, session};
            assert(writer.prepare(pixels.size()) && writer.open(error));
            assert(writer.write_image(pixels) == exact);
            if (exact) {
                assert(writer.file_offset() == 16 && writer.record_bytes() == png_bytes);
                assert(writer.bytes_written() == config.max_file_bytes);
                assert(!writer.write_image(pixels) && errno == EFBIG);
            } else {
                assert(errno == EFBIG);
                assert(writer.bytes_written() == 16);
            }
            assert(writer.close());
            const auto bytes = read(session.image_path());
            assert(bytes.size() == (exact ? config.max_file_bytes : 16));
            if (exact)
                decode(std::span<const std::byte>{bytes}.subspan(16), pixels, 3, 2);
        }
        std::filesystem::remove_all(directory);
    }

public:
    static void run() {
        encoder();
        limit_and_queue();
        grouping_and_cleanup();
        png_stream();
        png_stream_budget();
    }
};

} // namespace dart::tests

int main() { dart::tests::RecorderPngTest::run(); }
