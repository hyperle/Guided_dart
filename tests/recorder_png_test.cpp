#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <png.h>
#include <thread>
#include <vector>

#include "recorder/recorder.cpp"

namespace dart::tests {

static std::atomic<bool> hold_png_sync{false}, png_sync_blocked{false};
extern "C" int __real_fsync(int);
extern "C" int __wrap_fsync(int fd) {
    if (hold_png_sync.load()) {
        png_sync_blocked.store(true);
        while (hold_png_sync.load())
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    return __real_fsync(fd);
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
        config.queue_slots = 16;
        config.image_format = recorder::ImageFormat::kPng;
        config.output_directory = directory;
        config.max_file_bytes = first_size * 2 + second_size - 1;
        std::vector<recorder::FrameEvent> events;
        config.event_sink = event_sink;
        config.event_sink_context = &events;
        recorder::Recorder rec{config};
        assert(rec.start());
        camera::FrameLeaseAccess::Slot slot;
        slot.release = release;
        slot.view.width = slot.view.stride = 640;
        slot.view.height = 480;
        unsigned submitted = 0;
        unsigned tick = 0;
        hold_png_sync.store(true);
        png_sync_blocked.store(false);
        for (unsigned i = 0; i < 12; ++i) {
            slot.view.luma =
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
                    break;
                }
                assert(admission == recorder::Admission::kBusy);
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
            if (i == 0) {
                for (unsigned wait = 0; wait < 1000 && !png_sync_blocked.load(); ++wait)
                    std::this_thread::sleep_for(std::chrono::milliseconds{1});
                assert(png_sync_blocked.load());
            }
        }
        assert(submitted == 12);
        hold_png_sync.store(false);
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
        recorder::Recorder next{config};
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
        recorder::ImageWriter writer{config, session};
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

public:
    static void run() {
        encoder();
        limit_and_queue();
        grouping_and_cleanup();
    }
};

} // namespace dart::tests

int main() { dart::tests::RecorderPngTest::run(); }
