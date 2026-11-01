#include <cstdint>
#include <cstddef>
#include <expected>
#include <span>
#include <utility>
#include <string_view>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <thread>
#include <vector>

#include "recorder/encoding/binary_decoder.hpp"
#include "recorder/recorder.hpp"

#include "recorder/encoding/binary_encoder.hpp"
#include "recorder/encoding/binary_format.hpp"
#include "recorder/encoding/png_encoder.hpp"
#include "recorder/storage/session.hpp"

#include "recorder/storage/image_writer.hpp"

namespace dart::tests {

class RecorderBinaryTest final {
    static void check_pixels(
        const recorder::BinaryDecoder& decoder, const std::vector<std::byte>& pixels,
        unsigned width, unsigned height, unsigned threshold) {
        const auto bits = decoder.bits();
        const auto stride = (width + 7U) / 8U;
        for (unsigned y = 0; y < height; ++y) {
            for (unsigned x = 0; x < width; ++x) {
                const bool decoded = (std::to_integer<unsigned>(bits[y * stride + x / 8U])
                                      & (128U >> (x % 8U))) != 0;
                assert(decoded == (std::to_integer<unsigned>(pixels[y * width + x]) >= threshold));
            }
            if (width % 8U)
                assert((std::to_integer<unsigned>(bits[(y + 1) * stride - 1])
                        & ((1U << (8U - width % 8U)) - 1U)) == 0);
        }
    }
    static void roundtrip() {
        for (const unsigned width : {1U, 7U, 8U, 9U, 17U, 640U}) {
            for (const unsigned threshold : {0U, 1U, 128U, 255U}) {
                recorder::BinaryEncoder encoder;
                recorder::BinaryDecoder decoder;
                assert(encoder.prepare(width, 7, threshold) && decoder.prepare(width, 7));
                std::vector<std::byte> pixels(width * 7);
                for (unsigned frame = 0; frame < 8; ++frame) {
                    if (frame % 2 == 0)
                        for (std::size_t i = 0; i < pixels.size(); ++i)
                            pixels[i] = std::byte((i * 73 + frame * 127) & 255);
                    const auto record = encoder.encode(pixels);
                    std::uint32_t code = 0;
                    const auto prefix = recorder::BinaryFormat::parse(record, code);
                    assert(prefix > 0 && record.size() == (code >> 2U) + prefix);
                    assert(code != 0 && (code >> 2U) > 0); // Identical inputs also get image data.
                    assert(decoder.decode(code, record.subspan(prefix)));
                    check_pixels(decoder, pixels, width, 7, threshold);
                }
                encoder.reset();
                const auto record = encoder.encode(pixels);
                assert((std::to_integer<unsigned>(record[0]) & 3U) != 0);
            }
        }
    }
    static void malformed() {
        recorder::BinaryDecoder decoder;
        assert(!decoder.prepare(0, 1) && !decoder.prepare(1, 0));
        assert(!decoder.prepare(16385, 1));
        assert(decoder.prepare(640, 480));
        assert(!decoder.decode(0, {})); // First frame cannot reference a predecessor.
        std::uint32_t value = 0;
        assert(recorder::BinaryFormat::parse({}, value) == 0);
        const std::array<std::byte, 5> overflow{
            std::byte{255}, std::byte{255}, std::byte{255}, std::byte{255}, std::byte{16}};
        assert(recorder::BinaryFormat::parse(overflow, value) == -1);
        const std::array<std::byte, 2> noncanonical{std::byte{128}, std::byte{0}};
        assert(recorder::BinaryFormat::parse(noncanonical, value) == -1);
        for (const std::uint32_t code : {0U, 1U, 127U, 128U, 16383U, 16384U, 0xffffffffU}) {
            std::byte bytes[5];
            const auto size = recorder::BinaryFormat::prefix(bytes, code);
            assert(recorder::BinaryFormat::parse({bytes, size}, value) == static_cast<int>(size));
            assert(value == code);
        }
        recorder::BinaryEncoder encoder;
        assert(encoder.prepare(640, 480, 128));
        std::vector<std::byte> pixels(640 * 480);
        const auto encoded = encoder.encode(pixels);
        std::vector<std::byte> corrupt(encoded.begin(), encoded.end());
        const auto prefix = recorder::BinaryFormat::parse(corrupt, value);
        assert(prefix > 0 && (value & 3U) == recorder::BinaryFormat::kFull);
        corrupt.back() ^= std::byte{1}; // zlib's existing checksum detects damage.
        assert(!decoder.decode(value, std::span{corrupt}.subspan(prefix)));
        assert(decoder.decode(value, encoded.subspan(prefix)));
        assert(!decoder.decode(value, encoded.subspan(prefix + 1)));
        pixels.back() = std::byte{255};
        const auto sparse = encoder.encode(pixels);
        const auto sparse_prefix = recorder::BinaryFormat::parse(sparse, value);
        assert(sparse_prefix > 0 && (value & 3U) == recorder::BinaryFormat::kSparse);
        assert(sparse.size() <= 5 && decoder.decode(value, sparse.subspan(sparse_prefix)));
        check_pixels(decoder, pixels, 640, 480, 128);
        const std::array<std::byte, 3> outside{std::byte{255}, std::byte{127}, std::byte{1}};
        recorder::BinaryDecoder small;
        assert(small.prepare(17, 9));
        const std::vector<std::byte> raw(27);
        assert(small.decode((27U << 2U) | recorder::BinaryFormat::kRaw, raw));
        assert(!small.decode(3U << 2U, outside));
    }
    static void compression() {
        recorder::BinaryEncoder encoder;
        recorder::BinaryDecoder decoder;
        recorder::PngEncoder png;
        assert(encoder.prepare(640, 480, 128) && decoder.prepare(640, 480));
        assert(png.prepare(640, 480, 128));
        std::vector<std::byte> background(640 * 480), pixels;
        std::uint32_t random = 0x12345678;
        for (auto& pixel : background) {
            random ^= random << 13;
            random ^= random >> 17;
            random ^= random << 5;
            pixel = random & 1 ? std::byte{255} : std::byte{0};
        }
        std::size_t binary_bytes = recorder::BinaryFormat::kHeaderBytes, png_bytes = 0;
        unsigned kinds[4]{};
        std::chrono::microseconds binary_time{};
        for (unsigned frame = 0; frame < 120; ++frame) {
            pixels = background;
            for (unsigned y = 200; y < 210; ++y)
                for (unsigned x = frame; x < frame + 10; ++x)
                    pixels[y * 640 + x] = std::byte{255};
            const auto start = std::chrono::steady_clock::now();
            const auto record = encoder.encode(pixels);
            binary_time += std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - start);
            std::uint32_t code = 0;
            const auto prefix = recorder::BinaryFormat::parse(record, code);
            assert(prefix > 0 && decoder.decode(code, record.subspan(prefix)));
            check_pixels(decoder, pixels, 640, 480, 128);
            ++kinds[code & 3U];
            binary_bytes += record.size();
            png_bytes += png.encode(pixels).size();
        }
        assert(kinds[recorder::BinaryFormat::kRaw] >= 1);
        assert(kinds[0] + kinds[recorder::BinaryFormat::kDelta] > 0 && binary_bytes < png_bytes / 10);
        std::printf("DBV synthetic fixed noise + moving target: frames=120 bytes=%zu PNG=%zu "
                    "encode_us_per_frame=%lld raw=%u full=%u delta=%u sparse=%u\n",
                    binary_bytes, png_bytes, static_cast<long long>(binary_time.count() / 120),
                    kinds[1], kinds[2], kinds[3], kinds[0]);
    }
    static std::expected<void, camera::CameraError>
        release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }
    static void sink(const recorder::FrameEvent& event, void* context) noexcept {
        static_cast<std::vector<recorder::FrameEvent>*>(context)->push_back(event);
    }
    static void recording() {
        char directory[] = "/tmp/dart-binary-XXXXXX";
        assert(::mkdtemp(directory));
        recorder::Config config;
        config.width = 9;
        config.height = 2;
        config.queue_slots = 4;
        config.image_format = recorder::ImageFormat::kDbv;
        config.output_directory = directory;
        config.max_file_bytes = 22; // 12-byte geometry + two identical 5-byte raw frames.
        std::vector<recorder::FrameEvent> events;
        config.event_sink = sink;
        config.event_sink_context = &events;
        recorder::Recorder rec{config};
        assert(rec.start());
        std::vector<std::byte> pixels(18);
        camera::FrameLeaseAccess::Slot slot;
        slot.release = release;
        slot.view.width = slot.view.stride = 9;
        slot.view.height = 2;
        slot.view.luma = pixels;
        for (unsigned frame = 0; frame < 2;) {
            slot.view.sequence++;
            slot.view.received += std::chrono::microseconds{20'000};
            slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
            auto lease = camera::FrameLeaseAccess::make(&slot);
            const auto admission = rec.try_submit(std::move(lease));
            (void)lease.release();
            while (rec.pending_releases())
                (void)rec.drain_releases();
            if (admission == recorder::Admission::kAccepted)
                ++frame;
            else
                assert(admission == recorder::Admission::kBusy);
        }
        for (unsigned wait = 0; wait < 1000 && !rec.limit_reached(); ++wait)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(rec.limit_reached());
        rec.stop();
        assert(rec.last_error() == recorder::Error::kNone);
        assert(rec.stats().written == 2 && rec.stats().bytes == config.max_file_bytes);
        const auto file = std::filesystem::path{rec.recording_path()};
        assert(file.filename() == "recording.dbv" && std::filesystem::file_size(file) == 22);
        unsigned accepted = 0;
        for (const auto& event : events) {
            if (event.decision != recorder::EventDecision::kAccepted)
                continue;
            assert(event.image_index == ++accepted);
            assert(std::string_view{event.image_name.data()} == "img/recording.dbv");
            assert(event.file_offset == (accepted == 1 ? 0U : 17U));
            assert(event.file_bytes == (accepted == 1 ? 17U : 5U));
        }
        assert(accepted == 2);
        // Restart the same encoder: its first frame must be independent again.
        assert(rec.start());
        rec.stop();
        config.keep_sessions = 1;
        recorder::RecorderError error;
        recorder::Session next{config, error};
        assert(next.open() && !std::filesystem::exists(file));
        std::filesystem::remove_all(directory);
    }
    static void boundary() {
        char directory[] = "/tmp/dart-binary-boundary-XXXXXX";
        assert(::mkdtemp(directory));
        recorder::Config config;
        config.width = 9;
        config.height = 2;
        config.image_format = recorder::ImageFormat::kDbv;
        config.output_directory = directory;
        config.max_file_bytes = 20; // First frame 17; a distinct raw frame needs 5 more.
        recorder::RecorderError error;
        recorder::Session session{config, error};
        assert(session.open());
        recorder::ImageWriter writer{config, session};
        assert(writer.prepare(18) && writer.open(error));
        std::vector<std::byte> pixels(18);
        assert(writer.write_image(pixels) && writer.record_bytes() == 17);
        pixels.assign(18, std::byte{255});
        assert(!writer.write_image(pixels) && errno == EFBIG);
        assert(writer.close());
        assert(std::filesystem::file_size(session.image_path()) == 17);
        // Reopening resets temporal history even after encoding an oversized frame.
        assert(writer.open(error) && writer.write_image(pixels));
        assert(writer.record_bytes() == 17 && writer.file_offset() == 0 && writer.close());
        std::filesystem::remove_all(directory);
    }

public:
    static void run() { roundtrip(); malformed(); compression(); recording(); boundary(); }
};

} // namespace dart::tests

int main() { dart::tests::RecorderBinaryTest::run(); }
