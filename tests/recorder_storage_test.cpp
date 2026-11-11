#include <cstdint>
#include <cstdio>
#include <sys/types.h>
#include <cstddef>
#include <expected>
#include <chrono>
#include <utility>
#include <string>
#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sys/statvfs.h>
#include <thread>
#include <vector>

#include "tests/recorder_fixture.hpp"

#include "recorder/storage/image_writer.hpp"
#include "recorder/storage/session.hpp"

namespace dart::tests {

static std::uint64_t available_bytes = 0;
static std::uint64_t volume_bytes = 4'000'000'000;
static std::uint64_t used_bytes = 0;
static std::uint64_t block_bytes = 1;
static bool storage_failure = false;
static bool write_failure = false;
static std::atomic<unsigned> sync_count{0}, flush_count{0};
static std::atomic<bool> block_write{false};
static std::atomic<bool> write_blocked{false};

extern "C" std::size_t __real_fwrite(const void*, std::size_t, std::size_t, std::FILE*);
extern "C" ssize_t __real_write(int fd, const void* data, std::size_t bytes);

extern "C" int __wrap_statvfs(const char*, struct statvfs* storage) {
    if (storage_failure) {
        errno = EIO;
        return -1;
    }
    *storage = {};
    storage->f_frsize = storage->f_bsize = block_bytes;
    storage->f_blocks = volume_bytes / block_bytes;
    storage->f_bfree = (volume_bytes - used_bytes) / block_bytes;
    storage->f_bavail = available_bytes / block_bytes;
    return 0;
}

static bool storage_ready() {
    if (block_write.load()) {
        write_blocked.store(true);
        while (block_write.load())
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    if (write_failure) {
        errno = EIO;
        return false;
    }
    return true;
}

extern "C" std::size_t __wrap_fwrite(const void* data, std::size_t size, std::size_t count, std::FILE* file) {
    return storage_ready() ? __real_fwrite(data, size, count, file) : 0;
}

extern "C" ssize_t __wrap_write(int fd, const void* data, std::size_t bytes) {
    return storage_ready() ? __real_write(fd, data, bytes) : -1;
}

extern "C" int __wrap_fflush(std::FILE*) {
    ++flush_count;
    errno = EIO;
    return -1;
}

extern "C" int __wrap_fsync(int) {
    ++sync_count;
    errno = EIO;
    return -1;
}

class RecorderStorageTest final {
public:
    static void run() {
        char path[] = "/tmp/dart-recorder-storage-XXXXXX";
        assert(::mkdtemp(path) != nullptr);
        recorder::Config config{};
        config.output_directory = path;
        config.width = config.height = 1;
        recorder::RecorderError error;

        // A full or inaccessible volume must not create an empty session.
        for (const bool failed_query : {false, true}) {
            storage_failure = failed_query;
            available_bytes = recorder::Config::kStorageReserveBytes;
            recorder::Session session{config, error};
            assert(!session.open());
            assert(error.last_error() == recorder::Error::kStorage);
            assert(error.last_system_error() == (failed_query ? EIO : ENOSPC));
            assert(std::filesystem::is_empty(path));
        }
        storage_failure = false;
        // One 1x1 PGM record is a complete 11-byte header plus one pixel.
        constexpr std::size_t record_bytes = sizeof("P5\n1 1\n255\n") - 1 + 1;
        available_bytes = recorder::Config::kStorageReserveBytes + 2 * record_bytes;
        recorder::Session session{config, error};
        assert(session.open());
        assert(session.file_limit_bytes() == 2 * record_bytes);
        ImageWriterFixture writer{config, session};
        assert(writer.prepare(1) && writer.open(error));
        const std::array<std::byte, 1> pixel{std::byte{123}};
        assert(writer.write_image(pixel));
        assert(flush_count == 0 && sync_count == 0);
        assert(std::filesystem::file_size(session.image_path()) == 0);
        assert(writer.write_image(pixel));
        assert(!writer.write_image(pixel) && errno == EFBIG);
        assert(writer.close());
        assert(std::filesystem::file_size(session.image_path()) == 2 * record_bytes);

        // Write errors must propagate instead of reporting success.
        recorder::Session failed_session{config, error};
        assert(failed_session.open());
        ImageWriterFixture failed_writer{config, failed_session};
        assert(failed_writer.prepare(1) && failed_writer.open(error));
        write_failure = true;
        assert(!failed_writer.write_image(pixel) && errno == EIO);
        write_failure = false;
        assert(failed_writer.close());
        std::filesystem::remove_all(path);
        packing();
        buffered_frame();
        png_storage();
        partition_limit();
        h264_storage();
        assert(sync_count == 0 && flush_count == 0);
    }

private:
    static void h264_storage() {
        char path[] = "/tmp/dart-h264-storage-XXXXXX";
        assert(::mkdtemp(path));
        recorder::Config config;
        config.output_directory = path;
        config.width = config.height = 2;
        config.fps = 30;
        config.image_format = recorder::ImageFormat::kH264;
        available_bytes = recorder::Config::kStorageReserveBytes + 1'000'000;
        recorder::RecorderError error;
        recorder::Session session{config, error};
        assert(session.open());
        recorder::ImageWriter writer{config, session};
        recorder::EventWriter events{config};
        assert(writer.prepare(4) && writer.open(error));
        assert(events.prepare() && events.open(session, error));
        // Exercise storage with already encoded bytes, independently of VENC.
        const std::array<std::byte, 6> payload{
            std::byte{0}, std::byte{0}, std::byte{0}, std::byte{1}, std::byte{0x65}, std::byte{0x88}};
        for (unsigned index = 1; index <= 2; ++index) {
            assert(writer.write_encoded({{}, payload}, index));
            assert(writer.file_offset() == (index - 1U) * payload.size());
            recorder::FrameEvent event{};
            event.input_index = event.image_index = index;
            event.decision = recorder::EventDecision::kAccepted;
            event.file_offset = writer.file_offset();
            event.file_bytes = writer.record_bytes();
            std::snprintf(event.image_name.data(), event.image_name.size(), "%s", writer.image_name());
            assert(events.write(event));
        }
        assert(writer.close());
        assert(events.close());
        std::ifstream input(session.image_path(), std::ios::binary);
        const std::string actual{std::istreambuf_iterator<char>{input}, {}};
        const std::string expected{reinterpret_cast<const char*>(payload.data()), payload.size()};
        assert(actual == expected + expected);
        std::ifstream csv(session.event_path());
        const std::string metadata{std::istreambuf_iterator<char>{csv}, {}};
        assert(metadata.find("accepted,1,img/recording.h264,0,6") != std::string::npos);
        assert(metadata.find("accepted,2,img/recording.h264,6,6") != std::string::npos);
        std::filesystem::remove_all(path);
    }

    static void partition_limit() {
        char path[] = "/tmp/dart-partition-limit-XXXXXX";
        assert(::mkdtemp(path));
        const std::filesystem::path root{path};
        std::filesystem::create_directories(root / "rec_0001/img");
        std::ofstream{root / "rec_0001/img/recording.pgm"} << "earlier recording";
        recorder::Config config;
        config.output_directory = path;
        config.width = config.height = 8;
        config.max_storage_bytes = recorder::Config::kMaxStorageBytes;
        config.storage_check_frames = 1;
        recorder::RecorderError error;
        // Plenty of physical free space remains, but all earlier volume usage
        // counts against the total ceiling. No session or old file is removed.
        available_bytes = 1'000'000'000;
        for (const auto used : {config.max_storage_bytes - recorder::Config::kStorageReserveBytes,
                                config.max_storage_bytes, config.max_storage_bytes + 1}) {
            used_bytes = used;
            RecorderFixture rec{config};
            assert(!rec.start());
            assert(rec.last_error() == recorder::Error::kStorage);
            assert(rec.last_system_error() == ENOSPC);
            assert(!std::filesystem::exists(root / "rec_0002"));
            assert(std::filesystem::exists(root / "rec_0001/img/recording.pgm"));
        }
        used_bytes = config.max_storage_bytes - recorder::Config::kStorageReserveBytes - 1234;
        recorder::Session limited{config, error};
        assert(limited.open() && limited.file_limit_bytes() == 1234);

        const std::array<std::byte, 64> pixels{};
        for (const auto format : {recorder::ImageFormat::kDbv, recorder::ImageFormat::kPng, recorder::ImageFormat::kPgm,
                                  recorder::ImageFormat::kPbm}) {
            config.image_format = format;
            used_bytes = 0;
            recorder::Session session{config, error};
            assert(session.open());
            assert(session.file_limit_bytes() == recorder::Config::kMaxFileBytes);
            ImageWriterFixture writer{config, session};
            assert(writer.prepare(pixels.size()) && writer.open(error));
            assert(writer.write_image(pixels, 1));
            const auto bytes = writer.record_bytes();
            // Logs or another recording consume the remaining total budget.
            used_bytes = config.max_storage_bytes - recorder::Config::kStorageReserveBytes;
            assert(!writer.write_image(pixels, 2) && errno == ENOSPC);
            assert(writer.close());
            if (format == recorder::ImageFormat::kPng)
                assert(!std::filesystem::exists(
                    std::filesystem::path{session.directory()} / "img/b0000001/p0000001/00000002.png"));
            else
                assert(std::filesystem::file_size(session.image_path()) == bytes);
        }

        // A total budget must also bound cached checks, independently of ample
        // physical space. Each small PNG consumes a conservative 2 MiB charge.
        config.image_format = recorder::ImageFormat::kPng;
        config.storage_check_frames = 32;
        used_bytes = config.max_storage_bytes - recorder::Config::kStorageReserveBytes
                   - 4U * 1024U * 1024U;
        recorder::Session cached{config, error};
        assert(cached.open());
        ImageWriterFixture output{config, cached};
        assert(output.prepare(pixels.size()) && output.open(error));
        assert(output.write_image(pixels, 1));
        assert(output.write_image(pixels, 2));
        used_bytes += 4U * 1024U * 1024U;
        assert(!output.write_image(pixels, 3) && errno == ENOSPC);
        assert(output.close());

        // Non-byte blocks must count allocation, not logical PNG sizes.
        block_bytes = 4096;
        used_bytes = config.max_storage_bytes;
        recorder::Session allocated{config, error};
        assert(!allocated.open() && error.last_system_error() == ENOSPC);
        // Reject corrupt capacity data rather than wrapping a subtraction.
        used_bytes = 0;
        volume_bytes = 0;
        recorder::Session invalid{config, error};
        assert(!invalid.open() && error.last_system_error() == EOVERFLOW);
        volume_bytes = 4'000'000'000;
        block_bytes = 1;
        std::filesystem::remove_all(root);
    }

    static void png_storage() {
        char path[] = "/tmp/dart-png-storage-XXXXXX";
        assert(::mkdtemp(path));
        recorder::Config config;
        config.width = config.height = 8;
        config.image_format = recorder::ImageFormat::kPng;
        config.output_directory = path;
        config.storage_check_frames = 1;
        available_bytes = recorder::Config::kStorageReserveBytes + 8U * 1024U * 1024U;
        recorder::RecorderError error;
        recorder::Session session{config, error};
        assert(session.open());
        ImageWriterFixture writer{config, session};
        assert(writer.prepare(64) && writer.open(error));
        const std::array<std::byte, 64> pixels{};
        write_failure = true;
        assert(!writer.write_image(pixels, 1) && errno == EIO);
        write_failure = false;
        for (const auto& entry : std::filesystem::recursive_directory_iterator(session.directory()))
            assert(entry.path().extension() != ".png" && entry.path().extension() != ".tmp");
        assert(writer.write_image(pixels, 1));
        const auto first_bytes = writer.record_bytes();
        assert(first_bytes > 0);
        // Other files and FAT allocation can consume space after start().
        available_bytes = recorder::Config::kStorageReserveBytes;
        assert(!writer.write_image(pixels, 2) && errno == ENOSPC);
        assert(!std::filesystem::exists(
            std::filesystem::path{session.directory()} / "img/b0000001/p0000001/00000002.png"));
        available_bytes += 8U * 1024U * 1024U;
        storage_failure = true;
        assert(!writer.write_image(pixels, 2) && errno == EIO);
        storage_failure = false;
        assert(writer.close());
        // Exact fit writes a complete image; one byte less writes no image.
        for (const auto cap : {first_bytes - 1, first_bytes}) {
            config.max_file_bytes = cap;
            recorder::Session limited{config, error};
            assert(limited.open());
            ImageWriterFixture output{config, limited};
            assert(output.prepare(64) && output.open(error));
            assert(output.write_image(pixels, 1) == (cap == first_bytes));
            if (cap < first_bytes)
                assert(errno == EFBIG);
            assert(output.close());
        }
        // A cached space check must query again once its conservative allocation
        // budget runs out, even before the configured frame interval expires.
        config.max_file_bytes = recorder::Config::kMaxFileBytes;
        config.storage_check_frames = 32;
        available_bytes = recorder::Config::kStorageReserveBytes + 4U * 1024U * 1024U;
        recorder::Session cached{config, error};
        assert(cached.open());
        ImageWriterFixture cached_writer{config, cached};
        assert(cached_writer.prepare(64) && cached_writer.open(error));
        assert(cached_writer.write_image(pixels, 1));
        assert(cached_writer.write_image(pixels, 2));
        available_bytes = recorder::Config::kStorageReserveBytes;
        assert(!cached_writer.write_image(pixels, 3) && errno == ENOSPC);
        assert(cached_writer.close());
        std::filesystem::remove_all(path);
    }

    static std::expected<void, camera::CameraError>
        release(camera::FrameLeaseAccess::Slot* slot) noexcept {
        slot->state = camera::FrameLeaseAccess::SlotState::kFree;
        return {};
    }

    static void buffered_frame() {
        char path[] = "/tmp/dart-recorder-slow-disk-XXXXXX";
        assert(::mkdtemp(path));
        recorder::Config config{};
        config.output_directory = path;
        config.width = 8;
        config.height = 1;

        config.frame_timeout = std::chrono::milliseconds{5000};
        config.queue_slots = 1;
        RecorderFixture rec{config};
        assert(rec.start());
        std::array<std::byte, 8> pixels{};
        camera::FrameLeaseAccess::Slot slot{};
        slot.view.luma = pixels;
        slot.view.width = slot.view.stride = 8;
        slot.view.height = 1;
        slot.release = release;
        slot.view.received = std::chrono::microseconds{980'000};
        block_write.store(true);
        write_blocked.store(false);
        slot.state = camera::FrameLeaseAccess::SlotState::kHeld;
        auto lease = camera::FrameLeaseAccess::make(&slot);
        for (unsigned attempt = 0;; ++attempt) {
            assert(attempt < 1000);
            slot.view.received += std::chrono::milliseconds{20};
            if (rec.try_submit(std::move(lease)) == recorder::Admission::kAccepted) break;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        for (unsigned attempt = 0; attempt < 2000 && !write_blocked.load(); ++attempt)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(write_blocked.load() && !lease.valid());
        // The blocked sink retains an owned PGM copy and returns the camera slot.
        rec.wait_released();
        assert(slot.state == camera::FrameLeaseAccess::SlotState::kFree);
        assert(rec.stats().buffered == 1 && rec.stats().buffer_high_water == 1);
        camera::FrameLeaseAccess::Slot other{};
        other.view = slot.view;
        other.view.received += std::chrono::milliseconds{20};
        other.release = release;
        other.state = camera::FrameLeaseAccess::SlotState::kHeld;
        auto rejected = camera::FrameLeaseAccess::make(&other);
        assert(rec.try_submit(std::move(rejected)) == recorder::Admission::kBusy);
        assert(rejected.valid() && rejected.release());
        block_write.store(false);
        rec.wait_released();
        rec.stop();
        assert(rec.stats().written == 1 && rec.pending_releases() == 0);
        assert(rec.stats().copied == 0 && rec.last_error() == recorder::Error::kNone);
        std::ifstream input(rec.recording_path(), std::ios::binary);
        const std::string bytes{std::istreambuf_iterator<char>{input}, {}};
        assert(bytes == "P5\n8 1\n255\n" + std::string(8, '\0'));
        std::filesystem::remove_all(path);
    }

    static void packing() {
        char path[] = "/tmp/dart-recorder-packing-XXXXXX";
        assert(::mkdtemp(path));
        available_bytes = recorder::Config::kStorageReserveBytes + 1'000'000;
        for (const unsigned width : {1U, 7U, 8U, 9U, 15U, 16U, 17U, 257U}) {
            for (const unsigned threshold : {0U, 1U, 128U, 255U}) {
                recorder::Config config{};
                config.output_directory = path;
                config.width = width;
                config.height = 3;
                config.image_format = recorder::ImageFormat::kPbm;
                config.pbm_threshold = threshold;
                recorder::RecorderError error;
                recorder::Session session{config, error};
                assert(session.open());
                ImageWriterFixture writer{config, session};
                std::vector<std::byte> pixels(width * config.height);
                assert(writer.prepare(pixels.size()) && writer.open(error));
                for (unsigned pass = 0; pass < 2; ++pass) {
                    for (std::size_t i = 0; i < pixels.size(); ++i)
                        pixels[i] = static_cast<std::byte>((i + pass * 127U) % 256U);
                    assert(writer.write_image(pixels));
                }
                assert(writer.close());
                std::ifstream input(session.image_path(), std::ios::binary);
                const std::string bytes{std::istreambuf_iterator<char>{input}, {}};
                const std::string header = "P4\n" + std::to_string(width) + " 3\n";
                const auto row_bytes = (width + 7U) / 8U;
                const auto record_bytes = header.size() + row_bytes * config.height;
                assert(bytes.size() == 2 * record_bytes);
                for (unsigned pass = 0; pass < 2; ++pass) {
                    assert(bytes.substr(pass * record_bytes, header.size()) == header);
                    for (unsigned y = 0; y < config.height; ++y) {
                        for (unsigned x = 0; x < row_bytes * 8U; ++x) {
                            const auto packed = static_cast<unsigned char>(
                                bytes
                                    [pass * record_bytes + header.size() + y * row_bytes + x / 8U]);
                            const bool black = (packed & (0x80U >> (x % 8U))) != 0;
                            const auto gray = (y * width + x + pass * 127U) % 256U;
                            assert(black == (x < width && gray < threshold));
                        }
                    }
                }
            }
        }
        std::filesystem::remove_all(path);
    }
};

} // namespace dart::tests

int main() { dart::tests::RecorderStorageTest::run(); }
