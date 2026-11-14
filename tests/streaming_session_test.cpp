#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <expected>
#include <memory>
#include <span>
#include <thread>
#include <unistd.h>
#include <vector>

#include "camera/frame_view.hpp"
#include "streaming/session.hpp"

namespace dart::tests {

std::atomic_int opens{0}, uploads{0};
std::atomic_bool stalled{true}, fail_once{false};
class Transport final : public streaming::ITransport {
public:
    std::expected<void, streaming::Error>
        open(const streaming::Config&, const std::atomic_bool& stop) noexcept override {
        ++opens;
        if (fail_once.exchange(false))
            return std::unexpected(streaming::Error{streaming::Code::kConnect, 111});
        while (stalled.load() && !stop.load())
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        if (stop.load())
            return std::unexpected(streaming::Error{streaming::Code::kConnect, 1});
        return {};
    }
    std::expected<void, streaming::Error> upload(
        std::span<const std::uint8_t> jpeg, std::uint64_t sequence, std::uint64_t,
        const std::atomic_bool&) noexcept override {
        assert(sequence >= 1 && jpeg.size() > 100 && jpeg[0] == 0xff && jpeg[1] == 0xd8);
        ++uploads;
        return {};
    }
    void close() noexcept override {}
};
std::unique_ptr<streaming::ITransport> create_transport() noexcept {
    return std::make_unique<Transport>();
}
void write_config(const char* path, const char* suffix = "") {
    FILE* file = std::fopen(path, "w");
    assert(file);
    assert(
        std::fprintf(
            file, "host=127.0.0.1\ntoken=test-token\nfps=120\nwidth=16\nheight=16\n%s", suffix)
        > 0);
    assert(std::fclose(file) == 0);
}
void run() {
    char path[] = "/tmp/dart-streaming-test-XXXXXX";
    const int fd = mkstemp(path);
    assert(fd >= 0);
    close(fd);
    write_config(path);
    assert(streaming::load_config(path));
    for (const char* suffix :
         {"unknown=x\n", "fps=0\n", "host=1.2.3.4\n", "password=short\n", "port=-1\n",
          "port=65536\n", "quality=101\n"}) {
        write_config(path, suffix);
        assert(!streaming::load_config(path));
    }
    write_config(path);
    streaming::Session session;
    assert(session.start(path, 32, 32, &create_transport));
    std::vector<std::byte> pixels(32 * 40, std::byte{123});
    camera::FrameView frame;
    frame.pixels = pixels;
    frame.width = frame.height = 32;
    frame.stride = 40;
    frame.sequence = 1;
    for (unsigned i = 0; i < 60; ++i) {
        frame.received = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch());
        (void)session.try_submit(frame);
        ++frame.sequence;
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    assert(opens == 1 && uploads == 0);
    const auto stats = session.stats();
    assert(stats.accepted > 5 && stats.encoded > 5 && stats.jpeg_replaced > 0);
    frame.pixels = {};
    assert(!session.try_submit(frame));
    assert(session.stats().invalid == 1);
    stalled = false;
    for (unsigned i = 0; i < 100 && uploads == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    assert(uploads > 0);
    session.stop();
    assert(!session.try_submit(frame));
    // Restart must clear queue state and cancellation must release a stalled
    // transport before destruction. No network worker retains camera pixels.
    stalled = true;
    assert(session.start(path, 32, 32, &create_transport));
    frame.pixels = pixels;
    frame.received = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
    assert(session.try_submit(frame));
    for (unsigned i = 0; i < 100 && opens < 2; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    assert(opens == 2);
    const auto before = std::chrono::steady_clock::now();
    session.stop();
    assert(std::chrono::steady_clock::now() - before < std::chrono::seconds{1});
    // A refused connection drops its frame, backs off, then sends a newer one.
    stalled = false;
    fail_once = true;
    const int before_uploads = uploads.load();
    assert(session.start(path, 32, 32, &create_transport));
    for (unsigned i = 0; i < 60; ++i) {
        frame.received = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch());
        (void)session.try_submit(frame);
        ++frame.sequence;
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    session.stop();
    assert(opens >= 4 && uploads > before_uploads);
    assert(session.stats().network_errors == 1);
    // Stop while workers are entering their condition-variable wait. Repeated
    // empty sessions exercise startup rollback/notification/lifetime boundaries.
    for (unsigned i = 0; i < 30; ++i) {
        assert(session.start(path, 32, 32, &create_transport));
        session.stop();
    }
    assert(unlink(path) == 0);
}
} // namespace dart::tests

int main() { dart::tests::run(); }
