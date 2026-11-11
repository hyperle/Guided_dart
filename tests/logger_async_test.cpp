// Storage and console stalls must stay on the bounded sink's worker.
#include "logging/logger.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <thread>
#include <fcntl.h>
#include <unistd.h>

namespace dart::tests {

static std::atomic<bool> block_output{false}, output_entered{false}, producer_io{false};
static std::atomic<unsigned> sync_calls{0};
static std::thread::id producer;

extern "C" ssize_t __real_write(int fd, const void* bytes, std::size_t size);
extern "C" ssize_t __wrap_write(int fd, const void* bytes, std::size_t size) {
    if (std::this_thread::get_id() == producer) producer_io = true;
    output_entered = true;
    while (block_output.load()) std::this_thread::sleep_for(std::chrono::milliseconds{1});
    return __real_write(fd, bytes, size);
}
extern "C" int __wrap_fsync(int) { ++sync_calls; return 0; }
extern "C" int __wrap_fflush(std::FILE*) { ++sync_calls; return 0; }

class LoggerAsyncTest final {
    static std::string slurp(const std::filesystem::path& path) {
        std::ifstream input{path};
        return {std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
    }
    static std::size_t count_of(const std::string& text, std::string_view needle) {
        std::size_t count = 0;
        for (auto at = text.find(needle); at != std::string::npos; at = text.find(needle, at + needle.size()))
            ++count;
        return count;
    }
    static void wait_for_output() {
        for (unsigned i = 0; i < 3000 && !output_entered.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        assert(output_entered.load());
    }
    static void stall() { output_entered = false; block_output = true; }
    static std::uint64_t seed(logging::Logger& log, logging::Tag tag,
                              const char* text = "level=INFO op=seed\n") {
        stall();
        bool accepted = false;
        for (unsigned i = 0; i < 3000 && !accepted; ++i) {
            const auto before = log.dropped();
            log.line(tag, "%s", text);
            accepted = log.dropped() == before;
            if (!accepted) std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        assert(accepted);
        wait_for_output();
        return log.dropped();
    }

public:
    static void run() {
        producer = std::this_thread::get_id();
        const auto root = std::filesystem::path{"/tmp/dart_logger_async_test"};
        std::filesystem::remove_all(root);
        std::filesystem::create_directories(root);
        {
            logging::Logger log;
            assert(log.open((root / "ordered.log").string()));
            log.begin_batch();
            const auto dropped_before = seed(log, logging::Tag::kCamera);
            // The queue is empty, but its only row is still in the write call.
            assert(!log.flush(std::chrono::milliseconds{20}));
            const auto begin = std::chrono::steady_clock::now();
            for (unsigned i = 0; i < 200; ++i)
                log.line(logging::Tag::kRecord, "level=INFO op=ordered i=%u\n", i);
            assert(std::chrono::steady_clock::now() - begin < std::chrono::milliseconds{200});
            block_output = false;
            log.end_batch();
            assert(log.flush(std::chrono::milliseconds{5000}));
            assert(log.dropped() == dropped_before);
            const auto text = slurp(root / "ordered.log");
            assert(text.find("[camera] ts_ms=") != std::string::npos);
            assert(count_of(text, "[record]") == 200);
            std::size_t at = 0;
            for (unsigned i = 0; i < 200; ++i) {
                const auto next = text.find("i=" + std::to_string(i) + "\n", at);
                assert(next != std::string::npos);
                at = next + 1;
            }
        }
        std::uint64_t dropped = 0;
        {
            logging::Logger log;
            assert(log.open((root / "full.log").string()));
            const auto dropped_before = seed(log, logging::Tag::kApp);
            const auto begin = std::chrono::steady_clock::now();
            for (unsigned i = 0; i < 10000; ++i) log.line("level=INFO op=flood i=%u\n", i);
            assert(std::chrono::steady_clock::now() - begin < std::chrono::milliseconds{500});
            const auto stats = log.stats();
            assert(stats.queued == logging::LogSink::kLogQueueRows);
            assert(stats.high_water == logging::LogSink::kLogQueueRows);
            assert(stats.dropped - dropped_before == 10000 - logging::LogSink::kLogQueueRows);
            dropped = stats.dropped - dropped_before;
            block_output = false;
            assert(log.flush(std::chrono::milliseconds{5000}));
            assert(count_of(slurp(root / "full.log"), "op=flood") + dropped == 10000);
        }
        assert(slurp(root / "full.log").find("op=log_dropped") != std::string::npos);
        {
            logging::Logger log;
            assert(log.open((root / "tail.log").string()));
            (void)seed(log, logging::Tag::kApp);
            for (unsigned i = 0; i < 5; ++i) log.line("level=INFO op=tail i=%u\n", i);
            block_output = false;
        }
        assert(count_of(slurp(root / "tail.log"), "op=tail") == 5);

        // A console-only logger still starts a worker, applies the rate limit,
        // and reports its dropped tail at shutdown.
        const int saved_stderr = ::dup(STDERR_FILENO);
        const int console = ::open((root / "console.log").c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
        assert(saved_stderr >= 0 && console >= 0);
        assert(::dup2(console, STDERR_FILENO) >= 0);
        ::close(console);
        {
            logging::Logger log;
            log.set_console(true);
            assert(log.open(""));
            (void)seed(log, logging::Tag::kVision, "level=INFO op=console_seed\n");
            for (unsigned i = 0; i < 100; ++i)
                log.line(logging::Tag::kVision, "level=INFO op=console i=%u\n", i);
            block_output = false;
            assert(log.flush(std::chrono::milliseconds{5000}));
            assert(log.stats().console_dropped > 0);
            assert(log.stats().io_errors == 0);
        }
        {
            std::ofstream{root / "blocked_parent"} << "regular file";
            logging::Logger log;
            log.set_console(true);
            assert(log.open((root / "blocked_parent" / "app.log").string()));
            (void)seed(log, logging::Tag::kLog, "level=ERROR op=file_error_console\n");
            block_output = false;
            assert(log.flush(std::chrono::milliseconds{5000}));
            assert(log.stats().io_errors > 0);
        }
        assert(::dup2(saved_stderr, STDERR_FILENO) >= 0);
        ::close(saved_stderr);
        const auto mirrored = slurp(root / "console.log");
        assert(mirrored.find("[vision] ts_ms=") != std::string::npos);
        assert(mirrored.find("op=console_dropped") != std::string::npos);
        assert(mirrored.find("op=file_error_console") != std::string::npos);
        assert(!producer_io.load());
        assert(sync_calls.load() == 0);
        std::filesystem::remove_all(root);
    }
};

} // namespace dart::tests

int main() { dart::tests::LoggerAsyncTest::run(); }
