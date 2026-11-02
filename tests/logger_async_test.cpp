// Log sink isolation: a blocked fsync must never block the producer, queued
// lines must stay ordered and complete after drain(), the bounded queue must
// drop instead of growing, and the destructor must persist the tail.
//
// fsync is wrapped the same way recorder_png_test blocks the recorder's writer:
// the wrapper sleeps for a while so a synchronous logger would show up as a
// multi-second producer call.

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>

#include "logging/logger.cpp"

namespace dart::tests {

static std::atomic<int> fsync_delay_ms{0};
static std::atomic<unsigned> fsync_calls{0};

extern "C" int __real_fsync(int fd);
extern "C" int __wrap_fsync(int fd) {
    fsync_calls.fetch_add(1);
    const int delay = fsync_delay_ms.load();
    if (delay > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds{delay});
    return __real_fsync(fd);
}

class LoggerAsyncTest final {
    static std::string slurp(const std::filesystem::path& path) {
        std::ifstream input{path};
        return {std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
    }
    static std::size_t count_lines(const std::string& text) {
        std::size_t lines = 0;
        for (const char ch : text)
            lines += ch == '\n' ? 1 : 0;
        return lines;
    }
    static std::size_t count_of(const std::string& text, std::string_view needle) {
        std::size_t hits = 0;
        for (std::size_t at = text.find(needle); at != std::string::npos;
             at = text.find(needle, at + needle.size()))
            ++hits;
        return hits;
    }
    static bool wait_for_calls(unsigned at_least) {
        for (int i = 0; i < 2000; ++i) {
            if (fsync_calls.load() >= at_least)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        return false;
    }

public:
    static void run() {
        const auto root = std::filesystem::path{"/tmp/dart_logger_async_test"};
        std::filesystem::remove_all(root);
        std::filesystem::create_directories(root);

        /* A producer must not wait for storage: twenty lines while fsync sleeps
         * 300 ms each would take six seconds on the old synchronous path. */
        {
            const auto log_file = root / "app.log";
            logging::Logger log;
            assert(log.open(log_file.string().c_str()));
            const unsigned calls_before = fsync_calls.load();
            fsync_delay_ms.store(300);
            const auto begin = std::chrono::steady_clock::now();
            for (int i = 0; i < 20; ++i)
                log.line(logging::Tag::kApp, "level=INFO op=blocked i=%d\n", i);
            log.end_batch();
            const auto elapsed = std::chrono::steady_clock::now() - begin;
            assert(elapsed < std::chrono::milliseconds{200});
            assert(wait_for_calls(calls_before + 1));
            fsync_delay_ms.store(0);
            assert(log.flush(std::chrono::milliseconds{5000}));
            assert(count_lines(slurp(log_file)) == 20);
            assert(log.dropped() == 0);
        }

        /* Order and completeness survive the queue hand-off. */
        {
            const auto log_file = root / "ordered.log";
            logging::Logger log;
            assert(log.open(log_file.string().c_str()));
            for (int i = 0; i < 200; ++i) {
                log.line(logging::Tag::kApp, "level=INFO op=ordered i=%d\n", i);
                if (i % 32 == 31) {
                    log.end_batch();
                    assert(log.flush(std::chrono::milliseconds{5000}));
                }
            }
            log.end_batch();
            assert(log.flush(std::chrono::milliseconds{5000}));
            const std::string text = slurp(log_file);
            assert(count_lines(text) == 200);
            assert(log.dropped() == 0);
            std::size_t at = 0;
            for (int i = 0; i < 200; ++i) {
                const auto needle = "i=" + std::to_string(i) + "\n";
                const auto found = text.find(needle, at);
                assert(found != std::string::npos);
                at = found + needle.size();
            }
        }

        /* A full queue drops and counts instead of blocking or growing. */
        {
            const auto log_file = root / "dropped.log";
            logging::Logger log;
            assert(log.open(log_file.string().c_str()));
            const unsigned calls_before = fsync_calls.load();
            fsync_delay_ms.store(300);
            log.line(logging::Tag::kApp, "level=INFO op=fill\n");
            log.end_batch();
            assert(wait_for_calls(calls_before + 1));
            const auto begin = std::chrono::steady_clock::now();
            for (int i = 0; i < 10000; ++i)
                log.line(logging::Tag::kApp, "level=INFO op=flood i=%d\n", i);
            const auto elapsed = std::chrono::steady_clock::now() - begin;
            assert(elapsed < std::chrono::milliseconds{500});
            assert(log.dropped() > 0);
            fsync_delay_ms.store(0);
            assert(log.flush(std::chrono::milliseconds{5000}));
            const auto dropped = log.dropped();
            const std::string text = slurp(log_file);
            const std::size_t flood_lines = count_of(text, "op=flood");
            /* Every accepted row reached the file; the rest were counted. */
            assert(flood_lines + dropped == 10000);
            assert(dropped > 0);
            assert(text.find("op=log_dropped") != std::string::npos);
        }

        /* The destructor drains: the tail must reach the file at shutdown. */
        {
            const auto log_file = root / "shutdown.log";
            const unsigned calls_before = fsync_calls.load();
            {
                logging::Logger log;
                assert(log.open(log_file.string().c_str()));
                for (int i = 0; i < 5; ++i)
                    log.line(logging::Tag::kApp, "level=INFO op=tail i=%d\n", i);
            }
            assert(count_lines(slurp(log_file)) == 5);
            assert(fsync_calls.load() > calls_before);
        }

        std::filesystem::remove_all(root);
    }
};

} // namespace dart::tests

int main() { dart::tests::LoggerAsyncTest::run(); }
