// Logger console mirror: file/console parity, rate limit, drop summary on exit,
// and the write-control pause silencing both sinks.

#include <cassert>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include "logging/logger.cpp"

namespace dart::tests {

class LoggerConsoleTest final {
    /* Capture writes to STDERR_FILENO by swapping in a file descriptor. The
     * logger writes with ::write(), so redirecting the descriptor is enough. */
    class StderrCapture final {
    public:
        explicit StderrCapture(const std::filesystem::path& path) {
            saved_ = ::dup(STDERR_FILENO);
            assert(saved_ >= 0);
            const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
            assert(fd >= 0);
            assert(::dup2(fd, STDERR_FILENO) >= 0);
            ::close(fd);
        }
        ~StderrCapture() {
            ::fflush(stderr);
            if (saved_ >= 0) {
                (void)::dup2(saved_, STDERR_FILENO);
                ::close(saved_);
            }
        }
        StderrCapture(const StderrCapture&) = delete;
        StderrCapture& operator=(const StderrCapture&) = delete;

    private:
        int saved_ = -1;
    };

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

    static bool wait_paused(platform::WriteControl& writes, bool expected) {
        for (int i = 0; i < 400; ++i) {
            if (writes.paused() == expected)
                return true;
            ::usleep(5000);
        }
        return false;
    }

public:
    static void run() {
        const auto root = std::filesystem::path{"/tmp/dart_logger_console_test"};
        std::filesystem::remove_all(root);
        std::filesystem::create_directories(root);
        const auto pause = root / "write.pause";
        const auto log_file = root / "app.log";
        const auto console_file = root / "console.txt";

        platform::WriteControl writes;
        assert(writes.start(pause.c_str()));

        {
            StderrCapture capture{console_file};
            logging::Logger log{&writes};
            assert(log.open(log_file.string().c_str()));
            log.set_console(true);

            /* Burst far above the token bucket: the file log keeps every line,
             * the console keeps the initial burst and summarizes the rest. */
            for (int i = 0; i < 100; ++i)
                log.line("level=INFO module=test op=burst i=%d\n", i);
            log.end_batch();
        }

        const std::string console_text = slurp(console_file);
        const std::size_t console_burst = count_of(console_text, "op=burst");
        assert(count_lines(slurp(log_file)) == 100);
        assert(console_burst >= logging::kConsoleBurst);
        assert(console_burst < 100);
        assert(count_of(console_text, "op=console_dropped") == 1);
        assert(count_of(console_text, "module=log") == 1);
        /* The exit summary must account for every line the console dropped. */
        assert(console_text.find("count=" + std::to_string(100 - console_burst)) !=
               std::string::npos);

        /* Pause must silence the file log and the console mirror together. */
        std::ofstream{pause} << "pause";
        assert(wait_paused(writes, true));
        {
            StderrCapture capture{console_file};
            logging::Logger log{&writes};
            assert(log.open(log_file.string().c_str()));
            log.set_console(true);
            log.line("level=INFO module=test op=paused\n");
            log.end_batch();
            assert(slurp(console_file).find("op=paused") == std::string::npos);
        }
        assert(slurp(log_file).find("op=paused") == std::string::npos);

        std::filesystem::remove(pause);
        assert(wait_paused(writes, false));
        {
            StderrCapture capture{console_file};
            logging::Logger log{&writes};
            assert(log.open(log_file.string().c_str()));
            log.set_console(true);
            log.line("level=INFO module=test op=resumed\n");
            log.end_batch();
            assert(slurp(console_file).find("op=resumed") != std::string::npos);
        }
        assert(slurp(log_file).find("op=resumed") != std::string::npos);

        std::filesystem::remove_all(root);
    }
};

} // namespace dart::tests

int main() { dart::tests::LoggerConsoleTest::run(); }
