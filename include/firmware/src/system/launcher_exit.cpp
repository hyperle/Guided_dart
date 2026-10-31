module;
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

export module dart.system.launcher_exit;



export namespace dart::platform {

void notify_launcher_exit() noexcept {
    const char* marker = std::getenv("DART_EXIT_MARKER");
    if (marker != nullptr && marker[0] == '/' && std::strlen(marker) < 512) {
        const int fd = ::open(marker, O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (fd >= 0) {
            char pid_text[32];
            const int length =
                std::snprintf(pid_text, sizeof(pid_text), "%ld\n", static_cast<long>(::getpid()));
            if (length > 0 && static_cast<std::size_t>(length) < sizeof(pid_text)) {
                (void)::write(fd, pid_text, static_cast<std::size_t>(length));
                (void)::fsync(fd);
            }
            ::close(fd);
        }
    }
}

} // namespace dart::platform
