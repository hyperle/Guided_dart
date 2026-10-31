module;
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <sys/stat.h>

export module dart.storage.directories;



export namespace dart::storage {

void ensure_dir(const char* dir) noexcept {
    if (dir == nullptr || dir[0] == '\0')
        return;
    if (::mkdir(dir, 0777) == 0 || errno == EEXIST)
        return;
    if (errno == ENOENT) {
        char parent[512];
        const char* slash = std::strrchr(dir, '/');
        if (slash != nullptr && slash != dir) {
            const auto n = static_cast<std::size_t>(slash - dir);
            if (n < sizeof(parent)) {
                std::memcpy(parent, dir, n);
                parent[n] = '\0';
                (void)::mkdir(parent, 0777);
                (void)::mkdir(dir, 0777);
            }
        }
    }
}

void ensure_parent(const char* path) noexcept {
    char parent[512];
    const char* slash;
    if (path == nullptr)
        return;
    slash = std::strrchr(path, '/');
    if (slash == nullptr || slash == path)
        return;
    const auto n = static_cast<std::size_t>(slash - path);
    if (n >= sizeof(parent))
        return;
    std::memcpy(parent, path, n);
    parent[n] = '\0';
    ensure_dir(parent);
}

} // namespace dart::storage
