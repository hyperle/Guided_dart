#include "recorder/storage/storage_space.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <limits>
#include <sys/statvfs.h>

#include "recorder/config.hpp"

namespace dart::recorder {

bool StorageSpace::query(
    const char* path, std::uint64_t max_storage_bytes, std::uint64_t& available,
    std::uint64_t& block_bytes) noexcept {
    struct statvfs storage{};
    if (::statvfs(path, &storage) != 0)
        return false;
    block_bytes = storage.f_frsize != 0 ? storage.f_frsize : storage.f_bsize;
    if (block_bytes == 0
        || storage.f_bavail > std::numeric_limits<std::uint64_t>::max() / block_bytes) {
        errno = EOVERFLOW;
        return false;
    }
    available = static_cast<std::uint64_t>(storage.f_bavail) * block_bytes;
    if (max_storage_bytes != 0) {
        if (storage.f_blocks == 0 || storage.f_bfree > storage.f_blocks
            || storage.f_blocks > std::numeric_limits<std::uint64_t>::max() / block_bytes) {
            errno = EOVERFLOW;
            return false;
        }
        const auto used =
            static_cast<std::uint64_t>(storage.f_blocks - storage.f_bfree) * block_bytes;
        const auto remaining = used < max_storage_bytes ? max_storage_bytes - used : 0;
        available = std::min(available, remaining);
    }
    // Leave room for metadata, buffered writes and continuing application logs
    // within both the physical volume and the configured total ceiling.
    if (available <= Config::kStorageReserveBytes) {
        errno = ENOSPC;
        return false;
    }
    available -= Config::kStorageReserveBytes;
    return true;
}

} // namespace dart::recorder
