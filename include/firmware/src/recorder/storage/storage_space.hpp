#pragma once

#include <cstdint>

#include "recorder/config.hpp"

namespace dart::recorder {
// One volume budget for both session admission and periodic image checks.
// Count allocated blocks across the filesystem, including older recordings.
class StorageSpace final {
public:
    static bool query(
        const char* path, std::uint64_t max_storage_bytes, std::uint64_t& available,
        std::uint64_t& block_bytes) noexcept;
};

} // namespace dart::recorder
