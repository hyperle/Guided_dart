#pragma once

#include <cstddef>
#include <cstdint>

#include "recorder/config.hpp"
#include "recorder/storage/session.hpp"
#include "recorder/storage/storage_space.hpp"

namespace dart::recorder {
// Per-image filesystem state belongs to the image worker, independently of
// session creation and retention. Encoding never accesses the filesystem.
class ImageFiles final {
public:
    ImageFiles(const Config& config, const Session& session) noexcept
        : config_(config)
        , session_(session) {}
    void reset() noexcept;
    bool png_path(
        std::uint64_t index, char* path, std::size_t path_size, char* name,
        std::size_t name_size) noexcept;
    bool space_available(std::size_t bytes) noexcept;

private:
    static bool mkdir_if_needed(const char* path) noexcept;
    const Config& config_;
    const Session& session_;
    std::uint64_t image_group_ = 0;
    std::uint64_t space_budget_ = 0;
    std::uint32_t space_checks_remaining_ = 0;
};

} // namespace dart::recorder
