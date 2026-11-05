#include "recorder/storage/image_files.hpp"

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <sys/stat.h>

#include "recorder/config.hpp"
#include "recorder/storage/session.hpp"
#include "recorder/storage/storage_space.hpp"

namespace dart::recorder {

void ImageFiles::reset() noexcept {
    image_group_ = space_budget_ = 0;
    space_checks_remaining_ = 0;
}

bool ImageFiles::mkdir_if_needed(const char* path) noexcept {
    if (::mkdir(path, 0777) == 0)
        return true;
    if (errno != EEXIST)
        return false;
    struct stat st{};
    return ::stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

bool ImageFiles::png_path(
    std::uint64_t index, char* path, std::size_t path_size, char* name,
    std::size_t name_size) noexcept {
    // Bound FAT directory size and lookup cost even for highly compressible frames.
    if (index == 0 || index > 99'999'999) {
        errno = EFBIG;
        return false;
    }
    char part[Session::kMaxPath]{}, bucket_path[Session::kMaxPath]{};
    const auto group_index = (index - 1U) / Config::kImagesPerGroup;
    const auto bucket =
        static_cast<unsigned long long>(group_index / Config::kGroupsPerBucket + 1U);
    const auto group = static_cast<unsigned long long>(group_index % Config::kGroupsPerBucket + 1U);
    const int b = std::snprintf(
        bucket_path, sizeof(bucket_path), "%s/b%07llu", session_.image_directory(), bucket);
    const int p = std::snprintf(part, sizeof(part), "%s/p%07llu", bucket_path, group);
    // Eight digits plus extension is a native FAT 8.3 name. A shared long
    // prefix would force repeated short-name alias searches for every file.
    const int n = std::snprintf(
        name, name_size, "img/b%07llu/p%07llu/%08llu.png", bucket, group,
        static_cast<unsigned long long>(index));
    const int f = std::snprintf(path, path_size, "%s/%s", session_.directory(), name);
    if (b <= 0 || static_cast<std::size_t>(b) >= sizeof(bucket_path) || p <= 0
        || static_cast<std::size_t>(p) >= sizeof(part) || n <= 0
        || static_cast<std::size_t>(n) >= name_size || f <= 0
        || static_cast<std::size_t>(f) >= path_size) {
        errno = ENAMETOOLONG;
        return false;
    }
    if (image_group_ == group_index + 1U)
        return true;
    if (!mkdir_if_needed(bucket_path) || !mkdir_if_needed(part))
        return false;
    image_group_ = group_index + 1U;
    return true;
}

bool ImageFiles::space_available(std::size_t bytes) noexcept {
    constexpr std::uint64_t guard = 1024U * 1024U;
    const bool continuous = config_.image_format == ImageFormat::kDbv;
    const auto charge = continuous ? static_cast<std::uint64_t>(bytes)
                                   : std::max<std::uint64_t>(bytes + guard, guard * 2U);
    if (space_checks_remaining_ > 0 && space_budget_ >= charge) {
        --space_checks_remaining_;
        space_budget_ -= charge;
        return true;
    }
    std::uint64_t available = 0, block = 0;
    if (!StorageSpace::query(
            session_.image_directory(), config_.max_storage_bytes, available, block))
        return false;
    // Include allocation rounding and room for directory growth. Logical PNG
    // sizes alone substantially underestimate FAT usage for small images.
    // Continuous DBV records share clusters. Cover pending stdio bytes once
    // per query instead of charging every small record as an independent file.
    const auto allocation =
        continuous ? ((bytes + 256U * 1024U + block - 1U) / block) * block
                   : std::max<std::uint64_t>(charge, ((bytes + block - 1U) / block + 2U) * block);
    if (available < allocation) {
        errno = ENOSPC;
        return false;
    }
    space_budget_ = available - allocation;
    space_checks_remaining_ = config_.storage_check_frames - 1U;
    return true;
}

} // namespace dart::recorder
