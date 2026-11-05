#include "recorder/error.hpp"

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace dart::recorder {

void RecorderError::reset() noexcept {
    set_error(Error::kNone);
    system_error_ = 0;
    path_[0] = '\0';
}

void RecorderError::set_file_error(Error error, const char* path) noexcept {
    system_error_ = errno;
    std::snprintf(path_, sizeof(path_), "%s", path);
    set_error(error);
}

} // namespace dart::recorder
