#pragma once

#include <cstddef>

namespace dart::storage {
void ensure_dir(const char* dir) noexcept;

void ensure_parent(const char* path) noexcept;

} // namespace dart::storage
