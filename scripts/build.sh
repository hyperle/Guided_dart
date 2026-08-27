#!/usr/bin/env bash
# ============================================================================
# 编译所有业务应用（或指定应用）
#
# 用法:
#   bash scripts/build.sh            # 编译 apps/ 下全部应用
#   bash scripts/build.sh my_app     # 只编译指定应用
#
# 产物: build/<应用名>/<应用名>  （riscv64 静态链接 ELF，可直接拷到板子运行）
# 环境变量（可选）:
#   K230_SDK_ROOT   SDK 根目录，默认 /mnt/mydata/kRTOSSDK
#   BUILD_DIR       构建目录，默认 build
# ============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SDK_ROOT="${K230_SDK_ROOT:-/mnt/mydata/kRTOSSDK}"
BUILD_DIR="${BUILD_DIR:-build}"
APP="${1:-}"

cd "${REPO_ROOT}"

cmake -S . -B "${BUILD_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE="${REPO_ROOT}/cmake/toolchain-riscv64-musl.cmake" \
    -DK230_SDK_ROOT="${SDK_ROOT}" \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [[ -n "${APP}" ]]; then
    cmake --build "${BUILD_DIR}" --target "${APP}"
else
    cmake --build "${BUILD_DIR}"
fi

echo
echo "编译完成，产物位于:"
find "${BUILD_DIR}" -maxdepth 2 -type f -executable ! -path '*CMakeFiles*' || true
echo "部署到板子: bash scripts/deploy.sh <应用名> <板子IP>"
