#!/usr/bin/env bash
# ============================================================================
# 安装 riscv64 musl 交叉工具链（首次使用本仓库时运行一次）
#
# 默认安装到仓库内 third_party/toolchain/（自包含，已被 .gitignore 忽略）
# 若希望使用 SDK 官方的 ~/.kendryte 位置，可传 --sdk：
#   bash scripts/setup-toolchain.sh --sdk
# ============================================================================
set -euo pipefail

TOOLCHAIN_URL="https://kendryte-download.canaan-creative.com/k230/toolchain/riscv64-unknown-linux-musl-rv64imafdcv-lp64d-20230420.tar.bz2"
TARBALL="riscv64-unknown-linux-musl-rv64imafdcv-lp64d-20230420.tar.bz2"
DIR_NAME="riscv64-linux-musleabi_for_x86_64-pc-linux-gnu"

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [[ "${1:-}" == "--sdk" ]]; then
    DEST="${HOME}/.kendryte/k230_toolchains"
else
    DEST="${REPO_ROOT}/third_party/toolchain"
fi

if [[ -x "${DEST}/${DIR_NAME}/bin/riscv64-unknown-linux-musl-gcc" ]]; then
    echo "工具链已存在: ${DEST}/${DIR_NAME}"
    exit 0
fi

echo "下载工具链（约 108MB）..."
mkdir -p "${DEST}"
curl -fL --retry 3 -o "${DEST}/${TARBALL}" "${TOOLCHAIN_URL}"
echo "解压中..."
tar xf "${DEST}/${TARBALL}" -C "${DEST}"
rm -f "${DEST}/${TARBALL}"

echo
echo "安装完成: ${DEST}/${DIR_NAME}"
echo "下一步: bash scripts/build.sh"
