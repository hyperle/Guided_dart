#!/usr/bin/env bash
# ============================================================================
# 把编译好的应用拷贝到板子（移植到 RTOS 板端）
#
# 用法:
#   bash scripts/deploy.sh <应用名> <板子IP> [目标目录]
#
# 示例:
#   bash scripts/deploy.sh my_app 192.168.1.100
#   bash scripts/deploy.sh my_app 192.168.1.100 /sdcard/app
#
# 默认拷贝到板端 /tmp，然后在板端 msh 终端中运行:
#   msh> /tmp/my_app
#
# 若你的固件把应用放在 /sdcard/app/ 下（官方默认镜像布局），可指定目标
# 目录为 /sdcard/app，板端运行: msh> /sdcard/app/my_app
# 也可在板端自行搬运: msh> cp /tmp/my_app /sdcard/app/
#
# 环境变量（可选）: BOARD_USER（默认 root）、BUILD_DIR（默认 build）
# ============================================================================
set -euo pipefail

APP="${1:-}"
BOARD_IP="${2:-}"
DEST_DIR="${3:-/tmp}"
BUILD_DIR="${BUILD_DIR:-build}"
BOARD_USER="${BOARD_USER:-root}"

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [[ -z "${APP}" || -z "${BOARD_IP}" ]]; then
    echo "用法: bash scripts/deploy.sh <应用名> <板子IP> [目标目录]" >&2
    exit 1
fi

ELF="${REPO_ROOT}/${BUILD_DIR}/${APP}/${APP}"
if [[ ! -f "${ELF}" ]]; then
    echo "找不到产物: ${ELF}（先运行 bash scripts/build.sh ${APP}）" >&2
    exit 1
fi

echo "拷贝 ${ELF} -> ${BOARD_USER}@${BOARD_IP}:${DEST_DIR}/ ..."
scp "${ELF}" "${BOARD_USER}@${BOARD_IP}:${DEST_DIR}/"

echo
echo "部署完成。板端（msh 终端）运行:"
echo "  msh> ${DEST_DIR}/${APP}"
