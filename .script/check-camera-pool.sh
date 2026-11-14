#!/usr/bin/env bash
# Host regression for the real camera lifecycle with fake SDK calls.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SDK_MPP="${K230_SDK_ROOT:-/mnt/mydata/kRTOSSDK}/src/rtsmart/mpp"
TEST_DIR="$(mktemp -d /tmp/dart-camera-pool-test.XXXXXX)"
CAMERA_DIR="${REPO_ROOT}/include/firmware/src/camera"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    -I"${REPO_ROOT}/include/firmware/src" \
    -I"${SDK_MPP}/include" -I"${SDK_MPP}/include/comm" \
    -I"${SDK_MPP}/include/ioctl" -I"${SDK_MPP}/userapps/api" \
    "${REPO_ROOT}/tests/camera_pool_test.c" \
    "${CAMERA_DIR}/c_api/config.c" "${CAMERA_DIR}/c_api/lease.c" \
    "${CAMERA_DIR}/c_api/raw10.c" "${CAMERA_DIR}/k230/mcm.c" \
    "${CAMERA_DIR}/k230/media.c" "${CAMERA_DIR}/k230/frame.c" \
    "${CAMERA_DIR}/k230/lifecycle.c" "${CAMERA_DIR}/k230/exposure.c" \
    "${CAMERA_DIR}/k230/sensor.c" "${CAMERA_DIR}/k230/sc038hgs.c" \
    "${CAMERA_DIR}/k230/sc132gs.c" "${CAMERA_DIR}/k230/isp.c" \
    -Wl,--wrap=open -Wl,--wrap=close -Wl,--wrap=mmap -Wl,--wrap=munmap \
    -o "${TEST_DIR}/camera_pool_test"
"${TEST_DIR}/camera_pool_test"
