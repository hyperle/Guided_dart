#!/usr/bin/env bash
# ============================================================================
# Recorder 主机侧回归测试：x86 上用桩 MPP 跑 recorder/frame_pool/mpp_map 真代码。
#
#   bash tests/host/run.sh
#
# 为什么要它：这几轮的失败（VB 块漏光、逐包 mmap/munmap、在途块复用、拆机顺序）
# 全都只在板端才暴露，而板端一次验证要拔卡、上电、再拔卡。逻辑先在主机上咬死，
# 上板只验证"SDK/硬件行为"，两边的问题就不会互相污染。
# ============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SDK="${K230_SDK_ROOT:-/mnt/mydata/kRTOSSDK}"
BUILD="${REPO_ROOT}/build/host_test"
mkdir -p "${BUILD}"

INC=(
    -I"${REPO_ROOT}/include"
    -I"${SDK}/include"
    -I"${SDK}/src/rtsmart/mpp/include"
    -I"${SDK}/src/rtsmart/mpp/include/comm"
    -I"${SDK}/src/rtsmart/mpp/include/ioctl"
    -I"${SDK}/src/rtsmart/mpp/userapps/api"
    -I"${SDK}/src/rtsmart/mpp/userapps/api/framework"
    -include"${SDK}/include/generated/autoconf.h"
)

set -x
g++ -std=gnu++20 -O1 -g -Wall -Wextra \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    "${INC[@]}" \
    "${REPO_ROOT}/tests/host/recorder_host_test.cpp" \
    "${REPO_ROOT}/tests/host/stub_mpp.cpp" \
    "${REPO_ROOT}/src/recording/recorder.cpp" \
    "${REPO_ROOT}/src/vision/frame_pool.cpp" \
    "${REPO_ROOT}/src/core/mpp_map.cpp" \
    "${REPO_ROOT}/src/core/mmz.cpp" \
    "${REPO_ROOT}/src/evidence/evidence_writer.cpp" \
    "${REPO_ROOT}/src/core/log.cpp" \
    -lpthread -o "${BUILD}/recorder_host_test"
set +x

"${BUILD}/recorder_host_test"

# ---------------------------------------------------------------------------
# 识别/跟踪层：合成二值图 + 假时钟，把"启动确认 / 坏点剔除 / 状态机 / 尺度自适应滤波 /
# ROI 门控"整条链在 x86 上跑一遍（不需要 RVV、相机、SDK）。
#   bash tests/host/run.sh detection   # 只跑这一组（改动识别层时用）
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "" || "${1:-}" == "detection" ]]; then
    set -x
    g++ -std=gnu++20 -O1 -g -Wall -Wextra \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"${REPO_ROOT}/include" \
        "${REPO_ROOT}/tests/host/detection_host_test.cpp" \
        "${REPO_ROOT}/src/detection/light.cpp" \
        "${REPO_ROOT}/src/detection/blob_measure.cpp" \
        "${REPO_ROOT}/src/detection/armer.cpp" \
        "${REPO_ROOT}/src/detection/kalman.cpp" \
        "${REPO_ROOT}/src/detection/roi_prediction.cpp" \
        "${REPO_ROOT}/src/detection/tracker.cpp" \
        "${REPO_ROOT}/src/detection/pipeline.cpp" \
        -o "${BUILD}/detection_host_test"
    set +x

    "${BUILD}/detection_host_test"
fi
