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
        "${REPO_ROOT}/src/detection/armor.cpp" \
        "${REPO_ROOT}/src/detection/kalman.cpp" \
        "${REPO_ROOT}/src/detection/roi_prediction.cpp" \
        "${REPO_ROOT}/src/detection/tracker.cpp" \
        "${REPO_ROOT}/src/detection/pipeline.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_geometry.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_scale.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_rules.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_windows.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_detector.cpp" \
        -o "${BUILD}/detection_host_test"
    set +x

    "${BUILD}/detection_host_test"
fi

# ---------------------------------------------------------------------------
# 装甲板识别（第二路输出）：几何（交叉连线求交）、变阈值门限、远档（3x1px 灯条）、
# 检测-跟踪（关联优先/保持/释放）、与既有测量器同口径。
#   bash tests/host/run.sh armor       # 只跑这一组（改装甲板那一层时用）
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "" || "${1:-}" == "armor" ]]; then
    set -x
    g++ -std=gnu++20 -O1 -g -Wall -Wextra \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"${REPO_ROOT}/include" \
        "${REPO_ROOT}/tests/host/armor_host_test.cpp" \
        "${REPO_ROOT}/src/detection/blob_measure.cpp" \
        "${REPO_ROOT}/src/detection/light.cpp" \
        "${REPO_ROOT}/src/detection/armor.cpp" \
        "${REPO_ROOT}/src/detection/kalman.cpp" \
        "${REPO_ROOT}/src/detection/roi_prediction.cpp" \
        "${REPO_ROOT}/src/detection/tracker.cpp" \
        "${REPO_ROOT}/src/detection/pipeline.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_geometry.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_scale.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_rules.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_windows.cpp" \
        "${REPO_ROOT}/src/detection/armor/armor_detector.cpp" \
        -o "${BUILD}/armor_host_test"
    set +x

    "${BUILD}/armor_host_test"
fi

# ---------------------------------------------------------------------------
# 控制层（滑模）：状态机 / 视觉外环 PD 滑模 / 姿态内环滑模 / 控制分配 → 归一化舵偏。
# 把"PD 滑模的带内带外、逐通道像素符号、能量保护、离散化约束、分配矩阵不变量、
# 闭环交战、失效路径"整条链在 x86 上跑一遍（不需要 IMU、相机、SDK）。
#   bash tests/host/run.sh control   # 只跑这一组（改动控制层时用）
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "" || "${1:-}" == "control" ]]; then
    set -x
    g++ -std=gnu++20 -O1 -g -Wall -Wextra \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"${REPO_ROOT}/include" \
        "${REPO_ROOT}/tests/host/control_host_test.cpp" \
        "${REPO_ROOT}/src/control/flight_state_machine.cpp" \
        "${REPO_ROOT}/src/control/visual_outer_loop.cpp" \
        "${REPO_ROOT}/src/control/attitude_inner_loop.cpp" \
        "${REPO_ROOT}/src/control/control_allocator.cpp" \
        "${REPO_ROOT}/src/control/flight_controller.cpp" \
        -o "${BUILD}/control_host_test"
    set +x

    "${BUILD}/control_host_test"
fi

# ---------------------------------------------------------------------------
# 联合 EKF（自身运动 + 像平面残差）：状态分块 / 雅可比 / 互协方差 / 锚定语义 /
# 噪声自洽（ΣNIS）/ 与旧两级架构的对照 / 长跑与出框保护。
# 测试直接 #include 被测 .cpp（本模块没有头文件），见 CONTROL_TODO.md 的构建接线一条。
#   bash tests/host/run.sh predictor   # 只跑这一组（改观测器时用）
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "" || "${1:-}" == "predictor" ]]; then
    set -x
    g++ -std=gnu++20 -O1 -g -Wall -Wextra \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        "${REPO_ROOT}/tests/host/predictor_host_test.cpp" \
        -o "${BUILD}/predictor_host_test"
    set +x

    "${BUILD}/predictor_host_test"
fi
