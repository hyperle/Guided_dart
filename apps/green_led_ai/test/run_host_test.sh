#!/usr/bin/env bash
# ============================================================================
# 在**宿主机**上跑检测器正确性测试（不需要板子）
#
# 只编译 detect_color.c 的**标量路径**（宿主机是 x86，没有 RVV），验证：
#   1) 纯灰画面 -> 不命中
#   2) 灰底 + 半径 20 的绿灯（中心 320,240）-> 命中且中心误差 <= 1px
#   3) 全画面扫描(step 1) 与 ROI+step2 结果一致
#   4) 全画面逐像素扫描的纯检测耗时（仅作量级参考，板上是 RVV 路径）
# 另跑 postproc.c（YOLOv8 解码 / letterbox 反算 / NMS / 选目标）与 rec_dir.c
# （录像会话编号 + 目录容量淘汰，这段会真的删文件，所以重点验证删除顺序与成对删除）的单元测试。
#
# 板上 RVV 路径与标量路径的逐位一致性由应用**开机自检**保证
# （日志里的 `selftest rvv-vs-scalar PASS`），因为宿主机没有 RISC-V 模拟器。
#
# 用法: bash apps/green_led_ai/test/run_host_test.sh
# ============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
SDK_ROOT="${K230_SDK_ROOT:-/mnt/mydata/kRTOSSDK}"
APPDIR="${REPO_ROOT}/apps/green_led_ai"
OUTDIR="${REPO_ROOT}/build/host_test"

if [[ ! -d "${SDK_ROOT}/src/rtsmart/mpp/include" ]]; then
    echo "找不到 SDK: ${SDK_ROOT}（用 K230_SDK_ROOT=... 指定）" >&2
    exit 1
fi

mkdir -p "${OUTDIR}"

INC=(-I"${APPDIR}"
     -I"${SDK_ROOT}/src/rtsmart/mpp/include"
     -I"${SDK_ROOT}/src/rtsmart/mpp/include/comm"
     -I"${SDK_ROOT}/src/rtsmart/mpp/userapps/api"
     -include "${SDK_ROOT}/include/generated/autoconf.h")

echo "=== 1/3 颜色检测器（detect_color.c，标量路径） ==="
gcc -O2 -o "${OUTDIR}/detector_host_test" "${INC[@]}" \
    "${APPDIR}/test/host_stub.c" \
    "${APPDIR}/test/detector_host_test.c" \
    "${APPDIR}/detect_color.c" \
    -lm
"${OUTDIR}/detector_host_test"

echo
echo "=== 2/3 后处理解码/NMS（postproc.c，纯逻辑） ==="
gcc -O2 -o "${OUTDIR}/postproc_host_test" "${INC[@]}" \
    "${APPDIR}/test/postproc_host_test.c" \
    "${APPDIR}/postproc.c" \
    -lm
"${OUTDIR}/postproc_host_test"

echo
echo "=== 3/3 录像目录管理（rec_dir.c：会话编号 + 容量淘汰，会真的删文件） ==="
gcc -O2 -o "${OUTDIR}/recdir_host_test" \
    -I"${APPDIR}" \
    "${APPDIR}/test/recdir_host_test.c" \
    "${APPDIR}/rec_dir.c"
"${OUTDIR}/recdir_host_test"
