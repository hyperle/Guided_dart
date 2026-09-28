#!/usr/bin/env bash
# ============================================================================
# 在 **qemu-riscv64** 上跑识别层的 RVV 路径（上板之前的第一道闸）
#
#   bash scripts/rvv_qemu.sh              # 交叉编译 + qemu 运行 + 判据
#   bash scripts/rvv_qemu.sh --build-only # 只交叉编译（没装 qemu 时也能用）
#
# 补的是哪段空白：主机侧测试跑的是标量参考路径，板端才有真 RVV。中间这段过去只能靠
# "板端开机自检 + 人读反汇编"，错一次要浪费一轮上板（拔卡/上电/再拔卡）。qemu 8.x 支持
# RVV 1.0，可以在这里先把向量路径按板端同样的 VLEN=128 跑一遍。
#
# 装 qemu（不装的话本脚本会明确告诉你，并以 --build-only 的结果收尾）：
#   sudo apt install -y qemu-user                       # → /usr/bin/qemu-riscv64
#   # 想直接运行 riscv64 程序（binfmt 透明执行）再装：
#   sudo apt install -y qemu-user-static binfmt-support && sudo update-binfmts --enable qemu-riscv64
#
# 注意：qemu 的时间**不是板端的时间**（逐指令翻译执行），所以脚本只对耗时做数量级检查，
# 真正的性能判据仍然是板端日志里的 `识别: 状态=… 扫描 avg/max=…us`。
# ============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TC_DIR="${K230_TOOLCHAIN_DIR:-${REPO_ROOT}/third_party/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu}/bin"
CXX="${TC_DIR}/riscv64-unknown-linux-musl-g++"
OUT="${REPO_ROOT}/build/rvv_qemu/rvv_selftest"

# 与板端完全一致的编译参数（-march 里的 v + zvl128b 决定了 VLEN=128 与 RVV 1.0）
ARCH_FLAGS=(-march=rv64imafdcv -mabi=lp64d -mcmodel=medany --static -O2)

if [[ ! -x "${CXX}" ]]; then
    echo "找不到交叉编译器: ${CXX}" >&2
    echo "（先跑 bash scripts/setup-toolchain.sh，或用 K230_TOOLCHAIN_DIR=... 指定）" >&2
    exit 1
fi

mkdir -p "$(dirname "${OUT}")"

echo "=== 1/3 交叉编译（riscv64, RVV 1.0, VLEN=128）==="
set -x
"${CXX}" "${ARCH_FLAGS[@]}" -Wall -Wextra \
    -I"${REPO_ROOT}/include" \
    "${REPO_ROOT}/tests/rvv/rvv_selftest.cpp" \
    "${REPO_ROOT}/src/detection/scanner/tile_scanner.cpp" \
    "${REPO_ROOT}/src/detection/measure/roi_measure.cpp" \
    "${REPO_ROOT}/src/detection/track/blip_confirmer.cpp" \
    "${REPO_ROOT}/src/detection/track/kalman.cpp" \
    "${REPO_ROOT}/src/detection/track/roi_prediction.cpp" \
    "${REPO_ROOT}/src/detection/track/tracker.cpp" \
    "${REPO_ROOT}/src/detection/pipeline.cpp" \
    "${REPO_ROOT}/src/detection/armor/armor_geometry.cpp" \
    "${REPO_ROOT}/src/detection/armor/armor_scale.cpp" \
    "${REPO_ROOT}/src/detection/armor/armor_rules.cpp" \
    "${REPO_ROOT}/src/detection/armor/armor_windows.cpp" \
    "${REPO_ROOT}/src/detection/armor/armor_detector.cpp" \
    -o "${OUT}"
set +x
echo "产物: ${OUT}"

if [[ "${1:-}" == "--build-only" ]]; then
    echo "（--build-only：跳过 qemu 运行）"
    exit 0
fi

echo
echo "=== 2/3 找 qemu-riscv64 ==="
QEMU=""
for cand in qemu-riscv64 qemu-riscv64-static; do
    if command -v "${cand}" >/dev/null 2>&1; then
        QEMU="${cand}"
        break
    fi
done

if [[ -z "${QEMU}" ]]; then
    cat >&2 <<'EOF'
没找到 qemu-riscv64 —— 交叉编译已经通过，但 RVV 路径这一轮没跑起来。
装它（一条命令）：
    sudo apt install -y qemu-user
装完重跑本脚本即可；也可以先 bash scripts/rvv_qemu.sh --build-only 只做编译检查。
EOF
    exit 2
fi
echo "使用: ${QEMU} ($(${QEMU} --version | head -1))"

echo
echo "=== 3/3 qemu 运行（-cpu rv64,v=true,vlen=128，对应 C908 的 VLEN=128）==="
# v=true 打开 RVV；vlen=128 必须显式给（默认 128，但写出来才不会被别处的默认值悄悄改掉）
# 若这版 qemu 要求显式规范版本，补 ,vext_spec=v1.0 即可。
set -x
"${QEMU}" -cpu rv64,v=true,vlen=128 "${OUT}"
set +x

echo
echo "RVV 路径（qemu/riscv64）回归通过；接下来才是上板验证「真 RVV 时序 + 内存带宽」。"
