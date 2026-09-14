#!/usr/bin/env bash
# ============================================================================
# 把构建产物直接拷进**已挂载的 SD 卡**（板子拔卡、在电脑上改的场景）
#
# 适用原因（2025-09 实测，别再用 MTP 覆盖文件）：
#   板端 MTP（gphoto2/gvfs）覆盖已存在文件时会**先把文件删掉、再写不回来**：
#   实测 deploy 之后 startup_final.list 直接从卡上消失（cp 仍返回 0），
#   紧接着整个 gphoto2 挂载点掉了（gvfsd-gphoto2 进程消失、gio mount 报
#   "找不到匹配的 udev 设备"），既读不到也写不了，只能重插 USB 或拔卡。
#   结论：**新增文件**可以用 MTP 传，但**覆盖/替换已有文件一律走读卡器**
#   （把卡挂到电脑上，用本脚本写）。
# 用读卡器时先卸载再插卡，避免挂载点陈旧：
#   udisksctl unmount -b /dev/sdX2 && udisksctl mount -b /dev/sdX2
#
# 用法:
#   bash scripts/deploy-sdcard.sh /media/$USER/<卡>          # 卡根目录
#   bash scripts/deploy-sdcard.sh /media/$USER/<卡>/app      # 直接给 app 目录也行
#
# 做的事（板端布局：/sdcard/app/ 就是卡上的 app/）:
#   app/launcher            <- build/launcher/launcher
#   app/green_led_ai        <- build/green_led_ai/green_led_ai   （视觉+录像主体，color 后端）
#   app/green_led_ai_kpu    <- build/green_led_ai_kpu/...        （含 kpu 后端，需 kmodel）
#   app/board_probe         <- build/board_probe/board_probe     （板端环境探针）
#   app/kpu_bench           <- build/kpu_bench/kpu_bench         （AI2D+KPU 计时器，含 nncase）
#   app/best.kmodel         <- SDK 示例模型（kpu_bench 冒烟测试用，配置好真模型后可删）
#   app/startup_final.list  <- apps/launcher/startup.list（launcher 读的清单）
#   app/usb_cam_stream      -> 删除（历史上临时占位用的名字）
#   并清掉 app/logs/*.log、app/recording/*（可选，见 --keep-logs）
# ============================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-build}"
KEEP_LOGS=0
DEST=""
for arg in "$@"; do
    case "${arg}" in
    --keep-logs) KEEP_LOGS=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) DEST="${arg}" ;;
    esac
done

if [[ -z "${DEST}" ]]; then
    echo "用法: bash scripts/deploy-sdcard.sh <SD卡挂载点> [--keep-logs]" >&2
    exit 1
fi
if [[ ! -d "${DEST}" ]]; then
    echo "找不到目录: ${DEST}" >&2
    exit 1
fi

# 定位板端的 /sdcard/app
if [[ -d "${DEST}/app" ]]; then
    APPDIR="${DEST}/app"
elif [[ "$(basename "${DEST}")" == "app" ]]; then
    APPDIR="${DEST}"
else
    APPDIR="${DEST}/app"
    mkdir -p "${APPDIR}"
fi
echo "目标 app 目录: ${APPDIR}"

copy_art() {
    local name="$1"
    local src="${REPO_ROOT}/${BUILD_DIR}/${name}/${name}"
    if [[ ! -f "${src}" ]]; then
        echo "缺少产物 ${src}（先 bash scripts/build.sh ${name}）" >&2
        return 1
    fi
    cp -f "${src}" "${APPDIR}/${name}"
    echo "  ${name}  <- ${src}"
}

echo "拷贝二进制产物:"
copy_art launcher
copy_art green_led_ai
if [[ -f "${REPO_ROOT}/${BUILD_DIR}/green_led_ai_kpu/green_led_ai_kpu" ]]; then
    copy_art green_led_ai_kpu
else
    echo "  green_led_ai_kpu 未编译，跳过（bash scripts/build.sh green_led_ai_kpu）"
fi
# 探针是调试工具，编译过就一起带上（没有则跳过，不影响部署）
if [[ -f "${REPO_ROOT}/${BUILD_DIR}/board_probe/board_probe" ]]; then
    copy_art board_probe
else
    echo "  board_probe 未编译，跳过（bash scripts/build.sh board_probe 可单独编）"
fi
if [[ -f "${REPO_ROOT}/${BUILD_DIR}/kpu_bench/kpu_bench" ]]; then
    copy_art kpu_bench
else
    echo "  kpu_bench 未编译，跳过（bash scripts/build.sh kpu_bench 可单独编）"
fi

# kpu_bench 冒烟测试用的示例模型（SDK 自带的 YOLOv8 三分类模型）
_KM="${K230_SDK_ROOT:-/mnt/mydata/kRTOSSDK}/src/rtsmart/examples/ai/usage_kpu/utils/best.kmodel"
if [[ -f "${_KM}" ]]; then
    cp -f "${_KM}" "${APPDIR}/best.kmodel"
    echo "  best.kmodel  <- ${_KM}"
fi

cp -f "${REPO_ROOT}/apps/launcher/startup.list" "${APPDIR}/startup_final.list"
echo "  startup_final.list  <- apps/launcher/startup.list"

# 清掉过去的临时占位名字（清单里不该再出现它）
if [[ -e "${APPDIR}/usb_cam_stream" ]]; then
    rm -f "${APPDIR}/usb_cam_stream"
    echo "  删除 usb_cam_stream（历史临时占位）"
fi

if [[ "${KEEP_LOGS}" == "0" ]]; then
    rm -f "${APPDIR}"/logs/*.log 2>/dev/null || true
    rm -f "${APPDIR}"/recording/* 2>/dev/null || true
    echo "  清空 logs/*.log 与 recording/*（--keep-logs 可保留）"
fi

mkdir -p "${APPDIR}/logs" "${APPDIR}/recording"
sync

echo
echo "清单里会启动的项:"
grep -vE '^\s*#|^\s*$' "${APPDIR}/startup_final.list" | sed 's/^/  /'
echo
echo "校验（应与本地构建一致）:"
md5sum "${APPDIR}/launcher" "${APPDIR}/green_led_ai" \
       "${REPO_ROOT}/${BUILD_DIR}/launcher/launcher" \
       "${REPO_ROOT}/${BUILD_DIR}/green_led_ai/green_led_ai"
echo
echo "完成后插卡回板子、上电即可；日志在 app/logs/，录像在 app/recording/。"
