#!/usr/bin/env bash
# 将交叉编译产物经 GIO/MTP 写入板端 /sdcard/app，保留日志、录像和旧文件备份。
# 用法：bash .script/deploy-mtp.sh <存储根 URI 或 app URI> [--only launcher|dart_app|pwm_swing|imu_raw_probe|list]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=.script/_rtos-common.sh
source "${SCRIPT_DIR}/_rtos-common.sh"
DEST="${1:-}"
[[ -n "${DEST}" ]] || { echo '需要 gphoto2:// 或 mtp:// 存储 URI' >&2; exit 2; }
shift
ONLY=''
while [[ $# -gt 0 ]]; do
    case "$1" in
        --only) shift; ONLY="${1:-}" ;;
        --only=*) ONLY="${1#--only=}" ;;
        --keep-logs) ;; # MTP 部署始终保留日志和录像。
        *) echo "未知选项：$1" >&2; exit 2 ;;
    esac
    shift
done
case "${ONLY}" in ''|launcher|dart_app|pwm_swing|imu_raw_probe|list) ;; *) echo '无效的 --only 目标' >&2; exit 2 ;; esac
case "${DEST}" in gphoto2://*|mtp://*) ;; *) echo '需要 gphoto2:// 或 mtp:// 存储 URI' >&2; exit 2 ;; esac
DEST="${DEST%/}"
# 与统一入口共用分区和应用目录解析；直接调用也会预检。
BOARD_IP=''
SDCARD="${DEST}"
rtos_resolve_transport
APP="${RTOS_DIR}"
if [[ -z "${ONLY}" ]]; then
    rtos_resolve_data_transport
fi
command -v sha256sum >/dev/null || die '找不到 sha256sum'

NAMES=()
SOURCES=()
for name in launcher dart_app imu_raw_probe list pwm_swing; do
    [[ "${name}" != pwm_swing || "${ONLY}" == pwm_swing ]] || continue
    [[ -z "${ONLY}" || "${ONLY}" == "${name}" ]] || continue
    if [[ "${name}" == list ]]; then
        NAMES+=(startup_final.list)
        SOURCES+=("${REPO_ROOT}/runtime/launcher/startup.list")
    else
        NAMES+=("${name}")
        SOURCES+=("$(rtos_artifact_path "${name}")")
    fi
done
for src in "${SOURCES[@]}"; do
    [[ -s "${src}" ]] || die "缺少非空产物：${src}"
done

# 不经 /run/user/.../gvfs 的 FUSE 写入，直接使用 URI 交给 GIO。
# gphoto2 后端不支持 chmod，上传成功后通过完整回读校验内容。
remote_hash() { gio cat "$1" | sha256sum | cut -d' ' -f1; }
local_hash() { sha256sum "$1" | cut -d' ' -f1; }
verify() { [[ "$(local_hash "$1")" == "$(remote_hash "$2")" ]]; }
ensure_dir() { gio info "$1" >/dev/null 2>&1 || gio mkdir "$1"; }
ensure_dir "${APP}"

BACKUP_DIR="$(mktemp -d /tmp/dart-mtp-backup.XXXXXX)"
HAD_OLD=()
cleanup() {
    local uri
    for uri in "${STAGED[@]}"; do gio remove "${uri}" >/dev/null 2>&1 || true; done
}
trap cleanup EXIT
printf '目标：%s（板端 %s）\n旧文件备份：%s\n' "${APP}" "${BOARD_DIR}" "${BACKUP_DIR}"

# 当前 gvfsd-gphoto2 的 gio move/rename 会误报 -103（即使目标不存在），
# 用 gio copy 覆盖已有文件又会写出 0 字节。可靠序列只有：
#   删除目标名 -> 从本地上传到该名字 -> 完整回读校验
# 因此不再上传 .upload-* 临时文件；同名临时文件也会让紧接着的 copy 误报 -103。
# 重试一次，覆盖后端偶尔残留的旧目录项。
replace_file() {
    local src=$1 dst=$2
    gio remove -f "${dst}" >/dev/null 2>&1 || true
    gio copy -T "${src}" "${dst}" && return 0
    gio remove -f "${dst}" >/dev/null 2>&1 || true
    gio copy -T "${src}" "${dst}"
}

# 先备份所有旧文件；用 gio cat 读回，避免 gio copy 到本地污染后端缓存。
for i in "${!NAMES[@]}"; do
    dst="${APP}/${NAMES[i]}"
    if gio info "${dst}" >/dev/null 2>&1; then
        mkdir -p "${BACKUP_DIR}/$(dirname "${NAMES[i]}")"
        gio cat "${dst}" > "${BACKUP_DIR}/${NAMES[i]}"
        verify "${BACKUP_DIR}/${NAMES[i]}" "${dst}" || die "旧文件备份校验失败：${NAMES[i]}"
        HAD_OLD+=(1)
    else
        HAD_OLD+=(0)
    fi
done

# 再逐个替换；任何一步失败都先删掉坏文件，再用主机备份恢复。
for i in "${!NAMES[@]}"; do
    dst="${APP}/${NAMES[i]}"
    if ! replace_file "${SOURCES[i]}" "${dst}"; then
        gio remove -f "${dst}" >/dev/null 2>&1 || true
        if [[ "${HAD_OLD[i]}" == 1 ]]; then
            replace_file "${BACKUP_DIR}/${NAMES[i]}" "${dst}" \
                || die "替换和恢复均失败；主机备份位于 ${BACKUP_DIR}"
        fi
        die "替换失败：${NAMES[i]}；主机备份位于 ${BACKUP_DIR}（如提示 -103，请重插 USB 或重新 gio mount）"
    fi
    if ! verify "${SOURCES[i]}" "${dst}"; then
        gio remove -f "${dst}" >/dev/null 2>&1 || true
        if [[ "${HAD_OLD[i]}" == 1 ]]; then
            replace_file "${BACKUP_DIR}/${NAMES[i]}" "${dst}" \
                || die "恢复失败；主机备份位于 ${BACKUP_DIR}"
        fi
        die "最终校验失败：${NAMES[i]}；主机备份位于 ${BACKUP_DIR}"
    fi
    printf '  %s：%s bytes，SHA-256 %s\n' "${NAMES[i]}" \
        "$(stat -c %s "${SOURCES[i]}")" "$(local_hash "${SOURCES[i]}")"
done

if [[ -z "${ONLY}" ]]; then
    ensure_dir "${RTOS_DATA_DIR}"
    ensure_dir "${RTOS_DATA_DIR}/logs"
    ensure_dir "${RTOS_DATA_DIR}/records"
fi
printf '\n部署完成；日志和录像已保留。旧文件备份：%s\n' "${BACKUP_DIR}"
if [[ "${ONLY}" == pwm_swing ]]; then
    printf '板端手动运行：%s/pwm_swing（约 43 秒，会驱动四路舵机）\n' "${BOARD_DIR}"
    exit 0
fi
if [[ "${ONLY}" == imu_raw_probe ]]; then
    printf '首次启用还需 deploy-rtos list；重启后自动读取。手动运行：%s/imu_raw_probe\n' "${BOARD_DIR}"
    exit 0
fi
printf '板端验证：%s/launcher -t\n' "${BOARD_DIR}"
printf '注意：写入不会重启当前进程；startup_final.list 需重启启动器或板子后加载。\n'
