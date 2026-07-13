#!/usr/bin/env bash
set -euo pipefail

RULE_PATH="/etc/udev/rules.d/99-openmv-sdcard-protect.rules"
LEGACY_RO_RULE="/etc/udev/rules.d/99-openmv-sdcard-ro.rules"
BLOCKDEV="/usr/sbin/blockdev"

if [[ "${EUID}" -ne 0 ]]; then
    echo "run with sudo: sudo $0" >&2
    exit 1
fi

if [[ ! -x "${BLOCKDEV}" ]]; then
    echo "missing ${BLOCKDEV}" >&2
    exit 1
fi

tmp_rule="$(mktemp)"
trap 'rm -f "${tmp_rule}"' EXIT

cat >"${tmp_rule}" <<EOF
# Protect this OpenMV SD-card mass-storage device on this host.
# udisks must ignore it, so desktop/file-manager automount cannot touch FAT.
# If the block device still appears, force it read-only as an extra guard.
ACTION=="add|change", SUBSYSTEM=="block", ENV{DEVTYPE}=="disk", ENV{ID_BUS}=="usb", ENV{ID_VENDOR_ID}=="37c5", ENV{ID_MODEL_ID}=="1204", ENV{ID_MODEL}=="pyboard_SDCard", ENV{ID_SERIAL_SHORT}=="365A396B3230", ENV{UDISKS_IGNORE}="1"
ACTION=="add|change", SUBSYSTEM=="block", ENV{DEVTYPE}=="disk", ENV{ID_BUS}=="usb", ENV{ID_VENDOR_ID}=="37c5", ENV{ID_MODEL_ID}=="1204", ENV{ID_MODEL}=="pyboard_SDCard", ENV{ID_SERIAL_SHORT}=="365A396B3230", RUN+="${BLOCKDEV} --setro /dev/%k"
EOF

install -m 0644 "${tmp_rule}" "${RULE_PATH}"
echo "installed ${RULE_PATH}"

if [[ -e "${LEGACY_RO_RULE}" && "${LEGACY_RO_RULE}" != "${RULE_PATH}" ]]; then
    disabled_path="${LEGACY_RO_RULE}.disabled.$(date +%Y%m%d%H%M%S)"
    mv "${LEGACY_RO_RULE}" "${disabled_path}"
    echo "disabled legacy read-only automount rule: ${disabled_path}"
fi

udevadm control --reload-rules
udevadm trigger --subsystem-match=block
echo "reloaded udev rules"
echo "unplug and replug OpenMV, then verify that findmnt prints no mount for the SD card"
