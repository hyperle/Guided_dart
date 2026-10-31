#!/usr/bin/env python3
"""Build a CanMV interpreter with the app's RAW camera; never modify the SDK."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
BOARD = "k230d_rtos_dart_v1"
CONFIG = "k230d_canmv_dart_v1_defconfig"


def build(args):
    sdk = args.sdk.resolve()
    destination = args.build_dir.resolve()
    if any(char.isspace() for char in str(sdk) + str(destination) + str(ROOT)):
        raise SystemExit("SDK/checkout/build paths must not contain whitespace (CanMV Makefile)")
    port = sdk / "src/canmv/port"
    original_build = sdk / "output" / CONFIG
    if not (original_build / "canmv/micropython").is_file():
        raise SystemExit("Build the Dart V1 MicroPython SDK once before running this helper")
    if f'CONFIG_BOARD_CONFIG_NAME="{CONFIG}"' not in (sdk / ".config").read_text():
        raise SystemExit(f"SDK .config must select {CONFIG}")
    abi = json.loads((ROOT / "include/firmware/src/camera/k230/mcm_abi.json").read_text())
    for relative, expected in abi.items():
        library = sdk / "src/rtsmart/mpp" / relative
        if hashlib.sha256(library.read_bytes()).hexdigest() != expected:
            raise SystemExit(f"Audited RAW MCM library changed: {library}")

    toolchain = args.toolchain_bin.resolve()
    compiler = toolchain / "riscv64-unknown-linux-musl-gcc"
    if not compiler.is_file():
        raise SystemExit(f"Missing compiler: {compiler}")
    destination.mkdir(parents=True, exist_ok=True)
    stage = destination / "source"
    staged_port = stage / "port"
    staged_port.mkdir(parents=True, exist_ok=True)
    # Source overlay: build tools may touch core/main.c, so keep core and the
    # target board private. Network sources are private for optional socket fixes.
    # Every other source is only read through a symlink.
    for source in port.iterdir():
        target = staged_port / source.name
        if source.name in ("boards", "core"):
            continue
        if source.name == "network":
            if target.is_symlink():
                target.unlink()
            shutil.copytree(source, target, dirs_exist_ok=True)
        elif source.name == "Makefile":
            shutil.copy2(source, target)
        elif not target.exists():
            target.symlink_to(source, target_is_directory=source.is_dir())
    if getattr(args, "wifi_socket_fix", False):
        subprocess.run(["patch", "--batch", "--fuzz=0", "-p1", "-i",
                        str(ROOT / "tools/wifi/network-socket-fix.patch")],
                       cwd=staged_port / "network", check=True)
    # Version probes must use the SDK's CanMV repository, not the workspace
    # containing this isolated overlay (the IDE also reads these strings).
    makefile = staged_port / "Makefile"
    makefile.write_text(makefile.read_text().replace(
        "$(shell git ", '$(shell git -C "$(SDK_CANMV_SRC_DIR)" ').replace(
        "CANMV_SRC_CXX += $(wildcard kpu/*.cpp)",
        "CANMV_SRC_CXX += $(wildcard $(BOARD_DIR)/*.cpp)\n"
        "CANMV_SRC_CXX += $(wildcard kpu/*.cpp)"))
    shutil.copytree(port / "core", staged_port / "core", dirs_exist_ok=True)
    staged_boards = staged_port / "boards"
    staged_boards.mkdir(exist_ok=True)
    for source in (port / "boards").iterdir():
        target = staged_boards / source.name
        if source.name == BOARD:
            shutil.copytree(source, target, dirs_exist_ok=True)
        elif not target.exists():
            target.symlink_to(source, target_is_directory=source.is_dir())
    for name in ("micropython", "tools"):
        target = stage / name
        if not target.exists():
            target.symlink_to(sdk / "src/canmv" / name, target_is_directory=True)

    board = staged_boards / BOARD
    for name in ("moddart_raw.c", "moddart_detect.c", "detector_bridge.h"):
        shutil.copy2(HERE / name, board / name)
    # The interpreter's GCC may predate named-module support. Compile the real
    # detector with the application's module toolchain and link its C ABI archive.
    # Remove only the obsolete generated source from older private overlays.
    (board / "detector_bridge.cpp").unlink(missing_ok=True)
    module_build = destination / "detector-modules"
    subprocess.run([
        "cmake", "-S", str(ROOT), "-B", str(module_build), "-G", "Ninja",
        f"-DCMAKE_TOOLCHAIN_FILE={ROOT / 'cmake/toolchain-riscv64-musl.cmake'}",
        f"-DK230_TOOLCHAIN_DIR={toolchain}", f"-DK230_SDK_ROOT={sdk}",
        "-DDART_CAMERA_K230=OFF", "-DDART_IMU_K230=OFF", "-DDART_PWM_K230=OFF",
        "-DDART_CAMERA_CLANGD_COMMANDS=OFF", "-DDART_IMU_CLANGD_COMMANDS=OFF",
        "-DBUILD_TESTING=OFF",
    ], check=True)
    subprocess.run(["cmake", "--build", str(module_build), "--target",
                    "dart_canmv_detector", "--parallel", str(args.jobs)], check=True)
    detector_archives = [module_build / "src/libdart_canmv_detector.a",
                         module_build / "src/libdart_detection.a",
                         module_build / "include/libdart_camera_types.a"]

    # Recompile the actual app sources, not a second implementation of exposure,
    # unpacking, cutoff, cache handling, or DMA ownership.
    camera = ROOT / "include/firmware/src/camera"
    sources = ("c_api/config.c", "c_api/lease.c", "c_api/raw10.c", "k230/lifecycle.c",
               "k230/frame.c", "k230/exposure.c", "k230/mcm.c")
    for source in sources:
        name = "dart_" + source.replace("/", "_")
        (board / name).write_text(f'#include "{camera / source}"\n')

    # CanMV already initializes global VB before the Python VM starts. Borrow
    # that owner; keep the backend's unchanged camera/pool attach and cleanup.
    (board / "dart_media.c").write_text(
        '#define dart_media_create dart_standalone_media_create\n'
        '#define dart_media_destroy dart_standalone_media_destroy\n'
        f'#include "{camera / "k230/media.c"}"\n'
        '#undef dart_media_create\n#undef dart_media_destroy\n'
        'int dart_media_create(dart_media_t **media) {\n'
        '  if (media == NULL) return DART_CAMERA_E_INVALID_ARGUMENT;\n'
        '  *media = NULL;\n'
        '  if (media_owner != NULL) return DART_CAMERA_E_BUSY;\n'
        '  dart_media_t *value = calloc(1, sizeof(*value));\n'
        '  if (value == NULL) return DART_CAMERA_E_NO_MEMORY;\n'
        '  value->vb_initialized = 1;\n'
        '  *media = media_owner = value;\n'
        '  return DART_CAMERA_OK;\n}\n'
        'int dart_media_destroy(dart_media_t *media) {\n'
        '  if (media == NULL || media != media_owner) return DART_CAMERA_E_INVALID_ARGUMENT;\n'
        '  if (media->camera != NULL) return DART_CAMERA_E_BUSY;\n'
        '  media_owner = NULL; free(media); return DART_CAMERA_OK;\n}\n'
    )
    (board / "mpconfigport.mk").write_text(
        f'CFLAGS_EXTRA += -I{ROOT / "include/firmware/src"}\n'
        'LDFLAGS_EXTRA += -Wl,--wrap=ioctl\n'
        f'INC += -I{ROOT} -I{ROOT / "include/firmware/src"}\n'
        'LDFLAGS_EXTRA += -Wl,--start-group ' + " ".join(map(str, detector_archives))
        + ' -Wl,--end-group\n'
    )
    (board / "manifest.py").write_text(
        f'include("{port / "boards/manifest.py"}")\n'
        f'module("dart_board.py", base_path="{sdk / "boards" / BOARD / "micropython"}")\n'
    )

    output = destination / "canmv"
    if not output.exists():
        shutil.copytree(original_build / "canmv", output)
    # Rewrite both targets and prerequisites, including the SDK's double slashes.
    # Otherwise old objects would miss qstr changes and retain invalid string IDs.
    for dependency in output.rglob("*.P"):
        text = re.sub(r"/{2,}", "/", dependency.read_text())
        prefixes = {
            f"/workspace/output/{CONFIG}/canmv": str(output),
            str(original_build / "canmv"): str(output),
            "/workspace": str(sdk),
            "/opt/toolchains/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu":
                str(toolchain.parent),
        }
        # Match complete path prefixes once. A second build must not replace
        # '/workspace' inside this checkout's '/mnt/mydata/workspace/...' path.
        pattern = r"(?<![\w/.-])(?:" + "|".join(
            re.escape(prefix) for prefix in sorted(prefixes, key=len, reverse=True)
        ) + r")(?=/|[\s:]|$)"
        text = re.sub(pattern, lambda match: prefixes[match.group()], text)
        dependency.write_text(text)
    # These files embed make-time version strings; make does not otherwise
    # track a command-line macro change when reusing the SDK's object files.
    for relative in ("core/main.o", "mp_modules/overlay_modos.o", "omv/ide_dbg.o"):
        (output / relative).unlink(missing_ok=True)
    # This SDK's makemanifest checks only .mpy timestamps and otherwise reuses
    # frozen_content.c even when native qstrs have changed. Regenerate its extra
    # qstr pool too, or the same spelling can acquire two IDs and dict lookup
    # from Python can fail for keys introduced by a new native module.
    (output / "frozen_content.c").unlink(missing_ok=True)
    command = [
        "make", "-C", str(staged_port), f"-j{args.jobs}",
        f"SDK_SRC_ROOT_DIR={sdk}", f"SDK_CANMV_SRC_DIR={sdk / 'src/canmv'}",
        f"SDK_CANMV_BUILD_DIR={output}", f"SDK_RTSMART_SRC_DIR={sdk / 'src/rtsmart'}",
        f"SDK_RTSMART_BUILD_DIR={original_build / 'rtsmart'}",
        f"CROSS_COMPILE={toolchain}/riscv64-unknown-linux-musl-",
        f"MPY_CROSS={sdk / 'src/canmv/micropython/mpy-cross/build/mpy-cross'}",
    ]
    subprocess.run(command, check=True)
    native_qstrs = set(re.findall(r"QDEF\((MP_QSTR_[^,]+),",
                                  (output / "genhdr/qstrdefs.generated.h").read_text()))
    frozen_enum = re.search(r"enum \{(.*?)\n\};", (output / "frozen_content.c").read_text(), re.S)
    if frozen_enum:
        duplicates = native_qstrs.intersection(re.findall(r"MP_QSTR_\w+", frozen_enum.group(1)))
        if duplicates:
            raise SystemExit("Stale frozen qstr pool: " + ", ".join(sorted(duplicates)))
    artifact = destination / "micropython"
    shutil.copy2(output / "micropython", artifact)
    shutil.copy2(ROOT / "tools/canmv_tune.py", destination / "canmv_tune.py")
    metadata = {
        "board": BOARD,
        "camera_abi": 6,
        "pixel_format": "gray8",
        "gray_feature_version": 1,
        "track_feature_version": 1,
        "camera_path": "native app C backend; CanMV borrows VM-owned VB",
        "hardware_tested": False,
        "wifi_socket_fix": bool(getattr(args, "wifi_socket_fix", False)),
        "sdk": str(sdk),
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sorted(HERE.glob("*.[ch]")) + sorted(HERE.glob("*.cpp"))
                    + [ROOT / "tools/canmv_tune.py"] + sorted((ROOT / "src/detector").glob("*.cpp"))
                    + sorted(camera.rglob("*.c")) + sorted(camera.rglob("*.h"))},
        "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
    }
    (destination / "BUILD_INFO.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Built {artifact}\nSHA256 {metadata['sha256']}")
    print("Stage with tools/canmv_tuning/stage.py, activate by rename, then hard reboot (see README).")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, default=Path("/mnt/mydata/kRTOSSDK"))
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build/canmv-tuning")
    parser.add_argument("--toolchain-bin", type=Path, default=Path(os.environ.get(
        "K230_TOOLCHAIN_DIR",
        "/mnt/mydata/SelfGuidingDart/third_party/toolchain/"
        "riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin")))
    parser.add_argument("--wifi-socket-fix", action="store_true",
                        help="fix RT-Smart nonblocking TCP EOF/connect semantics in the private overlay")
    parser.add_argument("--jobs", type=int, default=8)
    build(parser.parse_args())


if __name__ == "__main__":
    main()
