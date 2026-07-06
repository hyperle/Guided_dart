#!/usr/bin/env python3
from __future__ import annotations

import argparse
import configparser
import json
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

CTRL_C = b"\x03"
CTRL_D = b"\x04"
RAW_REPL_ENTER = b"\x01"
RAW_REPL_EXIT = b"\x02"
RAW_REPL_PROMPT = b"raw REPL; CTRL-B to exit\r\n>"
FLASH_REMOTE_ROOT = PurePosixPath("/flash")
REPO_ROOT = Path(__file__).resolve().parents[2]
HOST_GUIDANCE_DIR = REPO_ROOT / "Host_tools" / "guidance"
GUIDANCE_PARAMS_PATH = HOST_GUIDANCE_DIR / "guidance_params.yaml"
GENERATED_PARAMS_MODULE = PurePosixPath("generated_guidance_params.py")


@dataclass(frozen=True)
class ProjectConfig:
    project_dir: Path
    src_dir: Path
    entry_script: Path
    build_dir: Path
    remote_manifest: PurePosixPath
    serial_port: str
    serial_baudrate: int
    sdcard_root: Path | None
    stlink_openocd: str
    stlink_interface_cfg: str
    stlink_target_cfg: str
    stlink_transport: str
    stlink_firmware_image: Path | None
    stlink_firmware_address: str


@dataclass(frozen=True)
class Artifact:
    source_path: Path
    source_relative: PurePosixPath
    local_path: Path
    remote_path: PurePosixPath
    kind: str


class DirectRawReplSession:
    def __init__(self, port: str, baudrate: int):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required: python3 -m pip install -r OpenMV_guidance/requirements.txt"
            ) from exc

        self._serial_mod = serial
        self._device_path = port
        self._baudrate = int(baudrate)
        self._port = None
        self._hard_reset_requested = False

    def __enter__(self) -> "DirectRawReplSession":
        self._port = self._serial_mod.Serial(self._device_path, self._baudrate, timeout=0.1)
        self._enter_raw_repl()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            if self._port is not None and not self._hard_reset_requested:
                self._write_all(RAW_REPL_EXIT)
                time.sleep(0.1)
        finally:
            if self._port is not None:
                self._port.close()
                self._port = None

    def _write_all(self, payload: bytes) -> None:
        if self._port is None:
            raise RuntimeError("serial port is not open")
        self._port.write(payload)
        self._port.flush()

    def _read_until(self, marker: bytes, timeout_s: float) -> bytes:
        if self._port is None:
            raise RuntimeError("serial port is not open")

        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = self._port.read(1)
            if not chunk:
                continue
            data.extend(chunk)
            if data.endswith(marker):
                return bytes(data)
        raise TimeoutError(f"timeout waiting for {marker!r}, got {bytes(data)!r}")

    def _drain(self, timeout_s: float = 0.2) -> None:
        if self._port is None:
            raise RuntimeError("serial port is not open")

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._port.read(1024)

    def _enter_raw_repl(self) -> None:
        self._drain()
        self._write_all(CTRL_C)
        time.sleep(0.1)
        self._write_all(CTRL_C)
        time.sleep(0.1)
        self._write_all(RAW_REPL_EXIT)
        time.sleep(0.05)
        self._drain()
        self._write_all(RAW_REPL_ENTER)
        self._read_until(RAW_REPL_PROMPT, 2.0)
        self._write_all(CTRL_D)
        self._read_until(RAW_REPL_PROMPT, 4.0)

    def exec_raw(self, command: str, timeout_s: float = 5.0) -> str:
        payload = command.encode("utf-8")
        self._write_all(payload)
        self._write_all(CTRL_D)
        self._read_until(b"OK", 2.0)
        stdout_frame = self._read_until(b"\x04", timeout_s)
        stderr_frame = self._read_until(b"\x04", timeout_s)
        stdout_bytes = stdout_frame[:-1]
        stderr_bytes = stderr_frame[:-1]
        if stderr_bytes:
            raise RuntimeError(stderr_bytes.decode("utf-8", errors="replace"))
        return stdout_bytes.decode("utf-8", errors="replace")

    def hard_reset(self) -> None:
        self._hard_reset_requested = True
        payload = b"import machine\nmachine.reset()\n"
        self._write_all(payload)
        self._write_all(CTRL_D)
        time.sleep(0.2)


def resolve_project_path(project_dir: Path, value: str) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path.resolve()
    return (project_dir / path).resolve()


def resolve_optional_project_path(project_dir: Path, value: str) -> Path | None:
    text = value.strip()
    if not text:
        return None
    return resolve_project_path(project_dir, text)


def resolve_optional_cfg_path(project_dir: Path, value: str) -> str:
    text = value.strip()
    if not text:
        return ""
    candidate = project_dir / text
    if candidate.exists():
        return str(candidate.resolve())
    return text


def load_config(config_path: Path) -> ProjectConfig:
    parser = configparser.ConfigParser()
    if not parser.read(config_path):
        raise RuntimeError(f"OpenMV config not found: {config_path}")

    project_dir = config_path.parent.resolve()
    src_dir = resolve_project_path(project_dir, parser.get("project", "src_dir", fallback="src"))
    entry_script = resolve_project_path(project_dir, parser.get("project", "entry_script", fallback="src/main.py"))
    build_dir = resolve_project_path(project_dir, parser.get("project", "build_dir", fallback=".openmv_build"))

    remote_manifest_value = parser.get("project", "remote_manifest", fallback="_guided_dart_manifest.txt").strip()
    remote_manifest_value = remote_manifest_value.lstrip("/") or "_guided_dart_manifest.txt"
    remote_manifest = PurePosixPath(remote_manifest_value)

    serial_port = parser.get("serial", "port", fallback=parser.get("device", "port", fallback="")).strip()
    serial_baudrate = parser.getint(
        "serial",
        "baudrate",
        fallback=parser.getint("device", "baudrate", fallback=115200),
    )

    sdcard_root = resolve_optional_project_path(project_dir, parser.get("sdcard", "root", fallback=""))

    stlink_openocd = parser.get("stlink", "openocd", fallback="openocd").strip() or "openocd"
    stlink_interface_cfg = parser.get("stlink", "interface_cfg", fallback="interface/stlink.cfg").strip()
    stlink_target_cfg = parser.get("stlink", "target_cfg", fallback="").strip()
    stlink_transport = parser.get("stlink", "transport", fallback="hla_swd").strip() or "hla_swd"
    stlink_firmware_image = resolve_optional_project_path(
        project_dir, parser.get("stlink", "firmware_image", fallback="")
    )
    stlink_firmware_address = parser.get("stlink", "firmware_address", fallback="").strip()

    if not src_dir.is_dir():
        raise RuntimeError(f"OpenMV source directory not found: {src_dir}")
    if not entry_script.is_file():
        raise RuntimeError(f"OpenMV entry script not found: {entry_script}")
    try:
        entry_script.relative_to(src_dir)
    except ValueError as exc:
        raise RuntimeError(f"entry_script must stay under src_dir: {entry_script}") from exc

    return ProjectConfig(
        project_dir=project_dir,
        src_dir=src_dir,
        entry_script=entry_script,
        build_dir=build_dir,
        remote_manifest=remote_manifest,
        serial_port=serial_port,
        serial_baudrate=serial_baudrate,
        sdcard_root=sdcard_root,
        stlink_openocd=stlink_openocd,
        stlink_interface_cfg=stlink_interface_cfg,
        stlink_target_cfg=stlink_target_cfg,
        stlink_transport=stlink_transport,
        stlink_firmware_image=stlink_firmware_image,
        stlink_firmware_address=stlink_firmware_address,
    )


def collect_source_files(config: ProjectConfig) -> list[Path]:
    paths: list[Path] = []
    for path in sorted(config.src_dir.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        if path.name.startswith("."):
            continue
        paths.append(path)
    if config.entry_script not in paths:
        raise RuntimeError(f"entry script is missing from source tree: {config.entry_script}")
    return paths


def validate_source_file(path: Path) -> None:
    source_text = path.read_text(encoding="utf-8")
    compile(source_text, str(path), "exec")


def validate_sources(paths: list[Path]) -> None:
    for path in paths:
        validate_source_file(path)


def resolve_mpy_cross(explicit_path: str) -> str:
    if explicit_path:
        return explicit_path
    command = shutil.which("mpy-cross")
    if command:
        return command
    raise RuntimeError("mpy-cross was not found. Install it or rerun without --mpy.")


def build_dir_fs(config: ProjectConfig) -> Path:
    return config.build_dir / "fs"


def build_generated_params_text() -> str:
    if str(HOST_GUIDANCE_DIR) not in sys.path:
        sys.path.insert(0, str(HOST_GUIDANCE_DIR))
    from openmv_detector_params import build_openmv_generated_params_text

    return build_openmv_generated_params_text(GUIDANCE_PARAMS_PATH)


def validate_generated_params() -> None:
    text = build_generated_params_text()
    compile(text, GENERATED_PARAMS_MODULE.as_posix(), "exec")


def generated_params_artifact(config: ProjectConfig, stage_dir: Path) -> Artifact:
    text = build_generated_params_text()
    local_path = stage_dir / GENERATED_PARAMS_MODULE.as_posix()
    local_path.parent.mkdir(parents=True, exist_ok=True)
    local_path.write_text(text, encoding="utf-8")
    validate_source_file(local_path)
    return Artifact(
        source_path=local_path,
        source_relative=GENERATED_PARAMS_MODULE,
        local_path=local_path,
        remote_path=GENERATED_PARAMS_MODULE,
        kind="py",
    )


def artifact_from_source(
    config: ProjectConfig,
    source_path: Path,
    stage_dir: Path,
    use_mpy: bool,
    mpy_cross_cmd: str,
) -> Artifact:
    relative_path = source_path.relative_to(config.src_dir)
    source_relative = PurePosixPath(relative_path.as_posix())
    remote_path = source_relative
    local_path = stage_dir / relative_path
    kind = "py"

    is_entry_script = source_path == config.entry_script
    if use_mpy and not is_entry_script:
        remote_path = source_relative.with_suffix(".mpy")
        local_path = (stage_dir / relative_path).with_suffix(".mpy")
        local_path.parent.mkdir(parents=True, exist_ok=True)
        result = subprocess.run(
            [mpy_cross_cmd, "-o", str(local_path), str(source_path)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            details = (result.stderr or result.stdout or "").strip()
            raise RuntimeError(f"mpy-cross failed for {source_path}: {details}")
        kind = "mpy"
    else:
        local_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, local_path)

    return Artifact(
        source_path=source_path,
        source_relative=source_relative,
        local_path=local_path,
        remote_path=PurePosixPath(remote_path.as_posix()),
        kind=kind,
    )


def build_project(config: ProjectConfig, use_mpy: bool, mpy_cross: str) -> list[Artifact]:
    sources = collect_source_files(config)
    validate_sources(sources)

    stage_root = config.build_dir
    stage_dir = build_dir_fs(config)
    if stage_root.exists():
        shutil.rmtree(stage_root)
    stage_dir.mkdir(parents=True, exist_ok=True)

    mpy_cross_cmd = resolve_mpy_cross(mpy_cross) if use_mpy else ""
    artifacts = [
        artifact_from_source(config, source_path, stage_dir, use_mpy, mpy_cross_cmd)
        for source_path in sources
    ]
    if any(artifact.remote_path == GENERATED_PARAMS_MODULE for artifact in artifacts):
        raise RuntimeError(f"source tree must not contain generated module: {GENERATED_PARAMS_MODULE.as_posix()}")
    artifacts.append(generated_params_artifact(config, stage_dir))

    manifest_path = config.build_dir / "manifest.json"
    manifest_payload = {
        "entry_script": str(config.entry_script.relative_to(config.project_dir)),
        "artifacts": [
            {
                "source": str(artifact.source_path.relative_to(REPO_ROOT)),
                "artifact": str(artifact.local_path.relative_to(config.project_dir)),
                "remote_path": artifact.remote_path.as_posix(),
                "kind": artifact.kind,
            }
            for artifact in artifacts
        ],
    }
    manifest_path.write_text(json.dumps(manifest_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return artifacts


def list_serial_ports(include_all: bool = False) -> list[dict[str, str]]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []

    ports: list[dict[str, str]] = []
    for port in list_ports.comports():
        ports.append(
            {
                "device": str(port.device),
                "description": str(port.description or "n/a"),
                "hwid": str(port.hwid or "n/a"),
            }
        )

    ports.sort(key=lambda item: item["device"])
    if include_all:
        return ports

    preferred = [
        item
        for item in ports
        if item["device"].startswith("/dev/ttyACM")
        or item["device"].startswith("/dev/ttyUSB")
        or item["description"] != "n/a"
        or "USB" in item["hwid"]
    ]
    return preferred or ports


def format_serial_ports(include_all: bool = False) -> str:
    ports = list_serial_ports(include_all=include_all)
    if not ports:
        return "no serial devices detected"
    return "\n".join(f"{item['device']}\t{item['description']}\t{item['hwid']}" for item in ports)


def ensure_serial_port_available(port: str) -> None:
    if not port:
        raise RuntimeError(
            "serial port is empty; set [serial] port in OpenMV_guidance/openmv.ini or pass --port\n"
            + format_serial_ports()
        )
    if Path(port).exists():
        return
    raise RuntimeError(f"serial port not found: {port}\n{format_serial_ports()}")


def ensure_remote_parent(session: DirectRawReplSession, remote_path: PurePosixPath, created_dirs: set[str]) -> None:
    parent = remote_path.parent
    if parent.as_posix() in ("", ".", "/flash"):
        return

    parts = [part for part in parent.parts if part not in ("", ".", "/")]
    commands = ["import os"]
    current: list[str] = []
    if parent.is_absolute() and parts[:1] == ["flash"]:
        current.append("flash")
        parts = parts[1:]
    dirty = False
    for part in parts:
        current.append(part)
        current_path = ("/" if parent.is_absolute() else "") + "/".join(current)
        if current_path in created_dirs:
            continue
        commands.extend(
            [
                "try:",
                f"    os.mkdir({current_path!r})",
                "except OSError:",
                "    pass",
            ]
        )
        created_dirs.add(current_path)
        dirty = True
    if dirty:
        session.exec_raw("\n".join(commands) + "\n", timeout_s=5.0)


def delete_remote_paths(session: DirectRawReplSession, remote_paths: set[str]) -> None:
    if not remote_paths:
        return

    commands = ["import os"]
    for remote_path in sorted(remote_paths):
        commands.extend(
            [
                "try:",
                f"    os.remove({remote_path!r})",
                "except OSError:",
                "    pass",
            ]
        )
    session.exec_raw("\n".join(commands) + "\n", timeout_s=10.0)


def upload_bytes(session: DirectRawReplSession, remote_path: PurePosixPath, payload: bytes, chunk_size: int) -> None:
    remote_path_text = remote_path.as_posix()
    if not payload:
        session.exec_raw(f"f = open({remote_path_text!r}, 'wb')\nf.close()\nprint('ok')\n", timeout_s=5.0)
        return

    offset = 0
    mode = "wb"
    while offset < len(payload):
        chunk = payload[offset : offset + chunk_size]
        output = session.exec_raw(
            "\n".join(
                [
                    "import ubinascii",
                    f"f = open({remote_path_text!r}, {mode!r})",
                    f"f.write(ubinascii.unhexlify({chunk.hex()!r}))",
                    "f.close()",
                    f"print({offset + len(chunk)})",
                ]
            )
            + "\n",
            timeout_s=10.0,
        ).strip()
        expected = str(offset + len(chunk))
        if output.splitlines()[-1:] != [expected]:
            raise RuntimeError(
                f"short upload confirmation for {remote_path_text}: expected {expected}, got {output!r}"
            )
        offset += len(chunk)
        mode = "ab"


def read_remote_manifest(session: DirectRawReplSession, remote_manifest: PurePosixPath) -> set[str]:
    output = session.exec_raw(
        "\n".join(
            [
                "try:",
                f"    f = open({remote_manifest.as_posix()!r}, 'r')",
                "    data = f.read()",
                "    f.close()",
                "    print(data)",
                "except OSError:",
                "    pass",
            ]
        )
        + "\n",
        timeout_s=5.0,
    )
    return {line.strip() for line in output.splitlines() if line.strip()}


def read_cleanup_paths(artifacts: list[Artifact]) -> set[str]:
    cleanup_paths: set[str] = set()
    for artifact in artifacts:
        cleanup_paths.add(artifact.source_relative.with_suffix(".py").as_posix())
        cleanup_paths.add(artifact.source_relative.with_suffix(".mpy").as_posix())
    return cleanup_paths


def flash_remote_path(remote_path: PurePosixPath | str) -> PurePosixPath:
    path = PurePosixPath(remote_path)
    if ".." in path.parts:
        raise RuntimeError(f"unsafe remote path: {remote_path}")
    if path.is_absolute():
        if path.parts[:2] == ("/", "flash"):
            return path
        path = PurePosixPath(*path.parts[1:])
    return FLASH_REMOTE_ROOT / path


def flash_project(config: ProjectConfig, artifacts: list[Artifact], chunk_size: int, reset: bool) -> None:
    ensure_serial_port_available(config.serial_port)
    created_dirs: set[str] = set()
    cleanup_paths = read_cleanup_paths(artifacts)
    remote_manifest = flash_remote_path(config.remote_manifest)

    with DirectRawReplSession(config.serial_port, config.serial_baudrate) as session:
        stale_paths = {flash_remote_path(path).as_posix() for path in read_remote_manifest(session, remote_manifest)}
        stale_paths.add(remote_manifest.as_posix())
        stale_paths.update(flash_remote_path(path).as_posix() for path in cleanup_paths)
        delete_remote_paths(session, stale_paths)

        uploaded_paths: list[str] = []
        for artifact in artifacts:
            remote_path = flash_remote_path(artifact.remote_path)
            ensure_remote_parent(session, remote_path, created_dirs)
            payload = artifact.local_path.read_bytes()
            upload_bytes(session, remote_path, payload, chunk_size)
            uploaded_paths.append(remote_path.as_posix())
            print(f"uploaded {remote_path.as_posix()}")

        manifest_payload = ("\n".join(uploaded_paths) + "\n").encode("utf-8")
        upload_bytes(session, remote_manifest, manifest_payload, chunk_size)
        print(f"uploaded {remote_manifest.as_posix()}")

        if reset:
            session.hard_reset()
            print("triggered hard reset to run main.py")


def relative_posix_to_host_path(root: Path, relative_path: PurePosixPath) -> Path:
    if relative_path.is_absolute() or ".." in relative_path.parts:
        raise RuntimeError(f"unsafe relative path: {relative_path.as_posix()}")
    parts = [part for part in relative_path.parts if part not in ("", ".")]
    return root.joinpath(*parts)


def read_host_manifest(root: Path, manifest_relative: PurePosixPath) -> set[str]:
    manifest_path = relative_posix_to_host_path(root, manifest_relative)
    if not manifest_path.exists():
        return set()
    return {line.strip() for line in manifest_path.read_text(encoding="utf-8").splitlines() if line.strip()}


def prune_empty_parents(start: Path, stop_at: Path) -> None:
    current = start
    stop_at = stop_at.resolve()
    while True:
        try:
            if current.resolve() == stop_at:
                return
        except FileNotFoundError:
            pass
        if not current.exists():
            current = current.parent
            continue
        try:
            current.rmdir()
        except OSError:
            return
        current = current.parent


def delete_host_paths(root: Path, relative_paths: set[str]) -> None:
    for relative_text in sorted(relative_paths, reverse=True):
        relative_path = PurePosixPath(relative_text)
        target = relative_posix_to_host_path(root, relative_path)
        if target.is_file() or target.is_symlink():
            target.unlink(missing_ok=True)
            prune_empty_parents(target.parent, root)


def sync_host_directory(config: ProjectConfig, artifacts: list[Artifact], destination_root: Path) -> None:
    destination_root.mkdir(parents=True, exist_ok=True)
    cleanup_paths = read_cleanup_paths(artifacts)
    stale_paths = read_host_manifest(destination_root, config.remote_manifest)
    stale_paths.add(config.remote_manifest.as_posix())
    stale_paths.update(cleanup_paths)
    delete_host_paths(destination_root, stale_paths)

    uploaded_paths: list[str] = []
    for artifact in artifacts:
        target = relative_posix_to_host_path(destination_root, artifact.remote_path)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(artifact.local_path, target)
        uploaded_paths.append(artifact.remote_path.as_posix())
        print(f"synced {target}")

    manifest_path = relative_posix_to_host_path(destination_root, config.remote_manifest)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text("\n".join(uploaded_paths) + "\n", encoding="utf-8")
    print(f"wrote manifest {manifest_path}")


def resolve_openocd_binary(command: str) -> str:
    if os.path.isabs(command) and os.access(command, os.X_OK):
        return command
    resolved = shutil.which(command)
    if resolved:
        return resolved
    raise RuntimeError(f"OpenOCD executable not found: {command}")


def tcl_quote(value: str) -> str:
    return "{" + value.replace("}", "\\}") + "}"


def build_openocd_program_command(image_path: Path, address: str) -> str:
    suffix = image_path.suffix.lower()
    command = f"program {tcl_quote(str(image_path))}"
    if suffix == ".bin":
        if not address:
            raise RuntimeError("binary firmware images require a flash address; set [stlink] firmware_address or pass --address")
        command += f" {address}"
    command += " verify reset exit"
    return command


def run_openocd_flash(
    openocd_cmd: str,
    interface_cfg: str,
    target_cfg: str,
    transport: str,
    image_path: Path,
    address: str,
) -> None:
    command = [
        openocd_cmd,
        "-f",
        interface_cfg,
        "-f",
        target_cfg,
        "-c",
        f"transport select {transport}",
        "-c",
        build_openocd_program_command(image_path, address),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.stdout.strip():
        print(result.stdout.strip())
    if result.returncode != 0:
        details = (result.stderr or result.stdout or "").strip()
        raise RuntimeError(f"OpenOCD ST-Link flash failed: {details}")
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)


def clean_project(config: ProjectConfig) -> None:
    if config.build_dir.exists():
        shutil.rmtree(config.build_dir)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build and deploy the OpenMV Python project")
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[1] / "openmv.ini"),
        help="path to the OpenMV project config",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    ports_parser = subparsers.add_parser("ports", help="list detected serial devices")
    ports_parser.add_argument("--all", action="store_true", help="include legacy ttyS devices in the listing")
    ports_parser.set_defaults(handler=handle_ports)

    check_parser = subparsers.add_parser("check", help="syntax-check the OpenMV Python sources")
    check_parser.set_defaults(handler=handle_check)

    build_parser = subparsers.add_parser("build", help="stage files for deployment")
    build_parser.add_argument("--mpy", action="store_true", help="compile non-entry modules to .mpy with mpy-cross")
    build_parser.add_argument("--mpy-cross", default="", help="override the mpy-cross executable path")
    build_parser.set_defaults(handler=handle_build)

    flash_parser = subparsers.add_parser("flash", help="build and upload Python files over OpenMV USB serial raw REPL")
    flash_parser.add_argument("--mpy", action="store_true", help="compile non-entry modules to .mpy with mpy-cross")
    flash_parser.add_argument("--mpy-cross", default="", help="override the mpy-cross executable path")
    flash_parser.add_argument("--port", default="", help="override [serial] port from config")
    flash_parser.add_argument("--baudrate", type=int, default=0, help="override [serial] baudrate from config")
    flash_parser.add_argument("--chunk-size", type=int, default=256, help="bytes written per raw-REPL chunk")
    flash_parser.add_argument("--no-reset", action="store_true", help="leave the board in REPL after upload")
    flash_parser.set_defaults(handler=handle_flash)

    sd_parser = subparsers.add_parser("sd-sync", help="build and sync Python files into a mounted SD-card boot filesystem")
    sd_parser.add_argument("--mpy", action="store_true", help="compile non-entry modules to .mpy with mpy-cross")
    sd_parser.add_argument("--mpy-cross", default="", help="override the mpy-cross executable path")
    sd_parser.add_argument("--dest", default="", help="mounted SD-card root; overrides [sdcard] root from config")
    sd_parser.set_defaults(handler=handle_sd_sync)

    stlink_parser = subparsers.add_parser("stlink-flash", help="flash an OpenMV firmware image through ST-Link via OpenOCD")
    stlink_parser.add_argument("--image", default="", help="firmware image path; overrides [stlink] firmware_image")
    stlink_parser.add_argument("--address", default="", help="flash address for .bin images; overrides [stlink] firmware_address")
    stlink_parser.add_argument("--interface-cfg", default="", help="OpenOCD ST-Link interface cfg")
    stlink_parser.add_argument("--target-cfg", default="", help="OpenOCD target cfg for your OpenMV board MCU")
    stlink_parser.add_argument("--transport", default="", help="OpenOCD transport, e.g. hla_swd or swd")
    stlink_parser.add_argument("--openocd", default="", help="OpenOCD executable path")
    stlink_parser.set_defaults(handler=handle_stlink_flash)

    clean_parser = subparsers.add_parser("clean", help="remove local build artifacts")
    clean_parser.set_defaults(handler=handle_clean)

    return parser


def handle_ports(args: argparse.Namespace, config: ProjectConfig) -> int:
    del config
    listing = format_serial_ports(include_all=bool(args.all))
    print(listing)
    return 0


def handle_check(args: argparse.Namespace, config: ProjectConfig) -> int:
    del args
    sources = collect_source_files(config)
    validate_sources(sources)
    validate_generated_params()
    print(f"checked {len(sources)} source files and generated {GENERATED_PARAMS_MODULE.as_posix()}")
    return 0


def handle_build(args: argparse.Namespace, config: ProjectConfig) -> int:
    artifacts = build_project(config, use_mpy=bool(args.mpy), mpy_cross=str(args.mpy_cross))
    print(f"built {len(artifacts)} artifacts into {config.build_dir}")
    return 0


def handle_flash(args: argparse.Namespace, config: ProjectConfig) -> int:
    effective_config = ProjectConfig(
        project_dir=config.project_dir,
        src_dir=config.src_dir,
        entry_script=config.entry_script,
        build_dir=config.build_dir,
        remote_manifest=config.remote_manifest,
        serial_port=args.port or config.serial_port,
        serial_baudrate=int(args.baudrate or config.serial_baudrate),
        sdcard_root=config.sdcard_root,
        stlink_openocd=config.stlink_openocd,
        stlink_interface_cfg=config.stlink_interface_cfg,
        stlink_target_cfg=config.stlink_target_cfg,
        stlink_transport=config.stlink_transport,
        stlink_firmware_image=config.stlink_firmware_image,
        stlink_firmware_address=config.stlink_firmware_address,
    )
    artifacts = build_project(effective_config, use_mpy=bool(args.mpy), mpy_cross=str(args.mpy_cross))
    flash_project(
        effective_config,
        artifacts,
        chunk_size=max(32, int(args.chunk_size)),
        reset=not bool(args.no_reset),
    )
    return 0


def handle_sd_sync(args: argparse.Namespace, config: ProjectConfig) -> int:
    destination = args.dest.strip()
    if destination:
        destination_root = resolve_project_path(config.project_dir, destination)
    elif config.sdcard_root is not None:
        destination_root = config.sdcard_root
    else:
        raise RuntimeError("sdcard root is empty; set [sdcard] root in OpenMV_guidance/openmv.ini or pass --dest")

    artifacts = build_project(config, use_mpy=bool(args.mpy), mpy_cross=str(args.mpy_cross))
    sync_host_directory(config, artifacts, destination_root)
    return 0


def handle_stlink_flash(args: argparse.Namespace, config: ProjectConfig) -> int:
    image = resolve_optional_project_path(config.project_dir, args.image) if args.image else config.stlink_firmware_image
    if image is None:
        raise RuntimeError("firmware image is empty; set [stlink] firmware_image in OpenMV_guidance/openmv.ini or pass --image")
    if not image.is_file():
        raise RuntimeError(f"firmware image not found: {image}")

    openocd_cmd = resolve_openocd_binary(args.openocd.strip() or config.stlink_openocd)
    interface_cfg = resolve_optional_cfg_path(config.project_dir, args.interface_cfg or config.stlink_interface_cfg)
    target_cfg = resolve_optional_cfg_path(config.project_dir, args.target_cfg or config.stlink_target_cfg)
    transport = (args.transport or config.stlink_transport).strip() or "hla_swd"
    address = (args.address or config.stlink_firmware_address).strip()

    if not interface_cfg:
        raise RuntimeError("OpenOCD interface cfg is empty; set [stlink] interface_cfg or pass --interface-cfg")
    if not target_cfg:
        raise RuntimeError("OpenOCD target cfg is empty; set [stlink] target_cfg or pass --target-cfg")

    run_openocd_flash(openocd_cmd, interface_cfg, target_cfg, transport, image, address)
    print(f"flashed firmware image via ST-Link: {image}")
    return 0


def handle_clean(args: argparse.Namespace, config: ProjectConfig) -> int:
    del args
    clean_project(config)
    print(f"removed {config.build_dir}")
    return 0


def main(argv: list[str]) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(Path(args.config).resolve())
    return int(args.handler(args, config))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
