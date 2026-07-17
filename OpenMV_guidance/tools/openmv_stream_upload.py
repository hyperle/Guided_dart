#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import time
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from openmv_cli import (
    ProjectConfig,
    build_project,
    flash_remote_path,
    load_config,
    read_cleanup_paths,
)

CTRL_C = b"\x03"
CTRL_D = b"\x04"
RAW_REPL_ENTER = b"\x01"
RAW_REPL_EXIT = b"\x02"
RAW_REPL_PROMPT = b"raw REPL; CTRL-B to exit\r\n>"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "openmv.ini"
DEFAULT_EXCLUDED_FLASH_MODULES = {"stream_debug.py", "tuner_runtime.py"}
SD_BOOT_STUB_SOURCE = (
    "import os\n"
    "os.chdir(\"/flash\")\n"
    "exec(open(\"/flash/main.py\").read())\n"
)


RECEIVER_SOURCE = r'''
import os
import pyb
import ubinascii

VCP = pyb.USB_VCP()
TMP_ROOT = "/flash/.gd_upload_tmp"
MANIFEST = "/flash/_guided_dart_manifest.txt"


def _write_line(text):
    VCP.write((text + "\n").encode())


def _read_line():
    data = bytearray()
    while True:
        chunk = VCP.read(1)
        if not chunk:
            pyb.delay(1)
            continue
        value = chunk[0]
        if value == 10:
            return bytes(data).decode().strip()
        if value != 13:
            data.append(value)


def _is_dir(path):
    try:
        return (os.stat(path)[0] & 0x4000) != 0
    except OSError:
        return False


def _parent(path):
    index = path.rfind("/")
    if index <= 0:
        return "/"
    return path[:index]


def _mkdirs(path):
    parts = [part for part in path.split("/") if part]
    current = ""
    for part in parts:
        current = current + "/" + part
        try:
            os.mkdir(current)
        except OSError:
            pass


def _remove_tree(path):
    try:
        names = os.listdir(path)
    except OSError:
        names = []
    for name in names:
        child = path + "/" + name
        if _is_dir(child):
            _remove_tree(child)
            try:
                os.rmdir(child)
            except OSError:
                pass
        else:
            try:
                os.remove(child)
            except OSError:
                pass
    try:
        os.rmdir(path)
    except OSError:
        pass


def _clear_flash():
    _remove_tree(TMP_ROOT)
    try:
        names = os.listdir("/flash")
    except OSError:
        names = []
    for name in names:
        child = "/flash/" + name
        if _is_dir(child):
            _remove_tree(child)
        else:
            try:
                os.remove(child)
            except OSError:
                pass
    _mkdirs(TMP_ROOT)


def _clean_rel(path):
    if not path or path.startswith("/"):
        raise RuntimeError("bad path")
    parts = path.split("/")
    for part in parts:
        if part in ("", ".", ".."):
            raise RuntimeError("bad path")
    return "/".join(parts)


def _to_rel(path):
    path = path.strip()
    if path.startswith("/flash/"):
        path = path[7:]
    elif path.startswith("/"):
        path = path[1:]
    return _clean_rel(path)


def _read_old_manifest():
    paths = []
    try:
        f = open(MANIFEST, "r")
        for line in f:
            line = line.strip()
            if line:
                try:
                    paths.append(_to_rel(line))
                except Exception:
                    pass
        f.close()
    except OSError:
        pass
    return paths


def _checksum(data, current):
    for value in data:
        current = (current + value) & 0xffffffff
    return current


old_paths = _read_old_manifest()
cleanup_paths = []
_remove_tree(TMP_ROOT)
_mkdirs(TMP_ROOT)
_write_line("READY")

while True:
    line = _read_line()
    if not line:
        continue
    parts = line.split()
    command = parts[0]

    if command == "CLEAR_ALL":
        _clear_flash()
        old_paths = []
        cleanup_paths = []
        _write_line("OK CLEAR")

    elif command == "FILE":
        rel = _clean_rel(parts[1])
        expected_size = int(parts[2])
        expected_checksum = int(parts[3])
        tmp_path = TMP_ROOT + "/" + rel
        dst_path = "/flash/" + rel
        _mkdirs(_parent(tmp_path))
        try:
            os.remove(dst_path)
        except OSError:
            pass
        got_size = 0
        got_checksum = 0
        f = open(tmp_path, "wb")
        while got_size < expected_size:
            payload = _read_line()
            data = ubinascii.a2b_base64(payload)
            f.write(data)
            got_size += len(data)
            got_checksum = _checksum(data, got_checksum)
            _write_line("C %d" % got_size)
        f.close()
        if got_size != expected_size or got_checksum != expected_checksum:
            raise RuntimeError("verify failed %s" % rel)
        _write_line("OK %s %d %d" % (rel, got_size, got_checksum))

    elif command == "CLEAN":
        count = int(parts[1])
        for _ in range(count):
            try:
                cleanup_paths.append(_clean_rel(_read_line()))
            except Exception:
                pass
        _write_line("OK CLEAN")

    elif command == "COMMIT":
        count = int(parts[1])
        keep = []
        for _ in range(count):
            rel = _clean_rel(_read_line())
            keep.append(rel)
            src = TMP_ROOT + "/" + rel
            dst = "/flash/" + rel
            _mkdirs(_parent(dst))
            try:
                os.remove(dst)
            except OSError:
                pass
            os.rename(src, dst)
        for rel in old_paths + cleanup_paths:
            if rel and rel not in keep:
                try:
                    os.remove("/flash/" + rel)
                except OSError:
                    pass
        _remove_tree(TMP_ROOT)
        try:
            os.sync()
        except Exception:
            pass
        _write_line("OK COMMIT")

    elif command == "DONE":
        try:
            os.sync()
        except Exception:
            pass
        _write_line("DONE")
        break

    else:
        raise RuntimeError("bad command")
'''


@dataclass(frozen=True)
class UploadItem:
    rel_path: str
    payload: bytes


class StreamingRawReplSession:
    def __init__(self, port: str, baudrate: int):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required: python3 -m pip install -r OpenMV_guidance/requirements.txt"
            ) from exc
        self._serial_mod = serial
        self._port_name = port
        self._baudrate = int(baudrate)
        self._port = None
        self._hard_reset_requested = False

    def __enter__(self) -> "StreamingRawReplSession":
        self._port = self._serial_mod.Serial(
            self._port_name,
            self._baudrate,
            timeout=0.1,
            write_timeout=5.0,
        )
        self._enter_raw_repl()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            if self._port is not None and not self._hard_reset_requested:
                try:
                    self._write_all(CTRL_C)
                    time.sleep(0.1)
                    self._write_all(RAW_REPL_EXIT)
                    time.sleep(0.1)
                except Exception:
                    if exc_type is None:
                        raise
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

    def _drain(self, timeout_s: float = 0.2) -> bytes:
        if self._port is None:
            raise RuntimeError("serial port is not open")
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = self._port.read(1024)
            if chunk:
                data.extend(chunk)
        return bytes(data)

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
        self._read_until(RAW_REPL_PROMPT, 3.0)
        self._write_all(CTRL_D)
        self._read_until(RAW_REPL_PROMPT, 5.0)

    def start_receiver(self) -> None:
        self._write_all(RECEIVER_SOURCE.encode("utf-8"))
        self._write_all(CTRL_D)
        self._read_until(b"OK", 5.0)
        line = self.read_line(10.0)
        if line != "READY":
            raise RuntimeError(f"receiver did not become ready: {line!r}")

    def read_line(self, timeout_s: float) -> str:
        if self._port is None:
            raise RuntimeError("serial port is not open")
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = self._port.read(1)
            if not chunk:
                continue
            value = chunk[0]
            if value == 10:
                return bytes(data).decode("utf-8", errors="replace").strip()
            if value not in (10, 13, 4):
                data.append(value)
        raise TimeoutError(f"timeout waiting for receiver line, got {bytes(data)!r}")

    def write_line(self, text: str) -> None:
        self._write_all(text.encode("utf-8") + b"\n")

    def finish_receiver(self) -> None:
        self.write_line("DONE")
        line = self.read_line(10.0)
        if line != "DONE":
            raise RuntimeError(f"receiver did not finish cleanly: {line!r}")
        self._read_until(b"\x04", 5.0)
        self._read_until(b"\x04", 5.0)
        self._drain(0.1)

    def exec_raw(self, command: str, timeout_s: float = 5.0) -> str:
        self._write_all(command.encode("utf-8"))
        self._write_all(CTRL_D)
        self._read_until(b"OK", 2.0)
        stdout_frame = self._read_until(b"\x04", timeout_s)
        stderr_frame = self._read_until(b"\x04", timeout_s)
        stdout = stdout_frame[:-1]
        stderr = stderr_frame[:-1]
        if stderr:
            raise RuntimeError(stderr.decode("utf-8", errors="replace"))
        return stdout.decode("utf-8", errors="replace")

    def hard_reset(self) -> None:
        self._hard_reset_requested = True
        self._write_all(b"import machine\nmachine.reset()\n")
        self._write_all(CTRL_D)
        time.sleep(0.2)


def payload_checksum(payload: bytes) -> int:
    checksum = 0
    for value in payload:
        checksum = (checksum + value) & 0xFFFFFFFF
    return checksum


def remote_relative_path(remote_path: PurePosixPath | str) -> str:
    path = flash_remote_path(remote_path)
    parts = path.parts
    if len(parts) < 3 or parts[0] != "/" or parts[1] != "flash":
        raise RuntimeError(f"remote path is not under /flash: {path.as_posix()}")
    rel_parts = parts[2:]
    if not rel_parts or any(part in ("", ".", "..") for part in rel_parts):
        raise RuntimeError(f"unsafe remote path: {path.as_posix()}")
    rel = "/".join(rel_parts)
    if any(char.isspace() for char in rel):
        raise RuntimeError(f"remote path contains whitespace: {path.as_posix()}")
    return rel


def flash_artifact_included(artifact, include_debug: bool) -> bool:
    if include_debug:
        return True
    return artifact.source_relative.as_posix() not in DEFAULT_EXCLUDED_FLASH_MODULES


def build_upload_items(config: ProjectConfig, use_mpy: bool, mpy_cross: str, include_debug: bool) -> tuple[list[UploadItem], set[str]]:
    artifacts = build_project(config, use_mpy=use_mpy, mpy_cross=mpy_cross)
    upload_artifacts = [artifact for artifact in artifacts if flash_artifact_included(artifact, include_debug)]
    skipped = [artifact.source_relative.as_posix() for artifact in artifacts if artifact not in upload_artifacts]
    if skipped:
        print("skipping debug-only files on /flash: " + ", ".join(skipped))
    items = [
        UploadItem(remote_relative_path(artifact.remote_path), artifact.local_path.read_bytes())
        for artifact in upload_artifacts
    ]
    manifest_payload = (
        "\n".join(flash_remote_path(PurePosixPath(item.rel_path)).as_posix() for item in items) + "\n"
    ).encode("utf-8")
    items.append(UploadItem(remote_relative_path(config.remote_manifest), manifest_payload))
    return items, read_cleanup_paths(artifacts)

def upload_file(session: StreamingRawReplSession, item: UploadItem, chunk_size: int) -> None:
    size = len(item.payload)
    checksum = payload_checksum(item.payload)
    session.write_line(f"FILE {item.rel_path} {size} {checksum}")
    offset = 0
    while offset < size:
        chunk = item.payload[offset : offset + chunk_size]
        offset += len(chunk)
        session.write_line(base64.b64encode(chunk).decode("ascii"))
        expected_ack = f"C {offset}"
        ack = session.read_line(10.0)
        if ack != expected_ack:
            details = [ack]
            for _ in range(8):
                try:
                    details.append(session.read_line(0.2))
                except TimeoutError:
                    break
            raise RuntimeError(
                f"bad chunk ack for {item.rel_path}: expected {expected_ack!r}, "
                f"got {ack!r}; receiver output: {' | '.join(details)!r}"
            )
    expected = f"OK {item.rel_path} {size} {checksum}"
    ack = session.read_line(10.0)
    if ack != expected:
        raise RuntimeError(f"bad file ack for {item.rel_path}: expected {expected!r}, got {ack!r}")


def clear_flash_storage(session: StreamingRawReplSession) -> None:
    session.write_line("CLEAR_ALL")
    ack = session.read_line(30.0)
    if ack != "OK CLEAR":
        raise RuntimeError(f"bad clear ack: {ack!r}")


def send_cleanup_paths(session: StreamingRawReplSession, cleanup_paths: set[str]) -> None:
    safe_paths = [remote_relative_path(path) for path in sorted(cleanup_paths)]
    session.write_line(f"CLEAN {len(safe_paths)}")
    for rel in safe_paths:
        session.write_line(rel)
    ack = session.read_line(10.0)
    if ack != "OK CLEAN":
        raise RuntimeError(f"bad cleanup ack: {ack!r}")


def commit_files(session: StreamingRawReplSession, items: list[UploadItem]) -> None:
    session.write_line(f"COMMIT {len(items)}")
    for item in items:
        session.write_line(item.rel_path)
    ack = session.read_line(20.0)
    if ack != "OK COMMIT":
        raise RuntimeError(f"bad commit ack: {ack!r}")


def install_sd_boot_stub(session: StreamingRawReplSession) -> None:
    command = "\n".join(
        [
            "import os",
            f"payload = {SD_BOOT_STUB_SOURCE!r}",
            "installed = False",
            "for root in ('/sdcard', '/sd'):",
            "    try:",
            "        os.stat(root)",
            "    except OSError:",
            "        continue",
            "    f = open(root + '/main.py', 'w')",
            "    f.write(payload)",
            "    f.close()",
            "    installed = True",
            "    print(root + '/main.py')",
            "    break",
            "try:",
            "    os.sync()",
            "except Exception:",
            "    pass",
            "if not installed:",
            "    print('NO_SD_BOOT_ROOT')",
        ]
    ) + "\n"
    output = session.exec_raw(command, timeout_s=10.0).strip()
    if output == "NO_SD_BOOT_ROOT":
        print("warning: SD boot stub was not installed; no SD root was found")
    elif output:
        print(f"installed SD boot stub: {output}")
    else:
        print("warning: SD boot stub install returned no output")


def stream_flash(config: ProjectConfig, items: list[UploadItem], cleanup_paths: set[str], chunk_size: int, reset: bool, clear_first: bool, install_boot_stub: bool) -> None:
    if not config.serial_port:
        raise RuntimeError("serial port is empty; pass --port or set [serial] port in OpenMV_guidance/openmv.ini")
    print(f"streaming {len(items)} files to /flash via {config.serial_port}")
    with StreamingRawReplSession(config.serial_port, config.serial_baudrate) as session:
        session.start_receiver()
        if clear_first:
            clear_flash_storage(session)
            print("cleared /flash before upload")
        for index, item in enumerate(items, start=1):
            upload_file(session, item, chunk_size)
            print(f"uploaded {index}/{len(items)} /flash/{item.rel_path} ({len(item.payload)} bytes)")
        send_cleanup_paths(session, cleanup_paths)
        commit_files(session, items)
        session.finish_receiver()
        print("committed files to /flash")
        if install_boot_stub:
            install_sd_boot_stub(session)
        if reset:
            session.hard_reset()
            print("triggered hard reset to run /flash/main.py")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stream-upload OpenMV Python files to internal /flash")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="path to the OpenMV project config")
    parser.add_argument("--mpy", action="store_true", help="compile non-entry modules to .mpy with mpy-cross")
    parser.add_argument("--mpy-cross", default="", help="override the mpy-cross executable path")
    parser.add_argument("--port", default="", help="override [serial] port from config")
    parser.add_argument("--baudrate", type=int, default=0, help="override [serial] baudrate from config")
    parser.add_argument("--chunk-size", type=int, default=256, help="raw payload bytes per acknowledged transfer chunk")
    parser.add_argument("--yes", action="store_true", help="clear OpenMV /flash before uploading")
    parser.add_argument("--include-debug", action="store_true", help="also upload stream_debug.py and tuner_runtime.py to /flash")
    parser.add_argument("--no-reset", action="store_true", help="leave the board in raw REPL after upload")
    parser.add_argument("--no-sd-boot-stub", action="store_true", help="skip installing the minimal SD main.py trampoline required by this firmware")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    config = load_config(Path(args.config).resolve())
    effective_config = ProjectConfig(
        project_dir=config.project_dir,
        src_dir=config.src_dir,
        entry_script=config.entry_script,
        build_dir=config.build_dir,
        remote_manifest=config.remote_manifest,
        serial_port=args.port or config.serial_port,
        serial_baudrate=int(args.baudrate or config.serial_baudrate),
        stlink_openocd=config.stlink_openocd,
        stlink_interface_cfg=config.stlink_interface_cfg,
        stlink_target_cfg=config.stlink_target_cfg,
        stlink_transport=config.stlink_transport,
        stlink_firmware_image=config.stlink_firmware_image,
        stlink_firmware_address=config.stlink_firmware_address,
    )
    items, cleanup_paths = build_upload_items(
        effective_config,
        bool(args.mpy),
        str(args.mpy_cross),
        include_debug=bool(args.include_debug),
    )
    stream_flash(
        effective_config,
        items,
        cleanup_paths,
        chunk_size=max(32, int(args.chunk_size)),
        reset=not bool(args.no_reset),
        clear_first=bool(args.yes),
        install_boot_stub=not bool(args.no_sd_boot_stub),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
