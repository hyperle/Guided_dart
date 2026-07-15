#!/usr/bin/env python3
"""ESP32 BLE telemetry client for target, 3-axis velocity, and 3-axis acceleration."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import csv
import os
import struct
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

from bleak import BleakClient, BleakScanner

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
SNAPSHOT_CHARACTERISTIC_UUID = "7b3f5a10-2d36-4c8f-9a7f-3a2d542fd1b2"
STATUS_CHARACTERISTIC_UUID = "3d7f7b30-5602-4f2d-9d45-1f1be1b4c001"
LEGACY_CHARACTERISTIC_UUIDS = {
    "beb5483e-36e1-4688-b7f5-ea07361b26a8",
    "9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21",
    "de24d570-5f81-4d6b-8e4d-2f1309367d91",
}
DEFAULT_DEVICE_NAME = "Dart_1"
DEFAULT_SCAN_TIMEOUT = 10.0
DEFAULT_RECONNECT_DELAY = 1.5
DEFAULT_REFRESH_HZ = 10.0
DEFAULT_LOG_DIR = "logs"

SNAPSHOT_FORMAT = "<HBBHHIIffffffffHfffH"
SNAPSHOT_SIZE = struct.calcsize(SNAPSHOT_FORMAT)
SNAPSHOT_MAGIC = 0xDA7A
SNAPSHOT_VERSION = 2
SNAPSHOT_TYPE = 1

FLAG_TARGET_VALID = 1 << 0
FLAG_GUIDANCE_SEEN = 1 << 1
FLAG_MOTION_SEEN = 1 << 2
FLAG_ACCEL_SEEN = 1 << 3
FLAG_TARGET_LOST = 1 << 4
FLAG_GUIDANCE_STALE = 1 << 5
FLAG_MOTION_STALE = 1 << 6

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class TelemetrySnapshot:
    sequence: int
    flags: int
    esp_time_ms: int
    source_time_ms: int
    target_x: float
    target_y: float
    velocity: Vector3
    acceleration: Vector3
    dart_launch_counter_ticks: int
    dart_launch_velocity: Vector3
    received_at: float
    wall_time: datetime

    @property
    def target_valid(self) -> bool:
        return bool(self.flags & FLAG_TARGET_VALID)

    @property
    def source_age_ms(self) -> int:
        if self.esp_time_ms >= self.source_time_ms:
            return self.esp_time_ms - self.source_time_ms
        return 0


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


class SnapshotDecodeError(ValueError):
    pass


def normalize_ble_address(address: str) -> str:
    return address.strip().upper()


def bluez_cached_devices() -> list[tuple[str, str]]:
    try:
        result = subprocess.run(
            ["bluetoothctl", "devices"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return []

    devices: list[tuple[str, str]] = []
    for raw_line in result.stdout.splitlines():
        line = raw_line.strip()
        if not line.startswith("Device "):
            continue
        parts = line.split(maxsplit=2)
        if len(parts) >= 3:
            devices.append((normalize_ble_address(parts[1]), parts[2].strip()))
    return devices


def bluez_cached_name_for_address(address: str) -> Optional[str]:
    normalized = normalize_ble_address(address)
    for cached_address, name in bluez_cached_devices():
        if cached_address == normalized:
            return name
    return None


def bluez_cached_address_for_name(device_name: str) -> Optional[str]:
    target = device_name.lower()
    for address, name in bluez_cached_devices():
        if name.lower() == target:
            return address
    return None


class SnapshotDecoder:
    def decode(self, data: bytes | bytearray) -> TelemetrySnapshot:
        raw = bytes(data)
        if len(raw) != SNAPSHOT_SIZE:
            raise SnapshotDecodeError(f"unexpected snapshot size {len(raw)}, expected {SNAPSHOT_SIZE}")

        *fields, received_crc = struct.unpack(SNAPSHOT_FORMAT, raw)
        calculated_crc = crc16_ccitt(raw[:-2])
        if calculated_crc != received_crc:
            raise SnapshotDecodeError(f"crc mismatch: got 0x{received_crc:04X}, calc 0x{calculated_crc:04X}")

        (
            magic,
            version,
            packet_type,
            sequence,
            flags,
            esp_time_ms,
            source_time_ms,
            target_x,
            target_y,
            velocity_x,
            velocity_y,
            velocity_z,
            accel_x,
            accel_y,
            accel_z,
            dart_launch_counter_ticks,
            dart_launch_velocity_x,
            dart_launch_velocity_y,
            dart_launch_velocity_z,
        ) = fields

        if magic != SNAPSHOT_MAGIC:
            raise SnapshotDecodeError(f"bad magic 0x{magic:04X}")
        if version != SNAPSHOT_VERSION:
            raise SnapshotDecodeError(f"unsupported snapshot version {version}")
        if packet_type != SNAPSHOT_TYPE:
            raise SnapshotDecodeError(f"unsupported packet type {packet_type}")

        return TelemetrySnapshot(
            sequence=sequence,
            flags=flags,
            esp_time_ms=esp_time_ms,
            source_time_ms=source_time_ms,
            target_x=target_x,
            target_y=target_y,
            velocity=(velocity_x, velocity_y, velocity_z),
            acceleration=(accel_x, accel_y, accel_z),
            dart_launch_counter_ticks=dart_launch_counter_ticks,
            dart_launch_velocity=(dart_launch_velocity_x, dart_launch_velocity_y, dart_launch_velocity_z),
            received_at=time.monotonic(),
            wall_time=datetime.now().astimezone(),
        )


class CsvTelemetryLogger:
    def __init__(self, output_dir: Path):
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        self.path = output_dir / f"ble_telemetry_{timestamp}.csv"
        self._file = self.path.open("x", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                "wall_time",
                "sequence",
                "flags",
                "esp_time_ms",
                "source_time_ms",
                "target_valid",
                "target_x",
                "target_y",
                "velocity_x_dps",
                "velocity_y_dps",
                "velocity_z_dps",
                "accel_x_g",
                "accel_y_g",
                "accel_z_g",
                "dart_launch_counter_ticks",
                "dart_launch_velocity_x_dps",
                "dart_launch_velocity_y_dps",
                "dart_launch_velocity_z_dps",
            ]
        )
        self._file.flush()

    def append(self, sample: TelemetrySnapshot) -> None:
        self._writer.writerow(
            [
                sample.wall_time.isoformat(timespec="milliseconds"),
                sample.sequence,
                f"0x{sample.flags:04X}",
                sample.esp_time_ms,
                sample.source_time_ms,
                int(sample.target_valid),
                f"{sample.target_x:.6f}",
                f"{sample.target_y:.6f}",
                f"{sample.velocity[0]:.6f}",
                f"{sample.velocity[1]:.6f}",
                f"{sample.velocity[2]:.6f}",
                f"{sample.acceleration[0]:.6f}",
                f"{sample.acceleration[1]:.6f}",
                f"{sample.acceleration[2]:.6f}",
                sample.dart_launch_counter_ticks,
                f"{sample.dart_launch_velocity[0]:.6f}",
                f"{sample.dart_launch_velocity[1]:.6f}",
                f"{sample.dart_launch_velocity[2]:.6f}",
            ]
        )
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class PipeWriter:
    def __init__(self, pipe_path: str):
        self.path = pipe_path
        self._fd: Optional[int] = None
        dirname = os.path.dirname(pipe_path)
        if dirname:
            os.makedirs(dirname, exist_ok=True)
        with contextlib.suppress(OSError):
            os.unlink(pipe_path)
        os.mkfifo(pipe_path)

    def write(self, data: bytes) -> None:
        if self._fd is None:
            try:
                self._fd = os.open(self.path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:
                return
        try:
            os.write(self._fd, data)
        except (BlockingIOError, BrokenPipeError, OSError):
            with contextlib.suppress(OSError):
                os.close(self._fd)
            self._fd = None

    def close(self) -> None:
        if self._fd is not None:
            with contextlib.suppress(OSError):
                os.close(self._fd)
            self._fd = None


class TelemetryStore:
    def __init__(self):
        self.latest_sample: Optional[TelemetrySnapshot] = None
        self.packet_count = 0
        self.dropped_packets = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.status_text = ""
        self.connected = False
        self.device_address: Optional[str] = None
        self._last_sequence: Optional[int] = None
        self._rx_times: deque[float] = deque(maxlen=200)
        self._last_error_print = 0.0

    def update_sample(self, sample: TelemetrySnapshot) -> None:
        if self._last_sequence is not None:
            expected = (self._last_sequence + 1) & 0xFFFF
            if sample.sequence != expected:
                self.dropped_packets += (sample.sequence - expected) & 0xFFFF
        self._last_sequence = sample.sequence
        self.latest_sample = sample
        self.packet_count += 1
        self._rx_times.append(sample.received_at)

    def rx_hz(self) -> float:
        if len(self._rx_times) < 2:
            return 0.0
        elapsed = self._rx_times[-1] - self._rx_times[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self._rx_times) - 1) / elapsed

    def record_decode_error(self, message: str) -> None:
        if "crc" in message.lower():
            self.crc_errors += 1
        else:
            self.format_errors += 1
        now = time.monotonic()
        if now - self._last_error_print >= 2.0:
            print(f"BLE snapshot decode error: {message}")
            self._last_error_print = now


class MatplotlibPlotter:
    def __init__(self, window_seconds: float = 10.0):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.window_seconds = window_seconds
        self.start_time = time.monotonic()
        self.times: deque[float] = deque(maxlen=1000)
        self.target_x: deque[float] = deque(maxlen=1000)
        self.target_y: deque[float] = deque(maxlen=1000)
        self.velocity = [deque(maxlen=1000) for _ in range(3)]
        self.acceleration = [deque(maxlen=1000) for _ in range(3)]

        plt.ion()
        self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        self.fig.canvas.manager.set_window_title("Dart BLE Telemetry")
        self.lines = []
        self.lines.extend(self.axes[0].plot([], [], label="target_x"))
        self.lines.extend(self.axes[0].plot([], [], label="target_y"))
        for axis_name, values in zip(("vx", "vy", "vz"), self.velocity):
            self.lines.extend(self.axes[1].plot([], [], label=axis_name))
        for axis_name, values in zip(("ax", "ay", "az"), self.acceleration):
            self.lines.extend(self.axes[2].plot([], [], label=axis_name))
        self.axes[0].set_ylabel("target px")
        self.axes[1].set_ylabel("deg/s")
        self.axes[2].set_ylabel("g")
        self.axes[2].set_xlabel("time s")
        for axis in self.axes:
            axis.grid(True, alpha=0.25)
            axis.legend(loc="upper left")
        self.fig.tight_layout()
        self._last_draw = 0.0

    def update(self, sample: TelemetrySnapshot) -> None:
        t = sample.received_at - self.start_time
        self.times.append(t)
        self.target_x.append(sample.target_x)
        self.target_y.append(sample.target_y)
        for index, value in enumerate(sample.velocity):
            self.velocity[index].append(value)
        for index, value in enumerate(sample.acceleration):
            self.acceleration[index].append(value)

    def draw(self) -> None:
        now = time.monotonic()
        if now - self._last_draw < 0.05:
            return
        self._last_draw = now
        x_values = list(self.times)
        if not x_values:
            return

        line_index = 0
        for values in (self.target_x, self.target_y):
            self.lines[line_index].set_data(x_values, list(values))
            line_index += 1
        for values in self.velocity:
            self.lines[line_index].set_data(x_values, list(values))
            line_index += 1
        for values in self.acceleration:
            self.lines[line_index].set_data(x_values, list(values))
            line_index += 1

        xmin = max(0.0, x_values[-1] - self.window_seconds)
        xmax = max(self.window_seconds, x_values[-1])
        for axis in self.axes:
            axis.set_xlim(xmin, xmax)
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        self.plt.pause(0.001)


class ESP32TelemetryClient:
    def __init__(
        self,
        device_name: str,
        scan_timeout: float,
        address: Optional[str],
        store: TelemetryStore,
        logger: Optional[CsvTelemetryLogger],
        pipe_writer: Optional[PipeWriter],
    ):
        self.device_name = device_name
        self.scan_timeout = scan_timeout
        self.address = address
        self.store = store
        self.logger = logger
        self.pipe_writer = pipe_writer
        self.decoder = SnapshotDecoder()
        self.client: Optional[BleakClient] = None
        self.disconnect_event = asyncio.Event()
        self.stop_event = asyncio.Event()
        self._logger_error_printed = False

    async def connect(self) -> bool:
        self.disconnect_event.clear()

        if self.address:
            requested_address = normalize_ble_address(self.address)
            print(f"正在按地址连接 BLE 设备: {requested_address}")

            target = await BleakScanner.find_device_by_address(requested_address, timeout=self.scan_timeout)
            if target is not None:
                return await self._connect_target(target, target.name or requested_address, target.address)

            cached_name = bluez_cached_name_for_address(requested_address)
            if cached_name is not None:
                print(f"扫描未发现该地址，但 BlueZ 缓存中存在 {cached_name}，尝试主机缓存直连")
                return await self._connect_target(requested_address, cached_name, requested_address)

            cached_address = bluez_cached_address_for_name(self.device_name)
            if cached_address and cached_address != requested_address:
                print(
                    f"未找到 {requested_address}；BlueZ 缓存中 {self.device_name} 的 BLE 地址是 "
                    f"{cached_address}，改用缓存地址直连"
                )
                return await self._connect_target(cached_address, self.device_name, cached_address)

            print("扫描和 BlueZ 缓存都没有命中，最后尝试直接交给 BlueZ 连接该地址")
            try:
                return await self._connect_target(requested_address, requested_address, requested_address)
            except Exception as exc:
                print(f"BlueZ 直连失败: {exc}")
                await self.disconnect()
                return False

        cached_address = bluez_cached_address_for_name(self.device_name)
        if cached_address:
            print(f"从 BlueZ 缓存找到 {self.device_name}: {cached_address}，优先尝试主机缓存直连")
            try:
                return await self._connect_target(cached_address, self.device_name, cached_address)
            except Exception as exc:
                print(f"BlueZ 缓存直连失败，改为主动扫描: {exc}")
                await self.disconnect()

        print(f"正在搜索 BLE 设备: {self.device_name}")
        target = await self.find_target_device()
        if target is None:
            print(f"未找到设备: {self.device_name}")
            return False
        return await self._connect_target(target, target.name or self.device_name, target.address)

    async def _connect_target(self, connect_target, display_name: str, address_hint: str) -> bool:
        self.client = BleakClient(connect_target, disconnected_callback=self._on_disconnected, timeout=self.scan_timeout)
        await self.client.connect()
        self.store.connected = True
        self.store.device_address = normalize_ble_address(address_hint)
        print(f"已连接到 {display_name} ({self.store.device_address})")
        return True

    async def find_target_device(self):
        devices = await BleakScanner.discover(timeout=self.scan_timeout, return_adv=True)
        service_candidates = []
        for device, advertisement in devices.values():
            names = {
                (device.name or "").lower(),
                (getattr(advertisement, "local_name", None) or "").lower(),
            }
            if self.device_name.lower() in names:
                return device
            service_uuids = {uuid.lower() for uuid in (getattr(advertisement, "service_uuids", None) or [])}
            if SERVICE_UUID.lower() in service_uuids:
                service_candidates.append(device)

        if len(service_candidates) == 1:
            candidate = service_candidates[0]
            print(f"按服务 UUID 找到唯一候选设备: {candidate.address}")
            return candidate

        if len(service_candidates) > 1:
            print("发现多个带目标服务 UUID 的设备，请用 --address 指定：")
            for candidate in service_candidates:
                print(f"- {candidate.name or '<无设备名>'} ({candidate.address})")
        else:
            print_discovery_hint(devices)
        return None

    async def enable_notifications(self) -> None:
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE 设备尚未连接")

        services = self.client.services
        snapshot_char = services.get_characteristic(SNAPSHOT_CHARACTERISTIC_UUID)
        if snapshot_char is None:
            print_available_characteristics(self.client)
            if self._has_legacy_characteristics():
                address = self.store.device_address or self.address or "<BLE地址>"
                raise RuntimeError(
                    "当前设备暴露的是旧版 GATT 表，BlueZ 可能缓存了刷固件前的服务。"
                    f"请执行 bluetoothctl remove {address} 后重新运行 ble-connect"
                )
            raise RuntimeError(
                "当前设备没有 telemetry snapshot 特征；请确认 ESP32 已刷入新的 bridge 固件"
            )


        await self.client.start_notify(SNAPSHOT_CHARACTERISTIC_UUID, self._snapshot_handler)
        status_char = services.get_characteristic(STATUS_CHARACTERISTIC_UUID)
        if status_char is not None:
            await self.client.start_notify(STATUS_CHARACTERISTIC_UUID, self._status_handler)
        print("已订阅 telemetry snapshot 通知")

    def _has_legacy_characteristics(self) -> bool:
        if not self.client or self.client.services is None:
            return False
        for service in self.client.services:
            for characteristic in service.characteristics:
                if characteristic.uuid.lower() in LEGACY_CHARACTERISTIC_UUIDS:
                    return True
        return False

    async def disconnect(self) -> None:
        if self.client is not None:
            with contextlib.suppress(Exception):
                if self.client.is_connected:
                    await self.client.disconnect()
            self.client = None
        self.store.connected = False

    def _snapshot_handler(self, sender, data: bytearray) -> None:
        del sender
        raw = bytes(data)
        if self.pipe_writer is not None:
            self.pipe_writer.write(raw)
        try:
            sample = self.decoder.decode(raw)
        except SnapshotDecodeError as exc:
            self.store.record_decode_error(str(exc))
            return

        self.store.update_sample(sample)
        if self.logger is not None:
            try:
                self.logger.append(sample)
            except Exception as exc:
                if not self._logger_error_printed:
                    print(f"日志写入失败，后续继续监视: {exc}")
                    self._logger_error_printed = True

    def _status_handler(self, sender, data: bytearray) -> None:
        del sender
        self.store.status_text = bytes(data).decode("utf-8", errors="replace")

    def _on_disconnected(self, client: BleakClient) -> None:
        del client
        self.store.connected = False
        print("BLE 连接已断开")
        self.disconnect_event.set()


async def display_loop(store: TelemetryStore, refresh_hz: float, plotter: Optional[MatplotlibPlotter]) -> None:
    interval = 1.0 / refresh_hz if refresh_hz > 0 else 0.1
    last_sequence: Optional[int] = None
    while True:
        sample = store.latest_sample
        if sample is not None and sample.sequence != last_sequence:
            if plotter is not None:
                plotter.update(sample)
            last_sequence = sample.sequence

        render_dashboard(store)
        if plotter is not None:
            plotter.draw()
        await asyncio.sleep(interval)


def render_dashboard(store: TelemetryStore) -> None:
    sample = store.latest_sample
    if sample is None and not store.connected:
        return
    sys.stdout.write("\x1b[H\x1b[J")
    connected = "connected" if store.connected else "disconnected"
    print("Dart BLE Telemetry")
    print(f"state={connected} address={store.device_address or '-'} rx_hz={store.rx_hz():5.1f}")
    print(
        f"packets={store.packet_count} drops={store.dropped_packets} "
        f"crc_errors={store.crc_errors} format_errors={store.format_errors}"
    )

    if sample is None:
        print("\n等待 telemetry snapshot...")
        if store.status_text:
            print(f"\nESP32 status: {store.status_text}")
        sys.stdout.flush()
        return

    age_ms = (time.monotonic() - sample.received_at) * 1000.0
    target_text = f"<{sample.target_x:.1f}, {sample.target_y:.1f}>" if sample.target_valid else "<-1, -1>"
    flags = format_flags(sample.flags)

    print(
        f"seq={sample.sequence:05d} age={age_ms:6.0f}ms "
        f"esp_time={sample.esp_time_ms}ms source_age={sample.source_age_ms}ms flags={flags}"
    )
    print(f"target {target_text}")
    print()
    print("axis        X          Y          Z")
    print(
        "velocity "
        f"{sample.velocity[0]:10.3f} {sample.velocity[1]:10.3f} {sample.velocity[2]:10.3f} deg/s"
    )
    print(
        "accel    "
        f"{sample.acceleration[0]:10.3f} {sample.acceleration[1]:10.3f} {sample.acceleration[2]:10.3f} g"
    )
    print(
        f"launch_tick={sample.dart_launch_counter_ticks:04d} "
        "tick10_speed "
        f"{sample.dart_launch_velocity[0]:10.3f} "
        f"{sample.dart_launch_velocity[1]:10.3f} "
        f"{sample.dart_launch_velocity[2]:10.3f} deg/s"
    )
    if store.status_text:
        print(f"\nESP32 status: {store.status_text}")
    sys.stdout.flush()


def format_flags(flags: int) -> str:
    names = []
    if flags & FLAG_TARGET_VALID:
        names.append("target")
    if flags & FLAG_GUIDANCE_SEEN:
        names.append("guidance")
    if flags & FLAG_MOTION_SEEN:
        names.append("motion")
    if flags & FLAG_ACCEL_SEEN:
        names.append("accel")
    if flags & FLAG_TARGET_LOST:
        names.append("lost")
    if flags & FLAG_GUIDANCE_STALE:
        names.append("guidance_stale")
    if flags & FLAG_MOTION_STALE:
        names.append("motion_stale")
    return ",".join(names) if names else "none"


def print_available_characteristics(client: BleakClient) -> None:
    print("当前设备暴露的服务/特征如下：")
    for service in client.services:
        print(f"- service {service.uuid}")
        for characteristic in service.characteristics:
            print(f"  - char {characteristic.uuid} ({','.join(characteristic.properties)})")


def print_discovery_hint(devices) -> None:
    print("扫描到了这些 BLE 设备（可用 --address 指定）：")
    shown_any = False
    for device, advertisement in devices.values():
        local_name = getattr(advertisement, "local_name", None)
        service_uuids = getattr(advertisement, "service_uuids", None) or []
        if local_name or device.name or service_uuids:
            shown_any = True
            print(
                f"- name={device.name or '<空>'}, adv_name={local_name or '<空>'}, "
                f"addr={device.address}, uuids={service_uuids}"
            )
    if not shown_any:
        print("- 当前扫描结果中没有可用的广播名/服务 UUID 信息")


async def scan_and_select(timeout: float, prefix: str = "") -> tuple[Optional[str], Optional[str]]:
    print(f"正在扫描 BLE 设备 ({timeout:.0f}s)...\n")
    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
    if not devices:
        print("未发现任何 BLE 设备")
        return None, None

    entries = []
    for idx, (device, adv) in enumerate(devices.values(), start=1):
        name = device.name or getattr(adv, "local_name", None) or "<未命名>"
        rssi = getattr(adv, "rssi", 0) or 0
        uuids = [str(uuid) for uuid in (getattr(adv, "service_uuids", None) or [])]
        entries.append((idx, name, str(device.address), rssi, uuids))

    if prefix:
        filtered = [entry for entry in entries if entry[1].lower().startswith(prefix.lower())]
        if filtered:
            entries = filtered
        else:
            print(f"未找到匹配前缀 '{prefix}' 的设备，显示全部：\n")

    print(f"{'#':>3}  {'设备名称':<30} {'地址':<18} {'RSSI':>5}")
    print("-" * 70)
    for idx, name, addr, rssi, uuids in entries:
        print(f"{idx:>3}  {name:<30} {addr:<18} {rssi:>4}dBm  [{', '.join(uuids) if uuids else '-'}]")

    if len(entries) == 1:
        _, name, addr, _, _ = entries[0]
        return name, addr

    while True:
        try:
            raw = input("选择设备编号 (q 退出): ").strip()
            if raw.lower() == "q":
                return None, None
            selected = int(raw)
            for idx, name, addr, _, _ in entries:
                if idx == selected:
                    return name, addr
            print("编号不在列表中")
        except ValueError:
            print("请输入有效数字")
        except (EOFError, KeyboardInterrupt):
            return None, None


async def run_client(args: argparse.Namespace) -> None:
    logger = None if args.no_log else CsvTelemetryLogger(Path(args.log_dir))
    pipe_writer = PipeWriter(args.pipe) if args.pipe else None
    store = TelemetryStore()
    plotter = None
    if args.plot:
        plotter = MatplotlibPlotter(window_seconds=args.plot_window)

    client = ESP32TelemetryClient(
        device_name=args.device_name,
        scan_timeout=args.scan_timeout,
        address=args.address,
        store=store,
        logger=logger,
        pipe_writer=pipe_writer,
    )

    print("启动主机端 BLE telemetry 可视化")
    print(f"目标设备名: {args.device_name}")
    if args.address:
        print(f"指定蓝牙地址: {args.address}")
    if logger is not None:
        print(f"CSV 日志: {logger.path}")
    if pipe_writer is not None:
        print(f"snapshot 管道: {pipe_writer.path}")

    display_task = asyncio.create_task(display_loop(store, args.refresh, plotter))
    try:
        while not client.stop_event.is_set():
            try:
                connected = await client.connect()
                if not connected:
                    await asyncio.sleep(DEFAULT_RECONNECT_DELAY)
                    continue
                await client.enable_notifications()
                await client.disconnect_event.wait()
            except KeyboardInterrupt:
                client.stop_event.set()
            except Exception as exc:
                print(f"运行出错: {exc}")
            finally:
                await client.disconnect()

            if not client.stop_event.is_set():
                print(f"{DEFAULT_RECONNECT_DELAY:.1f} 秒后尝试重连...")
                await asyncio.sleep(DEFAULT_RECONNECT_DELAY)
    finally:
        display_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await display_task
        await client.disconnect()
        if logger is not None:
            logger.close()
        if pipe_writer is not None:
            pipe_writer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="接收并可视化 ESP32 BLE telemetry snapshot")
    parser.add_argument("--scan", action="store_true", help="扫描附近 BLE 设备并交互选择")
    parser.add_argument("--prefix", default="", help="扫描模式下按设备名前缀过滤，如 Dart_")
    parser.add_argument("--device-name", default=DEFAULT_DEVICE_NAME, help=f"BLE 设备名，默认 {DEFAULT_DEVICE_NAME}")
    parser.add_argument("--address", help="BLE 设备地址；扫不到名字时建议直接指定")
    parser.add_argument("--scan-timeout", type=float, default=DEFAULT_SCAN_TIMEOUT, help="BLE 扫描超时秒数")
    parser.add_argument("--refresh", type=float, default=DEFAULT_REFRESH_HZ, help="终端刷新频率 Hz")
    parser.add_argument("--plot", action="store_true", help="启用 matplotlib 实时曲线窗口")
    parser.add_argument("--plot-window", type=float, default=10.0, help="曲线显示窗口秒数")
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR, help=f"CSV 日志目录，默认 {DEFAULT_LOG_DIR}")
    parser.add_argument("--no-log", action="store_true", help="不写 CSV 日志")
    parser.add_argument("--pipe", default="", help="创建 FIFO 并写入原始 50 字节 snapshot 包")
    return parser.parse_args()


async def main() -> None:
    args = parse_args()
    if args.scan:
        selected_name, selected_addr = await scan_and_select(args.scan_timeout, args.prefix)
        if selected_addr is None:
            print("未选择设备，退出")
            return
        args.device_name = selected_name or args.device_name
        args.address = selected_addr
        print(f"\n已选择: {args.device_name} ({args.address})\n")
    await run_client(args)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print()
        print("已退出 BLE telemetry 可视化")
