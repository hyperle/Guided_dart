#!/usr/bin/env python3
"""本地 BLE 客户端：接收 ESP32 推送的 IMU 数据并在终端展示。"""

import argparse
import asyncio
import contextlib
import math
import os
import struct
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

from bleak import BleakClient, BleakScanner

# 与 ESP32 代码中相同的 UUID
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CONTROL_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
IMU_CHARACTERISTIC_UUID = "9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21"
ACK_CHARACTERISTIC_UUID = "de24d570-5f81-4d6b-8e4d-2f1309367d91"
STATUS_CHARACTERISTIC_UUID = "3d7f7b30-5602-4f2d-9d45-1f1be1b4c001"
DEFAULT_DEVICE_NAME = "Dart_1"
DEFAULT_SCAN_TIMEOUT = 10.0
DEFAULT_RECONNECT_DELAY = 2.0
DEFAULT_REFRESH_HZ = 5.0
DEFAULT_LOG_DIR = "logs"
EXTENDED_IMU_PACKET_FORMAT = "<hhhhhhfff"
LEGACY_IMU_PACKET_FORMAT = "<hhhhhh"
EXTENDED_IMU_PACKET_SIZE = struct.calcsize(EXTENDED_IMU_PACKET_FORMAT)
LEGACY_IMU_PACKET_SIZE = struct.calcsize(LEGACY_IMU_PACKET_FORMAT)
Vector3 = tuple[float, float, float]

@dataclass
class ImuSample:
    """一次 IMU 数据快照。"""

    sequence: int
    monotonic_time: float
    wall_time: datetime
    packet_mode: str
    used_host_angular_accel: bool
    roll_raw: int
    pitch_raw: int
    yaw_raw: int
    gx_raw: int
    gy_raw: int
    gz_raw: int
    roll: float
    pitch: float
    yaw: float
    gx: float
    gy: float
    gz: float
    alpha_x: float
    alpha_y: float
    alpha_z: float
    body_x_world: Vector3
    body_y_world: Vector3
    body_z_world: Vector3
    angular_velocity_world: Vector3
    angular_accel_world: Vector3


def rotate_vector(matrix: tuple[Vector3, Vector3, Vector3], vector: Vector3) -> Vector3:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def body_to_world_rotation(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[Vector3, Vector3, Vector3]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def transform_to_cartesian(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    angular_velocity_dps: Vector3,
    angular_accel_dps2: Vector3,
) -> tuple[Vector3, Vector3, Vector3, Vector3, Vector3]:
    rotation = body_to_world_rotation(roll_deg, pitch_deg, yaw_deg)
    body_x_world = (rotation[0][0], rotation[1][0], rotation[2][0])
    body_y_world = (rotation[0][1], rotation[1][1], rotation[2][1])
    body_z_world = (rotation[0][2], rotation[1][2], rotation[2][2])
    angular_velocity_world = rotate_vector(rotation, angular_velocity_dps)
    angular_accel_world = rotate_vector(rotation, angular_accel_dps2)
    return (
        body_x_world,
        body_y_world,
        body_z_world,
        angular_velocity_world,
        angular_accel_world,
    )


def build_log_path(output_dir: Path) -> Path:
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    base_name = f"imu_session_{timestamp}"
    candidate = output_dir / f"{base_name}.md"
    suffix = 1

    while candidate.exists():
        candidate = output_dir / f"{base_name}_{suffix:02d}.md"
        suffix += 1

    return candidate


class MarkdownSessionLogger:
    """将原始数据和处理后的数据持久化到 Markdown 文件。"""

    def __init__(
        self,
        output_dir: Path,
        device_name: str,
        address: Optional[str],
    ):
        output_dir.mkdir(parents=True, exist_ok=True)
        self.path = build_log_path(output_dir)
        self._file = self.path.open("x", encoding="utf-8")
        self._closed = False
        self._write_header(device_name, address)

    def _write_header(self, device_name: str, address: Optional[str]) -> None:
        started_at = datetime.now().astimezone()
        self._file.write("# IMU 监视日志\n\n")
        self._file.write(f"- 启动时间: `{started_at.isoformat(timespec='seconds')}`\n")
        self._file.write(f"- 目标设备名: `{device_name}`\n")
        self._file.write(f"- BLE 地址: `{address or '自动扫描'}`\n")
        self._file.write("- 处理方式: 将姿态角生成的机体系坐标轴转换到世界直角坐标系，并将角速度/角加速度旋转到同一坐标系。\n")
        self._file.write(
            "- 协议兼容: 优先使用 ESP32 扩展 24 字节包；若收到旧版 12 字节包，则角加速度由主机端差分近似。\n\n"
        )
        self._file.write("## 数据\n\n")
        self._file.write(
            "| seq | timestamp | packet | alpha_src | roll_raw | pitch_raw | yaw_raw | "
            "gx_raw | gy_raw | gz_raw | roll_deg | pitch_deg | yaw_deg | "
            "gx_dps | gy_dps | gz_dps | alpha_x_dps2 | alpha_y_dps2 | alpha_z_dps2 | "
            "body_x_world_x | body_x_world_y | body_x_world_z | "
            "body_y_world_x | body_y_world_y | body_y_world_z | "
            "body_z_world_x | body_z_world_y | body_z_world_z | "
            "omega_world_x_dps | omega_world_y_dps | omega_world_z_dps | "
            "alpha_world_x_dps2 | alpha_world_y_dps2 | alpha_world_z_dps2 |\n"
        )
        self._file.write(
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
            "--- | --- | --- | --- | --- | --- | --- | --- | --- | "
            "--- | --- | --- | --- | --- | --- | --- | --- | --- | "
            "--- | --- | --- | --- | --- | --- |\n"
        )
        self._file.flush()

    @staticmethod
    def _fmt_float(value: float) -> str:
        return f"{value:.6f}"

    def append_sample(self, sample: ImuSample) -> None:
        body_x_world = sample.body_x_world
        body_y_world = sample.body_y_world
        body_z_world = sample.body_z_world
        angular_velocity_world = sample.angular_velocity_world
        angular_accel_world = sample.angular_accel_world
        alpha_source = "host" if sample.used_host_angular_accel else "esp32"

        row = (
            f"| {sample.sequence} | {sample.wall_time.isoformat(timespec='milliseconds')} | {sample.packet_mode} | "
            f"{alpha_source} | {sample.roll_raw} | {sample.pitch_raw} | {sample.yaw_raw} | "
            f"{sample.gx_raw} | {sample.gy_raw} | {sample.gz_raw} | "
            f"{self._fmt_float(sample.roll)} | {self._fmt_float(sample.pitch)} | {self._fmt_float(sample.yaw)} | "
            f"{self._fmt_float(sample.gx)} | {self._fmt_float(sample.gy)} | {self._fmt_float(sample.gz)} | "
            f"{self._fmt_float(sample.alpha_x)} | {self._fmt_float(sample.alpha_y)} | {self._fmt_float(sample.alpha_z)} | "
            f"{self._fmt_float(body_x_world[0])} | {self._fmt_float(body_x_world[1])} | {self._fmt_float(body_x_world[2])} | "
            f"{self._fmt_float(body_y_world[0])} | {self._fmt_float(body_y_world[1])} | {self._fmt_float(body_y_world[2])} | "
            f"{self._fmt_float(body_z_world[0])} | {self._fmt_float(body_z_world[1])} | {self._fmt_float(body_z_world[2])} | "
            f"{self._fmt_float(angular_velocity_world[0])} | {self._fmt_float(angular_velocity_world[1])} | {self._fmt_float(angular_velocity_world[2])} | "
            f"{self._fmt_float(angular_accel_world[0])} | {self._fmt_float(angular_accel_world[1])} | {self._fmt_float(angular_accel_world[2])} |\n"
        )
        self._file.write(row)
        self._file.flush()

    def close(self) -> None:
        if self._closed:
            return

        finished_at = datetime.now().astimezone()
        self._file.write(f"\n- 会话结束: `{finished_at.isoformat(timespec='seconds')}`\n")
        self._file.close()
        self._closed = True


class ImuPacketDecoder:
    """解析 BLE IMU 包，并在需要时做兼容差分。"""

    def __init__(self):
        self._previous_legacy_time: Optional[float] = None
        self._previous_legacy_gyro: Optional[Vector3] = None
        self._legacy_warning_printed = False

    @staticmethod
    def _decode_roll(raw_value: int) -> float:
        return raw_value / 32768 * 180

    @staticmethod
    def _decode_gyro(raw_value: int) -> float:
        return raw_value / 32768 * 2000

    def _estimate_legacy_angular_accel(self, sample_time: float, gyro_dps: Vector3) -> Vector3:
        alpha = (0.0, 0.0, 0.0)
        if self._previous_legacy_time is not None and self._previous_legacy_gyro is not None:
            dt_seconds = sample_time - self._previous_legacy_time
            if dt_seconds > 0:
                previous = self._previous_legacy_gyro
                alpha = (
                    (gyro_dps[0] - previous[0]) / dt_seconds,
                    (gyro_dps[1] - previous[1]) / dt_seconds,
                    (gyro_dps[2] - previous[2]) / dt_seconds,
                )

        self._previous_legacy_time = sample_time
        self._previous_legacy_gyro = gyro_dps
        return alpha

    def decode(self, data: bytearray, sequence: int) -> Optional[ImuSample]:
        sample_time = time.monotonic()
        wall_time = datetime.now().astimezone()

        if len(data) == EXTENDED_IMU_PACKET_SIZE:
            packet_mode = "extended24"
            used_host_angular_accel = False
            roll_raw, pitch_raw, yaw_raw, gx_raw, gy_raw, gz_raw, alpha_x, alpha_y, alpha_z = struct.unpack(
                EXTENDED_IMU_PACKET_FORMAT,
                data,
            )
        elif len(data) == LEGACY_IMU_PACKET_SIZE:
            packet_mode = "legacy12"
            used_host_angular_accel = True
            roll_raw, pitch_raw, yaw_raw, gx_raw, gy_raw, gz_raw = struct.unpack(LEGACY_IMU_PACKET_FORMAT, data)
            legacy_gyro = (
                self._decode_gyro(gx_raw),
                self._decode_gyro(gy_raw),
                self._decode_gyro(gz_raw),
            )
            alpha_x, alpha_y, alpha_z = self._estimate_legacy_angular_accel(sample_time, legacy_gyro)

            if not self._legacy_warning_printed:
                print(
                    "收到旧版 12 字节 IMU 包，角加速度正在由主机端按 BLE 到达时间差近似计算；"
                    "刷入新固件后会切换为 ESP32 端计算的扩展包。"
                )
                self._legacy_warning_printed = True
        else:
            return None

        roll = self._decode_roll(roll_raw)
        pitch = self._decode_roll(pitch_raw)
        yaw = self._decode_roll(yaw_raw)
        gx = self._decode_gyro(gx_raw)
        gy = self._decode_gyro(gy_raw)
        gz = self._decode_gyro(gz_raw)

        (
            body_x_world,
            body_y_world,
            body_z_world,
            angular_velocity_world,
            angular_accel_world,
        ) = transform_to_cartesian(
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=yaw,
            angular_velocity_dps=(gx, gy, gz),
            angular_accel_dps2=(alpha_x, alpha_y, alpha_z),
        )

        return ImuSample(
            sequence=sequence,
            monotonic_time=sample_time,
            wall_time=wall_time,
            packet_mode=packet_mode,
            used_host_angular_accel=used_host_angular_accel,
            roll_raw=roll_raw,
            pitch_raw=pitch_raw,
            yaw_raw=yaw_raw,
            gx_raw=gx_raw,
            gy_raw=gy_raw,
            gz_raw=gz_raw,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            gx=gx,
            gy=gy,
            gz=gz,
            alpha_x=alpha_x,
            alpha_y=alpha_y,
            alpha_z=alpha_z,
            body_x_world=body_x_world,
            body_y_world=body_y_world,
            body_z_world=body_z_world,
            angular_velocity_world=angular_velocity_world,
            angular_accel_world=angular_accel_world,
        )


class ESP32BLEClient:
    def __init__(
        self,
        device_name: str,
        scan_timeout: float,
        address: Optional[str],
        decoder: ImuPacketDecoder,
        logger: MarkdownSessionLogger,
        pipe_path: str = "",
    ):
        self.device_name = device_name
        self.scan_timeout = scan_timeout
        self.address = address
        self.decoder = decoder
        self.logger = logger
        self.pipe_path = pipe_path
        self.pipe_fh = None
        self.client: Optional[BleakClient] = None
        self.device_address: Optional[str] = None
        self.latest_sample: Optional[ImuSample] = None
        self.latest_control_message: Optional[str] = None
        self.packet_count = 0
        self.disconnect_event = asyncio.Event()
        self.stop_event = asyncio.Event()
        self.control_queue: asyncio.Queue[str] = asyncio.Queue()
        self._logger_error_printed = False

    async def scan_devices(self):
        """扫描附近 BLE 设备，并携带广播信息。"""
        return await BleakScanner.discover(timeout=self.scan_timeout, return_adv=True)

    async def connect(self) -> bool:
        """搜索并连接到指定 ESP32。"""
        target_device = None
        display_name = self.device_name
        connect_target = None

        if self.address:
            print(f"正在按地址连接 BLE 设备: {self.address}")
            self.device_address = self.address
            display_name = self.address
            connect_target = self.address

            try:
                return await self._connect_target(connect_target, display_name)
            except Exception as exc:
                print(f"按地址直连失败，改为扫描该地址重试: {exc}")

            target_device = await BleakScanner.find_device_by_address(
                self.address,
                timeout=self.scan_timeout,
            )
            if target_device is None:
                print(f"未找到地址为 {self.address} 的设备")
                return False

            self.device_address = target_device.address
            display_name = target_device.name or self.address
            connect_target = target_device
        else:
            print(f"正在搜索 BLE 设备: {self.device_name}")
            target_device = await self.find_target_device()
            if target_device is None:
                print(f"未找到设备: {self.device_name}")
                return False

            self.device_address = target_device.address
            display_name = target_device.name or self.device_name
            connect_target = target_device

        print(f"找到设备: {display_name} ({self.device_address})")
        return await self._connect_target(connect_target, display_name)

    async def find_target_device(self):
        """按名字、广播名、服务 UUID 查找目标设备。"""
        devices = await self.scan_devices()
        candidates_by_service = []

        for device, advertisement in devices.values():
            if self.matches_device_name(device, advertisement):
                return device

            if self.matches_service_uuid(advertisement):
                candidates_by_service.append(device)

        if len(candidates_by_service) == 1:
            device = candidates_by_service[0]
            print(
                "按服务 UUID 找到唯一候选设备，"
                f"其广播名可能不是 {self.device_name}: {device.address}"
            )
            return device

        if len(candidates_by_service) > 1:
            print("发现多个带目标服务 UUID 的设备，请改用 --address 指定：")
            for device in candidates_by_service:
                print(f"- {device.name or '<无设备名>'} ({device.address})")
            return None

        self.print_discovery_hint(devices)
        return None

    def matches_device_name(self, device, advertisement) -> bool:
        names = {
            self.safe_lower(device.name),
            self.safe_lower(getattr(device, "local_name", None)),
            self.safe_lower(getattr(advertisement, "local_name", None)),
        }
        return self.device_name.lower() in names

    def matches_service_uuid(self, advertisement) -> bool:
        service_uuids = getattr(advertisement, "service_uuids", None) or []
        normalized = {uuid.lower() for uuid in service_uuids}
        return SERVICE_UUID.lower() in normalized

    def print_discovery_hint(self, devices) -> None:
        print("扫描到了这些 BLE 设备（名字可能为空，建议确认地址后用 --address）：")
        shown_any = False
        for device, advertisement in devices.values():
            local_name = getattr(advertisement, "local_name", None)
            service_uuids = getattr(advertisement, "service_uuids", None) or []
            if local_name or device.name or service_uuids:
                shown_any = True
                print(
                    f"- name={device.name or '<空>'}, "
                    f"adv_name={local_name or '<空>'}, "
                    f"addr={device.address}, "
                    f"uuids={service_uuids}"
                )
        if not shown_any:
            print("- 当前扫描结果中没有可用的广播名/服务 UUID 信息")

    def print_available_characteristics(self) -> None:
        if not self.client or self.client.services is None:
            return

        print("当前设备暴露的服务/特征如下：")
        for service in self.client.services:
            print(f"- service {service.uuid}")
            for characteristic in service.characteristics:
                properties = ",".join(characteristic.properties)
                print(f"  - char {characteristic.uuid} ({properties})")

    async def _connect_target(self, connect_target, display_name: str) -> bool:
        self.latest_sample = None
        self.latest_control_message = None
        self.packet_count = 0
        self.disconnect_event.clear()
        self.client = BleakClient(
            connect_target,
            disconnected_callback=self._on_disconnected,
        )
        await self.client.connect()
        print(f"已连接到 {display_name}")
        return True

    @staticmethod
    def safe_lower(value: Optional[str]) -> str:
        return value.lower() if value else ""

    async def disconnect(self):
        """断开连接。"""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("已断开连接")

    async def enable_notifications(self):
        """启用控制通道和 IMU 通知。pipe 模式下订阅全部特征。"""
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE 设备尚未连接")

        if self.pipe_path:
            self._setup_pipe()

        services = self.client.services
        control_characteristic = services.get_characteristic(CONTROL_CHARACTERISTIC_UUID)
        imu_characteristic = services.get_characteristic(IMU_CHARACTERISTIC_UUID)

        if control_characteristic is not None:
            await self.client.start_notify(CONTROL_CHARACTERISTIC_UUID, self.control_notification_handler)

        if imu_characteristic is None:
            self.print_available_characteristics()
            raise RuntimeError(
                "当前连接到的设备没有 IMU 特征 9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21；"
                "很可能 ESP32 还在跑旧版 esp32_ble_server.ino，而不是 src/main.cpp"
            )

        await self.client.start_notify(IMU_CHARACTERISTIC_UUID, self.imu_notification_handler)

        if self.pipe_path:
            ack_char = services.get_characteristic(ACK_CHARACTERISTIC_UUID)
            if ack_char is not None:
                await self.client.start_notify(ACK_CHARACTERISTIC_UUID, self._pipe_handler)
            status_char = services.get_characteristic(STATUS_CHARACTERISTIC_UUID)
            if status_char is not None:
                await self.client.start_notify(STATUS_CHARACTERISTIC_UUID, self._pipe_handler)
            print(f"管道已创建: {self.pipe_path} (IMU/Telemetry/Ack/Status 全部转发)")
        else:
            print("已启用 IMU 通知")

    def _setup_pipe(self):
        dirname = os.path.dirname(self.pipe_path)
        if dirname:
            os.makedirs(dirname, exist_ok=True)
        try:
            os.unlink(self.pipe_path)
        except OSError:
            pass
        os.mkfifo(self.pipe_path)
        self._pipe_fifo = self.pipe_path

    def _write_pipe(self, data: bytes):
        if not hasattr(self, '_fifo_fd') or self._fifo_fd is None:
            try:
                self._fifo_fd = os.open(self._pipe_fifo, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:
                return
        try:
            os.write(self._fifo_fd, data)
        except (BlockingIOError, BrokenPipeError, OSError):
            pass

    def _pipe_handler(self, sender: int, data: bytearray):
        self._write_pipe(bytes(data))

    def imu_notification_handler(self, sender: int, data: bytearray):
        """处理 IMU 通知。"""
        if self.pipe_path:
            self._write_pipe(bytes(data))

        sample = self.decoder.decode(data, self.packet_count + 1)
        if sample is None:
            if not self.pipe_path:
                print(f"收到未知 IMU 数据: {len(data)} 字节 -> {data.hex()}")
            return

        self.packet_count = sample.sequence
        self.latest_sample = sample

        try:
            self.logger.append_sample(sample)
        except Exception as exc:
            if not self._logger_error_printed:
                print(f"日志写入失败，后续仅继续监视，不再重复提示: {exc}")
                self._logger_error_printed = True

    async def send_control(self, command: str):
        """向 ESP32 控制特征发送命令。"""
        if not self.client or not self.client.is_connected:
            print("当前未连接，命令未发送")
            return

        await self.client.write_gatt_char(CONTROL_CHARACTERISTIC_UUID, command.encode("utf-8"))
        print(f"已发送命令: {command}")

    def imu_notification_handler(self, sender: int, data: bytearray):
        """处理 IMU 通知。"""
        sample = self.decoder.decode(data, self.packet_count + 1)
        if sample is None:
            print(f"收到未知 IMU 数据: {len(data)} 字节 -> {data.hex()}")
            return

        self.packet_count = sample.sequence
        self.latest_sample = sample

        try:
            self.logger.append_sample(sample)
        except Exception as exc:
            if not self._logger_error_printed:
                print(f"日志写入失败，后续仅继续监视，不再重复提示: {exc}")
                self._logger_error_printed = True

    def control_notification_handler(self, sender: int, data: bytearray):
        """处理控制通道通知。"""
        message = data.decode("utf-8", errors="replace")
        self.latest_control_message = message
        self.control_queue.put_nowait(message)
        print(f"[控制回复] {message}")

    def _on_disconnected(self, client: BleakClient):
        print("BLE 连接已断开")
        self.disconnect_event.set()

    def latest_snapshot_line(self) -> str:
        """将最近的 IMU 数据格式化为单行文本。"""
        sample = self.latest_sample
        if sample is None:
            return "等待 IMU 数据..."

        age_ms = (time.monotonic() - sample.monotonic_time) * 1000.0
        body_x_world = sample.body_x_world
        angular_velocity_world = sample.angular_velocity_world
        angular_accel_world = sample.angular_accel_world
        return (
            f"#{sample.sequence:05d} "
            f"Roll {sample.roll:8.2f}°  "
            f"Pitch {sample.pitch:8.2f}°  "
            f"Yaw {sample.yaw:8.2f}°  |  "
            f"G [{sample.gx:8.2f}, {sample.gy:8.2f}, {sample.gz:8.2f}]°/s  |  "
            f"Alpha [{sample.alpha_x:9.2f}, {sample.alpha_y:9.2f}, {sample.alpha_z:9.2f}]°/s²  |  "
            f"Xw [{body_x_world[0]:7.3f}, {body_x_world[1]:7.3f}, {body_x_world[2]:7.3f}]  |  "
            f"Ow [{angular_velocity_world[0]:8.2f}, {angular_velocity_world[1]:8.2f}, {angular_velocity_world[2]:8.2f}]°/s  |  "
            f"Aw [{angular_accel_world[0]:9.2f}, {angular_accel_world[1]:9.2f}, {angular_accel_world[2]:9.2f}]°/s²  |  "
            f"延迟 {age_ms:6.0f} ms"
        )


async def display_loop(client: ESP32BLEClient, refresh_hz: float):
    """持续打印最新 IMU 数据。"""
    interval = 1.0 / refresh_hz if refresh_hz > 0 else 0.2
    last_sequence = -1
    last_wait_print = 0.0

    while not client.stop_event.is_set() and not client.disconnect_event.is_set():
        sample = client.latest_sample
        now = time.monotonic()

        if sample is None:
            if now - last_wait_print >= 2.0:
                print("等待 IMU 数据...")
                last_wait_print = now
        elif sample.sequence != last_sequence:
            print(client.latest_snapshot_line())
            last_sequence = sample.sequence

        await asyncio.sleep(interval)


async def command_loop(client: ESP32BLEClient):
    """可选交互命令输入。"""
    print("可用命令: help / imu / imu raw / imu cfg / imu rate 10 / quit")

    while not client.stop_event.is_set() and not client.disconnect_event.is_set():
        try:
            command = await asyncio.to_thread(input, "> ")
        except EOFError:
            client.stop_event.set()
            return

        command = command.strip()
        if not command:
            continue

        if command.lower() in {"quit", "exit"}:
            client.stop_event.set()
            return

        await client.send_control(command)


async def run_client(
    device_name: str,
    scan_timeout: float,
    refresh_hz: float,
    interactive: bool,
    address: Optional[str],
    log_dir: Path,
    pipe_path: str = "",
):
    """运行 BLE 监视器，断线后自动重连。"""
    logger = MarkdownSessionLogger(
        output_dir=log_dir,
        device_name=device_name,
        address=address,
    )
    decoder = ImuPacketDecoder()
    client = ESP32BLEClient(
        device_name=device_name,
        scan_timeout=scan_timeout,
        address=address,
        decoder=decoder,
        logger=logger,
        pipe_path=pipe_path,
    )

    print("启动主机端 IMU 监视器")
    print(f"目标设备名: {device_name}")
    if address:
        print(f"指定蓝牙地址: {address}")
    if interactive:
        print("当前模式: 监视 + 命令输入")
    else:
        print("当前模式: 仅监视（按 Ctrl+C 退出）")
    print("当前处理: 转换到世界直角坐标系（姿态轴向量 + 世界系角速度/角加速度）")
    print(f"Markdown 日志: {logger.path}")

    try:
        while not client.stop_event.is_set():
            try:
                connected = await client.connect()
                if not connected:
                    await asyncio.sleep(DEFAULT_RECONNECT_DELAY)
                    continue

                await client.enable_notifications()
                display_task = asyncio.create_task(display_loop(client, refresh_hz))
                command_task = None
                if interactive:
                    command_task = asyncio.create_task(command_loop(client))

                wait_tasks = [asyncio.create_task(client.disconnect_event.wait())]
                if command_task is not None:
                    wait_tasks.append(command_task)

                _, pending = await asyncio.wait(wait_tasks, return_when=asyncio.FIRST_COMPLETED)

                for task in pending:
                    task.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await task

                display_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await display_task

                if client.stop_event.is_set():
                    break

                print(f"{DEFAULT_RECONNECT_DELAY:.0f} 秒后尝试重连...\n")
                await client.disconnect()
                await asyncio.sleep(DEFAULT_RECONNECT_DELAY)
            except KeyboardInterrupt:
                client.stop_event.set()
                break
            except Exception as exc:
                print(f"运行出错: {exc}")
                await client.disconnect()
                await asyncio.sleep(DEFAULT_RECONNECT_DELAY)
    finally:
        await client.disconnect()
        logger.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="接收并展示 ESP32 推送的 IMU BLE 数据")
    parser.add_argument(
        "--pipe",
        default="",
        help="Unix socket 路径：将所有 BLE 通知写入管道，供其他工具（如 track-viewer）读取",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="扫描模式：列出附近所有 BLE 设备，交互选择后自动连接",
    )
    parser.add_argument(
        "--prefix",
        default="",
        help="批量设备名称前缀过滤，如 Dart_BLE_（仅扫描模式生效）",
    )
    parser.add_argument(
        "--device-name",
        default=DEFAULT_DEVICE_NAME,
        help=f"BLE 设备名，默认 {DEFAULT_DEVICE_NAME}",
    )
    parser.add_argument(
        "--address",
        help="BLE 设备地址；如果系统能看到设备但扫不到名字，建议直接指定它",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=DEFAULT_SCAN_TIMEOUT,
        help=f"BLE 扫描时长（秒），默认 {DEFAULT_SCAN_TIMEOUT}",
    )
    parser.add_argument(
        "--refresh",
        type=float,
        default=DEFAULT_REFRESH_HZ,
        help=f"终端输出刷新频率（Hz），默认 {DEFAULT_REFRESH_HZ}",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="启用命令输入模式，可向 ESP32 发送控制命令",
    )
    parser.add_argument(
        "--log-dir",
        default=DEFAULT_LOG_DIR,
        help=f"Markdown 日志目录，默认 {DEFAULT_LOG_DIR}",
    )
    return parser.parse_args()


async def scan_and_select(
    timeout: float,
    prefix: str = "",
) -> tuple[str, str] | tuple[None, None]:
    """扫描 BLE 设备并让用户交互选择，返回 (name, address)。"""
    print(f"正在扫描 BLE 设备 ({timeout:.0f}s)...\n")
    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)

    if not devices:
        print("未发现任何 BLE 设备")
        return None, None

    all_entries: list[tuple[int, str, str, int, list[str]]] = []
    for idx, (device, adv) in enumerate(devices.values(), start=1):
        name = device.name or adv.local_name or "<未命名>"
        rssi = getattr(adv, "rssi", 0) or 0
        uuids = [str(u) for u in (getattr(adv, "service_uuids", None) or [])]
        all_entries.append((idx, name, str(device.address), rssi, uuids))

    if prefix:
        lower_prefix = prefix.lower()
        candidates = [e for e in all_entries if e[1].lower().startswith(lower_prefix)]
        if not candidates:
            print(f"未找到匹配前缀 '{prefix}' 的设备，显示全部：\n")
        else:
            all_entries = candidates
            print(f"匹配前缀 '{prefix}' 的设备 ({len(all_entries)} 台)：\n")

    header = f"{'#':>3}  {'设备名称':<30} {'地址':<18} {'RSSI':>5}"
    print(header)
    print("-" * 65)
    for idx, name, addr, rssi, uuids in all_entries:
        uuids_str = ", ".join(uuids) if uuids else "-"
        print(f"{idx:>3}  {name:<30} {addr:<18} {rssi:>4}dBm  [{uuids_str}]")
    print()

    if len(all_entries) == 1:
        _, name, addr, _, _ = all_entries[0]
        return name, addr

    while True:
        try:
            raw = input("选择设备编号 (q 退出): ").strip()
            if raw.lower() == "q":
                return None, None
            num = int(raw)
            if 1 <= num <= len(all_entries):
                _, name, addr, _, _ = all_entries[num - 1]
                return name, addr
            print(f"编号超出范围 1-{len(all_entries)}")
        except ValueError:
            print("请输入有效数字")
        except (EOFError, KeyboardInterrupt):
            return None, None


async def main():
    args = parse_args()

    device_name = args.device_name
    address = args.address

    if args.scan:
        selected_name, selected_addr = await scan_and_select(
            timeout=args.scan_timeout,
            prefix=args.prefix,
        )
        if selected_addr is None:
            print("未选择设备，退出")
            return
        device_name = selected_name
        address = selected_addr
        print(f"\n已选择: {device_name} ({address})\n")

    await run_client(
        device_name=device_name,
        scan_timeout=args.scan_timeout,
        refresh_hz=args.refresh,
        interactive=args.interactive,
        address=address,
        log_dir=Path(args.log_dir),
        pipe_path=args.pipe,
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print()
        print("已退出 BLE 监视器")
