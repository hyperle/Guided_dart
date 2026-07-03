#!/usr/bin/env python3
import asyncio
import os
import queue
import select
import struct
import sys
import termios
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
PROTOCOL_DIR = os.path.join(REPO_ROOT, "protocol")

if PROTOCOL_DIR not in sys.path:
    sys.path.insert(0, PROTOCOL_DIR)

from guidance_protocol import GuidanceProtocolLoader, YamlLiteParser


class GuidanceHostPaths:
    @classmethod
    def protocol_schema_path(cls):
        return os.path.join(REPO_ROOT, "protocol", "guidance_protocol.yaml")

    @classmethod
    def default_config_path(cls):
        return os.path.join(REPO_ROOT, "Host_tools", "guidance", "guidance_params.yaml")


class GuidanceParamRegistry:
    def __init__(self, protocol):
        self._protocol = protocol
        self._status_text = {}

        for status in protocol.iter_param_status():
            self._status_text[status.value] = status.schema_name.upper()

    def flatten_params(self, config):
        flattened = {}
        self._flatten_node("", config, flattened)
        return flattened

    def build_param_items(self, config):
        flattened = self.flatten_params(config)
        items = []

        for definition in self._protocol.iter_param_definitions():
            if definition.dotted_name not in flattened:
                continue
            items.append(
                {
                    "name": definition.dotted_name,
                    "type": definition.value_type,
                    "key": definition.key,
                    "raw": flattened[definition.dotted_name],
                    "value": self._encode_value(definition.value_type, flattened[definition.dotted_name]),
                }
            )

        return items

    def format_ack_value(self, value_type, value_u32):
        if value_type == "f32":
            return struct.unpack("<f", struct.pack("<I", value_u32))[0]
        if value_type == "bool":
            return value_u32 != 0
        return value_u32

    def status_text(self, status):
        return self._status_text.get(status, f"UNKNOWN_STATUS_{status}")

    def applied_status_code(self):
        return self._protocol.param_status_value("applied")

    def _flatten_node(self, prefix, value, output):
        if isinstance(value, dict):
            for child_key, child_value in value.items():
                child_prefix = child_key if not prefix else f"{prefix}.{child_key}"
                self._flatten_node(child_prefix, child_value, output)
            return

        output[prefix] = value

    def _encode_value(self, value_type, value):
        if value_type == "f32":
            return struct.unpack("<I", struct.pack("<f", float(value)))[0]
        if value_type == "bool":
            return 1 if bool(value) else 0
        return int(value) & 0xFFFFFFFF


class FrameCodec:
    def __init__(self, protocol):
        self._frame_header_0 = protocol.frame_header_0
        self._frame_header_1 = protocol.frame_header_1
        self._message_type_param_command = protocol.message_type_value("param_command")
        self._message_type_param_ack = protocol.message_type_value("param_ack")
        self._sequence = 0
        self._reset_parser()

    def build_param_frame(self, key, value_u32):
        sequence = self._sequence
        payload = struct.pack("<HBI", sequence, key, value_u32)
        self._sequence = (self._sequence + 1) & 0xFFFF

        frame = bytearray()
        frame.append(self._frame_header_0)
        frame.append(self._frame_header_1)
        frame.append(self._message_type_param_command)
        frame.append(len(payload))
        frame.extend(payload)
        frame.append(sum(frame) & 0xFF)
        return bytes(frame), sequence

    def decode_param_ack(self, frame, expected_key=None, expected_sequence=None):
        if len(frame) != 13:
            raise ValueError(f"unexpected ack frame size: {len(frame)}")
        if frame[0] != self._frame_header_0 or frame[1] != self._frame_header_1:
            raise ValueError("bad frame header")
        if frame[2] != self._message_type_param_ack:
            raise ValueError(f"unexpected frame type: 0x{frame[2]:02X}")
        if frame[3] != 8:
            raise ValueError(f"unexpected ack payload length: {frame[3]}")
        if (sum(frame[:-1]) & 0xFF) != frame[-1]:
            raise ValueError("bad frame checksum")

        sequence, key, status, value_u32 = struct.unpack("<HBBI", frame[4:12])
        if (expected_key is not None) and (key != expected_key):
            raise ValueError(f"unexpected ack key: 0x{key:02X}, expected 0x{expected_key:02X}")
        if (expected_sequence is not None) and (sequence != expected_sequence):
            raise ValueError(
                f"unexpected ack sequence: {sequence}, expected {expected_sequence}"
            )

        return {"sequence": sequence, "key": key, "status": status, "value": value_u32}

    def is_param_ack_frame(self, frame):
        return (
            len(frame) >= 5 and
            frame[0] == self._frame_header_0 and
            frame[1] == self._frame_header_1 and
            frame[2] == self._message_type_param_ack
        )

    def consume_byte(self, byte_value):
        if self._state == 0:
            if byte_value == self._frame_header_0:
                self._buffer = bytearray([byte_value])
                self._state = 1
            return None

        if self._state == 1:
            if byte_value == self._frame_header_1:
                self._buffer.append(byte_value)
                self._state = 2
            elif byte_value == self._frame_header_0:
                self._buffer = bytearray([byte_value])
            else:
                self._reset_parser()
            return None

        if self._state == 2:
            self._buffer.append(byte_value)
            self._state = 3
            return None

        if self._state == 3:
            self._buffer.append(byte_value)
            self._expected_frame_size = byte_value + 5
            if self._expected_frame_size < 5 or self._expected_frame_size > 64:
                self._reset_parser()
                return None
            self._state = 4
            return None

        self._buffer.append(byte_value)
        if len(self._buffer) >= self._expected_frame_size:
            frame = bytes(self._buffer)
            self._reset_parser()
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                return None
            return frame
        return None

    def _reset_parser(self):
        self._state = 0
        self._expected_frame_size = 0
        self._buffer = bytearray()


class PosixSerialPort:
    BAUD_MAP = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
        921600: termios.B921600,
    }

    def __init__(self, port, baudrate):
        self._port = port
        self._baudrate = baudrate
        self._fd = -1

    def open(self):
        if self._baudrate not in self.BAUD_MAP:
            raise ValueError(f"unsupported baudrate: {self._baudrate}")

        self._fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self._fd)

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = self.BAUD_MAP[self._baudrate]
        attrs[5] = self.BAUD_MAP[self._baudrate]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        termios.tcflush(self._fd, termios.TCIOFLUSH)
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)

    def close(self):
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def write_all(self, payload):
        offset = 0
        while offset < len(payload):
            written = os.write(self._fd, payload[offset:])
            if written <= 0:
                raise OSError("serial write failed")
            offset += written

    def read_byte(self, timeout_s):
        readable, _, _ = select.select([self._fd], [], [], timeout_s)
        if not readable:
            return None

        data = os.read(self._fd, 1)
        if not data:
            return None

        return data[0]


class SerialTransport:
    def __init__(self, port, baudrate, protocol):
        self._port = PosixSerialPort(port, baudrate)
        self._codec = FrameCodec(protocol)

    def open(self):
        self._port.open()

    def close(self):
        self._port.close()

    def send_frame(self, frame):
        self._port.write_all(frame)

    def recv_frame(self, timeout_ms):
        deadline = time.monotonic() + (timeout_ms / 1000.0)

        while time.monotonic() < deadline:
            byte_value = self._port.read_byte(max(deadline - time.monotonic(), 0.0))
            if byte_value is None:
                continue

            frame = self._codec.consume_byte(byte_value)
            if frame is not None:
                return frame

        raise TimeoutError("serial receive timeout")

    def recv_param_ack(self, timeout_ms, expected_key=None, expected_sequence=None):
        deadline = time.monotonic() + (timeout_ms / 1000.0)

        while time.monotonic() < deadline:
            frame = self.recv_frame(max(int((deadline - time.monotonic()) * 1000.0), 1))
            if not self._codec.is_param_ack_frame(frame):
                continue

            ack = self._codec.decode_param_ack(frame, expected_key, expected_sequence)
            return frame, ack

        raise TimeoutError("serial ack timeout")


class BleTransport:
    def __init__(self,
                 protocol,
                 address,
                 device_name,
                 connect_timeout_ms,
                 service_uuid,
                 downlink_char_uuid,
                 uplink_char_uuid,
                 ack_char_uuid=""):
        self._codec = FrameCodec(protocol)
        self._address = address
        self._device_name = device_name
        self._connect_timeout_ms = connect_timeout_ms
        self._service_uuid = service_uuid.lower()
        self._downlink_char_uuid = downlink_char_uuid.lower()
        self._uplink_char_uuid = uplink_char_uuid.lower()
        self._ack_char_uuid = ack_char_uuid.lower() if ack_char_uuid else ""
        self._client = None
        self._notify_queue = queue.Queue()
        self._ack_queue = queue.Queue()
        self._bleak_module = None

    def open(self):
        self._load_bleak()
        asyncio.run(self._async_open())

    def close(self):
        if self._client is not None:
            asyncio.run(self._async_close())

    def send_frame(self, frame):
        asyncio.run(self._async_send_frame(frame))

    def recv_frame(self, timeout_ms):
        try:
            return self._notify_queue.get(timeout=timeout_ms / 1000.0)
        except queue.Empty as exc:
            raise TimeoutError("ble receive timeout") from exc

    def recv_param_ack(self, timeout_ms, expected_key=None, expected_sequence=None):
        queue_ref = self._ack_queue if self._ack_char_uuid else self._notify_queue
        deadline = time.monotonic() + (timeout_ms / 1000.0)

        while time.monotonic() < deadline:
            try:
                frame = queue_ref.get(timeout=max(deadline - time.monotonic(), 0.001))
            except queue.Empty as exc:
                raise TimeoutError("ble ack timeout") from exc

            try:
                ack = self._codec.decode_param_ack(frame, expected_key, expected_sequence)
            except ValueError:
                continue

            return frame, ack

        raise TimeoutError("ble ack timeout")

    def _load_bleak(self):
        if self._bleak_module is not None:
            return

        try:
            import bleak
        except ImportError as exc:
            raise RuntimeError("BLE mode requires 'bleak'. Install it with: python3 -m pip install bleak") from exc

        self._bleak_module = bleak

    async def _async_open(self):
        BleakClient = self._bleak_module.BleakClient
        BleakScanner = self._bleak_module.BleakScanner

        timeout_s = self._connect_timeout_ms / 1000.0
        device = None

        if self._address:
            device = await self._scan_for_device(
                BleakScanner, timeout_s,
                address=self._address
            )

        if device is None:
            device = await self._scan_for_device(
                BleakScanner, timeout_s,
                device_name=self._device_name
            )

        if device is None:
            raise RuntimeError(
                f"BLE device not found: address={self._address or 'auto'}, "
                f"name={self._device_name}"
            )

        self._client = BleakClient(device, timeout=timeout_s)
        await self._client.connect()
        await self._client.start_notify(self._uplink_char_uuid, self._handle_notify)
        if self._ack_char_uuid:
            await self._client.start_notify(self._ack_char_uuid, self._handle_ack_notify)

    async def _scan_for_device(self, BleakScanner, timeout, address=None, device_name=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                devices = await BleakScanner.discover(
                    timeout=min(2.0, max(0.5, deadline - time.monotonic())),
                    return_adv=True,
                )
            except Exception:
                await asyncio.sleep(0.5)
                continue

            for dev, adv in devices.values():
                if address and dev.address.lower() == address.lower():
                    return dev
                if device_name:
                    name = dev.name or getattr(adv, "local_name", None) or ""
                    if device_name.lower() == name.lower():
                        return dev

            await asyncio.sleep(0.2)

        return None

    async def _async_close(self):
        try:
            await self._client.stop_notify(self._uplink_char_uuid)
        except Exception:
            pass
        if self._ack_char_uuid:
            try:
                await self._client.stop_notify(self._ack_char_uuid)
            except Exception:
                pass
        try:
            await self._client.disconnect()
        finally:
            self._client = None

    async def _async_send_frame(self, frame):
        if self._client is None:
            raise RuntimeError("BLE transport not connected")
        await self._client.write_gatt_char(self._downlink_char_uuid, frame, response=True)

    def _handle_notify(self, characteristic, data):
        del characteristic
        self._notify_queue.put(bytes(data))

    def _handle_ack_notify(self, characteristic, data):
        del characteristic
        self._ack_queue.put(bytes(data))


class PipeTransport:
    def __init__(self, protocol, pipe_path):
        self._codec = FrameCodec(protocol)
        self._pipe_path = pipe_path
        self._fd = None

    def open(self):
        try:
            self._fd = os.open(self._pipe_path, os.O_RDONLY | os.O_NONBLOCK)
        except FileNotFoundError:
            raise RuntimeError(f"Pipe not found: {self._pipe_path} (start ble-connect --pipe first)")
        except PermissionError:
            raise RuntimeError(f"Permission denied: {self._pipe_path}")

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def recv_frame(self, timeout_ms):
        if self._fd is None:
            raise RuntimeError("PipeTransport not open")

        deadline = time.monotonic() + (timeout_ms / 1000.0)
        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            ready, _, _ = select.select([self._fd], [], [], min(remaining, 0.1))
            if not ready:
                continue
            try:
                data = os.read(self._fd, 1024)
            except BlockingIOError:
                continue
            if not data:
                raise TimeoutError("pipe closed (ble-connect disconnected)")
            for byte_val in data:
                frame = self._codec.consume_byte(byte_val)
                if frame is not None:
                    return frame
        raise TimeoutError("pipe receive timeout")


class GuidanceTransportFactory:
    def __init__(self, protocol):
        self._protocol = protocol

    def create(self, args, transport_mode, serial_cfg, ble_cfg):
        pipe_path = getattr(args, "pipe", "")
        if pipe_path:
            return PipeTransport(self._protocol, pipe_path)

        if transport_mode == "ble":
            return BleTransport(
                self._protocol,
                address=args.ble_address or str(ble_cfg.get("address", "")),
                device_name=args.ble_device_name or str(ble_cfg.get("device_name", "Dart_1")),
                connect_timeout_ms=int(args.ble_connect_timeout_ms or ble_cfg.get("connect_timeout_ms", 8000)),
                service_uuid=args.ble_service_uuid or str(ble_cfg.get("service_uuid", "4fafc201-1fb5-459e-8fcc-c5c9c331914b")),
                downlink_char_uuid=args.ble_downlink_char_uuid or str(ble_cfg.get("downlink_char_uuid", "beb5483e-36e1-4688-b7f5-ea07361b26a8")),
                uplink_char_uuid=args.ble_uplink_char_uuid or str(ble_cfg.get("uplink_char_uuid", "9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21")),
                ack_char_uuid=getattr(args, "ble_ack_char_uuid", "") or str(ble_cfg.get("ack_char_uuid", "de24d570-5f81-4d6b-8e4d-2f1309367d91")),
            )

        port = args.port or serial_cfg.get("port")
        baudrate = int(args.baudrate or serial_cfg.get("baudrate", 115200))
        if not port:
            raise ValueError("serial.port is required for serial transport")

        return SerialTransport(port, baudrate, self._protocol)


class GuidanceProtocolContext:
    def __init__(self, schema_path=""):
        loader = GuidanceProtocolLoader()
        self.protocol = loader.load(schema_path or GuidanceHostPaths.protocol_schema_path())
        self.param_registry = GuidanceParamRegistry(self.protocol)
        self.frame_codec = FrameCodec(self.protocol)
        self.transport_factory = GuidanceTransportFactory(self.protocol)
