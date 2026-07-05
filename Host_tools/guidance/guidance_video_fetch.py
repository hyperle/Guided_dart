#!/usr/bin/env python3
import argparse
import binascii
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from guidance_host_common import GuidanceHostPaths, GuidanceProtocolContext, PosixSerialPort, YamlLiteParser

OPENMV_PASSTHROUGH_PARAM = 'system.openmv_passthrough'
OPENMV_PASSTHROUGH_ENABLE = 1
RAW_REPL_EXIT = b'\x02'
RAW_REPL_ENTER = b'\x01'
CTRL_C = b'\x03'
CTRL_D = b'\x04'
PASSTHROUGH_ESCAPE = b'\x1d\x1d\x1d'
DEFAULT_OUTPUT_DIR = os.path.join(REPO_ROOT, 'record')


def ensure_directory(path):
    os.makedirs(path, exist_ok=True)


def load_config(config_path):
    parser = YamlLiteParser()
    return parser.parse_file(config_path)


def find_param_key(protocol):
    for definition in protocol.iter_param_definitions():
        if definition.dotted_name == OPENMV_PASSTHROUGH_PARAM:
            return definition.key
    raise RuntimeError(f'missing protocol param: {OPENMV_PASSTHROUGH_PARAM}')


def send_passthrough_enable(config, args, protocol_ctx):
    serial_cfg = config.get('serial', {})
    transport = protocol_ctx.transport_factory.create(args, 'serial', serial_cfg, {})
    key = find_param_key(protocol_ctx.protocol)
    frame, sequence = protocol_ctx.frame_codec.build_param_frame(key, OPENMV_PASSTHROUGH_ENABLE)
    transport.open()
    try:
        transport.send_frame(frame)
        ack_frame, ack = transport.recv_param_ack(1000, key, sequence)
        if ack is None:
            ack = protocol_ctx.frame_codec.decode_param_ack(ack_frame, key, sequence)
        if ack['status'] != protocol_ctx.param_registry.applied_status_code():
            raise RuntimeError(f'passthrough rejected with status={ack["status"]}')
    finally:
        transport.close()


class RawReplSession:
    def __init__(self, port, baudrate):
        self._port = PosixSerialPort(port, baudrate)

    def __enter__(self):
        self._port.open()
        self._enter_passthrough_raw_repl()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self._soft_reset_and_exit()
        finally:
            self._leave_passthrough()
            self._port.close()

    def _write_all(self, payload):
        self._port.write_all(payload)

    def _read_until(self, marker, timeout_s):
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while time.monotonic() < deadline:
            byte_value = self._port.read_byte(max(deadline - time.monotonic(), 0.0))
            if byte_value is None:
                continue
            data.append(byte_value)
            if data.endswith(marker):
                return bytes(data)
        raise TimeoutError(f'timeout waiting for {marker!r}, got {bytes(data)!r}')

    def _drain(self, timeout_s=0.2):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._port.read_byte(0.02)

    def _enter_passthrough_raw_repl(self):
        self._drain()
        self._write_all(PASSTHROUGH_ESCAPE)
        time.sleep(0.1)
        self._write_all(CTRL_C)
        time.sleep(0.1)
        self._write_all(CTRL_C)
        time.sleep(0.1)
        self._write_all(RAW_REPL_EXIT)
        time.sleep(0.05)
        self._drain()
        self._write_all(RAW_REPL_ENTER)
        self._read_until(b'raw REPL; CTRL-B to exit\r\n>', 2.0)
        self._write_all(CTRL_D)
        self._read_until(b'soft reboot\r\n', 2.0)
        self._read_until(b'raw REPL; CTRL-B to exit\r\n>', 2.0)

    def _soft_reset_and_exit(self):
        try:
            self._write_all(CTRL_D)
            time.sleep(0.2)
        except Exception:
            pass
        try:
            self._write_all(RAW_REPL_EXIT)
            time.sleep(0.1)
        except Exception:
            pass

    def _leave_passthrough(self):
        try:
            self._write_all(PASSTHROUGH_ESCAPE)
            time.sleep(0.1)
        except Exception:
            pass

    def exec_raw(self, command, timeout_s=5.0):
        payload = command.encode('utf-8')
        self._write_all(payload)
        self._write_all(CTRL_D)
        self._read_until(b'OK', 2.0)
        data = self._read_until(b'\x04', timeout_s)
        error = self._read_until(b'\x04', timeout_s)
        stdout_bytes = data[:-1]
        stderr_bytes = error[:-1]
        if stderr_bytes:
            raise RuntimeError(stderr_bytes.decode('utf-8', errors='replace'))
        return stdout_bytes.decode('utf-8', errors='replace')


def build_listing_command():
    return """
import os
roots = ['/sdcard/recordings', '/sd/recordings']
for root in roots:
    try:
        entries = os.listdir(root)
        for name in sorted(entries):
            path = root + '/' + name
            size = os.stat(path)[6]
            print(root + '|' + name + '|' + str(size))
    except OSError:
        pass
"""


def build_read_command(remote_path, chunk_size, offset):
    return f"""
import ubinascii
f = open({remote_path!r}, 'rb')
f.seek({offset})
data = f.read({chunk_size})
f.close()
print(ubinascii.hexlify(data).decode())
"""


def parse_listing(text):
    items = []
    for line in text.splitlines():
        line = line.strip()
        if not line or '|' not in line:
            continue
        root, name, size_text = line.split('|', 2)
        items.append({'root': root, 'name': name, 'size': int(size_text)})
    return items


def list_recordings(session):
    return parse_listing(session.exec_raw(build_listing_command()))


def select_recording(items, target_name):
    if not items:
        raise RuntimeError('no recordings found on OpenMV storage')
    if target_name:
        for item in items:
            if item['name'] == target_name:
                return item
        raise RuntimeError(f'recording not found: {target_name}')
    return items[-1]


def download_recording(session, item, output_dir, chunk_size):
    ensure_directory(output_dir)
    remote_path = item['root'] + '/' + item['name']
    local_path = os.path.join(output_dir, item['name'])
    remaining = item['size']
    offset = 0
    with open(local_path, 'wb') as handle:
        while remaining > 0:
            read_size = min(chunk_size, remaining)
            output = session.exec_raw(build_read_command(remote_path, read_size, offset), timeout_s=10.0).strip()
            chunk = binascii.unhexlify(output.encode('ascii')) if output else b''
            if len(chunk) != read_size:
                raise RuntimeError(f'short read at offset {offset}: expected {read_size}, got {len(chunk)}')
            handle.write(chunk)
            offset += len(chunk)
            remaining -= len(chunk)
            print(f'pulled {offset}/{item["size"]} bytes', file=sys.stderr)
    return local_path


def build_arg_parser():
    parser = argparse.ArgumentParser(description='Fetch OpenMV recordings through the control-board serial link')
    parser.add_argument('action', choices=['list', 'pull'], help='list recordings or pull one recording')
    parser.add_argument('--config', default=GuidanceHostPaths.default_config_path(), help='path to guidance config')
    parser.add_argument('--port', default='', help='override serial.port from config')
    parser.add_argument('--baudrate', type=int, default=0, help='override serial.baudrate from config')
    parser.add_argument('--name', default='', help='recording filename to pull; default is latest')
    parser.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR, help='directory for downloaded recordings')
    parser.add_argument('--chunk-size', type=int, default=256, help='bytes per OpenMV REPL read')
    return parser


def main(argv):
    args = build_arg_parser().parse_args(argv)
    config = load_config(args.config)
    protocol_ctx = GuidanceProtocolContext()

    serial_cfg = config.get('serial', {})
    port = args.port or serial_cfg.get('port')
    baudrate = int(args.baudrate or serial_cfg.get('baudrate', 115200))
    if not port:
        raise ValueError('serial.port is required for video fetch')

    send_passthrough_enable(config, args, protocol_ctx)

    with RawReplSession(port, baudrate) as session:
        items = list_recordings(session)
        if args.action == 'list':
            for item in items:
                print(f"{item['name']}\t{item['size']}\t{item['root']}")
            return 0

        item = select_recording(items, args.name)
        local_path = download_recording(session, item, args.output_dir, max(16, args.chunk_size))
        print(local_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
