#!/usr/bin/env python3
import base64
import time
from pathlib import Path
import serial

SRC_DIR = Path(__file__).resolve().parents[2] / "OpenMV_guidance" / "src"
PORT = "/dev/ttyACM0"
BAUD = 115200


def main():
    files = sorted(SRC_DIR.glob("*.py"))
    manifest_content = "\n".join(f.name for f in files) + "\n"

    ser = serial.Serial(PORT, BAUD, timeout=0.5)

    # Drain any pending data
    ser.reset_input_buffer()
    ser.write(b"\x02")
    time.sleep(0.2)
    ser.write(b"\x03\x03")
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.read(4096)

    # Enter raw REPL
    ser.write(b"\x01")
    data = b""
    dl = time.monotonic() + 5.0
    while time.monotonic() < dl:
        b = ser.read(1)
        if b:
            data += b
            if data.endswith(b"raw REPL; CTRL-B to exit\r\n>"):
                break
    else:
        raise RuntimeError(f"raw REPL entry failed: {data!r}")

    ser.write(b"\x04")
    time.sleep(0.5)
    ser.read(8192)  # drain soft reset output

    # Step 1: switch to SD card and delete old files
    ser.write(b"import os\nos.chdir('/sdcard')\nfiles=os.listdir()\nto_del=[f for f in files if f.endswith('.py') or f.endswith('.mpy') or f=='_guided_dart_manifest.txt']\nfor f in to_del:os.remove(f)\nprint('del%d'%len(to_del))\n\x04")
    data = bytearray()
    dl = time.monotonic() + 5.0
    ok = False
    while time.monotonic() < dl:
        b = ser.read(1)
        if b:
            data.extend(b)
            if not ok and data.endswith(b"OK"):
                ok = True
            elif ok and data.count(b"\x04") >= 2:
                break
    parts = bytes(data).split(b"\x04")
    print(f"deleted: {parts[-3]!r}")
    time.sleep(0.05)
    ser.read(4096)

    # Step 2: upload each file
    for py_file in files:
        b64 = base64.b64encode(py_file.read_bytes()).decode("ascii")
        name = py_file.name

        lines = ["import ubinascii", f"f=open('/sdcard/{name}','wb')"]
        while b64:
            lines.append(f"f.write(ubinascii.a2b_base64({b64[:1000]!r}))")
            b64 = b64[1000:]
        lines.append("f.close()")
        lines.append("print('ok')")
        cmd = "\n".join(lines).encode()

        ser.write(cmd + b"\x04")
        data = bytearray()
        dl = time.monotonic() + 20.0
        ok = False
        while time.monotonic() < dl:
            b = ser.read(1)
            if b:
                data.extend(b)
                if not ok and data.endswith(b"OK"):
                    ok = True
                elif ok and data.count(b"\x04") >= 2:
                    break
        parts = bytes(data).split(b"\x04")
        stdout = parts[-3]
        stderr = parts[-2]
        if stderr:
            raise RuntimeError(f"upload {name} stderr: {stderr!r}")
        if b"ok" not in stdout:
            raise RuntimeError(f"upload {name} failed: {stdout!r}")
        print(f"  OK: {name} ({py_file.stat().st_size} bytes)")
        time.sleep(0.05)
        ser.read(4096)

    # Step 3: upload manifest
    b64 = base64.b64encode(manifest_content.encode("utf-8")).decode("ascii")
    lines = ["import ubinascii", "f=open('/sdcard/_guided_dart_manifest.txt','wb')"]
    while b64:
        lines.append(f"f.write(ubinascii.a2b_base64({b64[:1000]!r}))")
        b64 = b64[1000:]
    lines.append("f.close()")
    lines.append("print('ok')")
    cmd = "\n".join(lines).encode()
    ser.write(cmd + b"\x04")
    data = bytearray()
    dl = time.monotonic() + 10.0
    ok = False
    while time.monotonic() < dl:
        b = ser.read(1)
        if b:
            data.extend(b)
            if not ok and data.endswith(b"OK"):
                ok = True
            elif ok and data.count(b"\x04") >= 2:
                break
    parts = bytes(data).split(b"\x04")
    if parts[-2]:
        raise RuntimeError(f"manifest stderr: {parts[-2]!r}")
    print(f"  OK: manifest {len(manifest_content)} bytes")

    # Step 4: exit raw REPL and reset
    ser.write(b"\x02")
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(b"import machine\r\nmachine.reset()\r\n")
    ser.flush()
    time.sleep(0.5)
    ser.close()

    print("Hard reset triggered.")
    for _ in range(30):
        time.sleep(0.5)
        for p in ["/dev/ttyACM0", "/dev/ttyACM1"]:
            if Path(p).exists():
                time.sleep(3)
                print(f"Board rebooted on {p}.")
                return 0
    print("Port did not reappear.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
