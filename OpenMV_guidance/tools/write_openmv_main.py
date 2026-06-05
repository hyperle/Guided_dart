#!/usr/bin/env python3
import argparse
import sys
import time


def write_script(port, script_path):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required: python3 -m pip install pyserial") from exc

    with open(script_path, "r", encoding="utf-8") as handle:
        content = handle.read()

    with serial.Serial(port, 115200, timeout=1) as ser:
        ser.write(b"\x03")
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        command = (
            "f = open('main.py', 'w')\n"
            "f.write(%r)\n"
            "f.close()\n" % content
        )
        ser.write(command.encode("utf-8"))
        ser.write(b"\x04")


def main(argv):
    parser = argparse.ArgumentParser(description="Write OpenMV main.py over the REPL UART")
    parser.add_argument("--port", required=True, help="serial port")
    parser.add_argument("--script", required=True, help="path to main.py")
    args = parser.parse_args(argv)

    write_script(args.port, args.script)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
