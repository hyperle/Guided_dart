#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
import time


def collect_scripts(entry_script_path):
    script_path = Path(entry_script_path).resolve()
    source_dir = script_path.parent
    scripts = sorted(
        source_dir.glob("*.py"),
        key=lambda path: (path.name == script_path.name, path.name),
    )
    if script_path not in scripts:
        raise RuntimeError("OpenMV entry script not found in source dir: %s" % script_path)
    return scripts


def build_write_commands(script_paths):
    commands = []
    for script_path in script_paths:
        with open(script_path, "r", encoding="utf-8") as handle:
            content = handle.read()
        commands.extend(
            [
                "f = open(%r, 'w')" % script_path.name,
                "f.write(%r)" % content,
                "f.close()",
            ]
        )
    return "\n".join(commands) + "\n"


def write_scripts(port, script_path):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required: python3 -m pip install pyserial") from exc

    command = build_write_commands(collect_scripts(script_path))

    with serial.Serial(port, 115200, timeout=1) as ser:
        ser.write(b"\x03")
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(command.encode("utf-8"))
        ser.write(b"\x04")


def main(argv):
    parser = argparse.ArgumentParser(
        description="Write OpenMV Python sources from the entry script directory over the REPL UART"
    )
    parser.add_argument("--port", required=True, help="serial port")
    parser.add_argument("--script", required=True, help="path to the OpenMV entry script")
    args = parser.parse_args(argv)

    write_scripts(args.port, args.script)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
