#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from guidance_host_common import GuidanceHostPaths
from guidance_tuner_app import launch_tuner


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tune OpenMV green-light detection on recorded video.")
    parser.add_argument("--video", default="", help="path to the MJPEG/video file; if omitted, choose one in the UI")
    parser.add_argument("--config", default=GuidanceHostPaths.default_config_path(), help="path to the guidance parameter file for detector defaults")
    parser.add_argument("--port", default="", help="override serial.port from config")
    parser.add_argument("--baudrate", type=int, default=0, help="override serial.baudrate from config")
    return parser


def main(argv: list[str]) -> int:
    args = build_arg_parser().parse_args(argv)
    return launch_tuner(args.video, args.config, port=args.port, baudrate=args.baudrate)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
