#!/usr/bin/env python3
"""Play an Annex-B H.264 elementary stream on the host with ffplay."""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Play a raw Annex-B H.264 file")
    parser.add_argument("input", type=Path, help="input .h264 file")
    parser.add_argument("--framerate", type=float, default=30.0,
                        help="input frame rate (default: 30)")
    parser.add_argument("--loop", action="store_true", help="loop forever")
    parser.add_argument("--fullscreen", action="store_true",
                        help="start ffplay fullscreen")
    parser.add_argument("--mute", action="store_true", help="mute audio output")
    parser.add_argument("--ffplay", default="ffplay",
                        help="ffplay executable (default: ffplay)")
    args = parser.parse_args()

    input_path = args.input.expanduser()
    if not input_path.is_file():
        print(f"error: input file not found: {input_path}", file=sys.stderr)
        return 2
    if args.framerate <= 0:
        print("error: --framerate must be greater than zero", file=sys.stderr)
        return 2
    ffplay = shutil.which(args.ffplay)
    if ffplay is None:
        print("error: ffplay not found; install ffmpeg or pass --ffplay PATH",
              file=sys.stderr)
        return 127

    command = [ffplay, "-f", "h264", "-framerate", str(args.framerate),
               "-i", str(input_path), "-autoexit"]
    if args.fullscreen:
        command.append("-fs")
    if args.mute:
        command.append("-an")
    try:
        # Raw H.264 has no seekable container index. Restarting ffplay after
        # EOF provides looping without ffplay repeatedly trying to seek.
        while True:
            result = subprocess.run(command, check=False)
            if not args.loop or result.returncode != 0:
                return result.returncode
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
