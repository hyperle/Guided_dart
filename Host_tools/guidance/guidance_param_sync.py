#!/usr/bin/env python3
import sys


def main(argv):
    del argv
    raise RuntimeError("guidance hot parameter sync has been removed; use telemetry viewers and rebuild firmware/OpenMV assets for configuration changes")


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
