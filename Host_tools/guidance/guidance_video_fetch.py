#!/usr/bin/env python3
import sys

from guidance_host_common import YamlLiteParser


def load_config(config_path):
    return YamlLiteParser().parse_file(config_path)


def main(argv):
    del argv
    raise RuntimeError("OpenMV recording fetch through STM32 passthrough has been removed with the hot parameter path; use OpenMV USB tools instead")


if __name__ == '__main__':
    try:
        raise SystemExit(main(sys.argv[1:]))
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
