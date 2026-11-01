#!/usr/bin/env python3
"""Regress stale SDK headers silently removing the CanMV IDE service."""

import importlib.util
from pathlib import Path
import shutil
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("canmv_build", ROOT / "tools/canmv_tuning/build.py")
BUILD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILD)


class CanmvBuildConfigTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("cc"), "host C preprocessor required")
    def test_canmv_options_override_stale_header_and_preserve_kernel(self):
        header = BUILD.config_header(
            'CONFIG_CANMV_IDE_SUPPORT=y\n'
            'CONFIG_CANMV_REPL_UART_ID=-1\n'
            '# CONFIG_CANMV_OBSOLETE is not set\n'
            'CONFIG_BOARD_NAME="CanMV"\n',
            '#define CONFIG_CANMV_IDE_SUPPORT 0\n'
            '#define CONFIG_CANMV_REPL_UART_ID 0\n'
            '#define CONFIG_CANMV_OBSOLETE 1\n'
            '#define CONFIG_BOARD_NAME "kernel"\n'
            '#define CONFIG_MEM_MMZ_SIZE 0x4600000\n'
            '#define CONFIG_ENABLE_NETWORK_RT_WLAN 1\n',
        )
        # Exercise what the compiler sees, including duplicate definitions.
        checks = '''
#if CONFIG_CANMV_IDE_SUPPORT != 1 || CONFIG_CANMV_REPL_UART_ID != -1
#error IDE or USB REPL configuration lost
#endif
#ifdef CONFIG_CANMV_OBSOLETE
#error Disabled CanMV option retained from stale SDK header
#endif
#if CONFIG_MEM_MMZ_SIZE != 0x4600000 || CONFIG_ENABLE_NETWORK_RT_WLAN != 1
#error Kernel configuration changed
#endif
_Static_assert(sizeof(CONFIG_BOARD_NAME) == sizeof("kernel"), "kernel board name changed");
'''
        result = subprocess.run(
            ["cc", "-std=c11", "-Werror", "-fsyntax-only", "-x", "c", "-"],
            input=header + checks, text=True, capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_disabled_ide_is_rejected_even_if_old_header_enables_it(self):
        for config in ('# CONFIG_CANMV_IDE_SUPPORT is not set\n',
                       'CONFIG_CANMV_IDE_SUPPORT=n\n', ''):
            with self.subTest(config=config):
                with self.assertRaisesRegex(SystemExit, "CONFIG_CANMV_IDE_SUPPORT=y"):
                    BUILD.config_header(config, '#define CONFIG_CANMV_IDE_SUPPORT 1\n')


if __name__ == "__main__":
    unittest.main()
