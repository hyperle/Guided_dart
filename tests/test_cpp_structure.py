#!/usr/bin/env python3
"""Architecture checks must reject regressions independently of a build cache."""
from pathlib import Path
import unittest

from check_cpp_structure import source_errors


class StructureTest(unittest.TestCase):
    def test_cpp_include_is_forbidden_in_c_and_cpp(self):
        for path in ('src/use.cpp', 'include/firmware/src/camera/use.c'):
            with self.subTest(path=path):
                errors = source_errors(Path(path), '#include "implementation.' + 'cpp"\n')
                self.assertTrue(any('including .cpp is forbidden' in error for error in errors))

    def test_module_with_global_fragment_and_import(self):
        errors = source_errors(Path('src/control/example.cpp'), '''module;
#include <array>
export module dart.control.example;
import dart.detection.result;
export namespace dart::control { class Example {}; }
''')
        self.assertEqual(errors, [])

    def test_module_implementation_unit(self):
        errors = source_errors(Path('src/control/implementation.cpp'), """module;
#include <array>
module dart.control.example;
namespace dart::control { int value() { return 42; } }
""")
        self.assertEqual(errors, [])

    def test_reusable_source_needs_module_and_no_guard(self):
        errors = source_errors(Path('src/control/example.cpp'), '''#ifndef DART_EXAMPLE_CPP
#define DART_EXAMPLE_CPP
namespace dart::control { class Example {}; }
#endif
''')
        self.assertTrue(any('header include guards' in error for error in errors))
        self.assertTrue(any('named module' in error for error in errors))

    def test_namespace_alias_and_header_after_module(self):
        errors = source_errors(Path('src/control/example.cpp'), '''module;
export module dart.control.example;
#include <array>
namespace control = dart::control;
''')
        self.assertTrue(any('textual includes must precede' in error for error in errors))
        self.assertTrue(any('aliases' in error for error in errors))


if __name__ == '__main__':
    unittest.main()
