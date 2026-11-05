#!/usr/bin/env python3
"""Protect source organization without imposing header/source pairing."""
from pathlib import Path
import unittest

from check_cpp_structure import source_errors


class StructureTest(unittest.TestCase):
    def test_cpp_include_is_forbidden_in_c_cpp_and_headers(self):
        for path in ('src/use.cpp', 'src/use.hpp', 'include/firmware/src/camera/use.c'):
            with self.subTest(path=path):
                errors = source_errors(Path(path), '#include "implementation.' + 'cpp"\n')
                self.assertTrue(any('including .cpp is forbidden' in error for error in errors))

    def test_normal_header_with_guard_and_inline_template(self):
        self.assertEqual(source_errors(Path('src/control/example.hpp'), '''#pragma once
#include <array>
namespace dart::control {
template<class T> T identity(T value) { return value; }
class Example { public: void run(); };
}
'''), [])

    def test_ordinary_source_uses_one_enclosing_namespace(self):
        self.assertEqual(source_errors(Path('src/control/example.cpp'), '''#include "example.hpp"
namespace dart::control {
static int helper() { return 42; }
void Example::run() { (void)helper(); }
}
'''), [])

    def test_multiple_or_anonymous_namespaces_are_rejected(self):
        for text in ('namespace a {} namespace b {}', 'namespace {}'):
            self.assertTrue(source_errors(Path('src/example.hpp'), text))

    def test_source_without_matching_header(self):
        self.assertEqual(source_errors(Path('src/entry.cpp'), 'int main() { return 0; }'), [])

    def test_named_modules_are_rejected(self):
        for statement in ('module;', 'export module dart.control.example;',
                          'import dart.control.example;', 'export import dart.control.example;'):
            with self.subTest(statement=statement):
                errors = source_errors(Path('src/control/example.cpp'), statement)
                self.assertTrue(any('instead of named modules' in error for error in errors))

    def test_source_guard_is_rejected_but_header_guard_is_allowed(self):
        text = '#ifndef DART_EXAMPLE_HPP\n#define DART_EXAMPLE_HPP\nnamespace dart { class Example {}; }\n#endif\n'
        self.assertEqual(source_errors(Path('src/control/example.hpp'), text), [])
        self.assertTrue(source_errors(Path('src/control/example.cpp'), text))

    def test_layer_dependency_is_checked_in_headers_too(self):
        errors = source_errors(Path('include/firmware/src/camera/use.hpp'),
                               '#include "src/detector/result.hpp"\n')
        self.assertTrue(any('lower-level library includes L4' in error for error in errors))
        errors = source_errors(Path('src/vision/use.hpp'), '#include <pthread.h>\n')
        self.assertTrue(any('platform implementation header' in error for error in errors))

    def test_commented_rules_and_strings_are_not_code(self):
        text = '// import example;\n/* #include "ignored.' + 'cpp" */\n'
        text += 'namespace dart { const char* message = "import example;"; }\n'
        self.assertEqual(source_errors(Path('src/use.cpp'), text), [])


if __name__ == '__main__':
    unittest.main()
