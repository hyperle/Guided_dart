#!/usr/bin/env python3
"""Check ordinary C++ headers/sources and the existing dependency direction.

Behavior and ODR correctness are verified by building and linking real targets.
This check does not require a header/source pair. Each C++ file has one enclosing
named namespace, except for process and C ABI entry points.
"""

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TOKENS = r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|(?<![\w])\'(?:\\.|[^\'\\\n])*\''


def strip_literals(text):
    return re.sub(TOKENS, lambda match: re.sub(r'[^\n]', ' ', match.group()), text)


def includes_in(text):
    uncommented = re.sub(
        TOKENS,
        lambda match: re.sub(r'[^\n]', ' ', match.group())
        if match.group().startswith(('//', '/*')) else match.group(),
        text,
    )
    return re.findall(r'^\s*#\s*include\s*[<"]([^">]+)[">]', uncommented, re.MULTILINE)


def source_errors(relative, text):
    errors = []
    clean = strip_literals(text)
    for include in includes_in(text):
        if include.endswith('.cpp'):
            errors.append(f'{relative}: including .cpp is forbidden: {include}')
        if relative.parts[0] == 'src':
            if (include.startswith(('sys/', 'camera/k230/', 'spi/'))
                    or include in ('unistd.h', 'fcntl.h', 'pthread.h', 'signal.h', 'dirent.h')
                    or re.match(r'(?:mpi_|k_|drv_).*\.h$', include)):
                errors.append(f'{relative}: L4 includes a platform implementation header: {include}')
        if relative.parts[0] == 'include':
            candidates = (ROOT / relative.parent / include, ROOT / include,
                          ROOT / 'include/firmware/src' / include)
            if any(candidate.resolve().is_relative_to(ROOT / 'src') for candidate in candidates):
                errors.append(f'{relative}: lower-level library includes L4: {include}')
    if re.search(r'^\s*(?:export\s+)?(?:module|import)\b', clean, re.MULTILINE):
        errors.append(f'{relative}: use headers and sources instead of named modules')
    if relative.suffix in ('.cpp', '.hpp'):
        namespaces = list(re.finditer(r'\bnamespace\s*(?:(\w+(?:::\w+)*)\s*)?\{', clean))
        entry_point = bool(re.search(r'\bint\s+main\s*\(', clean) or 'extern "C"' in text)
        if len(namespaces) != 1 and not (entry_point and not namespaces):
            errors.append(f'{relative}: use one enclosing named namespace per C++ file')
        elif namespaces and not namespaces[0].group(1):
            errors.append(f'{relative}: use a named namespace for the file')
    if relative.suffix == '.cpp' and (
            re.search(r'^\s*#\s*ifndef\s+(\w+)\s*\n\s*#\s*define\s+\1\b', clean, re.MULTILINE)
            or re.search(r'^\s*#\s*pragma\s+once\b', clean, re.MULTILINE)):
        errors.append(f'{relative}: .cpp must not use header include guards')
    return errors


def check():
    listed = subprocess.check_output(
        ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=ROOT
    ).decode().split('\0')
    paths = sorted({ROOT / name for name in listed if name and (ROOT / name).is_file()})
    errors = []
    sources = headers = 0
    for path in paths:
        relative = path.relative_to(ROOT)
        if path.suffix in ('.c', '.h', '.cpp', '.hpp'):
            errors.extend(source_errors(relative, path.read_text()))
            sources += path.suffix == '.cpp'
            headers += path.suffix == '.hpp'
        elif path.suffix in ('.cmake', '.py') or path.name == 'CMakeLists.txt':
            if re.search(r'#\s*include\s*[^\n]*\.cpp(?:["\'>}]|\\["\'])', path.read_text()):
                errors.append(f'{relative}: generated .cpp inclusion is forbidden')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f'C++ structure passed: {sources} sources, {headers} headers; no named modules or .cpp includes')
    return 0


if __name__ == '__main__':
    sys.exit(check())
