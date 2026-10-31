#!/usr/bin/env python3
"""Check independently compiled C++ named modules and their dependency direction.

Compilation, module scanning and final links remain the behavioral acceptance;
this check rejects textual .cpp inclusion even outside C++ source files.
"""

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def strip_literals(text):
    return re.sub(
        r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        lambda match: re.sub(r'[^\n]', ' ', match.group()),
        text,
    )


def source_errors(relative, text):
    errors = []
    clean = strip_literals(text)
    includes = re.findall(r'^\s*#\s*include\s*[<"]([^">]+)[">]', text, re.MULTILINE)
    for include in includes:
        if include.endswith('.cpp'):
            errors.append(f'{relative}: including .cpp is forbidden: {include}')
        if include.endswith(('.hpp', '.hxx', '.hh')):
            errors.append(f'{relative}: C++ headers are forbidden: {include}')
    if relative.suffix != '.cpp':
        return errors
    if (re.search(r'^\s*#\s*ifndef\s+(\w+)\s*\n\s*#\s*define\s+\1\b', clean, re.MULTILINE)
            or re.search(r'^\s*#\s*pragma\s+once\b', clean, re.MULTILINE)):
        errors.append(f'{relative}: .cpp must not use header include guards')
    namespaces = list(re.finditer(r'\bnamespace\b', clean))
    has_main = bool(re.search(r'\bint\s+main\s*\(', clean))
    if len(namespaces) != 1 and not (has_main and not namespaces):
        errors.append(f'{relative}: expected exactly one named namespace')
    elif namespaces:
        declaration = clean[namespaces[0].start():]
        if not re.match(r'namespace\s+[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*\s*\{', declaration):
            errors.append(f'{relative}: anonymous namespaces, aliases and using namespace are forbidden')
    module = re.search(r'^(?:export\s+)?module\s+([\w.]+)\s*;', clean, re.MULTILINE)
    production = relative.parts[0] in ('src', 'include', 'runtime', 'tools')
    bridge = bool(re.search(r'\bextern\s+"C"', text))
    if production and not has_main and not bridge and not module:
        errors.append(f'{relative}: reusable C++ source must declare a named module')
    if module:
        if not re.match(r'\s*module\s*;', clean):
            errors.append(f'{relative}: includes belong in a global module fragment')
        if re.search(r'^\s*#\s*include\b', clean[module.end():], re.MULTILINE):
            errors.append(f'{relative}: textual includes must precede the named module declaration')
    if relative.parts[0] == 'src':
        for include in includes:
            if (include.startswith(('sys/', 'camera/k230/', 'spi/'))
                    or include in ('unistd.h', 'fcntl.h', 'pthread.h', 'signal.h', 'dirent.h')
                    or re.match(r'(?:mpi_|k_|drv_).*\.h$', include)):
                errors.append(f'{relative}: L4 includes a platform implementation header: {include}')
    return errors


def check():
    listed = subprocess.check_output(
        ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=ROOT
    ).decode().split('\0')
    paths = sorted({ROOT / name for name in listed if name and (ROOT / name).is_file()})
    errors, modules, consumers = [], {}, []
    count = 0
    for path in paths:
        relative = path.relative_to(ROOT)
        if path.suffix in ('.hpp', '.hxx', '.hh', '.cc', '.cxx', '.ixx', '.cppm'):
            errors.append(f'{relative}: C++ sources must use .cpp; C++ headers are forbidden')
        if path.suffix in ('.c', '.h', '.cpp'):
            text = path.read_text()
            errors.extend(source_errors(relative, text))
            if path.suffix != '.cpp':
                # Repository .h files describe C ABIs; SDK headers live elsewhere.
                if path.suffix == '.h' and re.search(r'\b(?:class|template)\b', strip_literals(text)):
                    errors.append(f'{relative}: repository headers must remain C ABI headers')
                continue
            count += 1
            clean = strip_literals(text)
            match = re.search(r'^export\s+module\s+([\w.]+)\s*;', clean, re.MULTILINE)
            if match:
                name = match.group(1)
                if name in modules:
                    errors.append(f'{relative}: duplicate module {name}')
                modules[name] = relative
            imports = re.findall(r'^\s*(?:export\s+)?import\s+([\w.]+)\s*;', clean, re.MULTILINE)
            implementation = re.search(r'^module\s+([\w.]+)\s*;', clean, re.MULTILINE)
            if implementation:
                imports.append(implementation.group(1))
            consumers.append((relative, imports))
        elif path.suffix in ('.cmake', '.py') or path.name == 'CMakeLists.txt':
            # Also reject build helpers that generate a textual C++ amalgamation.
            text = path.read_text()
            if re.search(r'#\s*include\s*[^\n]*\.cpp(?:["\'>}]|\\["\'])', text):
                errors.append(f'{relative}: generated .cpp inclusion is forbidden')
    for relative, imports in consumers:
        for dependency in imports:
            owner = modules.get(dependency)
            if owner is None:
                errors.append(f'{relative}: missing named module {dependency}')
            elif relative.parts[0] == 'include' and owner.parts[0] == 'src':
                errors.append(f'{relative}: lower-level library imports L4 module {dependency}')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f'C++ structure passed: {count} translation units, {len(modules)} named modules; no .cpp includes or C++ headers')
    return 0


if __name__ == '__main__':
    sys.exit(check())
