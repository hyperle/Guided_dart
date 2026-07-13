#!/usr/bin/env python3
import argparse
import os
import sys
from dataclasses import dataclass


class YamlLiteParser:
    def __init__(self):
        self._root = {}
        self._stack = [(-1, self._root)]

    def parse_file(self, path):
        self._root = {}
        self._stack = [(-1, self._root)]

        with open(path, "r", encoding="utf-8") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                self._parse_line(raw_line.rstrip("\n"), line_number)

        return self._root

    def _parse_line(self, line, line_number):
        content = line.split("#", 1)[0].rstrip()
        if not content:
            return

        indent = len(content) - len(content.lstrip(" "))
        if indent % 2 != 0:
            raise ValueError(f"line {line_number}: indentation must use 2 spaces")

        level = indent // 2
        while self._stack and self._stack[-1][0] >= level:
            self._stack.pop()

        if not self._stack:
            raise ValueError(f"line {line_number}: invalid indentation")

        container = self._stack[-1][1]
        stripped = content.lstrip(" ")

        if ":" not in stripped:
            raise ValueError(f"line {line_number}: missing ':'")

        key, value = stripped.split(":", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            raise ValueError(f"line {line_number}: empty key")

        if not value:
            child = {}
            container[key] = child
            self._stack.append((level, child))
            return

        container[key] = self._parse_scalar(value)

    def _parse_scalar(self, value):
        lowered = value.lower()
        if lowered == "true":
            return True
        if lowered == "false":
            return False

        if value.startswith(("'", '"')) and value.endswith(("'", '"')) and len(value) >= 2:
            return value[1:-1]

        try:
            if any(marker in value for marker in (".", "e", "E")):
                return float(value)
            if value.startswith(("+0x", "-0x", "0x", "+0X", "-0X", "0X")):
                return int(value, 16)
            return int(value, 10)
        except ValueError:
            return value


@dataclass(frozen=True)
class GuidanceNamedValue:
    schema_name: str
    macro: str
    value: int


class GuidanceProtocolDefinition:
    def __init__(self, schema_path, schema_root):
        self.schema_path = os.path.abspath(schema_path)
        self.frame_header_0 = 0
        self.frame_header_1 = 0
        self._message_types = {}
        self._load(schema_root)

    def iter_message_types(self):
        return tuple(self._message_types.values())

    def message_type_value(self, name):
        return self._message_types[name].value

    def _load(self, schema_root):
        frame = self._require_dict(schema_root, "frame")
        self.frame_header_0 = self._require_int(frame, "frame.header_0")
        self.frame_header_1 = self._require_int(frame, "frame.header_1")
        self._message_types = self._parse_named_values(schema_root, "message_types")

    def _parse_named_values(self, schema_root, section_name):
        section = self._require_dict(schema_root, section_name)
        named_values = {}
        seen_values = set()
        seen_macros = set()

        for schema_name, raw_value in section.items():
            value_dict = self._require_dict(section, f"{section_name}.{schema_name}", raw_value)
            macro = self._require_string(value_dict, f"{section_name}.{schema_name}.macro")
            value = self._require_int(value_dict, f"{section_name}.{schema_name}.value")
            if macro in seen_macros:
                raise ValueError(f"duplicate macro in {section_name}: {macro}")
            if value in seen_values:
                raise ValueError(f"duplicate value in {section_name}: {value}")
            named_values[schema_name] = GuidanceNamedValue(schema_name=schema_name, macro=macro, value=value)
            seen_values.add(value)
            seen_macros.add(macro)

        return named_values

    def _require_dict(self, container, key, value=None):
        raw_value = container.get(key) if value is None else value
        if not isinstance(raw_value, dict):
            raise ValueError(f"{key} must be a mapping")
        return raw_value

    def _require_string(self, container, key):
        value = container.get(key.rsplit(".", 1)[-1])
        if not isinstance(value, str) or not value:
            raise ValueError(f"{key} must be a non-empty string")
        return value

    def _require_int(self, container, key):
        value = container.get(key.rsplit(".", 1)[-1])
        if not isinstance(value, int):
            raise ValueError(f"{key} must be an integer")
        if value < 0:
            raise ValueError(f"{key} must be non-negative")
        return value


class GuidanceProtocolLoader:
    def __init__(self):
        self._parser = YamlLiteParser()

    def load(self, schema_path):
        schema_root = self._parser.parse_file(schema_path)
        return GuidanceProtocolDefinition(schema_path, schema_root)


class GuidanceProtocolHeaderGenerator:
    def __init__(self, protocol):
        self._protocol = protocol

    def build_text(self):
        lines = [
            "#ifndef GUIDANCE_PROTOCOL_GENERATED_H",
            "#define GUIDANCE_PROTOCOL_GENERATED_H",
            "",
            "/* Auto-generated from protocol/guidance_protocol.yaml. */",
            "",
            f"#define GUIDANCE_PROTOCOL_FRAME_HEADER_0 {self._format_u32(self._protocol.frame_header_0)}",
            f"#define GUIDANCE_PROTOCOL_FRAME_HEADER_1 {self._format_u32(self._protocol.frame_header_1)}",
            "",
        ]

        for item in self._protocol.iter_message_types():
            lines.append(f"#define {item.macro} {self._format_u32(item.value)}")

        lines.extend([
            "",
            "#endif",
            "",
        ])
        return "\n".join(lines)

    def _format_u32(self, value):
        if value <= 0xFF:
            return f"0x{value:02X}U"
        if value <= 0xFFFF:
            return f"0x{value:04X}U"
        return f"0x{value:08X}U"


class GuidanceProtocolCli:
    def __init__(self):
        self._loader = GuidanceProtocolLoader()

    def run(self, argv):
        args = self._build_arg_parser().parse_args(argv)
        protocol = self._loader.load(args.schema)

        if args.header:
            content = GuidanceProtocolHeaderGenerator(protocol).build_text()
            self._write_if_changed(args.header, content)

        if args.print_summary:
            self._print_summary(protocol)

        return 0

    def _build_arg_parser(self):
        parser = argparse.ArgumentParser(description="Load the shared guidance uplink protocol schema")
        parser.add_argument(
            "--schema",
            default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "guidance_protocol.yaml"),
            help="path to the shared protocol schema",
        )
        parser.add_argument("--header", default="", help="write the generated C header to this path")
        parser.add_argument("--print-summary", action="store_true", help="print a short schema summary")
        return parser

    def _write_if_changed(self, path, content):
        existing = None
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as handle:
                existing = handle.read()
        if existing == content:
            return

        with open(path, "w", encoding="utf-8") as handle:
            handle.write(content)

    def _print_summary(self, protocol):
        print(f"schema={protocol.schema_path}")
        print(f"frame_header=0x{protocol.frame_header_0:02X} 0x{protocol.frame_header_1:02X}")
        print(f"message_types={len(protocol.iter_message_types())}")


if __name__ == "__main__":
    raise SystemExit(GuidanceProtocolCli().run(sys.argv[1:]))
