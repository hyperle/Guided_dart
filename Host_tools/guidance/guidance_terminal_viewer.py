#!/usr/bin/env python3
import argparse
import math
import os
import shutil
import struct
import sys
import time
from dataclasses import dataclass

from guidance_host_common import GuidanceHostPaths, GuidanceProtocolContext, YamlLiteParser


NO_TARGET_COORDINATE = 0xFFFF
GUIDANCE_TELEMETRY_PAYLOAD_LEGACY = 18
GUIDANCE_TELEMETRY_PAYLOAD_EXTENDED = 24
GUIDANCE_TELEMETRY_PAYLOAD_ATTITUDE = 28
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480


@dataclass
class GuidanceTelemetry:
    sequence: int
    protocol_version: int
    target_detected: bool
    task_finished: bool
    task_success: bool
    measurement_x: int
    measurement_y: int
    measurement_area: int
    delta_x: int
    delta_y: int
    setpoint_x: int
    setpoint_y: int
    image_width: int
    image_height: int
    measurement_radius_px: int
    relative_pitch_deg: float
    relative_roll_deg: float
    received_at_s: float


class GuidanceTelemetryDecoder:
    FLAG_TARGET_DETECTED = 1 << 0
    FLAG_TASK_FINISHED = 1 << 1
    FLAG_TASK_SUCCESS = 1 << 2

    def __init__(self, protocol, fallback_image_width, fallback_image_height):
        self._frame_header_0 = protocol.frame_header_0
        self._frame_header_1 = protocol.frame_header_1
        self._message_type_telemetry = protocol.message_type_value("guidance_telemetry")
        self._fallback_image_width = max(1, int(fallback_image_width))
        self._fallback_image_height = max(1, int(fallback_image_height))

    def decode(self, frame):
        if len(frame) < 5:
            return None
        if frame[0] != self._frame_header_0 or frame[1] != self._frame_header_1:
            return None
        if frame[2] != self._message_type_telemetry:
            return None
        if frame[3] != len(frame) - 5:
            return None
        if (sum(frame[:-1]) & 0xFF) != frame[-1]:
            return None

        payload = frame[4:-1]
        payload_size = len(payload)
        now_s = time.monotonic()

        if payload_size == GUIDANCE_TELEMETRY_PAYLOAD_ATTITUDE:
            unpacked = struct.unpack("<HBBHHHhhHHHHHhh", payload)
            image_width = unpacked[10] or self._fallback_image_width
            image_height = unpacked[11] or self._fallback_image_height
            radius_px = unpacked[12]
            relative_pitch_deg = unpacked[13] / 100.0
            relative_roll_deg = unpacked[14] / 100.0
        elif payload_size == GUIDANCE_TELEMETRY_PAYLOAD_EXTENDED:
            unpacked = struct.unpack("<HBBHHHhhHHHHH", payload)
            image_width = unpacked[10] or self._fallback_image_width
            image_height = unpacked[11] or self._fallback_image_height
            radius_px = unpacked[12]
            relative_pitch_deg = 0.0
            relative_roll_deg = 0.0
        elif payload_size == GUIDANCE_TELEMETRY_PAYLOAD_LEGACY:
            unpacked = struct.unpack("<HBBHHHhhHH", payload)
            image_width = self._fallback_image_width
            image_height = self._fallback_image_height
            radius_px = self._estimate_radius_px(unpacked[5])
            relative_pitch_deg = 0.0
            relative_roll_deg = 0.0
        else:
            return None

        if radius_px == 0 and unpacked[5] > 0:
            radius_px = self._estimate_radius_px(unpacked[5])

        return GuidanceTelemetry(
            sequence=unpacked[0],
            protocol_version=unpacked[2],
            target_detected=bool(unpacked[1] & self.FLAG_TARGET_DETECTED),
            task_finished=bool(unpacked[1] & self.FLAG_TASK_FINISHED),
            task_success=bool(unpacked[1] & self.FLAG_TASK_SUCCESS),
            measurement_x=unpacked[3],
            measurement_y=unpacked[4],
            measurement_area=unpacked[5],
            delta_x=unpacked[6],
            delta_y=unpacked[7],
            setpoint_x=unpacked[8],
            setpoint_y=unpacked[9],
            image_width=image_width,
            image_height=image_height,
            measurement_radius_px=radius_px,
            relative_pitch_deg=relative_pitch_deg,
            relative_roll_deg=relative_roll_deg,
            received_at_s=now_s,
        )

    def _estimate_radius_px(self, area_px):
        if area_px <= 0:
            return 0
        radius_px = int(math.isqrt((int(area_px) * 1000) // 3141))
        return max(radius_px, 1)


class TerminalRenderer:
    def __init__(self, refresh_ms):
        self._refresh_ms = max(20, int(refresh_ms))

    def start(self):
        sys.stdout.write("\x1b[2J\x1b[H\x1b[?25l")
        sys.stdout.flush()

    def stop(self):
        sys.stdout.write("\x1b[?25h\x1b[0m\n")
        sys.stdout.flush()

    def render(self, telemetry, yaw_deg=0.0, yaw_delta=0.0, pitch_delta=0.0):
        columns, lines = shutil.get_terminal_size(fallback=(120, 40))
        image_width = max(1, telemetry.image_width)
        image_height = max(1, telemetry.image_height)
        plot_width, plot_height = self._fit_plot(columns, lines, image_width, image_height)
        grid = self._build_grid(plot_width, plot_height)

        setpoint_px = self._resolve_setpoint(telemetry.setpoint_x, telemetry.setpoint_y, image_width, image_height)
        measurement_px = self._resolve_measurement(
            telemetry.measurement_x,
            telemetry.measurement_y,
            image_width,
            image_height,
        )

        setpoint_cell = self._map_point(setpoint_px[0], setpoint_px[1], image_width, image_height, plot_width, plot_height)
        measurement_cell = self._map_point(
            measurement_px[0],
            measurement_px[1],
            image_width,
            image_height,
            plot_width,
            plot_height,
        )

        box_half = max(3, min(6, min(plot_width, plot_height) // 8))
        horizontal_radius = self._scale_radius(telemetry.measurement_radius_px, image_width, plot_width)

        self._draw_box_corner(grid, setpoint_cell[0], setpoint_cell[1], box_half, "tl")
        self._draw_box_corner(grid, setpoint_cell[0], setpoint_cell[1], box_half, "br")
        self._draw_vertical_diameter(grid, setpoint_cell[0], setpoint_cell[1], box_half)
        self._draw_center(grid, setpoint_cell[0], setpoint_cell[1])

        self._draw_box_corner(grid, measurement_cell[0], measurement_cell[1], box_half, "tr")
        self._draw_box_corner(grid, measurement_cell[0], measurement_cell[1], box_half, "bl")
        self._draw_horizontal_diameter(grid, measurement_cell[0], measurement_cell[1], horizontal_radius)
        self._draw_center(grid, measurement_cell[0], measurement_cell[1])

        self._overlay_text(grid, 2, 1, f"Yaw {yaw_deg:+7.1f}  dYaw{yaw_delta:+7.1f}")
        self._overlay_text(grid, 2, 2, f"dPitch{pitch_delta:+7.1f}  Roll{telemetry.relative_roll_deg:+7.1f}")

        output_lines = self._build_status_lines(telemetry, setpoint_px, measurement_px)
        output_lines.extend("".join(row) for row in grid)

        sys.stdout.write("\x1b[H\x1b[J")
        sys.stdout.write("\n".join(output_lines))
        sys.stdout.flush()

    def _fit_plot(self, columns, lines, image_width, image_height):
        available_columns = max(12, columns - 2)
        available_rows = max(10, lines - 6)
        target_ratio = (float(image_width) / float(max(image_height, 1))) * 2.0

        plot_width = min(available_columns - 2, max(10, int(round((available_rows - 2) * target_ratio))))
        plot_height = max(6, int(round(plot_width / max(target_ratio, 0.1))))

        if plot_height > (available_rows - 2):
            plot_height = available_rows - 2
            plot_width = max(10, min(available_columns - 2, int(round(plot_height * target_ratio))))

        return max(10, plot_width), max(6, plot_height)

    def _build_grid(self, plot_width, plot_height):
        total_width = plot_width + 2
        total_height = plot_height + 2
        grid = [[" " for _ in range(total_width)] for _ in range(total_height)]

        for col in range(total_width):
            grid[0][col] = "-"
            grid[total_height - 1][col] = "-"
        for row in range(total_height):
            grid[row][0] = "|"
            grid[row][total_width - 1] = "|"

        grid[0][0] = "+"
        grid[0][total_width - 1] = "+"
        grid[total_height - 1][0] = "+"
        grid[total_height - 1][total_width - 1] = "+"
        return grid

    def _build_status_lines(self, telemetry, setpoint_px, measurement_px):
        age_ms = int((time.monotonic() - telemetry.received_at_s) * 1000.0)
        target_text = "detected" if telemetry.target_detected else "lost"
        task_text = "success" if telemetry.task_success else "pending/fail"
        return [
            "Guidance terminal viewer",
            (
                f"seq={telemetry.sequence} ver={telemetry.protocol_version} "
                f"image={telemetry.image_width}x{telemetry.image_height} "
                f"target={target_text} task={task_text} age={age_ms}ms"
            ),
            (
                f"setpoint=({setpoint_px[0]},{setpoint_px[1]}) "
                f"measurement=({measurement_px[0]},{measurement_px[1]}) "
                f"radius={telemetry.measurement_radius_px}px area={telemetry.measurement_area}"
            ),
            f"delta=({telemetry.delta_x},{telemetry.delta_y}) refresh<={self._refresh_ms}ms",
        ]

    def _resolve_setpoint(self, setpoint_x, setpoint_y, image_width, image_height):
        return (
            self._resolve_axis(setpoint_x, image_width),
            self._resolve_axis(setpoint_y, image_height),
        )

    def _resolve_measurement(self, measurement_x, measurement_y, image_width, image_height):
        if measurement_x == NO_TARGET_COORDINATE or measurement_y == NO_TARGET_COORDINATE:
            return (image_width // 2, image_height // 2)
        return (
            self._clamp_axis(measurement_x, image_width),
            self._clamp_axis(measurement_y, image_height),
        )

    def _resolve_axis(self, value, limit):
        if limit <= 0:
            return 0
        if value == NO_TARGET_COORDINATE:
            return limit // 2
        return self._clamp_axis(value, limit)

    def _clamp_axis(self, value, limit):
        if limit <= 0:
            return 0
        return max(0, min(int(value), limit - 1))

    def _map_point(self, x_px, y_px, image_width, image_height, plot_width, plot_height):
        if image_width <= 1:
            col = 1 + (plot_width // 2)
        else:
            col = 1 + int(round((float(x_px) * float(plot_width - 1)) / float(image_width - 1)))
        if image_height <= 1:
            row = 1 + (plot_height // 2)
        else:
            row = 1 + int(round((float(y_px) * float(plot_height - 1)) / float(image_height - 1)))
        return col, row

    def _scale_radius(self, radius_px, image_width, plot_width):
        if radius_px <= 0 or image_width <= 1:
            return 0
        scaled = int(round((float(radius_px) * float(plot_width - 1)) / float(image_width - 1)))
        return max(1, scaled)

    def _draw_box_corner(self, grid, center_x, center_y, half, corner):
        if corner == "tl":
            for dx in range(-half, 1):
                self._plot(grid, center_x + dx, center_y - half, "-")
            for dy in range(-half, 1):
                self._plot(grid, center_x - half, center_y + dy, "|")
        elif corner == "tr":
            for dx in range(0, half + 1):
                self._plot(grid, center_x + dx, center_y - half, "-")
            for dy in range(-half, 1):
                self._plot(grid, center_x + half, center_y + dy, "|")
        elif corner == "bl":
            for dx in range(-half, 1):
                self._plot(grid, center_x + dx, center_y + half, "-")
            for dy in range(0, half + 1):
                self._plot(grid, center_x - half, center_y + dy, "|")
        elif corner == "br":
            for dx in range(0, half + 1):
                self._plot(grid, center_x + dx, center_y + half, "-")
            for dy in range(0, half + 1):
                self._plot(grid, center_x + half, center_y + dy, "|")

    def _draw_vertical_diameter(self, grid, center_x, center_y, radius):
        for offset in range(-radius, radius + 1):
            self._plot(grid, center_x, center_y + offset, "|")

    def _draw_horizontal_diameter(self, grid, center_x, center_y, radius):
        for offset in range(-radius, radius + 1):
            self._plot(grid, center_x + offset, center_y, "-")

    def _draw_center(self, grid, center_x, center_y):
        self._plot(grid, center_x, center_y, "+")

    def _overlay_text(self, grid, x, y, text):
        if y <= 0 or y >= (len(grid) - 1):
            return

        for index, char in enumerate(text):
            col = x + index
            if col <= 0 or col >= (len(grid[0]) - 1):
                break
            grid[y][col] = char

    def _plot(self, grid, x, y, char):
        if y <= 0 or y >= (len(grid) - 1) or x <= 0 or x >= (len(grid[0]) - 1):
            return

        current = grid[y][x]
        if char == "+":
            grid[y][x] = "+"
            return
        if char == "-":
            if current == "|":
                grid[y][x] = "+"
            elif current != "+":
                grid[y][x] = "-"
            return
        if char == "|":
            if current == "-":
                grid[y][x] = "+"
            elif current != "+":
                grid[y][x] = "|"
            return
        if current == " ":
            grid[y][x] = char


def _default_telemetry(image_width: int, image_height: int) -> GuidanceTelemetry:
    return GuidanceTelemetry(
        sequence=0,
        protocol_version=0,
        target_detected=False,
        task_finished=False,
        task_success=False,
        measurement_x=NO_TARGET_COORDINATE,
        measurement_y=NO_TARGET_COORDINATE,
        measurement_area=0,
        delta_x=0,
        delta_y=0,
        setpoint_x=NO_TARGET_COORDINATE,
        setpoint_y=NO_TARGET_COORDINATE,
        image_width=image_width,
        image_height=image_height,
        measurement_radius_px=0,
        relative_pitch_deg=0.0,
        relative_roll_deg=0.0,
        received_at_s=0.0,
    )


class GuidanceTerminalViewer:
    def __init__(self):
        self._parser = YamlLiteParser()

    def run(self, argv):
        args = self._build_arg_parser().parse_args(argv)
        protocol_ctx = GuidanceProtocolContext(args.schema)
        config = self._load_config(args.config)

        transport_cfg = config.get("transport", {})
        serial_cfg = config.get("serial", {})
        ble_cfg = config.get("ble", {})
        viewer_cfg = config.get("viewer", {})

        transport_mode = args.transport or str(transport_cfg.get("mode", "serial"))
        image_width = int(args.image_width or viewer_cfg.get("fallback_image_width", DEFAULT_IMAGE_WIDTH))
        image_height = int(args.image_height or viewer_cfg.get("fallback_image_height", DEFAULT_IMAGE_HEIGHT))
        refresh_ms = int(args.refresh_ms or viewer_cfg.get("refresh_ms", 50))

        transport = protocol_ctx.transport_factory.create(args, transport_mode, serial_cfg, ble_cfg)
        decoder = GuidanceTelemetryDecoder(protocol_ctx.protocol, image_width, image_height)
        renderer = TerminalRenderer(refresh_ms)

        transport.open()
        renderer.start()
        try:
            last_render_s = 0.0
            latest = None
            prev_pitch = None
            prev_yaw = None

            while True:
                try:
                    frame = transport.recv_frame(refresh_ms)
                except TimeoutError:
                    now_s = time.monotonic()
                    if (now_s - last_render_s) * 1000.0 >= refresh_ms:
                        if latest is not None:
                            renderer.render(latest)
                        else:
                            renderer.render(_default_telemetry(image_width, image_height))
                        last_render_s = now_s
                    continue

                telemetry = decoder.decode(frame)
                if telemetry is None:
                    continue

                pitch = telemetry.relative_pitch_deg
                yaw = 0.0

                pitch_delta = pitch - prev_pitch if prev_pitch is not None else 0.0
                yaw_delta = yaw - prev_yaw if prev_yaw is not None else 0.0
                prev_pitch = pitch
                prev_yaw = yaw

                latest = telemetry
                now_s = time.monotonic()
                if (now_s - last_render_s) * 1000.0 >= refresh_ms:
                    renderer.render(latest, yaw_deg=yaw, yaw_delta=yaw_delta, pitch_delta=pitch_delta)
                    last_render_s = now_s
        except KeyboardInterrupt:
            return 0
        finally:
            renderer.stop()
            transport.close()

    def _build_arg_parser(self):
        parser = argparse.ArgumentParser(description="Render guidance telemetry as a terminal reticle animation")
        parser.add_argument(
            "--pipe",
            default="",
            help="read telemetry from a pipe (created by ble-connect --pipe), skip BLE connection",
        )
        parser.add_argument(
            "--config",
            default=GuidanceHostPaths.default_config_path(),
            help="path to the yaml-like config file",
        )
        parser.add_argument(
            "--schema",
            default=GuidanceHostPaths.protocol_schema_path(),
            help="path to the shared protocol schema",
        )
        parser.add_argument(
            "--transport",
            choices=["serial", "ble"],
            default="",
            help="override transport.mode from config",
        )
        parser.add_argument("--port", default="", help="override serial.port from config")
        parser.add_argument("--baudrate", type=int, default=0, help="override serial.baudrate from config")
        parser.add_argument("--ble-device-name", default="", help="override ble.device_name from config")
        parser.add_argument("--ble-address", default="", help="override ble.address from config")
        parser.add_argument("--ble-connect-timeout-ms", type=int, default=0, help="override ble.connect_timeout_ms")
        parser.add_argument("--ble-service-uuid", default="", help="override ble.service_uuid from config")
        parser.add_argument("--ble-downlink-char-uuid", default="", help="override ble.downlink_char_uuid")
        parser.add_argument("--ble-uplink-char-uuid", default="", help="override ble.uplink_char_uuid")
        parser.add_argument("--ble-ack-char-uuid", default="", help="override ble.ack_char_uuid")
        parser.add_argument("--image-width", type=int, default=0, help="fallback image width for legacy telemetry")
        parser.add_argument("--image-height", type=int, default=0, help="fallback image height for legacy telemetry")
        parser.add_argument("--refresh-ms", type=int, default=0, help="limit terminal redraw cadence")
        return parser

    def _load_config(self, config_path):
        if not os.path.exists(config_path):
            return {}
        return self._parser.parse_file(config_path)


if __name__ == "__main__":
    raise SystemExit(GuidanceTerminalViewer().run(sys.argv[1:]))
