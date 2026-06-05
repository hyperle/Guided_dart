#!/usr/bin/env python3
import argparse
import os
import sys
import time

from guidance_host_common import GuidanceHostPaths, GuidanceProtocolContext, YamlLiteParser


class GuidanceParamSyncTool:
    def __init__(self):
        self._parser = YamlLiteParser()

    def run(self, argv):
        args = self._build_arg_parser().parse_args(argv)
        protocol_ctx = GuidanceProtocolContext(args.schema)
        config = self._parser.parse_file(args.config)

        transport_cfg = config.get("transport", {})
        sync_cfg = config.get("sync", {})
        serial_cfg = config.get("serial", {})
        ble_cfg = config.get("ble", {})

        transport_mode = args.transport or str(transport_cfg.get("mode", "serial"))
        ack_timeout_ms = int(sync_cfg.get("ack_timeout_ms", 300))
        watch_interval_ms = int(sync_cfg.get("watch_interval_ms", 200))

        transport = protocol_ctx.transport_factory.create(args, transport_mode, serial_cfg, ble_cfg)
        transport.open()
        try:
            if args.watch:
                self._watch_file(
                    transport,
                    protocol_ctx,
                    args.config,
                    ack_timeout_ms,
                    watch_interval_ms,
                )
            else:
                self._push_config(transport, protocol_ctx, config, ack_timeout_ms, None)
        finally:
            transport.close()

        return 0

    def _build_arg_parser(self):
        parser = argparse.ArgumentParser(description="Sync guidance parameters to the ESP32 bridge")
        parser.add_argument(
            "--config",
            default=GuidanceHostPaths.default_config_path(),
            help="path to the yaml-like parameter file",
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
        parser.add_argument("--watch", action="store_true", help="watch config file and hot-push changed values")
        return parser

    def _watch_file(self, transport, protocol_ctx, config_path, ack_timeout_ms, watch_interval_ms):
        last_mtime_ns = 0
        last_sent = None

        while True:
            current_mtime_ns = os.stat(config_path).st_mtime_ns
            if current_mtime_ns != last_mtime_ns:
                config = self._parser.parse_file(config_path)
                last_sent = self._push_config(transport, protocol_ctx, config, ack_timeout_ms, last_sent)
                last_mtime_ns = current_mtime_ns
            time.sleep(max(watch_interval_ms, 20) / 1000.0)

    def _push_config(self, transport, protocol_ctx, config, ack_timeout_ms, last_sent):
        items = protocol_ctx.param_registry.build_param_items(config)
        current_map = {item["name"]: item for item in items}
        applied_status_code = protocol_ctx.param_registry.applied_status_code()

        for item in items:
            if last_sent is not None:
                previous = last_sent.get(item["name"])
                if (previous is not None) and (previous["value"] == item["value"]):
                    continue

            frame = protocol_ctx.frame_codec.build_param_frame(item["key"], item["value"])
            transport.send_frame(frame)
            ack_frame = transport.recv_frame(ack_timeout_ms)
            ack = protocol_ctx.frame_codec.decode_param_ack(ack_frame, item["key"])
            ack_value = protocol_ctx.param_registry.format_ack_value(item["type"], ack["value"])
            status_text = protocol_ctx.param_registry.status_text(ack["status"])
            print(f"{item['name']}: {status_text} -> {ack_value}")
            if ack["status"] != applied_status_code:
                raise RuntimeError(f"{item['name']} apply failed: {status_text}")

        return current_map


if __name__ == "__main__":
    raise SystemExit(GuidanceParamSyncTool().run(sys.argv[1:]))
