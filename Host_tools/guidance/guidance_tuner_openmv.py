from __future__ import annotations

import os
import threading
from types import SimpleNamespace

import cv2
import numpy as np

from guidance_host_common import GuidanceProtocolContext
from guidance_video_fetch import RawReplSession, load_config, send_passthrough_enable
from guidance_tuner_core import (
    BlobCandidate,
    DetectionDebug,
    DetectorParams,
    ValidationFrameResult,
    ValidationRunConfig,
    apply_simulation,
    point_in_roi,
)


DEFAULT_VALIDATION_CHUNK_SIZE = 16
DEFAULT_VALIDATION_CHUNK_BYTES = 512 * 1024


class OpenMvEvaluator:
    def __init__(self, repo_root: str, config_path: str, port_override: str = "", baudrate_override: int = 0):
        self._repo_root = repo_root
        self._config_path = config_path
        self._port_override = port_override
        self._baudrate_override = int(baudrate_override)
        self._protocol_ctx = GuidanceProtocolContext()
        self._config = load_config(config_path)
        self._session: RawReplSession | None = None
        self._workspace = ""
        self._connected = False
        self._single_cache_key: tuple[int, float, float, float, float, float] | None = None
        self._single_remote_path = ""
        self._operation_lock = threading.RLock()
        self._active_operations = 0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def workspace(self) -> str:
        return self._workspace

    @property
    def busy(self) -> bool:
        with self._operation_lock:
            return self._active_operations > 0

    def _begin_operation(self) -> None:
        self._operation_lock.acquire()
        self._active_operations += 1

    def _end_operation(self) -> None:
        try:
            self._active_operations = max(0, self._active_operations - 1)
        finally:
            self._operation_lock.release()

    def connect(self) -> None:
        self._begin_operation()
        try:
            if self._connected:
                return

            args = SimpleNamespace(
                port=self._port_override,
                baudrate=self._baudrate_override,
                ble_address="",
                ble_device_name="",
                ble_connect_timeout_ms=0,
                ble_service_uuid="",
                ble_downlink_char_uuid="",
                ble_uplink_char_uuid="",
            )
            send_passthrough_enable(self._config, args, self._protocol_ctx)

            serial_cfg = self._config.get("serial", {})
            port = self._port_override or str(serial_cfg.get("port", ""))
            baudrate = int(self._baudrate_override or serial_cfg.get("baudrate", 115200))
            if not port:
                raise ValueError("serial.port is required for OpenMV tuning")

            self._session = RawReplSession(port, baudrate)
            self._session.__enter__()
            try:
                self._upload_support_file()
                self._workspace = self._prepare_workspace()
                self._connected = True
                self._single_cache_key = None
                self._single_remote_path = ""
            except Exception:
                self.disconnect()
                raise
        finally:
            self._end_operation()

    def disconnect(self) -> None:
        self._begin_operation()
        try:
            if self._session is not None:
                try:
                    self._session.__exit__(None, None, None)
                finally:
                    self._session = None
            self._workspace = ""
            self._connected = False
            self._single_cache_key = None
            self._single_remote_path = ""
        finally:
            self._end_operation()

    def _exec(self, command: str, timeout_s: float) -> str:
        if self._session is None:
            raise RuntimeError("OpenMV session is not connected")
        return self._session.exec_raw(command, timeout_s=timeout_s)

    def _upload_support_file(self) -> None:
        support_path = os.path.join(self._repo_root, "OpenMV_guidance", "src", "tuner_runtime.py")
        with open(support_path, "rb") as handle:
            payload = handle.read()
        self._write_binary_file("tuner_runtime.py", payload)

    def _prepare_workspace(self) -> str:
        output = self._exec(
            "\n".join(
                [
                    "import sys",
                    "sys.modules.pop('tuner_runtime', None)",
                    "import tuner_runtime",
                    "root = tuner_runtime.detect_storage_root()",
                    "if not root:",
                    "    root = '.'",
                    "workspace = root + '/tuner_frames'",
                    "tuner_runtime.ensure_clean_dir(workspace)",
                    "tuner_runtime.clear_sequence_state()",
                    "print(workspace)",
                ]
            )
            + "\n",
            timeout_s=10.0,
        ).strip()
        if not output:
            raise RuntimeError("failed to prepare OpenMV tuner workspace")
        return output.splitlines()[-1].strip()

    def clear_workspace(self) -> None:
        self._begin_operation()
        try:
            if not self._connected or not self._workspace:
                return
            self._exec(
                "import tuner_runtime\n"
                f"tuner_runtime.ensure_clean_dir({self._workspace!r})\n"
                "print('ok')\n",
                timeout_s=10.0,
            )
            self._single_cache_key = None
            self._single_remote_path = ""
        finally:
            self._end_operation()

    def _write_binary_file(self, remote_path: str, payload: bytes, chunk_size: int = 512) -> None:
        if not payload:
            self._exec(f"f = open({remote_path!r}, 'wb')\nf.close()\nprint('ok')\n", timeout_s=5.0)
            return

        offset = 0
        mode = "wb"
        while offset < len(payload):
            chunk = payload[offset : offset + chunk_size]
            command = (
                "import ubinascii\n"
                f"f = open({remote_path!r}, {mode!r})\n"
                f"f.write(ubinascii.unhexlify({chunk.hex()!r}))\n"
                "f.close()\n"
                f"print({offset + len(chunk)})\n"
            )
            self._exec(command, timeout_s=10.0)
            offset += len(chunk)
            mode = "ab"

    @staticmethod
    def params_dict(params: DetectorParams) -> dict[str, int]:
        return {
            "threshold_l_min": int(params.threshold_l_min),
            "threshold_l_max": int(params.threshold_l_max),
            "threshold_a_min": int(params.threshold_a_min),
            "threshold_a_max": int(params.threshold_a_max),
            "threshold_b_min": int(params.threshold_b_min),
            "threshold_b_max": int(params.threshold_b_max),
            "min_area": int(params.min_area),
            "max_area": int(params.max_area),
            "roundness_min_x1000": int(params.roundness_min_x1000),
            "merge_margin": int(params.merge_margin),
            "track_window_radius_px": int(params.track_window_radius_px),
            "center_filter_gain_x100": int(params.center_filter_gain_x100),
            "max_missed_frames": int(params.max_missed_frames),
        }

    @staticmethod
    def encode_frame(frame_bgr: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            raise RuntimeError("failed to encode frame for OpenMV upload")
        return bytes(encoded)

    def upload_single_frame(self, frame_bgr: np.ndarray, cache_key: tuple[int, float, float, float, float, float]) -> str:
        self._begin_operation()
        try:
            if self._single_cache_key == cache_key and self._single_remote_path:
                return self._single_remote_path
            if not self._workspace:
                raise RuntimeError("OpenMV workspace is not ready")

            remote_path = self._workspace + "/single.jpg"
            self._write_binary_file(remote_path, self.encode_frame(frame_bgr))
            self._single_cache_key = cache_key
            self._single_remote_path = remote_path
            return remote_path
        finally:
            self._end_operation()

    def evaluate_single_frame(
        self,
        remote_path: str,
        params: DetectorParams,
        expected_roi: tuple[int, int, int, int] | None,
    ) -> DetectionDebug:
        self._begin_operation()
        try:
            command = "\n".join(
                [
                    "import tuner_runtime",
                    f"params = {self.params_dict(params)!r}",
                    f"expected_roi = {(expected_roi,)!r}[0]",
                    f"result = tuner_runtime.evaluate_single_frame({remote_path!r}, params, expected_roi)",
                    "print('meta|reason|' + result['reason'])",
                    "print('meta|detected|' + str(1 if result['detected'] else 0))",
                    "print('meta|locked|' + str(1 if result['locked'] else 0))",
                    "print('meta|background|' + str(1 if result['background_misdetect'] else 0))",
                    "print('meta|fallback|' + str(1 if result['fallback_used'] else 0))",
                    "print('meta|area|' + str(result['area']))",
                    "print('meta|radius|' + str(result['radius_px']))",
                    "if result['search_roi'] is not None:",
                    "    print('search_roi|%d|%d|%d|%d' % result['search_roi'])",
                    "if result['raw_center'] is not None:",
                    "    print('raw_center|%d|%d' % result['raw_center'])",
                    "if result['filtered_center'] is not None:",
                    "    print('filtered_center|%d|%d' % result['filtered_center'])",
                    "for candidate in result['candidates']:",
                    "    rect = candidate['rect']",
                    "    print('cand|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d' % (candidate['center_x'], candidate['center_y'], candidate['area'], candidate['roundness_x1000'], candidate['radius_px'], rect[0], rect[1], rect[2], rect[3], 1 if candidate['passes_area'] else 0, 1 if candidate['passes_roundness'] else 0))",
                ]
            ) + "\n"
            return self.parse_single_frame_result(self._exec(command, timeout_s=20.0), expected_roi)
        finally:
            self._end_operation()

    @staticmethod
    def parse_single_frame_result(output: str, expected_roi: tuple[int, int, int, int] | None) -> DetectionDebug:
        reason = ""
        detected = False
        locked = False
        background_misdetect = False
        fallback_used = False
        area = 0
        radius_px = 0
        raw_center: tuple[int, int] | None = None
        filtered_center: tuple[int, int] | None = None
        search_roi: tuple[int, int, int, int] | None = None
        candidates: list[BlobCandidate] = []

        for raw_line in output.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split("|")
            if parts[0] == "meta" and len(parts) >= 3:
                if parts[1] == "reason":
                    reason = "|".join(parts[2:])
                elif parts[1] == "detected":
                    detected = parts[2] == "1"
                elif parts[1] == "locked":
                    locked = parts[2] == "1"
                elif parts[1] == "background":
                    background_misdetect = parts[2] == "1"
                elif parts[1] == "fallback":
                    fallback_used = parts[2] == "1"
                elif parts[1] == "area":
                    area = int(parts[2])
                elif parts[1] == "radius":
                    radius_px = int(parts[2])
            elif parts[0] == "search_roi" and len(parts) == 5:
                search_roi = (int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
            elif parts[0] == "raw_center" and len(parts) == 3:
                raw_center = (int(parts[1]), int(parts[2]))
            elif parts[0] == "filtered_center" and len(parts) == 3:
                filtered_center = (int(parts[1]), int(parts[2]))
            elif parts[0] == "cand" and len(parts) == 12:
                center = (int(parts[1]), int(parts[2]))
                bbox = (int(parts[6]), int(parts[7]), int(parts[8]), int(parts[9]))
                candidates.append(
                    BlobCandidate(
                        contour=None,
                        area=int(parts[3]),
                        roundness_x1000=int(parts[4]),
                        center_x=center[0],
                        center_y=center[1],
                        radius_px=int(parts[5]),
                        bbox=bbox,
                        passes_area=parts[10] == "1",
                        passes_roundness=parts[11] == "1",
                        overlaps_expected_roi=point_in_roi(center, expected_roi),
                    )
                )

        best_candidate = None
        if raw_center is not None:
            for candidate in candidates:
                if candidate.center_x == raw_center[0] and candidate.center_y == raw_center[1] and candidate.area == area:
                    best_candidate = candidate
                    break

        return DetectionDebug(
            candidates=candidates,
            best_candidate=best_candidate,
            mask=np.zeros((1, 1), dtype=np.uint8),
            search_roi=search_roi,
            fallback_used=fallback_used,
            reason=reason or "未返回结果",
            detected=detected,
            locked=locked,
            background_misdetect=background_misdetect,
            raw_center=raw_center,
            filtered_center=filtered_center,
            area=area,
            radius_px=radius_px,
        )

    def _reset_sequence_state(self, params: DetectorParams) -> None:
        command = (
            "import tuner_runtime\n"
            f"tuner_runtime.reset_sequence_state({self.params_dict(params)!r})\n"
            "print('ok')\n"
        )
        self._exec(command, timeout_s=10.0)

    def _clear_sequence_state(self) -> None:
        if not self._connected:
            return
        self._exec("import tuner_runtime\ntuner_runtime.clear_sequence_state()\nprint('ok')\n", timeout_s=10.0)

    def _evaluate_validation_chunk(
        self,
        start_index: int,
        batch_names: list[str],
        params: DetectorParams,
        expected_roi: tuple[int, int, int, int] | None,
        frame_scales: list[tuple[float, float]],
    ) -> list[ValidationFrameResult]:
        command = (
            "import tuner_runtime\n"
            f"names = {batch_names!r}\n"
            f"params = {self.params_dict(params)!r}\n"
            f"expected_roi = {expected_roi!r}\n"
            f"tuner_runtime.print_sequence_chunk_results({self._workspace!r}, names, params, expected_roi, start_index={start_index})\n"
        )
        timeout_s = max(30.0, float(len(batch_names)) * 1.5)
        output = self._exec(command, timeout_s=timeout_s)

        expected_total: int | None = None
        results: list[ValidationFrameResult] = []
        for raw_line in output.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("total|"):
                parts = line.split("|", 1)
                if len(parts) == 2:
                    expected_total = int(parts[1])
                continue
            if not line.startswith("frame|"):
                continue
            parts = line.split("|", 12)
            if len(parts) != 13:
                continue

            frame_index = int(parts[1])
            batch_offset = frame_index - start_index
            if batch_offset < 0 or batch_offset >= len(batch_names):
                raise RuntimeError(f"OpenMV validation returned unexpected frame index {frame_index}")
            if parts[2] != batch_names[batch_offset]:
                raise RuntimeError(f"OpenMV validation returned frame name {parts[2]!r}, expected {batch_names[batch_offset]!r}")

            exposure_scale, gain_scale = frame_scales[batch_offset]
            raw_x = int(parts[6])
            raw_y = int(parts[7])
            filtered_x = int(parts[8])
            filtered_y = int(parts[9])
            results.append(
                ValidationFrameResult(
                    frame_index=frame_index,
                    detected=parts[3] == "1",
                    locked=parts[4] == "1",
                    background_misdetect=parts[5] == "1",
                    reason=parts[12],
                    raw_center=None if raw_x < 0 or raw_y < 0 else (raw_x, raw_y),
                    filtered_center=None if filtered_x < 0 or filtered_y < 0 else (filtered_x, filtered_y),
                    area=int(parts[10]),
                    radius_px=int(parts[11]),
                    exposure_scale=exposure_scale,
                    gain_scale=gain_scale,
                )
            )

        if expected_total is not None and expected_total != len(batch_names):
            raise RuntimeError(f"OpenMV validation reported total={expected_total}, expected {len(batch_names)} uploaded frames")
        if len(results) != len(batch_names):
            raise RuntimeError(f"OpenMV validation returned {len(results)} frame results, expected {len(batch_names)}")
        return results

    def stream_validation(self, video_source, run_config: ValidationRunConfig, progress_callback=None) -> list[ValidationFrameResult]:
        self._begin_operation()
        try:
            if not self._workspace:
                raise RuntimeError("OpenMV workspace is not ready")

            total_frames = video_source.frame_count
            results: list[ValidationFrameResult] = []
            batch_names: list[str] = []
            batch_payloads: list[bytes] = []
            batch_scales: list[tuple[float, float]] = []
            batch_start_index = 0
            batch_bytes = 0

            def flush_batch() -> None:
                nonlocal batch_names, batch_payloads, batch_scales, batch_start_index, batch_bytes, results
                if not batch_names:
                    return
                self.clear_workspace()
                for name, payload in zip(batch_names, batch_payloads):
                    self._write_binary_file(self._workspace + "/" + name, payload)
                results.extend(
                    self._evaluate_validation_chunk(
                        batch_start_index,
                        batch_names,
                        run_config.detector_params,
                        run_config.expected_roi,
                        batch_scales,
                    )
                )
                completed = batch_start_index + len(batch_names)
                if progress_callback is not None:
                    progress_callback(completed, total_frames)
                batch_names = []
                batch_payloads = []
                batch_scales = []
                batch_bytes = 0

            self._reset_sequence_state(run_config.detector_params)
            try:
                for frame_index, frame in enumerate(video_source.iter_frames()):
                    if run_config.use_sweep:
                        exposure_scale, gain_scale = run_config.sweep_params.values_for_frame(frame_index)
                    else:
                        exposure_scale, gain_scale = (1.0, 1.0)

                    adjusted = apply_simulation(
                        frame,
                        run_config.sim_params,
                        dynamic_exposure_scale=exposure_scale,
                        dynamic_gain_scale=gain_scale,
                    )
                    payload = self.encode_frame(adjusted)

                    if not batch_names:
                        batch_start_index = frame_index
                    elif len(batch_names) >= DEFAULT_VALIDATION_CHUNK_SIZE or (batch_bytes + len(payload)) > DEFAULT_VALIDATION_CHUNK_BYTES:
                        flush_batch()
                        batch_start_index = frame_index

                    batch_names.append(f"frame_{len(batch_names):05d}.jpg")
                    batch_payloads.append(payload)
                    batch_scales.append((exposure_scale, gain_scale))
                    batch_bytes += len(payload)

                flush_batch()
            finally:
                self.clear_workspace()
                self._clear_sequence_state()

            return results
        finally:
            self._end_operation()
