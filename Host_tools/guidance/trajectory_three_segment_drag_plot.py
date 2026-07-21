#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SegmentSpec:
    name: str
    start_x_m: float
    end_x_m: float
    alpha_deg: float


@dataclass(frozen=True)
class TrajectorySample:
    x_m: float
    z_m: float
    gamma_deg: float
    t_s: float
    alpha_deg: float
    cd: float
    segment_index: int


@dataclass(frozen=True)
class TrajectorySummary:
    initial_speed_mps: float
    impact_time_s: float
    impact_gamma_deg: float
    terminal_start_x_m: float
    terminal_start_z_m: float
    terminal_start_gamma_deg: float
    terminal_start_in_corridor: bool


@dataclass(frozen=True)
class TrajectoryCaseResult:
    stall_limit_deg: float
    planned_segments: tuple[SegmentSpec, ...]
    planned_samples: list[TrajectorySample]
    planned_summary: TrajectorySummary
    baseline_samples: list[TrajectorySample]
    baseline_summary: TrajectorySummary


@dataclass(frozen=True)
class ModelConfig:
    range_m: float
    terminal_range_m: float
    launch_angle_deg: float
    impact_height_m: float
    gravity_mps2: float
    drag_scale: float
    cd0: float
    cd_aoa_abs: float
    cd_aoa_sq: float
    normal_accel_scale: float
    segment1_end_x_m: float
    samples: int
    dt_s: float
    max_time_s: float
    output_prefix: Path


class DragAwareTrajectoryModel:
    def __init__(self, config: ModelConfig) -> None:
        self.config = config
        self.launch_angle_rad = math.radians(config.launch_angle_deg)

    def solve_case(self, stall_limit_deg: float) -> TrajectoryCaseResult:
        planned_segments = self._build_planned_segments(stall_limit_deg)
        baseline_segments = self._build_baseline_segments()

        planned_v0 = self._solve_initial_speed(planned_segments)
        baseline_v0 = self._solve_initial_speed(baseline_segments)

        planned_samples = self._sample_trajectory(planned_v0, planned_segments)
        baseline_samples = self._sample_trajectory(baseline_v0, baseline_segments)

        planned_summary = self._build_summary(planned_samples, planned_v0)
        baseline_summary = self._build_summary(baseline_samples, baseline_v0)
        return TrajectoryCaseResult(
            stall_limit_deg=stall_limit_deg,
            planned_segments=planned_segments,
            planned_samples=planned_samples,
            planned_summary=planned_summary,
            baseline_samples=baseline_samples,
            baseline_summary=baseline_summary,
        )

    def _build_planned_segments(self, stall_limit_deg: float) -> tuple[SegmentSpec, ...]:
        cfg = self.config
        terminal_start_x = cfg.range_m - cfg.terminal_range_m
        return (
            SegmentSpec(
                name="up_turn",
                start_x_m=0.0,
                end_x_m=cfg.segment1_end_x_m,
                alpha_deg=+stall_limit_deg,
            ),
            SegmentSpec(
                name="down_turn_accel",
                start_x_m=cfg.segment1_end_x_m,
                end_x_m=terminal_start_x,
                alpha_deg=-stall_limit_deg,
            ),
            SegmentSpec(
                name="down_turn_terminal",
                start_x_m=terminal_start_x,
                end_x_m=cfg.range_m,
                alpha_deg=-0.40 * stall_limit_deg,
            ),
        )

    def _build_baseline_segments(self) -> tuple[SegmentSpec, ...]:
        return (
            SegmentSpec(
                name="baseline",
                start_x_m=0.0,
                end_x_m=self.config.range_m,
                alpha_deg=0.0,
            ),
        )

    def _solve_initial_speed(self, segments: tuple[SegmentSpec, ...]) -> float:
        low_v = 1.0
        high_v = 200.0
        low_residual = self._range_residual(low_v, segments)
        high_residual = self._range_residual(high_v, segments)

        while high_residual <= 0.0 and high_v < 1000.0:
            high_v *= 1.5
            high_residual = self._range_residual(high_v, segments)

        if low_residual > 0.0:
            return low_v

        if high_residual <= 0.0:
            raise ValueError("could not bracket an initial speed for this drag case")

        for _ in range(48):
            mid_v = 0.5 * (low_v + high_v)
            mid_residual = self._range_residual(mid_v, segments)
            if mid_residual <= 0.0:
                low_v = mid_v
            else:
                high_v = mid_v

        return 0.5 * (low_v + high_v)

    def _range_residual(self, v0: float, segments: tuple[SegmentSpec, ...]) -> float:
        state = self._initial_state(v0)
        previous = state
        time_s = 0.0

        while time_s < self.config.max_time_s:
            next_state = self._rk4_step(previous, segments)
            if next_state[0] >= self.config.range_m:
                hit_state = self._interpolate_state(previous, next_state, self.config.range_m)
                return hit_state[1] - self.config.impact_height_m
            if next_state[1] <= self.config.impact_height_m:
                return next_state[1] - self.config.impact_height_m
            previous = next_state
            time_s += self.config.dt_s

        return previous[1] - self.config.impact_height_m

    def _sample_trajectory(self, v0: float, segments: tuple[SegmentSpec, ...]) -> list[TrajectorySample]:
        cfg = self.config
        previous = self._initial_state(v0)
        current = previous
        time_s = 0.0
        target_index = 0
        samples: list[TrajectorySample] = []

        while (target_index < cfg.samples) and (time_s < cfg.max_time_s):
            target_x = cfg.range_m * float(target_index) / float(cfg.samples - 1)
            while (target_index < cfg.samples) and (previous[0] <= target_x <= current[0]):
                samples.append(self._interpolate_sample(previous, current, target_x, segments))
                target_index += 1
                if target_index >= cfg.samples:
                    break
                target_x = cfg.range_m * float(target_index) / float(cfg.samples - 1)

            if target_index >= cfg.samples:
                break

            previous = current
            current = self._rk4_step(previous, segments)
            time_s += cfg.dt_s

        if len(samples) < cfg.samples:
            raise ValueError("trajectory did not reach the target range")

        return samples

    def _initial_state(self, v0: float) -> tuple[float, float, float, float, float]:
        vx = v0 * math.cos(self.launch_angle_rad)
        vz = v0 * math.sin(self.launch_angle_rad)
        return (0.0, 0.0, vx, vz, 0.0)

    def _rk4_step(
        self,
        state: tuple[float, float, float, float, float],
        segments: tuple[SegmentSpec, ...],
    ) -> tuple[float, float, float, float, float]:
        cfg = self.config
        k1 = self._derivative(state, segments)
        s2 = (
            state[0] + 0.5 * cfg.dt_s * k1[0],
            state[1] + 0.5 * cfg.dt_s * k1[1],
            state[2] + 0.5 * cfg.dt_s * k1[2],
            state[3] + 0.5 * cfg.dt_s * k1[3],
            state[4] + 0.5 * cfg.dt_s * k1[4],
        )
        k2 = self._derivative(s2, segments)
        s3 = (
            state[0] + 0.5 * cfg.dt_s * k2[0],
            state[1] + 0.5 * cfg.dt_s * k2[1],
            state[2] + 0.5 * cfg.dt_s * k2[2],
            state[3] + 0.5 * cfg.dt_s * k2[3],
            state[4] + 0.5 * cfg.dt_s * k2[4],
        )
        k3 = self._derivative(s3, segments)
        s4 = (
            state[0] + cfg.dt_s * k3[0],
            state[1] + cfg.dt_s * k3[1],
            state[2] + cfg.dt_s * k3[2],
            state[3] + cfg.dt_s * k3[3],
            state[4] + cfg.dt_s * k3[4],
        )
        k4 = self._derivative(s4, segments)

        return (
            state[0] + cfg.dt_s * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
            state[1] + cfg.dt_s * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
            state[2] + cfg.dt_s * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0,
            state[3] + cfg.dt_s * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]) / 6.0,
            state[4] + cfg.dt_s * (k1[4] + 2.0 * k2[4] + 2.0 * k3[4] + k4[4]) / 6.0,
        )

    def _derivative(
        self,
        state: tuple[float, float, float, float, float],
        segments: tuple[SegmentSpec, ...],
    ) -> tuple[float, float, float, float, float]:
        x_m, z_m, vx, vz, t_s = state
        alpha_deg, cd = self._segment_alpha_and_cd(x_m, segments)
        speed = math.hypot(vx, vz)
        drag_factor = self.config.drag_scale * cd * speed
        normal_accel = self.config.normal_accel_scale * math.radians(alpha_deg) * speed * speed

        if speed > 1.0e-6:
            normal_x = -vz / speed
            normal_z = vx / speed
        else:
            normal_x = 0.0
            normal_z = 0.0

        ax = -drag_factor * vx + normal_accel * normal_x
        az = -self.config.gravity_mps2 - drag_factor * vz + normal_accel * normal_z
        return (vx, vz, ax, az, 1.0)

    def _segment_alpha_and_cd(self, x_m: float, segments: tuple[SegmentSpec, ...]) -> tuple[float, float]:
        segment = self._segment_for_x(x_m, segments)
        alpha_rad = math.radians(abs(segment.alpha_deg))
        cd = (
            self.config.cd0
            + self.config.cd_aoa_abs * alpha_rad
            + self.config.cd_aoa_sq * alpha_rad * alpha_rad
        )
        return segment.alpha_deg, cd

    def _segment_for_x(self, x_m: float, segments: tuple[SegmentSpec, ...]) -> SegmentSpec:
        for index, segment in enumerate(segments):
            if (x_m >= segment.start_x_m) and (x_m <= segment.end_x_m):
                return segment
            if index == len(segments) - 1:
                return segment
        return segments[-1]

    def _interpolate_state(
        self,
        previous: tuple[float, float, float, float, float],
        current: tuple[float, float, float, float, float],
        target_x: float,
    ) -> tuple[float, float, float, float, float]:
        span = current[0] - previous[0]
        ratio = 0.0 if span == 0.0 else (target_x - previous[0]) / span
        return (
            target_x,
            previous[1] + ratio * (current[1] - previous[1]),
            previous[2] + ratio * (current[2] - previous[2]),
            previous[3] + ratio * (current[3] - previous[3]),
            previous[4] + ratio * (current[4] - previous[4]),
        )

    def _interpolate_sample(
        self,
        previous: tuple[float, float, float, float, float],
        current: tuple[float, float, float, float, float],
        target_x: float,
        segments: tuple[SegmentSpec, ...],
    ) -> TrajectorySample:
        x_m, z_m, vx, vz, t_s = self._interpolate_state(previous, current, target_x)
        segment = self._segment_for_x(target_x, segments)
        alpha_deg = segment.alpha_deg
        cd = self._segment_alpha_and_cd(target_x, segments)[1]
        gamma_deg = math.degrees(math.atan2(vz, vx))
        segment_index = self._segment_index_for_x(target_x, segments)
        return TrajectorySample(
            x_m=x_m,
            z_m=z_m,
            gamma_deg=gamma_deg,
            t_s=t_s,
            alpha_deg=alpha_deg,
            cd=cd,
            segment_index=segment_index,
        )

    def _segment_index_for_x(self, x_m: float, segments: tuple[SegmentSpec, ...]) -> int:
        for index, segment in enumerate(segments):
            if (x_m >= segment.start_x_m) and (x_m <= segment.end_x_m):
                return index
        return len(segments) - 1

    def _build_summary(self, samples: list[TrajectorySample], initial_speed_mps: float) -> TrajectorySummary:
        cfg = self.config
        impact = samples[-1]
        terminal_start_x = cfg.range_m - cfg.terminal_range_m
        terminal_start = min(samples, key=lambda sample: abs(sample.x_m - terminal_start_x))
        terminal_low = cfg.terminal_range_m * math.tan(math.radians(40.0)) + cfg.impact_height_m
        terminal_high = cfg.terminal_range_m * math.tan(math.radians(60.0)) + cfg.impact_height_m
        return TrajectorySummary(
            initial_speed_mps=initial_speed_mps,
            impact_time_s=impact.t_s,
            impact_gamma_deg=impact.gamma_deg,
            terminal_start_x_m=terminal_start_x,
            terminal_start_z_m=terminal_start.z_m,
            terminal_start_gamma_deg=terminal_start.gamma_deg,
            terminal_start_in_corridor=(terminal_start.z_m >= terminal_low) and (terminal_start.z_m <= terminal_high),
        )


class TrajectoryCsvWriter:
    def write(self, path: Path, samples: list[TrajectorySample]) -> None:
        with path.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.writer(output_file)
            writer.writerow(
                [
                    "x_m",
                    "z_m",
                    "gamma_deg",
                    "time_s",
                    "alpha_deg",
                    "cd",
                    "segment_index",
                ]
            )
            for sample in samples:
                writer.writerow(
                    [
                        f"{sample.x_m:.4f}",
                        f"{sample.z_m:.4f}",
                        f"{sample.gamma_deg:.4f}",
                        f"{sample.t_s:.4f}",
                        f"{sample.alpha_deg:.4f}",
                        f"{sample.cd:.4f}",
                        sample.segment_index,
                    ]
                )


class TrajectorySvgPlot:
    def __init__(
        self,
        config: ModelConfig,
        stall_limit_deg: float,
        planned_segments: tuple[SegmentSpec, ...],
        planned_samples: list[TrajectorySample],
        planned_summary: TrajectorySummary,
        baseline_samples: list[TrajectorySample],
        baseline_summary: TrajectorySummary,
    ) -> None:
        self.config = config
        self.stall_limit_deg = stall_limit_deg
        self.planned_segments = planned_segments
        self.planned_samples = planned_samples
        self.planned_summary = planned_summary
        self.baseline_samples = baseline_samples
        self.baseline_summary = baseline_summary
        self.width = 1120
        self.height = 760
        self.left = 76
        self.right = 30
        self.top = 42
        self.bottom = 110
        self.plot_width = self.width - self.left - self.right
        self.plot_height = self.height - self.top - self.bottom
        self.z_min = min(
            -1.0,
            min(sample.z_m for sample in planned_samples) - 0.5,
            min(sample.z_m for sample in baseline_samples) - 0.5,
        )
        corridor_high = self.config.terminal_range_m * math.tan(math.radians(60.0))
        self.z_max = max(
            max(sample.z_m for sample in planned_samples) + 1.0,
            max(sample.z_m for sample in baseline_samples) + 1.0,
            corridor_high + 1.0,
        )

    def write(self, path: Path) -> None:
        lines = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" viewBox="0 0 {self.width} {self.height}">',
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            '<style>text{font-family:Arial,Helvetica,sans-serif;fill:#111827}.small{font-size:15px}.tiny{font-size:12px}.title{font-size:22px;font-weight:700}.axis{stroke:#111827;stroke-width:1.4}.grid{stroke:#e5e7eb;stroke-width:1}.label{font-size:14px}</style>',
            f'<text class="title" x="76" y="28">Drag-aware controlled three-segment trajectory, stall limit = {self.stall_limit_deg:.0f} deg</text>',
        ]
        lines.extend(self._grid())
        lines.extend(self._terminal_corridor())
        lines.extend(self._baseline_path())
        lines.extend(self._planned_path())
        lines.extend(self._legend())
        lines.extend(self._summary())
        lines.append("</svg>")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _grid(self) -> list[str]:
        lines: list[str] = []
        for x_m in range(0, int(math.ceil(self.config.range_m)) + 1, 5):
            sx, _ = self._screen(float(x_m), 0.0)
            lines.append(f'<line class="grid" x1="{sx:.1f}" y1="{self.top}" x2="{sx:.1f}" y2="{self.top + self.plot_height}"/>')
            lines.append(f'<text class="tiny" x="{sx - 10:.1f}" y="{self.top + self.plot_height + 24}">{x_m}</text>')
        z_step = 2.0
        z_tick = math.floor(self.z_min / z_step) * z_step
        while z_tick <= self.z_max:
            _, sy = self._screen(0.0, z_tick)
            lines.append(f'<line class="grid" x1="{self.left}" y1="{sy:.1f}" x2="{self.left + self.plot_width}" y2="{sy:.1f}"/>')
            lines.append(f'<text class="tiny" x="{self.left - 44}" y="{sy + 4:.1f}">{z_tick:.0f}</text>')
            z_tick += z_step
        x0, y0 = self._screen(0.0, 0.0)
        x1, _ = self._screen(self.config.range_m, 0.0)
        lines.append(f'<line class="axis" x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y0:.1f}"/>')
        lines.append(f'<line class="axis" x1="{x0:.1f}" y1="{self.top}" x2="{x0:.1f}" y2="{self.top + self.plot_height}"/>')
        lines.append(f'<text class="small" x="{self.left + self.plot_width / 2 - 52:.1f}" y="{self.height - 28}">horizontal distance x (m)</text>')
        lines.append(f'<text class="small" transform="translate(22 {self.top + self.plot_height / 2 + 56:.1f}) rotate(-90)">height z relative to launch (m)</text>')
        return lines

    def _terminal_corridor(self) -> list[str]:
        cfg = self.config
        x_start = cfg.range_m - cfg.terminal_range_m
        x_end = cfg.range_m
        z40 = cfg.terminal_range_m * math.tan(math.radians(40.0))
        z50 = cfg.terminal_range_m * math.tan(math.radians(50.0))
        z60 = cfg.terminal_range_m * math.tan(math.radians(60.0))
        p0 = self._screen(x_start, z60)
        p1 = self._screen(x_end, cfg.impact_height_m)
        p2 = self._screen(x_start, z40)
        p_mid = self._screen(x_start, z50)
        return [
            f'<polygon points="{p0[0]:.1f},{p0[1]:.1f} {p1[0]:.1f},{p1[1]:.1f} {p2[0]:.1f},{p2[1]:.1f}" fill="#fee2e2" stroke="#ef4444" stroke-width="1.2" opacity="0.72"/>',
            f'<line x1="{p_mid[0]:.1f}" y1="{p_mid[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" stroke="#dc2626" stroke-width="1.4" stroke-dasharray="7 5"/>',
            f'<text class="label" x="{p0[0] - 4:.1f}" y="{p0[1] - 10:.1f}">terminal -60 deg</text>',
            f'<text class="label" x="{p2[0] - 4:.1f}" y="{p2[1] + 20:.1f}">terminal -40 deg</text>',
            f'<text class="label" x="{p_mid[0] + 8:.1f}" y="{p_mid[1] - 6:.1f}">-50 deg reference</text>',
        ]

    def _baseline_path(self) -> list[str]:
        points = self._polyline_points(self.baseline_samples)
        start = self._screen(0.0, 0.0)
        impact = self._screen(self.config.range_m, self.config.impact_height_m)
        return [
            f'<polyline points="{points}" fill="none" stroke="#6b7280" stroke-width="2.0" stroke-dasharray="7 5" opacity="0.8"/>',
            f'<circle cx="{start[0]:.1f}" cy="{start[1]:.1f}" r="4.5" fill="#6b7280"/>',
            f'<circle cx="{impact[0]:.1f}" cy="{impact[1]:.1f}" r="5" fill="#6b7280"/>',
            f'<text class="label" x="{self.left + 16}" y="{self.top + 136}">baseline: alpha = 0 deg</text>',
        ]

    def _planned_path(self) -> list[str]:
        colors = ["#2563eb", "#f97316", "#16a34a"]
        lines: list[str] = []
        for index, segment in enumerate(self.planned_segments):
            start_index = self._index_for_x(segment.start_x_m, self.planned_samples)
            end_index = self._index_for_x(segment.end_x_m, self.planned_samples)
            points = self._polyline_points(self.planned_samples[start_index : end_index + 1])
            lines.append(
                f'<polyline points="{points}" fill="none" stroke="{colors[index]}" stroke-width="3.2"/>'
            )
            boundary = self._screen(segment.end_x_m, self._sample_at_x(segment.end_x_m, self.planned_samples).z_m)
            lines.append(
                f'<circle cx="{boundary[0]:.1f}" cy="{boundary[1]:.1f}" r="4.2" fill="{colors[index]}"/>'
            )

        terminal_start = self._screen(
            self.planned_summary.terminal_start_x_m,
            self.planned_summary.terminal_start_z_m,
        )
        impact = self._screen(self.config.range_m, self.config.impact_height_m)
        lines.extend(
            [
                f'<circle cx="{terminal_start[0]:.1f}" cy="{terminal_start[1]:.1f}" r="5.2" fill="#7c3aed"/>',
                f'<line x1="{terminal_start[0]:.1f}" y1="{self.top}" x2="{terminal_start[0]:.1f}" y2="{self.top + self.plot_height}" stroke="#7c3aed" stroke-width="1.1" stroke-dasharray="4 4"/>',
                f'<circle cx="{impact[0]:.1f}" cy="{impact[1]:.1f}" r="6" fill="#059669"/>',
                f'<text class="label" x="{terminal_start[0] - 52:.1f}" y="{self.top + 18}">x=17m</text>',
                f'<text class="label" x="{impact[0] - 58:.1f}" y="{impact[1] - 12:.1f}">target x=25m</text>',
            ]
        )
        return lines

    def _legend(self) -> list[str]:
        x = self.left + 16
        y = self.top + 34
        alpha_1 = self.planned_segments[0].alpha_deg
        alpha_2 = self.planned_segments[1].alpha_deg
        alpha_3 = self.planned_segments[2].alpha_deg
        return [
            f'<rect x="{x - 12}" y="{y - 20}" width="412" height="118" rx="6" fill="#ffffff" stroke="#d1d5db"/>',
            f'<line x1="{x}" y1="{y}" x2="{x + 42}" y2="{y}" stroke="#6b7280" stroke-width="2" stroke-dasharray="7 5"/>',
            f'<text class="small" x="{x + 52}" y="{y + 5}">baseline no-planning drag path</text>',
            f'<line x1="{x}" y1="{y + 26}" x2="{x + 42}" y2="{y + 26}" stroke="#2563eb" stroke-width="3"/>',
            f'<text class="small" x="{x + 52}" y="{y + 31}">segment 1 up_turn, alpha = {alpha_1:.1f} deg</text>',
            f'<line x1="{x}" y1="{y + 52}" x2="{x + 42}" y2="{y + 52}" stroke="#f97316" stroke-width="3"/>',
            f'<text class="small" x="{x + 52}" y="{y + 57}">segment 2 down_turn, alpha = {alpha_2:.1f} deg</text>',
            f'<line x1="{x}" y1="{y + 78}" x2="{x + 42}" y2="{y + 78}" stroke="#16a34a" stroke-width="3"/>',
            f'<text class="small" x="{x + 52}" y="{y + 83}">segment 3 terminal, alpha = {alpha_3:.1f} deg</text>',
            f'<rect x="{x}" y="{y + 92}" width="42" height="14" fill="#fee2e2" stroke="#ef4444" opacity="0.72"/>',
            f'<text class="small" x="{x + 52}" y="{y + 105}">target corridor: -40 to -60 deg over last 8 m</text>',
        ]

    def _summary(self) -> list[str]:
        y = self.height - 76
        corridor_text = "yes" if self.planned_summary.terminal_start_in_corridor else "no"
        note = (
            "No sustained lift term: alpha sign drives effective normal trajectory-control acceleration; |alpha| increases drag."
        )
        return [
            f'<text class="small" x="76" y="{y}">{note}</text>',
            f'<text class="small" x="76" y="{y + 22}">planned v0={self.planned_summary.initial_speed_mps:.2f} m/s, '
            f'time={self.planned_summary.impact_time_s:.2f}s, z@17m={self.planned_summary.terminal_start_z_m:.2f}m, gamma@17m={self.planned_summary.terminal_start_gamma_deg:.1f} deg, ',
            f'gamma@25m={self.planned_summary.impact_gamma_deg:.1f} deg, corridor={corridor_text}.</text>',
            f'<text class="small" x="76" y="{y + 44}">baseline v0={self.baseline_summary.initial_speed_mps:.2f} m/s, '
            f'time={self.baseline_summary.impact_time_s:.2f}s, gamma@25m={self.baseline_summary.impact_gamma_deg:.1f} deg.</text>',
        ]

    def _polyline_points(self, samples: list[TrajectorySample]) -> str:
        return " ".join(f"{self._screen(sample.x_m, sample.z_m)[0]:.1f},{self._screen(sample.x_m, sample.z_m)[1]:.1f}" for sample in samples)

    def _sample_at_x(self, target_x: float, samples: list[TrajectorySample]) -> TrajectorySample:
        if target_x <= samples[0].x_m:
            return samples[0]
        if target_x >= samples[-1].x_m:
            return samples[-1]

        for index in range(len(samples) - 1):
            left = samples[index]
            right = samples[index + 1]
            if (left.x_m <= target_x <= right.x_m) or (right.x_m <= target_x <= left.x_m):
                span = right.x_m - left.x_m
                ratio = 0.0 if span == 0.0 else (target_x - left.x_m) / span
                alpha_deg = left.alpha_deg + ratio * (right.alpha_deg - left.alpha_deg)
                cd = left.cd + ratio * (right.cd - left.cd)
                gamma_deg = left.gamma_deg + ratio * (right.gamma_deg - left.gamma_deg)
                t_s = left.t_s + ratio * (right.t_s - left.t_s)
                z_m = left.z_m + ratio * (right.z_m - left.z_m)
                segment_index = left.segment_index if ratio < 0.5 else right.segment_index
                return TrajectorySample(
                    x_m=target_x,
                    z_m=z_m,
                    gamma_deg=gamma_deg,
                    t_s=t_s,
                    alpha_deg=alpha_deg,
                    cd=cd,
                    segment_index=segment_index,
                )

        return samples[-1]

    def _index_for_x(self, x_m: float, samples: list[TrajectorySample]) -> int:
        if x_m <= samples[0].x_m:
            return 0
        if x_m >= samples[-1].x_m:
            return len(samples) - 1
        closest_index = 0
        closest_error = abs(samples[0].x_m - x_m)
        for index, sample in enumerate(samples[1:], start=1):
            error = abs(sample.x_m - x_m)
            if error < closest_error:
                closest_error = error
                closest_index = index
        return closest_index

    def _screen(self, x_m: float, z_m: float) -> tuple[float, float]:
        sx = self.left + (x_m / self.config.range_m) * self.plot_width
        sy = self.top + (self.z_max - z_m) / (self.z_max - self.z_min) * self.plot_height
        return sx, sy


class TrajectoryThreeSegmentCli:
    def run(self) -> int:
        args = self._parse_args()
        config = ModelConfig(
            range_m=args.range_m,
            terminal_range_m=args.terminal_range_m,
            launch_angle_deg=args.launch_angle_deg,
            impact_height_m=args.impact_height_m,
            gravity_mps2=args.gravity_mps2,
            drag_scale=args.drag_scale,
            cd0=args.cd0,
            cd_aoa_abs=args.cd_aoa_abs,
            cd_aoa_sq=args.cd_aoa_sq,
            normal_accel_scale=args.normal_accel_scale,
            segment1_end_x_m=args.segment1_end_x_m,
            samples=args.samples,
            dt_s=args.dt_s,
            max_time_s=args.max_time_s,
            output_prefix=Path(args.output_prefix),
        )
        model = DragAwareTrajectoryModel(config)
        csv_writer = TrajectoryCsvWriter()

        for stall_limit_deg in args.stall_limits:
            case = model.solve_case(stall_limit_deg)
            suffix = f"{stall_limit_deg:g}deg"
            output_prefix = config.output_prefix.with_name(f"{config.output_prefix.name}_{suffix}")
            svg_path = output_prefix.with_suffix(".svg")
            csv_path = output_prefix.with_suffix(".csv")
            svg_path.parent.mkdir(parents=True, exist_ok=True)

            TrajectorySvgPlot(
                config=config,
                stall_limit_deg=stall_limit_deg,
                planned_segments=case.planned_segments,
                planned_samples=case.planned_samples,
                planned_summary=case.planned_summary,
                baseline_samples=case.baseline_samples,
                baseline_summary=case.baseline_summary,
            ).write(svg_path)
            csv_writer.write(csv_path, case.planned_samples)

            print(f"wrote {svg_path}")
            print(f"wrote {csv_path}")
            print(
                f"stall_limit={stall_limit_deg:g} deg, "
                f"planned_v0={case.planned_summary.initial_speed_mps:.3f} m/s, "
                f"planned_gamma@17m={case.planned_summary.terminal_start_gamma_deg:.3f} deg, "
                f"planned_gamma@25m={case.planned_summary.impact_gamma_deg:.3f} deg, "
                f"planned_corridor={'yes' if case.planned_summary.terminal_start_in_corridor else 'no'}, "
                f"baseline_gamma@25m={case.baseline_summary.impact_gamma_deg:.3f} deg"
            )

        return 0

    def _parse_args(self) -> argparse.Namespace:
        parser = argparse.ArgumentParser(
            description="Plot a drag-aware, three-segment dart trajectory with AoA-coupled drag and effective normal trajectory control."
        )
        parser.add_argument("--stall-limits", type=float, nargs="+", default=[10.0, 15.0])
        parser.add_argument("--range-m", type=float, default=25.0)
        parser.add_argument("--terminal-range-m", type=float, default=8.0)
        parser.add_argument("--segment1-end-x-m", type=float, default=6.0)
        parser.add_argument("--launch-angle-deg", type=float, default=32.0)
        parser.add_argument("--impact-height-m", type=float, default=0.0)
        parser.add_argument("--gravity-mps2", type=float, default=9.80665)
        parser.add_argument("--drag-scale", type=float, default=0.14)
        parser.add_argument("--cd0", type=float, default=0.20)
        parser.add_argument("--cd-aoa-abs", type=float, default=0.80)
        parser.add_argument("--cd-aoa-sq", type=float, default=1.00)
        parser.add_argument("--normal-accel-scale", type=float, default=0.10)
        parser.add_argument("--samples", type=int, default=301)
        parser.add_argument("--dt-s", type=float, default=0.0005)
        parser.add_argument("--max-time-s", type=float, default=8.0)
        parser.add_argument(
            "--output-prefix",
            default="trajectory_three_segment_drag",
            help="Output path without extension. One SVG and one CSV are written per stall limit.",
        )
        return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(TrajectoryThreeSegmentCli().run())
