#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TrajectorySample:
    x_m: float
    z_m: float
    gamma_deg: float
    t_s: float


@dataclass(frozen=True)
class TrajectoryMetrics:
    initial_speed_mps: float
    impact_time_s: float
    terminal_start_x_m: float
    terminal_start_z_m: float
    terminal_start_gamma_deg: float
    impact_gamma_deg: float
    required_z_40_deg_m: float
    required_z_50_deg_m: float
    required_z_60_deg_m: float


@dataclass(frozen=True)
class PlotConfig:
    range_m: float
    terminal_range_m: float
    launch_angle_deg: float
    impact_height_m: float
    gravity_mps2: float
    samples: int
    drag_k: float
    dt_s: float
    output_prefix: Path


class NoLiftTrajectoryModel:
    def __init__(self, config: PlotConfig) -> None:
        self.config = config
        self.launch_angle_rad = math.radians(config.launch_angle_deg)

    def solve(self) -> tuple[list[TrajectorySample], TrajectoryMetrics]:
        if self.config.drag_k == 0.0:
            return self._solve_closed_form()
        return self._solve_with_drag()

    def _solve_closed_form(self) -> tuple[list[TrajectorySample], TrajectoryMetrics]:
        cfg = self.config
        cos_theta = math.cos(self.launch_angle_rad)
        tan_theta = math.tan(self.launch_angle_rad)
        denominator = 2.0 * cos_theta * cos_theta * (cfg.range_m * tan_theta - cfg.impact_height_m)
        if denominator <= 0.0:
            raise ValueError("impact height is too high for this no-lift launch geometry")

        v0 = math.sqrt(cfg.gravity_mps2 * cfg.range_m * cfg.range_m / denominator)
        vx0 = v0 * cos_theta
        vz0 = v0 * math.sin(self.launch_angle_rad)
        samples: list[TrajectorySample] = []

        for index in range(cfg.samples):
            x_m = cfg.range_m * float(index) / float(cfg.samples - 1)
            t_s = x_m / vx0
            z_m = x_m * tan_theta - 0.5 * cfg.gravity_mps2 * t_s * t_s
            gamma_deg = math.degrees(math.atan2(vz0 - cfg.gravity_mps2 * t_s, vx0))
            samples.append(TrajectorySample(x_m=x_m, z_m=z_m, gamma_deg=gamma_deg, t_s=t_s))

        metrics = self._metrics_from_samples(samples, v0)
        return samples, metrics

    def _solve_with_drag(self) -> tuple[list[TrajectorySample], TrajectoryMetrics]:
        low_v = 1.0
        high_v = 200.0
        low_z = self._impact_height_for_speed(low_v)
        high_z = self._impact_height_for_speed(high_v)

        while high_z < self.config.impact_height_m and high_v < 1000.0:
            high_v *= 1.5
            high_z = self._impact_height_for_speed(high_v)

        if low_z > self.config.impact_height_m or high_z < self.config.impact_height_m:
            raise ValueError("could not bracket an initial speed for this drag setting")

        for _ in range(48):
            mid_v = 0.5 * (low_v + high_v)
            mid_z = self._impact_height_for_speed(mid_v)
            if mid_z < self.config.impact_height_m:
                low_v = mid_v
            else:
                high_v = mid_v

        v0 = 0.5 * (low_v + high_v)
        samples = self._sample_drag_trajectory(v0)
        metrics = self._metrics_from_samples(samples, v0)
        return samples, metrics

    def _impact_height_for_speed(self, v0: float) -> float:
        samples = self._sample_drag_trajectory(v0)
        return samples[-1].z_m

    def _sample_drag_trajectory(self, v0: float) -> list[TrajectorySample]:
        cfg = self.config
        vx = v0 * math.cos(self.launch_angle_rad)
        vz = v0 * math.sin(self.launch_angle_rad)
        x_m = 0.0
        z_m = 0.0
        t_s = 0.0
        grid_index = 0
        samples: list[TrajectorySample] = []
        prev_state = (x_m, z_m, vx, vz, t_s)

        while x_m <= cfg.range_m and t_s < 10.0:
            target_x = cfg.range_m * float(grid_index) / float(cfg.samples - 1)
            while grid_index < cfg.samples and prev_state[0] <= target_x <= x_m:
                sample = self._interpolate_state(prev_state, (x_m, z_m, vx, vz, t_s), target_x)
                samples.append(sample)
                grid_index += 1
                if grid_index >= cfg.samples:
                    break
                target_x = cfg.range_m * float(grid_index) / float(cfg.samples - 1)

            if grid_index >= cfg.samples:
                break

            prev_state = (x_m, z_m, vx, vz, t_s)
            x_m, z_m, vx, vz = self._rk4_step(x_m, z_m, vx, vz, cfg.dt_s)
            t_s += cfg.dt_s

        if len(samples) < cfg.samples:
            raise ValueError("drag trajectory did not reach target range")
        return samples

    def _rk4_step(self, x_m: float, z_m: float, vx: float, vz: float, dt_s: float) -> tuple[float, float, float, float]:
        k1 = self._derivative(vx, vz)
        k2 = self._derivative(vx + 0.5 * dt_s * k1[2], vz + 0.5 * dt_s * k1[3])
        k3 = self._derivative(vx + 0.5 * dt_s * k2[2], vz + 0.5 * dt_s * k2[3])
        k4 = self._derivative(vx + dt_s * k3[2], vz + dt_s * k3[3])

        next_x = x_m + dt_s * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        next_z = z_m + dt_s * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        next_vx = vx + dt_s * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0
        next_vz = vz + dt_s * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]) / 6.0
        return next_x, next_z, next_vx, next_vz

    def _derivative(self, vx: float, vz: float) -> tuple[float, float, float, float]:
        speed = math.hypot(vx, vz)
        ax = -self.config.drag_k * speed * vx
        az = -self.config.gravity_mps2 - self.config.drag_k * speed * vz
        return vx, vz, ax, az

    def _interpolate_state(
        self,
        a: tuple[float, float, float, float, float],
        b: tuple[float, float, float, float, float],
        target_x: float,
    ) -> TrajectorySample:
        span = b[0] - a[0]
        ratio = 0.0 if span == 0.0 else (target_x - a[0]) / span
        z_m = a[1] + ratio * (b[1] - a[1])
        vx = a[2] + ratio * (b[2] - a[2])
        vz = a[3] + ratio * (b[3] - a[3])
        t_s = a[4] + ratio * (b[4] - a[4])
        gamma_deg = math.degrees(math.atan2(vz, vx))
        return TrajectorySample(x_m=target_x, z_m=z_m, gamma_deg=gamma_deg, t_s=t_s)

    def _metrics_from_samples(self, samples: list[TrajectorySample], v0: float) -> TrajectoryMetrics:
        terminal_start_x = self.config.range_m - self.config.terminal_range_m
        terminal_start = min(samples, key=lambda sample: abs(sample.x_m - terminal_start_x))
        impact = samples[-1]
        required_z_40 = self.config.terminal_range_m * math.tan(math.radians(40.0)) + self.config.impact_height_m
        required_z_50 = self.config.terminal_range_m * math.tan(math.radians(50.0)) + self.config.impact_height_m
        required_z_60 = self.config.terminal_range_m * math.tan(math.radians(60.0)) + self.config.impact_height_m
        return TrajectoryMetrics(
            initial_speed_mps=v0,
            impact_time_s=impact.t_s,
            terminal_start_x_m=terminal_start_x,
            terminal_start_z_m=terminal_start.z_m,
            terminal_start_gamma_deg=terminal_start.gamma_deg,
            impact_gamma_deg=impact.gamma_deg,
            required_z_40_deg_m=required_z_40,
            required_z_50_deg_m=required_z_50,
            required_z_60_deg_m=required_z_60,
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
                    "theta_min_alpha10_deg",
                    "theta_max_alpha10_deg",
                    "theta_min_alpha15_deg",
                    "theta_max_alpha15_deg",
                ]
            )
            for sample in samples:
                writer.writerow(
                    [
                        f"{sample.x_m:.4f}",
                        f"{sample.z_m:.4f}",
                        f"{sample.gamma_deg:.4f}",
                        f"{sample.t_s:.4f}",
                        f"{sample.gamma_deg - 10.0:.4f}",
                        f"{sample.gamma_deg + 10.0:.4f}",
                        f"{sample.gamma_deg - 15.0:.4f}",
                        f"{sample.gamma_deg + 15.0:.4f}",
                    ]
                )


class SvgTrajectoryPlot:
    def __init__(self, config: PlotConfig, samples: list[TrajectorySample], metrics: TrajectoryMetrics) -> None:
        self.config = config
        self.samples = samples
        self.metrics = metrics
        self.width = 1100
        self.height = 720
        self.left = 74
        self.right = 34
        self.top = 36
        self.bottom = 94
        self.plot_width = self.width - self.left - self.right
        self.plot_height = self.height - self.top - self.bottom
        self.z_min = min(-1.0, min(sample.z_m for sample in samples) - 0.5)
        terminal_z_max = max(metrics.required_z_40_deg_m, metrics.required_z_60_deg_m)
        self.z_max = max(terminal_z_max + 1.0, max(sample.z_m for sample in samples) + 1.0)

    def write(self, path: Path) -> None:
        lines = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" viewBox="0 0 {self.width} {self.height}">',
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            '<style>text{font-family:Arial,Helvetica,sans-serif;fill:#111827}.small{font-size:15px}.tiny{font-size:12px}.title{font-size:22px;font-weight:700}.axis{stroke:#111827;stroke-width:1.4}.grid{stroke:#e5e7eb;stroke-width:1}.label{font-size:14px}</style>',
            '<text class="title" x="74" y="26">No-lift vertical trajectory baseline: stall limits 10 deg vs 15 deg</text>',
        ]
        lines.extend(self._grid())
        lines.extend(self._terminal_corridor())
        lines.extend(self._trajectory_paths())
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
        _, y_axis_top = self._screen(0.0, self.z_max)
        lines.append(f'<line class="axis" x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y0:.1f}"/>')
        lines.append(f'<line class="axis" x1="{x0:.1f}" y1="{self.top}" x2="{x0:.1f}" y2="{self.top + self.plot_height}"/>')
        lines.append(f'<text class="small" x="{self.left + self.plot_width / 2 - 54:.1f}" y="{self.height - 26}">horizontal distance x (m)</text>')
        lines.append(f'<text class="small" transform="translate(22 {self.top + self.plot_height / 2 + 54:.1f}) rotate(-90)">height z relative to launch (m)</text>')
        lines.append(f'<line x1="{x0:.1f}" y1="{y_axis_top:.1f}" x2="{x0:.1f}" y2="{y_axis_top:.1f}" stroke="none"/>')
        return lines

    def _terminal_corridor(self) -> list[str]:
        x_start = self.metrics.terminal_start_x_m
        x_end = self.config.range_m
        z40 = self.metrics.required_z_40_deg_m
        z50 = self.metrics.required_z_50_deg_m
        z60 = self.metrics.required_z_60_deg_m
        impact_z = self.config.impact_height_m
        p0 = self._screen(x_start, z60)
        p1 = self._screen(x_end, impact_z)
        p2 = self._screen(x_start, z40)
        p_mid_start = self._screen(x_start, z50)
        p_mid_end = self._screen(x_end, impact_z)
        return [
            f'<polygon points="{p0[0]:.1f},{p0[1]:.1f} {p1[0]:.1f},{p1[1]:.1f} {p2[0]:.1f},{p2[1]:.1f}" fill="#fee2e2" stroke="#ef4444" stroke-width="1.2" opacity="0.72"/>',
            f'<line x1="{p_mid_start[0]:.1f}" y1="{p_mid_start[1]:.1f}" x2="{p_mid_end[0]:.1f}" y2="{p_mid_end[1]:.1f}" stroke="#dc2626" stroke-width="1.4" stroke-dasharray="7 5"/>',
            f'<text class="label" x="{p0[0] - 4:.1f}" y="{p0[1] - 10:.1f}">terminal -60 deg</text>',
            f'<text class="label" x="{p2[0] - 4:.1f}" y="{p2[1] + 20:.1f}">terminal -40 deg</text>',
            f'<text class="label" x="{p_mid_start[0] + 8:.1f}" y="{p_mid_start[1] - 6:.1f}">-50 deg reference</text>',
        ]

    def _trajectory_paths(self) -> list[str]:
        points = " ".join(f"{self._screen(sample.x_m, sample.z_m)[0]:.1f},{self._screen(sample.x_m, sample.z_m)[1]:.1f}" for sample in self.samples)
        start = self._screen(0.0, 0.0)
        impact = self._screen(self.config.range_m, self.config.impact_height_m)
        terminal_start = self._screen(self.metrics.terminal_start_x_m, self.metrics.terminal_start_z_m)
        return [
            f'<polyline points="{points}" fill="none" stroke="#2563eb" stroke-width="3.0"/>',
            f'<polyline points="{points}" fill="none" stroke="#f97316" stroke-width="2.0" stroke-dasharray="8 5"/>',
            f'<circle cx="{start[0]:.1f}" cy="{start[1]:.1f}" r="5" fill="#111827"/>',
            f'<circle cx="{impact[0]:.1f}" cy="{impact[1]:.1f}" r="6" fill="#16a34a"/>',
            f'<circle cx="{terminal_start[0]:.1f}" cy="{terminal_start[1]:.1f}" r="5" fill="#7c3aed"/>',
            f'<line x1="{terminal_start[0]:.1f}" y1="{self.top}" x2="{terminal_start[0]:.1f}" y2="{self.top + self.plot_height}" stroke="#7c3aed" stroke-width="1.1" stroke-dasharray="4 4"/>',
            f'<text class="label" x="{terminal_start[0] - 48:.1f}" y="{self.top + 18}">x=17m</text>',
            f'<text class="label" x="{impact[0] - 58:.1f}" y="{impact[1] - 12:.1f}">target x=25m</text>',
        ]

    def _legend(self) -> list[str]:
        x = self.left + 18
        y = self.top + 26
        return [
            f'<rect x="{x - 12}" y="{y - 20}" width="392" height="92" rx="6" fill="#ffffff" stroke="#d1d5db"/>',
            f'<line x1="{x}" y1="{y}" x2="{x + 42}" y2="{y}" stroke="#2563eb" stroke-width="3"/>',
            f'<text class="small" x="{x + 52}" y="{y + 5}">alpha limit +/-10 deg, no-lift path</text>',
            f'<line x1="{x}" y1="{y + 26}" x2="{x + 42}" y2="{y + 26}" stroke="#f97316" stroke-width="2" stroke-dasharray="8 5"/>',
            f'<text class="small" x="{x + 52}" y="{y + 31}">alpha limit +/-15 deg, same CM path</text>',
            f'<rect x="{x}" y="{y + 46}" width="42" height="14" fill="#fee2e2" stroke="#ef4444" opacity="0.72"/>',
            f'<text class="small" x="{x + 52}" y="{y + 59}">last 8m target corridor: -40 to -60 deg</text>',
        ]

    def _summary(self) -> list[str]:
        y = self.height - 76
        alpha_note = "No-lift assumption: alpha limit changes allowed body pitch, not center-of-mass trajectory."
        result_note = "Baseline misses terminal corridor: z@17m is below the -40 deg entry height and impact gamma is shallower than -40 deg."
        return [
            f'<text class="small" x="74" y="{y}">{alpha_note}</text>',
            f'<text class="small" x="74" y="{y + 22}">v0={self.metrics.initial_speed_mps:.2f} m/s, flight time={self.metrics.impact_time_s:.2f}s, gamma@17m={self.metrics.terminal_start_gamma_deg:.1f} deg, gamma@25m={self.metrics.impact_gamma_deg:.1f} deg.</text>',
            f'<text class="small" x="74" y="{y + 44}">z@17m={self.metrics.terminal_start_z_m:.2f}m; required z@17m is {self.metrics.required_z_40_deg_m:.2f}m to {self.metrics.required_z_60_deg_m:.2f}m.</text>',
            f'<text class="small" x="74" y="{y + 66}">{result_note}</text>',
        ]

    def _screen(self, x_m: float, z_m: float) -> tuple[float, float]:
        sx = self.left + (x_m / self.config.range_m) * self.plot_width
        sy = self.top + (self.z_max - z_m) / (self.z_max - self.z_min) * self.plot_height
        return sx, sy


class TrajectoryPlotCli:
    def run(self) -> int:
        args = self._parse_args()
        config = PlotConfig(
            range_m=args.range_m,
            terminal_range_m=args.terminal_range_m,
            launch_angle_deg=args.launch_angle_deg,
            impact_height_m=args.impact_height_m,
            gravity_mps2=args.gravity_mps2,
            samples=args.samples,
            drag_k=args.drag_k,
            dt_s=args.dt_s,
            output_prefix=Path(args.output_prefix),
        )
        samples, metrics = NoLiftTrajectoryModel(config).solve()
        svg_path = config.output_prefix.with_suffix(".svg")
        csv_path = config.output_prefix.with_suffix(".csv")
        svg_path.parent.mkdir(parents=True, exist_ok=True)
        SvgTrajectoryPlot(config, samples, metrics).write(svg_path)
        TrajectoryCsvWriter().write(csv_path, samples)
        print(f"wrote {svg_path}")
        print(f"wrote {csv_path}")
        print(f"initial_speed_mps={metrics.initial_speed_mps:.4f}")
        print(f"impact_time_s={metrics.impact_time_s:.4f}")
        print(f"z_at_terminal_start_m={metrics.terminal_start_z_m:.4f}")
        print(f"required_terminal_start_z_m={metrics.required_z_40_deg_m:.4f}..{metrics.required_z_60_deg_m:.4f}")
        print(f"gamma_at_terminal_start_deg={metrics.terminal_start_gamma_deg:.4f}")
        print(f"gamma_at_impact_deg={metrics.impact_gamma_deg:.4f}")
        return 0

    def _parse_args(self) -> argparse.Namespace:
        parser = argparse.ArgumentParser(
            description="Plot no-lift vertical trajectory baseline and 10/15 degree attack-angle envelopes."
        )
        parser.add_argument("--range-m", type=float, default=25.0)
        parser.add_argument("--terminal-range-m", type=float, default=8.0)
        parser.add_argument("--launch-angle-deg", type=float, default=32.0)
        parser.add_argument("--impact-height-m", type=float, default=0.0)
        parser.add_argument("--gravity-mps2", type=float, default=9.80665)
        parser.add_argument("--samples", type=int, default=251)
        parser.add_argument(
            "--drag-k",
            type=float,
            default=0.0,
            help="Optional quadratic drag acceleration coefficient in 1/m. Default 0 keeps the baseline analytic.",
        )
        parser.add_argument("--dt-s", type=float, default=0.001)
        parser.add_argument(
            "--output-prefix",
            default="trajectory_stall_limit",
            help="Output path without extension. The script writes .svg and .csv.",
        )
        return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(TrajectoryPlotCli().run())
