#!/usr/bin/env python3
"""
Trajectory planner v2 — drag-calibrated three-segment dart guidance.

Physical model
  Launch angle = 32°, unpowered projectile, range = 25 m, flight time ≈ 2 s.
  Target impact angle ≈ -45° (inside the [-40°, -60°] terminal corridor).

Control strategy (three segments, all α clamped to stall limit ±10°)
  Segment 1: up_turn        α = +α_max   pull up to gain altitude
  Segment 2: down_turn_dive  α = -α_max   dive to gain speed
  Segment 3: terminal_pull   α = +α_max   pull out toward target — the
    terminal segment is the *controlled* window (≈ 6–8 m horizontal).

The planner first calibrates the baseline drag from the observed constraints,
then solves the three-segment plan and analyses terminal linearity.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Sequence

# ── physical constants ──────────────────────────────────────────────────────
GRAVITY = 9.80665  # m/s²

# drag sub-model (same structure as existing DragAwareTrajectoryModel)
CD_AOA_ABS = 0.80  # linear AoA drag penalty
CD_AOA_SQ = 1.00  # quadratic AoA drag penalty


def _cd(alpha_deg: float, cd0: float) -> float:
    a_rad = abs(math.radians(alpha_deg))
    return cd0 + CD_AOA_ABS * a_rad + CD_AOA_SQ * a_rad * a_rad


# ── pure-math solver helpers ────────────────────────────────────────────────
def _bisect(
    f: Callable[[float], float],
    lo: float,
    hi: float,
    tol: float = 1e-7,
    max_iter: int = 64,
) -> float:
    """f(lo) < 0,  f(hi) > 0  →  return root."""
    f_lo = f(lo)
    f_hi = f(hi)
    if f_lo > 0.0 or f_hi < 0.0:
        raise ValueError(f"cannot bracket: f({lo})={f_lo:.3g}, f({hi})={f_hi:.3g}")
    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        f_mid = f(mid)
        if abs(f_mid) < tol:
            return mid
        if f_mid < 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _golden_search(
    f: Callable[[float], float],
    lo: float,
    hi: float,
    tol: float = 1e-5,
    max_iter: int = 64,
) -> float:
    """Minimise |f| over [lo, hi].  Returns NaN if f is NaN everywhere."""
    phi = (math.sqrt(5.0) + 1.0) / 2.0
    resphi = 2.0 - phi
    a, b = lo, hi
    c = a + resphi * (b - a)
    d = b - resphi * (b - a)
    fc = abs(f(c))
    fd = abs(f(d))
    if math.isnan(fc) or math.isnan(fd):
        # fallback: coarse linear scan
        best_x = lo
        best_val = float("inf")
        for x in (lo + (hi - lo) * i / 40 for i in range(41)):
            v = abs(f(x))
            if v < best_val:
                best_val = v
                best_x = x
        return best_x
    for _ in range(max_iter):
        if fc < fd:
            b, d = d, c
            fd = fc
            c = a + resphi * (b - a)
            fc = abs(f(c))
        else:
            a, c = c, d
            fc = fd
            d = b - resphi * (b - a)
            fd = abs(f(d))
        if abs(b - a) < tol:
            return 0.5 * (a + b)
    return 0.5 * (a + b)


# ── RK4 integration with segment-dependent α ────────────────────────────────
@dataclass(frozen=True)
class SegmentSpec:
    name: str
    start_x: float
    end_x: float
    alpha_deg: float


@dataclass
class TrajSample:
    x: float
    z: float
    gamma_deg: float
    speed: float
    t: float
    alpha_deg: float


def _alpha_for_x(x: float, segments: tuple[SegmentSpec, ...]) -> float:
    for seg in segments:
        if seg.start_x <= x <= seg.end_x:
            return seg.alpha_deg
    return segments[-1].alpha_deg


def _derivative(
    state: tuple[float, float, float, float, float],
    alpha_deg: float,
    drag_scale: float,
    cd0: float,
) -> tuple[float, float, float, float, float]:
    """state = (x, z, vx, vz, t), returns (dx, dz, dvx, dvz, dt)."""
    _, _, vx, vz, _ = state
    speed = math.hypot(vx, vz)
    if speed < 1e-9:
        return (vx, vz, 0.0, -GRAVITY, 1.0)

    cd = _cd(alpha_deg, cd0)
    drag_factor = drag_scale * cd * speed
    alpha_rad = math.radians(alpha_deg)
    normal_accel = drag_scale * alpha_rad * speed * speed  # same scale for simplicity

    normal_x = -vz / speed
    normal_z = vx / speed

    ax = -drag_factor * vx + normal_accel * normal_x
    az = -GRAVITY - drag_factor * vz + normal_accel * normal_z
    return (vx, vz, ax, az, 1.0)


def _rk4_step(
    state: tuple[float, float, float, float, float],
    alpha_deg: float,
    dt: float,
    drag_scale: float,
    cd0: float,
) -> tuple[float, float, float, float, float]:
    h = dt
    k1 = _derivative(state, alpha_deg, drag_scale, cd0)
    s2 = tuple(state[i] + 0.5 * h * k1[i] for i in range(5))
    k2 = _derivative(s2, alpha_deg, drag_scale, cd0)
    s3 = tuple(state[i] + 0.5 * h * k2[i] for i in range(5))
    k3 = _derivative(s3, alpha_deg, drag_scale, cd0)
    s4 = tuple(state[i] + h * k3[i] for i in range(5))
    k4 = _derivative(s4, alpha_deg, drag_scale, cd0)
    return tuple(
        state[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0
        for i in range(5)
    )


def _simulate_to_range(
    v0: float,
    launch_angle_rad: float,
    segments: tuple[SegmentSpec, ...],
    target_x: float,
    drag_scale: float,
    cd0: float,
    dt: float = 0.0005,
    max_t: float = 8.0,
) -> tuple[float, float, float, float]:
    """
    Return (z_at_target_x, gamma_at_target_x, speed_at_target_x, flight_time).
    If ground is struck before target_x, return (z_hit, NaN, NaN, hit_time).
    """
    vx0 = v0 * math.cos(launch_angle_rad)
    vz0 = v0 * math.sin(launch_angle_rad)
    prev = (0.0, 0.0, vx0, vz0, 0.0)
    cur = prev
    t = 0.0
    while t < max_t:
        a = _alpha_for_x(prev[0], segments)
        cur = _rk4_step(prev, a, dt, drag_scale, cd0)
        if cur[0] >= target_x:
            span = cur[0] - prev[0]
            ratio = (target_x - prev[0]) / span if span > 1e-12 else 0.0
            z_hit = prev[1] + ratio * (cur[1] - prev[1])
            vx_hit = prev[2] + ratio * (cur[2] - prev[2])
            vz_hit = prev[3] + ratio * (cur[3] - prev[3])
            gamma_hit = math.degrees(math.atan2(vz_hit, vx_hit))
            speed_hit = math.hypot(vx_hit, vz_hit)
            t_hit = prev[4] + ratio * (cur[4] - prev[4])
            return z_hit, gamma_hit, speed_hit, t_hit
        if cur[1] < 0.0:
            # struck ground
            return cur[1], float("nan"), float("nan"), cur[4]
        prev = cur
        t += dt
    return prev[1], float("nan"), float("nan"), prev[4]


def _sample_trajectory(
    v0: float,
    launch_angle_rad: float,
    segments: tuple[SegmentSpec, ...],
    drag_scale: float,
    cd0: float,
    n_samples: int = 501,
    max_x: float = 25.0,
    dt: float = 0.0001,
    max_t: float = 8.0,
) -> list[TrajSample]:
    vx0 = v0 * math.cos(launch_angle_rad)
    vz0 = v0 * math.sin(launch_angle_rad)
    prev = (0.0, 0.0, vx0, vz0, 0.0)
    cur = prev
    t_acc = 0.0
    idx = 0
    out: list[TrajSample] = []
    while idx < n_samples and t_acc < max_t:
        target = max_x * idx / (n_samples - 1)
        a = _alpha_for_x(prev[0], segments)
        cur = _rk4_step(prev, a, dt, drag_scale, cd0)
        while idx < n_samples and prev[0] <= target <= cur[0]:
            span = cur[0] - prev[0]
            r = (target - prev[0]) / span if span > 1e-12 else 0.0
            z = prev[1] + r * (cur[1] - prev[1])
            vx = prev[2] + r * (cur[2] - prev[2])
            vz = prev[3] + r * (cur[3] - prev[3])
            sp = math.hypot(vx, vz)
            gm = math.degrees(math.atan2(vz, vx))
            out.append(TrajSample(x=target, z=z, gamma_deg=gm, speed=sp,
                                  t=prev[4] + r * dt, alpha_deg=a))
            idx += 1
            if idx >= n_samples:
                break
            target = max_x * idx / (n_samples - 1)
        prev = cur
        t_acc += dt
    return out


# ═══════════════════════════════════════════════════════════════════════════
# 1. BASELINE DRAG CALIBRATION
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class CalibrationResult:
    v0: float  # m/s
    cd0: float  # calibrated zero-α drag coefficient
    drag_scale: float  # fixed
    flight_time: float  # s
    impact_gamma: float  # deg
    impact_speed: float  # m/s


def _find_v0_for_range(
    launch_rad: float, segments: tuple[SegmentSpec, ...],
    target_x: float, drag_scale: float, cd0: float,
    v0_hi_start: float = 200.0, v0_hi_max: float = 2000.0,
) -> Optional[float]:
    """Return v0 that makes z≈0 at x=target_x, or None."""
    lo, hi = 1.0, v0_hi_start

    def z_at_range(v: float) -> float:
        return _simulate_to_range(
            v, launch_rad, segments, target_x, drag_scale, cd0
        )[0]

    # Seek an upper bound where trajectory reaches target_x
    while hi <= v0_hi_max:
        z_hi = z_at_range(hi)
        # If dart hits ground before target_x, z_hi is the negative hit-z;
        # otherwise it's the (possibly negative or positive) z at target_x.
        if z_hi > 0.0:
            break  # we have bracketing: z(lo)<0, z(hi)>0
        hi *= 1.5
    else:
        return None  # never goes above ground at target_x

    z_lo = z_at_range(lo)
    if z_lo > 0.0:
        return lo  # even lo is too energetic

    return _bisect(z_at_range, lo, hi)


def calibrate_baseline(
    launch_angle_deg: float = 32.0,
    target_range: float = 25.0,
    target_impact_gamma_deg: float = -45.0,
    target_flight_time_s: float = 2.0,
    drag_scale: float = 0.14,
    cd0_range: tuple[float, float] = (0.005, 2.0),
) -> CalibrationResult:
    """
    Grid-search Cd0 to match the desired impact angle on the baseline
    (α=0) trajectory.
    """
    launch_rad = math.radians(launch_angle_deg)
    baseline_seg = (SegmentSpec("baseline", 0.0, target_range, 0.0),)
    best = None
    best_err = float("inf")
    n_grid = 200

    for i in range(n_grid + 1):
        cd0 = cd0_range[0] + (cd0_range[1] - cd0_range[0]) * i / n_grid
        v0 = _find_v0_for_range(
            launch_rad, baseline_seg, target_range, drag_scale, cd0
        )
        if v0 is None:
            continue
        _, gamma, speed, t = _simulate_to_range(
            v0, launch_rad, baseline_seg, target_range, drag_scale, cd0
        )
        if math.isnan(gamma):
            continue
        err = abs(gamma - target_impact_gamma_deg)
        if err < best_err:
            best_err = err
            best = CalibrationResult(
                v0=v0, cd0=cd0, drag_scale=drag_scale,
                flight_time=t, impact_gamma=gamma, impact_speed=speed,
            )
    if best is None:
        raise RuntimeError("No feasible Cd0 found in range")
    return best


# ═══════════════════════════════════════════════════════════════════════════
# 2. THREE-SEGMENT TRAJECTORY PLANNER
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class ThreeSegmentPlan:
    segments: tuple[SegmentSpec, ...]
    v0: float
    cd0: float
    drag_scale: float
    samples: list[TrajSample] = field(default_factory=list)

    @property
    def terminal_start_x(self) -> float:
        return self.segments[2].start_x

    @property
    def terminal_range(self) -> float:
        """horizontal distance of the terminal segment."""
        return self.segments[2].end_x - self.segments[2].start_x


@dataclass
class TerminalAnalysis:
    plan_label: str
    terminal_range: float
    x_start: float
    x_end: float
    z_start: float
    z_end: float
    gamma_start: float
    gamma_end: float
    speed_start: float
    speed_end: float
    best_fit_slope_deg: float  # linear regression slope of last segment
    max_deviation_m: float  # max deviation from best-fit line
    relative_deviation_pct: float  # max_dev / terminal_range * 100


def _solve_three_segment(
    up_turn_end_x: float,
    terminal_range: float,
    alpha_max_deg: float,
    terminal_alpha_deg: float,
    total_range: float,
    launch_angle_deg: float,
    drag_scale: float,
    cd0: float,
) -> Optional[tuple[float, tuple[SegmentSpec, ...]]]:
    """Return (v0, segments) for a concrete three-segment plan, or None."""
    launch_rad = math.radians(launch_angle_deg)
    segs = (
        SegmentSpec("up_turn", 0.0, up_turn_end_x, +alpha_max_deg),
        SegmentSpec("down_turn_dive", up_turn_end_x, total_range - terminal_range, -alpha_max_deg),
        SegmentSpec("terminal_pull", total_range - terminal_range, total_range, terminal_alpha_deg),
    )
    v0 = _find_v0_for_range(launch_rad, segs, total_range, drag_scale, cd0)
    if v0 is None:
        return None
    return v0, segs


def analyse_terminal_linearity(
    samples: list[TrajSample],
    terminal_start_x: float,
    label: str = "",
) -> TerminalAnalysis:
    """Quantify how close the final segment is to a straight line."""
    pts = [(s.x, s.z, s.gamma_deg, s.speed) for s in samples
           if s.x >= terminal_start_x - 0.001]
    if len(pts) < 2:
        raise ValueError("not enough samples in terminal phase")

    xs, zs = [p[0] for p in pts], [p[1] for p in pts]
    slope = (zs[-1] - zs[0]) / (xs[-1] - xs[0])
    intercept = zs[0] - slope * xs[0]
    devs = [abs(z - (slope * x + intercept)) for x, z in zip(xs, zs)]
    max_dev = max(devs)
    horiz = xs[-1] - xs[0]

    return TerminalAnalysis(
        plan_label=label,
        terminal_range=horiz,
        x_start=xs[0], x_end=xs[-1],
        z_start=zs[0], z_end=zs[-1],
        gamma_start=pts[0][2], gamma_end=pts[-1][2],
        speed_start=pts[0][3], speed_end=pts[-1][3],
        best_fit_slope_deg=math.degrees(math.atan2(zs[-1] - zs[0], xs[-1] - xs[0])),
        max_deviation_m=max_dev,
        relative_deviation_pct=max_dev / horiz * 100.0 if horiz > 0 else 0.0,
    )


# ═══════════════════════════════════════════════════════════════════════════
# 3. CLI
# ═══════════════════════════════════════════════════════════════════════════

def _fmt_ok(b: bool) -> str:
    return "✓" if b else "✗"


def _print_calibration(c: CalibrationResult) -> None:
    print()
    print("=" * 66)
    print("  BASELINE DRAG CALIBRATION (α = 0°)")
    print("=" * 66)
    print(f"  Target constraints:  range = 25.0 m,  γ_impact ≈ -45.0°,  t ≈ 2.0 s")
    print(f"  Calibrated Cd0     = {c.cd0:.5f}   (drag_scale = {c.drag_scale})")
    print(f"  → k_eff @ α=0      = {c.drag_scale * c.cd0:.5f}   (quadratic-drag coefficient)")
    print(f"  Launch speed  v0   = {c.v0:.3f} m/s")
    print(f"  Flight time        = {c.flight_time:.3f} s")
    print(f"  Impact gamma       = {c.impact_gamma:.3f}°")
    print(f"  Impact speed       = {c.impact_speed:.3f} m/s")
    print(f"  Cd @ α=±10°        = {_cd(10.0, c.cd0):.4f}")


def _print_plan(label: str, v0: float, segments: tuple[SegmentSpec, ...]) -> None:
    print(f"\n  [{label}]  v0 = {v0:.3f} m/s")
    for seg in segments:
        print(f"    {seg.name:<22s}  x = {seg.start_x:5.1f} → {seg.end_x:5.1f} m  "
              f"α = {seg.alpha_deg:+6.1f}°")


def _print_terminal(a: TerminalAnalysis) -> None:
    in_corridor = -60.0 <= a.best_fit_slope_deg <= -40.0
    straight = "very straight" if a.relative_deviation_pct < 2 else \
               "moderate" if a.relative_deviation_pct < 8 else "curved"
    print(f"    terminal |  Δx={a.terminal_range:.1f}m  "
          f"z: {a.z_start:.2f}→{a.z_end:.2f}m  "
          f"γ: {a.gamma_start:.1f}→{a.gamma_end:.1f}°  "
          f"speed: {a.speed_start:.1f}→{a.speed_end:.1f} m/s")
    print(f"             |  best-fit slope = {a.best_fit_slope_deg:+.2f}°  "
          f"corridor[-40..-60]: {_fmt_ok(in_corridor)}  "
          f"straightness: {straight}  (max dev {a.max_deviation_m:.3f} m = {a.relative_deviation_pct:.2f}%)")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Drag-calibrated three-segment dart trajectory planner."
    )
    ap.add_argument("--launch-angle-deg", type=float, default=32.0)
    ap.add_argument("--range-m", type=float, default=25.0)
    ap.add_argument("--target-impact-gamma-deg", type=float, default=-45.0)
    ap.add_argument("--target-flight-time-s", type=float, default=2.0)
    ap.add_argument("--stall-limit-deg", type=float, default=10.0)
    ap.add_argument("--terminal-alpha-deg", type=float, default=None,
                    help="terminal α override (default: same as stall-limit)")
    ap.add_argument("--drag-scale", type=float, default=0.14)
    ap.add_argument("--up-turn-end-x", type=float, nargs="+",
                    default=[4.0, 5.0, 6.0])
    ap.add_argument("--terminal-range", type=float, nargs="+",
                    default=[6.0, 7.0, 8.0])
    args = ap.parse_args()

    if args.terminal_alpha_deg is None:
        args.terminal_alpha_deg = args.stall_limit_deg

    # ── step 1: calibrate baseline drag ─────────────────────────────────
    cal = calibrate_baseline(
        launch_angle_deg=args.launch_angle_deg,
        target_range=args.range_m,
        target_impact_gamma_deg=args.target_impact_gamma_deg,
        target_flight_time_s=args.target_flight_time_s,
        drag_scale=args.drag_scale,
    )
    _print_calibration(cal)

    # ── step 2: three-segment sweep ─────────────────────────────────────
    print()
    print("=" * 66)
    print(f"  THREE-SEGMENT TRAJECTORY SWEEP  (α_max = ±{args.stall_limit_deg:.0f}°)")
    print(f"  Calibrated Cd0 = {cal.cd0:.5f}")
    print("=" * 66)

    for up_end in args.up_turn_end_x:
        for tr in args.terminal_range:
            launch_rad = math.radians(args.launch_angle_deg)
            result = _solve_three_segment(
                up_turn_end_x=up_end,
                terminal_range=tr,
                alpha_max_deg=args.stall_limit_deg,
                terminal_alpha_deg=args.terminal_alpha_deg,
                total_range=args.range_m,
                launch_angle_deg=args.launch_angle_deg,
                drag_scale=cal.drag_scale,
                cd0=cal.cd0,
            )
            if result is None:
                print(f"  [up_end={up_end:.0f}  term={tr:.0f}m]  NO SOLUTION")
                continue
            v0, segs = result

            label = f"up_end={up_end:.0f}m  term_range={tr:.0f}m"
            _print_plan(label, v0, segs)

            samples = _sample_trajectory(
                v0, launch_rad, segs,
                drag_scale=cal.drag_scale, cd0=cal.cd0,
                max_x=args.range_m,
            )
            ta = analyse_terminal_linearity(samples, segs[2].start_x, label)
            _print_terminal(ta)


if __name__ == "__main__":
    main()
