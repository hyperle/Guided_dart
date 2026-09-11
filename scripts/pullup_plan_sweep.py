#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无动力飞镖 "拉升(跃起)-归位贴合基础抛物线" 舵面转动计划扫参 (规划层, 轻量, 纯 Python 无依赖)

物理模型 (窗口内点质量, 速度坐标系):
    gamma_dot = (g/V)(n - cos(gamma)),   n = 0.5*rho*V^2*S*C_Ld*delta / (m*g)
    V_dot     = -g*sin(gamma) - D/m,     D = 0.5*rho*V^2*S*(C_D0 + kd*delta^2)
    h_dot = V*sin(gamma),  x_dot = V*cos(gamma)
    机械能损失率: E_dot_loss = D*V

计划参数族 (对应"恒定拉升 + 线性过渡"):
    [0,T1]      : delta = +dmax          (拉升保持)
    [T1,T1+T2]  : delta 线性 +dmax -> -dmax (线性过渡)
    [T1+T2,T1+T2+T3]: delta = -dmax      (下压保持, 可选)
    [..,+T4]    : delta 线性 -dmax -> 0   (归零, 默认 T4=T2)
    之后 delta=0 纯弹道。 扫 (T1, T2); T3/T4 固定输入。

约束(满足才计入可行解):
    A. 改出点相对高度 dH_w = h(tW)-h_base(tW) >= dh_req        (高度指标=改出点)
    B. 方向贴合: |dgamma_w| = |gamma(tW)-gamma_base(tW)| <= gam_tol (基础抛物线)
    C. 控制时长 tW <= T_ctrl_max 且全程 V >= V_min
目标: min 额外机械能损失 dE = (E_loss_dart - E_loss_base) 于改出点
附带: 输出时长-损失的 Pareto 前沿 与 基础(未控)轨迹对照, 验证"时长~水平距离"等价性。

用法:
    python3 scripts/pullup_plan_sweep.py                    # 全部默认(示例)参数
    python3 scripts/pullup_plan_sweep.py m=0.25 V0=25 S=0.02 CLd=2.0 CD0=0.03 \
              gam0=-35 dh_req=5 T1max=0.6 T2max=0.4        # 覆盖任意键值

注意: 所有默认值均为 EXAMPLE 占位(按 <0.3kg 小飞镖假设), 拿到实测气动/几何后直接覆盖重跑。
"""

import sys
import math

# ------------------------------ 默认参数 (EXAMPLE) ------------------------------
PAR = {
    # 气动/几何 (示例占位)
    "m": 0.25, "S": 0.02, "rho": 1.225,
    "CD0": 0.03,      # 零阻力系数 (弹道段也吃这个)
    "kd": 1.5,        # 阻力随舵偏增量: CD = CD0 + kd*delta_rad^2
    "CLd": 2.0,       # 舵偏升力斜率 1/rad
    # 触发点状态 (基础抛物线上某切点)
    "V0": 25.0, "gam0": -35.0, "h0": 20.0,
    # 舵偏限制
    "dmax": 15.0,     # 度
    # 计划结构
    "T3_hold": 0.0,   # -dmax 保持秒 (0 表示过渡到 -dmax 后立即线性回零)
    # 约束
    "dh_req": 5.0,    # 改出点相对高度需求 (m)
    "gam_tol": 0.5,   # 方向贴合容差 (deg)
    "T_ctrl_max": 1.0,# 控制时长上限 (s)
    "Vmin": 8.0,      # 全程最低速度 (m/s)
    # 数值
    "dt": 0.002, "horizon": 4.0,
    # 扫描范围 (T1: 拉升保持, T2: 线性过渡)
    "T1min": 0.02, "T1max": 0.60, "T1step": 0.02,
    "T2min": 0.02, "T2max": 0.40, "T2step": 0.02,
}

for kv in sys.argv[1:]:
    k, _, v = kv.partition("=")
    if k in PAR:
        PAR[k] = float(v)
    else:
        sys.exit("未知参数: %s (可覆盖: %s)" % (kv, sorted(PAR)))

D2R = math.pi / 180.0


def profile_delta(t, T1, T2, T3, T4):
    """piecewise: +dmax hold T1, linear -> -dmax over T2, hold T3, linear -> 0 over T4."""
    if t < 0:
        return PAR["dmax"]
    if t < T1:
        return PAR["dmax"]
    if t < T1 + T2:
        u = (t - T1) / T2
        return PAR["dmax"] * (1.0 - 2.0 * u)          # +dmax -> -dmax
    if t < T1 + T2 + T3:
        return -PAR["dmax"]
    if t < T1 + T2 + T3 + T4:
        u = (t - T1 - T2 - T3) / T4
        return -PAR["dmax"] * (1.0 - u)               # -dmax -> 0
    return 0.0


def deriv(V, gam, delta_deg):
    """返回 (dV, dgam, dh, dx, dloss); delta_deg 为舵偏(度), 带符号."""
    d = delta_deg * D2R
    qS = 0.5 * PAR["rho"] * V * V * PAR["S"]
    n = qS * PAR["CLd"] * d / (PAR["m"] * 9.80665)
    D = qS * (PAR["CD0"] + PAR["kd"] * d * d)
    cosg, sing = math.cos(gam), math.sin(gam)
    dV = -D / PAR["m"] - 9.80665 * sing
    dgam = 9.80665 / V * (n - cosg)
    return dV, dgam, V * sing, V * cosg, D * V


def simulate(T1, T2):
    """从触发点同时积分 飞镖(带舵) 与 基础弹道(无舵), 返回指标字典."""
    T3, T4 = PAR["T3_hold"], T2
    tW = T1 + T2 + T3 + T4
    HOR = max(PAR["horizon"], tW + 0.5)
    n_steps = int(round(HOR / PAR["dt"]))
    dt = PAR["dt"]

    st_d = [PAR["V0"], PAR["gam0"] * D2R, PAR["h0"], 0.0, 0.0]   # dart
    st_b = [PAR["V0"], PAR["gam0"] * D2R, PAR["h0"], 0.0, 0.0]   # base
    t = 0.0

    def rk4(s, delta_deg):
        k1 = deriv(s[0], s[1], delta_deg)
        k2 = deriv(s[0] + dt / 2 * k1[0], s[1] + dt / 2 * k1[1], delta_deg)
        k3 = deriv(s[0] + dt / 2 * k2[0], s[1] + dt / 2 * k2[1], delta_deg)
        k4 = deriv(s[0] + dt * k3[0], s[1] + dt * k3[1], delta_deg)
        return [s[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(5)]

    snap_d = snap_b = None   # 改出点(窗口结束)快照
    Vmin_d = 1e9
    for i in range(n_steps + 1):
        # 状态对应时间 t: 首次越过窗口结束时刻 -> 快照(指标全在改出点取)
        if snap_d is None and t >= tW - 1e-12:
            snap_d = list(st_d)
            snap_b = list(st_b)
        dlt = profile_delta(t, T1, T2, T3, T4)
        st_d = rk4(st_d, dlt)
        st_b = rk4(st_b, 0.0)
        Vmin_d = min(Vmin_d, st_d[0])
        t += dt

    Vd, gam_d, hd, xd, loss_d = snap_d
    Vb, gam_b, hb, xb, loss_b = snap_b
    dH_w = hd - hb                     # 改出点相对高度
    dgam_w = (gam_d - gam_b) / D2R     # 方向贴合误差 (deg)
    dE_w = loss_d - loss_b             # 额外阻力做功 (负=比基础弹道省)
    dE_mech = -dE_w                    # 改出点剩余总机械能差 = mg*dH + 0.5m(dV^2)
    dx_w = xd - xb
    dV_w = Vd - Vb
    return dict(T1=T1, T2=T2, tW=tW, dH_w=dH_w, dgam_w=dgam_w, dE_w=dE_w,
                dE_mech=dE_mech, dx_w=dx_w, dV_w=dV_w, V_w=Vd, Vb_w=Vb,
                Vmin_d=Vmin_d)


def arange(a, b, s):
    out, v = [], a
    while v <= b + 1e-9:
        out.append(round(v, 6))
        v += s
    return out


def main():
    g = 9.80665
    # ---- 触发点可用舵效诊断: 低速俯冲下拉不起来的临界速度 ----
    dmax_r = PAR["dmax"] * D2R
    Vcrit = math.sqrt(g * math.cos(PAR["gam0"] * D2R) * 2 * PAR["m"]
                      / (PAR["rho"] * PAR["S"] * PAR["CLd"] * dmax_r))
    print("=" * 74)
    print("无动力拉升计划扫参   (全部参数为示例默认值, 可用 key=value 覆盖)")
    print("=" * 74)
    print("触发点: V0=%.1f m/s  gam0=%.1f deg  h0=%.1f m" % (PAR["V0"], PAR["gam0"], PAR["h0"]))
    max_rate = (g / PAR["V0"] *
                (PAR["rho"] * PAR["V0"] * PAR["V0"] * PAR["S"] * PAR["CLd"] * dmax_r /
                 (2 * PAR["m"] * g) - math.cos(PAR["gam0"] * D2R)) * 180 / math.pi)
    print("舵效诊断: 保持 %.0fdeg 的最大纯拉升角速率 ≈ %.1f deg/s" %
          (PAR["dmax"], max_rate))
    note = ("   <-- 当前 V0 低于临界, 拉不起来!" if PAR["V0"] < Vcrit
            else "   (当前 V0 高于临界, 可行)")
    print("          该俯冲角下能拉起的最小速度 Vcrit ≈ %.1f m/s%s" % (Vcrit, note))
    print("约束: 改出点 dH >= %s m, |dgamma| <= %s deg, T_ctrl <= %s s, V >= %s m/s" %
          (PAR["dh_req"], PAR["gam_tol"], PAR["T_ctrl_max"], PAR["Vmin"]))

    rows = []
    sims = 0
    for T1 in arange(PAR["T1min"], PAR["T1max"], PAR["T1step"]):
        for T2 in arange(PAR["T2min"], PAR["T2max"], PAR["T2step"]):
            r = simulate(T1, T2)
            rows.append(r)
            sims += 1

    feas = [r for r in rows
            if r["dH_w"] >= PAR["dh_req"] - 1e-9
            and abs(r["dgam_w"]) <= PAR["gam_tol"]
            and r["tW"] <= PAR["T_ctrl_max"]
            and r["Vmin_d"] >= PAR["Vmin"]]
    feas.sort(key=lambda r: r["dE_w"])
    print("扫描 %d 组 (T1 x T2), 满足全部约束的可行解 %d 组\n" % (sims, len(feas)))

    # 基线: 原来的固定方案
    b0 = simulate(0.05, 0.05)
    print(f"原方案基线 (T1=50ms, T2=50ms): dH_w={b0['dH_w']:+.2f} m, "
          f"dgamma_w={b0['dgam_w']:+.2f} deg, dE_w={b0['dE_w']:+.3f} J, "
          f"dV_w={b0['dV_w']:+.2f} m/s, tW={b0['tW']*1e3:.0f} ms, dx_w={b0['dx_w']:+.2f} m")

    # ---- 需求扫描: 每个拉升高度需求 -> 最短时长 / 最小能量损失 ----
    print("\n需求扫描 (tW<=%.1fs, V>=%.0fm/s): 每个需求给 '最小能量损失解(T1/T2ms, tW, ΔE)'"
          % (PAR["T_ctrl_max"], PAR["Vmin"]))
    h1 = "贴合容差 %.1f deg" % PAR["gam_tol"]
    h2 = "贴合容差 %.1f deg" % (PAR["gam_tol"] + 1.5)
    print("  %6s | %-32s | %-32s" % ("dh_req", h1, h2))
    for req in arange(0.5, 10.001, 0.5):
        def best_for(tol):
            c = [r for r in rows
                 if r["dH_w"] >= req - 1e-9 and abs(r["dgam_w"]) <= tol
                 and r["tW"] <= PAR["T_ctrl_max"] and r["Vmin_d"] >= PAR["Vmin"]]
            if not c:
                return None
            b = min(c, key=lambda r: r["dE_w"])
            return b
        cell = []
        for tol in (PAR["gam_tol"], PAR["gam_tol"] + 1.5):
            b = best_for(tol)
            cell.append("   -   " if b is None else
                        f" {b['T1']*1e3:3.0f}/{b['T2']*1e3:3.0f}ms "
                        f"tW={b['tW']*1e3:4.0f}ms dE={b['dE_w']:+6.2f}J")
        print(f"  {req:6.1f} | {cell[0]:<32} | {cell[1]:<32}")

    if not feas:
        print("\n[当前约束下无可行解] 逐项检查: ① dh_req 太大? ② 贴合容差太紧? "
              "③ V0 接近/低于 Vcrit? ④ T_ctrl_max 太小?")
        near = sorted(rows, key=lambda r: abs(r["dH_w"] - PAR["dh_req"]))[:5]
        print("最接近高度需求的几组(供诊断, 未过滤贴合/时长):")
        for r in near:
            print(f"  T1={r['T1']*1e3:5.0f}ms T2={r['T2']*1e3:5.0f}ms  dH_w={r['dH_w']:+.2f} m "
                  f"dgam_w={r['dgam_w']:+.2f}deg dE={r['dE_w']:+.3f}J tW={r['tW']*1e3:.0f}ms")
        return

    best = feas[0]
    print(f"\n[最优] T1={best['T1']*1e3:.0f} ms  T2={best['T2']*1e3:.0f} ms  "
          f"(窗口总时长 {best['tW']*1e3:.0f} ms)")
    print(f"  改出点: dH_w={best['dH_w']:+.2f} m (要求 >= {PAR['dh_req']})   "
          f"dgamma_w={best['dgam_w']:+.3f} deg (贴合容差 {PAR['gam_tol']})")
    print(f"  额外阻力做功 dE_w={best['dE_w']:+.3f} J (剩余总机械能多保留 {-best['dE_w']:.3f} J)"
          f"   终速差 dV_w={best['dV_w']:+.2f} m/s  (V_w={best['V_w']:.1f} vs 基础 {best['Vb_w']:.1f})")
    print(f"  相对水平位移 dx_w={best['dx_w']:+.2f} m")

    print("\n可行解 Pareto 前沿 (按控制时长, 同时长取能量保留最多):")
    pareto, seen = [], set()
    for r in feas:
        key = round(r["tW"] * 1e3 / 50) * 50  # 50ms 桶
        if key in seen:
            continue
        seen.add(key)
        pareto.append(r)
    for r in pareto:
        print(f"  tW={r['tW']*1e3:5.0f}ms  T1={r['T1']*1e3:5.0f}ms T2={r['T2']*1e3:5.0f}ms "
              f"dH_w={r['dH_w']:+5.2f}m  dE_w={r['dE_w']:+.3f}J  dV_w={r['dV_w']:+4.2f}m/s  "
              f"dx_w={r['dx_w']:+5.2f}m")

    print("\n说明: dE_w 为改出点处相对基础弹道的额外阻力做功(负=保留更多能量); "
          "时长与水平位移 dx_w 同向, 验证了你的等价性判断.")


if __name__ == "__main__":
    main()
