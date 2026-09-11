#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仅 IMU 六维 + 静态参数(质量/面积) + 已知初速度 的无动力拉升控制 仿真演示
================================================================
目的: 验证控制架构不依赖气动模型, 展示估计/控制误差量级, 以及
      "迭代测试(用地面落地距离做零偏学习 + 推算峰值高度调过载)" 如何收敛。

架构 (控制与估计只用 IMU 模拟量):
  真值弹道(二维纵平面点质量) -> 只用于生成 IMU 输出与验收真值
  IMU 模型: 加计比力 f = F_aero/m (物理恒等式) + 零偏/噪声; 陀螺 q + 零偏/噪声
  估计器:   θ̂ = 陀螺积分 + 准静态加计校正
            v̂ = v0 + ∫(g + R(θ̂)f_meas - û)dt   -> γ̂, ĥ, n̂_z = f_z/g
  控制:     滑模面 s = (n̂_z - n_cmd) + λ∫(n̂_z - n_cmd)
            δ_cmd = -k1·sat(s/φ) - k2·q̂        (3 参数, 无模型项, 单位统一为 rad)
  拉升计划: t_trig 触发后执行过载剖面 n_cmd(t) (早期窗口)
  迭代测试: 每轮: ①用实测落地距离 x_imp 修正零偏 û  ②用推算峰值高度调 n_pull

用法: python3 scripts/imu_only_hop_demo.py [seed=1] [dh_target=1.5]
"""
import math
import random
import sys

DEG = math.pi / 180.0
G = 9.80665

SEED = 1
DX_TARGET = 4.0     # 迭代目标: 拉升后的落地距离相对基线 = +4.0 m (场地可测)
for kv in sys.argv[1:]:
    k, _, v = kv.partition("=")
    if k == "seed":
        SEED = int(v)
    if k == "dx_target":
        DX_TARGET = float(v)

# --------------------------- 仿真器参数 (真值, 控制器不知道) ---------------------------
TR = dict(
    m=0.25, S=0.02, rho=1.225,
    CD0=0.03, kd=1.5, CLd=2.0,      # C_L = CLd*delta (delta 以弧度计)
    CLalpha=4.5,                    # 合成 α (仅用于把比力转到机体坐标生成 IMU)
    V0=30.0, gam0=-10.0, h0=25.0,   # 发射初值(测试可测: 初速度)
    t_trig=0.25,                    # 拉升触发: 早期窗口
    T1=0.30, T2=0.45, T4=0.45,      # 剖面: +n 保持 -> 线性到 -n -> 回 0
    dt=0.002, Tmax=6.0,
    tau_act=0.015, rate_max=300.0 * DEG, dmax=15.0 * DEG,
)
IMU = dict(ab=0.02 * G, ag=0.015 * G, gb=0.5 * DEG, gg=0.15 * DEG)  # 零偏/噪声
CTL = dict(k1=15.0, k2=0.30, lam=2.0, phi=0.08, lp=20.0)            # 控制 3 参数
ADAPT = dict(damp_bias=0.20, gain_n=0.15, nmin=0.3, nmax=2.0)


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s), (s, c))


def rot_mul(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1], R[1][0] * v[0] + R[1][1] * v[1])


def aero_force(V, delta_rad):
    """风轴比力 (沿速度, 法向上) = (-D/m, L/m). delta>0 -> 正升力"""
    d = delta_rad
    qS = 0.5 * TR["rho"] * V * V * TR["S"]
    L = qS * TR["CLd"] * d
    D = qS * (TR["CD0"] + TR["kd"] * d * d)
    return (-D / TR["m"], L / TR["m"])


class Flight:
    def __init__(self, rng, uhat):
        self.rng = rng
        self.uhat = list(uhat)
        self.alpha_prev = 0.0

    def n_cmd(self, t):
        t0 = TR["t_trig"]
        if t < t0:
            return 0.0
        t -= t0
        if t < TR["T1"]:
            return self.np
        if t < TR["T1"] + TR["T2"]:
            u = (t - TR["T1"]) / TR["T2"]
            return self.np * (1.0 - 2.0 * u)
        if t < TR["T1"] + TR["T2"] + TR["T4"]:
            u = (t - TR["T1"] - TR["T2"]) / TR["T4"]
            return -self.np * (1.0 - u)
        return 0.0

    def fly(self, n_pull):
        self.np = n_pull
        r = self.rng
        bias_a = (r.gauss(0, IMU["ab"]), r.gauss(0, IMU["ab"]))
        bias_g = r.gauss(0, IMU["gb"])
        dt = TR["dt"]
        tW = TR["t_trig"] + TR["T1"] + TR["T2"] + TR["T4"]

        V, gam, h, x, d = TR["V0"], TR["gam0"] * DEG, TR["h0"], 0.0, 0.0   # d: rad

        # 估计器
        th = TR["gam0"] * DEG
        vx, vz = TR["V0"] * math.cos(gam), TR["V0"] * math.sin(gam)
        hh, xx = TR["h0"], 0.0
        nfilt, s_int = 0.0, 0.0
        lp_c = 2 * math.pi * CTL["lp"] * dt

        peak_h = h
        peak_hhat = hh
        gam_emax = h_emax = x_emax = 0.0
        n_acc = n_cnt = 0.0
        # 无舵基线孪生(真值侧): 用于给出"相对基础弹道同刻高度差"的拉升增益
        Vb, gamb, hb, xb = TR["V0"], TR["gam0"] * DEG, TR["h0"], 0.0
        pk_rel = 0.0

        t = 0.0
        while t < TR["Tmax"]:
            # ---------- 当前状态下的 IMU 输出 ----------
            alpha = TR["CLd"] * d / TR["CLalpha"]
            adot = (alpha - self.alpha_prev) / dt if t > 0 else 0.0
            self.alpha_prev = alpha
            qS = 0.5 * TR["rho"] * V * V * TR["S"]
            n = qS * TR["CLd"] * d / (TR["m"] * G)
            gdot = G / V * (n - math.cos(gam))
            q_true = gdot + adot
            q_m = q_true + bias_g + r.gauss(0, IMU["gg"])

            f_body = rot_mul(rot(-alpha), aero_force(V, d))
            f_meas = (f_body[0] + bias_a[0] + r.gauss(0, IMU["ag"]),
                      f_body[1] + bias_a[1] + r.gauss(0, IMU["ag"]))

            # ---------- 估计器 ----------
            # 姿态: 陀螺积分 + 加计校正(仅当读数≈"静态重力读数 g_body(θ̂)" 才信)
            mag = math.hypot(*f_meas)
            th_acc = math.atan2(f_meas[0], f_meas[1])
            f_static = (G * math.sin(th), G * math.cos(th))     # 静态时加计应读到的值
            aero_mag = math.hypot(f_meas[0] - f_static[0], f_meas[1] - f_static[1])
            kcorr = 4.0 if aero_mag < 0.35 * G else 0.15        # 机动/下压时关闭加计校正
            dth = th_acc - th
            dth = (dth + math.pi) % (2 * math.pi) - math.pi     # 回绕到 (-π,π]
            th += q_m * dt + kcorr * dth * dt
            th = (th + math.pi) % (2 * math.pi) - math.pi
            f_earth = rot_mul(rot(th), f_meas)
            vx += (f_earth[0] - self.uhat[0]) * dt
            vz += (f_earth[1] - G - self.uhat[1]) * dt
            hh += vz * dt
            xx += vx * dt
            if math.hypot(vx, vz) > 0.1:
                ghat = math.atan2(vz, vx)
            else:
                ghat = 0.0

            # ---------- 控制器 (滑模, 只用加计/陀螺; 指令单位 rad) ----------
            nz = f_body[1] / G
            nfilt += lp_c * (nz - nfilt)
            nc = self.n_cmd(t)
            e_n = nfilt - nc
            sat = max(-1.0, min(1.0, (e_n + CTL["lam"] * s_int) / CTL["phi"]))
            d_cmd = (-CTL["k1"] * sat - CTL["k2"] * q_m) * DEG
            d_cmd = max(-TR["dmax"], min(TR["dmax"], d_cmd))
            # 抗饱和: 舵面已顶死且误差方向不变时冻结积分
            if not (abs(d_cmd) >= TR["dmax"] - 1e-9 and e_n * d_cmd > 0):
                s_int += e_n * dt

            # ---------- 真值一步 (RK4, 舵机一阶滞后+速率/饱和限制) ----------
            def deriv(st):
                Vv, gv, hv, xv, dv = st
                qSv = 0.5 * TR["rho"] * Vv * Vv * TR["S"]
                nv = qSv * TR["CLd"] * dv / (TR["m"] * G)
                Dv = qSv * (TR["CD0"] + TR["kd"] * dv * dv)
                ddc = max(-TR["dmax"], min(TR["dmax"], d_cmd))
                dvr = max(-TR["rate_max"], min(TR["rate_max"], (ddc - dv) / TR["tau_act"]))
                return (-Dv / TR["m"] - G * math.sin(gv),
                        G / Vv * (nv - math.cos(gv)),
                        Vv * math.sin(gv), Vv * math.cos(gv), dvr)

            st = (V, gam, h, x, d)
            k1 = deriv(st)
            k2 = deriv(tuple(st[i] + dt / 2 * k1[i] for i in range(5)))
            k3 = deriv(tuple(st[i] + dt / 2 * k2[i] for i in range(5)))
            k4 = deriv(tuple(st[i] + dt * k3[i] for i in range(5)))
            V, gam, h, x, d = [st[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
                               for i in range(5)]

            # 基线孪生: 同初值无舵弹道
            def b_deriv(s):
                Vv, gv, hv, xv = s
                Dv = 0.5 * TR["rho"] * Vv * Vv * TR["S"] * TR["CD0"]
                return (-Dv / TR["m"] - G * math.sin(gv),
                        -G * math.cos(gv) / Vv, Vv * math.sin(gv), Vv * math.cos(gv))

            sb = (Vb, gamb, hb, xb)
            b1 = b_deriv(sb)
            b2 = b_deriv(tuple(sb[i] + dt / 2 * b1[i] for i in range(4)))
            b3 = b_deriv(tuple(sb[i] + dt / 2 * b2[i] for i in range(4)))
            b4 = b_deriv(tuple(sb[i] + dt * b3[i] for i in range(4)))
            Vb, gamb, hb, xb = [sb[i] + dt / 6 * (b1[i] + 2 * b2[i] + 2 * b3[i] + b4[i])
                                for i in range(4)]
            pk_rel = max(pk_rel, h - hb)

            # ---------- 指标 ----------
            peak_h = max(peak_h, h)
            peak_hhat = max(peak_hhat, hh)
            gam_emax = max(gam_emax, abs(ghat - gam))
            h_emax = max(h_emax, abs(hh - h))
            x_emax = max(x_emax, abs(xx - x))
            if TR["t_trig"] - 0.05 <= t <= tW + 0.05:
                n_acc += (nfilt - nc) ** 2
                n_cnt += 1.0
            t += dt
            if h <= 0.0:
                break

        return dict(impact_x=x, impact_t=t, xhat_end=xx, hhat_end=hh,
                    peak_h=peak_h, peak_hhat=peak_hhat, pk_rel=pk_rel,
                    gam_emax=gam_emax, h_emax=h_emax, x_emax=x_emax,
                    n_rms=math.sqrt(n_acc / max(1.0, n_cnt)))


def baseline():
    """无控弹道落地距离"""
    V, gam, h, x = TR["V0"], TR["gam0"] * DEG, TR["h0"], 0.0
    dt = TR["dt"]
    while True:
        qS = 0.5 * TR["rho"] * V * V * TR["S"]
        D = qS * TR["CD0"]
        V += (-D / TR["m"] - G * math.sin(gam)) * dt
        gam += (-G * math.cos(gam) / V) * dt
        h += V * math.sin(gam) * dt
        x += V * math.cos(gam) * dt
        if h <= 0.0 or V < 1.0:
            return x


def main():
    rng = random.Random(SEED)
    x_base = baseline()
    print("=" * 104)
    print("仅IMU无动力拉升 控制架构演示 (seed=%d)   —— 仿真器参数对控制器/估计器不可见" % SEED)
    print("=" * 104)
    print(f"初值(测试可测): V0={TR['V0']} m/s  γ0={TR['gam0']}°  h0={TR['h0']} m    "
          f"基线无控落地 x_base={x_base:.2f} m")
    print(f"拉升剖面: 触发 {TR['t_trig']*1e3:.0f} ms (早期窗口), 保持 {TR['T1']*1e3:.0f} ms, "
          f"线性 {TR['T2']*1e3:.0f} ms, 回中 {TR['T4']*1e3:.0f} ms")
    print(f"IMU: 加计零偏/噪声 {IMU['ab']/G:.2f}g/{IMU['ag']/G:.2f}g   "
          f"陀螺零偏/噪声 {IMU['gb']/DEG:.1f}°/s/{IMU['gg']/DEG:.1f}°/s")
    print(f"控制: k1={CTL['k1']}°  k2={CTL['k2']}°  λ={CTL['lam']}  φ={CTL['phi']}  (3参数, 无模型项)")
    print(f"迭代目标(场地验收量): 落地距离相对基线 Δx_imp ≈ {DX_TARGET:+.1f} m;  "
          f"每轮用实测落地距离修正估计零偏\n")

    hdr = (f"{'轮':>3} {'n_pull':>6} | {'Δh_rel峰值':>9} {'Δx落地':>7} {'目标':>6} | "
           f"{'|Δγ̂|max':>8} {'高度估差':>7} {'x̂估差':>7} {'n̂跟踪rms':>9}")
    print(hdr)
    print("-" * 104)
    uhat = [0.0, 0.0]
    n_pull = 1.4
    for it in range(1, 9):
        f = Flight(rng, uhat)
        R = f.fly(n_pull)
        t_imp = R["impact_t"]
        # ① 地面测量闭环: 实测落地距离 vs 推算落地距离 -> 修正估计零偏
        err_x = R["impact_x"] - R["xhat_end"]
        err_h = -R["hhat_end"]                    # 撞地时推算高度应≈0
        uhat[0] += ADAPT["damp_bias"] * (-2.0 * err_x) / (t_imp * t_imp)
        uhat[1] += ADAPT["damp_bias"] * (2.0 * err_h) / (t_imp * t_imp)
        uhat[0] = max(-0.15 * G, min(0.15 * G, uhat[0]))
        uhat[1] = max(-0.15 * G, min(0.15 * G, uhat[1]))
        # ② 迭代: 用场地实测落地距离调过载幅值 (无需知道真实气动)
        dx_imp = R["impact_x"] - x_base
        n_pull = max(ADAPT["nmin"], min(ADAPT["nmax"],
                                        n_pull + ADAPT["gain_n"] * (DX_TARGET - dx_imp)))
        print(f"{it:>3} {n_pull:6.2f} | {R['pk_rel']:9.2f} {dx_imp:7.2f} {DX_TARGET:6.1f} | "
              f"{R['gam_emax']/DEG:8.2f} {abs(R['peak_hhat'] - R['peak_h']):7.2f} "
              f"{R['x_emax']:7.2f} {R['n_rms']:9.3f}")
    print("\n注: Δh_rel=拉升轨迹相对无舵基线弹道同刻高度差的峰值(拉升增益); "
          "|Δγ̂|/估差为推算相对真值偏差; 现场用实测落地距离与IMU日志.")


if __name__ == "__main__":
    main()
