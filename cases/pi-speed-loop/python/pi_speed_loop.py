#!/usr/bin/env python3
"""直流电机 PI 转速环（单环，教学演示）。

电机沿用 dc-motor-open-loop 卡（他励励磁恒定 / 永磁，Ke = Kt = K）::

    La dia/dt = Va − Ra ia − K ω
    J  dω/dt  = K ia − B ω − T_L

转速 PI 直接给出电枢电压（理想可控电压源，0…220 V 限幅）::

    e  = ω_ref − ω
    Va = sat(Kp·e + z),  dz/dt = Ki·e（积分项）

抗积分饱和用「钳位」：输出已顶到上限且误差仍为正（或顶到下限且误差为负）时，
积分器停止积分。

工况 1（主图）：转速给定 0 → 1500 r/min 斜坡 1 s，t = 1.5 s 突加 T_L = 20 N·m，
算到 3 s；对比三组增益
    P   : Kp = 4,  Ki = 0
    PI-A: Kp = 4,  Ki = 45   （积分零点约消去电机慢极点，ζ ≈ 0.70）
    PI-B: Kp = 4,  Ki = 200  （积分过强，ζ ≈ 0.29，振荡）
工况 2（对照）：给定直接阶跃到 1500 r/min，用 PI-A，比较有 / 无抗饱和。

数值解用 solve_ivp（RK45），在给定 / 负载跳变处分段积分；用 Radau 再算一遍核对，
并与稳态解析值比对，偏差超限就报错退出（返回非零）。

用法（在仓库根目录或本目录均可）::

    python cases/pi-speed-loop/python/pi_speed_loop.py

无图形界面时只保存图片、不弹窗。依赖见 requirements.txt。
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    import matplotlib

    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from scipy.integrate import solve_ivp

# ---- 电机（与 dc-motor-open-loop 相同）----
RA = 0.5  # Ω
LA = 0.005  # H
K = 1.0  # V·s/rad = N·m/A
J = 0.2  # kg·m²
B = 0.005  # N·m·s/rad
I_N = 20.0  # A，示例额定电流
# ---- 控制与工况 ----
V_MAX = 220.0  # V，电压上限（直流母线 / 额定电压）
V_MIN = 0.0  # V，不可逆单象限，下限 0
N_REF = 1500.0  # r/min
T_RAMP = 1.0  # s，斜坡时间（工况 1）
TL = 20.0  # N·m
T_LOAD = 1.5  # s
T_END = 3.0  # s
BAND_RPM = 2.0  # r/min，恢复时间判据
GAINS = {"P": (4.0, 0.0), "PI-A": (4.0, 45.0), "PI-B": (4.0, 200.0)}
TOL_W = 0.01  # rad/s，RK45 与 Radau 允许最大偏差
TOL_SS = 0.01  # rad/s，末值与稳态解析值允许偏差

RPM = 60.0 / (2.0 * math.pi)
W_REF = N_REF / RPM


def w_ref(t: float, ramp: bool) -> float:
    if not ramp:
        return W_REF
    return W_REF * min(max(t / T_RAMP, 0.0), 1.0)


def load(t: float) -> float:
    return TL if t >= T_LOAD else 0.0


def control(t, x, kp, ki, ramp):
    """返回 (Va, 未限幅输出, 误差)。"""
    _, w, z = x
    e = w_ref(t, ramp) - w
    u = kp * e + z
    return min(max(u, V_MIN), V_MAX), u, e


def rhs_factory(kp, ki, ramp, anti_windup, tl_on):
    def rhs(t, x):
        ia, w, _ = x
        va, u, e = control(t, x, kp, ki, ramp)
        dz = ki * e
        if anti_windup and ((u > V_MAX and e > 0) or (u < V_MIN and e < 0)):
            dz = 0.0
        tl = TL if tl_on else 0.0
        return [
            (va - RA * ia - K * w) / LA,
            (K * ia - B * w - tl) / J,
            dz,
        ]

    return rhs


def simulate(kp, ki, ramp, anti_windup=True, method="RK45"):
    """分段积分（在 T_RAMP、T_LOAD 处断开），返回 t、x(3×N)、Va。"""
    edges = [0.0] + ([T_RAMP] if ramp else []) + [T_LOAD, T_END]
    ts, xs = [], []
    x0 = np.zeros(3)
    for k in range(len(edges) - 1):
        a, b = edges[k], edges[k + 1]
        n = int(round((b - a) / 1e-4)) + 1
        t_eval = np.linspace(a, b, n)
        sol = solve_ivp(
            rhs_factory(kp, ki, ramp, anti_windup, a >= T_LOAD),
            (a, b),
            x0,
            t_eval=t_eval,
            method=method,
            rtol=1e-8,
            atol=1e-8,
            max_step=1e-4 if method == "RK45" else 2e-4,
        )
        if not sol.success:
            raise RuntimeError(f"solve_ivp 失败: {sol.message}")
        x0 = sol.y[:, -1].copy()
        if k > 0:
            ts.append(sol.t[1:])
            xs.append(sol.y[:, 1:])
        else:
            ts.append(sol.t)
            xs.append(sol.y)
    t = np.concatenate(ts)
    x = np.concatenate(xs, axis=1)
    va = np.array([control(tt, x[:, i], kp, ki, ramp)[0] for i, tt in enumerate(t)])
    return t, x, va


def closed_loop_poles(kp, ki):
    a = np.array(
        [[-RA / LA, -(K + kp) / LA, 1.0 / LA], [K / J, -B / J, 0.0], [0.0, -ki, 0.0]]
    )
    return np.linalg.eigvals(a)


def steady_loaded(kp, ki):
    """带载稳态（解析）：返回 ω、ia、Va。"""
    if ki > 0:  # 有积分：无静差
        w = W_REF
    else:  # 纯 P：Kp(ωr − ω) = Ra ia + K ω，K ia = TL + B ω
        w = (kp * W_REF - RA * TL / K) / (kp + K + RA * B / K)
    ia = (TL + B * w) / K
    return w, ia, RA * ia + K * w


def steady_noload_p(kp):
    return (kp * W_REF) / (kp + K + RA * B / K)


def recovery_time(t, n, n_target):
    """t ≥ T_LOAD 之后最后一次超出 ±BAND 的时刻 − T_LOAD；始终在带内返回 0。"""
    m = t >= T_LOAD
    out = np.abs(n[m] - n_target) > BAND_RPM
    if not out.any():
        return 0.0
    idx = np.nonzero(out)[0][-1]
    if idx == out.size - 1:
        return math.inf
    return float(t[m][idx + 1] - T_LOAD)


def _enable_cjk() -> None:
    for path in (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    ):
        if Path(path).is_file():
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            return


def _show(key: str, v: float) -> None:
    if key == "rk45_vs_radau_rad_s":
        print(f"  {key:<26}= {v:.1e}")
    else:
        print(f"  {key:<26}= {0.0 if abs(v) < 5e-4 else v:.3f}")


def main() -> int:
    print("DC motor PI speed loop (single loop, Va limited to 0..220 V)")
    print(f"  {'n_ref_rpm':<26}= {N_REF:.3f}")
    print(f"  {'w_ref_rad_s':<26}= {W_REF:.6f}")
    print(f"  {'open_loop_drop_rpm':<26}= {RA * TL / (K * K + RA * B) * RPM:.3f}")

    res = {}
    ok = True
    for name, (kp, ki) in GAINS.items():
        t, x, va = simulate(kp, ki, ramp=True)
        _, x2, _ = simulate(kp, ki, ramp=True, method="Radau")
        err = float(np.max(np.abs(x2[1] - x[1])))
        n = x[1] * RPM
        p = closed_loop_poles(kp, ki)
        cpx = p[np.abs(p.imag) > 1e-9]
        zeta = float(-cpx[0].real / abs(cpx[0])) if cpx.size else 1.0
        w_ss, ia_ss, va_ss = steady_loaded(kp, ki)
        before = (t >= 1.2) & (t < T_LOAD)
        after = t >= T_LOAD
        n_target = N_REF if ki > 0 else w_ss * RPM
        rows = {
            "zeta": zeta,
            "n_before_load_rpm": float(n[np.searchsorted(t, T_LOAD) - 1]),
            "n_ss_loaded_rpm": w_ss * RPM,
            "n_end_rpm": float(n[-1]),
            "dip_rpm": float(n[np.searchsorted(t, T_LOAD) - 1] - n[after].min()),
            "t_recover_ms": recovery_time(t, n, n_target) * 1e3,
            "hold_max_minus_ref_rpm": float(n[(t >= T_RAMP) & (t < T_LOAD)].max() - N_REF),
            "ia_max_A": float(x[0].max()),
            "ia_ss_A": ia_ss,
            "va_ss_V": va_ss,
            "va_max_V": float(va.max()),
            "rk45_vs_radau_rad_s": err,
        }
        if ki == 0:
            rows["n_ss_noload_rpm"] = steady_noload_p(kp) * RPM
        if err > TOL_W or abs(x[1, -1] - w_ss) > TOL_SS:
            ok = False
        if va.max() >= V_MAX - 1e-9:
            print(f"  !! {name}: 工况 1 碰到电压上限，与卡片假设不符")
            ok = False
        res[name] = (t, x, va, rows)
        print(f"[{name}] Kp = {kp:g} V·s/rad, Ki = {ki:g} V/rad")
        print(f"  {'poles':<26}= " + ", ".join(f"{z:.3f}" for z in sorted(p, key=lambda z: z.real)))
        for k, v in rows.items():
            _show(k, v)

    # 工况 2：阶跃给定，有 / 无抗积分饱和
    kp, ki = GAINS["PI-A"]
    step = {}
    for aw in (True, False):
        t, x, va = simulate(kp, ki, ramp=False, anti_windup=aw)
        _, x2, _ = simulate(kp, ki, ramp=False, anti_windup=aw, method="Radau")
        err = float(np.max(np.abs(x2[1] - x[1])))
        n = x[1] * RPM
        sat = t[va >= V_MAX - 1e-9]
        rows = {
            "ia_max_A": float(x[0].max()),
            "ia_max_over_In": float(x[0].max()) / I_N,
            "t_sat_end_ms": float(sat.max() * 1e3) if sat.size else 0.0,
            "n_peak_rpm": float(n[t < T_LOAD].max()),
            "overshoot_rpm": float(n[t < T_LOAD].max() - N_REF),
            "n_end_rpm": float(n[-1]),
            "rk45_vs_radau_rad_s": err,
        }
        if err > TOL_W or abs(x[1, -1] - W_REF) > TOL_SS:
            ok = False
        label = "step+AW" if aw else "step-noAW"
        step[label] = (t, x, va)
        print(f"[{label}] PI-A, 给定阶跃到 {N_REF:g} r/min，抗饱和 = {aw}")
        for k, v in rows.items():
            _show(k, v)

    # ---- 作图 ----
    _enable_cjk()
    fig, ax = plt.subplots(2, 2, figsize=(12, 8), sharex="col")
    colors = {"P": "tab:gray", "PI-A": "tab:blue", "PI-B": "tab:red"}
    for name, (t, x, va, _) in res.items():
        kp, ki = GAINS[name]
        lab = f"{name}（Kp={kp:g}, Ki={ki:g}）"
        ax[0, 0].plot(t, x[1] * RPM, color=colors[name], lw=1.2, label=lab)
        ax[1, 0].plot(t, x[0], color=colors[name], lw=1.2, label=lab)
    tr = np.linspace(0, T_END, 601)
    ax[0, 0].plot(tr, [w_ref(v, True) * RPM for v in tr], "k--", lw=0.8, label="给定")
    ax[0, 0].set_title("工况 1：斜坡给定 + 1.5 s 突加 20 N·m")
    ax[0, 0].set_ylabel("n / (r/min)")
    ax[0, 0].set_ylim(0, 1700)
    ax[0, 0].legend(loc="lower right", fontsize=8)
    ax[1, 0].set_ylabel("ia / A")
    ax[1, 0].set_xlabel("t / s")
    for a in ax[:, 0]:
        a.grid(True, alpha=0.3)
    ins = ax[0, 0].inset_axes([0.36, 0.08, 0.28, 0.36])
    for name, (t, x, _, _) in res.items():
        m = (t > 1.4) & (t < 2.2)
        ins.plot(t[m], x[1, m] * RPM, color=colors[name], lw=1)
    ins.axhline(N_REF, color="k", ls="--", lw=0.6)
    ins.set_ylim(1470, 1525)
    ins.set_title("加载局部", fontsize=8)
    ins.tick_params(labelsize=7)
    for label, (t, x, va) in step.items():
        c = "tab:blue" if label == "step+AW" else "tab:orange"
        txt = "有抗饱和（钳位）" if label == "step+AW" else "无抗饱和"
        ax[0, 1].plot(t, x[1] * RPM, color=c, lw=1.2, label=txt)
        ax[1, 1].plot(t, x[0], color=c, lw=1.2, label=txt)
    ax[0, 1].axhline(N_REF, color="k", ls="--", lw=0.8)
    ax[0, 1].set_title("工况 2：给定阶跃（PI-A），电压 0–220 V 限幅")
    ax[0, 1].legend(loc="lower right", fontsize=8)
    ax[1, 1].set_xlabel("t / s")
    ax[0, 1].set_ylabel("n / (r/min)")
    ax[1, 1].set_ylabel("ia / A")
    ax[1, 1].axhline(I_N, color="k", ls=":", lw=0.8)
    ax[1, 1].text(2.2, I_N + 8, "额定 20 A", fontsize=8)
    for a in ax[:, 1]:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    out = Path(__file__).resolve().parent.parent / "img" / "pi_speed_loop_curves.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=140)
    print(f"  {'figure':<26}= {out}")
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        plt.show()

    if not ok:
        print("校验失败：数值解偏差或稳态值超限", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
