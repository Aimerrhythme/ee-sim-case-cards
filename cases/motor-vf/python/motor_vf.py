#!/usr/bin/env python3
"""异步电机恒压频比（V/f）与低频电压提升（教学演示）。

示例电机：4 极（极对数 2）、Y 接、380 V / 50 Hz 量级。参数是教科书上的
圆整数，不是某一台电机的铭牌。

电气侧用静止 αβ 坐标系、幅值不变的 Clarke 变换。状态是

    θ, i_sα, i_sβ, ψ_rα, ψ_rβ, ω_m

电压按频率指令生成理想正弦（不含逆变器斩波）：

    dθ/dt = 2π f
    v_sα = √2 · V_ph(f) · cos θ
    v_sβ = √2 · V_ph(f) · sin θ

无补偿时相电压有效值过原点：V_ph = V_ph,n · f / f_n。
有低频提升时，零频处抬高 V_boost，额定频率仍回到额定电压：

    V_ph = V_boost + (V_ph,n − V_boost) · f / f_n     （f ≤ f_n）

转子磁链和定子电流：

    dψ_rα/dt = −ψ_rα/τ_r − ω_r ψ_rβ + (L_m/τ_r) i_sα
    dψ_rβ/dt = −ψ_rβ/τ_r + ω_r ψ_rα + (L_m/τ_r) i_sβ
    σ L_s di_sα/dt = v_sα − R_s i_sα − (L_m/L_r) dψ_rα/dt
    σ L_s di_sβ/dt = v_sβ − R_s i_sβ − (L_m/L_r) dψ_rβ/dt

    τ_r = L_r / R_r
    σ   = 1 − L_m² / (L_s L_r)
    ω_r = n_p ω_m
    T_e = (3/2) n_p (L_m/L_r) (ψ_rα i_sβ − ψ_rβ i_sα)
    J dω_m/dt = T_e − T_L

稳态对照用 T 型等效电路（转子量已折算到定子）。主工况是不带提升的
斜坡到 50 Hz，空载后再突加 15 N·m。低速对照把频率定在 5 Hz、负载 8 N·m，
比较有无 V_boost。

积分是定步长 RK4，步长 0.2 ms。scipy 用来做两件对照：brentq 核对滑差，
solve_ivp 核对 50 Hz 堵转在稳态转速上的电磁转矩。打印的数字来自 RK4
和等效电路，MATLAB 脚本用同一套循环，应逐行相同。

用法（在仓库根目录或本目录均可）::

    python cases/motor-vf/python/motor_vf.py

无图形界面时只保存图片、不弹窗。依赖见 requirements.txt。
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

# 无 DISPLAY 时必须在导入 pyplot 之前选定非交互后端。
if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    import matplotlib

    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

NP = 2  # 极对数。4 极，同步转速 1500 r/min，不是 3000
FN = 50.0  # Hz
VLL = 380.0  # V，线电压有效值
VPH = VLL / math.sqrt(3.0)  # 相电压有效值
RS = 2.2  # Ω
RR = 1.8  # Ω，已折算到定子
LLS = 0.012  # H
LLR = 0.012  # H
LM = 0.24  # H
J = 0.04  # kg·m²
V_BOOST = 25.0  # V，相电压有效值，f = 0 处的提升
F_LOW = 5.0  # Hz
TL = 15.0  # N·m，50 Hz 稳态负载
TL_LOW = 8.0  # N·m，5 Hz 对照负载
T_RAMP = 1.5  # s，频率从 0 升到 50 Hz
T_LOAD = 2.2  # s，突加额定负载
T_END = 3.0  # s，主工况停止
T_LOW_ON = 0.4  # s，低速对照加上负载
T_LOW_END = 1.6  # s
H = 2e-4  # s，RK4 步长

LS = LLS + LM
LR = LLR + LM
SIGMA = 1.0 - (LM * LM) / (LS * LR)
TAU_R = LR / RR
K_PSI = LM / LR
LSIG = SIGMA * LS

# 读数窗口，两端都落在步长上。
T_NOLOAD_0 = 2.0
T_NOLOAD_1 = 2.2
T_LOAD_0 = 2.7
T_LOAD_1 = 3.0
T_LOW_0 = 1.3
T_LOW_1 = 1.6

# 等效电路相对 RK4 的允许偏差。机械窗口里还剩一点振荡，不是步长把稳态算偏了。
TOL_N_NOLOAD = 0.05  # r/min
TOL_N = 0.3  # r/min
TOL_TE = 0.05  # N·m
TOL_I = 0.02  # A
TOL_SLIP = 1e-8
TOL_IVP = 2e-3  # solve_ivp 相对 RK4 堵转转矩


def voltage_rms(freq: float, boost: bool) -> float:
    """相电压有效值。f ≥ f_n 时钳在额定，本卡的指令到不了那里。"""
    frac = freq / FN
    if frac > 1.0:
        frac = 1.0
    if boost:
        return V_BOOST + (VPH - V_BOOST) * frac
    return VPH * frac


def schedule(t: float, mode: str) -> tuple[float, float, bool]:
    """频率指令、负载转矩、这一次运行开不开低频提升。"""
    if mode == "rated":
        freq = FN * t / T_RAMP if t < T_RAMP else FN
        load = TL if t >= T_LOAD else 0.0
        return freq, load, False
    if mode == "low0":
        load = TL_LOW if t >= T_LOW_ON else 0.0
        return F_LOW, load, False
    if mode == "low1":
        load = TL_LOW if t >= T_LOW_ON else 0.0
        return F_LOW, load, True
    if mode == "lock":
        # 50 Hz、额定电压、空载。转速由调用者钉住，用来和等效电路对转矩。
        return FN, 0.0, False
    raise ValueError(mode)


def deriv(t: float, x: list[float] | np.ndarray, mode: str) -> tuple[tuple[float, ...], float]:
    theta = float(x[0])
    isa = float(x[1])
    isb = float(x[2])
    psira = float(x[3])
    psirb = float(x[4])
    wm = float(x[5])
    freq, load, boost = schedule(t, mode)
    wr = NP * wm
    vm = math.sqrt(2.0) * voltage_rms(freq, boost)
    vsa = vm * math.cos(theta)
    vsb = vm * math.sin(theta)
    dpr = (-psira / TAU_R) - wr * psirb + (LM / TAU_R) * isa
    dpb = (-psirb / TAU_R) + wr * psira + (LM / TAU_R) * isb
    dia = (vsa - RS * isa - K_PSI * dpr) / LSIG
    dib = (vsb - RS * isb - K_PSI * dpb) / LSIG
    te = 1.5 * NP * K_PSI * (psira * isb - psirb * isa)
    dw = (te - load) / J
    return (2.0 * math.pi * freq, dia, dib, dpr, dpb, dw), te


def steady(freq: float, boost: bool, slip: float) -> tuple[float, float]:
    """T 型等效电路。返回定子相电流有效值、电磁转矩。slip ≤ 0 当空载。"""
    w = 2.0 * math.pi * freq
    xs = w * LLS
    xr = w * LLR
    xm = w * LM
    volt = voltage_rms(freq, boost)
    if slip <= 0.0:
        zin = complex(RS, xs + xm)
        return abs(volt / zin), 0.0
    zr = complex(RR / slip, xr)
    zm = complex(0.0, xm)
    zpar = 1.0 / (1.0 / zm + 1.0 / zr)
    zin = complex(RS, xs) + zpar
    ist = volt / zin
    ir = ist * zm / (zm + zr)
    pag = 3.0 * (abs(ir) ** 2) * (RR / slip)
    te = pag / (w / NP)
    return abs(ist), te


def torque_max(freq: float, boost: bool) -> tuple[float, float]:
    """临界滑差和最大转矩。转矩对滑差只有一个峰，三分法搜。"""
    lo = 1e-6
    hi = 1.0
    for _ in range(80):
        m1 = lo + (hi - lo) / 3.0
        m2 = hi - (hi - lo) / 3.0
        t1 = steady(freq, boost, m1)[1]
        t2 = steady(freq, boost, m2)[1]
        if t1 < t2:
            lo = m1
        else:
            hi = m2
    slip = 0.5 * (lo + hi)
    return slip, steady(freq, boost, slip)[1]


def slip_for_torque(freq: float, boost: bool, load: float, slip_bd: float) -> float:
    """稳定支（滑差小于临界滑差）上，转矩等于负载的那个滑差。"""
    lo = 0.0
    hi = slip_bd
    for _ in range(70):
        mid = 0.5 * (lo + hi)
        if steady(freq, boost, mid)[1] < load:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def rk4_case(mode: str, t_end: float) -> dict[str, np.ndarray]:
    n = int(round(t_end / H))
    x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    t_out = np.empty(n)
    n_out = np.empty(n)
    te_out = np.empty(n)
    i_out = np.empty(n)
    f_out = np.empty(n)
    for i in range(n):
        t = i * H
        k1, _te = deriv(t, x, mode)
        x2 = [x[j] + 0.5 * H * k1[j] for j in range(6)]
        k2, _te = deriv(t + 0.5 * H, x2, mode)
        x3 = [x[j] + 0.5 * H * k2[j] for j in range(6)]
        k3, _te = deriv(t + 0.5 * H, x3, mode)
        x4 = [x[j] + H * k3[j] for j in range(6)]
        k4, _te = deriv(t + H, x4, mode)
        coef = H / 6.0
        x = [x[j] + coef * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]) for j in range(6)]
        ts = t + H
        te = 1.5 * NP * K_PSI * (x[3] * x[2] - x[4] * x[1])
        irms = math.hypot(x[1], x[2]) / math.sqrt(2.0)
        freq, _load, _boost = schedule(ts, mode)
        t_out[i] = ts
        n_out[i] = x[5] * 60.0 / (2.0 * math.pi)
        te_out[i] = te
        i_out[i] = irms
        f_out[i] = freq
    return {"t": t_out, "n": n_out, "te": te_out, "i": i_out, "f": f_out}


def window_mean(t: np.ndarray, y: np.ndarray, t0: float, t1: float) -> float:
    mask = (t >= t0) & (t < t1)
    return float(np.mean(y[mask]))


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


def plot_case(
    rated: dict[str, np.ndarray],
    low0: dict[str, np.ndarray],
    low1: dict[str, np.ndarray],
    n_off: float,
    n_on: float,
    out: Path,
) -> None:
    _enable_cjk()
    fig, axes = plt.subplots(3, 2, figsize=(9.6, 9.2))

    ax = axes[0, 0]
    ax.plot(rated["t"], rated["n"], color="#1f4e79", lw=1.4, label="转速")
    ax.plot(rated["t"], rated["f"] * 60.0 / NP, color="#9ca3af", lw=1.0, ls="--", label="同步转速")
    ax.axvline(T_LOAD, color="#e07a3d", lw=0.8, ls=":", label="突加负载")
    ax.set_xlim(0.0, T_END)
    ax.set_ylim(-40.0, 1680.0)
    ax.set_ylabel("转速 / (r/min)")
    ax.set_title("转速    无提升斜坡到 50 Hz，2.2 s 加 15 N·m")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right", frameon=False, fontsize=8)

    ax = axes[0, 1]
    ax.plot(rated["t"], rated["te"], color="#1f4e79", lw=1.2, label="电磁转矩")
    load_line = np.where(rated["t"] >= T_LOAD, TL, 0.0)
    ax.plot(rated["t"], load_line, color="#e07a3d", lw=1.0, ls="--", label="负载转矩")
    ax.set_xlim(0.0, T_END)
    ax.set_ylim(-4.0, 28.0)
    ax.set_ylabel("转矩 / (N·m)")
    ax.set_title("电磁转矩    空载只负责加速，稳态等于负载")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[1, 0]
    ax.plot(rated["t"], rated["i"], color="#1f4e79", lw=1.3, label="相电流有效值")
    ax.axhline(window_mean(rated["t"], rated["i"], T_NOLOAD_0, T_NOLOAD_1), color="#9ca3af", lw=0.8, ls="--", label="空载")
    ax.axhline(window_mean(rated["t"], rated["i"], T_LOAD_0, T_LOAD_1), color="#e07a3d", lw=0.8, ls="--", label="带载")
    ax.set_xlim(0.0, T_END)
    ax.set_ylim(0.0, 8.0)
    ax.set_ylabel("电流 / A")
    ax.set_title("定子电流    矢量模 / √2，平衡时即相电流有效值")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[1, 1]
    freq = np.linspace(0.0, FN, 200)
    v_off = np.array([voltage_rms(f, False) * math.sqrt(3.0) for f in freq])
    v_on = np.array([voltage_rms(f, True) * math.sqrt(3.0) for f in freq])
    ax.plot(freq, v_off, color="#1f4e79", lw=1.5, label="无补偿")
    ax.plot(freq, v_on, color="#e07a3d", lw=1.5, label="低频提升")
    for f_mark, boost, color in ((F_LOW, False, "#1f4e79"), (F_LOW, True, "#e07a3d"), (FN, False, "#1f4e79")):
        ax.plot(f_mark, voltage_rms(f_mark, boost) * math.sqrt(3.0), "o", color=color, ms=4.5)
    ax.set_xlim(0.0, FN)
    ax.set_ylim(0.0, 420.0)
    ax.set_xlabel("频率 f / Hz")
    ax.set_ylabel("线电压有效值 / V")
    ax.set_title("V/f    50 Hz 两条线都回到 380 V")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper left", frameon=False, fontsize=8)

    ax = axes[2, 0]
    ax.plot(low0["t"], low0["n"], color="#1f4e79", lw=1.4, label="无补偿")
    ax.plot(low1["t"], low1["n"], color="#e07a3d", lw=1.4, label="低频提升")
    ax.axhline(F_LOW * 60.0 / NP, color="#9ca3af", lw=0.9, ls="--", label="同步 150 r/min")
    ax.axvline(T_LOW_ON, color="#5b8a72", lw=0.8, ls=":", label="加上 8 N·m")
    ax.set_xlim(0.0, T_LOW_END)
    ax.set_ylim(-10.0, 200.0)
    ax.set_xlabel("时间 t / s")
    ax.set_ylabel("转速 / (r/min)")
    ax.set_title("低速    5 Hz，同一负载，提升把滑差收回来")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[2, 1]
    slips = np.linspace(0.004, 1.0, 500)
    for boost, color, name in ((False, "#1f4e79", "无补偿"), (True, "#e07a3d", "低频提升")):
        torque = np.array([steady(F_LOW, boost, s)[1] for s in slips])
        speed = (1.0 - slips) * (F_LOW * 60.0 / NP)
        ax.plot(speed, torque, color=color, lw=1.4, label=name)
    ax.axhline(TL_LOW, color="#5b8a72", lw=1.0, ls="--", label="8 N·m")
    ax.plot([n_off], [TL_LOW], "o", color="#1f4e79", ms=5)
    ax.plot([n_on], [TL_LOW], "o", color="#e07a3d", ms=5)
    ax.set_xlim(0.0, 160.0)
    ax.set_ylim(0.0, 46.0)
    ax.set_xlabel("转速 / (r/min)")
    ax.set_ylabel("电磁转矩 / (N·m)")
    ax.set_title("低速转矩    峰值在同一滑差，高度按电压平方")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    fig.suptitle(
        r"异步电机 V/f    4 极示例    380 V / 50 Hz    $V_{boost}=25\,\mathrm{V}$（相）",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def scipy_checks(slip_load: float, slip_bd: float) -> list[str]:
    """brentq 核对二分滑差；转速锁在稳态点上，solve_ivp 核对 RK4 转矩。"""
    problems: list[str] = []
    slip_b = brentq(lambda s: steady(FN, False, s)[1] - TL, 1e-6, slip_bd * 0.999)
    if abs(slip_b - slip_load) > TOL_SLIP:
        problems.append("二分滑差和 brentq 不一致")

    wm = (1.0 - slip_load) * 2.0 * math.pi * FN / NP

    def fun(t: float, y: np.ndarray) -> list[float]:
        # 频率、电压按额定、不提升；转速锁在等效电路的稳态上。
        y = np.array(y, dtype=float)
        y[5] = wm
        d, _te = deriv(t, y, "lock")
        return [d[0], d[1], d[2], d[3], d[4], 0.0]

    sol = solve_ivp(
        fun,
        (0.0, 0.8),
        [0.0, 0.0, 0.0, 0.0, 0.0, wm],
        method="RK45",
        rtol=1e-7,
        atol=1e-9,
        dense_output=True,
        max_step=H,
    )
    if not sol.success:
        problems.append("solve_ivp 没有积到稳态")
        return problems
    ts = np.linspace(0.6, 0.8, 400)
    y = sol.sol(ts)
    te = 1.5 * NP * K_PSI * (y[3] * y[2] - y[4] * y[1])
    # 同一锁速工况再用 RK4 走一遍，避免和「自由转速」的主工况混在一起。
    te_rk = _rk4_locked(wm)
    if abs(float(np.mean(te)) - TL) / TL > TOL_IVP:
        problems.append("solve_ivp 堵转转矩偏离等效电路")
    if te_rk <= 0.0 or abs(te_rk - float(np.mean(te))) / TL > TOL_IVP:
        problems.append("RK4 与 solve_ivp 的堵转转矩不一致")
    return problems


def _rk4_locked(wm: float) -> float:
    """把转速钉在 wm 上积分 0.8 s，返回最后 0.2 s 的平均转矩。"""
    x = [0.0, 0.0, 0.0, 0.0, 0.0, wm]
    n = int(round(0.8 / H))
    i0 = int(round(0.6 / H))
    acc = 0.0
    m = 0
    for i in range(n):
        t = i * H

        def der_locked(xx: list[float], tt: float) -> tuple[float, ...]:
            yy = list(xx)
            yy[5] = wm
            d, _te = deriv(tt, yy, "lock")
            return (d[0], d[1], d[2], d[3], d[4], 0.0)

        k1 = der_locked(x, t)
        x2 = [x[j] + 0.5 * H * k1[j] for j in range(6)]
        k2 = der_locked(x2, t + 0.5 * H)
        x3 = [x[j] + 0.5 * H * k2[j] for j in range(6)]
        k3 = der_locked(x3, t + 0.5 * H)
        x4 = [x[j] + H * k3[j] for j in range(6)]
        k4 = der_locked(x4, t + H)
        coef = H / 6.0
        x = [x[j] + coef * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]) for j in range(6)]
        x[5] = wm
        if i >= i0:
            te = 1.5 * NP * K_PSI * (x[3] * x[2] - x[4] * x[1])
            acc += te
            m += 1
    return acc / float(m)


def _line(key: str, value: float) -> str:
    text = f"{value:.6f}"
    if abs(value) < 5e-7:
        text = "0.000000"
    return f"  {key:<26} = {text}"


def main() -> int:
    rated = rk4_case("rated", T_END)
    low0 = rk4_case("low0", T_LOW_END)
    low1 = rk4_case("low1", T_LOW_END)

    i0_th, te0 = steady(FN, False, 0.0)
    i_start, te_start = steady(FN, False, 1.0)
    slip_bd, te_max = torque_max(FN, False)
    slip_load = slip_for_torque(FN, False, TL, slip_bd)
    i_load, te_load = steady(FN, False, slip_load)
    n_load = (1.0 - slip_load) * FN * 60.0 / NP
    wm_load = (1.0 - slip_load) * 2.0 * math.pi * FN / NP
    p_shaft = TL * wm_load

    slip_bd_low, te_max_low = torque_max(F_LOW, False)
    _slip_bd_on, te_max_on = torque_max(F_LOW, True)
    slip_off = slip_for_torque(F_LOW, False, TL_LOW, slip_bd_low)
    slip_on = slip_for_torque(F_LOW, True, TL_LOW, slip_bd_low)
    i_off, _te_off = steady(F_LOW, False, slip_off)
    i_on, _te_on = steady(F_LOW, True, slip_on)
    n_off = (1.0 - slip_off) * F_LOW * 60.0 / NP
    n_on = (1.0 - slip_on) * F_LOW * 60.0 / NP

    n_noload = window_mean(rated["t"], rated["n"], T_NOLOAD_0, T_NOLOAD_1)
    i_noload = window_mean(rated["t"], rated["i"], T_NOLOAD_0, T_NOLOAD_1)
    n_meas = window_mean(rated["t"], rated["n"], T_LOAD_0, T_LOAD_1)
    te_meas = window_mean(rated["t"], rated["te"], T_LOAD_0, T_LOAD_1)
    i_meas = window_mean(rated["t"], rated["i"], T_LOAD_0, T_LOAD_1)
    n_m0 = window_mean(low0["t"], low0["n"], T_LOW_0, T_LOW_1)
    n_m1 = window_mean(low1["t"], low1["n"], T_LOW_0, T_LOW_1)
    te_m0 = window_mean(low0["t"], low0["te"], T_LOW_0, T_LOW_1)
    te_m1 = window_mean(low1["t"], low1["te"], T_LOW_0, T_LOW_1)
    i_m0 = window_mean(low0["t"], low0["i"], T_LOW_0, T_LOW_1)
    i_m1 = window_mean(low1["t"], low1["i"], T_LOW_0, T_LOW_1)

    vph_off = voltage_rms(F_LOW, False)
    vph_on = voltage_rms(F_LOW, True)
    rows = [
        ("np_pairs", float(NP)),
        ("f_rated_Hz", FN),
        ("Vll_rated_V", VLL),
        ("Vph_rated_V", VPH),
        ("Vph_peak_V", VPH * math.sqrt(2.0)),
        ("Rs_ohm", RS),
        ("Rr_ohm", RR),
        ("Lls_H", LLS),
        ("Llr_H", LLR),
        ("Lm_H", LM),
        ("Ls_H", LS),
        ("Lr_H", LR),
        ("sigma", SIGMA),
        ("tau_r_s", TAU_R),
        ("J_kgm2", J),
        ("Vboost_V", V_BOOST),
        ("Vll_boost0_V", V_BOOST * math.sqrt(3.0)),
        ("I_boost_preset_A", V_BOOST / RS),
        ("vf_line_V_per_Hz", VLL / FN),
        ("n_sync_rpm", FN * 60.0 / NP),
        ("I0_theory_A", i0_th),
        ("Tl_Nm", TL),
        ("s_bd_rated", slip_bd),
        ("Temax_rated_Nm", te_max),
        ("s_load", slip_load),
        ("n_load_theory_rpm", n_load),
        ("I_load_theory_A", i_load),
        ("P_shaft_W", p_shaft),
        ("n_meas_noload_rpm", n_noload),
        ("I_meas_noload_A", i_noload),
        ("n_meas_load_rpm", n_meas),
        ("Te_meas_load_Nm", te_meas),
        ("I_meas_load_A", i_meas),
        ("f_low_Hz", F_LOW),
        ("n_sync_low_rpm", F_LOW * 60.0 / NP),
        ("Tl_low_Nm", TL_LOW),
        ("Vph_low_off_V", vph_off),
        ("Vph_low_on_V", vph_on),
        ("Vll_low_off_V", vph_off * math.sqrt(3.0)),
        ("Vll_low_on_V", vph_on * math.sqrt(3.0)),
        ("s_bd_low", slip_bd_low),
        ("Temax_low_off_Nm", te_max_low),
        ("Temax_low_on_Nm", te_max_on),
        ("Temax_ratio_low", te_max_on / te_max_low),
        ("V_ratio_sq_low", (vph_on / vph_off) ** 2),
        ("I_start_50_A", i_start),
        ("Te_start_50_Nm", te_start),
        ("s_low_off", slip_off),
        ("n_low_off_theory_rpm", n_off),
        ("I_low_off_theory_A", i_off),
        ("s_low_on", slip_on),
        ("n_low_on_theory_rpm", n_on),
        ("I_low_on_theory_A", i_on),
        ("n_meas_low_off_rpm", n_m0),
        ("n_meas_low_on_rpm", n_m1),
        ("Te_meas_low_off_Nm", te_m0),
        ("Te_meas_low_on_Nm", te_m1),
        ("I_meas_low_off_A", i_m0),
        ("I_meas_low_on_A", i_m1),
        ("dn_boost_rpm", n_m1 - n_m0),
        ("abs_err_noload_rpm", n_noload - FN * 60.0 / NP),
        ("abs_err_n_load_rpm", n_meas - n_load),
        ("abs_err_n_low_off_rpm", n_m0 - n_off),
        ("abs_err_n_low_on_rpm", n_m1 - n_on),
        ("rel_err_n_load", (n_meas - n_load) / n_load),
        ("rel_err_I_load", (i_meas - i_load) / i_load),
        ("rel_err_n_low_off", (n_m0 - n_off) / n_off),
        ("rel_err_n_low_on", (n_m1 - n_on) / n_on),
    ]

    print("Induction motor V/f")
    for key, value in rows:
        print(_line(key, value))

    out = Path(__file__).resolve().parent.parent / "img" / "motor_vf_curves.png"
    plot_case(rated, low0, low1, n_off, n_on, out)
    print(f"  {'figure':<26} = {out}")

    problems: list[str] = []
    if abs(te0) > 0.0:
        problems.append("空载等效电路转矩应为 0")
    if abs(te_load - TL) / TL > 1e-6:
        problems.append("等效电路没有把负载转矩配平")
    if abs(n_noload - 1500.0) > TOL_N_NOLOAD:
        problems.append("空载转速没有落到 1500 r/min")
    if abs(n_meas - n_load) > TOL_N:
        problems.append("带载转速偏离同步转速减滑差")
    if abs(te_meas - TL) > TOL_TE:
        problems.append("带载电磁转矩没有回到负载转矩")
    if abs(i_meas - i_load) > TOL_I:
        problems.append("带载电流偏离等效电路")
    if abs(i_noload - i0_th) > TOL_I:
        problems.append("空载电流偏离励磁电流")
    if n_meas >= 1500.0:
        problems.append("带载转速应低于同步转速")
    if te_max_on <= te_max_low * 2.0:
        problems.append("低频提升应明显提高最大转矩")
    if abs(te_m0 - TL_LOW) > TOL_TE or abs(te_m1 - TL_LOW) > TOL_TE:
        problems.append("5 Hz 稳态转矩没有等于 8 N·m")
    if abs(n_m0 - n_off) > TOL_N or abs(n_m1 - n_on) > TOL_N:
        problems.append("5 Hz 转速偏离等效电路")
    if n_m1 < n_m0 + 30.0:
        problems.append("低频提升应把转速拉近同步转速")
    if n_m0 > 120.0:
        problems.append("无补偿的 5 Hz 运行滑差应明显偏大")
    if slip_bd_low <= slip_load:
        problems.append("低速临界滑差应大于额定工况滑差")
    problems.extend(scipy_checks(slip_load, slip_bd))

    if problems:
        print("校验未通过：", file=sys.stderr)
        for item in problems:
            print(f"  - {item}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
