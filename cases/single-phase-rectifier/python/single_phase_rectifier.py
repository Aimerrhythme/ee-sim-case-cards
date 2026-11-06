#!/usr/bin/env python3
"""理想单相桥式整流 + 电容滤波（教学演示）。

交流源 v_s = Vm sin(ωt)，理想二极管桥（Vf = 0），电容 C 与负载 R 并联。
主工况电源内阻 Rs = 0：导通时电容电压贴着 |v_s|，截止后按 RC 放电。

    导通（|v_s| >= 按 RC 自由放电的电压，且桥电流 >= 0）:
        v_C = |v_s|
        i_D = C d|v_s|/dt + |v_s|/R
    截止:
        dv_C/dt = -v_C/(R C)
        i_D = 0

不接电容时 v_O = |v_s|，平均值 2 Vm/π。

接上电容后的课堂估算（把放电时间当成整整半个周期、电流当成 Vm/R）::

    Io ≈ Vm/R
    ΔV ≈ Io / (2 f C)
    Vdc ≈ Vm - ΔV/2

这组圆整参数下 Io = 1 A，ΔV = 10 V，Vdc = 95 V。估算偏大，因为二极管
只在正弦顶点附近导通，放电不到 10 ms。脚本另解一次导通角，用来和仿真对读数。

峰值合闸（初相 90°）时若 Rs = 0，电流是冲激。讨论冲击时串 Rs = 1 Ω，
合闸瞬间 i(0+) = Vm/Rs = 100 A。

用法（在仓库根目录或本目录均可）::

    python cases/single-phase-rectifier/python/single_phase_rectifier.py

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

VM = 100.0  # V，交流峰值。课堂圆整值，不是 220 V 电网的实测峰值
F_AC = 50.0  # Hz
R_LOAD = 100.0  # Ω
C_MAIN = 1000e-6  # F（1000 μF）
C_DOUBLE = 2000e-6  # F，纹波大约减半的对照
RS_MAIN = 0.0  # Ω，主工况理想内阻
RS_INRUSH = 1.0  # Ω，只在「峰值合闸」讨论里串上
PHI_INRUSH = math.pi / 2  # rad，合闸在电压峰值
DT = 5e-6  # s。5 ms 的顶点、10 ms 的过零都落在步长上
T_END = 0.2  # s，10 个工频周期
N_STEADY_CYCLES = 2

OMEGA = 2.0 * math.pi * F_AC
IO_EST = VM / R_LOAD
DV_EST = IO_EST / (2.0 * F_AC * C_MAIN)  # 10 V
VDC_EST = VM - 0.5 * DV_EST  # 95 V
DV_EST_C2 = IO_EST / (2.0 * F_AC * C_DOUBLE)  # 5 V
VDC_EST_C2 = VM - 0.5 * DV_EST_C2  # 97.5 V
VAVG_NOC = 2.0 * VM / math.pi

# 仿真相对导通角解析解的容差。步长 5 μs，谷值落在步与步之间，差零点几毫伏。
TOL_V = 2e-3  # V
TOL_I_FIRST = 0.05  # A
TOL_I_SS = 0.15  # A
TOL_COND = 0.01
TOL_NOC = 1e-4  # V
TOL_EXP = 1e-6  # V
TOL_EST_VAVG = 0.02  # 平均电压相对手算 95 V


def conduction(cap: float) -> dict[str, float]:
    """Rs = 0 的稳态导通角。

    电容贴着正弦走到电流回到 0 的角度 theta2，再按指数放电，
    下半周的正弦在 pi+theta1 追上它。顶点在导通窗口内部，所以峰值就是 Vm。
    """
    wrc = OMEGA * R_LOAD * cap
    theta2 = math.pi - math.atan(wrc)
    s2 = math.sin(theta2)

    def gap(theta1: float) -> float:
        return math.sin(theta1) - s2 * math.exp(-(math.pi + theta1 - theta2) / wrc)

    lo = 1e-12
    hi = math.pi / 2.0
    if gap(lo) * gap(hi) > 0.0:
        raise RuntimeError("导通角方程没有落在 (0, pi/2) 里。")
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if gap(lo) * gap(mid) <= 0.0:
            hi = mid
        else:
            lo = mid
    theta1 = 0.5 * (lo + hi)
    vmin = VM * math.sin(theta1)
    int_sin = VM * (-math.cos(theta2) + math.cos(theta1))
    decay = math.exp(-(math.pi + theta1 - theta2) / wrc)
    int_exp = VM * s2 * wrc * (1.0 - decay)
    vavg = (int_sin + int_exp) / math.pi
    span = theta2 - theta1
    return {
        "theta1": theta1,
        "theta2": theta2,
        "vmin": vmin,
        "vmax": VM,
        "dV": VM - vmin,
        "vavg": vavg,
        "cond": span / math.pi,
        "t_on": span / OMEGA,
        "t_off": (math.pi - span) / OMEGA,
        "i_ss": cap * VM * OMEGA * math.cos(theta1) + VM * math.sin(theta1) / R_LOAD,
        "i0": cap * OMEGA * VM,
        "i_first": (VM / R_LOAD) * math.sqrt(1.0 + wrc * wrc),
    }


def simulate_ideal(cap: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """定步长、Rs = 0。导通步把电压钳到 |v_s|，截止步用精确的指数衰减。

    返回 t、v_C、i_D，样本在每个步长结束的时刻。t = 0 的冲击电流另算，
    不放进这个数组：第一步结束时正弦已经离开过零点。
    """
    n = int(round(T_END / DT))
    t = np.empty(n)
    vc_out = np.empty(n)
    idc_out = np.empty(n)
    vc = 0.0
    rc = R_LOAD * cap
    for k in range(n):
        t_next = (k + 1) * DT
        vc_off = vc * math.exp(-DT / rc)
        s = math.sin(OMEGA * t_next)
        c = math.cos(OMEGA * t_next)
        vr = abs(VM * s)
        if vr >= vc_off:
            dabs = VM * OMEGA * c if s >= 0.0 else -VM * OMEGA * c
            i_try = cap * dabs + vr / R_LOAD
            if i_try >= 0.0:
                vc = vr
                idc = i_try
            else:
                vc = vc_off
                idc = 0.0
        else:
            vc = vc_off
            idc = 0.0
        t[k] = t_next
        vc_out[k] = vc
        idc_out[k] = idc
    return t, vc_out, idc_out


def deriv_rs(t: float, vc: float, rs: float, phi: float, cap: float) -> float:
    vs = VM * math.sin(OMEGA * t + phi)
    vr = abs(vs)
    idc = (vr - vc) / rs if vr > vc else 0.0
    return (idc - vc / R_LOAD) / cap


def diode_current(t: float, vc: float, rs: float, phi: float) -> float:
    vr = abs(VM * math.sin(OMEGA * t + phi))
    return (vr - vc) / rs if vr > vc else 0.0


def simulate_rs(rs: float, phi: float, cap: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """有内阻时用定步长 RK4。每个子步都重新判断二极管，返回含 t = 0 的序列。"""
    n = int(round(T_END / DT))
    t = np.empty(n + 1)
    vc_out = np.empty(n + 1)
    idc_out = np.empty(n + 1)
    vc = 0.0
    t[0] = 0.0
    vc_out[0] = vc
    idc_out[0] = diode_current(0.0, vc, rs, phi)
    tt = 0.0
    for k in range(n):
        k1 = deriv_rs(tt, vc, rs, phi, cap)
        k2 = deriv_rs(tt + 0.5 * DT, vc + 0.5 * DT * k1, rs, phi, cap)
        k3 = deriv_rs(tt + 0.5 * DT, vc + 0.5 * DT * k2, rs, phi, cap)
        k4 = deriv_rs(tt + DT, vc + DT * k3, rs, phi, cap)
        vc = vc + (DT / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        tt = (k + 1) * DT
        t[k + 1] = tt
        vc_out[k + 1] = vc
        idc_out[k + 1] = diode_current(tt, vc, rs, phi)
    return t, vc_out, idc_out


def steady_mask(t: np.ndarray) -> np.ndarray:
    """最后两个工频周期。窗口两端相位相同，只保留右端那一个样本。

    两端都算进去时，过零点的 0 会被算两次，不接电容的平均值大约偏低 8 mV。
    """
    t0 = T_END - N_STEADY_CYCLES / F_AC
    return (t > t0 + 0.25 * DT) & (t <= T_END + 0.25 * DT)


def exp_error(t: np.ndarray, vc: np.ndarray, idc: np.ndarray, cap: float) -> float:
    """稳态里最长的一段截止区间，端点应严格满足指数放电。"""
    off = idc <= 0.0
    best = 0
    span = (0, 0)
    i = 0
    n = off.size
    while i < n:
        if not off[i]:
            i += 1
            continue
        j = i
        while j < n and off[j]:
            j += 1
        if j - i > best:
            best = j - i
            span = (i, j - 1)
        i = j
    i0, i1 = span
    if i1 <= i0:
        return math.inf
    pred = float(vc[i0]) * math.exp(-(float(t[i1]) - float(t[i0])) / (R_LOAD * cap))
    return abs(pred - float(vc[i1]))


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
    t: np.ndarray,
    vc: np.ndarray,
    idc: np.ndarray,
    t2: np.ndarray,
    vc2: np.ndarray,
    t_rs: np.ndarray,
    vc_rs: np.ndarray,
    idc_rs: np.ndarray,
    vavg_theory: float,
    out: Path,
) -> None:
    _enable_cjk()
    fig, axes = plt.subplots(3, 2, figsize=(9.6, 9.2))
    vrect = np.abs(VM * np.sin(OMEGA * t))

    ax = axes[0, 0]
    m = t <= 0.04 + 0.5 * DT
    ax.plot(t[m] * 1e3, vrect[m], color="#9ca3af", lw=1.0, label=r"$|v_s|$")
    ax.plot(t[m] * 1e3, vc[m], color="#1f4e79", lw=1.4, label="vC，接上 C")
    ax.axhline(VDC_EST, color="#e07a3d", lw=0.9, ls="--", label="估算平均 95 V")
    ax.axhline(VAVG_NOC, color="#5b8a72", lw=0.9, ls=":", label="不接 C 的平均")
    ax.set_xlim(0.0, 40.0)
    ax.set_ylim(-2.0, 115.0)
    ax.set_ylabel("电压 / V")
    ax.set_title("启动    过零合闸，Rs = 0")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right", frameon=False, fontsize=8)

    ax = axes[0, 1]
    ax.plot(t[m] * 1e3, idc[m], color="#1f4e79", lw=1.0, label="桥电流 iD")
    ax.axhline(C_MAIN * OMEGA * VM, color="#e07a3d", lw=0.9, ls="--", label=r"$C\omega V_m=10\pi$ A")
    ax.set_xlim(0.0, 40.0)
    ax.set_ylabel("电流 / A")
    ax.set_title("启动电流    第一下最大")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[1, 0]
    ms = steady_mask(t)
    ax.plot(t[ms] * 1e3, vrect[ms], color="#9ca3af", lw=1.0, label=r"$|v_s|$")
    ax.plot(t[ms] * 1e3, vc[ms], color="#1f4e79", lw=1.5, label="C = 1000 μF")
    m2 = steady_mask(t2)
    ax.plot(t2[m2] * 1e3, vc2[m2], color="#e07a3d", lw=1.3, label="C = 2000 μF")
    ax.axhline(90.0, color="#c0392b", lw=0.8, ls=":", label="估算谷值 90 V")
    ax.set_ylim(86.0, 104.0)
    ax.set_ylabel("电压 / V")
    ax.set_title("稳态    最后两个工频周期")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower left", frameon=False, fontsize=8)

    ax = axes[1, 1]
    ax.plot(t[ms] * 1e3, idc[ms], color="#1f4e79", lw=1.2, label="iD")
    ax.set_ylabel("电流 / A")
    ax.set_title("稳态    电流只在顶点附近出现")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[2, 0]
    mn = t <= 0.04 + 0.5 * DT
    ax.plot(t[mn] * 1e3, vrect[mn], color="#1f4e79", lw=1.2, label=r"$|v_s|$")
    ax.axhline(VAVG_NOC, color="#e07a3d", lw=1.0, ls="--", label=r"平均 $2V_m/\pi$")
    ax.axhline(vavg_theory, color="#5b8a72", lw=1.0, ls=":", label="接上 C 之后的平均")
    ax.set_xlim(0.0, 40.0)
    ax.set_ylim(-2.0, 115.0)
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("电压 / V")
    ax.set_title("不接电容    馒头波")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper left", frameon=False, fontsize=8)

    ax = axes[2, 1]
    early = t_rs <= 0.02 + 0.5 * DT
    line_i, = ax.plot(t_rs[early] * 1e3, idc_rs[early], color="#c0392b", lw=1.2, label="iD")
    line_100 = ax.axhline(VM / RS_INRUSH, color="#e07a3d", lw=0.9, ls="--", label="i(0+) = 100 A")
    ax.set_xlim(0.0, 20.0)
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("电流 / A")
    ax.set_title("峰值合闸    初相 90°，串 1 Ω")
    ax.grid(True, alpha=0.35)
    # 电压爬升画在右侧，让「100 A 是在给电容充电」看得见。
    ax_v = ax.twinx()
    line_v, = ax_v.plot(t_rs[early] * 1e3, vc_rs[early], color="#1f4e79", lw=1.1, label="vC")
    ax_v.set_ylim(-2.0, 115.0)
    ax_v.set_ylabel("vC / V", color="#1f4e79")
    ax_v.tick_params(axis="y", labelcolor="#1f4e79")
    ax.legend(
        [line_i, line_100, line_v],
        ["iD，Rs = 1 Ω", "i(0+) = 100 A", "vC（右轴）"],
        loc="center right",
        frameon=False,
        fontsize=8,
    )

    fig.suptitle(
        rf"单相桥式整流 + 电容滤波    $V_m={VM:.0f}\,\mathrm{{V}}$    $f={F_AC:.0f}\,\mathrm{{Hz}}$",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def _line(key: str, value: float) -> str:
    text = f"{value:.6f}"
    if abs(value) < 5e-7:
        text = "0.000000"
    return f"  {key:<26} = {text}"


def _stats(t: np.ndarray, vc: np.ndarray, idc: np.ndarray) -> dict[str, float]:
    m = steady_mask(t)
    vc_s = vc[m]
    idc_s = idc[m]
    return {
        "Vavg": float(np.mean(vc_s)),
        "Vmax": float(np.max(vc_s)),
        "Vmin": float(np.min(vc_s)),
        "dV": float(np.max(vc_s) - np.min(vc_s)),
        "i_peak": float(np.max(idc_s)),
        "cond": float(np.mean(idc_s > 0.0)),
    }


def main() -> int:
    if min(VM, F_AC, R_LOAD, C_MAIN, C_DOUBLE, RS_INRUSH, DT, T_END) <= 0.0:
        print("教学参数要求 Vm、f、R、C、Rs、步长都大于 0。", file=sys.stderr)
        return 1

    thy = conduction(C_MAIN)
    thy2 = conduction(C_DOUBLE)
    t, vc, idc = simulate_ideal(C_MAIN)
    t2, vc2, idc2 = simulate_ideal(C_DOUBLE)
    st = _stats(t, vc, idc)
    st2 = _stats(t2, vc2, idc2)

    m_noc = steady_mask(t)
    vavg_noc = float(np.mean(np.abs(VM * np.sin(OMEGA * t[m_noc]))))
    first = t <= 5e-3 + 0.5 * DT
    i_first_meas = float(np.max(idc[first]))
    err_exp = exp_error(t[m_noc], vc[m_noc], idc[m_noc], C_MAIN)
    err_exp2 = exp_error(t2[steady_mask(t2)], vc2[steady_mask(t2)], idc2[steady_mask(t2)], C_DOUBLE)

    t_rs, vc_rs, idc_rs = simulate_rs(RS_INRUSH, 0.0, C_MAIN)
    t_pk, vc_pk, idc_pk = simulate_rs(RS_INRUSH, PHI_INRUSH, C_MAIN)
    st_rs = _stats(t_rs, vc_rs, idc_rs)
    st_pk = _stats(t_pk, vc_pk, idc_pk)
    i_start_rs = float(np.max(idc_rs[t_rs <= 0.01 + 0.5 * DT]))
    hit90 = np.where(vc_pk >= 90.0)[0]
    t_vc90 = float(t_pk[hit90[0]]) * 1e3 if hit90.size else math.inf
    # Rs 序列含 t = 0，下标就是 t/DT。
    vc_inrush_2ms = float(vc_pk[int(round(0.002 / DT))])
    vc_inrush_10ms = float(vc_pk[int(round(0.010 / DT))])

    rows = [
        ("Vm_V", VM),
        ("f_Hz", F_AC),
        ("R_ohm", R_LOAD),
        ("C_F", C_MAIN),
        ("Rs_main_ohm", RS_MAIN),
        ("omega_rad_s", OMEGA),
        ("Io_est_A", IO_EST),
        ("dV_est_V", DV_EST),
        ("Vdc_est_V", VDC_EST),
        ("Vavg_noc_theory_V", VAVG_NOC),
        ("Vavg_noc_meas_V", vavg_noc),
        ("theta1_deg", math.degrees(thy["theta1"])),
        ("theta2_deg", math.degrees(thy["theta2"])),
        ("t_on_ms", thy["t_on"] * 1e3),
        ("t_off_ms", thy["t_off"] * 1e3),
        ("cond_frac_theory", thy["cond"]),
        ("Vmax_theory_V", thy["vmax"]),
        ("Vmin_theory_V", thy["vmin"]),
        ("dV_theory_V", thy["dV"]),
        ("Vavg_theory_V", thy["vavg"]),
        ("Io_theory_A", thy["vavg"] / R_LOAD),
        ("i_ss_peak_theory_A", thy["i_ss"]),
        ("i0_theory_A", thy["i0"]),
        ("i_first_peak_theory_A", thy["i_first"]),
        ("Vmax_meas_V", st["Vmax"]),
        ("Vmin_meas_V", st["Vmin"]),
        ("dV_meas_V", st["dV"]),
        ("Vavg_meas_V", st["Vavg"]),
        ("i_ss_peak_meas_A", st["i_peak"]),
        ("i_first_peak_meas_A", i_first_meas),
        ("cond_frac_meas", st["cond"]),
        ("rel_err_Vavg_vs_est", (st["Vavg"] - VDC_EST) / VDC_EST),
        ("rel_err_dV_vs_est", (st["dV"] - DV_EST) / DV_EST),
        ("rel_err_Vavg_vs_thy", (st["Vavg"] - thy["vavg"]) / thy["vavg"]),
        ("rel_err_dV_vs_thy", (st["dV"] - thy["dV"]) / thy["dV"]),
        ("C2_F", C_DOUBLE),
        ("dV_est_C2_V", DV_EST_C2),
        ("Vdc_est_C2_V", VDC_EST_C2),
        ("dV_theory_C2_V", thy2["dV"]),
        ("Vavg_theory_C2_V", thy2["vavg"]),
        ("Vmin_theory_C2_V", thy2["vmin"]),
        ("dV_meas_C2_V", st2["dV"]),
        ("Vavg_meas_C2_V", st2["Vavg"]),
        ("Vmin_meas_C2_V", st2["Vmin"]),
        ("Rs_inrush_ohm", RS_INRUSH),
        ("phi_inrush_deg", math.degrees(PHI_INRUSH)),
        ("i_inrush_theory_A", VM / RS_INRUSH),
        ("i_inrush_peak_A", float(np.max(idc_pk))),
        ("t_vc90_inrush_ms", t_vc90),
        ("vc_inrush_2ms_V", vc_inrush_2ms),
        ("vc_inrush_10ms_V", vc_inrush_10ms),
        ("Vavg_Rs_meas_V", st_rs["Vavg"]),
        ("dV_Rs_meas_V", st_rs["dV"]),
        ("Vmax_Rs_meas_V", st_rs["Vmax"]),
        ("Vmin_Rs_meas_V", st_rs["Vmin"]),
        ("i_ss_Rs_A", st_rs["i_peak"]),
        ("i_start_Rs_A", i_start_rs),
        ("Vavg_Rs90_meas_V", st_pk["Vavg"]),
    ]

    print("Single-phase bridge rectifier + C filter")
    for key, value in rows:
        print(_line(key, value))
    print(f"  {'exp_err_V':<26} = {err_exp:.3e}")
    print(f"  {'exp_err_C2_V':<26} = {err_exp2:.3e}")

    out = Path(__file__).resolve().parent.parent / "img" / "single_phase_rectifier_curves.png"
    plot_case(t, vc, idc, t2, vc2, t_pk, vc_pk, idc_pk, thy["vavg"], out)
    print(f"  {'figure':<26} = {out}")

    problems: list[str] = []
    if abs(IO_EST - 1.0) > 1e-12 or abs(DV_EST - 10.0) > 1e-12 or abs(VDC_EST - 95.0) > 1e-12:
        problems.append("主工况手算没有落在 Io = 1 A、ΔV = 10 V、Vdc = 95 V")
    if abs(DV_EST_C2 - 5.0) > 1e-12 or abs(VDC_EST_C2 - 97.5) > 1e-12:
        problems.append("电容加倍后的手算不是 ΔV = 5 V、Vdc = 97.5 V")
    if abs(thy["i0"] - 10.0 * math.pi) > 1e-9:
        problems.append("过零合闸的 C·ω·Vm 应等于 10π")
    if abs(vavg_noc - VAVG_NOC) > TOL_NOC:
        problems.append("不接电容的平均值偏离 2Vm/π")
    if abs(st["Vmax"] - VM) > 1e-6:
        problems.append("理想桥的稳态峰值应到 Vm")
    if abs(st["Vmin"] - thy["vmin"]) > TOL_V or abs(st["Vavg"] - thy["vavg"]) > TOL_V:
        problems.append("稳态电压偏离导通角解析解")
    if abs(st["dV"] - thy["dV"]) > TOL_V:
        problems.append("纹波偏离导通角解析解")
    if not (0.7 * DV_EST < st["dV"] < DV_EST):
        problems.append("实测纹波应小于手算 10 V，又不能小得离谱")
    if abs(st["Vavg"] - VDC_EST) / VDC_EST > TOL_EST_VAVG:
        problems.append("平均电压离手算 95 V 超过 2%")
    if abs(i_first_meas - thy["i_first"]) > TOL_I_FIRST:
        problems.append("第一下充电电流偏离 sqrt(1+(ωRC)^2)·Vm/R")
    if abs(st["i_peak"] - thy["i_ss"]) > TOL_I_SS:
        problems.append("稳态电流脉冲峰值偏离导通起点的解析值")
    if abs(st["cond"] - thy["cond"]) > TOL_COND:
        problems.append("导通时间占比偏离解析导通角")
    if err_exp > TOL_EXP or err_exp2 > TOL_EXP:
        problems.append("截止区间不是 RC 指数放电")
    if abs(st2["dV"] - thy2["dV"]) > TOL_V or abs(st2["Vavg"] - thy2["vavg"]) > TOL_V:
        problems.append("C = 2000 μF 偏离导通角解析解")
    ratio = st2["dV"] / st["dV"]
    if not 0.45 <= ratio <= 0.65:
        problems.append("电容加倍后纹波应大约减半")
    if float(np.max(vc)) > VM + 1e-6 or float(np.max(vc2)) > VM + 1e-6:
        problems.append("理想二极管桥的电容电压不应超过 Vm")
    if float(np.min(vc)) < -1e-8:
        problems.append("电容电压不应为负")
    if abs(float(idc_pk[0]) - VM / RS_INRUSH) > 1e-9:
        problems.append("峰值合闸瞬间电流应等于 Vm/Rs = 100 A")
    if float(np.max(idc_pk)) > VM / RS_INRUSH + 1e-6:
        problems.append("峰值合闸的电流峰值应出现在 t = 0+")
    if not 70.0 < vc_inrush_2ms < 85.0:
        problems.append("峰值合闸后 2 ms，电容应停在 70–85 V，第一下充不满")
    if vc_inrush_10ms < 90.0 or t_vc90 > 12.0:
        problems.append("串 1 Ω 时要到下一个半周才充到 90 V")
    if abs(st_rs["Vavg"] - st_pk["Vavg"]) > 0.01:
        problems.append("过零合闸和峰值合闸的稳态平均电压应相同")
    if not (90.0 < st_rs["Vavg"] < thy["vavg"]):
        problems.append("串 1 Ω 后平均电压应低于理想值，仍远高于不接电容")
    if i_first_meas <= st["i_peak"]:
        problems.append("过零合闸的第一下电流应大于稳态脉冲")

    if problems:
        print("校验未通过：", file=sys.stderr)
        for item in problems:
            print(f"  - {item}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
