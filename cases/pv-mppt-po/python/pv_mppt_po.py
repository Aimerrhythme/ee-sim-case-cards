#!/usr/bin/env python3
"""光伏 MPPT 扰动观察法（P&O）教学仿真：单二极管 PV 阵列 + Boost 平均模型 + 步长对比。

单二极管模型（阵列 = NSER 块组件串联，每块 NS_CELL 片电池串联）::

    I = Iph − I0·[exp((V + I·Rs)/a) − 1] − (V + I·Rs)/Rsh
    a = NSER·NS_CELL·n·Vt，Vt = k·T/q，Iph = Isc·G/1000（忽略温度变化，T = 25 ℃）
    I 是 V 的隐函数，脚本每步用牛顿迭代求解（上一步的 I 作初值）。

Boost 平均模型（输出接 200 V 直流母线 / 蓄电池组，视为恒压源）::

    Cin·dVpv/dt = Ipv(Vpv) − iL
    L·diL/dt   = Vpv − RL·iL − (1 − D)·Vbus        （iL ≥ 0：二极管阻断反向电流）
    稳态 Vpv ≈ (1 − D)·Vbus + RL·iL —— 调 D 就是调 PV 工作电压，D 增大 → Vpv 降低

P&O（每 T_MPPT 采样一次 Vpv、Ipv，P = V·I）::

    dP = P − P_prev，dV = V − V_prev
    dP·dV > 0（功率随电压升高而增大，在 MPP 左侧）→ 升电压：D −= ΔD
    dP·dV < 0（在 MPP 右侧）                     → 降电压：D += ΔD
    稳态在 MPP 附近做三点振荡（D* − ΔD、D*、D* + ΔD）

脚本做的事：
  1. 求 1000 / 600 W/m² 的 I-V、P-V 曲线和 MPP（黄金分割，密集扫描核对）；
  2. 工况 A：0 s 从开路附近（D0 = 0.45）启动，1.5 s 辐照阶跃 1000 → 600 W/m²，
     对比 ΔD = 0.002 / 0.005 / 0.02 三种步长的跟踪时间、稳态振荡和跟踪效率；
  3. 工况 B：ΔD 扫描 0.001 ~ 0.03；
  4. 工况 C：辐照 600 → 1000 W/m² 斜坡（1 s 内），看 P&O 的「误判漂移」。

用法（在仓库根目录或本目录均可）::

    python cases/pv-mppt-po/python/pv_mppt_po.py

无图形界面时只保存图片、不弹窗。依赖见 requirements.txt（numpy、matplotlib）。
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

# ---- PV 组件 / 阵列（教学示例参数，不是某一型号组件的实测数据）----
K_B = 1.380649e-23
Q_E = 1.602176634e-19
T_CELL = 298.15  # K，25 ℃
VT = K_B * T_CELL / Q_E  # 25.693 mV
NS_CELL = 60  # 每块组件串联电池片数
N_IDEAL = 1.3  # 二极管理想因子
ISC_STC = 8.5  # A，STC 短路电流（取 Iph_STC = Isc）
VOC_MOD = 37.5  # V，单块组件开路电压（用于反推 I0，忽略 Rsh）
RS_MOD = 0.3  # Ω，单块组件串联电阻
RSH_MOD = 250.0  # Ω，单块组件并联电阻
NSER = 3  # 阵列串联组件数

A_ARR = NSER * NS_CELL * N_IDEAL * VT  # V，阵列修正热电压 a
RS = NSER * RS_MOD
RSH = NSER * RSH_MOD
I0 = ISC_STC / (math.exp(NSER * VOC_MOD / A_ARR) - 1.0)

# ---- Boost 平均模型 ----
VBUS = 200.0  # V，直流母线 / 蓄电池组电压
L_B = 3e-3  # H
C_IN = 47e-6  # F，PV 侧输入电容
R_L = 0.2  # Ω，电感 + 开关等效电阻
D_MIN, D_MAX = 0.05, 0.90
D0 = 0.45  # 起始占空比（Vpv ≈ 110 V，接近开路）

# ---- MPPT 与仿真 ----
T_MPPT = 20e-3  # s，P&O 执行周期
DT = 50e-6  # s，RK4 定步长
T_END = 3.0
T_STEP = 1.5  # s，辐照阶跃时刻
G1, G2 = 1000.0, 600.0
STEPS = (0.002, 0.005, 0.02)  # 对比的三种 ΔD
SWEEP = (0.001, 0.002, 0.003, 0.005, 0.01, 0.02, 0.03)
TRACK_TH = 0.99  # 跟踪判据：P ≥ 99% Pmpp
WIN1 = (1.2, 1.5)  # 1000 W/m² 稳态统计窗口
WIN2 = (2.7, 3.0)  # 600 W/m² 稳态统计窗口
RAMP = (1.5, 2.5, 600.0, 1000.0)  # 工况 C：t0、t1、G 起、G 止
T_END_C = 3.5  # 工况 C 仿真时长
D0_C = 0.50  # 工况 C 起始占空比（600 W/m² 时 Voc = 109.3 V < (1 − 0.45)·200 V，D0 = 0.45 会卡死，见 [D]）


def pv_current(v: float, g: float, i: float | None = None) -> float:
    """单二极管模型：给定端电压 v、辐照 g，牛顿迭代求电流。"""
    iph = ISC_STC * g / 1000.0
    if i is None:
        i = iph
    for _ in range(60):
        e = math.exp(min((v + i * RS) / A_ARR, 700.0))
        f = iph - I0 * (e - 1.0) - (v + i * RS) / RSH - i
        df = -I0 * RS / A_ARR * e - RS / RSH - 1.0
        di = f / df
        i -= di
        if abs(di) < 1e-12:
            break
    return i


def pv_curve(g: float, n: int = 1201):
    voc = pv_voc(g)
    v = np.linspace(0.0, voc, n)
    i = np.empty(n)
    ig = None
    for k, vk in enumerate(v):
        ig = pv_current(vk, g, ig)
        i[k] = ig
    return v, i


def pv_voc(g: float) -> float:
    lo, hi = 0.0, 2.0 * NSER * VOC_MOD
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if pv_current(mid, g) > 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def pv_mpp(g: float):
    """黄金分割求 MPP，返回 (Vmp, Imp, Pmp)。"""
    lo, hi = 0.0, pv_voc(g)
    r = (math.sqrt(5.0) - 1.0) / 2.0
    p = lambda v: v * pv_current(v, g)  # noqa: E731
    x1, x2 = hi - r * (hi - lo), lo + r * (hi - lo)
    f1, f2 = p(x1), p(x2)
    for _ in range(120):
        if f1 > f2:
            hi, x2, f2 = x2, x1, f1
            x1 = hi - r * (hi - lo)
            f1 = p(x1)
        else:
            lo, x1, f1 = x1, x2, f2
            x2 = lo + r * (hi - lo)
            f2 = p(x2)
    v = 0.5 * (lo + hi)
    i = pv_current(v, g)
    return v, i, v * i


def irradiance(t: float, case: dict) -> float:
    if case.get("ramp"):
        t0, t1, ga, gb = case["ramp"]
        if t <= t0:
            return ga
        if t >= t1:
            return gb
        return ga + (gb - ga) * (t - t0) / (t1 - t0)
    return G1 if t < case.get("t_step", T_STEP) else G2


def simulate(dstep: float, case: dict | None = None, t_end: float = T_END, dt: float = DT):
    """Boost 平均模型 + P&O，RK4 定步长。返回各量时间序列与 MPPT 采样记录。"""
    case = case or {}
    n = int(round(t_end / dt))
    n_mppt = int(round(T_MPPT / dt))
    t = np.arange(n + 1) * dt
    VPV = np.empty(n + 1)
    IPV = np.empty(n + 1)
    IL = np.empty(n + 1)
    DD = np.empty(n + 1)
    G = np.empty(n + 1)

    g0 = irradiance(0.0, case)
    vpv = pv_voc(g0)  # 起始：电容电压 = 开路电压，电感电流 0
    il = 0.0
    d = case.get("d0", D0)
    ig = pv_current(vpv, g0)
    v_prev = p_prev = None
    samples = []  # (t, V, I, P, D 更新后)

    def rhs(tt, v, i_l, i_guess):
        ipv = pv_current(v, irradiance(tt, case), i_guess)
        dv = (ipv - i_l) / C_IN
        di = (v - R_L * i_l - (1.0 - d) * VBUS) / L_B
        if i_l <= 0.0 and di < 0.0:  # 二极管阻断：电感电流不反向
            di = 0.0
        return dv, di, ipv

    for k in range(n + 1):
        tk = t[k]
        if k % n_mppt == 0 and k > 0:  # ---- P&O ----
            v_s, i_s = vpv, ig
            p_s = v_s * i_s
            if v_prev is None:
                d += dstep  # 第一次：先往降电压方向扰动
            else:
                dp, dv = p_s - p_prev, v_s - v_prev
                if dp != 0.0:
                    if dp * dv > 0:
                        d -= dstep  # 升电压
                    else:
                        d += dstep  # 降电压
            d = min(max(d, D_MIN), D_MAX)
            v_prev, p_prev = v_s, p_s
            samples.append((tk, v_s, i_s, p_s, d))
        ig = pv_current(vpv, irradiance(tk, case), ig)
        VPV[k], IPV[k], IL[k], DD[k], G[k] = vpv, ig, il, d, irradiance(tk, case)
        if k == n:
            break
        a1, b1, _ = rhs(tk, vpv, il, ig)
        a2, b2, _ = rhs(tk + dt / 2, vpv + dt / 2 * a1, il + dt / 2 * b1, ig)
        a3, b3, _ = rhs(tk + dt / 2, vpv + dt / 2 * a2, il + dt / 2 * b2, ig)
        a4, b4, _ = rhs(tk + dt, vpv + dt * a3, il + dt * b3, ig)
        vpv += dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        il = max(il + dt / 6 * (b1 + 2 * b2 + 2 * b3 + b4), 0.0)
    return {"t": t, "v": VPV, "i": IPV, "il": IL, "d": DD, "g": G, "p": VPV * IPV,
            "samples": np.array(samples)}


# ---- Pmpp(G) 查表（工况 C 斜坡用）----
_G_TAB = np.linspace(500.0, 1000.0, 51)
_MPP_TAB = np.array([pv_mpp(g) for g in _G_TAB])


def pmpp_of(g):
    return np.interp(g, _G_TAB, _MPP_TAB[:, 2])


def vmp_of(g):
    return np.interp(g, _G_TAB, _MPP_TAB[:, 0])


def first_reach(t, p, pref, t0=0.0):
    m = np.nonzero((t >= t0) & (p >= TRACK_TH * pref))[0]
    return (t[m[0]] - t0) if m.size else math.inf


def metrics(r):
    t, p = r["t"], r["p"]
    pm = pmpp_of(r["g"])
    out = {}
    pre = t < T_STEP
    out["t_trk"] = first_reach(t[pre], p[pre], pm[pre])  # 阶跃前未到达记为 inf
    # 辐照阶跃：LC 暂态造成的功率跌落深度、跌出 99% 带到第一次回到带内的时刻
    m_after = (t >= T_STEP) & (t < T_STEP + 0.3)
    ratio = p[m_after] / pm[m_after]
    out["p_min_after"] = float(np.min(ratio))
    below = np.nonzero(ratio < TRACK_TH)[0]
    if below.size:
        back = np.nonzero(ratio[below[0]:] >= TRACK_TH)[0]
        out["t_rec"] = float(t[m_after][below[0] + back[0]] - T_STEP) if back.size else math.inf
    else:
        out["t_rec"] = 0.0
    # P&O 重新进入 600 W/m² 稳态三点振荡（D 只在 WIN2 内出现过的几个值之间跳）的时刻
    m2 = (t >= WIN2[0]) & (t < WIN2[1])
    d_set = np.unique(np.round(r["d"][m2], 9))
    s = r["samples"]
    sa = s[s[:, 0] >= T_STEP]
    outside = np.nonzero(~np.isin(np.round(sa[:, 4], 9), d_set))[0]
    out["t_d_settle"] = float(sa[outside[-1] + 1, 0] - T_STEP) if outside.size else float(sa[0, 0] - T_STEP)
    k1 = int(np.argmax(s[:, 0] > T_STEP))  # 阶跃后第一次 P&O 采样
    out["dp_first"] = float(s[k1, 3] - s[k1 - 1, 3])
    out["dd_first"] = float(s[k1, 4] - s[k1 - 1, 4])
    out["dd_prev"] = float(s[k1 - 1, 4] - s[k1 - 2, 4])
    out["n_moves_settle"] = int(outside[-1] + 2) if outside.size else 1
    for key, (w0, w1) in (("1", WIN1), ("2", WIN2)):
        m = (t >= w0) & (t < w1)
        out["pp" + key] = float(p[m].max() - p[m].min())
        out["eta" + key] = float(np.mean(p[m]) / np.mean(pm[m]))
        out["vpp" + key] = float(r["v"][m].max() - r["v"][m].min())
        out["nd" + key] = len(np.unique(np.round(r["d"][m], 9)))
    out["eta_all"] = float(np.trapezoid(p, t) / np.trapezoid(pm, t))
    return out


def matplotlib_backend_interactive() -> bool:
    return plt.get_backend().lower() != "agg"


def metrics_const(r):
    """恒定 1000 W/m²（不加阶跃）：启动跟踪时间、稳态（WIN2 窗口）振荡与效率。"""
    t, p = r["t"], r["p"]
    pm = pmpp_of(r["g"])
    m = (t >= WIN2[0]) & (t < WIN2[1])
    return {
        "t_trk": first_reach(t, p, pm),
        "pp": float(p[m].max() - p[m].min()),
        "eta_ss": float(np.mean(p[m]) / np.mean(pm[m])),
        "vpp": float(r["v"][m].max() - r["v"][m].min()),
        "eta_all": float(np.trapezoid(p, t) / np.trapezoid(pm, t)),
    }


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


def _p(key: str, v, fmt: str = ".3f") -> None:
    print(f"  {key:<30}= {v:{fmt}}")


def main() -> int:
    ok = True
    print("PV MPPT perturb & observe (teaching example)")
    _p("a_array_V", A_ARR, ".4f")
    _p("I0_A", I0, ".4e")
    mpp = {}
    for g in (G1, G2):
        vmp, imp, pmp = pv_mpp(g)
        v, i = pv_curve(g, 20001)
        k = int(np.argmax(v * i))
        voc = pv_voc(g)
        isc = pv_current(0.0, g)
        mpp[g] = (vmp, imp, pmp, voc, isc)
        print(f"[MPP] G = {g:.0f} W/m²")
        _p("Isc_A", isc)
        _p("Voc_V", voc)
        _p("Vmp_V", vmp)
        _p("Imp_A", imp)
        _p("Pmp_W", pmp)
        _p("FF", pmp / (voc * isc), ".4f")
        _p("D_mpp (=1-(Vmp+RL*Imp)/Vbus)", 1 - (vmp + R_L * imp) / VBUS, ".4f")
        _p("scan_Pmax_W (20001 pts)", float(v[k] * i[k]))
        if abs(v[k] * i[k] - pmp) > 1e-3 * pmp:
            ok = False

    # ---- Boost 输入侧 LC 小信号：PV 在 MPP 的增量电导 g = −dI/dV 提供阻尼 ----
    vmp1, imp1 = mpp[G1][0], mpp[G1][1]
    h = 1e-4
    g_pv = -(pv_current(vmp1 + h, G1) - pv_current(vmp1 - h, G1)) / (2 * h)
    wn = 1.0 / math.sqrt(L_B * C_IN)
    zeta = 0.5 * g_pv * math.sqrt(L_B / C_IN) + 0.5 * R_L * math.sqrt(C_IN / L_B)
    print("[LC] Boost 输入侧 L–Cin（1000 W/m² MPP 处线性化）")
    _p("g_pv_S (= Imp/Vmp at MPP)", g_pv, ".5f")
    _p("Imp/Vmp_S", imp1 / vmp1, ".5f")
    _p("fn_LC_Hz", wn / (2 * math.pi), ".1f")
    _p("zeta_LC", zeta, ".3f")
    _p("settle_4/(zeta*wn)_ms", 4e3 / (zeta * wn), ".2f")

    # ---- 工况 A：三种步长 ----
    print(f"[A] 启动 D0 = {D0}，{T_STEP} s 辐照 {G1:.0f} → {G2:.0f} W/m²，T_MPPT = {T_MPPT * 1e3:.0f} ms")
    runs = {}
    for ds in STEPS:
        r = simulate(ds)
        mm = metrics(r)
        runs[ds] = (r, mm)
        print(f"  ΔD = {ds:.3f}（ΔV ≈ {ds * VBUS:.1f} V/步）")
        _p("  t_track_99%_s", mm["t_trk"], ".3f")
        _p("  P_pp_1000_W", mm["pp1"], ".2f")
        _p("  Vpv_pp_1000_V", mm["vpp1"], ".2f")
        _p("  eta_ss_1000_pct", 100 * mm["eta1"], ".3f")
        _p("  D_levels_1000", mm["nd1"], "d")
        _p("  P_min/Pmpp_after_step_pct", 100 * mm["p_min_after"], ".2f")
        _p("  t_back_in_99%_ms", 1e3 * mm["t_rec"], ".2f")
        _p("  first_PO_after_step_dP_W", mm["dp_first"], ".2f")
        _p("  PO_dD_before_step", mm["dd_prev"], "+.3f")
        _p("  first_PO_after_step_dD", mm["dd_first"], "+.3f")
        _p("  t_PO_settle_ms", 1e3 * mm["t_d_settle"], ".0f")
        _p("  P_pp_600_W", mm["pp2"], ".2f")
        _p("  eta_ss_600_pct", 100 * mm["eta2"], ".3f")
        _p("  eta_total_0-3s_pct", 100 * mm["eta_all"], ".3f")
        # 自洽检查：稳态平均 Vpv ≈ (1 − D)Vbus + RL·iL
        m = (r["t"] >= WIN1[0]) & (r["t"] < WIN1[1])
        v_avg = np.mean((1 - r["d"][m]) * VBUS + R_L * r["il"][m])
        if abs(v_avg - np.mean(r["v"][m])) > 0.2:
            ok = False
            print("  ！平均模型稳态关系不成立", file=sys.stderr)

    # ---- 工况 B：步长扫描 ----
    print(f"[B] ΔD 扫描（恒定 1000 W/m²，0~{T_END:.0f} s；稳态窗口 {WIN2[0]}~{WIN2[1]} s；η_A = 工况 A 全程）")
    sweep = []
    for ds in SWEEP:
        mm = metrics_const(simulate(ds, {"t_step": math.inf}))
        mm["eta_A"] = runs[ds][1]["eta_all"] if ds in runs else metrics(simulate(ds))["eta_all"]
        sweep.append((ds, mm))
        print(f"  ΔD={ds:.3f}  ΔV≈{ds * VBUS:4.1f} V  t_trk={mm['t_trk']:6.3f} s  "
              f"P_pp={mm['pp']:6.2f} W  Vpv_pp={mm['vpp']:5.2f} V  η_ss={100 * mm['eta_ss']:7.3f}%  "
              f"η_0-3s={100 * mm['eta_all']:7.3f}%  η_A={100 * mm['eta_A']:7.3f}%")

    # ---- 工况 C：辐照斜坡 ----
    t0, t1, ga, gb = RAMP
    print(f"[C] 辐照斜坡 {ga:.0f} → {gb:.0f} W/m²（{t0:.1f} ~ {t1:.1f} s，{(gb - ga) / (t1 - t0):.0f} W/m²/s）")
    ramp_runs = {}
    case_c = {"ramp": RAMP, "d0": D0_C}
    for ds in STEPS:
        r = simulate(ds, case_c, t_end=T_END_C)
        t, p = r["t"], r["p"]
        pm, vm = pmpp_of(r["g"]), vmp_of(r["g"])
        m = (t >= t0) & (t <= t1)
        s = r["samples"]
        ms = (s[:, 0] > t0) & (s[:, 0] <= t1)
        dd = np.diff(np.concatenate([[s[np.argmax(ms) - 1, 4]], s[ms, 4]]))
        n_up = int(np.sum(dd < 0))  # D 减小 = 升电压
        res = {
            "eta_ramp": float(np.mean(p[m]) / np.mean(pm[m])),
            "dv_max": float(np.max(np.abs(r["v"][m] - vm[m]))),
            "n_up": n_up, "n_dn": int(np.sum(dd > 0)),
            "eta_all": float(np.trapezoid(p, t) / np.trapezoid(pm, t)),
        }
        ramp_runs[ds] = (r, res)
        print(f"  ΔD={ds:.3f}  η_ramp={100 * res['eta_ramp']:.3f}%  max|Vpv−Vmp|={res['dv_max']:.2f} V  "
              f"升压/降压动作={res['n_up']}/{res['n_dn']}  η_total={100 * res['eta_all']:.3f}%")

    # ---- [D] 启动卡死：开路区 P = 0 → dP = 0 → 经典 P&O 不动作 ----
    print(f"[D] 600 W/m² 恒定，D0 = {D0}：(1 − D0)·Vbus = {(1 - D0) * VBUS:.1f} V，Voc(600) = {mpp[G2][3]:.3f} V")
    stuck = {}
    for ds in STEPS:
        r = simulate(ds, {"t_step": -1.0}, t_end=1.5)
        stuck[ds] = r
        k = r["t"] >= 1.4
        print(f"  ΔD={ds:.3f}  首次扰动后 (1 − D)·Vbus = {(1 - D0 - ds) * VBUS:6.1f} V  "
              f"t = 1.4~1.5 s 平均功率 {np.mean(r['p'][k]):7.2f} W（Pmpp {mpp[G2][2]:.2f} W），D = {r['d'][-1]:.3f}")

    # ---------- 作图 ----------
    _enable_cjk()
    fig, ax = plt.subplots(2, 4, figsize=(19, 8.6))
    colors = {STEPS[0]: "tab:blue", STEPS[1]: "tab:green", STEPS[2]: "tab:red"}

    a = ax[0, 0]
    a2 = a.twinx()
    for g, ls in ((G1, "-"), (G2, "--")):
        v, i = pv_curve(g)
        a.plot(v, i, "k" + ls, lw=1.2, label=f"I-V {g:.0f} W/m²")
        a2.plot(v, v * i, "tab:orange", ls=ls, lw=1.4, label=f"P-V {g:.0f} W/m²")
        vmp, imp, pmp = mpp[g][:3]
        a2.plot([vmp], [pmp], "o", color="tab:orange")
        a2.annotate(f"MPP {pmp:.1f} W\n@{vmp:.2f} V", (vmp, pmp), xytext=(-95, -8),
                    textcoords="offset points", fontsize=7)
    s = runs[STEPS[1]][0]["samples"]
    a2.plot(s[s[:, 0] < T_STEP, 1], s[s[:, 0] < T_STEP, 3], ".", ms=3, color="tab:green",
            label=f"P&O 采样点 ΔD={STEPS[1]}")
    a.set_xlabel("Vpv / V")
    a.set_ylabel("Ipv / A")
    a2.set_ylabel("Ppv / W")
    a.set_ylim(0, 9.5)
    a2.set_ylim(0, 760)
    h1, l1 = a.get_legend_handles_labels()
    h2, l2 = a2.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, fontsize=7, loc="lower left")
    a.set_title("单二极管 PV 阵列 I-V / P-V（3 块串联）", fontsize=9)

    a = ax[0, 1]
    rr = runs[STEPS[0]][0]
    a.plot(rr["t"], pmpp_of(rr["g"]), "k--", lw=1, label="Pmpp(G)")
    for ds in STEPS:
        r, mm = runs[ds]
        a.plot(r["t"], r["p"], color=colors[ds], lw=0.9,
               label=f"ΔD={ds}（{mm['t_trk']:.2f} s 到 99%）")
    a.set_xlabel("t / s")
    a.set_ylabel("Ppv / W")
    a.set_ylim(0, 760)
    a.legend(fontsize=7, loc="lower right")
    a.set_title(f"工况 A：启动跟踪 + {T_STEP} s 辐照 1000→600 W/m²", fontsize=9)

    a = ax[0, 2]
    a.plot(rr["t"], vmp_of(rr["g"]), "k--", lw=1, label="Vmp(G)")
    for ds in STEPS:
        r, _ = runs[ds]
        a.plot(r["t"], r["v"], color=colors[ds], lw=0.9, label=f"Vpv ΔD={ds}")
    a.set_xlabel("t / s")
    a.set_ylabel("Vpv / V")
    a3 = a.twinx()
    r = runs[STEPS[1]][0]
    a3.plot(r["t"], r["d"], color="tab:purple", lw=0.8, alpha=0.6, ls=":")
    a3.set_ylabel(f"占空比 D（紫色点线，ΔD={STEPS[1]}）")
    a.legend(fontsize=7, loc="upper right")
    a.set_title("PV 电压与占空比：D 增大 → Vpv 降低", fontsize=9)

    a = ax[1, 0]
    for ds in STEPS:
        r, mm = runs[ds]
        m = (r["t"] >= 1.3) & (r["t"] < 1.5)
        a.plot(r["t"][m], r["p"][m], color=colors[ds], lw=1,
               label=f"ΔD={ds}：峰峰 {mm['pp1']:.2f} W，η {100 * mm['eta1']:.2f}%")
    a.axhline(mpp[G1][2], color="k", ls="--", lw=1, label=f"Pmpp {mpp[G1][2]:.1f} W")
    a.set_xlabel("t / s")
    a.set_ylabel("Ppv / W")
    a.legend(fontsize=7, loc="lower left")
    a.set_title("稳态三点振荡（1000 W/m²，放大）", fontsize=9)

    a = ax[1, 1]
    for ds in STEPS:
        r, mm = runs[ds]
        m = (r["t"] >= 1.44) & (r["t"] < 1.84)
        a.plot(r["t"][m], r["p"][m], color=colors[ds], lw=1,
               label=f"ΔD={ds}：最低 {100 * mm['p_min_after']:.1f}%，{1e3 * mm['t_rec']:.1f} ms 回到 99%")
    m = (rr["t"] >= 1.44) & (rr["t"] < 1.84)
    a.plot(rr["t"][m], pmpp_of(rr["g"][m]), "k--", lw=1, label="Pmpp(G)")
    a.set_xlabel("t / s")
    a.set_ylabel("Ppv / W")
    a.legend(fontsize=7, loc="upper right")
    a.set_title("辐照阶跃 1000→600 W/m² 附近（放大）", fontsize=9)

    a = ax[1, 2]
    ds_arr = np.array([x[0] for x in sweep])
    a.plot(ds_arr, [x[1]["t_trk"] for x in sweep], "o-", color="tab:blue", label="启动跟踪时间（左轴）")
    a.set_xscale("log")
    a.set_xlabel("步长 ΔD（对数）")
    a.set_ylabel("跟踪到 99% Pmpp / s")
    a4 = a.twinx()
    a4.plot(ds_arr, [x[1]["pp"] for x in sweep], "s-", color="tab:red", label="稳态功率峰峰（右轴）")
    a4.set_ylabel("ΔP 峰峰 / W")
    h1, l1 = a.get_legend_handles_labels()
    h2, l2 = a4.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, fontsize=7, loc="upper center")
    a.set_title("步长取舍：快 ↔ 稳", fontsize=9)

    a = ax[0, 3]
    for ds in STEPS:
        r, res = ramp_runs[ds]
        a.plot(r["t"], r["v"], lw=0.9, color=colors[ds],
               label=f"ΔD={ds}：max|Vpv−Vmp| {res['dv_max']:.1f} V，η {100 * res['eta_ramp']:.1f}%")
    a.plot(r["t"], vmp_of(r["g"]), "k--", lw=1, label="Vmp(G)")
    a.axvspan(RAMP[0], RAMP[1], color="0.9")
    a.set_xlabel("t / s")
    a.set_ylabel("Vpv / V")
    a.legend(fontsize=7, loc="lower right")
    a.set_title("工况 C：辐照 600→1000 W/m² 斜坡（灰底）", fontsize=9)

    a = ax[1, 3]
    a.plot(ds_arr, [100 * x[1]["eta_ss"] for x in sweep], "o-", label="稳态 η（1000 W/m²）")
    a.plot(ds_arr, [100 * x[1]["eta_all"] for x in sweep], "s-", label="恒定 1000 W/m² 0~3 s η（含启动）")
    a.plot(ds_arr, [100 * x[1]["eta_A"] for x in sweep], "^-", label="工况 A 全程 η（含启动 + 阶跃）")
    a.set_xscale("log")
    a.set_xlabel("步长 ΔD（对数）")
    a.set_ylabel("跟踪效率 η / %")
    a.legend(fontsize=7, loc="lower center")
    a.set_title("跟踪效率 η = ∫P dt / ∫Pmpp dt", fontsize=9)

    for a_ in ax.flat:
        a_.grid(True, alpha=0.3)
    fig.tight_layout()
    out = Path(__file__).resolve().parent.parent / "img" / "pv_mppt_po_curves.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f"  {'figure':<30}= {out}")
    if matplotlib_backend_interactive():
        plt.show()
    if not ok:
        print("校验失败：MPP 扫描或平均模型自洽检查超限", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
