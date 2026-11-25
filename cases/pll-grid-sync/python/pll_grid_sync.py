#!/usr/bin/env python3
"""三相电网同步 SRF-PLL 锁相与动态跟踪（教学演示）。

信号链::

    va, vb, vc ──Clarke(等幅)──► vα, vβ ──Park(θ̂)──► vd, vq
                                                      │
                     θ̂ ◄──∫──◄ ω̂ = ω0 + Kp·vq + Ki·∫vq ◄┘   （vq → 0 即锁定）

    vα = (2/3)(va − vb/2 − vc/2)，vβ = (vb − vc)/√3
    vd =  vα·cos θ̂ + vβ·sin θ̂ = Vm·cos(θg − θ̂)
    vq = −vα·sin θ̂ + vβ·cos θ̂ = Vm·sin(θg − θ̂) ≈ Vm·(θg − θ̂)   （va = Vm·cos θg）

小信号线性化后是一个带零点的二阶系统::

    θ̂/θg = (Kp·Vm·s + Ki·Vm) / (s² + Kp·Vm·s + Ki·Vm)
    ωn² = Ki·Vm，2ζωn = Kp·Vm   ⇒   Kp = 2ζωn/Vm，Ki = ωn²/Vm

    相位误差 e = θg − θ̂：
      相位阶跃 Δθ   → e 从 Δθ 衰减到 0（有反向超调）
      频率阶跃 Δω   → 稳态 e = 0（PI 里含积分，Ⅱ型系统）
      频率斜坡 R    → 稳态 e = 2πR/(Ki·Vm) = 2πR/ωn²（R 单位 Hz/s）

脚本做的事：
  1. 定步长 RK4（dt = 10 μs）仿真 PLL 两个状态（∫vq、θ̂），电网电压按解析式给出；
  2. 工况 A 相位跳变 +30°、B 频率阶跃 50→49.5 Hz、C 频率斜坡 −5 Hz/s（100 ms）、
     D 电压暂降到 0.5 pu 同时跳变 +30°（有 / 无幅值归一化对比）、
     E 10% 负序 + 5% 5 次谐波（vq 出现 100 Hz / 300 Hz 脉动）；
  3. 用 scipy.signal 的线性模型核对锁定时间、超调、斜坡稳态误差、脉动幅值；
  4. 带宽 fn = ωn/2π 扫描与阻尼比 ζ 扫描。

用法（在仓库根目录或本目录均可）::

    python cases/pll-grid-sync/python/pll_grid_sync.py

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
from scipy import signal

# ---- 电网（教学示例，不是实测数据）----
V_LL = 380.0  # V，线电压有效值
VM = math.sqrt(2.0) * V_LL / math.sqrt(3.0)  # 相电压峰值 310.27 V
F0 = 50.0
W0 = 2.0 * math.pi * F0
# ---- PLL 设计 ----
FN = 20.0  # Hz，自然频率 ωn/2π
ZETA = 1.0 / math.sqrt(2.0)
# ---- 数值 ----
DT = 1e-5
T_EV = 0.10  # s，扰动时刻
T_END = 0.40
BAND_DEG = 2.0  # 相位锁定带 ±2°
BAND_HZ = 0.1  # 频率锁定带 ±0.1 Hz
TOL_LIN = 0.05  # 小信号工况（B、C）非线性仿真与线性模型的相对偏差上限

SH = (0.0, -2.0 * math.pi / 3.0, 2.0 * math.pi / 3.0)


def gains(fn=FN, zeta=ZETA, vm=VM):
    wn = 2.0 * math.pi * fn
    return 2.0 * zeta * wn / vm, wn * wn / vm, wn


def grid_theta(t, case):
    """电网正序相位 θg(t) 与频率 fg(t)。"""
    df = case.get("df", 0.0)  # Hz，频率阶跃
    rr = case.get("ramp", 0.0)  # Hz/s
    tr = case.get("t_ramp", 0.0)  # s，斜坡持续
    jump = math.radians(case.get("jump", 0.0))
    th = W0 * t
    f = F0
    if t >= T_EV:
        tau = t - T_EV
        th += jump + 2.0 * math.pi * df * tau
        f += df
        if rr:
            tt = min(tau, tr)
            th += 2.0 * math.pi * (0.5 * rr * tt * tt + rr * tr * (tau - tt))
            f += rr * tt
    return th, f


def grid_abc(t, case):
    th, f = grid_theta(t, case)
    v = VM * (case.get("sag", 1.0) if t >= T_EV else 1.0)
    k2, k5 = case.get("k2", 0.0), case.get("k5", 0.0)
    out = []
    for s in SH:
        x = math.cos(th + s)
        if k2:
            x += k2 * math.cos(th - s)  # 负序
        if k5:
            x += k5 * math.cos(5.0 * (th + s))  # 5 次谐波（自身是负序）
        out.append(v * x)
    return out, th, f


def clarke(va, vb, vc):
    return (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc), (vb - vc) / math.sqrt(3.0)


def simulate(case, fn=FN, zeta=ZETA, norm=False, dt=DT, t_end=T_END):
    """RK4 仿真。返回字典：t、e（相位误差 rad，已折算到 ±π）、f_pll、f_grid、vd、vq。"""
    kp, ki, _ = gains(fn, zeta)

    def vdq(t, th):
        (va, vb, vc), _, _ = grid_abc(t, case)
        al, be = clarke(va, vb, vc)
        c, s = math.cos(th), math.sin(th)
        vd = al * c + be * s
        vq = -al * s + be * c
        if norm:  # 幅值归一化：vq 除以 |vαβ|，再乘 VM 使额定点增益不变
            vq = vq * VM / max(math.hypot(al, be), 1e-6)
        return vd, vq

    def rhs(t, xi, th):
        _, vq = vdq(t, th)
        return vq, W0 + kp * vq + ki * xi

    n = int(round(t_end / dt))
    t = np.arange(n + 1) * dt
    xi, th = 0.0, 0.0  # 从锁定状态起算（θ̂ = θg，∫vq = 0）
    XI = np.empty(n + 1)
    TH = np.empty(n + 1)
    XI[0], TH[0] = xi, th
    for k in range(n):
        tk = t[k]
        a1, b1 = rhs(tk, xi, th)
        a2, b2 = rhs(tk + dt / 2, xi + dt / 2 * a1, th + dt / 2 * b1)
        a3, b3 = rhs(tk + dt / 2, xi + dt / 2 * a2, th + dt / 2 * b2)
        a4, b4 = rhs(tk + dt, xi + dt * a3, th + dt * b3)
        xi += dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        th += dt / 6 * (b1 + 2 * b2 + 2 * b3 + b4)
        XI[k + 1], TH[k + 1] = xi, th
    # 事后计算输出量（用状态而不是 RK4 中间量）
    thg = np.empty(n + 1)
    fg = np.empty(n + 1)
    vd = np.empty(n + 1)
    vq = np.empty(n + 1)
    for k in range(n + 1):
        thg[k], fg[k] = grid_theta(t[k], case)
        vd[k], vq[k] = vdq(t[k], TH[k])
    e = np.angle(np.exp(1j * (thg - TH)))
    w_pll = W0 + kp * vq + ki * XI
    return dict(t=t, e=e, f_pll=w_pll / (2 * math.pi), f_grid=fg, vd=vd, vq=vq, kp=kp, ki=ki)


def lock_time(t, err, band, t0=T_EV):
    """事件后误差最后一次越出 ±band 的时刻（相对 t0）；从未越出返回 0。"""
    m = t >= t0
    tt, ee = t[m], np.abs(err[m])
    out = np.nonzero(ee > band)[0]
    if out.size == 0:
        return 0.0
    k = out[-1]
    if k + 1 >= tt.size:
        return math.inf
    return float(tt[k + 1] - t0)


def lin_models(fn=FN, zeta=ZETA, scale=1.0):
    """线性模型。scale = 实际电压 / VM（无归一化暂降时环路增益按比例变化）。"""
    kp, ki, _ = gains(fn, zeta)
    a1, a0 = kp * VM * scale, ki * VM * scale
    den = [1.0, a1, a0]
    H = signal.lti([a1, a0], den)  # θ̂/θg
    E = signal.lti([1.0, 0.0, 0.0], den)  # e/θg
    return H, E, a1, a0


def lin_phase_step(jump_deg, fn=FN, zeta=ZETA, scale=1.0):
    _, E, a1, a0 = lin_models(fn, zeta, scale)
    tt = np.arange(0.0, 0.3 + DT / 2, DT)
    _, y = signal.step(E, T=tt)
    e = math.radians(jump_deg) * y
    w_dev = -np.gradient(e, tt)  # ω̂ − ωg = d(θ̂ − θg)/dt = −de/dt（t > 0）
    return tt, e, w_dev / (2 * math.pi)


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


def phase_metrics(r, jump_deg):
    t, e = r["t"], np.degrees(r["e"])
    ferr = r["f_pll"] - r["f_grid"]
    m = t > T_EV
    lt_ph = lock_time(t, e, BAND_DEG)
    lt_f = lock_time(t, ferr, BAND_HZ)
    em = e[m]
    if jump_deg > 0:
        os_pct = max(0.0, -em.min()) / jump_deg * 100.0
    else:
        os_pct = 0.0
    return dict(lt_ph=lt_ph, lt_f=lt_f, lt=max(lt_ph, lt_f), os=os_pct,
                fdev=float(np.max(np.abs(ferr[m]))), emax=float(np.max(np.abs(em))))


def steady_ripple(r, t0=0.30):
    m = r["t"] >= t0
    e = np.degrees(r["e"][m])
    return dict(e_pp=float(e.max() - e.min()), f_pp=float(r["f_pll"][m].max() - r["f_pll"][m].min()),
                vq_pp=float(r["vq"][m].max() - r["vq"][m].min()), vd_mean=float(r["vd"][m].mean()))


def main() -> int:
    _enable_cjk()
    ok = True
    kp, ki, wn = gains()
    print("SRF-PLL grid synchronization (teaching example)")
    _p("Vm_phase_peak_V", VM)
    _p("fn_Hz", FN, ".1f")
    _p("zeta", ZETA, ".4f")
    _p("wn_rad_s", wn)
    _p("Kp_rad_s_per_V", kp, ".5f")
    _p("Ki_rad_s2_per_V", ki, ".4f")
    _p("Kp_Vm (=2 zeta wn)", kp * VM)
    _p("Ki_Vm (=wn^2)", ki * VM, ".2f")
    H, E, a1, a0 = lin_models()
    w_bw = np.logspace(0, 4, 20000)
    _, mag, _ = signal.bode(H, w=w_bw)
    f3db = w_bw[np.nonzero(mag < -3.0)[0][0]] / (2 * math.pi)
    _p("closed_loop_-3dB_Hz", f3db, ".2f")

    # ---------- A：相位跳变 +30° ----------
    print("[A] 相位跳变 +30°")
    rA = simulate({"jump": 30.0})
    mA = phase_metrics(rA, 30.0)
    tl, el, fl = lin_phase_step(30.0)
    el_d = np.degrees(el)
    lin_lt = lock_time(tl, el_d, BAND_DEG, 0.0)
    lin_os = -el_d.min() / 30.0 * 100
    _p("lock_time_phase_2deg_ms", mA["lt_ph"] * 1e3, ".2f")
    _p("lock_time_freq_0.1Hz_ms", mA["lt_f"] * 1e3, ".2f")
    _p("overshoot_pct", mA["os"], ".2f")
    _p("peak_freq_dev_Hz", mA["fdev"], ".3f")
    _p("peak_freq_dev_theory_Hz", kp * VM * math.sin(math.radians(30)) / (2 * math.pi), ".3f")
    _p("linear_lock_time_phase_ms", lin_lt * 1e3, ".2f")
    _p("linear_overshoot_pct", lin_os, ".2f")
    _p("linear_peak_freq_dev_Hz", a1 * math.radians(30) / (2 * math.pi), ".3f")

    # ---------- B：频率阶跃 50 → 49.5 Hz ----------
    print("[B] 频率阶跃 50 → 49.5 Hz")
    rB = simulate({"df": -0.5})
    mB = phase_metrics(rB, 0.0)
    tt = np.arange(0.0, 0.3 + DT / 2, DT)
    # e 对频率阶跃：Θg = Δω/s² → E = Δω/(s²+a1 s+a0)，即 Δω × 1/(s²+a1 s+a0) 的冲激响应
    _, eB_lin = signal.impulse(signal.lti([1.0], [1.0, a1, a0]), T=tt)
    eB_lin = np.degrees(-2 * math.pi * 0.5 * eB_lin)
    eB_pk_sim = float(np.degrees(rB["e"][rB["t"] > T_EV]).min())
    _p("phase_err_peak_deg", eB_pk_sim, ".4f")
    _p("phase_err_peak_linear_deg", float(eB_lin.min()), ".4f")
    _p("phase_err_final_deg", float(np.degrees(rB["e"][-1])), ".2e")
    _p("lock_time_freq_0.1Hz_ms", mB["lt_f"] * 1e3, ".2f")
    _p("freq_overshoot_Hz", float(-(rB["f_pll"][rB["t"] > T_EV] - 49.5).min()), ".4f")
    _p("f_pll_final_Hz", float(rB["f_pll"][-1]), ".5f")
    if abs(eB_pk_sim - eB_lin.min()) > TOL_LIN * abs(eB_lin.min()):
        ok = False

    # ---------- C：频率斜坡 −5 Hz/s（100 ms，50 → 49.5 Hz）----------
    print("[C] 频率斜坡 −5 Hz/s，持续 100 ms")
    rC = simulate({"ramp": -5.0, "t_ramp": 0.1})
    tC = rC["t"]
    mC = (tC > 0.17) & (tC <= 0.20)
    eC_ss = float(np.degrees(rC["e"][mC]).mean())
    eC_th = math.degrees(2 * math.pi * -5.0 / (ki * VM))
    _p("ramp_ss_err_sim_deg", eC_ss, ".4f")
    _p("ramp_ss_err_theory_deg", eC_th, ".4f")
    fC = rC["f_pll"] - rC["f_grid"]
    _p("ramp_freq_lag_Hz", float(fC[mC].mean()), ".2e")
    _p("phase_err_peak_deg", float(np.degrees(rC["e"][tC > T_EV]).min()), ".4f")
    _p("freq_err_max_Hz", float(np.abs(fC[tC > T_EV]).max()), ".4f")
    _p("phase_err_final_deg", float(np.degrees(rC["e"][-1])), ".2e")
    _p("after_ramp_err_peak_deg", float(np.degrees(rC["e"][tC > 0.2]).min()), ".4f")
    if abs(eC_ss - eC_th) > TOL_LIN * abs(eC_th):
        ok = False

    # ---------- D：电压暂降 0.5 pu + 跳变 +30° ----------
    print("[D] 电压暂降到 0.5 pu 同时相位跳变 +30°")
    caseD = {"jump": 30.0, "sag": 0.5}
    rD = simulate(caseD)
    rDn = simulate(caseD, norm=True)
    mD, mDn = phase_metrics(rD, 30.0), phase_metrics(rDn, 30.0)
    _, El, a1s, a0s = lin_models(scale=0.5)
    _p("eff_wn_rad_s (no norm)", math.sqrt(a0s), ".3f")
    _p("eff_zeta (no norm)", a1s / (2 * math.sqrt(a0s)), ".4f")
    for lab, mm in (("no_norm", mD), ("norm", mDn)):
        _p(f"{lab}_lock_time_phase_ms", mm["lt_ph"] * 1e3, ".2f")
        _p(f"{lab}_lock_time_freq_ms", mm["lt_f"] * 1e3, ".2f")
        _p(f"{lab}_overshoot_pct", mm["os"], ".2f")
        _p(f"{lab}_peak_freq_dev_Hz", mm["fdev"], ".3f")
    _p("vd_after_sag_V", float(rD["vd"][-1]), ".2f")

    # ---------- E：10% 负序 + 5% 5 次谐波 ----------
    print("[E] 10% 负序 + 5% 5 次谐波（稳态，t ≥ 0.3 s）")
    caseE = {"k2": 0.10, "k5": 0.05}
    rE = simulate(caseE)
    sE = steady_ripple(rE)
    # 线性预测：vq 扰动 ≈ Vm·[k2·sin(−2θg) + k5·sin(−6θg)]，θ̂ = H(s)·扰动/Vm
    _, H2 = signal.freqresp(H, w=[2 * W0, 6 * W0])
    a2_lin = 0.10 * abs(H2[0])
    a6_lin = 0.05 * abs(H2[1])
    _p("vq_pp_V", sE["vq_pp"], ".2f")
    _p("phase_err_pp_deg", sE["e_pp"], ".3f")
    _p("f_pll_pp_Hz", sE["f_pp"], ".3f")
    _p("lin_|H(j2w)|", abs(H2[0]), ".4f")
    _p("lin_|H(j6w)|", abs(H2[1]), ".4f")
    _p("lin_err_amp_100Hz_deg", math.degrees(a2_lin), ".3f")
    _p("lin_err_amp_300Hz_deg", math.degrees(a6_lin), ".3f")
    # 频谱核对（0.2~0.4 s，整 10 个周期）
    mE = rE["t"] >= 0.2
    eE = np.degrees(rE["e"][mE][:-1])
    spec = np.abs(np.fft.rfft(eE - eE.mean())) * 2 / eE.size
    fr = np.fft.rfftfreq(eE.size, DT)
    amp100 = float(spec[np.argmin(np.abs(fr - 100))])
    amp300 = float(spec[np.argmin(np.abs(fr - 300))])
    _p("fft_err_amp_100Hz_deg", amp100, ".3f")
    _p("fft_err_amp_300Hz_deg", amp300, ".3f")
    if abs(amp100 - math.degrees(a2_lin)) > 0.1 * math.degrees(a2_lin):
        ok = False

    # ---------- 带宽扫描 ----------
    print("[sweep fn] ζ = 0.707：A 相位跳变 / E 畸变 / C 斜坡")
    fns = [5.0, 10.0, 20.0, 40.0, 80.0]
    sw = []
    for fn in fns:
        kpx, kix, _ = gains(fn)
        ra = phase_metrics(simulate({"jump": 30.0}, fn=fn, t_end=0.6 if fn < 10 else T_END), 30.0)
        re = steady_ripple(simulate(caseE, fn=fn, t_end=0.6 if fn < 10 else T_END),
                           t0=0.5 if fn < 10 else 0.30)
        ramp_th = math.degrees(2 * math.pi * 5.0 / (kix * VM))
        sw.append((fn, kpx, kix, ra["lt"], ra["fdev"], re["e_pp"], re["f_pp"], ramp_th))
        print(f"  fn={fn:5.1f} Hz  Kp={kpx:8.5f}  Ki={kix:9.4f}  lock(±2°&±0.1Hz)={ra['lt'] * 1e3:7.2f} ms  "
              f"fdev={ra['fdev']:7.3f} Hz  E: e_pp={re['e_pp']:6.3f}°  f_pp={re['f_pp']:7.3f} Hz  "
              f"ramp5Hz/s_err={ramp_th:6.3f}°")
    print("[sweep zeta] fn = 20 Hz：A 相位跳变")
    zs = [0.5, ZETA, 1.0]
    zr = []
    for z in zs:
        mz = phase_metrics(simulate({"jump": 30.0}, zeta=z), 30.0)
        zr.append((z, mz))
        print(f"  zeta={z:5.3f}  lock_phase={mz['lt_ph'] * 1e3:6.2f} ms  lock_all={mz['lt'] * 1e3:6.2f} ms  "
              f"overshoot={mz['os']:6.2f} %  fdev={mz['fdev']:6.3f} Hz")

    # ---------- 作图 ----------
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    # (0,0) 波形：abc / vd vq（工况 A 跳变附近）
    tA = rA["t"]
    w = (tA >= 0.08) & (tA <= 0.16)
    abc = np.array([grid_abc(x, {"jump": 30.0})[0] for x in tA[w]])
    for j, c in enumerate(("tab:red", "tab:green", "tab:blue")):
        ax[0, 0].plot((tA[w] - T_EV) * 1e3, abc[:, j], color=c, lw=0.7, alpha=0.6, label="v" + "abc"[j])
    ax[0, 0].plot((tA[w] - T_EV) * 1e3, rA["vd"][w], "k", lw=1.4, label="vd")
    ax[0, 0].plot((tA[w] - T_EV) * 1e3, rA["vq"][w], "m", lw=1.4, label="vq")
    ax[0, 0].set_title("工况 A：三相电压与 Park 输出 vd、vq（+30° 跳变）", fontsize=9)
    ax[0, 0].set_xlabel("t − t_ev / ms")
    ax[0, 0].set_ylabel("V")
    ax[0, 0].legend(fontsize=7, ncol=5, loc="lower right")
    # (0,1) A 相位误差 + 频率
    ta = (tA - T_EV) * 1e3
    w = (tA >= 0.09) & (tA <= 0.20)
    ax[0, 1].plot(ta[w], np.degrees(rA["e"][w]), "k", lw=1.3, label="相位误差（非线性仿真）")
    ax[0, 1].plot(tl[tl <= 0.1] * 1e3, el_d[tl <= 0.1], "c--", lw=1.0, label="线性模型")
    ax[0, 1].axhspan(-BAND_DEG, BAND_DEG, color="g", alpha=0.15, label="±2° 锁定带")
    ax[0, 1].axvline(mA["lt_ph"] * 1e3, color="g", ls=":", lw=0.8)
    ax[0, 1].set_ylabel(r"$\theta_g-\hat{\theta}$ / °")
    ax[0, 1].set_xlabel("t − t_ev / ms")
    a2 = ax[0, 1].twinx()
    a2.plot(ta[w], rA["f_pll"][w], "tab:orange", lw=0.9, label=r"$f_{pll}$")
    a2.set_ylabel(r"$f_{pll}$ / Hz", color="tab:orange")
    ax[0, 1].set_title(f"工况 A：相位跳变 +30°，锁定 {mA['lt'] * 1e3:.1f} ms，超调 {mA['os']:.1f}%", fontsize=9)
    ax[0, 1].legend(fontsize=7, loc="upper right")
    # (0,2) B / C 频率跟踪
    tb = rB["t"]
    w = (tb >= 0.08) & (tb <= 0.30)
    ax[0, 2].plot(tb[w] * 1e3, rB["f_grid"][w], "k:", lw=1.0, label="B 电网 f（阶跃）")
    ax[0, 2].plot(tb[w] * 1e3, rB["f_pll"][w], "tab:blue", lw=1.2, label="B PLL 输出频率")
    ax[0, 2].plot(tb[w] * 1e3, rC["f_grid"][w], "k--", lw=0.8, label="C 电网 f（−5 Hz/s 斜坡）")
    ax[0, 2].plot(tb[w] * 1e3, rC["f_pll"][w], "tab:red", lw=1.2, label="C PLL 输出频率")
    ax[0, 2].set_ylim(49.35, 50.08)
    ax[0, 2].set_xlabel("t / ms")
    ax[0, 2].set_ylabel("f / Hz")
    a3 = ax[0, 2].twinx()
    a3.plot(tb[w] * 1e3, np.degrees(rB["e"][w]), "tab:blue", lw=0.7, alpha=0.5)
    a3.plot(tb[w] * 1e3, np.degrees(rC["e"][w]), "tab:red", lw=0.7, alpha=0.5)
    a3.set_ylabel("相位误差 / °（细线）")
    ax[0, 2].set_title(f"B/C：频率阶跃零稳态误差；斜坡稳态误差 {eC_ss:.3f}°", fontsize=9)
    ax[0, 2].legend(fontsize=7, loc="lower left")
    # (1,0) D
    td = rD["t"]
    w = (td >= 0.09) & (td <= 0.25)
    ax[1, 0].plot((td[w] - T_EV) * 1e3, np.degrees(rD["e"][w]), "tab:red", lw=1.2,
                  label=f"无归一化（锁定 {mD['lt'] * 1e3:.1f} ms，超调 {mD['os']:.1f}%）")
    ax[1, 0].plot((td[w] - T_EV) * 1e3, np.degrees(rDn["e"][w]), "tab:blue", lw=1.2,
                  label=f"幅值归一化（锁定 {mDn['lt'] * 1e3:.1f} ms，超调 {mDn['os']:.1f}%）")
    ax[1, 0].axhspan(-BAND_DEG, BAND_DEG, color="g", alpha=0.15)
    ax[1, 0].set_title("工况 D：暂降 0.5 pu + 跳变 +30°", fontsize=9)
    ax[1, 0].set_xlabel("t − t_ev / ms")
    ax[1, 0].set_ylabel(r"$\theta_g-\hat{\theta}$ / °")
    ax[1, 0].legend(fontsize=7)
    # (1,1) E
    te = rE["t"]
    w = (te >= 0.30) & (te <= 0.34)
    ax[1, 1].plot((te[w] - 0.3) * 1e3, rE["vq"][w], "m", lw=1.0, label=f"vq（峰峰 {sE['vq_pp']:.1f} V）")
    ax[1, 1].plot((te[w] - 0.3) * 1e3, rE["vd"][w] - VM, "k", lw=0.8, label="vd − Vm")
    ax[1, 1].set_xlabel("t − 0.3 s / ms")
    ax[1, 1].set_ylabel("V")
    a4 = ax[1, 1].twinx()
    a4.plot((te[w] - 0.3) * 1e3, rE["f_pll"][w], "tab:orange", lw=1.0, label=f"PLL 频率（峰峰 {sE['f_pp']:.2f} Hz）")
    a4.set_ylabel(r"$f_{pll}$ / Hz", color="tab:orange")
    ax[1, 1].set_title(f"工况 E：10% 负序 + 5% 5 次谐波，相位误差峰峰 {sE['e_pp']:.2f}°", fontsize=9)
    h1, l1 = ax[1, 1].get_legend_handles_labels()
    h2, l2 = a4.get_legend_handles_labels()
    ax[1, 1].legend(h1 + h2, l1 + l2, fontsize=7, loc="lower right")
    # (1,2) 带宽扫描
    fa = np.array(sw)
    ax[1, 2].loglog(fa[:, 0], fa[:, 3] * 1e3, "ko-", label="A 锁定时间 / ms")
    ax[1, 2].loglog(fa[:, 0], fa[:, 5], "rs-", label="E 相位误差峰峰 / °")
    ax[1, 2].loglog(fa[:, 0], fa[:, 7], "b^-", label="−5 Hz/s 斜坡稳态误差 / °")
    ax[1, 2].axvline(FN, color="gray", ls=":")
    ax[1, 2].set_xticks(fns)
    ax[1, 2].set_xticklabels([f"{x:g}" for x in fns])
    ax[1, 2].minorticks_off()
    ax[1, 2].set_xlabel("fn = ωn/2π / Hz（ζ = 0.707）")
    ax[1, 2].set_title("带宽取舍：快 ↔ 抗畸变", fontsize=9)
    ax[1, 2].legend(fontsize=7)
    for a_ in ax.flat:
        a_.grid(True, alpha=0.3)
    fig.tight_layout()
    out = Path(__file__).resolve().parent.parent / "img" / "pll_grid_sync_curves.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"  {'figure':<30}= {out}")
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        plt.show()
    if not ok:
        print("校验失败：非线性仿真与线性模型偏差超限", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
