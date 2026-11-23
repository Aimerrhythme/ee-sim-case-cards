#!/usr/bin/env python3
"""无限大电源供电系统三相短路电流暂态（教学演示）。

每相等效电路（三相对称、Y 接中性点接地，三相互不耦合，各相单独列方程）::

    e_x(t) ──[ R ]──( L )──●──[ Rl ]──( Ll )── N
                           │
                           K（t = t_f 三相同时短路，把负荷短接）

    短路前： (L + Ll)·di/dt = e − (R + Rl)·i
    短路后：        L·di/dt = e − R·i          （线路电流 i 连续）

电源相电压 e_a = Em·sin(ω(t − t_f) + α)，e_b、e_c 依次滞后 120°。
α 是短路瞬间 a 相电压相位（教材里的「合闸角 / 短路初相角」）。

教材解析解（短路后 t' = t − t_f）::

    i_a(t') = Ipm·sin(ωt' + α − φk) + [I0m·sin(α − φ0) − Ipm·sin(α − φk)]·e^(−t'/Ta)
              └────── 周期分量 ──────┘   └──────────── 非周期分量 i_ap0 ────────────┘

    Ipm = Em/|R + jX|，φk = arctan(X/R)，Ta = L/R
    冲击电流 i_im ≈ Kim·Ipm，Kim = 1 + e^(−0.01/Ta)（空载、最严重相、半周期后）

脚本做的事：
  1. 定步长 RK4（dt = 10 μs）仿真三相，短路前从稳态起算，短路后算 15 个周期；
     与解析解逐点比对，偏差超限返回非零。
  2. 工况 A：a 相非周期分量最大的 α（含负荷电流）；工况 B：a 相非周期分量为零的 α；
     工况 C：空载（短路前无电流）+ α = φk + 90°（与 φk − 90° 只差符号），复现教材 Kim。
  3. α 扫描（0~180°，三相最大瞬时值）与 X/R 扫描（冲击系数 Kim）。

用法（在仓库根目录或本目录均可）::

    python cases/three-phase-short-circuit/python/three_phase_short_circuit.py

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

# ---- 系统（教学示例，不是某个变电站的实测参数）----
U_N = 10.5e3  # V，线电压有效值（无限大电源母线）
F = 50.0  # Hz
W = 2.0 * math.pi * F
E_PH = U_N / math.sqrt(3.0)  # 相电压有效值
EM = math.sqrt(2.0) * E_PH  # 相电压峰值
# ---- 短路回路（电源到短路点的线路 / 变压器等效）----
X = 0.5  # Ω
TA = 0.05  # s，非周期分量时间常数 Ta = L/R（X/R = ωTa ≈ 15.7）
L = X / W
R = L / TA
# ---- 短路前负荷（每相 Y 接）----
RL, XL = 13.6, 6.6  # Ω
LL = XL / W
# ---- 数值 ----
DT = 1e-5  # s
T = 1.0 / F
NPC = int(round(T / DT))
T_F = 2 * T  # 短路时刻（先跑 2 个周期稳态）
N_POST = 15  # 短路后周期数
T_END = T_F + N_POST * T
TOL_REL = 1e-6  # RK4 与解析解最大偏差 / Ipm

ZK = complex(R, X)
Z0 = complex(R + RL, X + XL)
IPM = EM / abs(ZK)  # 短路电流周期分量幅值
I0M = EM / abs(Z0)  # 短路前负荷电流幅值
PHI_K = math.atan2(X, R)
PHI_0 = math.atan2(X + XL, R + RL)
SHIFT = np.radians([0.0, -120.0, 120.0])  # a、b、c 相电压初相偏移
KIM_TEXT = 1.0 + math.exp(-0.01 / TA)


def i_dc0(alpha, i0m=I0M, phi0=PHI_0):
    """短路瞬间非周期分量初值（a 相，alpha 为弧度）。"""
    return i0m * np.sin(alpha - phi0) - IPM * np.sin(alpha - PHI_K)


def analytic(tp, alpha, i0m=I0M, phi0=PHI_0, ta=TA, ipm=IPM, phik=PHI_K):
    """短路后 a 相解析解（tp 为短路后时间，可为数组）。"""
    dc0 = i0m * np.sin(alpha - phi0) - ipm * np.sin(alpha - phik)
    return ipm * np.sin(W * tp + alpha - phik) + dc0 * np.exp(-tp / ta)


def simulate(alpha_deg, with_load=True):
    """三相定步长 RK4。返回 t (N,)、i (3, N)。"""
    a = math.radians(alpha_deg) + SHIFT  # 短路瞬间各相电压相位
    if with_load:
        rpre, lpre, z0 = R + RL, L + LL, Z0
    else:  # 空载：短路前线路电流为零
        rpre, lpre, z0 = R, L, None
    n = int(round(T_END / DT))
    t = np.arange(n + 1) * DT
    # t = 0 时刻的稳态初值：e 相位 = a − ωt_f
    th0 = a - W * T_F
    if z0 is None:
        i = np.zeros(3)
    else:
        i = EM / abs(z0) * np.sin(th0 - math.atan2(z0.imag, z0.real))
    out = np.empty((3, n + 1))
    out[:, 0] = i
    k_f = int(round(T_F / DT))

    def f(tt, ii, r, l):
        e = EM * np.sin(W * (tt - T_F) + a)
        return (e - r * ii) / l

    for k in range(n):
        tk = t[k]
        if k < k_f:
            if z0 is None:
                out[:, k + 1] = 0.0
                continue
            r, l = rpre, lpre
        else:
            r, l = R, L
        k1 = f(tk, i, r, l)
        k2 = f(tk + DT / 2, i + DT / 2 * k1, r, l)
        k3 = f(tk + DT / 2, i + DT / 2 * k2, r, l)
        k4 = f(tk + DT, i + DT * k3, r, l)
        i = i + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        out[:, k + 1] = i
    return t, out


def worst_alpha_scan(i0m=I0M, phi0=PHI_0, step=0.1):
    """α 扫描：首周期内三相最大 |i|（解析解，细网格）。"""
    tp = np.arange(0, T + DT / 2, DT)
    alphas = np.arange(0.0, 180.0 + step / 2, step)
    peaks = np.empty_like(alphas)
    for j, ad in enumerate(alphas):
        a = math.radians(ad) + SHIFT[:, None]
        ii = analytic(tp[None, :], a, i0m, phi0)
        peaks[j] = np.abs(ii).max()
    return alphas, peaks


def kim_exact(xr):
    """空载、最严重 α 下 首周期峰值 / Ipm（精确数值）与教材近似。"""
    ta = xr / W
    tp = np.arange(0, T + 1e-6, 1e-6)
    best = 0.0
    phik = math.atan(xr)
    for ad in np.arange(-30.0, 30.01, 0.1):  # 最严重点在 α ≈ φk − 90° 附近
        a = phik - math.pi / 2 + math.radians(ad)
        ii = analytic(tp, a, 0.0, 0.0, ta, 1.0, phik)
        best = max(best, float(np.abs(ii).max()))
    return best, 1.0 + math.exp(-0.01 / ta)


def per_cycle_dc(i_post):
    """短路后每个周期的平均值（非周期分量，近似）。"""
    nc = (i_post.size - 1) // NPC
    return i_post[: nc * NPC].reshape(nc, NPC).mean(axis=1)


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
    print(f"  {key:<28}= {v:{fmt}}")


def report(name, lab, alpha_deg, with_load, t, I):
    """打印一个工况，返回校验是否通过和作图要用的量。"""
    k_f = int(round(T_F / DT))
    tp = t[k_f:] - T_F
    post = I[:, k_f:]
    a = math.radians(alpha_deg) + SHIFT
    i0m = I0M if with_load else 0.0
    ref = np.vstack([analytic(tp, a[j], i0m) for j in range(3)])
    err = float(np.abs(post - ref).max()) / IPM
    dc0 = np.array([i_dc0(a[j], i0m) for j in range(3)])
    print(f"[{name}] {lab}")
    _p("alpha_deg", alpha_deg)
    _p("pre_fault_I_rms_A", (I0M if with_load else 0.0) / math.sqrt(2))
    for j, ph in enumerate("abc"):
        k = int(np.argmax(np.abs(post[j])))
        _p(f"dc0_{ph}_kA", dc0[j] / 1e3, ".4f")
        _p(f"peak_{ph}_kA", post[j, k] / 1e3, ".4f")
        _p(f"t_peak_{ph}_ms", tp[k] * 1e3)
    _p("dc0_sum_A", float(dc0.sum()), ".1e")
    j = int(np.argmax(np.abs(post).max(axis=1)))
    k = int(np.argmax(np.abs(post[j])))
    ipk = abs(post[j, k])
    print(f"  {'worst_phase':<28}= {'abc'[j]}")
    _p("i_im_kA", ipk / 1e3, ".4f")
    _p("Kim_measured", ipk / IPM, ".4f")
    # 冲击电流有效值：以峰值时刻为中心一个周期内的方均根
    lo, hi = max(0, k - NPC // 2), k + NPC // 2
    i_rms_win = float(np.sqrt(np.mean(post[j, lo:hi] ** 2)))
    _p("I_im_rms_window_kA", i_rms_win / 1e3, ".4f")
    kim = ipk / IPM
    _p("I_im_rms_formula_kA", IPM / math.sqrt(2) * math.sqrt(1 + 2 * (kim - 1) ** 2) / 1e3, ".4f")
    dcc = per_cycle_dc(post[j])
    _p("cyc_mean_1_kA", dcc[0] / 1e3, ".4f")
    _p("cyc_mean_5_kA", dcc[4] / 1e3, ".4f")
    _p("cyc_mean_10_kA", dcc[9] / 1e3, ".4f")
    if abs(dc0[j]) > 0.05 * IPM:
        idx = np.arange(10)
        slope = np.polyfit((idx + 0.5) * T, np.log(np.abs(dcc[idx])), 1)[0]
        _p("Ta_fit_ms", -1e3 / slope)
        r = np.abs(np.exp(-tp / TA) * dc0[j])
        k_lo = np.nonzero(r < 0.05 * IPM)[0]
        _p("dc_below_5pct_Ipm_ms", tp[k_lo[0]] * 1e3 if k_lo.size else -1.0)
    _p("rk4_vs_analytic_max_rel", err, ".1e")
    return err <= TOL_REL, dict(t=t, I=I, tp=tp, post=post, dc0=dc0, a=a, i0m=i0m, j=j, k=k)


def main() -> int:
    print("Three-phase symmetrical short circuit fed by an infinite bus")
    _p("E_phase_rms_V", E_PH, ".2f")
    _p("R_ohm", R, ".6f")
    _p("L_mH", L * 1e3, ".4f")
    _p("X_over_R", X / R)
    _p("Ta_ms", TA * 1e3)
    _p("phi_k_deg", math.degrees(PHI_K))
    _p("Ipm_kA", IPM / 1e3, ".4f")
    _p("Ip_rms_kA", IPM / math.sqrt(2) / 1e3, ".4f")
    _p("S_k_MVA", math.sqrt(3) * U_N * IPM / math.sqrt(2) / 1e6, ".2f")
    _p("pre_fault_I_rms_A", I0M / math.sqrt(2))
    _p("pre_fault_pf", math.cos(PHI_0), ".4f")
    _p("Kim_textbook", KIM_TEXT, ".4f")

    alphas, peaks = worst_alpha_scan()
    # 工况 A：a 相 |非周期分量初值| 最大的 α（含负荷）
    ag = np.radians(np.arange(0.0, 360.0, 0.01))
    alpha_a = float(np.degrees(ag[np.argmax(np.abs(i_dc0(ag)))]))
    if alpha_a > 180.0:
        alpha_a -= 180.0  # 与 +180° 只差符号
    # 工况 B：a 相非周期分量为零（取 0~180° 内的根）
    d = i_dc0(np.radians(np.arange(0.0, 180.0, 0.001)))
    alpha_b = float(np.arange(0.0, 180.0, 0.001)[np.argmin(np.abs(d))])
    alpha_c = math.degrees(PHI_K) - 90.0 + 180.0  # 空载最严重，取正角
    _p("alpha_scan_worst_deg", float(alphas[np.argmax(peaks)]), ".1f")
    _p("alpha_scan_max_peak_kA", float(peaks.max()) / 1e3, ".4f")
    _p("alpha_scan_min_peak_kA", float(peaks.min()) / 1e3, ".4f")
    _p("alpha_scan_min_at_deg", float(alphas[np.argmin(peaks)]), ".1f")

    ok = True
    res = {}
    for name, ad, wl, lab in (
        ("A", alpha_a, True, f"a 相非周期分量最大 α={alpha_a:.2f}°（带负荷）"),
        ("B", alpha_b, True, f"a 相非周期分量为零 α={alpha_b:.2f}°（带负荷）"),
        ("C", alpha_c, False, f"空载 α=φk+90°={alpha_c:.2f}°（教材最严重条件）"),
    ):
        t, I = simulate(ad, wl)
        good, r = report(name, lab, ad, wl, t, I)
        ok &= good
        res[name] = (lab, r)

    print("[X/R sweep] 空载、最严重 α，首周期峰值 / Ipm")
    xr_list = [3.0, 5.0, 10.0, X / R, 30.0, 50.0]
    kims = []
    for xr in xr_list:
        ke, kt = kim_exact(xr)
        kims.append((xr, ke, kt))
        print(f"  X/R={xr:7.3f}  Ta={xr / W * 1e3:8.3f} ms  Kim_exact={ke:.4f}  Kim_1+e^(-0.01/Ta)={kt:.4f}")

    # ---- 作图 ----
    _enable_cjk()
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    lab, r = res["A"]
    t = r["t"]
    m = t <= T_F + 8 * T
    cols = ("tab:orange", "tab:green", "tab:red")
    for j, ph in enumerate("abc"):
        ax[0, 0].plot((t[m] - T_F) * 1e3, r["I"][j, m] / 1e3, color=cols[j], lw=1.0, label=f"i_{ph}")
    ax[0, 0].axvline(0, color="k", lw=0.8, ls=":")
    ax[0, 0].axhline(IPM / 1e3, color="gray", ls="--", lw=0.8)
    ax[0, 0].axhline(-IPM / 1e3, color="gray", ls="--", lw=0.8)
    ax[0, 0].set_title(f"工况 A 三相电流（{lab}）", fontsize=9)
    ax[0, 0].set_xlabel("t − t_f / ms")
    ax[0, 0].set_ylabel("i / kA")
    ax[0, 0].legend(fontsize=8, loc="lower right")

    j = r["j"]
    tp = r["tp"]
    mp = tp <= 8 * T
    a = r["a"][j]
    per = IPM * np.sin(W * tp + a - PHI_K)
    dc = r["dc0"][j] * np.exp(-tp / TA)
    ax[0, 1].plot(tp[mp] * 1e3, r["post"][j, mp] / 1e3, color=cols[j], lw=1.2, label=f"全电流 i_{'abc'[j]}")
    ax[0, 1].plot(tp[mp] * 1e3, per[mp] / 1e3, "k--", lw=0.8, label="周期分量")
    ax[0, 1].plot(tp[mp] * 1e3, dc[mp] / 1e3, "b-", lw=1.2, label="非周期分量（包络中心）")
    k = r["k"]
    ax[0, 1].plot(tp[k] * 1e3, r["post"][j, k] / 1e3, "ko", ms=4)
    ax[0, 1].annotate(
        f"冲击电流 {abs(r['post'][j, k]) / 1e3:.2f} kA\n= {abs(r['post'][j, k]) / IPM:.3f}·Ipm",
        (tp[k] * 1e3, r["post"][j, k] / 1e3), xytext=(40, 0), textcoords="offset points", fontsize=8,
        arrowprops=dict(arrowstyle="->", lw=0.6),
    )
    ax[0, 1].set_title(f"工况 A 最严重相分解（Ta = {TA * 1e3:.0f} ms）", fontsize=9)
    ax[0, 1].set_xlabel("t − t_f / ms")
    ax[0, 1].set_ylabel("i / kA")
    ax[0, 1].legend(fontsize=8)

    ax[1, 0].plot(alphas, peaks / 1e3, "k", lw=1.2, label="三相最大瞬时值（首周期）")
    ax[1, 0].axhline(IPM / 1e3, color="gray", ls="--", lw=0.8)
    ax[1, 0].axhline(KIM_TEXT * IPM / 1e3, color="tab:red", ls=":", lw=0.8)
    ax[1, 0].text(2, IPM / 1e3 + 0.3, "Ipm", fontsize=8, color="gray")
    ax[1, 0].text(2, KIM_TEXT * IPM / 1e3 + 0.3, f"{KIM_TEXT:.3f}·Ipm", fontsize=8, color="tab:red")
    ax[1, 0].set_title("短路初相角 α 扫描（带负荷）", fontsize=9)
    ax[1, 0].set_xlabel("α / °（a 相电压短路瞬间相位）")
    ax[1, 0].set_ylabel("|i|max / kA")
    ax[1, 0].set_ylim(0, KIM_TEXT * IPM / 1e3 * 1.15)
    ax[1, 0].legend(fontsize=8, loc="lower right")

    xs = np.logspace(np.log10(1.0), np.log10(60.0), 60)
    ax[1, 1].semilogx(xs, 1 + np.exp(-0.01 * W / xs), "b-", lw=1.0, label="1 + e^(−0.01/Ta)")
    ax[1, 1].semilogx([k_[0] for k_ in kims], [k_[1] for k_ in kims], "ro", ms=4, label="精确数值（扫 α、细网格）")
    ax[1, 1].axvline(X / R, color="gray", ls=":", lw=0.8)
    ax[1, 1].text(X / R * 1.05, 1.1, f"本卡 X/R = {X / R:.1f}", fontsize=8, color="gray")
    ax[1, 1].set_title("冲击系数 Kim 与 X/R", fontsize=9)
    ax[1, 1].set_xlabel("X/R")
    ax[1, 1].set_ylabel("Kim")
    ax[1, 1].set_ylim(1.0, 2.0)
    ax[1, 1].legend(fontsize=8)
    for a_ in ax.flat:
        a_.grid(True, alpha=0.3)
    fig.tight_layout()
    out = Path(__file__).resolve().parent.parent / "img" / "three_phase_short_circuit_curves.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=140)
    print(f"  {'figure':<28}= {out}")
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        plt.show()
    if not ok:
        print("校验失败：RK4 与解析解偏差超限", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
