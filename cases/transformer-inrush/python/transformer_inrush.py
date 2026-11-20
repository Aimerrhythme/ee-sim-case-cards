#!/usr/bin/env python3
"""单相变压器空载合闸励磁涌流（教学演示）。

一次侧空载合闸，二次侧开路。等效电路只留绕组 / 电源电阻 R、漏感 Lσ 和
非线性励磁支路（不计铁耗、不计磁滞）::

    v(t) = Vm·sin(ωt + α) = R·i + Lσ·di/dt + dλ/dt,   i = f(λ)

f(λ) 是两段线性的 λ-i 饱和曲线（关于原点对称）::

    |λ| ≤ λk :  i = λ / Lm                （不饱和，Lm 很大）
    |λ| > λk :  i = ±[ik + (|λ| − λk)/Lair]  （饱和后只剩空心电感 Lair）

令总磁链 ψ = λ + Lσ·i，ψ 与 i 仍是两段线性关系，所以只有一个状态量::

    dψ/dt = v − R·i(ψ)

剩磁的处理：t = 0 时把铁心磁链设为 λr（单值曲线不计磁滞，i(0+) = λr/Lm，
比空载电流峰值还小，相对涌流可忽略）。

工况（α 是合闸瞬间电压相位，0° = 电压过零，90° = 电压峰值）::

    A: α = 0°,   λr = +0.8 pu   （最严重）
    B: α = 0°,   λr = 0
    C: α = 0°,   λr = −0.8 pu   （剩磁与磁链偏移方向相反）
    D: α = 90°,  λr = 0         （峰值合闸，无偏磁）
    E: α = 90°,  λr = +0.8 pu
    F: 选相合闸 α* = arccos(−λr/λN)，λr = +0.8 pu（稳态磁链在合闸瞬间正好等于剩磁）

主积分用定步长 RK4（dt = 10 μs，每周期 2000 点，6 个工况向量化一起算），
工况 A 前 0.3 s 再用 solve_ivp（Radau）核对；D、F 的峰值与空载稳态解析值比对。
偏差超限返回非零。

用法（在仓库根目录或本目录均可）::

    python cases/transformer-inrush/python/transformer_inrush.py

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

# ---- 额定值（教学示例，不是某台变压器的铭牌）----
S_N = 10e3  # VA
U_N = 220.0  # V（有效值，一次侧）
F = 50.0  # Hz
W = 2.0 * math.pi * F
Z_B = U_N**2 / S_N  # Ω，阻抗基值 4.84 Ω
I_N = S_N / U_N  # A，额定电流有效值
VM = math.sqrt(2.0) * U_N  # V，电压峰值
LAM_N = VM / W  # Wb，额定磁链峰值（磁链基值）
# ---- 等效电路（标幺值 → SI）----
R_PU, XS_PU, XM_PU, XAIR_PU = 0.01, 0.02, 50.0, 0.13
R = R_PU * Z_B
LS = XS_PU * Z_B / W  # 漏感
LM = XM_PU * Z_B / W  # 不饱和励磁电感（空载电流 2%）
LAIR = XAIR_PU * Z_B / W  # 饱和后空心电感
LAMK_PU = 1.2  # 饱和拐点 / λN
LAMK = LAMK_PU * LAM_N
IK = LAMK / LM
PSIK = LAMK + LS * IK
LR_PU = 0.8  # 剩磁 / λN
# ---- 数值 ----
DT = 1e-5  # s
T_END = 2.0  # s，100 个周期
T = 1.0 / F
NPC = int(round(T / DT))  # 每周期点数
HR_TH = (0.15, 0.20)  # 差动保护二次谐波制动常用整定范围
TOL_PEAK = 0.005  # 相对误差，RK4 与 Radau 峰值
TOL_SS = 0.01  # 相对误差，D/F 峰值与稳态解析值

ALPHA_SEL = math.degrees(math.acos(-LR_PU))
CASES = {
    "A": (0.0, +LR_PU, "α=0°，λr=+0.8"),
    "B": (0.0, 0.0, "α=0°，λr=0"),
    "C": (0.0, -LR_PU, "α=0°，λr=−0.8"),
    "D": (90.0, 0.0, "α=90°，λr=0"),
    "E": (90.0, +LR_PU, "α=90°，λr=+0.8"),
    "F": (ALPHA_SEL, +LR_PU, f"选相 α={ALPHA_SEL:.2f}°，λr=+0.8"),
}


def i_of_psi(psi):
    """总磁链 ψ → 电流（两段线性，向量化）。"""
    a = np.abs(psi)
    lin = psi / (LM + LS)
    sat = np.sign(psi) * (IK + (a - PSIK) / (LAIR + LS))
    return np.where(a <= PSIK, lin, sat)


def lam_of_i(i):
    """电流 → 铁心磁链 λ（作图用）。"""
    a = np.abs(i)
    return np.where(a <= IK, i * LM, np.sign(i) * (LAMK + (a - IK) * LAIR))


def psi0(lam_r):
    i0 = lam_r / LM  # 剩磁在不饱和段
    return lam_r + LS * i0


def rk4_all(alphas, lam_rs):
    """定步长 RK4，所有工况一起算。返回 t (N,)、i (ncase, N)。"""
    a = np.radians(np.asarray(alphas))
    psi = np.array([psi0(l) for l in lam_rs])
    n = int(round(T_END / DT))
    t = np.arange(n + 1) * DT
    i_out = np.empty((len(a), n + 1))
    i_out[:, 0] = i_of_psi(psi)

    def f(tt, p):
        return VM * np.sin(W * tt + a) - R * i_of_psi(p)

    for k in range(n):
        tk = t[k]
        k1 = f(tk, psi)
        k2 = f(tk + DT / 2, psi + DT / 2 * k1)
        k3 = f(tk + DT / 2, psi + DT / 2 * k2)
        k4 = f(tk + DT, psi + DT * k3)
        psi = psi + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        i_out[:, k + 1] = i_of_psi(psi)
    return t, i_out


def radau_check(alpha, lam_r, t_end=0.3):
    a = math.radians(alpha)
    sol = solve_ivp(
        lambda tt, p: [VM * math.sin(W * tt + a) - R * float(i_of_psi(np.array(p[0])))],
        (0.0, t_end),
        [psi0(lam_r)],
        method="Radau",
        rtol=1e-10,
        atol=1e-10,
        max_step=2e-5,
        dense_output=False,
        t_eval=np.linspace(0.0, t_end, int(round(t_end / DT)) + 1),
    )
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol.t, i_of_psi(sol.y[0])


def per_cycle(i):
    """每个工频周期：最大 |i|、基波幅值、二次谐波幅值、直流分量。"""
    nc = (i.size - 1) // NPC
    seg = i[: nc * NPC].reshape(nc, NPC)
    spec = np.fft.rfft(seg, axis=1) / NPC
    i1 = 2 * np.abs(spec[:, 1])
    i2 = 2 * np.abs(spec[:, 2])
    dc = spec[:, 0].real
    pk = np.abs(seg).max(axis=1)
    return pk, i1, i2, dc


def lam_max_pu(alpha, lam_r_pu):
    """忽略 R 时第一个周期铁心磁链最大值 / λN：λr + λN·(cos α + 1)。"""
    return lam_r_pu + math.cos(math.radians(alpha)) + 1.0


def est_peak(alpha, lam_r_pu):
    """忽略 R 的峰值粗估：超过拐点的磁链 ÷ (Lair + Lσ)，再加拐点电流。"""
    ex = (lam_max_pu(alpha, lam_r_pu) - LAMK_PU) * LAM_N
    if ex <= 0:
        return lam_max_pu(alpha, lam_r_pu) * LAM_N / LM
    return IK + ex / (LAIR + LS)


def steady_peak():
    """空载稳态（不饱和段线性）电流峰值。"""
    z = math.hypot(R, W * (LM + LS))
    return VM / z


def decay_fit(pk):
    """对前若干周期峰值做 ln(峰值) 线性拟合，取峰值 ≥ 30% 首峰的周期。"""
    m = pk >= 0.3 * pk[0]
    idx = np.nonzero(m)[0]
    idx = idx[idx == np.arange(idx.size)]  # 只取开头连续的一段
    if idx.size < 3:
        return math.nan, int(idx.size)
    tk = idx * T
    slope = np.polyfit(tk, np.log(pk[idx]), 1)[0]
    return -1.0 / slope, int(idx.size)


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
    print(f"  {key:<26}= {v:{fmt}}")


def main() -> int:
    print("Single-phase transformer no-load energization (inrush)")
    _p("I_N_A", I_N)
    _p("I_N_peak_A", math.sqrt(2) * I_N)
    _p("lambda_N_Wb", LAM_N, ".6f")
    _p("R_ohm", R, ".4f")
    _p("L_sigma_mH", LS * 1e3, ".4f")
    _p("L_m_H", LM, ".4f")
    _p("L_air_mH", LAIR * 1e3, ".4f")
    _p("lambda_knee_Wb", LAMK, ".6f")
    _p("i_knee_A", IK, ".4f")
    i0pk = steady_peak()
    _p("I0_peak_steady_A", i0pk, ".4f")
    _p("I0_rms_pct_of_IN", i0pk / math.sqrt(2) / I_N * 100)
    _p("tau_sat_ms", (LAIR + LS) / R * 1e3)
    _p("tau_lin_s", (LM + LS) / R)
    _p("alpha_select_deg", ALPHA_SEL)

    names = list(CASES)
    t, I = rk4_all([CASES[k][0] for k in names], [CASES[k][1] for k in names])
    ok = True
    res = {}
    for j, name in enumerate(names):
        alpha, lr, lab = CASES[name]
        i = I[j]
        pk, i1, i2, dc = per_cycle(i)
        hr = i2 / np.maximum(i1, 1e-12)
        k_pk = int(np.argmax(np.abs(i)))
        ipk = float(i[k_pk])
        tau, nfit = decay_fit(pk)
        below_in = np.nonzero(pk < math.sqrt(2) * I_N)[0]
        n_half = np.nonzero(pk < 0.5 * pk[0])[0]
        rows = {
            "alpha_deg": alpha,
            "lambda_r_pu": lr,
            "lam_max_noR_pu": lam_max_pu(alpha, lr),
            "est_peak_A_noR": est_peak(alpha, lr),
            "i_peak_A": ipk,
            "t_peak_ms": t[k_pk] * 1e3,
            "peak_over_IN_rms": abs(ipk) / I_N,
            "peak_over_IN_peak": abs(ipk) / (math.sqrt(2) * I_N),
            "peak_over_I0_peak": abs(ipk) / i0pk,
        }
        res[name] = (i, pk, hr, lab)
        print(f"[{name}] {lab}")
        for k, v in rows.items():
            _p(k, v)
        if pk[0] > 2 * i0pk:  # 有明显涌流才算下面这些
            _p("cyc1_I2_over_I1_pct", hr[0] * 100)
            _p("cyc1_Idc_over_I1_pct", dc[0] / i1[0] * 100)
            _p("min_I2_over_I1_pct_10cyc", hr[:10].min() * 100)
            _p("cyc20_I2_over_I1_pct", hr[19] * 100)
            _p("cyc_peak_2", pk[1])
            _p("cyc_peak_5", pk[4])
            _p("cyc_peak_10", pk[9])
            _p("cyc_peak_50", pk[49])
            _p("cyc_peak_100", pk[99])
            _p("tau_fit_ms", tau * 1e3)
            print(f"  {'tau_fit_cycles_used':<26}= {nfit}")
            print(f"  {'cycles_to_half_peak':<26}= {int(n_half[0]) if n_half.size else -1}")
            print(f"  {'cycles_to_below_IN_peak':<26}= {int(below_in[0]) if below_in.size else -1}")
        else:
            _p("cyc1_peak_A", pk[0], ".4f")
            if name in ("D", "F"):  # 应等于空载稳态峰值
                err = abs(pk[0] - i0pk) / i0pk
                _p("cyc1_vs_I0_steady_rel", err, ".1e")
                if err > TOL_SS:
                    ok = False
                    print(f"  !! {name}: 峰值偏离稳态解析值")
            _p("end_peak_A", pk[-1], ".4f")

    # Radau 核对（工况 A，前 0.3 s）
    tr, ir = radau_check(CASES["A"][0], CASES["A"][1])
    ia = I[0][: tr.size]
    rel = abs(np.abs(ir).max() - np.abs(ia).max()) / np.abs(ir).max()
    _p("A_rk4_vs_radau_peak_rel", rel, ".1e")
    _p("A_rk4_vs_radau_max_abs_A", float(np.max(np.abs(ir - ia))), ".1e")
    if rel > TOL_PEAK:
        ok = False

    # ---- 作图 ----
    _enable_cjk()
    colors = {"A": "tab:red", "B": "tab:orange", "C": "tab:green", "D": "tab:blue", "E": "tab:purple", "F": "k"}
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    m = t <= 0.2
    for name in ("A", "B", "E", "D", "F"):
        i, _, _, lab = res[name]
        ax[0, 0].plot(t[m] * 1e3, i[m], color=colors[name], lw=1.0, label=f"{name}：{lab}")
    ax[0, 0].axhline(math.sqrt(2) * I_N, color="gray", ls=":", lw=0.8)
    ax[0, 0].text(
        2, math.sqrt(2) * I_N + 12, "额定电流峰值 64.3 A", fontsize=8, color="gray",
        bbox=dict(fc="white", ec="none", alpha=0.8),
    )
    ax[0, 0].set_title("前 10 个周期的涌流波形（工况 C 峰值仅约 1.5 A，未画）")
    ax[0, 0].set_xlabel("t / ms")
    ax[0, 0].set_ylabel("i / A")
    ax[0, 0].legend(fontsize=8, loc="upper right")
    cyc = np.arange(1, res["A"][1].size + 1)
    for name in ("A", "B", "C", "E"):
        _, pk, _, lab = res[name]
        ax[0, 1].semilogy(cyc, pk, color=colors[name], lw=1.2, label=f"{name}：{lab}")
    ax[0, 1].axhline(math.sqrt(2) * I_N, color="gray", ls=":", lw=0.8)
    ax[0, 1].axhline(i0pk, color="gray", ls="--", lw=0.8)
    ax[0, 1].text(60, i0pk * 1.15, "空载稳态峰值", fontsize=8, color="gray")
    ax[0, 1].set_title("每周期电流峰值（2 s，100 个周期）")
    ax[0, 1].set_xlabel("周期序号")
    ax[0, 1].set_ylabel("|i|max / A")
    ax[0, 1].legend(fontsize=8)
    for name in ("A", "B", "E"):
        _, pk, hr, lab = res[name]
        n = 20
        ax[1, 0].plot(np.arange(1, n + 1), hr[:n] * 100, "o-", ms=3, color=colors[name], label=f"{name}：{lab}")
    for th in HR_TH:
        ax[1, 0].axhline(th * 100, color="gray", ls="--", lw=0.8)
    ax[1, 0].text(13, 22, "15% / 20% 制动整定", fontsize=8, color="gray")
    ax[1, 0].set_ylim(0, 90)
    ax[1, 0].set_title("每周期二次谐波含量 I2/I1")
    ax[1, 0].set_xlabel("周期序号")
    ax[1, 0].set_ylabel("I2/I1 / %")
    ax[1, 0].legend(fontsize=8)
    ii = np.linspace(-800, 800, 2001)
    ax[1, 1].plot(ii, lam_of_i(ii) / LAM_N, "k", lw=1.2)
    ax[1, 1].axhline(LAMK_PU, color="gray", ls=":", lw=0.8)
    ax[1, 1].axhline(LR_PU, color="tab:red", ls=":", lw=0.8)
    ax[1, 1].text(-780, LAMK_PU + 0.05, "拐点 1.2 pu", fontsize=8)
    ax[1, 1].text(-780, LR_PU - 0.18, "剩磁 0.8 pu", fontsize=8, color="tab:red")
    ax[1, 1].set_title("λ-i 饱和曲线（两段线性）")
    ax[1, 1].set_xlabel("i / A")
    ax[1, 1].set_ylabel("λ / λN")
    for a in ax.flat:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    out = Path(__file__).resolve().parent.parent / "img" / "transformer_inrush_curves.png"
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
