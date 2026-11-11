#!/usr/bin/env python3
"""三相两电平 SPWM 逆变器（教学演示）。

直流母线 Vdc，三只桥臂互补导通。载波是峰值为 1 的三角波，调制波是

    m_a = m sin(ωt)
    m_b = m sin(ωt − 2π/3)
    m_c = m sin(ωt + 2π/3)

上管在 m_x ≥ 载波时导通，下管取反。对直流中点的相电压在线性区（m ≤ 1）
一个载波周期内的平均正好是调制波乘 Vdc/2：

    V_AO,1(peak) = m · Vdc / 2
    V_LL,1(peak) = √3 · V_AO,1(peak) = (√3 / 2) · m · Vdc
    V_LL,1(rms)  = (√6 / 4) · m · Vdc

自然采样（每个载波斜坡上求调制波和三角波的交点）在线性区的基波就是上面
这几条，没有「再除一个 2」。m > 1 进入过调制，交点消失的那些载波周期被
连成宽脉冲，基波不再按 m 的直线涨，上限是六阶梯：

    V_LL,1(peak) = (2√3 / π) · Vdc

脚本做两件事。第一件用交点的分段常数波形直接积傅里叶系数，用来和公式对。
第二件在均匀时间网格上比较正弦和三角波，做 FFT、画曲线，并给星形 RL
负载积分电流。网格上的 FFT 会比公式差零点几伏里的零点零几，差在开关沿
被步长卡住，不是公式少了因子。

默认参数：Vdc = 200 V，m = 0.8，f1 = 50 Hz，mf = fc/f1 = 21，
R = 10 Ω，L = 20 mH。线电压基波峰值 = 80√3 V。

用法（在仓库根目录或本目录均可）::

    python cases/three-phase-inverter-spwm/python/three_phase_inverter_spwm.py

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

VDC = 200.0  # V，直流母线。课堂圆整值，不是 380 V 变频器的实测母线
M = 0.8  # 调制比，线性区
M_LOW = 0.5
M_UNITY = 1.0
M_OVER = 1.2
M_DEEP = 1.5
F1 = 50.0  # Hz，基波
MF = 21  # 频率调制比，奇数且是 3 的倍数：线电压里的 3 倍次边带互相抵消
R_LOAD = 10.0  # Ω
L_LOAD = 20e-3  # H（20 mH）
SPC = 1000  # 每个载波周期的采样点数，只用于波形、FFT 和电流
N_SIM_CYC = 3  # 积分 3 个基波周期，读最后一个
H_MAX = 84  # 谐波算到 84 次 = 4·mf，盖住 4 倍载波
BISECT_ITER = 60

FC = MF * F1
OMEGA = 2.0 * math.pi * F1
T_FUND = 1.0 / F1
HALF = VDC / 2.0
PHASES = (0.0, -2.0 * math.pi / 3.0, 2.0 * math.pi / 3.0)

# 自然采样相对公式的容差。交点用 60 次二分，剩下的是浮点误差。
TOL_NAT = 1e-8  # V
# 定步长 FFT 相对自然采样基波。步长大约 1 μs，开关沿量化以后允许 0.1%。
TOL_FFT = 1e-3
TOL_I = 2e-3


def tri_value(t: float, t0: float, rising: bool) -> float:
    """一个载波周期内的三角波，t0 是该周期起点（三角波 = −1）。"""
    if rising:
        return -1.0 + 4.0 * FC * (t - t0)
    return 3.0 - 4.0 * FC * (t - t0)


def pole_segments(modulation: float, phase: float, t_end: float) -> list[tuple[float, float, float]]:
    """自然采样：每段载波斜坡求一次交点，得到对直流中点的相电压 ±Vdc/2。"""
    n_car = int(round(t_end * FC))
    if abs(n_car / FC - t_end) > 1e-9:
        raise RuntimeError("分析窗口必须是载波周期的整数倍。")
    segs: list[tuple[float, float, float]] = []
    for p in range(n_car):
        t0 = p / FC
        halves = (
            (True, t0, t0 + 0.5 / FC),
            (False, t0 + 0.5 / FC, t0 + 1.0 / FC),
        )
        for rising, t_a, t_b in halves:
            def f(t: float, rising: bool = rising, t0: float = t0) -> float:
                return modulation * math.sin(OMEGA * t + phase) - tri_value(t, t0, rising)

            f_lo = f(t_a)
            f_hi = f(t_b)
            # 斜坡起点的三角波正好是 −1（上升）或 +1（下降）。
            left_on = modulation * math.sin(OMEGA * t_a + phase) >= (-1.0 if rising else 1.0)
            cross: float | None = None
            if f_lo == 0.0:
                cross = t_a
            elif f_lo * f_hi < 0.0:
                lo, hi = t_a, t_b
                f_left = f_lo
                for _ in range(BISECT_ITER):
                    mid = 0.5 * (lo + hi)
                    f_mid = f(mid)
                    if f_left * f_mid <= 0.0:
                        hi = mid
                    else:
                        lo = mid
                        f_left = f_mid
                cross = 0.5 * (lo + hi)
            if cross is None:
                level = HALF if left_on else -HALF
                segs.append((t_a, t_b, level))
            else:
                level0 = HALF if left_on else -HALF
                segs.append((t_a, cross, level0))
                segs.append((cross, t_b, -level0))
    return segs


def six_step_segments(phase: float, t_end: float) -> list[tuple[float, float, float]]:
    """六阶梯：上管在 sin(ωt+φ) ≥ 0 时一直导通，相当于 m → ∞。"""
    times = [0.0, t_end]
    n_min = math.floor(phase / math.pi) - 1
    n_max = math.ceil((OMEGA * t_end + phase) / math.pi) + 1
    for n in range(n_min, n_max + 1):
        t_zero = (n * math.pi - phase) / OMEGA
        if 0.0 < t_zero < t_end:
            times.append(t_zero)
    times.sort()
    segs: list[tuple[float, float, float]] = []
    for t_a, t_b in zip(times[:-1], times[1:]):
        mid = 0.5 * (t_a + t_b)
        positive = math.sin(OMEGA * mid + phase) >= 0.0
        segs.append((t_a, t_b, HALF if positive else -HALF))
    return segs


def fourier_all(
    segments: list[tuple[float, float, float]], h_max: int, t_end: float
) -> list[tuple[float, float]]:
    """各次谐波的 cos、sin 系数，波形写成 c·cos(hωt) + s·sin(hωt)。"""
    acc = [(0.0, 0.0) for _ in range(h_max)]
    for t0, t1, level in segments:
        if t1 <= t0:
            continue
        for h in range(1, h_max + 1):
            w = h * OMEGA
            c = level * (math.sin(w * t1) - math.sin(w * t0)) / w
            s = level * (math.cos(w * t0) - math.cos(w * t1)) / w
            c0, s0 = acc[h - 1]
            acc[h - 1] = (c0 + c, s0 + s)
    scale = 2.0 / t_end
    return [(c * scale, s * scale) for c, s in acc]


def peak(cs: tuple[float, float]) -> float:
    return math.hypot(cs[0], cs[1])


def line_cs(coeffs: list[list[tuple[float, float]]], harmonic: int, i: int = 0, j: int = 1) -> tuple[float, float]:
    """两相系数相减。harmonic 从 1 计。"""
    c1, s1 = coeffs[i][harmonic - 1]
    c2, s2 = coeffs[j][harmonic - 1]
    return c1 - c2, s1 - s2


def analyze_case(modulation: float | None) -> dict[str, float]:
    """modulation 为 None 时做六阶梯。返回线电压、相电压的基波和若干谐波。"""
    if modulation is None:
        legs = [six_step_segments(ph, T_FUND) for ph in PHASES]
    else:
        legs = [pole_segments(modulation, ph, T_FUND) for ph in PHASES]
    coeffs = [fourier_all(leg, H_MAX, T_FUND) for leg in legs]
    vao = peak(coeffs[0][0])
    ll = [peak(line_cs(coeffs, 1, i, j)) for i, j in ((0, 1), (1, 2), (2, 0))]
    h3 = peak(line_cs(coeffs, 3))
    h5 = peak(line_cs(coeffs, 5))
    h7 = peak(line_cs(coeffs, 7))
    side_n = 2
    side_v = 0.0
    thd_sq = 0.0
    kept = {19: 0.0, 23: 0.0, 43: 0.0}
    for h in range(2, H_MAX + 1):
        vh = peak(line_cs(coeffs, h))
        thd_sq += vh * vh
        if h in kept:
            kept[h] = vh
        if vh > side_v:
            side_v = vh
            side_n = h
    return {
        "Vao1": vao,
        "Vll1": ll[0],
        "Vll1_bc": ll[1],
        "Vll1_ca": ll[2],
        "h3": h3,
        "h5": h5,
        "h7": h7,
        "h19": kept[19],
        "h23": kept[23],
        "h43": kept[43],
        "side_n": float(side_n),
        "side_v": side_v,
        "thd": math.sqrt(thd_sq) / ll[0],
    }


def vll_theory(modulation: float) -> float:
    """线性区线电压基波峰值 (√3/2)·m·Vdc。m>1 时只是外推，不再成立。"""
    return (math.sqrt(3.0) / 2.0) * modulation * VDC


def vll_clip(modulation: float) -> float:
    """过调制的削顶平均模型，不是自然采样的精确值。

    把调制波限制在 ±1 里，线性区的平均值会复现这条削顶正弦。
    它的基波是 (√3·Vdc/π)·(m·arcsin(1/m) + √(1−1/m²))。
    m = 1 时回到 (√3/2)·Vdc，m → ∞ 时回到六阶梯 (2√3/π)·Vdc。
    """
    if modulation <= 1.0:
        return vll_theory(modulation)
    alpha = math.asin(1.0 / modulation)
    bracket = modulation * alpha + math.sqrt(1.0 - 1.0 / (modulation * modulation))
    return math.sqrt(3.0) * (VDC / math.pi) * bracket


def vll_six() -> float:
    return (2.0 * math.sqrt(3.0) / math.pi) * VDC


def sample_waveforms() -> dict[str, np.ndarray]:
    """均匀网格上比较正弦和三角波。返回整段 3 个基波周期。"""
    n_per = MF * SPC
    n = N_SIM_CYC * n_per
    k = np.arange(n)
    theta = 2.0 * np.pi * k / n_per
    pos = (k % SPC) / float(SPC)
    tri = 1.0 - 4.0 * np.abs(pos - 0.5)
    legs = []
    for phase in PHASES:
        mod = M * np.sin(theta + phase)
        legs.append(np.where(mod >= tri, HALF, -HALF))
    vao, vbo, vco = legs
    v_no = (vao + vbo + vco) / 3.0
    return {
        "t": k / (F1 * n_per),
        "tri": tri,
        "ma": M * np.sin(theta),
        "mb": M * np.sin(theta - 2.0 * math.pi / 3.0),
        "mc": M * np.sin(theta + 2.0 * math.pi / 3.0),
        "vao": vao,
        "vab": vao - vbo,
        "vbc": vbo - vco,
        "van": vao - v_no,
    }


def dft_cs(x: np.ndarray, harmonic: int) -> tuple[float, float]:
    """一个基波周期的网格，harmonic 次的 cos / sin 系数。"""
    n = x.size
    k = np.arange(n)
    th = 2.0 * np.pi * harmonic * k / n
    c = (2.0 / n) * float(np.dot(x, np.cos(th)))
    s = (2.0 / n) * float(np.dot(x, np.sin(th)))
    return c, s


def dft_peak(x: np.ndarray, harmonic: int) -> float:
    return peak(dft_cs(x, harmonic))


def integrate_rl(voltage: np.ndarray, dt: float) -> np.ndarray:
    """分段常数电压下 RL 的精确递推。电流样本在每个步长结束的时刻。"""
    tau = L_LOAD / R_LOAD
    a = math.exp(-dt / tau)
    b = (1.0 - a) / R_LOAD
    current = np.empty(voltage.size)
    acc = 0.0
    for i in range(voltage.size):
        acc = a * acc + b * float(voltage[i])
        current[i] = acc
    return current


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
    wave: dict[str, np.ndarray],
    i_a: np.ndarray,
    spec_f: np.ndarray,
    spec_v: np.ndarray,
    markers: list[tuple[float, float]],
    out: Path,
) -> None:
    _enable_cjk()
    fig, axes = plt.subplots(3, 2, figsize=(9.6, 9.2))
    t_ms = wave["t"] * 1e3
    n_per = MF * SPC
    # 最后一个基波周期，电流已经稳态。
    last = slice(2 * n_per, 3 * n_per)
    t_last = t_ms[last] - 40.0  # 画成 0–20 ms，方便和公式正弦对齐
    vao1 = M * HALF
    vll1 = vll_theory(M)
    # v_AB 基波 = Vll1 · cos(ωt − π/3)，t 从最后一个周期的起点算。
    theta = OMEGA * wave["t"][last]
    # 最后一个周期的起点是 t = 40 ms = 2/f1，ωt = 4π，和 0 同一相位。
    theta0 = theta - theta[0]

    ax = axes[0, 0]
    zoom = t_ms <= 3.0
    ax.plot(t_ms[zoom], wave["tri"][zoom], color="#9ca3af", lw=1.0, label="载波")
    ax.plot(t_ms[zoom], wave["ma"][zoom], color="#1f4e79", lw=1.3, label="m_a")
    ax.plot(t_ms[zoom], wave["mb"][zoom], color="#e07a3d", lw=1.1, label="m_b")
    ax.plot(t_ms[zoom], wave["mc"][zoom], color="#5b8a72", lw=1.1, label="m_c")
    ax.set_xlim(0.0, 3.0)
    ax.set_ylim(-1.35, 1.35)
    ax.set_ylabel("调制波 / 载波")
    ax.set_title("比较    前 3 ms，上管在调制波不低于载波时导通")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8, ncol=2)

    ax = axes[0, 1]
    ax.plot(t_last, wave["vao"][last], color="#1f4e79", lw=0.8, label="v_AO")
    ax.plot(t_last, vao1 * np.sin(theta0), color="#e07a3d", lw=1.3, label="基波 80 V")
    ax.set_xlim(0.0, 20.0)
    ax.set_ylim(-130.0, 150.0)
    ax.set_ylabel("电压 / V")
    ax.set_title("相电压（对直流中点）    两电平 ±100 V")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[1, 0]
    ax.plot(t_last, wave["vab"][last], color="#1f4e79", lw=0.8, label="v_AB")
    ax.plot(
        t_last,
        vll1 * np.cos(theta0 - math.pi / 3.0),
        color="#e07a3d",
        lw=1.3,
        label=r"基波 $(\sqrt{3}/2)\,m\,V_{dc}$",
    )
    ax.set_xlim(0.0, 20.0)
    ax.set_ylim(-260.0, 280.0)
    ax.set_ylabel("电压 / V")
    ax.set_title("线电压    三电平 +200 / 0 / −200 V")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[1, 1]
    show = spec_f <= 2500.0
    ax.stem(spec_f[show], spec_v[show], linefmt="#1f4e79", markerfmt=" ", basefmt="#d1d5db")
    ax.plot([F1], [vll1], "o", color="#c0392b", ms=5, label="公式 138.56 V")
    ax.axvline(FC, color="#e07a3d", lw=0.8, ls="--", label="载波 1050 Hz")
    ax.set_xlim(0.0, 2500.0)
    ax.set_ylabel("幅值 / V")
    ax.set_title("线电压 FFT    基波在 50 Hz，边带靠近 fc 与 2fc")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[2, 0]
    z_mag = math.hypot(R_LOAD, OMEGA * L_LOAD)
    i1 = vao1 / z_mag
    phi = math.atan2(OMEGA * L_LOAD, R_LOAD)
    ax.plot(t_last, i_a[last], color="#1f4e79", lw=1.2, label="i_A")
    ax.plot(t_last, i1 * np.sin(theta0 - phi), color="#e07a3d", lw=1.2, ls="--", label="基波相量")
    ax.set_xlim(0.0, 20.0)
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("电流 / A")
    ax.set_title("星形 RL    电流把斩波滤成正弦")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[2, 1]
    m_line = np.linspace(0.0, 1.6, 200)
    ax.plot(m_line[m_line <= 1.0], [vll_theory(m) for m in m_line if m <= 1.0], color="#1f4e79", lw=1.4, label="线性区公式")
    ax.plot(
        m_line[m_line >= 1.0],
        [vll_theory(m) for m in m_line if m >= 1.0],
        color="#1f4e79",
        lw=1.0,
        ls=":",
        label="把直线用到 m>1",
    )
    clip_m = m_line[m_line >= 1.0]
    ax.plot(clip_m, [vll_clip(m) for m in clip_m], color="#5b8a72", lw=1.1, ls="--", label="削顶平均模型")
    ax.axhline(vll_six(), color="#9ca3af", lw=0.9, label="六阶梯上限")
    mx = [m for m, _v in markers]
    my = [v for _m, v in markers]
    ax.plot(mx, my, "o", color="#c0392b", ms=5, label="自然采样")
    ax.set_xlim(0.0, 1.65)
    ax.set_ylim(0.0, 280.0)
    ax.set_xlabel("调制比 m")
    ax.set_ylabel("线电压基波峰值 / V")
    ax.set_title("调制比    过调制以后不再是直线")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper left", frameon=False, fontsize=7.5)

    fig.suptitle(
        rf"三相 SPWM    $V_{{dc}}={VDC:.0f}\,\mathrm{{V}}$    $m={M:.1f}$    $f_1={F1:.0f}\,\mathrm{{Hz}}$    $m_f={MF}$",
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


def spectrum_peak(v_ab_cycle: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    n = v_ab_cycle.size
    spec = np.fft.rfft(v_ab_cycle)
    mag = np.abs(spec) * (2.0 / n)
    mag[0] = abs(spec[0]) / n
    if n % 2 == 0:
        mag[-1] = abs(spec[-1]) / n
    freq = np.fft.rfftfreq(n, d=T_FUND / n)
    return freq, mag


def main() -> int:
    if min(VDC, M, F1, R_LOAD, L_LOAD, SPC) <= 0.0:
        print("教学参数要求 Vdc、m、f1、R、L、采样点数都大于 0。", file=sys.stderr)
        return 1
    if not MF % 2 == 1 or MF % 3 != 0:
        print("本卡的 mf 要同时是奇数和 3 的倍数。", file=sys.stderr)
        return 1

    main_case = analyze_case(M)
    low_case = analyze_case(M_LOW)
    unity_case = analyze_case(M_UNITY)
    over_case = analyze_case(M_OVER)
    deep_case = analyze_case(M_DEEP)
    six_case = analyze_case(None)

    vao_th = M * HALF
    vll_th = vll_theory(M)
    vll_rms = vll_th / math.sqrt(2.0)
    coef = math.sqrt(6.0) / 4.0  # Vll,rms / (m·Vdc)

    wave = sample_waveforms()
    n_per = MF * SPC
    dt = T_FUND / n_per
    i_a = integrate_rl(wave["van"], dt)
    cyc = slice(2 * n_per, 3 * n_per)
    v_ab_c = wave["vab"][cyc]
    v_an_c = wave["van"][cyc]
    i_c = i_a[cyc]
    vll_fft = dft_peak(v_ab_c, 1)
    c_v, s_v = dft_cs(v_an_c, 1)
    c_i, s_i = dft_cs(i_c, 1)
    i1_fft = peak((c_i, s_i))
    phi_v = math.atan2(c_v, s_v)
    phi_i = math.atan2(c_i, s_i)
    lag = math.degrees(phi_v - phi_i)
    if lag < 0.0:
        lag += 360.0
    z_mag = math.hypot(R_LOAD, OMEGA * L_LOAD)
    phi_deg = math.degrees(math.atan2(OMEGA * L_LOAD, R_LOAD))
    i1_th = vao_th / z_mag
    thd_i_sq = 0.0
    for h in range(2, H_MAX + 1):
        ih = dft_peak(i_c, h)
        thd_i_sq += ih * ih
    thd_i = math.sqrt(thd_i_sq) / i1_fft

    freq, mag = spectrum_peak(v_ab_c)
    fund_err = main_case["Vll1"] - vll_th

    rows = [
        ("Vdc_V", VDC),
        ("m", M),
        ("f1_Hz", F1),
        ("mf", float(MF)),
        ("fc_Hz", FC),
        ("R_ohm", R_LOAD),
        ("L_H", L_LOAD),
        ("Vao1_theory_V", vao_th),
        ("Vll1_theory_V", vll_th),
        ("Vll1_rms_theory_V", vll_rms),
        ("k_rms", coef),
        ("Vao1_natural_V", main_case["Vao1"]),
        ("Vll1_natural_V", main_case["Vll1"]),
        ("Vll1_bc_natural_V", main_case["Vll1_bc"]),
        ("Vll1_ca_natural_V", main_case["Vll1_ca"]),
        ("rel_err_Vll1", fund_err / vll_th),
        ("h3_V", main_case["h3"]),
        ("h5_V", main_case["h5"]),
        ("h7_V", main_case["h7"]),
        ("h19_V", main_case["h19"]),
        ("h23_V", main_case["h23"]),
        ("h_side_n", main_case["side_n"]),
        ("h_side_V", main_case["side_v"]),
        ("h43_V", main_case["h43"]),
        ("thd_v_to_h84", main_case["thd"]),
        ("Vll1_fft_V", vll_fft),
        ("rel_err_fft", (vll_fft - vll_th) / vll_th),
        ("m_low", M_LOW),
        ("Vll1_theory_low_V", vll_theory(M_LOW)),
        ("Vll1_natural_low_V", low_case["Vll1"]),
        ("m_unity", M_UNITY),
        ("Vll1_theory_unity_V", vll_theory(M_UNITY)),
        ("Vll1_natural_unity_V", unity_case["Vll1"]),
        ("m_over", M_OVER),
        ("Vll1_linear_over_V", vll_theory(M_OVER)),
        ("Vll1_clip_over_V", vll_clip(M_OVER)),
        ("Vll1_natural_over_V", over_case["Vll1"]),
        ("h5_over_V", over_case["h5"]),
        ("h7_over_V", over_case["h7"]),
        ("m_deep", M_DEEP),
        ("Vll1_linear_deep_V", vll_theory(M_DEEP)),
        ("Vll1_clip_deep_V", vll_clip(M_DEEP)),
        ("Vll1_natural_deep_V", deep_case["Vll1"]),
        ("Vll1_six_theory_V", vll_six()),
        ("Vll1_six_V", six_case["Vll1"]),
        ("Vao1_six_V", six_case["Vao1"]),
        ("h5_six_V", six_case["h5"]),
        ("h7_six_V", six_case["h7"]),
        ("Z_ohm", z_mag),
        ("phi_deg", phi_deg),
        ("I1_theory_A", i1_th),
        ("I1_fft_A", i1_fft),
        ("rel_err_I1", (i1_fft - i1_th) / i1_th),
        ("lag_meas_deg", lag),
        ("thd_i_to_h84", thd_i),
    ]

    print("Three-phase SPWM inverter")
    for key, value in rows:
        print(_line(key, value))
    print(f"  {'fund_err_V':<26} = {fund_err:.3e}")

    markers = [
        (M_LOW, low_case["Vll1"]),
        (M, main_case["Vll1"]),
        (M_UNITY, unity_case["Vll1"]),
        (M_OVER, over_case["Vll1"]),
        (M_DEEP, deep_case["Vll1"]),
    ]
    out = Path(__file__).resolve().parent.parent / "img" / "three_phase_inverter_spwm_curves.png"
    plot_case(wave, i_a, freq, mag, markers, out)
    print(f"  {'figure':<26} = {out}")

    problems: list[str] = []
    if abs(vao_th - 80.0) > 1e-12 or abs(vll_th - 80.0 * math.sqrt(3.0)) > 1e-12:
        problems.append("主工况手算应是相电压 80 V、线电压 80√3 V")
    if abs(vll_theory(M_UNITY) - math.sqrt(3.0) / 2.0 * VDC) > 1e-12:
        problems.append("m = 1 的线电压基波峰值应是 (√3/2)·Vdc")
    if abs(main_case["Vll1"] - vll_th) > TOL_NAT:
        problems.append("线性区自然采样的线电压基波偏离 (√3/2)·m·Vdc")
    if abs(main_case["Vao1"] - vao_th) > TOL_NAT:
        problems.append("线性区自然采样的相电压基波偏离 m·Vdc/2")
    if abs(main_case["Vll1"] - math.sqrt(3.0) * main_case["Vao1"]) > TOL_NAT:
        problems.append("线电压基波应是相电压基波的 √3 倍")
    for label, case, modulation in (
        ("low", low_case, M_LOW),
        ("unity", unity_case, M_UNITY),
    ):
        if abs(case["Vll1"] - vll_theory(modulation)) > TOL_NAT:
            problems.append(f"m = {modulation} 的线电压基波没有落在直线上")
        if case["h5"] > 1e-6 or case["h3"] > 1e-6:
            problems.append(f"m = {modulation} 的线电压不应含可观的 3、5 次")
    if main_case["h3"] > 1e-6 or main_case["h5"] > 1e-6 or main_case["h7"] > 1e-6:
        problems.append("线性区线电压的低次谐波应只剩数值噪声")
    if abs(main_case["h19"] - main_case["h23"]) > 1e-6:
        problems.append("19 次和 23 次边带应成对、幅值相同")
    if abs(main_case["h43"] - main_case["side_v"]) > 1e-6:
        problems.append("41 次和 43 次边带应成对、幅值相同")
    if abs(main_case["Vll1_bc"] - main_case["Vll1"]) > TOL_NAT:
        problems.append("三条线电压的基波幅值应相同")
    if abs(main_case["Vll1_ca"] - main_case["Vll1"]) > TOL_NAT:
        problems.append("三条线电压的基波幅值应相同")
    if abs(vll_fft - vll_th) / vll_th > TOL_FFT:
        problems.append("定步长 FFT 的基波离公式超过 0.1%")
    # 过调制：低于直线外推，高于 m = 1，并且靠近削顶模型，但不会和它逐位相同。
    if not (vll_theory(M_UNITY) < over_case["Vll1"] < vll_theory(M_OVER)):
        problems.append("m = 1.2 的基波应落在线性区上限和直线外推之间")
    if abs(over_case["Vll1"] - vll_clip(M_OVER)) / vll_clip(M_OVER) > 0.01:
        problems.append("m = 1.2 的自然采样离削顶模型超过 1%")
    if over_case["h5"] < 1.0:
        problems.append("过调制后线电压应出现明显的 5 次谐波")
    if not (over_case["Vll1"] < deep_case["Vll1"] < vll_six()):
        problems.append("m = 1.5 应比 m = 1.2 更靠近六阶梯，但仍低于上限")
    if abs(six_case["Vll1"] - vll_six()) > TOL_NAT:
        problems.append("六阶梯线电压基波应是 (2√3/π)·Vdc")
    if abs(six_case["Vao1"] - 2.0 * VDC / math.pi) > TOL_NAT:
        problems.append("六阶梯相电压基波应是 2·Vdc/π")
    if abs(six_case["h5"] - six_case["Vll1"] / 5.0) > 1e-6:
        problems.append("六阶梯 5 次谐波应是基波的 1/5")
    if abs(six_case["h7"] - six_case["Vll1"] / 7.0) > 1e-6:
        problems.append("六阶梯 7 次谐波应是基波的 1/7")
    if abs(i1_fft - i1_th) / i1_th > TOL_I:
        problems.append("电流基波偏离相电压基波除以 |Z|")
    if abs(lag - phi_deg) > 0.05:
        problems.append("电流滞后角偏离 atan(ωL/R)")
    if thd_i > 0.1:
        problems.append("20 mH 把电流谐波压到 10% 以内，否则电感参数不对")
    # 波形电平：线电压只有三档，悬浮中性点的相电压是 ±Vdc/3、±2Vdc/3。
    vab_levels = np.unique(np.round(wave["vab"][:n_per], 6))
    van_levels = np.unique(np.round(wave["van"][:n_per], 6))
    if vab_levels.size != 3 or abs(vab_levels[0] + VDC) > 1e-6 or abs(vab_levels[-1] - VDC) > 1e-6:
        problems.append("线电压应只取 −Vdc、0、+Vdc")
    expect_van = np.array([-2.0, -1.0, 0.0, 1.0, 2.0]) * VDC / 3.0
    if van_levels.size != 5 or np.max(np.abs(van_levels - expect_van)) > 1e-6:
        problems.append("悬浮星形的相电压应取 ±Vdc/3、±2Vdc/3 和 0")
    if int(main_case["side_n"]) % 3 == 0:
        problems.append("线电压里最大的边带不应落在 3 的倍数次上")

    if problems:
        print("校验未通过：", file=sys.stderr)
        for item in problems:
            print(f"  - {item}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
