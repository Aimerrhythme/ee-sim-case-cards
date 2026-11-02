#!/usr/bin/env python3
"""理想 Buck 变换器，开环 PWM（教学演示）。

开关模型，元件无损耗、电容无 ESR。状态是电感电流 iL 和输出电压 vo::

    开关导通:        vL = Vin - vo
    关断且 iL > 0:   vL = -vo          （二极管导通，连续或断续的导通段）
    两端都断:        vL = 0，iL 保持 0 （断续，DCM）

    L diL/dt = vL
    C dvo/dt = iL - vo/R

连续导通（CCM）时，稳态平均电压和纹波的教科书公式::

    Vo   = D * Vin
    ΔiL  = (Vin - Vo) * D / (L * fs) = Vin * D * (1 - D) / (L * fs)
    Δvo  = ΔiL / (8 * C * fs)

ΔiL、Δvo 把一个开关周期里的 vo 看成常数。vo 有纹波时，开关模型会和公式差一个零点几的百分数。

断续（DCM）时 Vo = D*Vin 不再成立。记 K = 2*L*fs/R，K < 1-D 进入断续::

    Vo/Vin = 2 / (1 + sqrt(1 + 4*K/D**2))

默认参数与案例卡一致：Vin = 12 V，D = 0.5，fs = 10 kHz，L = 1 mH，
C = 25 μF，R = 6 Ω。故 Vo = 6 V，ΔiL = 0.3 A，Δvo = 0.15 V，
CCM/DCM 边界电阻 40 Ω。R = 80 Ω 用作断续对照。

积分用定步长 RK4，步长 T/200，开通和关断都落在步长边界上。
CCM 主工况再用矩阵指数做同一线性开关模型的对照，确认 RK4 没有把步长误差当成纹波。

用法（在仓库根目录或本目录均可）::

    python cases/buck-pwm/python/buck_pwm.py

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
from scipy.linalg import expm

VIN = 12.0  # V
DUTY = 0.5
FS = 10_000.0  # Hz
L = 1e-3  # H（1 mH）
C = 25e-6  # F（25 μF）
R_CCM = 6.0  # Ω，连续导通
R_DCM = 80.0  # Ω，故意放到边界的两倍，进入断续
DUTY_LOW = 0.25
DUTY_HIGH = 0.75
N_PER = 200  # 每个开关周期的步数。0.25/0.5/0.75 乘上去都是整数，开关沿对齐步长。
N_CYCLES = 50  # 5 ms，启动振荡已经衰减
N_CYCLES_DCM = 400  # 40 ms，轻载阻尼更小，需要更长才进稳态
T_SW = 1.0 / FS
H = T_SW / N_PER

# 教科书公式相对开关模型的允许偏差。公式本身把 vo 当成平的，不是数值误差。
REL_RIPPLE = 0.02
REL_DCM = 0.01
ABS_VO = 1e-3  # V，CCM 平均电压相对 D*Vin
EXPM_TOL_I = 1e-4  # A
EXPM_TOL_V = 1e-4  # V


def ripple_theory(duty: float) -> tuple[float, float, float]:
    """CCM 公式：Vo、ΔiL、Δvo。只在连续导通时使用。"""
    vo = duty * VIN
    di_l = VIN * duty * (1.0 - duty) / (L * FS)
    dv_o = di_l / (8.0 * C * FS)
    return vo, di_l, dv_o


def dcm_gain(duty: float, resistance: float) -> tuple[float, float, float]:
    """返回 K、M=Vo/Vin、Vo。仅当 K < 1-duty 时这是断续公式。"""
    k_param = 2.0 * L * FS / resistance
    m_gain = 2.0 / (1.0 + math.sqrt(1.0 + 4.0 * k_param / (duty * duty)))
    return k_param, m_gain, m_gain * VIN


def rk4_step(
    switch_on: bool,
    il: float,
    vo: float,
    idle: bool,
    resistance: float,
    step: float,
) -> tuple[float, float]:
    """一个定步长 RK4。idle 为真时二极管已截止，电感电流保持 0。"""

    def deriv(current: float, voltage: float) -> tuple[float, float]:
        if idle:
            return 0.0, -voltage / (resistance * C)
        v_l = (VIN - voltage) if switch_on else -voltage
        return v_l / L, (current - voltage / resistance) / C

    k1_i, k1_v = deriv(il, vo)
    k2_i, k2_v = deriv(il + 0.5 * step * k1_i, vo + 0.5 * step * k1_v)
    k3_i, k3_v = deriv(il + 0.5 * step * k2_i, vo + 0.5 * step * k2_v)
    k4_i, k4_v = deriv(il + step * k3_i, vo + step * k3_v)
    il_n = il + (step / 6.0) * (k1_i + 2.0 * k2_i + 2.0 * k3_i + k4_i)
    vo_n = vo + (step / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)
    return il_n, vo_n


def simulate(
    duty: float,
    resistance: float,
    n_cycles: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """定步长 RK4。返回 t、iL、vo，样本取在每个步长结束的时刻。"""
    if abs(duty * N_PER - round(duty * N_PER)) > 1e-9:
        raise ValueError("占空比必须让开通时间等于整数步，否则开关沿会落在步长中间。")
    n_on = int(round(duty * N_PER))
    n_tot = n_cycles * N_PER
    t = np.empty(n_tot)
    il_out = np.empty(n_tot)
    vo_out = np.empty(n_tot)
    il = 0.0
    vo = 0.0
    p = 0
    for k in range(n_cycles):
        for i in range(N_PER):
            switch_on = i < n_on
            idle = (not switch_on) and il <= 0.0
            il, vo = rk4_step(switch_on, il, vo, idle, resistance, H)
            if (not switch_on) and il < 0.0:
                il = 0.0
            t[p] = (k * N_PER + i + 1) * H
            il_out[p] = il
            vo_out[p] = vo
            p += 1
    return t, il_out, vo_out


def expm_ccm(
    duty: float,
    resistance: float,
    n_cycles: int,
) -> tuple[np.ndarray, np.ndarray]:
    """CCM 线性开关模型在同一网格上的矩阵指数解，用来检验 RK4。"""
    n_on = int(round(duty * N_PER))
    a_mat = np.array(
        [[0.0, -1.0 / L], [1.0 / C, -1.0 / (resistance * C)]],
        dtype=float,
    )

    def discrete(b_vec: np.ndarray) -> np.ndarray:
        aug = np.zeros((3, 3))
        aug[:2, :2] = a_mat
        aug[:2, 2] = b_vec
        return expm(aug * H)

    ad_on = discrete(np.array([VIN / L, 0.0]))
    ad_off = discrete(np.array([0.0, 0.0]))
    il = 0.0
    vo = 0.0
    il_out = np.empty(n_cycles * N_PER)
    vo_out = np.empty(n_cycles * N_PER)
    p = 0
    for _k in range(n_cycles):
        for i in range(N_PER):
            ad = ad_on if i < n_on else ad_off
            nxt = ad @ np.array([il, vo, 1.0])
            il = float(nxt[0])
            vo = float(nxt[1])
            il_out[p] = il
            vo_out[p] = vo
            p += 1
    return il_out, vo_out


def steady_window(
    t: np.ndarray,
    il: np.ndarray,
    vo: np.ndarray,
    n_cycles_total: int,
    n_keep: int = 2,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """取最后 n_keep 个完整开关周期。"""
    t_end = n_cycles_total * T_SW
    mask = t >= t_end - n_keep * T_SW
    return t[mask], il[mask], vo[mask]


def cycle_stats(il: np.ndarray, vo: np.ndarray) -> dict[str, float]:
    return {
        "Vo": float(np.mean(vo)),
        "dvo": float(np.max(vo) - np.min(vo)),
        "iL": float(np.mean(il)),
        "diL": float(np.max(il) - np.min(il)),
        "imin": float(np.min(il)),
        "imax": float(np.max(il)),
        "zero_frac": float(np.mean(il <= 0.0)),
    }


def averaged_peak() -> dict[str, float]:
    """平均模型（把开关换成 D*Vin 的直流）的欠阻尼峰值。电阻并联在电容上。"""
    w0 = 1.0 / math.sqrt(L * C)
    zeta = math.sqrt(L / C) / (2.0 * R_CCM)
    wd = w0 * math.sqrt(1.0 - zeta * zeta)
    os_frac = math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta * zeta))
    vo, _, _ = ripple_theory(DUTY)
    return {
        "omega0_rad_s": w0,
        "zeta": zeta,
        "tp_avg_ms": (math.pi / wd) * 1e3,
        "os_avg_frac": os_frac,
        "vo_peak_avg_model_V": vo * (1.0 + os_frac),
    }


def ideal_il_triangle(t: np.ndarray) -> np.ndarray:
    """CCM、D=0.5 的教科书电流折线：0.85 A 到 1.15 A。"""
    _vo, di_l, _dv = ripple_theory(DUTY)
    i_min = ripple_theory(DUTY)[0] / R_CCM - 0.5 * di_l
    tau = np.mod(t, T_SW)
    t_on = DUTY * T_SW
    rise = tau < t_on
    up = i_min + di_l * (tau / t_on)
    down = i_min + di_l - di_l * ((tau - t_on) / (T_SW - t_on))
    return np.where(rise, up, down)


def _enable_cjk() -> None:
    for path in (
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
    t_ccm: np.ndarray,
    il_ccm: np.ndarray,
    vo_ccm: np.ndarray,
    t_dcm: np.ndarray,
    il_dcm: np.ndarray,
    vo_dcm: np.ndarray,
    out: Path,
) -> None:
    vo_th, di_th, dv_th = ripple_theory(DUTY)
    _k, _m, vo_dcm_th = dcm_gain(DUTY, R_DCM)
    t_ms = t_ccm * 1e3
    peak_i = int(np.argmax(vo_ccm))

    _enable_cjk()
    fig, axes = plt.subplots(3, 2, figsize=(9.6, 9.2))

    ax = axes[0, 0]
    ax.plot(t_ms, vo_ccm, color="#1f4e79", lw=1.4, label="RK4 开关模型")
    ax.axhline(vo_th, color="#9ca3af", lw=0.9, label=rf"$D\cdot V_{{in}}={vo_th:.0f}\,\mathrm{{V}}$")
    ax.scatter(
        [t_ms[peak_i]],
        [vo_ccm[peak_i]],
        s=28,
        color="#1f4e79",
        zorder=5,
        label="启动峰值",
    )
    ax.set_ylabel("输出电压 vo / V")
    ax.set_title("启动    CCM，D = 0.5，R = 6 Ω")
    ax.set_xlim(0.0, N_CYCLES * T_SW * 1e3)
    ax.set_ylim(-0.4, 7.8)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right", frameon=False, fontsize=8)

    ax = axes[0, 1]
    ax.plot(t_ms, il_ccm, color="#1f4e79", lw=1.4, label="RK4 开关模型")
    ax.axhline(vo_th / R_CCM, color="#9ca3af", lw=0.9, label="平均 1 A")
    ax.set_ylabel("电感电流 iL / A")
    ax.set_title("启动    电感电流")
    ax.set_xlim(0.0, N_CYCLES * T_SW * 1e3)
    ax.set_ylim(-0.05, 1.65)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    t_z, il_z, vo_z = steady_window(t_ccm, il_ccm, vo_ccm, N_CYCLES, n_keep=4)
    ax = axes[1, 0]
    ax.plot(t_z * 1e3, vo_z, color="#1f4e79", lw=1.6, label="RK4")
    ax.axhline(vo_th, color="#9ca3af", lw=0.8)
    ax.axhline(vo_th + 0.5 * dv_th, color="#e07a3d", lw=1.0, ls="--", label="公式峰 / 谷")
    ax.axhline(vo_th - 0.5 * dv_th, color="#e07a3d", lw=1.0, ls="--")
    ax.set_ylabel("vo / V")
    ax.set_title("稳态纹波    最后 4 个周期")
    ax.set_ylim(vo_th - 0.16, vo_th + 0.16)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[1, 1]
    ax.plot(t_z * 1e3, il_z, color="#1f4e79", lw=1.6, label="RK4")
    ax.plot(t_z * 1e3, ideal_il_triangle(t_z), color="#e07a3d", lw=1.2, ls="--", label="公式折线")
    ax.set_ylabel("iL / A")
    ax.set_title("稳态纹波    电感电流")
    ax.set_ylim(0.78, 1.22)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    t_d, il_d, vo_d = steady_window(t_dcm, il_dcm, vo_dcm, N_CYCLES_DCM, n_keep=4)
    ax = axes[2, 0]
    ax.plot(t_d * 1e3, vo_d, color="#1f4e79", lw=1.6, label="RK4")
    ax.axhline(vo_dcm_th, color="#e07a3d", lw=1.0, ls="--", label="DCM 公式")
    ax.axhline(DUTY * VIN, color="#9ca3af", lw=1.0, ls=":", label=r"误用 $D\cdot V_{in}$")
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("vo / V")
    ax.set_title("轻载断续    R = 80 Ω，D = 0.5")
    ax.set_ylim(5.6, 7.9)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="center right", frameon=False, fontsize=8)

    ax = axes[2, 1]
    ax.plot(t_d * 1e3, il_d, color="#1f4e79", lw=1.6, label="RK4")
    ax.axhline(0.0, color="#9ca3af", lw=0.8)
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("iL / A")
    ax.set_title("轻载断续    电流每个周期回到 0")
    ax.set_ylim(-0.02, 0.30)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    fig.suptitle(
        rf"Buck 开环 PWM    $V_{{in}}={VIN:.0f}\,\mathrm{{V}}$    $f_s={FS/1000:.0f}\,\mathrm{{kHz}}$",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def _line(key: str, value: float) -> str:
    text = f"{value:.6f}"
    # 小于 5e-7 时 %.6f 会印成 -0.000000，课堂对照表里写成 0。
    if abs(value) < 5e-7:
        text = "0.000000"
    return f"  {key:<26} = {text}"


def main() -> int:
    if min(L, C, R_CCM, R_DCM, FS, VIN) <= 0.0:
        print("教学参数要求 Vin、fs、L、C、R 都大于 0。", file=sys.stderr)
        return 1
    if not 0.0 < DUTY < 1.0:
        print("占空比要在 0 和 1 之间。", file=sys.stderr)
        return 1

    t_ccm, il_ccm, vo_ccm = simulate(DUTY, R_CCM, N_CYCLES)
    il_ref, vo_ref = expm_ccm(DUTY, R_CCM, N_CYCLES)
    err_i = float(np.max(np.abs(il_ccm - il_ref)))
    err_v = float(np.max(np.abs(vo_ccm - vo_ref)))

    _tz, il_s, vo_s = steady_window(t_ccm, il_ccm, vo_ccm, N_CYCLES, n_keep=2)
    st = cycle_stats(il_s, vo_s)
    vo_th, di_th, dv_th = ripple_theory(DUTY)
    i_th = vo_th / R_CCM
    r_crit = 2.0 * L * FS / (1.0 - DUTY)
    i_crit = vo_th / r_crit
    k_ccm = 2.0 * L * FS / R_CCM
    avg = averaged_peak()
    peak_i = int(np.argmax(vo_ccm))

    t_lo, il_lo, vo_lo = simulate(DUTY_LOW, R_CCM, N_CYCLES)
    _t, il_lo_s, vo_lo_s = steady_window(t_lo, il_lo, vo_lo, N_CYCLES)
    st_lo = cycle_stats(il_lo_s, vo_lo_s)
    vo_lo_th, di_lo_th, dv_lo_th = ripple_theory(DUTY_LOW)

    t_hi, il_hi, vo_hi = simulate(DUTY_HIGH, R_CCM, N_CYCLES)
    _t, il_hi_s, vo_hi_s = steady_window(t_hi, il_hi, vo_hi, N_CYCLES)
    st_hi = cycle_stats(il_hi_s, vo_hi_s)
    vo_hi_th, di_hi_th, dv_hi_th = ripple_theory(DUTY_HIGH)

    t_dcm, il_dcm, vo_dcm = simulate(DUTY, R_DCM, N_CYCLES_DCM)
    _t, il_d_s, vo_d_s = steady_window(t_dcm, il_dcm, vo_dcm, N_CYCLES_DCM)
    st_d = cycle_stats(il_d_s, vo_d_s)
    k_dcm, m_dcm, vo_dcm_th = dcm_gain(DUTY, R_DCM)

    rows = [
        ("Vin_V", VIN),
        ("duty", DUTY),
        ("fs_Hz", FS),
        ("L_H", L),
        ("C_F", C),
        ("R_ohm", R_CCM),
        ("Vo_theory_V", vo_th),
        ("iL_theory_A", i_th),
        ("delta_iL_theory_A", di_th),
        ("delta_vo_theory_V", dv_th),
        ("iL_min_theory_A", i_th - 0.5 * di_th),
        ("iL_max_theory_A", i_th + 0.5 * di_th),
        ("Rcrit_ohm", r_crit),
        ("Icrit_A", i_crit),
        ("K_ccm", k_ccm),
        ("omega0_rad_s", avg["omega0_rad_s"]),
        ("zeta", avg["zeta"]),
        ("tp_avg_ms", avg["tp_avg_ms"]),
        ("os_avg_frac", avg["os_avg_frac"]),
        ("vo_peak_avg_model_V", avg["vo_peak_avg_model_V"]),
        ("Vo_meas_V", st["Vo"]),
        ("iL_meas_A", st["iL"]),
        ("delta_iL_meas_A", st["diL"]),
        ("delta_vo_meas_V", st["dvo"]),
        ("iL_min_meas_A", st["imin"]),
        ("iL_max_meas_A", st["imax"]),
        ("rel_err_Vo", (st["Vo"] - vo_th) / vo_th),
        ("rel_err_diL", (st["diL"] - di_th) / di_th),
        ("rel_err_dvo", (st["dvo"] - dv_th) / dv_th),
        ("vo_peak_switched_V", float(vo_ccm[peak_i])),
        ("t_peak_ms", float(t_ccm[peak_i]) * 1e3),
        ("duty_low", DUTY_LOW),
        ("Vo_theory_low_V", vo_lo_th),
        ("delta_iL_theory_low_A", di_lo_th),
        ("delta_vo_theory_low_V", dv_lo_th),
        ("Vo_meas_low_V", st_lo["Vo"]),
        ("delta_iL_meas_low_A", st_lo["diL"]),
        ("delta_vo_meas_low_V", st_lo["dvo"]),
        ("duty_high", DUTY_HIGH),
        ("Vo_theory_high_V", vo_hi_th),
        ("delta_iL_theory_high_A", di_hi_th),
        ("delta_vo_theory_high_V", dv_hi_th),
        ("Vo_meas_high_V", st_hi["Vo"]),
        ("delta_iL_meas_high_A", st_hi["diL"]),
        ("delta_vo_meas_high_V", st_hi["dvo"]),
        ("R_dcm_ohm", R_DCM),
        ("K_dcm", k_dcm),
        ("M_dcm", m_dcm),
        ("Vo_dcm_theory_V", vo_dcm_th),
        ("Vo_dcm_if_ccm_V", DUTY * VIN),
        ("Vo_dcm_meas_V", st_d["Vo"]),
        ("iL_max_dcm_A", st_d["imax"]),
        ("iL_min_dcm_A", st_d["imin"]),
        ("zero_frac_dcm", st_d["zero_frac"]),
    ]

    print("Buck open-loop PWM")
    for key, value in rows:
        print(_line(key, value))
    print(f"  {'max_abs_err_il_A':<26} = {err_i:.3e}")
    print(f"  {'max_abs_err_vo_V':<26} = {err_v:.3e}")

    out = Path(__file__).resolve().parent.parent / "img" / "buck_pwm_curves.png"
    plot_case(t_ccm, il_ccm, vo_ccm, t_dcm, il_dcm, vo_dcm, out)
    print(f"  {'figure':<26} = {out}")

    # 第一个样本应接近 Vin/L * h：开关刚合上，电感还没有建立起电压降。
    il0_expected = (VIN / L) * H
    problems: list[str] = []
    if abs(float(il_ccm[0]) - il0_expected) > 1e-6:
        problems.append("t = 第一步的 di/dt 不是 Vin/L")
    if abs(st["Vo"] - vo_th) > ABS_VO:
        problems.append("CCM 平均电压偏离 D*Vin")
    if abs(st["diL"] - di_th) / di_th > REL_RIPPLE:
        problems.append("ΔiL 与教科书公式相差超过 2%")
    if abs(st["dvo"] - dv_th) / dv_th > REL_RIPPLE:
        problems.append("Δvo 与教科书公式相差超过 2%")
    if st["imin"] <= 0.5 * di_th:
        problems.append("R = 6 Ω 应处于连续导通，电感电流不应贴近 0")
    if err_i > EXPM_TOL_I or err_v > EXPM_TOL_V:
        problems.append("RK4 与矩阵指数不一致，纹波里混进了步长误差")
    if abs(st_lo["Vo"] - vo_lo_th) > ABS_VO or abs(st_hi["Vo"] - vo_hi_th) > ABS_VO:
        problems.append("D = 0.25 或 0.75 的平均电压没有跟着占空比走")
    if abs(st_lo["diL"] - di_lo_th) / di_lo_th > REL_RIPPLE:
        problems.append("D = 0.25 的电流纹波偏离公式")
    if abs(st_hi["diL"] - di_hi_th) / di_hi_th > REL_RIPPLE:
        problems.append("D = 0.75 的电流纹波偏离公式")
    if abs(st_lo["diL"] - st_hi["diL"]) > 1e-3:
        problems.append("D = 0.25 和 D = 0.75 的电流纹波应对称")
    if st["diL"] <= st_lo["diL"]:
        problems.append("电流纹波应在 D = 0.5 大于 D = 0.25")
    if k_dcm >= 1.0 - DUTY:
        problems.append("R = 80 Ω 的 K 应小于 1-D")
    if st_d["imin"] > 1e-9:
        problems.append("断续工况的电感电流应回到 0")
    if st_d["zero_frac"] < 0.15 or st_d["zero_frac"] > 0.25:
        problems.append("断续零电流所占时间不在预期范围")
    if abs(st_d["Vo"] - vo_dcm_th) / vo_dcm_th > REL_DCM:
        problems.append("断续电压偏离 DCM 公式超过 1%")
    if st_d["Vo"] < DUTY * VIN + 1.0:
        problems.append("断续电压应明显高于 D*Vin")
    if not 6.5 < float(vo_ccm[peak_i]) < 7.5:
        problems.append("启动峰值不在平均模型附近")

    if problems:
        print("校验未通过：", file=sys.stderr)
        for item in problems:
            print(f"  - {item}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
