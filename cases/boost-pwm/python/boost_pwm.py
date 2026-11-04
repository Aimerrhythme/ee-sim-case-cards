#!/usr/bin/env python3
"""理想 Boost 变换器，开环 PWM（教学演示）。

开关模型，元件无损耗、电容无 ESR。状态是电感电流 iL 和输出电压 vo::

    开关导通:              vL = Vin，         C dvo/dt = -vo/R   （二极管反偏）
    关断且二极管导通:      vL = Vin - vo，    C dvo/dt = iL - vo/R
    关断且 iL = 0、vo>Vin: vL = 0，iL 保持 0，C dvo/dt = -vo/R   （断续，DCM）

    L diL/dt = vL

连续导通（CCM）时的教科书公式::

    Vo   = Vin / (1 - D)
    IL   = Io / (1 - D) = Vo / (R * (1 - D))
    ΔiL  = Vin * D / (L * fs)
    Δvo  = Io * D / (C * fs)

ΔiL 只看导通段：开关导通时电感两端正好是 Vin，和负载无关。
Δvo 把导通段的负载电流看成常数 Io，由电容单独供电。

断续（DCM）时记 K = 2*L*fs/R，K < D*(1-D)**2 进入断续::

    Vo/Vin = (1 + sqrt(1 + 4*D**2/K)) / 2

默认参数与案例卡一致：Vin = 12 V，D = 0.5，fs = 10 kHz，L = 1 mH，
C = 100 μF，R = 48 Ω。故 Vo = 24 V，Io = 0.5 A，IL = 1 A，ΔiL = 0.6 A，
Δvo = 0.25 V，CCM/DCM 边界电阻 160 Ω。R = 320 Ω 用作断续对照。

积分用定步长 RK4，步长 T/200，开通和关断都落在步长边界上。
CCM 主工况再用矩阵指数做同一线性开关模型的对照，确认 RK4 没有把步长误差当成纹波。

用法（在仓库根目录或本目录均可）::

    python cases/boost-pwm/python/boost_pwm.py

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
C = 100e-6  # F（100 μF）
R_CCM = 48.0  # Ω，连续导通
R_DCM = 320.0  # Ω，边界的两倍，进入断续
DUTY_LOW = 0.25
DUTY_HIGH = 0.75
N_PER = 200  # 每个开关周期的步数。0.25/0.5/0.75 乘上去都是整数。
N_CYCLES = 1000  # 100 ms。开环 Boost 阻尼很小，启动振荡要几十毫秒才收住
N_CYCLES_DCM = 1500  # 150 ms
T_SW = 1.0 / FS
H = T_SW / N_PER

REL_RIPPLE = 0.03
REL_DCM = 0.01
REL_VO = 0.005
EXPM_TOL_I = 1e-4  # A
EXPM_TOL_V = 1e-4  # V


def ccm_theory(duty: float, resistance: float) -> dict[str, float]:
    """CCM 公式。只在连续导通时使用。"""
    vo = VIN / (1.0 - duty)
    io = vo / resistance
    il = io / (1.0 - duty)
    di_l = VIN * duty / (L * FS)
    dv_o = io * duty / (C * FS)
    return {"Vo": vo, "Io": io, "IL": il, "diL": di_l, "dvo": dv_o}


def dcm_gain(duty: float, resistance: float) -> tuple[float, float, float]:
    """返回 K、M=Vo/Vin、Vo。仅当 K < D(1-D)^2 时这是断续公式。"""
    k_param = 2.0 * L * FS / resistance
    m_gain = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * duty * duty / k_param))
    return k_param, m_gain, m_gain * VIN


def deriv(mode: int, il: float, vo: float, resistance: float) -> tuple[float, float]:
    """mode 0：开关导通；1：关断、二极管导通；2：断续，电感空闲。"""
    if mode == 0:
        return VIN / L, -vo / (resistance * C)
    if mode == 1:
        return (VIN - vo) / L, (il - vo / resistance) / C
    return 0.0, -vo / (resistance * C)


def rk4_step(mode: int, il: float, vo: float, resistance: float, step: float) -> tuple[float, float]:
    k1_i, k1_v = deriv(mode, il, vo, resistance)
    k2_i, k2_v = deriv(mode, il + 0.5 * step * k1_i, vo + 0.5 * step * k1_v, resistance)
    k3_i, k3_v = deriv(mode, il + 0.5 * step * k2_i, vo + 0.5 * step * k2_v, resistance)
    k4_i, k4_v = deriv(mode, il + step * k3_i, vo + step * k3_v, resistance)
    il_n = il + (step / 6.0) * (k1_i + 2.0 * k2_i + 2.0 * k3_i + k4_i)
    vo_n = vo + (step / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)
    return il_n, vo_n


def simulate(
    duty: float,
    resistance: float,
    n_cycles: int,
    sync: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """定步长 RK4。返回 t、iL、vo，样本取在每个步长结束的时刻。

    sync=True 时把二极管换成同步整流管：关断段始终是 vL = Vin - vo，电流可以变负，
    永远是连续导通。它只用来和矩阵指数对照，检验积分本身。
    """
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
            if i < n_on:
                mode = 0
            elif sync or il > 0.0 or vo < VIN:
                mode = 1
            else:
                mode = 2
            il, vo = rk4_step(mode, il, vo, resistance, H)
            if mode == 1 and il < 0.0 and not sync:
                il = 0.0
            t[p] = (k * N_PER + i + 1) * H
            il_out[p] = il
            vo_out[p] = vo
            p += 1
    return t, il_out, vo_out


def expm_ccm(duty: float, resistance: float, n_cycles: int) -> tuple[np.ndarray, np.ndarray]:
    """CCM 线性开关模型在同一网格上的矩阵指数解，用来检验 RK4。"""
    n_on = int(round(duty * N_PER))
    a_on = np.array([[0.0, 0.0], [0.0, -1.0 / (resistance * C)]])
    a_off = np.array([[0.0, -1.0 / L], [1.0 / C, -1.0 / (resistance * C)]])
    b_vec = np.array([VIN / L, 0.0])

    def discrete(a_mat: np.ndarray) -> np.ndarray:
        aug = np.zeros((3, 3))
        aug[:2, :2] = a_mat
        aug[:2, 2] = b_vec
        return expm(aug * H)

    ad_on = discrete(a_on)
    ad_off = discrete(a_off)
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


def steady_window(t, il, vo, n_cycles_total: int, n_keep: int = 2):
    t_end = n_cycles_total * T_SW
    mask = t >= t_end - n_keep * T_SW - 0.5 * H
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


def averaged_model(duty: float, resistance: float) -> dict[str, float]:
    """平均模型：L diL/dt = Vin-(1-D)vo，C dvo/dt = (1-D)iL - vo/R。

    等效成电感 L/(1-D)^2 接在 Vin/(1-D) 后面的 LC 滤波器，电阻并联在电容上。
    """
    dp = 1.0 - duty
    w0 = dp / math.sqrt(L * C)
    zeta = math.sqrt(L / C) / (2.0 * resistance * dp)
    wd = w0 * math.sqrt(1.0 - zeta * zeta)
    os_frac = math.exp(-zeta * math.pi / math.sqrt(1.0 - zeta * zeta))
    vo = VIN / dp
    return {
        "omega0_rad_s": w0,
        "zeta": zeta,
        "tp_avg_ms": (math.pi / wd) * 1e3,
        "os_avg_frac": os_frac,
        "vo_peak_avg_model_V": vo * (1.0 + os_frac),
        "settle_2pct_ms": 4.0 / (zeta * w0) * 1e3,
    }


def ideal_il_triangle(t: np.ndarray) -> np.ndarray:
    th = ccm_theory(DUTY, R_CCM)
    i_min = th["IL"] - 0.5 * th["diL"]
    tau = np.mod(t + 0.5 * H, T_SW) - 0.5 * H
    tau = np.clip(tau, 0.0, T_SW)
    t_on = DUTY * T_SW
    up = i_min + th["diL"] * (tau / t_on)
    down = i_min + th["diL"] - th["diL"] * ((tau - t_on) / (T_SW - t_on))
    return np.where(tau < t_on, up, down)


def _enable_cjk() -> None:
    for path in (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    ):
        if Path(path).is_file():
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            return


def plot_case(t_ccm, il_ccm, vo_ccm, t_dcm, il_dcm, vo_dcm, out: Path) -> None:
    th = ccm_theory(DUTY, R_CCM)
    _k, _m, vo_dcm_th = dcm_gain(DUTY, R_DCM)
    t_ms = t_ccm * 1e3
    peak_i = int(np.argmax(vo_ccm))

    _enable_cjk()
    fig, axes = plt.subplots(3, 2, figsize=(9.6, 9.2))

    ax = axes[0, 0]
    ax.plot(t_ms, vo_ccm, color="#1f4e79", lw=1.0, label="RK4 开关模型")
    ax.axhline(th["Vo"], color="#9ca3af", lw=0.9, label=rf"$V_{{in}}/(1-D)={th['Vo']:.0f}\,\mathrm{{V}}$")
    ax.scatter([t_ms[peak_i]], [vo_ccm[peak_i]], s=28, color="#c0392b", zorder=5, label="启动峰值")
    ax.set_ylabel("输出电压 vo / V")
    ax.set_title("启动    CCM，D = 0.5，R = 48 Ω")
    ax.set_xlim(0.0, N_CYCLES * T_SW * 1e3)
    ax.set_ylim(-1.0, 48.0)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[0, 1]
    ax.plot(t_ms, il_ccm, color="#1f4e79", lw=0.6, label="RK4 开关模型")
    ax.axhline(th["IL"], color="#9ca3af", lw=0.9, label="平均 1 A")
    ax.set_ylabel("电感电流 iL / A")
    ax.set_title("启动    电感电流")
    ax.set_xlim(0.0, N_CYCLES * T_SW * 1e3)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    t_z, il_z, vo_z = steady_window(t_ccm, il_ccm, vo_ccm, N_CYCLES, n_keep=4)
    ax = axes[1, 0]
    ax.plot(t_z * 1e3, vo_z, color="#1f4e79", lw=1.6, label="RK4")
    ax.axhline(th["Vo"], color="#9ca3af", lw=0.8)
    # Boost 的电压纹波是锯齿，不以平均值上下对称。从实测峰值往下量一个公式 Δvo。
    v_top = float(np.max(vo_z))
    ax.axhline(v_top, color="#e07a3d", lw=1.0, ls="--", label="峰值 − 公式 Δvo")
    ax.axhline(v_top - th["dvo"], color="#e07a3d", lw=1.0, ls="--")
    ax.set_ylabel("vo / V")
    ax.set_title("稳态纹波    最后 4 个周期")
    ax.set_ylim(th["Vo"] - 0.3, th["Vo"] + 0.3)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    ax = axes[1, 1]
    ax.plot(t_z * 1e3, il_z, color="#1f4e79", lw=1.6, label="RK4")
    ax.plot(t_z * 1e3, ideal_il_triangle(t_z), color="#e07a3d", lw=1.2, ls="--", label="公式折线")
    ax.set_ylabel("iL / A")
    ax.set_title("稳态纹波    电感电流")
    ax.set_ylim(0.55, 1.45)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    t_d, il_d, vo_d = steady_window(t_dcm, il_dcm, vo_dcm, N_CYCLES_DCM, n_keep=4)
    ax = axes[2, 0]
    ax.plot(t_d * 1e3, vo_d, color="#1f4e79", lw=1.6, label="RK4")
    ax.axhline(vo_dcm_th, color="#e07a3d", lw=1.0, ls="--", label="DCM 公式")
    ax.axhline(th["Vo"], color="#9ca3af", lw=1.0, ls=":", label=r"误用 $V_{in}/(1-D)$")
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("vo / V")
    ax.set_title("轻载断续    R = 320 Ω，D = 0.5")
    ax.set_ylim(22.0, 33.0)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="center right", frameon=False, fontsize=8)

    ax = axes[2, 1]
    ax.plot(t_d * 1e3, il_d, color="#1f4e79", lw=1.6, label="RK4")
    ax.axhline(0.0, color="#9ca3af", lw=0.8)
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("iL / A")
    ax.set_title("轻载断续    电流每个周期回到 0")
    ax.set_ylim(-0.05, 0.7)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)

    fig.suptitle(
        rf"Boost 开环 PWM    $V_{{in}}={VIN:.0f}\,\mathrm{{V}}$    $f_s={FS/1000:.0f}\,\mathrm{{kHz}}$",
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


def main() -> int:
    if min(L, C, R_CCM, R_DCM, FS, VIN) <= 0.0:
        print("教学参数要求 Vin、fs、L、C、R 都大于 0。", file=sys.stderr)
        return 1
    if not 0.0 < DUTY < 1.0:
        print("占空比要在 0 和 1 之间。", file=sys.stderr)
        return 1

    t_ccm, il_ccm, vo_ccm = simulate(DUTY, R_CCM, N_CYCLES)
    # 二极管版本在启动上冲时会短暂断续，线性模型对不上。积分精度用同步整流版本检验。
    _ts, il_sync, vo_sync = simulate(DUTY, R_CCM, N_CYCLES, sync=True)
    il_ref, vo_ref = expm_ccm(DUTY, R_CCM, N_CYCLES)
    err_i = float(np.max(np.abs(il_sync - il_ref)))
    err_v = float(np.max(np.abs(vo_sync - vo_ref)))
    zero_idx = np.where(il_ccm <= 0.0)[0]
    t_dcm_start = float(t_ccm[zero_idx[0]]) * 1e3 if zero_idx.size else 0.0
    t_dcm_end = float(t_ccm[zero_idx[-1]]) * 1e3 if zero_idx.size else 0.0
    il_min_sync = float(np.min(il_sync))

    _tz, il_s, vo_s = steady_window(t_ccm, il_ccm, vo_ccm, N_CYCLES, n_keep=2)
    st = cycle_stats(il_s, vo_s)
    th = ccm_theory(DUTY, R_CCM)
    k_crit = DUTY * (1.0 - DUTY) ** 2
    r_crit = 2.0 * L * FS / k_crit
    k_ccm = 2.0 * L * FS / R_CCM
    avg = averaged_model(DUTY, R_CCM)
    peak_i = int(np.argmax(vo_ccm))
    il_peak_start = float(np.max(il_ccm))

    res = {}
    for tag, duty in (("low", DUTY_LOW), ("high", DUTY_HIGH)):
        t_x, il_x, vo_x = simulate(duty, R_CCM, N_CYCLES)
        _t, il_xs, vo_xs = steady_window(t_x, il_x, vo_x, N_CYCLES)
        res[tag] = (cycle_stats(il_xs, vo_xs), ccm_theory(duty, R_CCM))

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
        ("Vo_theory_V", th["Vo"]),
        ("Io_theory_A", th["Io"]),
        ("iL_theory_A", th["IL"]),
        ("delta_iL_theory_A", th["diL"]),
        ("delta_vo_theory_V", th["dvo"]),
        ("iL_min_theory_A", th["IL"] - 0.5 * th["diL"]),
        ("iL_max_theory_A", th["IL"] + 0.5 * th["diL"]),
        ("K_ccm", k_ccm),
        ("K_crit", k_crit),
        ("Rcrit_ohm", r_crit),
        ("omega0_rad_s", avg["omega0_rad_s"]),
        ("zeta", avg["zeta"]),
        ("tp_avg_ms", avg["tp_avg_ms"]),
        ("os_avg_frac", avg["os_avg_frac"]),
        ("vo_peak_avg_model_V", avg["vo_peak_avg_model_V"]),
        ("settle_2pct_avg_ms", avg["settle_2pct_ms"]),
        ("vo_peak_switched_V", float(vo_ccm[peak_i])),
        ("t_peak_ms", float(t_ccm[peak_i]) * 1e3),
        ("iL_peak_startup_A", il_peak_start),
        ("startup_dcm_from_ms", t_dcm_start),
        ("startup_dcm_to_ms", t_dcm_end),
        ("iL_min_sync_startup_A", il_min_sync),
        ("vo_peak_sync_V", float(np.max(vo_sync))),
        ("Vo_meas_V", st["Vo"]),
        ("iL_meas_A", st["iL"]),
        ("delta_iL_meas_A", st["diL"]),
        ("delta_vo_meas_V", st["dvo"]),
        ("iL_min_meas_A", st["imin"]),
        ("iL_max_meas_A", st["imax"]),
        ("rel_err_Vo", (st["Vo"] - th["Vo"]) / th["Vo"]),
        ("rel_err_diL", (st["diL"] - th["diL"]) / th["diL"]),
        ("rel_err_dvo", (st["dvo"] - th["dvo"]) / th["dvo"]),
    ]
    for tag, duty in (("low", DUTY_LOW), ("high", DUTY_HIGH)):
        s_x, th_x = res[tag]
        rows += [
            (f"duty_{tag}", duty),
            (f"Vo_theory_{tag}_V", th_x["Vo"]),
            (f"iL_theory_{tag}_A", th_x["IL"]),
            (f"delta_iL_theory_{tag}_A", th_x["diL"]),
            (f"delta_vo_theory_{tag}_V", th_x["dvo"]),
            (f"Vo_meas_{tag}_V", s_x["Vo"]),
            (f"iL_meas_{tag}_A", s_x["iL"]),
            (f"delta_iL_meas_{tag}_A", s_x["diL"]),
            (f"delta_vo_meas_{tag}_V", s_x["dvo"]),
        ]
    rows += [
        ("R_dcm_ohm", R_DCM),
        ("K_dcm", k_dcm),
        ("M_dcm", m_dcm),
        ("Vo_dcm_theory_V", vo_dcm_th),
        ("Vo_dcm_if_ccm_V", th["Vo"]),
        ("Vo_dcm_meas_V", st_d["Vo"]),
        ("iL_max_dcm_A", st_d["imax"]),
        ("iL_min_dcm_A", st_d["imin"]),
        ("zero_frac_dcm", st_d["zero_frac"]),
    ]

    print("Boost open-loop PWM")
    for key, value in rows:
        print(_line(key, value))
    print(f"  {'max_abs_err_il_A':<26} = {err_i:.3e}")
    print(f"  {'max_abs_err_vo_V':<26} = {err_v:.3e}")

    out = Path(__file__).resolve().parent.parent / "img" / "boost_pwm_curves.png"
    plot_case(t_ccm, il_ccm, vo_ccm, t_dcm, il_dcm, vo_dcm, out)
    print(f"  {'figure':<26} = {out}")

    problems: list[str] = []
    if abs(float(il_ccm[0]) - (VIN / L) * H) > 1e-6:
        problems.append("第一步的 di/dt 不是 Vin/L")
    if abs(st["Vo"] - th["Vo"]) / th["Vo"] > REL_VO:
        problems.append("CCM 平均电压偏离 Vin/(1-D) 超过 0.5%")
    if abs(st["diL"] - th["diL"]) / th["diL"] > REL_RIPPLE:
        problems.append("ΔiL 与教科书公式相差超过 3%")
    if abs(st["dvo"] - th["dvo"]) / th["dvo"] > REL_RIPPLE:
        problems.append("Δvo 与教科书公式相差超过 3%")
    if st["imin"] <= 0.0:
        problems.append("R = 48 Ω 稳态应处于连续导通")
    if zero_idx.size and t_dcm_end > 10.0:
        problems.append("启动时的电流断续应在 10 ms 内结束")
    if err_i > EXPM_TOL_I or err_v > EXPM_TOL_V:
        problems.append("RK4 与矩阵指数不一致")
    for tag in ("low", "high"):
        s_x, th_x = res[tag]
        if abs(s_x["Vo"] - th_x["Vo"]) / th_x["Vo"] > REL_VO:
            problems.append(f"duty_{tag} 的平均电压没有跟着 Vin/(1-D) 走")
        if abs(s_x["diL"] - th_x["diL"]) / th_x["diL"] > REL_RIPPLE:
            problems.append(f"duty_{tag} 的电流纹波偏离公式")
    if k_dcm >= k_crit:
        problems.append("R = 320 Ω 的 K 应小于 D(1-D)^2")
    if st_d["imin"] > 1e-9:
        problems.append("断续工况的电感电流应回到 0")
    if abs(st_d["Vo"] - vo_dcm_th) / vo_dcm_th > REL_DCM:
        problems.append("断续电压偏离 DCM 公式超过 1%")

    if problems:
        print("校验未通过：", file=sys.stderr)
        for item in problems:
            print(f"  - {item}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
