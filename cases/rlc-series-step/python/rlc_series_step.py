#!/usr/bin/env python3
"""串联 RLC 直流阶跃：欠阻尼、临界阻尼、过阻尼（教学演示）。

零状态，v_C(0) = 0，i(0) = 0。t >= 0 时回路接通直流电压 Vs::

    L di/dt + R i + v_C = Vs
    i = C dv_C/dt

L、C 固定，只改 R，从而只改阻尼比。自然角频率三条曲线相同::

    omega0 = 1 / sqrt(L C)
    zeta   = (R / 2) * sqrt(C / L)

zeta < 1 欠阻尼，zeta = 1 临界阻尼，zeta > 1 过阻尼。
稳态都是 v_C -> Vs、i -> 0。电阻不决定稳态电压，只决定怎么靠近它。

默认参数与案例卡一致：Vs = 10 V，L = 0.1 H，C = 1e-5 F（10 μF），
故 omega0 = 1000 rad/s。R = 100 / 200 / 400 Ω 对应 zeta = 0.5 / 1 / 2。

用法（在仓库根目录或本目录均可）::

    python cases/rlc-series-step/python/rlc_series_step.py

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

VS = 10.0  # V
L = 0.1  # H（100 mH）
C = 1e-5  # F（10 μF）
W0 = 1.0 / math.sqrt(L * C)  # rad/s
T_END = 0.025  # s，过阻尼的慢极点也已经靠近稳态
N_POINTS = 2001
# 与临界阻尼公式切换的容差。zeta 与 1 的差小于它时用临界公式，避免 sqrt(1-zeta^2) 为 0。
ZETA_CRIT = 1e-9
V_TOL = 1e-4  # V
I_TOL = 1e-5  # A
KVL_TOL = 1e-8  # V

# (名称, 电阻 Ω)。名称只用于打印和配色；公式按算出来的 zeta 分支。
CASES: tuple[tuple[str, float], ...] = (
    ("under", 100.0),
    ("critical", 200.0),
    ("over", 400.0),
)


def damping_ratio(resistance: float) -> float:
    """zeta = (R/2) * sqrt(C/L)。"""
    return 0.5 * resistance * math.sqrt(C / L)


def analytic(t: np.ndarray, resistance: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回 v_C、i、di/dt 的解析值。

    di/dt 由解析式求导，不从状态方程回代，这样 KVL 残差才是真正的校验。
    """
    zeta = damping_ratio(resistance)
    alpha = zeta * W0
    time = np.asarray(t, dtype=float)

    if zeta < 1.0 - ZETA_CRIT:
        wd = W0 * math.sqrt(1.0 - zeta * zeta)
        beta = math.sqrt(1.0 - zeta * zeta)
        phi = math.acos(zeta)
        decay = np.exp(-alpha * time)
        v_c = VS * (1.0 - decay / beta * np.sin(wd * time + phi))
        amp = VS / (wd * L)
        current = amp * decay * np.sin(wd * time)
        didt = amp * decay * (-alpha * np.sin(wd * time) + wd * np.cos(wd * time))
        return v_c, current, didt

    if zeta <= 1.0 + ZETA_CRIT:
        decay = np.exp(-W0 * time)
        v_c = VS * (1.0 - decay * (1.0 + W0 * time))
        current = (VS / L) * time * decay
        didt = (VS / L) * decay * (1.0 - W0 * time)
        return v_c, current, didt

    hyp = W0 * math.sqrt(zeta * zeta - 1.0)
    s1 = -alpha + hyp
    s2 = -alpha - hyp
    e1 = np.exp(s1 * time)
    e2 = np.exp(s2 * time)
    v_c = VS * (1.0 + (s2 * e1 - s1 * e2) / (s1 - s2))
    k_i = (VS / L) / (s1 - s2)
    current = k_i * (e1 - e2)
    didt = k_i * (s1 * e1 - s2 * e2)
    return v_c, current, didt


def simulate(t: np.ndarray, resistance: float) -> tuple[np.ndarray, np.ndarray]:
    """RK45 积分状态方程，返回与 t 对齐的 v_C、i。"""

    def rhs(_time: float, y: np.ndarray) -> list[float]:
        v_c = float(y[0])
        current = float(y[1])
        return [current / C, (VS - resistance * current - v_c) / L]

    sol = solve_ivp(
        rhs,
        (float(t[0]), float(t[-1])),
        [0.0, 0.0],
        t_eval=t,
        method="RK45",
        rtol=1e-6,
        atol=1e-9,
    )
    if not sol.success:
        raise RuntimeError(f"solve_ivp 失败 (R={resistance}): {sol.message}")
    return sol.y[0], sol.y[1]


def _under_times(resistance: float) -> dict[str, float]:
    """欠阻尼的峰值、谷值和 2% 时刻。只在 zeta < 1 时调用。"""
    zeta = damping_ratio(resistance)
    alpha = zeta * W0
    wd = W0 * math.sqrt(1.0 - zeta * zeta)
    beta = math.sqrt(1.0 - zeta * zeta)
    t_peak = math.pi / wd
    t_valley = 2.0 * math.pi / wd
    overshoot = math.exp(-zeta * math.pi / beta)
    t_env = -math.log(0.02 * beta) / alpha

    # 谷值过后电压从下向上穿过 0.98*Vs。二分与网格无关，MATLAB 用同样次数。
    lo = t_valley
    hi = 3.0 * math.pi / wd
    target = 0.98 * VS
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        v_mid = float(analytic(np.array([mid]), resistance)[0][0])
        if v_mid < target:
            lo = mid
        else:
            hi = mid
    t_2pct = 0.5 * (lo + hi)
    return {
        "tp_s": t_peak,
        "v_peak_V": VS * (1.0 + overshoot),
        "os_frac": overshoot,
        "t_valley_s": t_valley,
        "v_valley_V": VS * (1.0 - math.exp(-alpha * t_valley)),
        "t_2pct_s": t_2pct,
        "t_env2pct_s": t_env,
        "wd": wd,
        "Td_s": 2.0 * math.pi / wd,
    }


def _current_peak(resistance: float) -> tuple[float, float]:
    """电流第一个峰值的时刻和电流值。"""
    zeta = damping_ratio(resistance)
    alpha = zeta * W0
    if zeta < 1.0 - ZETA_CRIT:
        wd = W0 * math.sqrt(1.0 - zeta * zeta)
        t_i = math.atan(wd / alpha) / wd
    elif zeta <= 1.0 + ZETA_CRIT:
        t_i = 1.0 / W0
    else:
        hyp = W0 * math.sqrt(zeta * zeta - 1.0)
        s1 = -alpha + hyp
        s2 = -alpha - hyp
        t_i = math.log(s2 / s1) / (s1 - s2)
    current = float(analytic(np.array([t_i]), resistance)[1][0])
    return t_i, current


def key_points() -> list[tuple[str, float]]:
    """案例卡第 4 节要对照的量化指标，顺序即打印顺序。"""
    under_r = 100.0
    crit_r = 200.0
    over_r = 400.0
    under = _under_times(under_r)
    ti_u, i_u = _current_peak(under_r)
    ti_c, i_c = _current_peak(crit_r)
    ti_o, i_o = _current_peak(over_r)
    alpha_u = 0.5 * W0
    ti_neg = (math.pi + math.atan(under["wd"] / alpha_u)) / under["wd"]
    i_neg = float(analytic(np.array([ti_neg]), under_r)[1][0])
    v_c_1, _, _ = analytic(np.array([0.001, 0.005]), under_r)
    v_crit, _, _ = analytic(np.array([0.001, 0.005, 0.010]), crit_r)
    v_over, _, _ = analytic(np.array([0.001, 0.005, 0.010, T_END]), over_r)
    zeta_over = damping_ratio(over_r)
    alpha_over = zeta_over * W0
    hyp = W0 * math.sqrt(zeta_over * zeta_over - 1.0)
    s1 = -alpha_over + hyp
    s2 = -alpha_over - hyp
    return [
        ("omega0_rad_s", W0),
        ("di_dt0_A_per_s", VS / L),
        ("curvature0_V_per_s2", VS / (L * C)),
        ("under_R_ohm", under_r),
        ("under_zeta", damping_ratio(under_r)),
        ("under_wd_rad_s", under["wd"]),
        ("under_Td_ms", under["Td_s"] * 1e3),
        ("under_tp_ms", under["tp_s"] * 1e3),
        ("under_v_peak_V", under["v_peak_V"]),
        ("under_os_frac", under["os_frac"]),
        ("under_v_valley_V", under["v_valley_V"]),
        ("under_ti_ms", ti_u * 1e3),
        ("under_i_peak_A", i_u),
        ("under_ti_neg_ms", ti_neg * 1e3),
        ("under_i_neg_A", i_neg),
        ("under_t_2pct_ms", under["t_2pct_s"] * 1e3),
        ("under_t_env2pct_ms", under["t_env2pct_s"] * 1e3),
        ("under_vc_1ms_V", float(v_c_1[0])),
        ("under_vc_5ms_V", float(v_c_1[1])),
        ("crit_R_ohm", crit_r),
        ("crit_zeta", damping_ratio(crit_r)),
        ("crit_ti_ms", ti_c * 1e3),
        ("crit_i_peak_A", i_c),
        ("crit_vc_1ms_V", float(v_crit[0])),
        ("crit_vc_5ms_V", float(v_crit[1])),
        ("crit_vc_10ms_V", float(v_crit[2])),
        ("over_R_ohm", over_r),
        ("over_zeta", zeta_over),
        ("over_s1_rad_s", s1),
        ("over_s2_rad_s", s2),
        ("over_ti_ms", ti_o * 1e3),
        ("over_i_peak_A", i_o),
        ("over_vc_1ms_V", float(v_over[0])),
        ("over_vc_5ms_V", float(v_over[1])),
        ("over_vc_10ms_V", float(v_over[2])),
        ("over_vc_25ms_V", float(v_over[3])),
    ]


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


def plot(
    t: np.ndarray,
    curves: dict[str, tuple[np.ndarray, np.ndarray]],
    numeric: dict[str, tuple[np.ndarray, np.ndarray]],
    out: Path,
) -> None:
    points = dict(key_points())
    t_ms = t * 1e3
    style = {
        "under": ("#1f4e79", rf"欠阻尼  $\zeta=0.5$"),
        "critical": ("#2f7d4a", rf"临界  $\zeta=1$"),
        "over": ("#c45c26", rf"过阻尼  $\zeta=2$"),
    }

    _enable_cjk()
    fig, axes = plt.subplots(2, 1, figsize=(8.6, 7.2), sharex=True)

    ax = axes[0]
    for name, _resistance in CASES:
        color, label = style[name]
        v_num, _i_num = numeric[name]
        v_c, _current = curves[name]
        ax.plot(t_ms, v_num, color=color, lw=1.15, ls="--", alpha=0.95)
        ax.plot(t_ms, v_c, color=color, lw=2.0, label=label)
    ax.axhline(VS, color="#9ca3af", lw=0.8, label="稳态 10 V")
    ax.scatter(
        [points["under_tp_ms"]],
        [points["under_v_peak_V"]],
        s=36,
        color="#1f4e79",
        zorder=5,
        label="欠阻尼峰值",
    )
    ax.axvline(points["under_tp_ms"], color="#d6d3d1", lw=0.8, zorder=0)
    ax.set_ylabel("电容电压 v_C / V")
    ax.set_title(rf"串联 RLC 阶跃    $\omega_0={W0:.0f}\,\mathrm{{rad/s}}$    只改 R")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8, ncol=2)
    ax.set_ylim(-0.6, 13.2)

    ax = axes[1]
    peaks = {
        "under": (points["under_ti_ms"], points["under_i_peak_A"] * 1e3),
        "critical": (points["crit_ti_ms"], points["crit_i_peak_A"] * 1e3),
        "over": (points["over_ti_ms"], points["over_i_peak_A"] * 1e3),
    }
    labeled_peak = False
    for name, _resistance in CASES:
        color, label = style[name]
        _v_num, i_num = numeric[name]
        _v_c, current = curves[name]
        ax.plot(t_ms, i_num * 1e3, color=color, lw=1.15, ls="--", alpha=0.95)
        ax.plot(t_ms, current * 1e3, color=color, lw=2.0, label=label)
        t_peak_ms, i_peak_mA = peaks[name]
        ax.scatter(
            [t_peak_ms],
            [i_peak_mA],
            s=28,
            color=color,
            zorder=5,
            label="电流峰值" if not labeled_peak else None,
        )
        labeled_peak = True
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("电流 i / mA")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8, ncol=2)
    ax.set_xlim(0.0, T_END * 1e3)
    ax.set_ylim(-15.0, 70.0)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def _check_case(t: np.ndarray, resistance: float, v_num: np.ndarray, i_num: np.ndarray) -> tuple[float, float, float]:
    v_ref, i_ref, didt = analytic(t, resistance)
    err_v = float(np.max(np.abs(v_num - v_ref)))
    err_i = float(np.max(np.abs(i_num - i_ref)))
    kvl = float(np.max(np.abs(VS - resistance * i_ref - L * didt - v_ref)))
    zeta = damping_ratio(resistance)

    if abs(float(v_ref[0])) > 1e-12 or abs(float(i_ref[0])) > 1e-12:
        raise RuntimeError(f"R={resistance}: 初值不是零状态")
    if abs(float(didt[0]) - VS / L) > 1e-8:
        raise RuntimeError(f"R={resistance}: t=0 的 di/dt 不是 Vs/L")
    if zeta < 1.0 - ZETA_CRIT:
        if float(np.max(v_ref)) <= VS:
            raise RuntimeError(f"R={resistance}: 欠阻尼应越过电源电压")
    elif float(np.max(v_ref)) > VS + 1e-6:
        raise RuntimeError(f"R={resistance}: 临界或过阻尼不应过冲")
    if abs(float(v_ref[-1]) - VS) > 0.05:
        raise RuntimeError(f"R={resistance}: 仿真结束时电容电压离稳态还太远")
    return err_v, err_i, kvl


def main() -> int:
    if L <= 0 or C <= 0 or VS == 0:
        print("教学参数要求 L > 0、C > 0，且 Vs 不为 0。", file=sys.stderr)
        return 1
    for _name, resistance in CASES:
        if resistance <= 0:
            print("教学参数要求 R > 0，否则阻尼比无定义。", file=sys.stderr)
            return 1

    # 参数表上的三档必须正好落在欠 / 临界 / 过，避免单位写错还画出图。
    expected = {"under": 0.5, "critical": 1.0, "over": 2.0}
    for name, resistance in CASES:
        zeta = damping_ratio(resistance)
        if abs(zeta - expected[name]) > 1e-9:
            print(
                f"{name} 的 zeta={zeta:.6f}，与案例卡的 {expected[name]} 不一致。",
                file=sys.stderr,
            )
            return 1

    t = np.linspace(0.0, T_END, N_POINTS)
    numeric: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    err_v_max = 0.0
    err_i_max = 0.0
    kvl_max = 0.0
    for name, resistance in CASES:
        v_num, i_num = simulate(t, resistance)
        err_v, err_i, kvl = _check_case(t, resistance, v_num, i_num)
        numeric[name] = (v_num, i_num)
        err_v_max = max(err_v_max, err_v)
        err_i_max = max(err_i_max, err_i)
        kvl_max = max(kvl_max, kvl)

    print("RLC series step response")
    for key, value in key_points():
        print(f"  {key:<22} = {value:.6f}")
    print(f"  {'max_abs_err_vc_V':<22} = {err_v_max:.3e}")
    print(f"  {'max_abs_err_i_A':<22} = {err_i_max:.3e}")
    print(f"  {'max_abs_kvl_V':<22} = {kvl_max:.3e}")

    out = Path(__file__).resolve().parent.parent / "img" / "rlc_series_step_curves.png"
    # 图上画解析解。RK45 与解析解的偏差由上面的 max_abs_err 给出，叠在一起会完全重合。
    analytic_curves = {
        name: analytic(t, resistance)[:2] for name, resistance in CASES
    }
    plot(t, analytic_curves, numeric, out)
    print(f"  {'figure':<22} = {out}")

    if err_v_max > V_TOL or err_i_max > I_TOL or kvl_max > KVL_TOL:
        print("校验未通过：数值解与解析解不一致，或 KVL 残差过大。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
