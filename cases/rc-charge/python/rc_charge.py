#!/usr/bin/env python3
"""串联 RC 充电与放电（教学演示）。

充电（零状态，v_C(0) = 0，t >= 0）::

    C dv_C/dt = (Vs - v_C) / R
    tau = R C
    v_C(t) = Vs * (1 - exp(-t / tau))
    i(t)   = (Vs / R) * exp(-t / tau)

放电（零输入，v_C(0) = Vs，电源已断开）::

    C dv_C/dt = - v_C / R
    v_C(t) = Vs * exp(-t / tau)
    i_dis(t) = (Vs / R) * exp(-t / tau)

i_dis 取从电容正极板流出、经电阻回到负极的方向，所以放电电流画成正值。

默认参数与案例卡一致：Vs = 10 V，R = 10 kΩ，C = 1 μF，故 tau = 10 ms。
充电电压在 t = tau 到达稳态的 63.2%；放电电压、充电电流在同一时刻剩 36.8%。

用法（在仓库根目录或本目录均可）::

    python cases/rc-charge/python/rc_charge.py

无图形界面时只保存图片、不弹窗。依赖见 requirements.txt。
"""

from __future__ import annotations

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
R = 10e3  # Ω（10 kΩ）
C = 1e-6  # F（1 μF）
TAU = R * C  # s
T_END = 5.0 * TAU  # 5τ
N_POINTS = 1001
V_TOL = 1e-4  # V，数值解相对解析解的允许最大偏差


def charge(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """充电：返回 v_C、i（电流从电源正极流出）。"""
    v_c = VS * (1.0 - np.exp(-t / TAU))
    current = (VS / R) * np.exp(-t / TAU)
    return v_c, current


def discharge(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """放电：返回 v_C、i_dis（从正极板流出为正）。"""
    v_c = VS * np.exp(-t / TAU)
    current = (VS / R) * np.exp(-t / TAU)
    return v_c, current


def _integrate(rhs, y0: float, t: np.ndarray) -> np.ndarray:
    sol = solve_ivp(
        rhs,
        (float(t[0]), float(t[-1])),
        [y0],
        t_eval=t,
        method="RK45",
        rtol=1e-6,
        atol=1e-9,
    )
    if not sol.success:
        raise RuntimeError(f"solve_ivp 失败: {sol.message}")
    return sol.y[0]


def simulate_charge(t: np.ndarray) -> np.ndarray:
    """RK45 积分充电状态方程，返回与 t 对齐的 v_C。"""

    def rhs(_time: float, y: np.ndarray) -> list[float]:
        return [(VS - float(y[0])) / (R * C)]

    return _integrate(rhs, 0.0, t)


def simulate_discharge(t: np.ndarray) -> np.ndarray:
    """RK45 积分放电状态方程，返回与 t 对齐的 v_C。"""

    def rhs(_time: float, y: np.ndarray) -> list[float]:
        return [-float(y[0]) / (R * C)]

    return _integrate(rhs, VS, t)


def key_points() -> dict[str, float]:
    """案例卡第 4 节要对照的量化指标。"""
    v_tau, i_tau = charge(np.array([TAU]))
    v_3, _ = charge(np.array([3.0 * TAU]))
    v_5, _ = charge(np.array([5.0 * TAU]))
    v_dis, i_dis = discharge(np.array([TAU]))
    return {
        "tau_s": TAU,
        "v_inf_V": VS,
        "v_c_tau_charge_V": float(v_tau[0]),
        "pct_charge_at_tau": float(v_tau[0] / VS),
        "v_c_3tau_charge_V": float(v_3[0]),
        "v_c_5tau_charge_V": float(v_5[0]),
        "v_c_tau_discharge_V": float(v_dis[0]),
        "pct_discharge_at_tau": float(v_dis[0] / VS),
        "i0_mA": (VS / R) * 1e3,
        "i_tau_mA": float(i_tau[0]) * 1e3,
        "i_dis_tau_mA": float(i_dis[0]) * 1e3,
        "slope0_V_per_s": VS / TAU,
    }


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


def _mark_tau(ax: plt.Axes, y: float, label: str) -> None:
    ax.axvline(TAU * 1e3, color="#a8a29e", lw=0.8, zorder=0)
    ax.scatter([TAU * 1e3], [y], s=36, color="#1f4e79", zorder=5, label=label)


def plot(
    t: np.ndarray,
    v_charge_num: np.ndarray,
    v_dis_num: np.ndarray,
    out: Path,
) -> None:
    v_c, i_c = charge(t)
    v_d, i_d = discharge(t)
    points = key_points()
    t_ms = t * 1e3
    i_c_mA = i_c * 1e3
    i_d_mA = i_d * 1e3

    _enable_cjk()
    fig, axes = plt.subplots(2, 2, figsize=(9.6, 7.0), sharex=True)

    ax = axes[0, 0]
    ax.plot(t_ms, v_c, color="#1f4e79", lw=2.0, label="解析解")
    ax.plot(t_ms, v_charge_num, color="#e07a3d", lw=1.2, ls="--", label="RK45")
    ax.plot(
        [0.0, TAU * 1e3],
        [0.0, VS],
        color="#6b7280",
        ls=":",
        label="t=0 切线",
    )
    ax.axhline(VS, color="#9ca3af", lw=0.8)
    _mark_tau(ax, points["v_c_tau_charge_V"], r"$t=\tau$，63.2%")
    ax.set_ylabel("电压 / V")
    ax.set_title("充电  电容电压")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right", frameon=False, fontsize=8)
    ax.set_ylim(-0.5, 11.6)

    ax = axes[0, 1]
    ax.plot(t_ms, v_d, color="#1f4e79", lw=2.0, label="解析解")
    ax.plot(t_ms, v_dis_num, color="#e07a3d", lw=1.2, ls="--", label="RK45")
    ax.plot(
        [0.0, TAU * 1e3],
        [VS, 0.0],
        color="#6b7280",
        ls=":",
        label="t=0 切线",
    )
    _mark_tau(ax, points["v_c_tau_discharge_V"], r"$t=\tau$，36.8%")
    ax.set_title("放电  电容电压")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)
    ax.set_ylim(-0.5, 11.6)

    ax = axes[1, 0]
    ax.plot(t_ms, i_c_mA, color="#1f4e79", lw=2.0, label="解析解")
    i_from_state = ((VS - v_charge_num) / R) * 1e3
    ax.plot(t_ms, i_from_state, color="#e07a3d", lw=1.2, ls="--", label="由 RK45 的 v_C 算出")
    _mark_tau(ax, points["i_tau_mA"], r"$t=\tau$，36.8%")
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("电流 / mA")
    ax.set_title("充电  电流")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)
    ax.set_ylim(-0.05, 1.15)

    ax = axes[1, 1]
    ax.plot(t_ms, i_d_mA, color="#1f4e79", lw=2.0, label="解析解")
    i_dis_from_state = (v_dis_num / R) * 1e3
    ax.plot(t_ms, i_dis_from_state, color="#e07a3d", lw=1.2, ls="--", label="由 RK45 的 v_C 算出")
    _mark_tau(ax, points["i_dis_tau_mA"], r"$t=\tau$，36.8%")
    ax.set_xlabel("时间 t / ms")
    ax.set_title("放电  电流")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right", frameon=False, fontsize=8)
    ax.set_ylim(-0.05, 1.15)

    fig.suptitle(
        rf"串联 RC 充放电    $\tau=RC={TAU * 1e3:.0f}\,\mathrm{{ms}}$",
        fontsize=13,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def main() -> int:
    if R <= 0 or C <= 0:
        print("教学参数要求 R > 0 且 C > 0，否则 τ = RC 无定义。", file=sys.stderr)
        return 1

    t = np.linspace(0.0, T_END, N_POINTS)
    v_charge_num = simulate_charge(t)
    v_dis_num = simulate_discharge(t)
    v_c, i_c = charge(t)
    v_d, i_d = discharge(t)

    err_c = float(np.max(np.abs(v_charge_num - v_c)))
    err_d = float(np.max(np.abs(v_dis_num - v_d)))
    kvl_c = float(np.max(np.abs(VS - i_c * R - v_c)))
    kvl_d = float(np.max(np.abs(i_d * R - v_d)))
    points = key_points()

    print("RC charge and discharge")
    print(f"  tau_s                  = {points['tau_s']:.6f}")
    print(f"  v_inf_V                = {points['v_inf_V']:.6f}")
    print(f"  v_c_tau_charge_V       = {points['v_c_tau_charge_V']:.6f}")
    print(f"  pct_charge_at_tau      = {points['pct_charge_at_tau']:.6f}")
    print(f"  v_c_3tau_charge_V      = {points['v_c_3tau_charge_V']:.6f}")
    print(f"  v_c_5tau_charge_V      = {points['v_c_5tau_charge_V']:.6f}")
    print(f"  v_c_tau_discharge_V    = {points['v_c_tau_discharge_V']:.6f}")
    print(f"  pct_discharge_at_tau   = {points['pct_discharge_at_tau']:.6f}")
    print(f"  i0_mA                  = {points['i0_mA']:.6f}")
    print(f"  i_tau_mA               = {points['i_tau_mA']:.6f}")
    print(f"  slope0_V_per_s         = {points['slope0_V_per_s']:.6f}")
    print(f"  max_abs_err_vc_charge  = {err_c:.3e}")
    print(f"  max_abs_err_vc_dis     = {err_d:.3e}")
    print(f"  max_abs_kvl_charge_V   = {kvl_c:.3e}")
    print(f"  max_abs_kvl_dis_V      = {kvl_d:.3e}")

    out = Path(__file__).resolve().parent.parent / "img" / "rc_charge_curves.png"
    plot(t, v_charge_num, v_dis_num, out)
    print(f"  figure                 = {out}")

    if err_c > V_TOL or err_d > V_TOL or kvl_c > 1e-9 or kvl_d > 1e-9:
        print("校验未通过：数值解与解析解不一致，或 KVL 残差过大。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
