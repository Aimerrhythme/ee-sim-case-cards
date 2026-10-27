#!/usr/bin/env python3
"""串联 RL 直流阶跃响应（教学演示）。

状态方程（t >= 0，i(0) = 0）::

    L di/dt = Vs - R i

解析解::

    tau = L / R
    i(t)   = (Vs / R) * (1 - exp(-t / tau))
    v_L(t) = L di/dt = Vs * exp(-t / tau)
    v_R(t) = R i(t)

默认参数与案例卡一致：Vs = 10 V，R = 10 Ω，L = 0.1 H，故 tau = 10 ms，
稳态电流 1 A。t = 0 处切线斜率 Vs/L，在 t = tau 时碰到稳态值。

用法（在仓库根目录或本目录均可）::

    python cases/rl-step/python/rl_step.py

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

VS = 10.0  # V，阶跃幅值
R = 10.0  # Ω
L = 0.1  # H（100 mH）
TAU = L / R  # s
T_END = 5.0 * TAU  # 5τ，电流约到稳态的 99.3%
N_POINTS = 1001
I_TOL = 1e-4  # A，数值解相对解析解的允许最大偏差


def analytic(t: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回 i、v_L、v_R 的解析值。"""
    i = (VS / R) * (1.0 - np.exp(-t / TAU))
    v_l = VS * np.exp(-t / TAU)
    v_r = R * i
    return i, v_l, v_r


def simulate(t: np.ndarray) -> np.ndarray:
    """用 RK45 积分状态方程，返回与 t 对齐的电流。"""

    def rhs(_time: float, y: np.ndarray) -> list[float]:
        current = float(y[0])
        return [(VS - R * current) / L]

    sol = solve_ivp(
        rhs,
        (float(t[0]), float(t[-1])),
        [0.0],
        t_eval=t,
        method="RK45",
        rtol=1e-6,
        atol=1e-9,
    )
    if not sol.success:
        raise RuntimeError(f"solve_ivp 失败: {sol.message}")
    return sol.y[0]


def key_points() -> dict[str, float]:
    """案例卡第 4 节要对照的量化指标。"""
    i_tau, v_l_tau, _ = analytic(np.array([TAU]))
    i_3, _, _ = analytic(np.array([3.0 * TAU]))
    i_5, _, _ = analytic(np.array([5.0 * TAU]))
    return {
        "tau_s": TAU,
        "i_inf_A": VS / R,
        "i_tau_A": float(i_tau[0]),
        "i_3tau_A": float(i_3[0]),
        "i_5tau_A": float(i_5[0]),
        "v_l_0_V": VS,
        "v_l_tau_V": float(v_l_tau[0]),
        "slope0_A_per_s": VS / L,
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


def plot(t: np.ndarray, i_num: np.ndarray, out: Path) -> None:
    i_ref, v_l, v_r = analytic(t)
    points = key_points()
    t_ms = t * 1e3

    _enable_cjk()
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.4), sharex=True)

    ax = axes[0]
    ax.plot(t_ms, i_ref, color="#1f4e79", lw=2.0, label="解析解")
    ax.plot(t_ms, i_num, color="#e07a3d", lw=1.2, ls="--", label="RK45")
    t_tan_ms = np.array([0.0, TAU * 1e3])
    ax.plot(t_tan_ms, (VS / L) * (t_tan_ms / 1e3), color="#6b7280", ls=":", label="t=0 切线")
    ax.axhline(points["i_inf_A"], color="#9ca3af", lw=0.8)
    ax.scatter(
        [TAU * 1e3],
        [points["i_tau_A"]],
        s=36,
        color="#1f4e79",
        zorder=5,
        label=r"$t=\tau$，63.2%",
    )
    ax.set_ylabel("电流 i / A")
    ax.set_title(rf"串联 RL 阶跃    $\tau=L/R={TAU * 1e3:.0f}\,\mathrm{{ms}}$")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right", frameon=False)
    ax.set_xlim(0, T_END * 1e3)
    ax.set_ylim(0, 1.15)

    ax = axes[1]
    ax.plot(t_ms, v_l, color="#1f4e79", lw=2.0, label=r"$v_L$")
    ax.plot(t_ms, v_r, color="#2f7d4a", lw=2.0, label=r"$v_R$")
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("电压 / V")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="center right", frameon=False)
    ax.set_ylim(-0.4, 11.0)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def main() -> int:
    if R <= 0 or L <= 0:
        print("教学参数要求 R > 0 且 L > 0，否则 τ = L/R 无定义。", file=sys.stderr)
        return 1

    t = np.linspace(0.0, T_END, N_POINTS)
    i_num = simulate(t)
    i_ref, v_l, v_r = analytic(t)
    err = float(np.max(np.abs(i_num - i_ref)))
    kvl = float(np.max(np.abs(VS - v_r - v_l)))
    points = key_points()

    print("RL series step response")
    print(f"  tau_s            = {points['tau_s']:.6f}")
    print(f"  i_inf_A          = {points['i_inf_A']:.6f}")
    print(f"  i_tau_A          = {points['i_tau_A']:.6f}")
    print(f"  i_3tau_A         = {points['i_3tau_A']:.6f}")
    print(f"  i_5tau_A         = {points['i_5tau_A']:.6f}")
    print(f"  v_l_0_V          = {points['v_l_0_V']:.6f}")
    print(f"  v_l_tau_V        = {points['v_l_tau_V']:.6f}")
    print(f"  slope0_A_per_s   = {points['slope0_A_per_s']:.6f}")
    print(f"  max_abs_err_i_A  = {err:.3e}")
    print(f"  max_abs_kvl_V    = {kvl:.3e}")

    out = Path(__file__).resolve().parent.parent / "img" / "rl_step_curves.png"
    plot(t, i_num, out)
    print(f"  figure           = {out}")

    if err > I_TOL or kvl > 1e-9:
        print("校验未通过：数值解与解析解不一致，或 KVL 残差过大。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
