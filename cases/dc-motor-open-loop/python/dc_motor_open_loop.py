#!/usr/bin/env python3
"""他励（励磁恒定）/ 永磁直流电机开环直接启动（教学演示）。

励磁不变时，反电势系数和转矩系数相等，记作 K（SI 单位下 Ke = Kt）::

    La dia/dt = Va − Ra ia − K ω
    J  dω/dt  = K ia − B ω − T_L

两个状态 x = [ia, ω]，是线性定常系统 dx/dt = A x + b u::

    A = [[−Ra/La, −K/La],
         [  K/J ,  −B/J]]

主工况：t = 0 把 Va = 220 V 直接加到静止电机上（空载，只有粘滞摩擦 B），
t = 0.8 s 突加 T_L = 20 N·m，算到 1.5 s。
对照工况：电枢回路串启动电阻 R_st = 5 Ω，同样 0 时刻加 220 V、空载。

解析解用矩阵指数（scipy.linalg.expm，MATLAB 用 expm），峰值电流用
特征根闭式求出。数值解用 solve_ivp（RK45）核对，偏差超限就报错退出。
打印的关键数字全部来自解析式，MATLAB 脚本按同一公式算，应逐行相同。

用法（在仓库根目录或本目录均可）::

    python cases/dc-motor-open-loop/python/dc_motor_open_loop.py

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
from scipy.linalg import expm

VA = 220.0  # V，电枢额定电压
RA = 0.5  # Ω，电枢电阻
LA = 0.005  # H，电枢电感（5 mH）
K = 1.0  # V·s/rad = N·m/A，Ke = Kt
J = 0.2  # kg·m²，电机 + 负载折算惯量
B = 0.005  # N·m·s/rad，粘滞摩擦
TL = 20.0  # N·m，额定负载转矩（突加）
I_N = 20.0  # A，示例额定电流（只用来算倍数）
R_ST = 5.0  # Ω，对照工况的启动电阻
T_LOAD = 0.8  # s，突加负载时刻
T_END = 1.5  # s
N_POINTS = 3001
TOL_I = 0.05  # A，数值解相对解析解的允许最大偏差
TOL_W = 0.01  # rad/s

RPM = 60.0 / (2.0 * math.pi)


def system(r_total: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回 A、电压输入列 b_v、负载输入列 b_l。"""
    a = np.array([[-r_total / LA, -K / LA], [K / J, -B / J]])
    b_v = np.array([1.0 / LA, 0.0])
    b_l = np.array([0.0, -1.0 / J])
    return a, b_v, b_l


def steady(r_total: float, tl: float) -> tuple[float, float]:
    """稳态 ia、ω：Va = R ia + K ω，K ia = T_L + B ω。"""
    w = (K * VA - r_total * tl) / (K * K + r_total * B)
    ia = (tl + B * w) / K
    return ia, w


def analytic(t: np.ndarray, r_total: float, with_load: bool) -> np.ndarray:
    """矩阵指数解，返回形状 (2, N)：第 0 行 ia，第 1 行 ω。"""
    a, _, _ = system(r_total)
    out = np.empty((2, t.size))
    x0 = np.zeros(2)
    xs0 = np.array(steady(r_total, 0.0))
    for k, tk in enumerate(t):
        if (not with_load) or tk < T_LOAD:
            out[:, k] = xs0 + expm(a * tk) @ (x0 - xs0)
        else:
            x_switch = xs0 + expm(a * T_LOAD) @ (x0 - xs0)
            xs1 = np.array(steady(r_total, TL))
            out[:, k] = xs1 + expm(a * (tk - T_LOAD)) @ (x_switch - xs1)
    return out


def simulate(t: np.ndarray, r_total: float, with_load: bool) -> np.ndarray:
    a, b_v, b_l = system(r_total)

    def rhs(time: float, x: np.ndarray) -> np.ndarray:
        tl = TL if (with_load and time >= T_LOAD) else 0.0
        return a @ x + b_v * VA + b_l * tl

    sol = solve_ivp(
        rhs,
        (float(t[0]), float(t[-1])),
        [0.0, 0.0],
        t_eval=t,
        method="RK45",
        rtol=1e-8,
        atol=1e-8,
        max_step=2e-4,
    )
    if not sol.success:
        raise RuntimeError(f"solve_ivp 失败: {sol.message}")
    return sol.y


def start_peak(r_total: float) -> tuple[float, float]:
    """零初值、空载阶跃启动时电流峰值的时刻和大小（过阻尼，两个实根）。

    ia(t) − ia_ss = c1 e^{p1 t} + c2 e^{p2 t}，dia/dt = 0 给出
    t* = ln(−c2 p2 / (c1 p1)) / (p1 − p2)。
    """
    a, _, _ = system(r_total)
    p = np.linalg.eigvals(a)
    if np.any(np.abs(p.imag) > 0):
        raise RuntimeError("本卡参数应为过阻尼（两个实根）")
    p1, p2 = sorted(p.real)  # p1 更负（快），p2 慢
    ia_ss, _ = steady(r_total, 0.0)
    # 初值 ia(0) = 0，dia/dt(0) = Va/La
    #   c1 + c2 = −ia_ss
    #   c1 p1 + c2 p2 = Va/La
    m = np.array([[1.0, 1.0], [p1, p2]])
    c1, c2 = np.linalg.solve(m, np.array([-ia_ss, VA / LA]))
    t_pk = math.log(-c2 * p2 / (c1 * p1)) / (p1 - p2)
    i_pk = ia_ss + c1 * math.exp(p1 * t_pk) + c2 * math.exp(p2 * t_pk)
    return t_pk, i_pk


def key_points() -> dict[str, float]:
    a, _, _ = system(RA)
    p = np.sort(np.linalg.eigvals(a).real)
    ia0, w0 = steady(RA, 0.0)
    ia1, w1 = steady(RA, TL)
    t_pk, i_pk = start_peak(RA)
    t_pk_st, i_pk_st = start_peak(RA + R_ST)
    p_st = np.sort(np.linalg.eigvals(system(RA + R_ST)[0]).real)
    w_end_st = float(analytic(np.array([T_END]), RA + R_ST, False)[1, 0])
    xs = analytic(np.array([T_LOAD - 1e-12]), RA, False)[:, 0]
    return {
        "tau_e_s": LA / RA,
        "tau_m_s": J * RA / (K * K),
        "tau_fast_s": -1.0 / p[0],
        "tau_slow_s": -1.0 / p[1],
        "i_stall_A": VA / RA,
        "i_stall_over_In": VA / RA / I_N,
        "t_peak_ms": t_pk * 1e3,
        "i_peak_A": i_pk,
        "i_peak_over_In": i_pk / I_N,
        "w0_rad_s": w0,
        "n0_rpm": w0 * RPM,
        "ia0_A": ia0,
        "n_at_load_rpm": xs[1] * RPM,
        "w1_rad_s": w1,
        "n1_rpm": w1 * RPM,
        "ia1_A": ia1,
        "dn_load_rpm": (w0 - w1) * RPM,
        "e1_V": K * w1,
        "p_shaft_W": TL * w1,
        "i_stall_rst_A": VA / (RA + R_ST),
        "i_peak_rst_A": i_pk_st,
        "t_peak_rst_ms": t_pk_st * 1e3,
        "tau_slow_rst_s": -1.0 / p_st[1],
        "n_end_rst_rpm": w_end_st * RPM,
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


def plot(t: np.ndarray, main: np.ndarray, rst: np.ndarray, pts: dict[str, float], out: Path) -> None:
    _enable_cjk()
    fig, axes = plt.subplots(3, 1, figsize=(8.4, 8.4))

    ax = axes[0]
    ax.plot(t * 1e3, main[0], color="#1f4e79", lw=1.8, label="直接启动")
    ax.plot(t * 1e3, rst[0], color="#e07a3d", lw=1.6, label=f"串 R_st = {R_ST:g} Ω")
    ax.axhline(pts["i_stall_A"], color="#9ca3af", lw=0.8, ls=":")
    ax.text(1450, pts["i_stall_A"] - 25, "Va/Ra = 440 A（堵转电流）", ha="right", fontsize=9, color="#6b7280")
    ax.scatter([pts["t_peak_ms"]], [pts["i_peak_A"]], s=30, color="#1f4e79", zorder=5)
    ax.annotate(
        f"峰值 {pts['i_peak_A']:.1f} A @ {pts['t_peak_ms']:.2f} ms",
        (pts["t_peak_ms"], pts["i_peak_A"]),
        xytext=(120, pts["i_peak_A"] - 40),
        fontsize=9,
    )
    ax.axvline(T_LOAD * 1e3, color="#9ca3af", lw=0.8, ls="--")
    ax.set_ylabel("电枢电流 ia / A")
    ax.set_title("直流电机开环启动：Va = 220 V 阶跃，0.8 s 突加 20 N·m")
    ax.set_xlim(0, T_END * 1e3)
    ax.set_ylim(-10, 470)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="center right", frameon=False)

    ax = axes[1]
    ax.plot(t * 1e3, main[0], color="#1f4e79", lw=1.8)
    ax.axhline(pts["i_stall_A"], color="#9ca3af", lw=0.8, ls=":")
    ax.set_xlim(0, 100)
    ax.set_ylim(-10, 470)
    ax.set_ylabel("ia / A（前 100 ms 放大）")
    ax.grid(True, alpha=0.35)

    ax = axes[2]
    ax.plot(t * 1e3, main[1] * RPM, color="#1f4e79", lw=1.8, label="直接启动")
    ax.plot(t * 1e3, rst[1] * RPM, color="#e07a3d", lw=1.6, label=f"串 R_st = {R_ST:g} Ω")
    ax.axhline(pts["n0_rpm"], color="#9ca3af", lw=0.8, ls=":")
    ax.axvline(T_LOAD * 1e3, color="#9ca3af", lw=0.8, ls="--")
    ax.set_xlabel("时间 t / ms")
    ax.set_ylabel("转速 n / (r/min)")
    ax.set_xlim(0, T_END * 1e3)
    ax.set_ylim(0, 2300)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right", frameon=False)

    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    plt.close(fig)


def main() -> int:
    if min(RA, LA, K, J) <= 0 or B < 0:
        print("教学参数要求 Ra、La、K、J 为正，B 不小于 0。", file=sys.stderr)
        return 1

    t = np.linspace(0.0, T_END, N_POINTS)
    main_ref = analytic(t, RA, True)
    main_num = simulate(t, RA, True)
    rst_ref = analytic(t, RA + R_ST, False)
    rst_num = simulate(t, RA + R_ST, False)
    err_i = float(max(np.max(np.abs(main_num[0] - main_ref[0])), np.max(np.abs(rst_num[0] - rst_ref[0]))))
    err_w = float(max(np.max(np.abs(main_num[1] - main_ref[1])), np.max(np.abs(rst_num[1] - rst_ref[1]))))
    pts = key_points()

    print("DC motor open-loop start (separately excited / PM)")
    for key, val in pts.items():
        print(f"  {key:<18} = {val:.6f}")
    print(f"  {'max_abs_err_i_A':<18} = {err_i:.3e}")
    print(f"  {'max_abs_err_w':<18} = {err_w:.3e}")

    out = Path(__file__).resolve().parent.parent / "img" / "dc_motor_open_loop_curves.png"
    plot(t, main_ref, rst_ref, pts, out)
    print(f"  {'figure':<18} = {out}")

    if err_i > TOL_I or err_w > TOL_W:
        print("校验未通过：RK45 与矩阵指数解析解不一致。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
