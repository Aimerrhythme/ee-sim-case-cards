# Simulink 搭建步骤 · dc-motor-open-loop

两种搭法，参数和案例卡第 3 节一致。A 只用 Simulink 基础库，任何装了 Simulink 的机房都能做；B 用 Simscape Electrical 的物理模块，用来对照「电路」而不是「方框图」。本目录只放步骤，不提交二进制 `.slx`。

## A. 基础库：两个积分器（推荐先做）

方程：

```text
La dia/dt = Va − Ra·ia − K·ω
J  dω/dt  = K·ia − B·ω − T_L
```

信号流（上面一行是电气环，下面一行是机械环，`K` 出现两次，一次是反电势，一次是转矩）：

```text
Step(Va) ──►(+)Sum1(−)(−)──► Gain 1/La ──► ∫ ──┬──► ia ──► Scope
              ▲     ▲                          │
              │     └──── Gain Ra ◄────────────┤
              │                                │
              │                                └──► Gain K (Kt) ──► Te
              │                                                      │
              └────────── Gain K (Ke) ◄──────┐                       ▼
                                             │         (+)Sum2(−)(−) ──► Gain 1/J ──► ∫ ──┬──► ω ──► Gain 60/(2π) ──► Scope (r/min)
                                             │               ▲     ▲                      │
                                             │               │     └──── Gain B ◄─────────┤
                                             │        Step(T_L)                           │
                                             └────────────────────────────────────────────┘
```

### 模块清单

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Step `Va` | Simulink / Sources | Step time = `0`，Initial = `0`，Final = `220` |
| Step `T_L` | Simulink / Sources | Step time = `0.8`，Initial = `0`，Final = `20` |
| Sum1 | Simulink / Math Operations | 符号 `+--`（Va，减 Ra·ia，减 K·ω） |
| Gain `1/La` | Simulink / Math Operations | `200`（即 1/0.005） |
| Integrator `ia` | Simulink / Continuous | Initial condition = `0` |
| Gain `Ra` | Simulink / Math Operations | `0.5` |
| Gain `Kt` | Simulink / Math Operations | `1` |
| Sum2 | Simulink / Math Operations | 符号 `+--`（Te，减 B·ω，减 T_L） |
| Gain `1/J` | Simulink / Math Operations | `5`（即 1/0.2） |
| Integrator `ω` | Simulink / Continuous | Initial condition = `0` |
| Gain `B` | Simulink / Math Operations | `0.005` |
| Gain `Ke` | Simulink / Math Operations | `1` |
| Gain `rpm` | Simulink / Math Operations | `60/(2*pi)` |
| Scope ×2 | Simulink / Sinks | 一个看 `ia`，一个看 `n` |

也可以把电气环换成 Transfer Fcn `1/(0.005s+0.5)`、机械环换成 `1/(0.2s+0.005)`，中间用两个 `K` 交叉反馈。两种写法是同一个系统。不要只用一个 `K/((La s+Ra)(J s+B)+K²)` 的传递函数去看电流：那个输出是转速，电流要另外拿。

### 求解器

**Modeling → Model Settings → Solver**（Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `1.5` |
| Type | Variable-step |
| Solver | `ode45` |
| Max step size | `2e-4` |
| Relative tolerance | `1e-6` |

### 读数

- `ia` 在 26.6 ms 附近到顶，约 **367.3 A**，不是 440 A。440 A 是转子卡住不动（堵转）的电流。
- 转速 0.8 s 前停在约 **2095.6 r/min**，0.8 s 加 20 N·m 后落到约 **2000.4 r/min**。
- 加载后 `ia` 从约 1.1 A 升到约 **21.05 A**。

对照工况：把 Sum1 的 `Ra` 改成 `5.5`（0.5 Ω + 5 Ω 启动电阻），T_L 的 Step Final 改成 `0`。电流峰值约 39.8 A，1.5 s 时转速只到约 1541 r/min。

## B. Simscape Electrical：DC Motor 物理模块

1. 新建模型，拖入 `Solver Configuration`（Simscape / Utilities）。
2. `DC Voltage Source`（Simscape / Foundation Library / Electrical / Electrical Sources），Constant voltage = `220` V。想看「合闸」可以换成 `Controlled Voltage Source` + Simulink Step（经 `Simulink-PS Converter`）。
3. `DC Motor`（Simscape Electrical / Electromechanical / Brushed Motors）：
   - Model parameterization = **By equivalent circuit parameters**
   - Armature resistance = `0.5` Ω，Armature inductance = `0.005` H
   - Back-emf constant = `1` V/(rad/s)
   - Rotor inertia = `0.2` kg·m²，Rotor damping = `0.005` N·m/(rad/s)
   - 初始转速 0，初始电流 0
4. `Current Sensor` 串在电源正极到电机 `+` 之间；`Ideal Rotational Motion Sensor` 接在电机轴 `R` 与壳 `C` 之间，W 端口经 `PS-Simulink Converter`（单位填 `rpm`）进 Scope。
5. 负载：`Ideal Torque Source` 的两个机械端口接在轴和 `Mechanical Rotational Reference` 之间，控制端经 `Simulink-PS Converter` 接 Step（Step time `0.8`，Final `20`）。
6. 电源负极、电机 `−` 接 `Electrical Reference`；电机壳 `C` 接 `Mechanical Rotational Reference`。
7. 求解器：Solver Configuration 勾 *Use local solver* 不必要；模型求解器用 `ode23t` 或 `ode15s`，Stop time `1.5`，Max step size `2e-4`。

加载后如果转速反而**升高**，说明 Ideal Torque Source 的方向和本卡相反：把 Step 的 Final 改成 `-20`，或者交换它的两个机械端口。读数应和 A 搭法一致（峰值约 367.3 A，稳态约 2095.6 / 2000.4 r/min）。

### 可选：Specialized Power Systems 的 DC Machine

已安装该工具箱、课堂又习惯 `powergui` 时可用。`powergui` 设 Continuous；DC Machine 选永磁（Field type = Permanent magnet，若版本没有这一项，就把励磁电压固定，使 `Laf·If = 1`），Ra = `0.5`，La = `0.005`，转矩常数 = `1`，J = `0.2`，Bm = `0.005`，Tf = `0`；TL 端口接 Step。求解器 `ode23tb`，Max step `2e-4`。
