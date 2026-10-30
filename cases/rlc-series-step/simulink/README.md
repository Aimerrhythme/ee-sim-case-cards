# Simulink 搭建步骤 · rlc-series-step

不依赖 Simscape / Specialized Power Systems，用基础模块搭状态方程。物理电路接法见案例卡正文第 2 节。

本目录只放步骤，不提交二进制 `.slx`。一次仿真里放三条链，R 分别是 100 Ω、200 Ω、400 Ω，这样欠阻尼、临界、过阻尼出现在同一个 Scope 上。L、C、Vs 不要各改各的。

## 模型

新建空白模型，保存为 `rlc_series_step.slx`。

状态方程（`vC(0) = 0`，`i(0) = 0`）：

```
di/dt = (Vs - R*i - vC) / L
dvC/dt = i / C
```

`L = 0.1`，所以 `1/L = 10`。`C = 1e-5`，所以 `1/C = 1e5`。

一条链的信号流：

```
                         +---------------- vC -------------------+
                         |                                       |
                         v                                       |
Step(Vs) -->(+) Sum(+--) --> Gain(1/L) --> Integrator --> i --+--> Gain(1/C) --> Integrator --> vC
                 ^    ^                         |              |
                 |    |                         |              +--> 去电压 Mux
                 |    +---------- Gain(R) <-----+
                 |                               +--> 去电流 Mux
                 +-------------- vC
```

Sum 的输出是 `di/dt` 前面的那一项，还要乘 `1/L` 才是 `di/dt`。不要在 Sum 后面直接接电流示波器。

### 模块清单（一条链）

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Step | Simulink / Sources | Step time = `0`，Initial = `0`，Final = `10`，Sample time = `0` |
| Sum | Simulink / Math Operations | 符号列表 `+--` |
| Gain `1/L` | Simulink / Math Operations | Gain = `10` |
| Integrator（电流） | Simulink / Continuous | Initial condition = `0`，输出为 `i` |
| Gain `R` | Simulink / Math Operations | 第一份 `100`，第二份 `200`，第三份 `400` |
| Gain `1/C` | Simulink / Math Operations | Gain = `1e5` |
| Integrator（电压） | Simulink / Continuous | Initial condition = `0`，输出为 `vC` |
| Mux | Simulink / Signal Routing | 输入个数 `3`。电压用一个，电流用一个 |
| Scope ×2 | Simulink / Sinks | 一个接电压 Mux，一个接电流 Mux |

Step 只放一个，输出同时接到三个 Sum 的正端。每个积分器的输出分三路：示波器方向、反馈、下一级。电流积分器的反馈是 Gain `R`，回到本链 Sum 的第三个输入（第二个减号）。电压积分器的反馈回到本链 Sum 的第二个输入（第一个减号）。不要把第一份的 `vC` 接到第二份的 Sum 上。

三条链的 R 用注解标成 `ζ = 0.5`、`ζ = 1`、`ζ = 2`，避免 Scope 里分不清颜色。Mux 的输入顺序固定为 100 Ω、200 Ω、400 Ω，和案例卡图上的蓝、绿、橙一致。

电流 Scope 的纵轴是安培。欠阻尼峰值约 **0.055 A**。若要直接读毫安，在每条电流进 Mux 之前加 Gain = `1000`。

## 求解器

菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `0.025` |
| Type | Variable-step |
| Solver | `ode45` |
| Max step size | `1e-5` |
| Relative tolerance | `1e-4` |

`1e-5` 是为过阻尼的快极点留的。那一档的快时间常数约 0.27 ms，步长若放到 `1e-3`，临界阻尼也会被步长振出波纹。

运行后把横轴放到 0–25 ms：

- 100 Ω：电压从 0 出发，约 3.63 ms 到 **11.63 V**，再落到 10 V 附近；电流先到约 0.055 A，随后出现一小段负值，25 ms 时回到 0
- 200 Ω：电压不过 10 V，约 10 ms 时已经在 **9.995 V**；电流峰值约 0.037 A，出现在 1 ms
- 400 Ω：电压最慢，10 ms 时约 **9.26 V**，25 ms 时约 **9.99 V**；电流峰值约 0.022 A
- 三条电压在最开始的零点几毫秒几乎重合，分叉发生在电流起来之后
- 不要接 Derivative

## 可选：传递函数

若这门课已经在用 Transfer Fcn，可以用它对照积分器模型，不要两套同时去改参数。

```
V_C(s) / Vs(s) = 1 / (L C s^2 + R C s + 1)
```

| 档 | 分子 | 分母 |
|----|------|------|
| R = 100 | `[1]` | `[1e-6  1e-3  1]` |
| R = 200 | `[1]` | `[1e-6  2e-3  1]` |
| R = 400 | `[1]` | `[1e-6  4e-3  1]` |

Step 的 Final 仍是 `10`。分子保持 `[1]`。若 Step 用单位阶跃（Final = 1），再在输出乘 10，不要两处都乘。

## 可选：Specialized Power Systems 物理接法

仅当已安装该工具箱时使用，用来对照「电路」而不是「方框图」。

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. `DC Voltage Source` = 10 V。三条支路可以各用一个源，也可以一个源带三条串联支路；每条支路自己形成回路，不要三套电容并联在同一个理想电压源上。
3. 每条用一个 `Series RLC Branch`：Inductance = `0.1` H，Capacitance = `1e-5` F，电感初始电流 = 0，电容初始电压 = 0。Resistance 分别是 `100`、`200`、`400`。
4. `Current Measurement` 串在各支路里；`Voltage Measurement` 并在各电容两端。三路电压、三路电流分别 Mux 后进 Scope。
5. 求解器改为 `ode23tb`，Stop time = `0.025`，Max step size = `1e-5`。

Capacitance 留成 `inf` 时支路是 RL，电流会停在 `Vs/R`，那是 [`rl-step`](../../rl-step/README.md) 的现象。Inductance 留成 `0` 时支路是 RC，电压不会过冲。两个初值都要是 0，否则看到的是零状态响应叠上一段零输入响应。
