# Simulink 搭建步骤 · rl-step

不依赖 Simscape / Specialized Power Systems，用基础模块搭状态方程。物理电路接法见案例卡正文第 2 节；若实验室装有电力工具箱，文末给出可选接法。

## 模型

新建空白模型，保存为 `rl_step.slx`（本目录只放步骤，不提交二进制模型）。

状态方程：

```
di/dt = (Vs - R*i) / L
vL    = Vs - R*i
```

信号流：

```
Step(Vs) -->(+) Sum --+--> Gain(1/L) --> Integrator --> i --> Scope
               (-)    |                      |
                ^     +--> vL --> Scope      |
                |                            |
                +---------- Gain(R) <--------+
```

Sum 的输出就是电感电压 `vL`。积分器输出是电流 `i`。

## 模块清单

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Step | Simulink / Sources | Step time = `0`，Initial = `0`，Final = `10`，Sample time = `0` |
| Sum | Simulink / Math Operations | 符号列表 `+-` |
| Gain `1/L` | Simulink / Math Operations | Gain = `10`（即 1/0.1） |
| Integrator | Simulink / Continuous | Initial condition = `0` |
| Gain `R` | Simulink / Math Operations | Gain = `10` |
| Scope ×2 | Simulink / Sinks | 一个看 `i`，一个看 `vL`；也可再加一条 `R*i` 看 `vR` |

先加 Step、Sum、两个 Gain、Integrator、两个 Scope，再按上图连线。积分器输出分两路：一路进 Scope，一路进 Gain(R) 后回到 Sum 的减号口。Sum 输出分两路：一路进 Gain(1/L)，一路直接进 `vL` 的 Scope。

## 求解器

菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `0.05`（5τ，τ = 10 ms） |
| Type | Variable-step |
| Solver | `ode45` |
| Max step size | `1e-4` |
| Relative tolerance | `1e-4` |

运行后用 Zoom 看前 50 ms：

- `i(0) = 0`，单调升向 1 A，10 ms 处约 **0.632 A**
- `vL(0)` 约为 10 V，随后指数降到 0
- 不要在阶跃上再接 Derivative 模块

## 可选：Specialized Power Systems 物理接法

仅当已安装该工具箱时使用，用来对照「电路」而不是「方框图」。

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. `DC Voltage Source` = 10 V。
3. `Series RLC Branch`：Resistance = `10` Ω，Inductance = `0.1` H，**Capacitance = `inf`**。电感初始电流 = 0。
4. `Current Measurement` 串在回路中；`Voltage Measurement` 并在电感两端。
5. 求解器改为 `ode23tb`，Stop time 仍为 `0.05`，Max step size = `1e-4`。

Capacitance 若留着默认的微法量级，电路会变成 RLC，电流出现振荡，那就不是这张卡要看的一阶响应。
