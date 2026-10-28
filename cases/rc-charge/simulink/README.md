# Simulink 搭建步骤 · rc-charge

不依赖 Simscape / Specialized Power Systems，用基础模块搭状态方程。物理电路接法见案例卡正文第 2 节。充电和放电是两次换路，分开搭；不要在同一个积分器上把初值设成 0 又希望它同时表示放电。

本目录只放步骤，不提交二进制 `.slx`。

## 充电模型

新建空白模型，保存为 `rc_charge.slx`。

状态方程（`vC(0) = 0`）：

```
dvC/dt = (Vs - vC) / (R*C)
i      = (Vs - vC) / R
```

`R*C = 0.01`，所以 `1/(R*C) = 100`。

信号流：

```
Step(Vs) -->(+) Sum --+--> Gain(1/(R*C)) --> Integrator --> vC --> Scope
               (-)    |                         ^
                ^     +--> Gain(1/R) --> i --> Scope
                |                          
                +---------------------------+
```

Sum 的输出是电阻电压 `vR = Vs - vC`，不是电流。电流要再乘 `1/R`。

### 模块清单（充电）

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Step | Simulink / Sources | Step time = `0`，Initial = `0`，Final = `10`，Sample time = `0` |
| Sum | Simulink / Math Operations | 符号列表 `+-` |
| Gain `1/(RC)` | Simulink / Math Operations | Gain = `1/(10e3*1e-6)`（即 100） |
| Integrator | Simulink / Continuous | Initial condition = `0`，输出为 `vC` |
| Gain `1/R` | Simulink / Math Operations | Gain = `1/10e3`（即 `1e-4`），输出为电流，单位 A |
| Scope ×2 | Simulink / Sinks | 一个看 `vC`，一个看 `i` |

积分器输出分两路：一路进 `vC` 的 Scope，一路回到 Sum 的减号口。Sum 输出分两路：一路进 Gain `1/(RC)`，一路进 Gain `1/R`。

电流 Scope 的纵轴是安培。`t = 0` 附近应看到 **0.001 A**（1 mA），不是 1。若课堂想直接读毫安，在电流支路再串一个 Gain = `1000`。

## 放电模型

可把充电模型复制成另一张图，或在同一模型里另搭一组。电源已经断开，只剩电阻把电容能量放掉。

状态方程（`vC(0) = 10`）：

```
dvC/dt = -vC / (R*C)
i_dis  =  vC / R
```

`i_dis` 从电容正极板流出，经电阻回到负极。

信号流：

```
        +--> Gain(1/R) --> i_dis --> Scope
        |
vC <----+-- Integrator(初值 10) <---- Gain(-1/(R*C)) <--+
        |                                                |
        +------------------------------------------------+
```

积分器自己形成闭环，没有 Step。Gain 用一个负号：`Gain = -1/(10e3*1e-6)`，即 `-100`。不要再加 Sum，否则容易把符号乘两次，电压会往上长。

### 模块清单（放电）

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Integrator | Simulink / Continuous | Initial condition = `10`，输出为 `vC` |
| Gain `-1/(RC)` | Simulink / Math Operations | Gain = `-100`，输出接回积分器输入 |
| Gain `1/R` | Simulink / Math Operations | Gain = `1e-4`，从 `vC` 引出，输出为 `i_dis`（A） |
| Scope ×2 | Simulink / Sinks | 一个看 `vC`，一个看 `i_dis` |

`t = 0` 时 `vC` 必须是 10 V，然后单调降到 0。若一运行就是 0，是初值忘了改。

## 求解器

两个模型用同一组设置。菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `0.05`（5τ，τ = 10 ms） |
| Type | Variable-step |
| Solver | `ode45` |
| Max step size | `1e-4` |
| Relative tolerance | `1e-4` |

运行后把横轴放大到 0–50 ms：

- 充电：`vC(0) = 0`，10 ms 处约 **6.32 V**，50 ms 处约 **9.93 V**；电流从 0.001 A 往 0 掉，10 ms 处约 **0.368 mA**
- 放电：`vC(0) = 10`，10 ms 处约 **3.68 V**；`i_dis` 从 0.001 A 往 0 掉
- 竖线标在 10 ms。充电电压的 63.2% 和放电电压的 36.8% 是同一个时刻
- 不要接 Derivative 模块

## 可选：Specialized Power Systems 物理接法

仅当已安装该工具箱时使用，用来对照「电路」而不是「方框图」。

充电：

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. `DC Voltage Source` = 10 V。
3. `Series RLC Branch`：Resistance = `10000` Ω，Inductance = `0` H，Capacitance = `1e-6` F。电容初始电压 = 0。
4. `Current Measurement` 串在回路中；`Voltage Measurement` 并在电容两端。
5. 求解器改为 `ode23tb`，Stop time = `0.05`，Max step size = `1e-4`。

放电：去掉直流源，电阻仍为 `10000` Ω，电容 `1e-6` F、初始电压 = `10`，电感和电源都不要留在回路里。电容正极经电阻回到负极。

Inductance 若留着默认值，电路会变成 RLC，电压出现振荡，那就不是这张卡要看的一阶响应。Capacitance 不要写成 `1`（那是 1 F，τ 会变成 10000 s）。
