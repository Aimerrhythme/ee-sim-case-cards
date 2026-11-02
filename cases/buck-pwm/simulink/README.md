# Simulink 搭建步骤 · buck-pwm

物理电路优先用 Simscape Electrical。没有该工具箱时，用后面的基础模块搭连续导通的状态方程。本目录只放步骤，不提交二进制 `.slx`。

参数与案例卡一致：`Vin = 12 V`，`D = 0.5`，`fs = 10 kHz`（`T = 100 μs`），`L = 1e-3 H`，`C = 25e-6 F`，`R = 6 Ω`。初值都是 0。

## 1. Simscape Electrical

新建空白模型，保存为 `buck_pwm.slx`。

电路：

```text
Vin+ --- 理想开关 ---+--- 电流传感器 --- 电感 ---+--- vo 节点 ---+
                     |                           |              |
                   二极管                        电容           电阻
                  （阴极朝上）                    |              |
                     |                           |              |
Vin- ----------------+---------------------------+--------------+--- 参考地
                                                                      |
                                                            Solver Configuration
```

栅极不在这张电气图里：Pulse Generator → Simulink-PS Converter → 理想开关的 G 端。电流、电压经 PS-Simulink Converter 进 Scope。

### 模块与参数

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Solver Configuration | Simscape / Utilities | 默认即可，连到参考地或任一电气节点 |
| DC Voltage Source | Simscape / Foundation Library / Electrical / Electrical Sources / DC Voltage Source | Constant voltage = `12` V |
| Ideal Semiconductor Switch | Simscape / Electrical / Semiconductors & Converters / Ideal Semiconductor Switch | On-state resistance = `0.01` Ω，Off-state conductance = `1e-6` 1/Ω，Threshold voltage = `0.5` V |
| Diode | Simscape / Electrical / Semiconductors & Converters / Diode | Diode model = `Piecewise linear`，Forward voltage = `0` V，On resistance = `0.01` Ω，Off conductance = `1e-6` 1/Ω |
| Inductor | Simscape / Foundation Library / Electrical / Electrical Elements / Inductor | Inductance = `1e-3` H，Series resistance = `0`，Initial current = `0` |
| Capacitor | 同上目录 / Capacitor | Capacitance = `25e-6` F，Series resistance = `0`，Initial voltage = `0` |
| Resistor | 同上目录 / Resistor | Resistance = `6` Ω |
| Electrical Reference | 同上目录 / Electrical Reference | 全图只有一个参考地 |
| Current Sensor | Simscape / Foundation Library / Electrical / Electrical Sensors / Current Sensor | 串在电感支路 |
| Voltage Sensor | 同上目录 / Voltage Sensor | 正端接输出节点，负端接参考地 |
| Pulse Generator | Simulink / Sources / Pulse Generator | 见下表 |
| Simulink-PS Converter | Simscape / Utilities | Input signal unit = `1`，Filtering and derivatives = `Provide signals` |
| PS-Simulink Converter ×2 | Simscape / Utilities | Output signal unit 分别填 `A`、`V` |
| Scope ×2 | Simulink / Sinks | 一个看 `vo`，一个看 `iL` |

Pulse Generator（Time based）：

| 项 | 取值 |
|----|------|
| Amplitude | `1` |
| Period | `1e-4` |
| Pulse width | `50` |
| Phase delay | `0` |
| Sample time | `0` |

Pulse width 的单位是百分数。50 表示半个周期，对应公式里的 `D = 0.5`。不要在这个框里填 `0.5`，那会变成 0.5% 占空比。

二极管若还有 Reverse recovery、Junction capacitance 之类的选项，保持关闭或 None。本卡不看反向恢复。

电感或电容若还有 Parallel conductance，先留着模块默认的很小电导。把它改成 0 以后若求解器报电路奇异，再改回默认值。这个电导不影响 0.15 V 这一位上的纹波。串联电阻和电容 ESR 必须是 0，否则纹波里会多出 `i·R` 的台阶。

理想开关的 Ron 不要填 0。Simscape 需要一个很小的导通电阻，`0.01 Ω` 在 1 A 时只掉 0.01 V。平均电压会比严格的 6 V 低若干毫伏，这是模块的需要，不是公式错了。

### 连线顺序

1. 放下 DC Voltage Source、理想开关、二极管、电流传感器、电感、电容、电阻、Electrical Reference、Solver Configuration。
2. 电源正极接开关的一个电气端，开关另一端接到「开关节点」。
3. 二极管的阴极（符号上的横杠那一侧）接开关节点，阳极接参考地。方向反了会在开关导通时把电源短路。
4. 电流传感器的电气端串在开关节点和电感之间，传感器箭头指向电感，这个方向就是 `iL`。
5. 电感的另一端是输出节点 `vo`。电容和电阻都从 `vo` 接到参考地。
6. 电源负极接参考地。Solver Configuration 也接参考地。
7. Voltage Sensor 的 `+` 接 `vo`，`−` 接参考地。两个传感器的物理信号输出各接一个 PS-Simulink Converter，再接 Scope。
8. Pulse Generator 接 Simulink-PS Converter，再接理想开关的栅极 G。阈值是 0.5 V，脉冲幅值 1 可以让开关按脉冲开通。Simulink-PS Converter 不要打开输入滤波，否则栅极变成斜坡，占空比会漂。

### 求解器

菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `0.005` |
| Type | Variable-step |
| Solver | `ode23t` |
| Max step size | `1e-6` |
| Relative tolerance | `1e-4` |

`1e-6` 是开关周期的一百分之一。步长若放到 `1e-4`，一个周期里只剩一步，纹波会被吃掉。

运行后：

- 0–2 ms：电压从 0 冲到大约 6.9 V，再收到 6 V 附近。最高点在 0.6 ms 左右
- 4.6–5.0 ms：把横轴放大。`iL` 在 0.85 A 和 1.15 A 之间来回，`vo` 的峰峰值约 0.15 V，中心在 6 V
- 电压纹波应是圆滑的抛物线。若看成方波台阶，回到电容参数里把 ESR 设为 0

断续对照：电阻改成 `80`，Stop time 改成 `0.04`，最大步长仍是 `1e-6`。稳态电压应在 7.4 V 附近，而不是 6 V；电流每个周期有一段贴在 0。

## 2. 只有基础库时（连续导通）

不安装 Simscape 也能看主工况。这条链是 `vL = q·Vin − vo`，电流可以变负，所以只适用于 `R = 6 Ω` 这种远离断续边界的参数。不要用它演示 `R = 80 Ω`。

状态方程（`iL(0) = 0`，`vo(0) = 0`）：

```text
diL/dt = (q*Vin - vo) / L
dvo/dt = (iL - vo/R) / C
```

`1/L = 1000`，`1/C = 40000`，`1/R = 1/6`。

```text
Pulse(q) --×(Vin)-->(+) Sum(+-) --> Gain(1/L) --> Integrator --> iL --> Scope
                         (-)
                          ^
                          +-------------------------------- vo

iL -->(+) Sum(+-) --> Gain(1/C) --> Integrator --> vo --> Scope
       (-)
        ^
        +-------------- Gain(1/R) <--- vo
```

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Pulse Generator | Simulink / Sources | 与上一节相同：Period = `1e-4`，Pulse width = `50`，Amplitude = `1` |
| Gain `Vin` | Simulink / Math Operations | Gain = `12`，输入是脉冲 `q` |
| Sum | Simulink / Math Operations | 符号 `+-`。正端是 `q·Vin`，负端是 `vo` |
| Gain `1/L` | 同上 | Gain = `1000` |
| Integrator | Simulink / Continuous | Initial condition = `0`，输出为 `iL` |
| Sum | 同上 | 符号 `+-`。正端是 `iL`，负端是 `vo/R` |
| Gain `1/R` | 同上 | Gain = `1/6` |
| Gain `1/C` | 同上 | Gain = `40000` |
| Integrator | Simulink / Continuous | Initial condition = `0`，输出为 `vo` |
| Scope ×2 | Simulink / Sinks | 一个看 `iL`，一个看 `vo` |

求解器改为 **Variable-step / ode45**，Stop time = `0.005`，Max step size = `1e-6`，Relative tolerance = `1e-4`。

读数与上一节相同：4.6–5.0 ms 的电流峰峰值约 0.30 A，电压峰峰值约 0.15 V，平均值 6 V。这里没有 0.01 Ω 的导通电阻，平均电压应比 Simscape 那一版更靠近 6.000 V。不要再接 Derivative。

## 3. 可选：Specialized Power Systems

仅当已安装该工具箱、课堂又习惯 `powergui` 时使用。

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. `DC Voltage Source` = 12 V。
3. `Ideal Switch`（Power Electronics）：Internal resistance Ron = `0.01` Ω，Initial state = `0`（断开）。Snubber resistance = `inf`，Snubber capacitance = `0`，把缓冲电路去掉。默认的 `1e5 Ω` 缓冲会在开关节点上拉出额外电流。
4. `Diode`：Ron = `0.01` Ω，Forward voltage = `0`，Snubber 同样设成 `Rs = inf`、`Cs = 0`。阴极接开关节点，阳极接负极母线。
5. 三段 `Series RLC Branch`：
   - 电感：Resistance = `0`，Inductance = `1e-3`，Capacitance = `inf`，初始电流 0
   - 电容：Resistance = `0`，Inductance = `0`，Capacitance = `25e-6`，初始电压 0
   - 负载：Resistance = `6`，Inductance = `0`，Capacitance = `inf`
6. `Current Measurement` 串在电感支路，`Voltage Measurement` 并在负载两端。
7. 门极用 `PWM Generator (DC-DC)`：Switching frequency = `10000` Hz，Duty cycle = `0.5`。这里的占空比是分数。不要照着 Pulse Generator 的习惯填 50。
8. 求解器改为 `ode23tb`，Stop time = `0.005`，Max step size = `1e-6`。

Capacitance 若留着默认的微法量级，电感支路会变成 RLC，电流波形不再是干净的三角波。
