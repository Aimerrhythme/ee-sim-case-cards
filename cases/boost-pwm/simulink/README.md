# Simulink 搭建步骤 · boost-pwm

物理电路优先用 Simscape Electrical。没有该工具箱时，用后面的基础模块搭同步整流版的状态方程。本目录只放步骤，不提交二进制 `.slx`。

参数与案例卡一致：`Vin = 12 V`，`D = 0.5`，`fs = 10 kHz`（`T = 100 μs`），`L = 1e-3 H`，`C = 100e-6 F`，`R = 48 Ω`。初值都是 0。

## 1. Simscape Electrical

新建空白模型，保存为 `boost_pwm.slx`。

电路：

```text
Vin+ --- 电流传感器 --- 电感 ---+--- 二极管（阳极→阴极）---+--- vo 节点 ---+
                                |                         |              |
                            理想开关                      电容           电阻
                           （开关节点到地）                |              |
                                |                         |              |
Vin- ---------------------------+-------------------------+--------------+--- 参考地
                                                                               |
                                                                     Solver Configuration
```

和 Buck 的区别：开关从「开关节点」接到地，二极管从开关节点指向输出。电感在输入侧，电容和电阻在二极管后面。

栅极不在这张电气图里：Pulse Generator → Simulink-PS Converter → 理想开关的 G 端。电流、电压经 PS-Simulink Converter 进 Scope。

### 模块与参数

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Solver Configuration | Simscape / Utilities | 默认即可，连到参考地或任一电气节点 |
| DC Voltage Source | Simscape / Foundation Library / Electrical / Electrical Sources / DC Voltage Source | Constant voltage = `12` V |
| Ideal Semiconductor Switch | Simscape / Electrical / Semiconductors & Converters / Ideal Semiconductor Switch | On-state resistance = `0.01` Ω，Off-state conductance = `1e-6` 1/Ω，Threshold voltage = `0.5` V |
| Diode | Simscape / Electrical / Semiconductors & Converters / Diode | Diode model = `Piecewise linear`，Forward voltage = `0` V，On resistance = `0.01` Ω，Off conductance = `1e-6` 1/Ω |
| Inductor | Simscape / Foundation Library / Electrical / Electrical Elements / Inductor | Inductance = `1e-3` H，Series resistance = `0`，Initial current = `0` |
| Capacitor | 同上目录 / Capacitor | Capacitance = `100e-6` F，Series resistance = `0`，Initial voltage = `0` |
| Resistor | 同上目录 / Resistor | Resistance = `48` Ω |
| Electrical Reference | 同上目录 / Electrical Reference | 全图只有一个参考地 |
| Current Sensor | Simscape / Foundation Library / Electrical / Electrical Sensors / Current Sensor | 串在电源正极和电感之间 |
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

Pulse width 的单位是百分数。50 表示半个周期，对应公式里的 `D = 0.5`。不要在这个框里填 `0.5`，那会变成 0.5% 占空比，输出只比 12 V 高一点点。

二极管若还有 Reverse recovery、Junction capacitance 之类的选项，保持关闭或 None。

电感或电容若还有 Parallel conductance，先留着模块默认的很小电导；改成 0 以后若求解器报电路奇异，再改回默认值。串联电阻和电容 ESR 必须是 0，否则纹波里会多出 `i·R` 的台阶。

理想开关的 Ron 不要填 0，`0.01 Ω` 就够。Boost 对损耗比 Buck 敏感：电感电流 1 A 流过 Ron 和二极管导通电阻，平均电压会比严格的 24 V 低几十毫伏。

### 连线顺序

1. 放下 DC Voltage Source、电流传感器、电感、理想开关、二极管、电容、电阻、Electrical Reference、Solver Configuration。
2. 电源正极经电流传感器接电感，传感器箭头指向电感。电感另一端就是「开关节点」。
3. 理想开关的两个电气端：一端接开关节点，另一端接参考地。
4. 二极管的阳极接开关节点，阴极（横杠那一侧）接输出节点 `vo`。方向反了，电容会经二极管直接被开关短路放电，输出起不来。
5. 电容和电阻都从 `vo` 接到参考地。电源负极接参考地，Solver Configuration 也接参考地。
6. Voltage Sensor 的 `+` 接 `vo`，`−` 接参考地。两个传感器的物理信号输出各接一个 PS-Simulink Converter，再接 Scope。
7. Pulse Generator 接 Simulink-PS Converter，再接理想开关的栅极 G。Simulink-PS Converter 不要打开输入滤波，否则栅极变成斜坡，占空比会漂。

### 求解器

菜单 **Modeling → Model Settings → Solver**（或 Ctrl+E）：

| 项 | 取值 |
|----|------|
| Stop time | `0.1` |
| Type | Variable-step |
| Solver | `ode23t` |
| Max step size | `1e-6` |
| Relative tolerance | `1e-4` |

Boost 开环的启动阻尼很小（`ζ ≈ 0.066`），要 40 ms 左右才收敛，所以 Stop time 给到 100 ms。仿真会比 Buck 那张卡慢一些。

运行后：

- 0–3 ms：电压从 0 冲到大约 43.7 V，最高点在 2 ms 附近，电感电流冲到约 8 A
- 2.2–5.4 ms：电流有一段贴在 0。这是启动里电压过冲、电感电流放完后二极管截止，属于短暂的断续，不是接线错误
- 40 ms 以后：收到 24 V 附近
- 99.6–100 ms：把横轴放大。`iL` 在 0.7 A 和 1.3 A 之间来回；`vo` 是锯齿，开关导通时线性下降，关断时回升，峰峰值约 0.25 V

断续对照：电阻改成 `320`，Stop time 改成 `0.15`，最大步长仍是 `1e-6`。稳态电压应在 30.7 V 附近，而不是 24 V；电流每个周期有一段贴在 0。

## 2. 只有基础库时（同步整流，始终连续）

不安装 Simscape 也能看主工况。把二极管换成和主开关互补的理想开关（同步整流），`q` 是 0/1 的脉冲：

```text
diL/dt = (Vin - (1-q)*vo) / L
dvo/dt = ((1-q)*iL - vo/R) / C
```

`1/L = 1000`，`1/C = 10000`，`1/R = 1/48`。

```text
Pulse(q) --> Sum(Constant 1, -q) --> (1-q) ----------+----------------+
                                                     |                |
Constant(Vin) -->(+) Sum(+-) --> Gain(1/L) --> Integrator --> iL      |
                      (-)                                     |       |
                       ^                                      v       v
                       +---- Product((1-q), vo)          Product((1-q), iL)
                                                              |
                                    (+) Sum(+-) <-------------+
                                     (-)  |
                                      ^   +--> Gain(1/C) --> Integrator --> vo --> Scope
                                      |
                                      +------ Gain(1/R) <--- vo
```

| 模块 | 库路径 | 参数 |
|------|--------|------|
| Pulse Generator | Simulink / Sources | 与上一节相同：Period = `1e-4`，Pulse width = `50`，Amplitude = `1` |
| Constant ×2 | Simulink / Sources | `1`（算 `1−q`）和 `12`（Vin） |
| Sum `1−q` | Simulink / Math Operations | 符号 `+-` |
| Product ×2 | Simulink / Math Operations | 一个算 `(1−q)·vo`，一个算 `(1−q)·iL` |
| Sum | 同上 | 符号 `+-`。正端 `Vin`，负端 `(1−q)·vo` |
| Gain `1/L` | 同上 | Gain = `1000` |
| Integrator | Simulink / Continuous | Initial condition = `0`，输出为 `iL` |
| Sum | Simulink / Math Operations | 符号 `+-`。正端 `(1−q)·iL`，负端 `vo/R` |
| Gain `1/R` | 同上 | Gain = `1/48` |
| Gain `1/C` | 同上 | Gain = `10000` |
| Integrator | Simulink / Continuous | Initial condition = `0`，输出为 `vo` |
| Scope ×2 | Simulink / Sinks | 一个看 `iL`，一个看 `vo` |

求解器改为 **Variable-step / ode45**，Stop time = `0.1`，Max step size = `1e-6`，Relative tolerance = `1e-4`。

读数：稳态与 Simscape 相同，电流峰峰值约 0.60 A，电压峰峰值约 0.25 V，平均值 24 V。启动段不同：这条链的电流可以变负，最低大约 −4.8 A，没有那段贴在 0 的断续；电压峰值仍在 2 ms 附近、约 43.7 V。不要用它演示 `R = 320 Ω` 的断续。

## 3. 可选：Specialized Power Systems

仅当已安装该工具箱、课堂又习惯 `powergui` 时使用。

1. 拖入 `powergui`，Simulation type = **Continuous**。
2. `DC Voltage Source` = 12 V。
3. `Ideal Switch`（Power Electronics）：Ron = `0.01` Ω，Initial state = `0`，Snubber resistance = `inf`，Snubber capacitance = `0`。一端接开关节点，另一端接负极母线。
4. `Diode`：Ron = `0.01` Ω，Forward voltage = `0`，Snubber 同样设成 `Rs = inf`、`Cs = 0`。阳极接开关节点，阴极接输出。
5. 三段 `Series RLC Branch`：
   - 电感：Resistance = `0`，Inductance = `1e-3`，Capacitance = `inf`，初始电流 0
   - 电容：Resistance = `0`，Inductance = `0`，Capacitance = `100e-6`，初始电压 0
   - 负载：Resistance = `48`，Inductance = `0`，Capacitance = `inf`
6. `Current Measurement` 串在电感支路，`Voltage Measurement` 并在负载两端。
7. 门极用 `PWM Generator (DC-DC)`：Switching frequency = `10000` Hz，Duty cycle = `0.5`。这里的占空比是分数。
8. 求解器改为 `ode23tb`，Stop time = `0.1`，Max step size = `1e-6`。
